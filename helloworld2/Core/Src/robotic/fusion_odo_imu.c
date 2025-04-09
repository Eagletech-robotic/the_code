/*
 * fusion_odo_inu.c
 *
 *  Created on: Apr 6, 2025
 *      Author: nboulay
 */

#include "robotic/fusion_odo_imu.h"

// Il s'agit d'un moyen de mélanger les données des odomètres et de la central intertiel (IMU)
// L'IMU donne une orientation absolue pas trop mauvaise
// l'odomètre donne une position très précise si il n'y a pas de glissement
// L'idéal serait de faire une fusion de capteur avec un filtre de kalmann.
// ici je fais plus simple, j'utilise l'orientation de l'IMU et l'avancement de l'odomètre

#include <stdio.h>
#include <math.h>

//void fusion_odo_imu_init(fusion_odo_imu_t *fusion_odo_imu) {
//	fusion_odo_imu->previous_float_yaw_imu_deg = 0.0;
//}

/**
 * @brief
 *  Fusion simple de l'odométrie (2 codeurs) et de l'IMU (delta de yaw).
 *  Retourne les incréments de déplacement (delta_x_m, delta_y_m, delta_theta_deg)
 *  sur l'intervalle [t-1, t], à partir :
 *    - ax_imu_g, ay_imu_g : accélération latérale/longitudinale (éventuellement utilisable)
 *    - delta_yaw_imu_deg : variation d'angle (en degrés) mesurée par l'IMU depuis le dernier cycle
 *    - delta_motor_left_ticks, delta_motor_right_ticks : variations de ticks des codeurs
 *    - dt_s : temps écoulé (en secondes)
 *    - theta_deg : orientation précédente du robot (en degrés)
 *    - alpha_orientation_ratio : [0..1] poids de l'odométrie vs IMU pour la variation d'angle
 *    - ticks_per_rev, wheel_circumference_m, wheel_base_m : constantes mécaniques du robot
 *
 * @param fusion_odo_imu [in/out] structure (optionnelle ici, conservée pour compatibilité)
 * @param ax_imu_g [in]  accélération sur X en G (IMU), potentiellement exploitable pour la confiance
 * @param ay_imu_g [in]  accélération sur Y en G (IMU), potentiellement exploitable pour la confiance
 * @param delta_yaw_imu_deg [in]  variation de yaw (en degrés) sur l'intervalle
 * @param delta_motor_left_ticks [in]  variation de ticks du codeur gauche depuis le dernier cycle
 * @param delta_motor_right_ticks [in] variation de ticks du codeur droit depuis le dernier cycle
 * @param dt_s [in] durée (secondes) entre deux appels
 * @param theta_deg [in] orientation précédente du robot (en degrés)
 *
 * @param delta_x_m [out]   déplacement en x (m) depuis le dernier appel
 * @param delta_y_m [out]   déplacement en y (m) depuis le dernier appel
 * @param delta_theta_deg [out] variation d'angle (degrés) depuis le dernier appel
 *
 * @param alpha_orientation_ratio [in] pondération [0..1] de la fusion orientation (odom vs IMU)
 * @param ticks_per_rev [in] nombre de ticks par tour de roue
 * @param wheel_circumference_m [in] circonférence de la roue (m)
 * @param wheel_base_m [in] empattement (distance entre les roues) (m)
 */
void fusion_odo_imu_fuse(
    float ax_imu_g, float ay_imu_g,
    float delta_yaw_imu_deg,
    int delta_motor_left_ticks, int delta_motor_right_ticks,
    float dt_s,
    float theta_deg,
    float *delta_x_m, float *delta_y_m, float *delta_theta_deg,
    const float alpha_orientation_ratio,
    const float ticks_per_rev,
    const float wheel_circumference_m,
    const float wheel_base_m
)
{
    //--------------------------------------------------------------------------
    // 1) Conversion des ticks en distances
    //--------------------------------------------------------------------------
    // Les ticks sont déjà des delta (depuis le dernier cycle),
    // donc on convertit directement en mètres.
    float dist_left_m  = ( (float)delta_motor_left_ticks  / ticks_per_rev ) * wheel_circumference_m;
    float dist_right_m = ( (float)delta_motor_right_ticks / ticks_per_rev ) * wheel_circumference_m;
    float dist_avg_m   = 0.5f * (dist_left_m + dist_right_m);

    //--------------------------------------------------------------------------
    // 2) Variation d’angle calculée par l’odométrie (en degrés)
    //--------------------------------------------------------------------------
    // Δθ (radians) = (dist_right_m - dist_left_m) / wheel_base_m
    // => converti ensuite en degrés
    float delta_theta_odom_deg = ((dist_right_m - dist_left_m) / wheel_base_m)
                                * (180.0f / (float)M_PI);

    //--------------------------------------------------------------------------
    // 3) Variation d’angle IMU (en degrés) fournie directement : delta_yaw_imu_deg
    //--------------------------------------------------------------------------

    //--------------------------------------------------------------------------
    // 4) Fusion de la variation d'angle (filtre complémentaire)
    //--------------------------------------------------------------------------
    // alpha_orientation_ratio = 1.0 => tout l'odométrie
    // alpha_orientation_ratio = 0.0 => tout l'IMU
    float fused_delta_theta_deg =
        alpha_orientation_ratio         * delta_theta_odom_deg +
        (1.0f - alpha_orientation_ratio) * delta_yaw_imu_deg;

    // On fournit en sortie la variation d'angle du cycle
    *delta_theta_deg = fused_delta_theta_deg;

    //--------------------------------------------------------------------------
    // 5) Calcul de l'orientation moyenne (pour projeter la translation)
    //--------------------------------------------------------------------------
    // On suppose que, sur dt, l'orientation a varié de fused_delta_theta_deg.
    // => orientation moyenne = (theta_deg + (theta_deg + delta_theta)) / 2
    // Simplifié : theta_deg + (delta_theta / 2)
    float orientation_mid_deg = theta_deg + (fused_delta_theta_deg * 0.5f);
    float orientation_mid_rad = orientation_mid_deg * ((float)M_PI / 180.0f);

    //--------------------------------------------------------------------------
    // 6) Projection en Δx, Δy
    //--------------------------------------------------------------------------
    float dx_m = dist_avg_m * cosf(orientation_mid_rad);
    float dy_m = dist_avg_m * sinf(orientation_mid_rad);

    *delta_x_m = dx_m;
    *delta_y_m = dy_m;

    //--------------------------------------------------------------------------
    // 7) On ne met plus à jour de yaw IMU précédent (inutilisé),
    //    la struct 'fusion_odo_imu_t' est ici si besoin d'autres mémoires.
    //--------------------------------------------------------------------------
    // Exemple : fusion_odo_imu->previous_float_yaw_imu_deg = ??;
    //           (on ne fait rien pour l'instant)

    //--------------------------------------------------------------------------
    // Note : On ne fait pas (ici) de pondération supplémentaire par ax_imu_g, ay_imu_g,
    //        mais c'est possible de moduler alpha_orientation_ratio selon l'accélération
    //        latérale si on suspecte du glissement.
    //--------------------------------------------------------------------------
}

///OLD
/**
 * @brief
 *  Fusion simple de l'odométrie (2 codeurs) et de l'IMU (yaw).
 *  Retourne les incréments de déplacement (delta_x_m, delta_y_m, delta_theta_deg)
 *  sur l'intervalle [t-1, t], à partir :
 *    - ax_imu_g, ay_imu_g : accélération latérale/longitudinale (éventuellement utilisable)
 *    - yaw_imu_deg       : angle IMU absolu (en degrés)
 *    - delta_motor_left_ticks, delta_motor_right_ticks : variations de ticks des codeurs
 *    - dt_s : temps écoulé (en secondes)
 *    - theta_deg : orientation précédente du robot (en degrés)
 *    - alpha_orientation_ratio : [0..1] poids de l'odométrie vs IMU pour la variation d'angle
 *    - ticks_per_rev, wheel_circumference_m, wheel_base_m : constantes mécaniques du robot
 *
 * @param fusion_odo_imu [in/out] structure conservant l'état nécessaire (ici le yaw IMU précédent)
 * @param ax_imu_g [in]  accélération sur X en G (IMU), potentiellement exploitable pour la confiance
 * @param ay_imu_g [in]  accélération sur Y en G (IMU), potentiellement exploitable pour la confiance
 * @param yaw_imu_deg [in]  angle Yaw de l'IMU (en degrés), "absolu" ou déjà filtré
 * @param delta_motor_left_ticks [in]  variation de ticks du codeur gauche depuis le dernier cycle
 * @param delta_motor_right_ticks [in] variation de ticks du codeur droit depuis le dernier cycle
 * @param dt_s [in] durée (secondes) entre deux appels
 * @param theta_deg [in] orientation précédente du robot (degrés)
 *
 * @param delta_x_m [out]   déplacement en x (m) depuis le dernier appel
 * @param delta_y_m [out]   déplacement en y (m) depuis le dernier appel
 * @param delta_theta_deg [out] variation d'angle (degrés) depuis le dernier appel
 *
 * @param alpha_orientation_ratio [in] pondération [0..1] de la fusion orientation (odom vs IMU)
 * @param ticks_per_rev [in] nombre de ticks par tour de roue
 * @param wheel_circumference_m [in] circonférence de la roue (m)
 * @param wheel_base_m [in] empattement (distance entre les roues) (m)
 */

//
//
//void fusion_odo_imu_fuse_(
//    struct fusion_odo_imu_t *fusion_odo_imu,
//    float ax_imu_g, float ay_imu_g,  // non utilisés ici, mais disponibles si vous souhaitez pondérer
//    float yaw_imu_deg,
//    int delta_motor_left_ticks, int delta_motor_right_ticks,
//    float dt_s,
//    float theta_deg,
//    float *delta_x_m, float *delta_y_m, float *delta_theta_deg,
//    const float alpha_orientation_ratio,
//    const float ticks_per_rev,
//    const float wheel_circumference_m,
//    const float wheel_base_m
//)
//{
//    //--------------------------------------------------------------------------
//    // 1) Conversion des ticks en distances
//    //--------------------------------------------------------------------------
//    // Les ticks sont déjà des delta (depuis le dernier cycle),
//    // donc on convertit directement en mètres.
//    float dist_left_m  = ( (float)delta_motor_left_ticks  / ticks_per_rev ) * wheel_circumference_m;
//    float dist_right_m = ( (float)delta_motor_right_ticks / ticks_per_rev ) * wheel_circumference_m;
//    float dist_avg_m   = 0.5f * (dist_left_m + dist_right_m);
//
//    //--------------------------------------------------------------------------
//    // 2) Variation d’angle calculée par l’odométrie (en degrés)
//    //--------------------------------------------------------------------------
//    // Δθ (radians) = (dist_right_m - dist_left_m) / wheel_base_m
//    // => converti ensuite en degrés
//    float delta_theta_odom_deg = ((dist_right_m - dist_left_m) / wheel_base_m) * (180.0f / (float)M_PI);
//
//    //--------------------------------------------------------------------------
//    // 3) Variation d’angle mesurée par l'IMU (en degrés)
//    //--------------------------------------------------------------------------
//    float delta_theta_imu_deg = yaw_imu_deg - fusion_odo_imu->previous_float_yaw_imu_deg;
//
//    //--------------------------------------------------------------------------
//    // 4) Fusion de la variation d'angle (filtre complémentaire)
//    //--------------------------------------------------------------------------
//    // alpha_orientation_ratio = 1.0 => tout l'odométrie
//    // alpha_orientation_ratio = 0.0 => tout l'IMU
//    float fused_delta_theta_deg =
//        alpha_orientation_ratio * delta_theta_odom_deg +
//        (1.0f - alpha_orientation_ratio) * delta_theta_imu_deg;
//
//    // On fournit en sortie la variation d'angle du cycle
//    *delta_theta_deg = fused_delta_theta_deg;
//
//    //--------------------------------------------------------------------------
//    // 5) Calcul de l'orientation moyenne (pour projeter la translation)
//    //--------------------------------------------------------------------------
//    // On suppose que, sur dt, l'orientation a varié de fused_delta_theta_deg.
//    // => orientation moyenne = (theta_deg + (theta_deg + delta_theta)) / 2
//    // Simplifié : theta_deg + (delta_theta / 2)
//    float orientation_mid_deg = theta_deg + (fused_delta_theta_deg * 0.5f);
//    float orientation_mid_rad = orientation_mid_deg * ((float)M_PI / 180.0f);
//
//    //--------------------------------------------------------------------------
//    // 6) Projection en Δx, Δy
//    //--------------------------------------------------------------------------
//    float dx_m = dist_avg_m * cosf(orientation_mid_rad);
//    float dy_m = dist_avg_m * sinf(orientation_mid_rad);
//
//    *delta_x_m = dx_m;
//    *delta_y_m = dy_m;
//
//    //--------------------------------------------------------------------------
//    // 7) Mise à jour de la structure (pour le prochain cycle)
//    //--------------------------------------------------------------------------
//    fusion_odo_imu->previous_float_yaw_imu_deg = yaw_imu_deg;
//
//    //--------------------------------------------------------------------------
//    // Note : On ne fait pas (ici) de pondération supplémentaire par ax_imu_g, ay_imu_g,
//    //        mais c'est possible de moduler alpha_orientation_ratio selon l'accélération
//    //        latérale si on suspecte du glissement.
//    //--------------------------------------------------------------------------
//}
//
