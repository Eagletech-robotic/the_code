#include "robotic/fusion_odo_imu.hpp"

// Il s'agit d'un moyen de mélanger les données des odomètres et de la central intertiel (IMU)
// L'IMU donne une orientation absolue pas trop mauvaise
// l'odomètre donne une position très précise si il n'y a pas de glissement
// L'idéal serait de faire une fusion de capteur avec un filtre de kalmann.
// ici je fais plus simple, j'utilise une fusion linéiaire l'orientation de l'IMU et l'avancement de l'odomètre

#include "utils/myprintf.hpp"
#include <cmath>
#include <cstdio>

/**
 * @brief
 *  Fusion simple de l'odométrie (2 codeurs) et de l'IMU (delta de yaw).
 *  Retourne les incréments de déplacement (out_delta_x, out_delta_y, delta_theta)
 *  sur l'intervalle [t-1, t], à partir :
 *    - ax_imu_g, ay_imu_g : accélération latérale/longitudinale (éventuellement utilisable)
 *    - delta_yaw_imu : variation d'angle (en radians) mesurée par l'IMU depuis le dernier cycle
 *    - delta_motor_left_ticks, delta_motor_right_ticks : variations de ticks des codeurs
 *    - dt : temps écoulé (en secondes)
 *    - theta : orientation précédente du robot (en radians)
 *    - alpha_orientation_ratio : [0..1] poids de l'odométrie vs IMU pour la variation d'angle
 *    - ticks_per_rev, wheel_circumference_m, wheel_base_m : constantes mécaniques du robot
 *
 * @param fusion_odo_imu [in/out] structure (optionnelle ici, conservée pour compatibilité)
 * @param ax_imu_g [in]  accélération sur X en G (IMU), potentiellement exploitable pour la confiance
 * @param ay_imu_g [in]  accélération sur Y en G (IMU), potentiellement exploitable pour la confiance
 * @param delta_yaw_imu [in]  variation de yaw (en radians) sur l'intervalle
 * @param delta_motor_left_ticks [in]  variation de ticks du codeur gauche depuis le dernier cycle
 * @param delta_motor_right_ticks [in] variation de ticks du codeur droit depuis le dernier cycle
 * @param dt [in] durée (secondes) entre deux appels
 * @param theta [in] orientation précédente du robot (en radians)
 *
 * @param out_delta_x [out]   déplacement en x (m) depuis le dernier appel
 * @param out_delta_y [out]   déplacement en y (m) depuis le dernier appel
 * @param out_delta_theta [out] variation d'angle (radians) depuis le dernier appel
 *
 * @param alpha_orientation_ratio [in] pondération [0..1] de la fusion orientation (odom vs IMU)
 * @param ticks_per_rev [in] nombre de ticks par tour de roue
 * @param wheel_circumference_m [in] circonférence de la roue (m)
 * @param wheel_base_m [in] empattement (distance entre les roues) (m)
 */
void fusion_odo_imu_fuse(float /*ax_imu_g*/, float /*ay_imu_g*/, float delta_yaw_imu, int delta_motor_left_ticks,
                         int delta_motor_right_ticks, float /*dt*/, float theta, float *out_delta_x, float *out_delta_y,
                         float *out_delta_theta, const float alpha_orientation_ratio, const float ticks_per_rev,
                         const float wheel_circumference_m, const float wheel_base_m) {
    //--------------------------------------------------------------------------
    // 1) Conversion des ticks en distances
    //--------------------------------------------------------------------------
    // Les ticks sont déjà des delta (depuis le dernier cycle),
    // donc on convertit directement en mètres.
    float dist_left_m = ((float)delta_motor_left_ticks / ticks_per_rev) * wheel_circumference_m;
    float dist_right_m = ((float)delta_motor_right_ticks / ticks_per_rev) * wheel_circumference_m;
    float dist_avg_m = 0.5f * (dist_left_m + dist_right_m);

    //--------------------------------------------------------------------------
    // 2) Variation d'angle calculée par l'odométrie
    //--------------------------------------------------------------------------
    // Δθ (radians) = (dist_right_m - dist_left_m) / wheel_base_m
    float delta_theta_odom = (dist_right_m - dist_left_m) / wheel_base_m;

    //--------------------------------------------------------------------------
    // 3) Variation d'angle IMU fournie directement : delta_yaw_imu
    //--------------------------------------------------------------------------

    //--------------------------------------------------------------------------
    // 4) Fusion de la variation d'angle (filtre complémentaire)
    //--------------------------------------------------------------------------
    // alpha_orientation_ratio = 1.0 => tout l'odométrie
    // alpha_orientation_ratio = 0.0 => tout l'IMU
    float fused_delta_theta =
        alpha_orientation_ratio * delta_theta_odom + (1.0f - alpha_orientation_ratio) * delta_yaw_imu;

    // On fournit en sortie la variation d'angle du cycle
    *out_delta_theta = fused_delta_theta;

    //--------------------------------------------------------------------------
    // 5) Calcul de l'orientation moyenne (pour projeter la translation)
    //--------------------------------------------------------------------------
    // On suppose que, sur dt, l'orientation a varié de fused_delta_theta.
    // => orientation moyenne = (theta + (theta + out_delta_theta)) / 2
    // Simplifié : theta + (out_delta_theta / 2)
    float orientation_mid = theta + fused_delta_theta * 0.5f;

    //--------------------------------------------------------------------------
    // 6) Projection en Δx, Δy
    //--------------------------------------------------------------------------
    float dx = dist_avg_m * cosf(orientation_mid);
    float dy = dist_avg_m * sinf(orientation_mid);

    *out_delta_x = dx;
    *out_delta_y = dy;

    //--------------------------------------------------------------------------
    // 7) On ne met plus à jour de yaw IMU précédent (inutilisé),
    //    la struct 'fusion_odo_imu_t' est ici si besoin d'autres mémoires.
    //--------------------------------------------------------------------------
    // Exemple : fusion_odo_imu->previous_float_yaw_imu = ??;
    //           (on ne fait rien pour l'instant)

    //--------------------------------------------------------------------------
    // Note : On ne fait pas (ici) de pondération supplémentaire par ax_imu_g, ay_imu_g,
    //        mais c'est possible de moduler alpha_orientation_ratio selon l'accélération
    //        latérale si on suspecte du glissement.
    //--------------------------------------------------------------------------
}
