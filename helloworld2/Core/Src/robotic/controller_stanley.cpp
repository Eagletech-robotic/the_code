/*
 * controller_stanley.c
 *
 *  Created on: Apr 8, 2025
 *      Author: nboulay
 */
#include <math.h>
#include <stdbool.h>
#include <stdio.h>

/**
 * @brief Stanley Controller simplifié pour un robot différentiel.
 *
 * @param robot_x_m, robot_y_m, robot_theta_deg : Position et orientation actuelles du robot (en mètres, degrés).
 * @param x_start_m, y_start_m : Point de départ, définissant la trajectoire-ligne à suivre (jusqu'à x_target_m,
 * y_target_m).
 * @param x_target_m, y_target_m : Point cible final de la ligne.
 * @param x_next_m, y_next_m : Point suivant après la cible, pour gérer le virage à l'arrivée.
 * @param Vmax : Vitesse linéaire maximale (m/s).
 * @param Wmax : Vitesse angulaire maximale (rad/s) du robot.
 * @param kStanley : Coefficient du contrôleur Stanley (plus grand => plus réactif sur l'erreur latérale).
 * @param wheelBase_m : Entraxe du robot (distance entre les deux roues).
 * @param arrivalThreshold : Distance en-dessous de laquelle on considère être “arrivé”.
 * @param out_speed_right, out_speed_left : Résultats, vitesses des roues (m/s).
 * @return true si on est arrivé à target
 */
bool stanley_controller(float robot_x_m, float robot_y_m, float robot_theta_deg, float x_start_m, float y_start_m,
                        float x_target_m, float y_target_m, float x_next_m, float y_next_m, float Vmax, float Wmax,
                        float kStanley, float wheelBase_m, float arrivalThreshold, float *out_speed_left,
                        float *out_speed_right) {
    //------------------------------------------------------------------
    // 1) Calcul de la distance du robot à la cible
    //------------------------------------------------------------------
    float dx_to_target = x_target_m - robot_x_m;
    float dy_to_target = y_target_m - robot_y_m;
    float dist_to_target = sqrtf(dx_to_target * dx_to_target + dy_to_target * dy_to_target);

    //------------------------------------------------------------------
    // 2) Si on est proche de la cible, on tourne vers x_next_m, y_next_m
    //------------------------------------------------------------------
    //   => on diminue fortement la vitesse linéaire,
    //      et on oriente le robot vers la direction de la prochaine étape.
    //------------------------------------------------------------------
    if (dist_to_target < arrivalThreshold) {
        // 2.1) Calcul de l'angle souhaité pour pointer vers (x_next_m, y_next_m)
        float dx_next = x_next_m - robot_x_m;
        float dy_next = y_next_m - robot_y_m;
        float desired_heading_rad = atan2f(dy_next, dx_next); // [radians]

        // 2.2) Conversion de la pose robot en radians
        float robot_theta_rad = robot_theta_deg * ((float)M_PI / 180.0f);

        // 2.3) Erreur d'angle
        float heading_error = desired_heading_rad - robot_theta_rad;
        // Normalisation [-pi, +pi]
        while (heading_error > M_PI)
            heading_error -= 2.0f * (float)M_PI;
        while (heading_error < -M_PI)
            heading_error += 2.0f * (float)M_PI;

        // 2.4) On avance très lentement (ou pas du tout) et on tourne pour s'orienter
        float v = 0.0f; // on peut mettre un petit v = 100.0f si on veut continuer d'avancer
        // Contrôle proportionnel sur l'erreur d'angle
        float w =
            heading_error * 2000.0 * wheelBase_m * 0.5f; // (on pourrait mettre un gain, ex. w = 1.5f * heading_error)

        // Saturation en vitesse angulaire
        if (w > Wmax)
            w = Wmax;
        if (w < -Wmax)
            w = -Wmax;

        // Calcul des vitesses roues
        float v_left = v - w;
        float v_right = v + w;

        *out_speed_left = v_left;
        *out_speed_right = v_right;
        return heading_error < 0.1;
    }

    //------------------------------------------------------------------
    // 3) Contrôleur Stanley sur la ligne (x_start_m, y_start_m) -> (x_target_m, y_target_m)
    //------------------------------------------------------------------
    // 3.1) Calcul de l'angle de la ligne ("chemin")
    //------------------------------------------------------------------
    float path_dx = x_target_m - x_start_m;
    float path_dy = y_target_m - y_start_m;
    float path_heading_rad = atan2f(path_dy, path_dx); // orientation de la ligne

    // 3.2) Conversion de l'orientation robot en radians
    float robot_heading_rad = robot_theta_deg * ((float)M_PI / 180.0f);

    // 3.3) Erreur de cap (heading) : angle de la ligne - angle robot
    float heading_error = path_heading_rad - robot_heading_rad;
    // Normalisation dans [-pi..+pi]
    while (heading_error > M_PI)
        heading_error -= 2.0f * (float)M_PI;
    while (heading_error < -M_PI)
        heading_error += 2.0f * (float)M_PI;

    // 3.4) Calcul du cross-track (distance latérale à la ligne)
    //     Formule pour distance point-ligne :
    //        crossTrack = ((x_target - x_start)*(y_start - robot_y)
    //                     - (y_target - y_start)*(x_start - robot_x)) / norm(path)
    //     On utilise le signe pour savoir de quel côté de la ligne on se trouve
    float path_len = sqrtf(path_dx * path_dx + path_dy * path_dy);
    float crossTrack = 0.0f;
    if (path_len > 1e-6f) {
        crossTrack = (path_dx * (y_start_m - robot_y_m) - path_dy * (x_start_m - robot_x_m)) / path_len;
    }

    // 3.5) Vitesse linéaire : on avance à Vmax
    float v = Vmax; // (on peut moduler selon la courbure ou l'éloignement)

    // 3.6) Contrôle Stanley : Steering = heading_error + arctan(k * crosstrack / vitesse)
    //     => On va convertir ce 'steering' (en rad) en vitesse angulaire w.
    //     Pour un robot différentiel, steering ~ w * (wheelBase / v)
    //     => w = (v / wheelBase) * steering
    float crosstrack_correction = 0.0f;
    if (fabsf(v) > 1e-6f) {
        crosstrack_correction = atanf(kStanley * crossTrack / v);
    }
    float steering_angle = heading_error + crosstrack_correction;

    // 3.7) Conversion Steering -> w
    float w = (v / wheelBase_m) * steering_angle; // rad/s

    // 3.8) Saturation en vitesse angulaire
    if (w > Wmax)
        w = Wmax;
    if (w < -Wmax)
        w = -Wmax;

    //------------------------------------------------------------------
    // 4) Conversion (v, w) en (v_left, v_right) pour un différentiel
    //------------------------------------------------------------------
    // v_left  = v - (w * wheelBase/2)
    // v_right = v + (w * wheelBase/2)
    float v_left = v - (w * wheelBase_m / 2.0f);
    float v_right = v + (w * wheelBase_m / 2.0f);

    // On peut éventuellement resaturer si on dépasse la vitesse max
    // (ex. si w est énorme, on veut éviter des vitesses négatives trop grandes)
    float max_rpm = fmaxf(fabsf(v_left), fabsf(v_right));
    if (max_rpm > Vmax) {
        float scale = Vmax / max_rpm;
        v_left *= scale;
        v_right *= scale;
    }

    *out_speed_left = v_left;
    *out_speed_right = v_right;
    return false;
}

/**
 * @brief Contrôleur différentiel simple pour conduire le robot (x, y, theta)
 *        vers la cible (x_target_m, y_target_m).
 *
 *        Hypothèses :
 *         - On fait un simple contrôle proportionnel pour la distance et l'angle.
 *         - En différentiel, (v, w) -> (vG, vD).
 *         - Si la distance à la cible < arrivalThreshold, on s'arrête (v=0).
 *
 * @param robot_x_m       Position X actuelle du robot (m)
 * @param robot_y_m       Position Y actuelle du robot (m)
 * @param robot_theta_deg Orientation actuelle du robot (en degrés)
 * @param x_target_m      Cible X (m)
 * @param y_target_m      Cible Y (m)
 * @param Vmax            Vitesse linéaire max (m/s)
 * @param wheelBase_m     Entraxe du robot (distance entre roues) (m)
 * @param arrivalThreshold Distance en dessous de laquelle on considère le robot "arrivé"
 * @param out_speed_left [out] Vitesse roue gauche (m/s)
 * @param out_speed_right  [out] Vitesse roue droite (m/s)
 *
 * @return true si le robot est dans le rayon d'arrivée, sinon false
 */
bool controller_pid(float robot_x_m, float robot_y_m, float robot_theta_deg, float x_target_m, float y_target_m,
                    float Vmax, float wheelBase_m, float arrivalThreshold, float *out_speed_left,
                    float *out_speed_right) {
    //----------------------------------------------------------------------
    // 1) Calcul de la distance à la cible
    //----------------------------------------------------------------------
    float dx = x_target_m - robot_x_m;
    float dy = y_target_m - robot_y_m;
    float distance = sqrtf(dx * dx + dy * dy);

    // Si on est dans la zone d'arrivée, on s'arrête
    if (distance <= arrivalThreshold) {
        *out_speed_left = 0.0f;
        *out_speed_right = 0.0f;
        return true; // arrivé
    }

    //----------------------------------------------------------------------
    // 2) Calcul de l'angle vers la cible
    //----------------------------------------------------------------------
    float desired_angle_rad = atan2f(dy, dx); // en radians
    float robot_angle_rad = robot_theta_deg * (float)M_PI / 180.0f;

    float error_angle_rad = desired_angle_rad - robot_angle_rad;

    // Normalisation dans [-pi, +pi]
    while (error_angle_rad > M_PI)
        error_angle_rad -= 2.0f * (float)M_PI;
    while (error_angle_rad < -M_PI)
        error_angle_rad += 2.0f * (float)M_PI;

    //----------------------------------------------------------------------
    // 3) Contrôle proportionnel distance & angle
    //----------------------------------------------------------------------
    //   v ~ Kp_dist * distance
    //   w ~ Kp_angle * angle_error
    // Ajustez ces gains au besoin
    //----------------------------------------------------------------------
    const float Kp_dist = 5.0f;  // Gain proportionnel distance
    const float Kp_angle = 3.0f; // Gain proportionnel angle

    float v = Kp_dist * distance;           // m/s
    float w = Kp_angle * (error_angle_rad); // rad/s

    // Saturation linéaire
    if (v > Vmax)
        v = Vmax;

    //----------------------------------------------------------------------
    // 4) Conversion (v, w) -> (vGauche, vDroite)
    //----------------------------------------------------------------------
    //  vG = v - (w * (wheelBase/2))
    //  vD = v + (w * (wheelBase/2))
    float v_left = v - (w * (wheelBase_m * 0.5f));
    float v_right = v + (w * (wheelBase_m * 0.5f));

    //----------------------------------------------------------------------
    // 5) Éventuellement saturer si l'une des roues dépasse ±Vmax
    //----------------------------------------------------------------------
    float max_abs = fmaxf(fabsf(v_left), fabsf(v_right));
    if (max_abs > Vmax) {
        float scale = Vmax / max_abs;
        v_left *= scale;
        v_right *= scale;
    }

    //----------------------------------------------------------------------
    // 6) Sorties
    //----------------------------------------------------------------------
    *out_speed_left = v_left;
    *out_speed_right = v_right;

    return false; // en cours
}
