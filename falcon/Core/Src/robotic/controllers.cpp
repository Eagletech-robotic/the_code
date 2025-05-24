#include "robotic/controllers.hpp"

#include <math.h>
#include <stdio.h>

#include "robotic/pid.hpp"
#include "utils/angles.hpp"
#include "utils/myprintf.hpp"

/**
 * @brief Stanley Controller simplifié pour un robot différentiel.
 *
 * @param robot_x, robot_y, robot_theta : Position et orientation actuelles du robot (en mètres, radians).
 * @param x_start, y_start : Point de départ, définissant la trajectoire-ligne à suivre (jusqu'à x_target,
 * y_target).
 * @param x_target, y_target : Point cible final de la ligne.
 * @param x_next, y_next : Point suivant après la cible, pour gérer le virage à l'arrivée.
 * @param Vmax : Vitesse linéaire maximale (m/s).
 * @param Wmax : Vitesse angulaire maximale (rad/s) du robot.
 * @param Varrival : Vitesse à partir du franchissement de arrivalThreshold (m/s).
 * @param kStanley : Coefficient du contrôleur Stanley (plus grand => plus réactif sur l'erreur latérale).
 * @param wheelBase : Entraxe du robot (distance entre les deux roues).
 * @param arrivalThreshold : Distance en-dessous de laquelle on considère être “arrivé”.
 * @param out_speed_right, out_speed_left : Résultats, vitesses des roues (m/s).
 * @return true si on est arrivé à target
 */
bool stanley_controller(float robot_x, float robot_y, float robot_theta, float x_start, float y_start, float x_target,
                        float y_target, float x_next, float y_next, float Vmax, float Wmax, float Varrival,
                        float kStanley, float wheelBase, float arrivalThreshold, float *out_speed_left,
                        float *out_speed_right) {
    //------------------------------------------------------------------
    // 1) Calcul de la distance du robot à la cible
    //------------------------------------------------------------------
    float dx_to_target = x_target - robot_x;
    float dy_to_target = y_target - robot_y;
    float dist_to_target = sqrtf(dx_to_target * dx_to_target + dy_to_target * dy_to_target);

    //------------------------------------------------------------------
    // 2) Si on est proche de la cible, on tourne vers x_next, y_next
    //------------------------------------------------------------------
    //   => on diminue fortement la vitesse linéaire,
    //      et on oriente le robot vers la direction de la prochaine étape.
    //------------------------------------------------------------------
    if (dist_to_target < arrivalThreshold) {
        // 2.1) Calcul de l'angle souhaité pour pointer vers (x_next, y_next)
        float dx_next = x_next - robot_x;
        float dy_next = y_next - robot_y;
        float desired_heading = atan2f(dy_next, dx_next);

        // 2.3) Erreur d'angle
        float heading_error = angle_normalize(desired_heading - robot_theta);

        // 2.4) On avance très lentement (ou pas du tout) et on tourne pour s'orienter
        // Contrôle proportionnel sur l'erreur d'angle
        float w = heading_error * 2000.0 * wheelBase * 0.5f;
        // (on pourrait mettre un gain, ex. w = 1.5f * heading_error)

        // Saturation en vitesse angulaire
        if (w > Wmax)
            w = Wmax;
        if (w < -Wmax)
            w = -Wmax;

        // Calcul des vitesses roues
        float v_left = Varrival - w;
        float v_right = Varrival + w;

        *out_speed_left = v_left;
        *out_speed_right = v_right;

        myprintf("arrival phase v_left:%f v_right:%f\n", v_left, v_right);
        return heading_error < 0.1;
    }

    //------------------------------------------------------------------
    // 3) Contrôleur Stanley sur la ligne (x_start, y_start) -> (x_target, y_target)
    //------------------------------------------------------------------
    // 3.1) Calcul de l'angle de la ligne ("chemin")
    //------------------------------------------------------------------
    float path_dx = x_target - x_start;
    float path_dy = y_target - y_start;
    float path_heading = atan2f(path_dy, path_dx); // orientation de la ligne

    // 3.3) Erreur de cap (heading) : angle de la ligne - angle robot
    float heading_error = angle_normalize(path_heading - robot_theta);

    // 3.4) Calcul du cross-track (distance latérale à la ligne)
    //     Formule pour distance point-ligne :
    //        crossTrack = ((x_target - x_start)*(y_start - robot_y)
    //                     - (y_target - y_start)*(x_start - robot_x)) / norm(path)
    //     On utilise le signe pour savoir de quel côté de la ligne on se trouve
    float path_len = sqrtf(path_dx * path_dx + path_dy * path_dy);
    float crossTrack = 0.0f;
    if (path_len > 1e-6f) {
        crossTrack = (path_dx * (y_start - robot_y) - path_dy * (x_start - robot_x)) / path_len;
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
    myprintf("crossTrack: %f heading_error: %f crosstrack_correction: %f\n", crossTrack, to_degrees(heading_error),
             to_degrees(crosstrack_correction));

    // 3.7) Conversion Steering -> w
    float w = (v / wheelBase) * steering_angle; // rad/s

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
    float v_left = v - (w * wheelBase / 2.0f);
    float v_right = v + (w * wheelBase / 2.0f);

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
 *        vers la cible (x_target, y_target).
 *
 *        Hypothèses :
 *         - On fait un simple contrôle proportionnel pour la distance et l'angle.
 *         - En différentiel, (v, w) -> (vG, vD).
 *         - Si la distance à la cible < arrivalThreshold, on s'arrête (v=0).
 *
 * @param robot_x       Position X actuelle du robot (m)
 * @param robot_y       Position Y actuelle du robot (m)
 * @param robot_theta     Orientation actuelle du robot (en radians)
 * @param x_target      Cible X (m)
 * @param y_target      Cible Y (m)
 * @param Vmax            Vitesse linéaire max (m/s)
 * @param wheelBase     Entraxe du robot (distance entre roues) (m)
 * @param arrivalThreshold Distance en dessous de laquelle on considère le robot "arrivé"
 * @param out_speed_left [out] Vitesse roue gauche (m/s)
 * @param out_speed_right  [out] Vitesse roue droite (m/s)
 *
 * @return true si le robot est dans le rayon d'arrivée, sinon false
 */
bool pid_controller_1(float robot_x, float robot_y, float robot_theta, float x_target, float y_target, float Vmax,
                      float wheelBase, float arrivalThreshold, float *out_speed_left, float *out_speed_right) {
    //----------------------------------------------------------------------
    // 1) Calcul de la distance à la cible
    //----------------------------------------------------------------------
    float dx = x_target - robot_x;
    float dy = y_target - robot_y;
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
    float desired_angle = atan2f(dy, dx);
    float error_angle = angle_normalize(desired_angle - robot_theta);

    //----------------------------------------------------------------------
    // 3) Contrôle proportionnel distance & angle
    //----------------------------------------------------------------------
    //   v ~ Kp_dist * distance
    //   w ~ Kp_angle * angle_error
    // Ajustez ces gains au besoin
    //----------------------------------------------------------------------
    const float Kp_dist = 5.0f;  // Gain proportionnel distance
    const float Kp_angle = 2.0f; // Gain proportionnel angle

    float v = Kp_dist * distance;     // m/s
    float w = Kp_angle * error_angle; // rad/s

    // Saturation linéaire
    if (v > Vmax)
        v = Vmax;

    //----------------------------------------------------------------------
    // 4) Conversion (v, w) -> (vGauche, vDroite)
    //----------------------------------------------------------------------
    //  vG = v - (w * (wheelBase/2))
    //  vD = v + (w * (wheelBase/2))
    float v_left = v - (w * (wheelBase * 0.5f));
    float v_right = v + (w * (wheelBase * 0.5f));

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

PID_t *pid_theta;
PID_t *pid_speed;
// le but est d'avoir un pid pour le controler de cap et de vitesse linéaire
void controllers_pid_init(PID_t *pid_theta_, PID_t *pid_speed_) {
    pid_theta = pid_theta_;
    pid_speed = pid_speed_;
    pid_init(pid_theta);
    pid_tune(pid_theta, 300.0f, 0.0000000f, 0.f);
    pid_limits(pid_theta, -2.0f, 2.0f);
    pid_frequency(pid_theta, 250);

    pid_init(pid_speed);
    // avec ki =0 est le seul moyen d'avoir R inférieur à 1% d'erreur (autour de 12-15v), ki fait diverger
    pid_tune(pid_speed, 10.0f, 0.0000000f, 0.0f);
    pid_limits(pid_speed, -2.0f, 2.0f);
    pid_frequency(pid_speed, 250);
}

/**
 * @brief  Contrôleur différentiel « nerveux » :
 *         - Vmax = vitesse max de la roue la plus rapide
 *         - Correction d’angle agressive
 * @return true si le robot est arrivé, sinon false
 */
bool pid_controller(float robot_x, float robot_y, float robot_theta, float x_target, float y_target, float Vmax,
                    float wheelBase, float arrivalThreshold, float *out_speed_left, float *out_speed_right) {
    /* 1) Distance à la cible ------------------------------------------------*/
    float dx = x_target - robot_x;
    float dy = y_target - robot_y;
    float distance = sqrtf(dx * dx + dy * dy);

    if (distance <= arrivalThreshold) { /* On est arrivé */
        *out_speed_left = 0.f;
        *out_speed_right = 0.f;
        return true;
    }

    /* 2) Erreur angulaire ---------------------------------------------------*/
    float desired_angle = atan2f(dy, dx);
    float error_angle = angle_normalize(desired_angle - robot_theta);

    /* 3) Boucles proportionnelles ------------------------------------------*/
    const float Kp_dist = 10.0f;   /* distance → vitesse linéaire   */
    const float Kp_angle = 250.0f; /* angle    → vitesse angulaire  */

    /* Option : si l’angle est trop grand, on réduit v pour tourner vite (>45°)*/
    float v;
    if (fabsf(error_angle) > (M_PI / 4.f)) {
        v = 0.0f;
    } else {
        v = Kp_dist * distance;
    }

    float w = Kp_angle * error_angle; /* rad/s */

    /* 4) (v, w) → vitesses roues -------------------------------------------*/
    float halfBase = wheelBase * 0.5f;
    float v_left = v - w * halfBase;
    float v_right = v + w * halfBase;

    /* 5) Saturation par la roue la plus rapide -----------------------------*/
    float max_abs = fmaxf(fabsf(v_left), fabsf(v_right));
    if (max_abs > Vmax) {
        float scale = Vmax / max_abs;
        v_left *= scale;
        v_right *= scale;
    }

    /* 6) Sorties ------------------------------------------------------------*/
    *out_speed_left = v_left;
    *out_speed_right = v_right;
    return false; /* encore en route */
}
/**
 * @brief  Contrôleur différentiel utilisant 2 PID externes :
 *         - pid_speed  : distance  → vitesse linéaire  v  (m/s)
 *         - pid_theta  : erreur θ  → vitesse angulaire w  (rad/s)
 *         Vmax désigne toujours la limite ± de **la roue** la plus rapide.
 *
 * @return true si le robot est dans le rayon d’arrivée.
 */
bool pid_controller_kd(float robot_x, float robot_y, float robot_theta, float x_target, float y_target, float Vmax,
                       float wheelBase, float arrivalThreshold, float *out_speed_left, float *out_speed_right) {
    /* 1) Distance et angle vers la cible ---------------------------------- */
    float dx = x_target - robot_x;
    float dy = y_target - robot_y;
    float distance = sqrtf(dx * dx + dy * dy);

    if (distance <= arrivalThreshold) { /* on est arrivé */
        *out_speed_left = 0.f;
        *out_speed_right = 0.f;
        return true;
    }

    float desired_angle = atan2f(dy, dx);
    float err_theta = angle_normalize(desired_angle - robot_theta);

    /* 2) PIDs -------------------------------------------------------------- *
       Règle de base  :
         - on veut      distance → 0  ⇒  consigne = 0, mesure = distance
         - on veut erreurs θ  → 0     ⇒  consigne = 0, mesure = err_theta
       Or avec ces choix, l’erreur (Setpoint – Input) devient négative
       tant que le robot n’est pas aligné ⇒ la sortie des PID est négative.
       Pour garder un v (avance) ET un w (rotation) POSITIFS dans le même
       sens que distance et err_theta, il suffit d’inverser le signe.        */

    /* vitesse linéaire v (m/s) : seulement si la cible est ±90° devant     */
    float v = 0.f;
    if (fabsf(err_theta) < (M_PI / 2.f))
        v = -pid_(pid_speed, 0.0f, distance); /* - ( … )  ⇒ signe + */

    /* vitesse angulaire w (rad/s) : toujours active                         */
    float w = -pid_(pid_theta, 0.0f, err_theta); /* - ( … )  ⇒ signe + */

    /* 3) (v,w) → roues ----------------------------------------------------- */
    float h = 0.5f * wheelBase;
    float v_left = v - w * h;
    float v_right = v + w * h;

    /* 4) Saturation : la roue la plus rapide = ±Vmax ---------------------- */
    float maxAbs = fmaxf(fabsf(v_left), fabsf(v_right));
    if (maxAbs > Vmax) {
        float k = Vmax / maxAbs;
        v_left *= k;
        v_right *= k;
    }

    *out_speed_left = v_left;
    *out_speed_right = v_right;
    return false; /* en mouvement */
}
