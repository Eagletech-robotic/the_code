/*
 * carre.c
 *
 *  Created on: Aug 30, 2024
 *      Author: nboulay
 */

// le but est de faire parcourir au robot un carré de taille 1m par 1m de façon répété
//  cela permet de vérifier la précision du positionnement
//  ce fichier devrait être un maximum indépendant de la façon de fonctionner du robot pour permettre de réutiliser ce
//  test

// avec maxima :
// (%i1) solve(v2/v1 = (R-e/2)/(R+e/2),R);
//
//                                     e v2 + e v1
//(%o1)                         [R = - -----------]
//                                     2 v2 - 2 v1
// soit R = e* (v1+v2) / (v1-v2)

#include "robotic/carre.hpp"
#include "robotic/projection.hpp"
#include <math.h>
#include <stdio.h>

void carre_init(carre_t *carre, float cycle_period_s) {
    // output_init(carre->output);
    carre->time_from_start_s = 0.0;
    carre->start_sequence_time_s = 0.0;
    carre->timer_period_s = cycle_period_s; // cela dépend de la vitesse de cycle choisi à la configuration
    printf("séquence %f\r\n", cycle_period_s);
}
//
// calcul de vitesses avec la vitesse moyenne et le rayon de courbure voulu selon l'entre-axe
void r_to_v(float /*r*/, float /*v*/, float * /*v1*/, float * /*v2*/) {
    //     //    *v1 = v * (1 + WHEELBASE_M / (2 * r));
    //     //    *v2 = v * (1 - WHEELBASE_M / (2 * r));
}
//
int carre_is_elapsed_time(carre_t *c, float t_s) { return c->time_from_start_s < (c->start_sequence_time_s + t_s); }

void carre_sequence(carre_t * /*c*/, output_t * /*output*/) {
    //    float v_r = 1000.0;      // 2000
    //    float v_curve_r = 750.0; // 1500
    //    float t_curve_s = 0.65f;
    //    float t_line_s = 0.5;
    //    if (carre_is_elapsed_time(c, t_line_s)) {
    //        // ligne de droite
    //        output->motor_left_ratio = v_r;
    //        output->motor_right_ratio = v_r;
    //    } else if (carre_is_elapsed_time(c, t_line_s + t_curve_s)) {
    //        // demi courbe
    //        float v1, v2;
    //        r_to_v(.2, v_curve_r, &v1, &v2);
    //        output->motor_left_ratio = v1;
    //        output->motor_right_ratio = v2;
    //    } else {
    //        // sequence terminée, on recommence
    //        c->start_sequence_time_s = c->time_from_start_s;
    //    }
    //    // marche avant : TODO: mettre la vitesse positive en marche avant
    //    output->motor_left_ratio = -output->motor_left_ratio;
    //    output->motor_right_ratio = -output->motor_right_ratio;
}

// typedef struct line_t {
//	int started;
//	float start_x_m;
//	float start_y_m;
//	float start_theta_deg;
// } line_t;
//
// void line_init(line_t *line) {
//	line->start_x_m = 0.0f;
//	line->start_y_m = 0.0f;
//	line->start_theta_deg = 0.0f;
//	line->first_call=1;
// }
//
//// avance sur une ligne
//// retourne true quand la ligne est faite, false sinon
// int line_step(line * line, float v, float d_m, float robot_x, float robot_y, float theta_deg) {
//	if(line->first_call) {
//		line->
//	}
// }

// void carre_output_commit(carre_t * c) {
////	printf("v : %f %f \r\n", c->output.ratio15, c->output.ratio2);
//	output_set(c->output);
//}

void carre_in_loop(carre_t *c, output_t *output) {
    c->time_from_start_s += c->timer_period_s; // il y a peut être un timer interne plus précis
    carre_sequence(c, output);
}

//////////////////// OLD stuff /// Delete ?

void carre_in_loop_with_mag(carre_t * /*c*/, const float /*mag*/[3], output_t * /*output*/) {
    // c->time_from_start_s += c->timer_period_s; // il y a peut être un timer interne plus précis
    //
    // float v_r = 1000.0;
    // float v_curve_r = 1000.0;
    // // float t_curve_s = 0.65f;
    // float t_line_s = 0.5;
    // if (carre_is_elapsed_time(c, t_line_s)) {
    //     // ligne de droite
    //     output->motor_left_ratio = v_r;
    //     output->motor_right_ratio = v_r;
    //     c->mag[0] = mag[0];
    //     c->mag[1] = mag[1];
    //     c->mag[2] = mag[2];
    //     printf("d\r\n");
    // } else if (angleBetweenVectors(c->mag, mag) < .15) {
    //     // demi courbe
    //     float v1, v2;
    //     r_to_v(.2, v_curve_r, &v1, &v2);
    //     output->motor_left_ratio = v1;
    //     output->motor_right_ratio = v2;
    // } else {
    //     // sequence terminée, on recommence
    //     c->start_sequence_time_s = c->time_from_start_s;
    //     printf("RRR\r\n");
    // }
}

void carre_in_loop_with_heading(carre_t * /*c*/, const float /*heading*/, output_t * /*output*/) {
    // c->time_from_start_s += c->timer_period_s; // il y a peut être un timer interne plus précis
    //
    // float v_r = 1000.0;
    // float v_curve_r = 1000.0;
    // // float t_curve_s = 0.65f;
    // float t_line_s = 0.5;
    // if (carre_is_elapsed_time(c, t_line_s)) {
    //     // ligne de droite
    //     output->motor_left_ratio = v_r;
    //     output->motor_right_ratio = v_r;
    //     c->mag[0] = heading;
    //     printf("d\r\n");
    // } else if (fabs(c->mag[0] - heading) < 5.0) {
    //     // demi courbe
    //     float v1, v2;
    //     r_to_v(.3, v_curve_r, &v1, &v2);
    //     output->motor_left_ratio = v1;
    //     output->motor_right_ratio = v2;
    // } else {
    //     // sequence terminée, on recommence
    //     c->start_sequence_time_s = c->time_from_start_s;
    //     printf("RRR\r\n");
    // }
}

// calcul d'un déplacement selon un arc de cercle
// x0 y0 est sur l'arc de cercle sinon le calcul n'a pas de sens
// la distance est positive en sens horaire
void circleMov(float x0, float y0, float distance, float circleX, float circleY, float *x1, float *y1) {
    float circleR = sqrtf((x0 - circleX) * (x0 - circleX) + (y0 - circleY) * (y0 - circleY));
    float deltaTheta = distance / circleR;

    float theta0 = atan2f((y0 - circleY), (x0 - circleX));
    float theta1 = theta0 + deltaTheta;

    *x1 = circleX + circleR * cosf(theta1);
    *y1 = circleY + circleR * sinf(theta1);
    printf("from (%f;%f) %f circle (%f;%f) %f\r\n %f %f %f\r\n", x0, y0, distance, circleX, circleY, circleR,
           deltaTheta / M_PI, theta0 / M_PI, theta1 / M_PI);
}

// on considère égal 2 nombre à 1% (l'égalité n'a pas de sens avec des floats)
int floatEqual(float f, float expected) { return fabsf(f - expected) <= 0.01; }

int circleMov_test() {
    float x1, y1;
    circleMov(0.0, 0.0, M_PI / 2, 1.0, 0.0, &x1, &y1);
    int r = (floatEqual(x1, 1.0) && floatEqual(y1, -1.0));
    if (!r)
        printf(" %f %f != 1.0 -1.0\r\n", x1, y1);

    circleMov(1.0, 2.0, M_PI, 3.0, 2.0, &x1, &y1);
    r &= (floatEqual(x1, 3.0) && floatEqual(y1, 0.0));
    if (!r)
        printf(" %f %f != 3.0 0.0\r\n", x1, y1);

    circleMov(1.0, 2.0, 2.0 * M_PI, 3.0, 2.0, &x1, &y1);
    r &= (floatEqual(x1, 5.0) && floatEqual(y1, 2.0));
    if (!r)
        printf(" %f %f != 5.0 2.0\r\n", x1, y1);

    return r;
}

int carre_test() {
    int m = circleMov_test();

    return m;
}
