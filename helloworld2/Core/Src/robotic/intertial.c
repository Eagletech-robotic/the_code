/*
 * intertial.c
 *
 *  Created on: Aug 30, 2024
 *      Author: nboulay
 */


#include "robotic/inertial.h"

#include <math.h>
#include <stdio.h>

// todo :
// remplacer les const par des paramètres
// faire une structure pour stoquer l'état interne et la vitesse
// -> vérifier la différence entre la vitesse entrée et recalculé
// appeler la fonction une fois par cycle
//

//TODO : ses valeurs doivent être des paramètres et non des constantes pour que le code soit réutilisable
const float r_wheel         = 0.065; //m ? rayon de roue ?
const float r_axel          = 0.33;  // m écartement des roues ?
const float delta_t         = 0.01;  // s pas de temps entre 2 lectures ? normalise la vitesse -> paramètres
const float pi              = 3.14159265358979;
//const float w_accel_max     = 10.0;
//const float deg_rad         = pi / 180.0;
const float holes           = 500.0;
//const float v_collect       = 0.0;
//const float v_max           = 2.0;
//const float sensor1_x       = 0.0;
//const float sensor1_y       = 0.0;
//const float sensor2_x       = 0.0;
//const float sensor2_y       = 0.0;
//

// on considère des petits pas de temps, la machine parcourent des arcs de cercles

// fonction original qui fonctionne avec les codeuses
int inertial(	/*DEBUG*/
		int isDebug,
		/* INPUTS */
		float w1_imp, // distance effectuée par la roue 1 en impulsion
		float w2_imp, // distance effectuée par la roue 2
		/* INPUTS/OUTPUTS */
		float* p_cg_i_ex,  // X
		float* p_cg_i_ey,  // Y
		float* theta_e,    // heading
		/* OUTPUTS */
		float* v_cg_i_ex, // vitesse
		float* v_cg_i_ey  //vitesse
	)
{
    /* Outputs from module */

    /* Local Variables */
    float a_cg_bx_e = 0;
    float v_cg_by_e = 0;
    float w_cg_ib_e = 0;

    float w1_mes = 0;
    float w2_mes = 0;

    float a_cg_i_ex = 0;
    float a_cg_i_ey = 0;

    float del_p_i_ex = 0;
    float del_p_i_ey = 0;

  //  float reste = -1;
    float div 	= -1;

/* Generate angle swept out since previous measurement by each wheel */
/* ----------------------------------------------------------------- */
    w1_mes = w1_imp * ((2*pi) / holes); // in radian ? distance / wheel ray
    w2_mes = w2_imp * ((2*pi) / holes);

/* Update r2d2 dynamics */
/* -------------------- */
    a_cg_bx_e = (r_wheel*r_wheel * (w1_mes*w1_mes - w2_mes*w2_mes)) / (2.0*r_axel); /* Normal Acceleration */
    v_cg_by_e = (r_wheel*(w1_mes + w2_mes))/2.0;                                    /* Tangential Velocity */
    w_cg_ib_e = (r_wheel/r_axel)*(w2_mes - w1_mes);                                 /* Rotation rate about axel centre */

/* Update rotation angle */
/* --------------------- */
    *theta_e = *theta_e + w_cg_ib_e;
    div = *theta_e/(2*pi);
    *theta_e = *theta_e - floor(div)*2*pi;
    if(isDebug)
        printf("\ntheta_e: %lf\ndiv: %lf\nfloor(div): %lf\n",*theta_e,div,floor(div));

/* Transform normal acceleration from Body to Inertial Axes */
/* -------------------------------------------------------- */
    a_cg_i_ex =      cos(*theta_e) * a_cg_bx_e;
    a_cg_i_ey = -1 * sin(*theta_e) * a_cg_bx_e;

/* Transform velocity from Body to Inertial Axes */
/* --------------------------------------------- */
    *v_cg_i_ex =  sin(*theta_e) * v_cg_by_e;
    *v_cg_i_ey =  cos(*theta_e) * v_cg_by_e;

/* Update R2D2 cofg position in Inertial Axes */
/* ------------------------------------------ */
    del_p_i_ex = (*v_cg_i_ex + 0.5 * a_cg_i_ex );
    del_p_i_ey = (*v_cg_i_ey + 0.5 * a_cg_i_ey );

    *p_cg_i_ex = *p_cg_i_ex + del_p_i_ex;
    *p_cg_i_ey = *p_cg_i_ey + del_p_i_ey;

/* Create proper estimated velocity output */
/* --------------------------------------- */
    *v_cg_i_ex = *v_cg_i_ex / delta_t;
    *v_cg_i_ey = *v_cg_i_ey / delta_t;

/* Output results to the screen */
/* ---------------------------- */
    if(isDebug)
        printf("acceleration = %6.10f\n", a_cg_bx_e);
    return 0;
}




int inertial_with_angle(	/*DEBUG*/
		int isDebug,
		/* INPUTS */
		float w1_mes, // tour de roue en radian
		float w2_mes, //
		/* INPUTS/OUTPUTS */
		float* p_cg_i_ex,  // X
		float* p_cg_i_ey,  // Y
		float* theta_e,    // heading
		/* OUTPUTS */
		float* v_cg_i_ex, // vitesse
		float* v_cg_i_ey  //vitesse
	)
{
    /* Outputs from module */

    /* Local Variables */
    float a_cg_bx_e = 0;
    float v_cg_by_e = 0;
    float w_cg_ib_e = 0;

    float a_cg_i_ex = 0;
    float a_cg_i_ey = 0;

    float del_p_i_ex = 0;
    float del_p_i_ey = 0;

    float div 	= -1;

/* Generate angle swept out since previous measurement by each wheel */
/* ----------------------------------------------------------------- */
//    w1_mes = w1_imp * ((2*pi) / holes); // in radian ? distance / wheel ray
//    w2_mes = w2_imp * ((2*pi) / holes);

/* Update r2d2 dynamics */
/* -------------------- */
    a_cg_bx_e = (r_wheel*r_wheel * (w1_mes*w1_mes - w2_mes*w2_mes)) / (2.0*r_axel); /* Normal Acceleration */
    v_cg_by_e = (r_wheel*(w1_mes + w2_mes))/2.0;                                    /* Tangential Velocity */
    w_cg_ib_e = (r_wheel/r_axel)*(w2_mes - w1_mes);                                 /* Rotation rate about axel centre */

/* Update rotation angle */
/* --------------------- */
    *theta_e = *theta_e + w_cg_ib_e;
    div = *theta_e/(2*pi);
    *theta_e = *theta_e - floor(div)*2*pi;
    if(isDebug)
        printf("\ntheta_e: %lf\ndiv: %lf\nfloor(div): %lf\n",*theta_e,div,floor(div));

/* Transform normal acceleration from Body to Inertial Axes */
/* -------------------------------------------------------- */
    a_cg_i_ex =      cos(*theta_e) * a_cg_bx_e;
    a_cg_i_ey = -1 * sin(*theta_e) * a_cg_bx_e;

/* Transform velocity from Body to Inertial Axes */
/* --------------------------------------------- */
    *v_cg_i_ex =  sin(*theta_e) * v_cg_by_e;
    *v_cg_i_ey =  cos(*theta_e) * v_cg_by_e;

/* Update R2D2 cofg position in Inertial Axes */
/* ------------------------------------------ */
    del_p_i_ex = (*v_cg_i_ex + 0.5 * a_cg_i_ex );
    del_p_i_ey = (*v_cg_i_ey + 0.5 * a_cg_i_ey );

    *p_cg_i_ex = *p_cg_i_ex + del_p_i_ex;
    *p_cg_i_ey = *p_cg_i_ey + del_p_i_ey;

/* Create proper estimated velocity output */
/* --------------------------------------- */
    *v_cg_i_ex = *v_cg_i_ex / delta_t;
    *v_cg_i_ey = *v_cg_i_ey / delta_t;

/* Output results to the screen */
/* ---------------------------- */
    if(isDebug)
        printf("acceleration = %6.10f\n", a_cg_bx_e);
    return 0;
}


int inertial_with_distance(	/*DEBUG*/
		int isDebug,
		/* INPUTS */
		float w1_d, // distance effectuée par la roue 1
		float w2_d, // distance effectuée par la roue 2
		/* INPUTS/OUTPUTS */
		float* p_cg_i_ex,  // X
		float* p_cg_i_ey,  // Y
		float* theta_e,    // heading
		/* OUTPUTS */
		float* v_cg_i_ex, // vitesse
		float* v_cg_i_ey  //vitesse
	)
{
	float w1_mes = w1_d / r_wheel;
    float w2_mes = w2_d / r_wheel;
	return inertial_with_angle(isDebug, w1_mes, w2_mes, p_cg_i_ex, p_cg_i_ey, theta_e, v_cg_i_ex, v_cg_i_ey);
}
