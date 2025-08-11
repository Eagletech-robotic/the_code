#pragma once

typedef struct inertial_t {
    // constant du robot
    float r_wheel; //        = 0.065/2; // rayon des roue en m
    float r_axel;  //      = 0.33;    // écartement des roues en m
    float delta_t; //    = 0.006;    // pas de temps entre 2 lectures en s.
    float holes;   //  = 36*4*500;   // nombre d'impulsion pour faire un tour de roue
    int isDebug;
    /* INPUTS/OUTPUTS */
    float p_cg_i_ex; // X
    float p_cg_i_ey; // Y
    float theta_e;   // heading

} inertial_t;

// int inertial_run(	/*DEBUG*/
//		int isDebug,
//		/* INPUTS */
//		float w1_imp,
//		float w2_imp,
//		/* INPUTS/OUTPUTS */
//		float* p_cg_i_ex,
//		float* p_cg_i_ey,
//		float* theta_e,
//		/* OUTPUTS */
//		float* v_cg_i_ex,
//		float* v_cg_i_ey,
//		/* CONSTANT*/
//		float r_wheel,    // rayon des roue en m
//		float r_axel,    // écartement des roues en m
//		float delta_t,    // pas de temps entre 2 lectures en s.
//		float holes      // impulsion par tour de roue
//	);
void inertial_init(inertial_t *inertial, float r_wheel, float r_axel, float delta_t, float holes);
void inertial_step(inertial_t *inertial, float w1_imp, float w2_imp, float *v_cg_i_ex, float *v_cg_i_ey);
