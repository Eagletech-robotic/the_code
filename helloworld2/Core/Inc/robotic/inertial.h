/*
 * inertial.h
 *
 *  Created on: Aug 30, 2024
 *      Author: nboulay
 */
#pragma once

int inertial(	/*DEBUG*/
		int isDebug,
		/* INPUTS */
		float w1_imp,
		float w2_imp,
		/* INPUTS/OUTPUTS */
		float* p_cg_i_ex,
		float* p_cg_i_ey,
		float* theta_e,
		/* OUTPUTS */
		float* v_cg_i_ex,
		float* v_cg_i_ey

	);
