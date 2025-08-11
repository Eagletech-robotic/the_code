#include "iot01A/pwm.h"
#include "main.h"
#include <stdio.h>

// Gestion des pwm pour le moteur
// Pour une fréquence de 50 khz, cela donne un compteur qui va jusqu'à 1600. (80 Mhz / 50 khz) sans prescaler
// tout cela est programmer dans le ioc
// PWM1 : TIM2_CH1
// PWM2 : TIM2_CH3

extern TIM_HandleTypeDef htim2;  // moteur
extern TIM_HandleTypeDef htim16; // PWM16
extern TIM_HandleTypeDef htim17; // PWM17

uint32_t ratio_to_step(float ratio, uint32_t period) {
    uint32_t step = ratio * (1.0f * period);

    const float minimum_to_move = 55; // à 55 cela tourne à peine à vide

    step = (period - minimum_to_move) * ratio + minimum_to_move;

    return step;
}

void PWMset_1(TIM_HandleTypeDef *htim, float ratio1, float ratio2) {
    uint32_t period = htim->Init.Period;

    htim->Instance->CCR1 = ratio_to_step(ratio1, period);

    htim->Instance->CCR3 = ratio_to_step(ratio2, period);
    // printf("PWM %li %li\r\n", htim->Instance->CCR1, htim->Instance->CCR3);
}

// 0.0<=ratio<=1.0
// cela permet de changer la fréquence et le compteur sans changer les utilisations

void PWMset(float ratio1, float ratio2) { PWMset_1(&htim2, ratio1, ratio2); }

void PWMstart() {
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim17, TIM_CHANNEL_1);
}

void PWMSet_lp(TIM_HandleTypeDef *htim, float ratio) {
    uint32_t period = htim->Init.Period;

    htim->Instance->CCR1 = ratio_to_step(ratio, period);
}

void PWMSet_16(float ratio) { PWMSet_lp(&htim16, ratio); }

void PWMSet_17(float ratio) { PWMSet_lp(&htim17, ratio); }

uint32_t ratio(float ratio, uint32_t period) {
    uint32_t step = ratio * (1.0f * period);

    return step;
}
