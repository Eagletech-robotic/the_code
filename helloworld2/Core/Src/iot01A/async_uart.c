/*
 * async_uart.c
 *
 *  Created on: Mar 23, 2025
 *      Author: nboulay
 */
#include "main.h"
#include "robotic/um7.h"
#include <stdio.h>
#include <string.h>
// lecture de um7 + configuration du port série en interruption
// lecture du BT

extern UART_HandleTypeDef hlpuart1;  // UM7 min 19400BAUD 115200 1N8
extern UART_HandleTypeDef huart3;    // BT 9600 1N8 puis 200k ?

uint8_t RxData1;
uint8_t RxData2;

void async_uart_init() {
	HAL_UART_Init(&hlpuart1);
	HAL_UART_Init(&huart3);
	//um7_factory_reset(&hlpuart1) ;
	//um7_zero_gyros(&hlpuart1) ; // ne met que des zéro
	//HAL_Delay(3000);
	//um7_calibrate_accelerometers(&hlpuart1) ; // que zero ensuite
	//HAL_Delay(2000);
	//um7_set_mag_reference(&hlpuart1); //que des zéros
	//HAL_Delay(2000);
	//um7_set_home_position(&hlpuart1); // ne fait rien sur euler et acc
	//um7_set_misc_settings(&hlpuart1, 0, 1, 1, 1);

	um7_reset_kalman_filter(&hlpuart1); // ne semble rien faire
	um7_set_position_rate(&hlpuart1, 250);
	//um7_set_pose_rate(&huart3, 10);

	HAL_UART_Receive_IT(&huart3, &RxData1, 1);
	HAL_UART_Receive_IT(&hlpuart1, &RxData2, 1);
}

/**
  * @brief  Callback appelé lorsque la réception UART est complétée (IT).
  * @param  huart : pointeur sur la structure UART_HandleTypeDef
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Vérifie quelle UART a reçu un octet */
  if (huart == &huart3)
  {
    // Traitement de la donnée reçue sur UART1 : RxData1
	  printf("lpuart1\n");
    // Relancer la réception en IT
    HAL_UART_Receive_IT(&huart3, &RxData1, 1);
  }
  else if (huart == &hlpuart1)
  {
    // Traitement de la donnée reçue sur UART2 : RxData2
	  //printf("uart3\n");
	 um7_decode(RxData2);
    // Relancer la réception en IT
    HAL_UART_Receive_IT(&hlpuart1, &RxData2, 1);
  }
}


// --- Envoie du printf vers le BT et le debug

uint8_t buff[100];
static int volatile uart_busy = 0; // sorte de mutex pour l'utilisation du printf sur it
#ifdef __cplusplus
extern "C"  {
#endif
	int _write(int file, char *ptr, int len) {
		if(len > sizeof(buff)) { // évite le débordement de buffer. Si le buffer est trop grand, de toute façon il n'y a pas le temps de transmission
			len = sizeof(buff);
		}
		memcpy(buff,ptr,len);
		HAL_UART_Transmit_IT(&huart3, buff, len); // interface BT
		return len;
	}
#ifdef __cplusplus
}
#endif

