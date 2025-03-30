/*
 * async_uart.c
 *
 *  Created on: Mar 23, 2025
 *      Author: nboulay
 */
#include "main.h"
#include "robotic/um7.h"
#include <stdio.h>

// lecture de um7 + configuration du port série en interruption
// lecture du BT

extern UART_HandleTypeDef hlpuart1;  // BT
extern UART_HandleTypeDef huart3;    // UM7

uint8_t RxData1;
uint8_t RxData2;

void async_uart_init() {
	HAL_UART_Init(&hlpuart1);
	HAL_UART_Init(&huart3);
	HAL_UART_Receive_IT(&hlpuart1, &RxData1, 1);
	HAL_UART_Receive_IT(&huart3, &RxData2, 1);
}

/**
  * @brief  Callback appelé lorsque la réception UART est complétée (IT).
  * @param  huart : pointeur sur la structure UART_HandleTypeDef
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Vérifie quelle UART a reçu un octet */
  if (huart == &hlpuart1)
  {
    // Traitement de la donnée reçue sur UART1 : RxData1
	  printf("lpuart1\n");
    // Relancer la réception en IT
    HAL_UART_Receive_IT(&hlpuart1, &RxData1, 1);
  }
  else if (huart == &huart3)
  {
    // Traitement de la donnée reçue sur UART2 : RxData2
	  //printf("uart3\n");
	  printf("%c\n", RxData2);
	 // um7_decode(RxData2);
    // Relancer la réception en IT
    HAL_UART_Receive_IT(&huart3, &RxData2, 1);
  }
}
