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
extern UART_HandleTypeDef huart3;    // BT 9600 1N8 puis 115200 ?

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
	uint8_t pData[] = "AT+BAUD8"; //BT grande vitesse, il faudrait vérifier la vitesse, c'est lent je trouve. (uart3 est en autobaud)
	HAL_UART_Transmit(&huart3, pData, sizeof(pData), 0);

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
	printf("Falcon online !\n");

    // Relancer la réception en IT
    HAL_UART_Receive_IT(&huart3, &RxData1, 1);
  }
  else if (huart == &hlpuart1)
  {
    // Traitement de la donnée reçue sur UART2 : RxData2
	 um7_decode(RxData2);
    // Relancer la réception en IT
    HAL_UART_Receive_IT(&hlpuart1, &RxData2, 1);
  }
}


// --- Envoie du printf vers le BT et le debug

#define TX_BUFFER_SIZE 256

// Déclaration du buffer circulaire et des variables d'index
volatile uint8_t txBuffer[TX_BUFFER_SIZE];
volatile int txHead = 0;
volatile int txTail = 0;
volatile int txBusy = 0;  // Indique si une transmission est en cours

/**
  * @brief  Ajoute des données au buffer circulaire et démarre la transmission si nécessaire.
  * @param  pData : pointeur vers les données à transmettre.
  * @param  len : nombre d'octets à transmettre.
  * @retval None
  */
extern "C"  {
int _write(int file, char *pData, int len)
{
    for(int i = 0; i < len; i++)
    {
        // Calcul de l'index suivant dans le buffer circulaire
        int nextHead = (txHead + 1) % TX_BUFFER_SIZE;

        // Vérification de la saturation du buffer (on pourrait gérer cette erreur différemment)
        if(nextHead == txTail)
        {
            // Le buffer est plein, ici vous pouvez soit retourner, attendre, ou traiter l'erreur
            return 0; // ou gérer l'overflow
        }

        // Place l'octet dans le buffer et incrémente l'index "head"
        txBuffer[txHead] = pData[i];
        txHead = nextHead;
    }

    // Si aucune transmission n'est en cours, la démarrer
    if(txBusy == 0)
    {
        txBusy = 1;
        // Envoyer le premier octet depuis le buffer
        uint8_t byte = txBuffer[txTail];
        // Incrémenter le pointeur de lecture
        txTail = (txTail + 1) % TX_BUFFER_SIZE;
        // Lancer la transmission non bloquante en mode interruption
        if(HAL_UART_Transmit_IT(&huart3, &byte, 1) != HAL_OK)
        {
            // Traitement d'erreur si besoin
        }
    }
    return len;
}
}
/**
  * @brief  Callback appelée lors de la fin d'une transmission par interruption.
  * @param  huart: gestionnaire d'UART concerné.
  * @retval None
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart == &huart3)
    {
        // Si des données restent dans le buffer, poursuivre la transmission
        if(txTail != txHead)
        {
            uint8_t nextByte = txBuffer[txTail];
            txTail = (txTail + 1) % TX_BUFFER_SIZE;
            HAL_UART_Transmit_IT(huart, &nextByte, 1);
        }
        else
        {
            // Le buffer est vide, transmission terminée
            txBusy = 0;
        }
    }
}

void USART3_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart3);
}


