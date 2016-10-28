#ifndef _ROBOT_ARM_CTRL_BOARD_H_
#define _ROBOT_ARM_CTRL_BOARD_H_

#include <stdint.h>
#include <string.h>
#include  <ucos_ii.h>
#include "stm32f10x.h"

#define LED0_ON 		GPIOB->BSRR = GPIO_Pin_15
#define LED0_OFF 		GPIOB->BRR = GPIO_Pin_15
#define LED0_TOGGLE 	GPIOB->ODR ^= GPIO_Pin_15

#define LED1_ON 		GPIOB->BSRR = GPIO_Pin_14
#define LED1_OFF 		GPIOB->BRR = GPIO_Pin_14
#define LED1_TOGGLE 	GPIOB->ODR ^= GPIO_Pin_14

void TIM_PWM_Init(void);
void UART_COM_Init(void);
void LED_Init(void);
void BSP_Init(void);

#endif