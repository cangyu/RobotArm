#include <stdint.h>
#include "stm32f10x.h"

#define ROBOTIC_ARM_NUM 8

GPIO_InitTypeDef GPIO_InitStructure;  
TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;  
TIM_OCInitTypeDef  TIM_OCInitStructure;  
USART_InitTypeDef USART_InitStructure;

__IO uint32_t TimingDelay;

void delay_ms(__IO uint32_t nTime)
{
	TimingDelay = nTime;
	while(TimingDelay != 0);	
}

void TIM_PWM_Init(void)  
{    
	/* Clock */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB , ENABLE);

	/* GPIO */ 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_Init(GPIOA,&GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_Init(GPIOB,&GPIO_InitStructure);

	/* TIM */
	TIM_TimeBaseStructure.TIM_Period = 9999;
	TIM_TimeBaseStructure.TIM_Prescaler = 143;//50Hz
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); 

	/* PWM Output */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

	TIM_OC1Init(TIM3, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);//TIM3_CH1
	TIM_OC2Init(TIM3, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);//TIM3_CH2
	TIM_OC3Init(TIM3, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);//TIM3_CH3
	TIM_OC4Init(TIM3, &TIM_OCInitStructure);
	TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);//TIM3_CH4

	TIM_OC1Init(TIM4, &TIM_OCInitStructure);   
	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);//TIM4_CH1
	TIM_OC2Init(TIM4, &TIM_OCInitStructure); 
	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);//TIM4_CH2
	TIM_OC3Init(TIM4, &TIM_OCInitStructure);   
	TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);//TIM4_CH3
	TIM_OC4Init(TIM4, &TIM_OCInitStructure); 
	TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);//TIM4_CH4

	TIM_ARRPreloadConfig(TIM3, ENABLE);
	TIM_CtrlPWMOutputs(TIM3,ENABLE);
	TIM_Cmd(TIM3, ENABLE);

	TIM_ARRPreloadConfig(TIM4, ENABLE);
	TIM_CtrlPWMOutputs(TIM4,ENABLE);//Master Output Enable, 官方例程上没有这个，存疑？
	TIM_Cmd(TIM4, ENABLE);
}

void UART_COM_Init(void)  
{    
	/* Clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	/* GPIO */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_Init(GPIOA,&GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_Init(GPIOA,&GPIO_InitStructure);

	/* USART1 */
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure);
	USART_Cmd(USART1, ENABLE);
}

void LED_Init(void)  
{    
	/* Clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	/* GPIO */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_Init(GPIOB,&GPIO_InitStructure);

	GPIO_ResetBits(GPIOB, GPIO_Pin_14 | GPIO_Pin_15);
}

#define LED0_ON 		GPIOB->BSRR = GPIO_Pin_15
#define LED0_OFF 		GPIOB->BRR = GPIO_Pin_15
#define LED0_TOGGLE 	GPIOB->ODR ^= GPIO_Pin_15

#define LED1_ON 		GPIOB->BSRR = GPIO_Pin_14
#define LED1_OFF 		GPIOB->BRR = GPIO_Pin_14
#define LED1_TOGGLE 	GPIOB->ODR ^= GPIO_Pin_14

float angle[ROBOTIC_ARM_NUM];
uint16_t pwm[ROBOTIC_ARM_NUM]={0};

//Servo PWM Standard Period: 20ms, (pwm signal 2000 total)
//0-0.5ms-50
//45-1ms-100
//90-1.5ms-150
//135-2ms-200
//180-2.5ms-250
void angle_to_pwm()
{
	for(int i=0;i<ROBOTIC_ARM_NUM;i++)
	{
		if(angle[i]<0.0 || angle[i]>180.0)
		{
			LED1_TOGGLE;
			//angle[i]=90.0;
			continue;
		}

		pwm[i]=50+200.0/180*angle[i];
	}
}

int main(void)
{
	if (SysTick_Config(SystemCoreClock / 1000))
		while (1);//Capture error 
	
	TIM_PWM_Init();
	UART_COM_Init();
	LED_Init();
	
	LED0_ON;

	for(int i=0;i<ROBOTIC_ARM_NUM;i++)
		angle[i]=0.0;

	while(1)
	{
		angle_to_pwm();

		TIM_SetCompare1(TIM3,pwm[0]);
		TIM_SetCompare2(TIM3,pwm[1]);
		TIM_SetCompare3(TIM3,pwm[2]);
		TIM_SetCompare4(TIM3,pwm[3]);

		TIM_SetCompare1(TIM4,pwm[4]);
		TIM_SetCompare2(TIM4,pwm[5]);
		TIM_SetCompare3(TIM4,pwm[6]);
		TIM_SetCompare4(TIM4,pwm[7]);

		delay_ms(500);
		LED0_TOGGLE;
	}
}
