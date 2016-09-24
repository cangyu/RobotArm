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
	TIM_TimeBaseStructure.TIM_Period = 1999;
	TIM_TimeBaseStructure.TIM_Prescaler = 719;//50Hz
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
void angle_to_pwm(void)
{
	for(int i=0;i<ROBOTIC_ARM_NUM;i++)
	{
		if(angle[i]<0.0 || angle[i]>180.0)
		{
			LED1_TOGGLE;
			continue;
		}

		pwm[i]=50+200.0/180*angle[i];
	}
}

//查询方式从UART1取一个byte
uint8_t recv_byte(void)
{
	while(USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET);
	return USART_ReceiveData(USART1);
}

//Data format: 0xCC (0x00~0x07 0x00~0xB4 0x00~0xBB)* 0xFF
//			   STX      SEQ       ANG       CHK      END
//Return val: 0 -- ok
//            1 -- no incoming cmd
//			  2 -- invalid STX
//            3 -- invalid index exists
//            4 -- invalid angle exists
//            5 -- invalid check sum exists
uint8_t recv_cmd(void)
{
	uint8_t data = 0xFF;
	uint8_t index = 0xFF;
	uint8_t ang=0x00;
	uint8_t chk_sum=0xFF;
	uint8_t ret=0x00;	
	
	if(USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET)
		return 0x01;
	
	data = USART_ReceiveData(USART1);
	if(data==0xCC)//起始字节
	{		
		data=recv_byte();
		while(data!=0xFF)//终止字节
		{
			index=data;
			if(index<ROBOTIC_ARM_NUM)//序号
			{
				ang=recv_byte();//角度
				if(ang<=180)
				{	
					chk_sum=recv_byte();//校验和
					if(chk_sum==index+ang)
						angle[index]=data*1.0;
					else
						ret = 0x05;
				}
				else
					ret=0x04;
			}
			else
				ret=0x03;
			
			while(USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET);
			data=USART_ReceiveData(USART1);
		}
		return ret;
	}
	else
		return 0x02;	
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
		if(!recv_cmd())
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
			
			LED1_TOGGLE;
		}
	}
}
