#include <stdint.h>
#include "stm32f10x.h"

#define ROBOTIC_ARM_NUM 10
#define MOVE_SLICE 6

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
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB , ENABLE);

	/* GPIO */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_Init(GPIOA,&GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_Init(GPIOB,&GPIO_InitStructure);

	/* TIM */
	TIM_TimeBaseStructure.TIM_Period = 2999;
	TIM_TimeBaseStructure.TIM_Prescaler = 71;//333.33Hz
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); 

	TIM_TimeBaseStructure.TIM_Period = 1999;
	TIM_TimeBaseStructure.TIM_Prescaler = 719;//50Hz
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

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

	TIM_OC1Init(TIM2, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);//TIM2_CH1
	TIM_OC2Init(TIM2, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);//TIM2_CH2

	TIM_ARRPreloadConfig(TIM3, ENABLE);
	TIM_CtrlPWMOutputs(TIM3,ENABLE);
	TIM_Cmd(TIM3, ENABLE);

	TIM_ARRPreloadConfig(TIM4, ENABLE);
	TIM_CtrlPWMOutputs(TIM4,ENABLE);
	TIM_Cmd(TIM4, ENABLE);

	TIM_ARRPreloadConfig(TIM2, ENABLE);
	TIM_CtrlPWMOutputs(TIM2, ENABLE);
	TIM_Cmd(TIM2, ENABLE);
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

float angle[ROBOTIC_ARM_NUM]={45.0, 45.0, 45.0, 45.0, 45.0, 45.0, 45.0, 45.0, 90.0, 90.0};
float angle_min[ROBOTIC_ARM_NUM] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
float angle_max[ROBOTIC_ARM_NUM] = { 90.0, 90.0, 90.0, 90.0, 90.0, 90.0, 90.0, 90.0, 180.0, 180.0 };

uint16_t pwm[ROBOTIC_ARM_NUM]={1520, 1520, 1520, 1520, 1520, 1520, 1520, 1520, 1000, 1000};
uint16_t pwm_min[ROBOTIC_ARM_NUM]={900, 900, 900, 900, 900, 900, 900, 900, 50, 50};
uint16_t pwm_max[ROBOTIC_ARM_NUM]={2100, 2100, 2100, 2100, 2100, 2100, 2100, 2100, 250, 250};

void angle_to_pwm(void)
{
	for(int i=0;i<ROBOTIC_ARM_NUM;i++)
	{
		if(angle[i]<angle_min[i] || angle[i]>angle_max[i])
		{
			LED0_TOGGLE;
			continue;
		}

		pwm[i]=(uint16_t)(pwm_min[i]+(pwm_max[i]-pwm_min[i])/angle_max[i]*angle[i]);
	}
}

//Polling UART1 to fetch a byte
uint8_t recv_byte(void)
{
	while(USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET);
	return USART_ReceiveData(USART1);
}

/**
  *	Data format: 0xCC (0x00~0x07 0x00~0xB4 0x00~0xBB)* 0xFF, interperated as: STX (SEQ ANG CHK)* END
  *	Return val:
  *	0 -- ok
  *	1 -- no incoming cmd
  *	2 -- invalid STX
  *	3 -- invalid index exists
  *	4 -- invalid angle exists
  *	5 -- invalid check sum exists
  */
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
						angle[index]=ang*1.0;
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

//Drive the servos physically
void set_servo(uint16_t _pwm[])
{
	TIM_SetCompare1(TIM3, _pwm[0]);
	TIM_SetCompare2(TIM3, _pwm[1]);
	TIM_SetCompare3(TIM3, _pwm[2]);
	TIM_SetCompare4(TIM3, _pwm[3]);

	TIM_SetCompare1(TIM4, _pwm[4]);
	TIM_SetCompare2(TIM4, _pwm[5]);
	TIM_SetCompare3(TIM4, _pwm[6]);
	TIM_SetCompare4(TIM4, _pwm[7]);

	delay_ms(10);
}

void move_servo(void)
{
	uint16_t prev_pwm[ROBOTIC_ARM_NUM], step[ROBOTIC_ARM_NUM], tmp_pwm[ROBOTIC_ARM_NUM];

	memcpy(prev_pwm, pwm, sizeof(cur_pwm));
	angle_to_pwm();

	for (int i = 0; i < ROBOTIC_ARM_NUM; i++)
		step[i] = (pwm[i] - prev_pwm[i]) / MOVE_SLICE;

	for (int i = 1; i < MOVE_SLICE; i++)
	{
		for (int j = 0; j < ROBOTIC_ARM_NUM; j++)
			tmp_pwm[j] = prev_pwm[j] + step[j] * i;

		set_servo(tmp_pwm);
	}
	set_servo(pwm);
}

int main(void)
{
	if (SysTick_Config(SystemCoreClock / 1000))
		while (1);//Capture error
	
	TIM_PWM_Init();
	UART_COM_Init();
	LED_Init();
	set_servo(pwm);//Init all servo to mid-point
	
	LED0_ON;
	while(1)
	{
		if(!recv_cmd())
		{
			LED1_TOGGLE;
			move_servo();
		}
	}
}
