#include "Servo.h"

inline int min(int a, int b) { return a < b ? a : b; }
inline int max(int a, int b) { return a < b ? b : a; }

extern OS_FLAG_GRP  *ServoModify;
float angle[ROBOTIC_ARM_NUM]={45.0, 45.0, 45.0, 45.0, 45.0, 45.0, 45.0, 45.0, 90.0, 90.0};

/**
  *	Polling UART1 to fetch a byte
  */
uint8_t recv_byte(void)
{
	while(USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET);
	return USART_ReceiveData(USART1);
}

/**
  *	Data format: 0xCC (0x00~0x07 0x00~0xB4 0x00~0x09 0x00~0xBB)* 0xFF, interperated as: STX (SEQ ANG1 ANG2 CHK)* END
  *	Return val:
  *	0 -- ok
  *	1 -- no incoming cmd
  *	2 -- invalid STX
  *	3 -- invalid index exists
  *	4 -- invalid angle exists
  *	5 -- invalid fraction part of angle
  * 6 -- invalid check sum exists
  */
uint8_t recv_cmd(void)
{
	uint8_t data = 0xFF;
	uint8_t index = 0xFF;
	uint8_t ang=0x00;
	uint8_t ang_frac=0x00;
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
				  	ang_frac=recv_byte();//小数部分
					if(ang_frac<10)
					{
						chk_sum=recv_byte();//校验和
						if(chk_sum==index+ang+ang_frac)
							angle[index]=ang*1.0 +ang_frac*0.1;
						else
							ret = 0x06;
					}
					else
					  ret=0x05;
				}
				else
					ret=0x04;
			}
			else
				ret=0x03;
			
			data=recv_byte();
		}
		return ret;
	}
	else
		return 0x02;	
}

struct Servo
{
  	int index;
  	float cur_angle, target_angle;
	float ang_min, ang_max;
	int cur_pwm, target_pwm;
	int pwm_min, pwm_max;
	int gap;
	int pwm_resolution;
	OS_EVENT *SemApply;
	
	int angle2pwm(float _ang)
	{
		if(_ang<ang_min || _ang>ang_max)
		{
			LED0_TOGGLE;
			return cur_pwm;
		}
		else
		  return (int)((_ang-ang_min)/(ang_max-ang_min)*(pwm_max-pwm_min)+pwm_min);
	}
	
	float pwm2ang(int _pwm)
	{
	  	if(_pwm<pwm_min || _pwm>pwm_max)
		{
		  	LED0_TOGGLE;
			return cur_angle;
		}
		else
		  	return ang_min+(_pwm-pwm_min)*1.0/(pwm_max-pwm_min)*(ang_max-ang_min);
	}
	
	void move(void)
	{
	  	uint8_t flag=0x00;
		
		OSSemPend(SemApply, 0, &flag);
		if(flag==OS_ERR_NONE)
		{
		  	target_pwm=angle2pwm(target_angle);
			while(cur_pwm!=target_pwm)
			{
				advance_one();
				OSTimeDlyHMSM(0, 0, 0, gap);
			}
			
			OSFlagPost(ServoModify,(OS_FLAGS)(1<<index),OS_FLAG_SET,&flag);
			assert(flag==OS_ERR_NONE);
		}
	}
	
	void advance_one(void)
	{
	  	if(target_pwm>cur_pwm)
		  	cur_pwm=min(target_pwm, cur_pwm+pwm_resolution);
		else
		  	cur_pwm=max(target_pwm, cur_pwm-pwm_resolution);
		
		pos_update();
	}
	
	void pos_update(void)
	{
		cur_angle=pwm2ang(cur_pwm);
		switch(index)
		{
		case 0:
			TIM_SetCompare1(TIM3, cur_pwm);
			break;
		case 1:
			TIM_SetCompare2(TIM3, cur_pwm);
			break;
		case 2:
			TIM_SetCompare3(TIM3, cur_pwm);
			break;
		case 3:
			TIM_SetCompare4(TIM3, cur_pwm);
			break;
		case 4:
			TIM_SetCompare1(TIM4, cur_pwm);
			break;
		case 5:
			TIM_SetCompare2(TIM4, cur_pwm);
			break;
		case 6:
			TIM_SetCompare3(TIM4, cur_pwm);
			break;
		case 7:
			TIM_SetCompare4(TIM4, cur_pwm);
			break;
		case 8:
			TIM_SetCompare1(TIM2, cur_pwm);
			break;
		case 9:
			TIM_SetCompare2(TIM2, cur_pwm);
			break;
		default:
		  	break;
		}
	}
};

Servo servo[ROBOTIC_ARM_NUM];
/*
{
  {0, 45.0, 45.0, 0.0, 90.0, 1520, 1520, 900, 2100, 20, 1,  NULL, NULL},
  {1, 45.0, 45.0, 0.0, 90.0, 1520, 1520, 900, 2100, 20, 1,  NULL, NULL},
  {2, 45.0, 45.0, 0.0, 90.0, 1520, 1520, 900, 2100, 20, 1,  NULL, NULL},
  {3, 45.0, 45.0, 0.0, 90.0, 1520, 1520, 900, 2100, 20, 1,  NULL, NULL},
  {4, 45.0, 45.0, 0.0, 90.0, 1520, 1520, 900, 2100, 20, 1,  NULL, NULL},
  {5, 45.0, 45.0, 0.0, 90.0, 1520, 1520, 900, 2100, 20, 1,  NULL, NULL},
  {6, 45.0, 45.0, 0.0, 90.0, 1520, 1520, 900, 2100, 20, 1,  NULL, NULL},
  {7, 45.0, 45.0, 0.0, 90.0, 1520, 1520, 900, 2100, 20, 1,  NULL, NULL},
  {8, 45.0, 45.0, 0.0, 180.0, 150, 150, 50, 250, 20, 1,  NULL, NULL},
  {9, 45.0, 45.0, 0.0, 180.0, 150, 150, 50, 250, 20, 1,  NULL, NULL}  
};
*/

void Servo_Run(void)
{	
  	uint8_t flag=0x00;
	
	if(!recv_cmd())
	{
		LED1_TOGGLE;
		for(int i=0;i<ROBOTIC_ARM_NUM;i++)
		{
		  	servo[i].target_angle=angle[i];
			flag=OSSemPost(servo[i].SemApply);
			assert(flag==OS_ERR_NONE);
		}
	}
}
