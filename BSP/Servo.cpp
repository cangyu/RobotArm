#include "Servo.h"

struct Servo
{
  	int index;
  	float cur_angle, target_angle;
	float ang_min, ang_max;
	int cur_pwm, target_pwm;
	int pwm_min, pwm_max;
	int gap;
	int pwm_resolution;
	OS_EVENT *SemModify, *SemApply;
	
	void move(void)
	{
	  	uint8_t flag=0x00;
		
		OSSemPend(SemApply, 0, &flag);
		if(flag==OS_ERR_NONE)
		{
			while(cur_pwm!=target_pwm)
			{
				advance_one();
				OSTimeDlyHMSM(0, 0, 0, gap);
			}
			
			flag=OSSemPost(SemModify);
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
}


Servo servo[ROBOTIC_ARM_NUM]={
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

float angle[ROBOTIC_ARM_NUM]={45.0, 45.0, 45.0, 45.0, 45.0, 45.0, 45.0, 45.0, 90.0, 90.0};
float angle_target[ROBOTIC_ARM_NUM]={45.0, 45.0, 45.0, 45.0, 45.0, 45.0, 45.0, 45.0, 90.0, 90.0};
float angle_min[ROBOTIC_ARM_NUM] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
float angle_max[ROBOTIC_ARM_NUM] = { 90.0, 90.0, 90.0, 90.0, 90.0, 90.0, 90.0, 90.0, 180.0, 180.0 };
uint16_t pwm[ROBOTIC_ARM_NUM]={1520, 1520, 1520, 1520, 1520, 1520, 1520, 1520, 1000, 1000};
uint16_t pwm_target[ROBOTIC_ARM_NUM]={1520, 1520, 1520, 1520, 1520, 1520, 1520, 1520, 1000, 1000};
uint16_t pwm_min[ROBOTIC_ARM_NUM]={900, 900, 900, 900, 900, 900, 900, 900, 50, 50};
uint16_t pwm_max[ROBOTIC_ARM_NUM]={2100, 2100, 2100, 2100, 2100, 2100, 2100, 2100, 250, 250};
uint16_t gap[ROBOTIC_ARM_NUM]={5, 5, 5, 5, 5, 5, 5, 5, 5, 5};

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
	
	TIM_SetCompare1(TIM2, _pwm[8]);
	TIM_SetCompare2(TIM2, _pwm[9]);

	OSTimeDlyHMSM(0, 0, 0, STEP_GAP);
}

void move_servo(void)
{
	uint16_t prev_pwm[ROBOTIC_ARM_NUM], step[ROBOTIC_ARM_NUM], tmp_pwm[ROBOTIC_ARM_NUM];

	memcpy(prev_pwm, pwm, sizeof(prev_pwm));
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

void Servo_Run(void)
{	
	if(!recv_cmd())
	{
		LED1_TOGGLE;
		move_servo();
	}
}
