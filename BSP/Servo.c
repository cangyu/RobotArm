#include "Servo.h"

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
 *	6 -- invalid check sum exists
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

/**
 *	Details to describe a servo
 */
char* servo_name[ROBOTIC_ARM_NUM]={"Servo0", "Servo1", "Servo2", "Servo3", "Servo4", "Servo5", "Servo6", "Servo7", "Servo8", "Servo9"};
float cur_angle[ROBOTIC_ARM_NUM]={45.0, 45.0, 45.0, 45.0, 45.0, 45.0, 45.0, 45.0, 45.0, 45.0};
float target_angle[ROBOTIC_ARM_NUM]={45.0, 45.0, 45.0, 45.0, 45.0, 45.0, 45.0, 45.0, 45.0, 45.0};
float ang_min[ROBOTIC_ARM_NUM]={0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float ang_max[ROBOTIC_ARM_NUM]={90.0, 90.0, 90.0, 90.0, 90.0, 90.0, 90.0, 90.0, 180.0, 180.0};
int cur_pwm[ROBOTIC_ARM_NUM]={1520, 1520, 1520, 1520, 1520, 1520, 1520, 1520, 150, 150};
int target_pwm[ROBOTIC_ARM_NUM]={1520, 1520, 1520, 1520, 1520, 1520, 1520, 1520, 150, 150};
int pwm_min[ROBOTIC_ARM_NUM]={900, 900, 900, 900, 900, 900, 900, 900, 50, 50};
int pwm_max[ROBOTIC_ARM_NUM]={2100, 2100, 2100, 2100, 2100, 2100, 2100, 2100, 250, 250};
int gap[ROBOTIC_ARM_NUM]={20, 20, 20, 20, 20, 20, 20, 20, 20, 20};
int pwm_resolution[ROBOTIC_ARM_NUM]={1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
OS_EVENT *SemApply[ROBOTIC_ARM_NUM];
char* sem_name[ROBOTIC_ARM_NUM]={"ServoSem0", "ServoSem1", "ServoSem2", "ServoSem3", "ServoSem4", "ServoSem5", "ServoSem6", "ServoSem7", "ServoSem8", "ServoSem9"};

inline int min(int a, int b) { return a < b ? a : b; }

inline int max(int a, int b) { return a < b ? b : a; }

int angle2pwm(int i, float _ang)
{
	if(_ang<ang_min[i] || _ang>ang_max[i])
	{
		LED0_TOGGLE;
		return cur_pwm[i];
	}
	else
	  return (int)((_ang-ang_min[i])/(ang_max[i]-ang_min[i])*(pwm_max[i]-pwm_min[i])+pwm_min[i]);
}

float pwm2ang(int i, int _pwm)
{
	if(_pwm<pwm_min[i] || _pwm>pwm_max[i])
	{
		LED0_TOGGLE;
		return cur_angle[i];
	}
	else
		return ang_min[i]+(_pwm-pwm_min[i])*1.0/(pwm_max[i]-pwm_min[i])*(ang_max[i]-ang_min[i]);
}

void move(int i)
{
	uint8_t flag=0x00;
	
	OSSemPend(SemApply[i], 0, &flag);
	if(flag==OS_ERR_NONE)
	{
		target_pwm[i]=angle2pwm(i, target_angle[i]);
		while(cur_pwm[i]!=target_pwm[i])
		{
			advance_one(i);
			OSTimeDlyHMSM(0, 0, 0, gap[i]);
		}
		
		OSFlagPost(ServoModify,(OS_FLAGS)(1<<i),OS_FLAG_SET,&flag);
		assert(flag==OS_ERR_NONE);
	}
}

void advance_one(int i)
{
	if(target_pwm[i]>cur_pwm[i])
		cur_pwm[i]=min(target_pwm[i], cur_pwm[i]+pwm_resolution[i]);
	else
		cur_pwm[i]=max(target_pwm[i], cur_pwm[i]-pwm_resolution[i]);
	
	pos_update(i);
}

void pos_update(int index)
{
	cur_angle[index]=pwm2ang(index, cur_pwm[index]);
	switch(index)
	{
	case 0:
		TIM_SetCompare1(TIM3, cur_pwm[index]);
		break;
	case 1:
		TIM_SetCompare2(TIM3, cur_pwm[index]);
		break;
	case 2:
		TIM_SetCompare3(TIM3, cur_pwm[index]);
		break;
	case 3:
		TIM_SetCompare4(TIM3, cur_pwm[index]);
		break;
	case 4:
		TIM_SetCompare1(TIM4, cur_pwm[index]);
		break;
	case 5:
		TIM_SetCompare2(TIM4, cur_pwm[index]);
		break;
	case 6:
		TIM_SetCompare3(TIM4, cur_pwm[index]);
		break;
	case 7:
		TIM_SetCompare4(TIM4, cur_pwm[index]);
		break;
	case 8:
		TIM_SetCompare1(TIM2, cur_pwm[index]);
		break;
	case 9:
		TIM_SetCompare2(TIM2, cur_pwm[index]);
		break;
	default:
		break;
	}
}

void Servo_InitMove(void)
{
  	for(int i=0;i<ROBOTIC_ARM_NUM;i++)
	{
	  	if(!(SERVOS_USED&(1<<i)))
		  	continue;
		
		pos_update(i);
	}
}


/**
 *	Signal all used servos.
 */
void Servo_Run(void)
{	
  	uint8_t flag=0x00;
	
	while(recv_cmd());

	LED1_TOGGLE;
	for(int i=0;i<ROBOTIC_ARM_NUM;i++)
	{
		if(!(SERVOS_USED&(1<<i)))
			continue;
		
		target_angle[i]=angle[i];
		flag=OSSemPost(SemApply[i]);
		assert(flag==OS_ERR_NONE); 	
	}
}
