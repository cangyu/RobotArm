#include "Servo.h"

extern OS_FLAG_GRP  *ServoModify;

/**
 *	Details to control the servos
 */
//Semaphores
OS_EVENT *SemApply[ROBOTIC_ARM_NUM];
//char* servo_name[ROBOTIC_ARM_NUM]={"Servo0", "Servo1", "Servo2", "Servo3", "Servo4", "Servo5", "Servo6", "Servo7", "Servo8", "Servo9"};
//char* sem_name[ROBOTIC_ARM_NUM]={"ServoSem0", "ServoSem1", "ServoSem2", "ServoSem3", "ServoSem4", "ServoSem5", "ServoSem6", "ServoSem7", "ServoSem8", "ServoSem9"};

//Receive buffer
float angle[ROBOTIC_ARM_NUM]={45.0, 45.0, 45.0, 45.0, 45.0, 45.0, 45.0, 45.0, 90.0, 90.0};

//Degree settings
float cur_angle[ROBOTIC_ARM_NUM]={45.0, 45.0, 45.0, 45.0, 45.0, 45.0, 45.0, 45.0, 90.0, 45.0};
float target_angle[ROBOTIC_ARM_NUM]={45.0, 45.0, 45.0, 45.0, 45.0, 45.0, 45.0, 45.0, 90.0, 45.0};
const float ang_min[ROBOTIC_ARM_NUM]={0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
const float ang_max[ROBOTIC_ARM_NUM]={90.0, 90.0, 90.0, 90.0, 90.0, 90.0, 90.0, 90.0, 90.0, 90.0};

//PWM settings
uint16_t cur_pwm[ROBOTIC_ARM_NUM]={1520, 1520, 1520, 1520, 1520, 1520, 1520, 1520, 250, 150};
uint16_t target_pwm[ROBOTIC_ARM_NUM]={1520, 1520, 1520, 1520, 1520, 1520, 1520, 1520, 250, 150};
const uint16_t pwm_min[ROBOTIC_ARM_NUM]={900, 900, 900, 900, 800, 800, 900, 900, 50, 50};
const uint16_t pwm_max[ROBOTIC_ARM_NUM]={2100, 2100, 2100, 2100, 2200, 2200, 2100, 2100, 250, 250};
const uint16_t pwm_period[ROBOTIC_ARM_NUM]={3, 3, 3, 3, 3, 3, 3, 3, 3, 20};//ms

//Speed settings
S_Curve s[ROBOTIC_ARM_NUM];
uint16_t next_pwm[ROBOTIC_ARM_NUM];
const uint16_t pwm_resolution[ROBOTIC_ARM_NUM]={16, 20, 20, 16, 40, 40, 40, 16, 16, 16};// x0.1 -> degree
//const float intrinsic_speed[ROBOTIC_ARM_NUM]={2.83, 2.83, 2.83, 2.83, 1.0, 1.0, 2.83, 2.83, 16, 16};// ms/degree
const float max_speed[ROBOTIC_ARM_NUM]={45.0, 45.0, 45.0, 45.0, 45.0, 45.0, 45.0, 45.0, 45.0, 45.0};// degree/s
float time_dist[ROBOTIC_ARM_NUM][SEGMENT_NUM]={
  {0.08, 0.14, 0.08, 0.7, 0.08, 0.14, 0.08},
  {0.08, 0.14, 0.08, 0.7, 0.08, 0.14, 0.08},
  {0.08, 0.14, 0.08, 0.7, 0.08, 0.14, 0.08},
  {0.08, 0.14, 0.08, 0.7, 0.08, 0.14, 0.08},
  {0.08, 0.14, 0.08, 0.7, 0.08, 0.14, 0.08},
  {0.08, 0.14, 0.08, 0.7, 0.08, 0.14, 0.08},
  {0.08, 0.14, 0.08, 0.7, 0.08, 0.14, 0.08},
  {0.08, 0.14, 0.08, 0.7, 0.08, 0.14, 0.08},
  {0.08, 0.14, 0.08, 0.7, 0.08, 0.14, 0.08},
  {0.08, 0.14, 0.08, 0.7, 0.08, 0.14, 0.08},
};

/**
 *	Helper function.
 */
inline int min(int a, int b) { return a < b ? a : b; }
inline int max(int a, int b) { return a < b ? b : a; }

/**
 *	Polling UART1 to fetch a byte.
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
 *	Signal all used servos.
 */
void Servo_Run(void)
{		
	while(recv_cmd());

	LED1_TOGGLE;
	for(int i=0;i<ROBOTIC_ARM_NUM;i++)
	{
		if(!(SERVOS_USED&(1<<i)))
			continue;
		
		target_angle[i]=angle[i];
		uint8_t flag=OSSemPost(SemApply[i]);
		assert(flag==OS_ERR_NONE);
	}
}

/**
 *	Convert angle value to relevant PWM count.
 */
uint16_t angle2pwm(int i, float _ang)
{
	if(_ang<ang_min[i] || _ang>ang_max[i])
	{
		LED0_TOGGLE;
		return cur_pwm[i];
	}
	else
	  return (uint16_t)((_ang-ang_min[i])/(ang_max[i]-ang_min[i])*(pwm_max[i]-pwm_min[i]))+pwm_min[i];
}

/**
 *	Convert PWM count to relevant angle value.
 */
float pwm2ang(int i, uint16_t _pwm)
{
	if(_pwm<pwm_min[i] || _pwm>pwm_max[i])
	{
		LED0_TOGGLE;
		return cur_angle[i];
	}
	else
		return ang_min[i]+(_pwm-pwm_min[i])*1.0/(pwm_max[i]-pwm_min[i])*(ang_max[i]-ang_min[i]);
}

/**
 *	High level motion control with synchronization primitives. 
 */
void move(int i)
{
	uint8_t flag=0x00;
	
	OSSemPend(SemApply[i], 0, &flag);
	assert(flag==OS_ERR_NONE);
	
	target_pwm[i]=angle2pwm(i, target_angle[i]);
	optimized_move(i);
	OSFlagPost(ServoModify,(OS_FLAGS)(1<<i),OS_FLAG_SET,&flag);
	assert(flag==OS_ERR_NONE);
}

/**
 *	Move the servo according to given S-Curve. 
 */
void optimized_move(int index)
{
  	const int delta_pwm=target_pwm[index]-cur_pwm[index];//PWM count
	const float delta_t=fabs(target_angle[index]-cur_angle[index])/max_speed[index]*1000.0;//ms
	
	if(abs(delta_pwm)<pwm_resolution[index] || delta_t<pwm_period[index])
	  	return;
	
	const int pwm_base=cur_pwm[index];
	
	
	s_curve_create(&s[index], delta_pwm, delta_t, time_dist[index]);
	
	const int N=(int)(delta_t/pwm_period[index]);
	for(int i=1;i<=N;i++)
		unguarded_move_one(index,(int)(get_position(&s[index], i*pwm_period[index]))+pwm_base);
	
	unguarded_move_one(index, target_pwm[index]);//final move
}

/**
 *	Unguarded move one step to next PWM position. 
 */
inline void unguarded_move_one(int index, uint16_t _pwm_pos)
{
  	next_pwm[index]=_pwm_pos;
	advance_to_next(index);
	OSTimeDlyHMSM(0, 0, 0, pwm_period[index]);
}

/**
 *	Update current PWM and position with constraints.
 */
void advance_to_next(int i)
{
  	if(next_pwm[i]<pwm_min[i])
	  	cur_pwm[i]=pwm_min[i];
	else if(next_pwm[i]>pwm_max[i])
	  	cur_pwm[i]=pwm_max[i];
	else if(next_pwm[i]==cur_pwm[i])
	  	return;
	else
	  	cur_pwm[i]=next_pwm[i];
	
	pos_update(i);
}

/**
 *	Move the arm physically.
 */
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

/**
 *	Move the arm to initial position when powered up.
 */
void Servo_InitMove(void)
{
  	for(int i=0;i<ROBOTIC_ARM_NUM;i++)
	{
	  	if(!(SERVOS_USED&(1<<i)))
		  	continue;
		
		pos_update(i);
	}
}
