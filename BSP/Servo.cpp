#include "Servo.h"

/**
 *	7段式S曲线，用于舵机运动的平滑加减速控制，减少冲击。
 */
struct S_Curve
{
	double S, T;
	double a, b;
	double ratio[SEGMENT_NUM];
	double t[SEGMENT_NUM], p[SEGMENT_NUM];
	double aa_mat[SEGMENT_NUM];
	double a_mat[SEGMENT_NUM][2];
	double vp[SEGMENT_NUM], v_mat[SEGMENT_NUM][3];
	double sp[SEGMENT_NUM], s_mat[SEGMENT_NUM][4];

	void Time_Init(void)
	{
		for (int i = 0; i < SEGMENT_NUM; i++)
			t[i] = ratio[i] * T;

		p[0] = t[0];
		for (int i = 1; i < SEGMENT_NUM; i++)
			p[i] = p[i - 1] + t[i];
	}

	double calc_a(void)
	{
		double dividend = S * 6 * (t[4] + 2 * t[5] + t[6]);
		double divisor = -2 * p[5] * p[5] * t[0] + 4 * p[5] * p[6] * t[0] - 2 * p[6] * p[6] * t[0] - 4 * p[5] * p[5] * t[1] + 8 * p[5] * p[6] * t[1] - 4 * p[6] * p[6] * t[1] - 2 * p[5] * p[5] * t[2] + \
			4 * p[5] * p[6] * t[2] - 2 * p[6] * p[6] * t[2] - p[3] * p[3] * (t[0] + 2 * t[1] + t[2]) + 2 * p[3] * p[4] * (t[0] + 2 * t[1] + t[2]) - \
			p[4] * p[4] * (t[0] + 2 * t[1] + t[2]) + p[0] * p[0] * t[4] + 2 * p[1] * p[1] * t[4] - 4 * p[1] * p[2] * t[4] + 2 * p[2] * p[2] * t[4] + 3 * p[1] * t[1] * t[4] + \
			3 * t[0] * t[2] * t[4] + 6 * t[1] * t[2] * t[4] + 3 * t[0] * t[3] * t[4] + 6 * t[1] * t[3] * t[4] + 3 * t[2] * t[3] * t[4] + 3 * t[0] * t[4] * t[4] + 6 * t[1] * t[4] * t[4] + \
			3 * t[2] * t[4] * t[4] + 2 * p[0] * p[0] * t[5] + 4 * p[1] * p[1] * t[5] - 8 * p[1] * p[2] * t[5] + 4 * p[2] * p[2] * t[5] - 3 * p[5] * t[0] * t[5] + 6 * p[1] * t[1] * t[5] - \
			6 * p[5] * t[1] * t[5] - 3 * p[5] * t[2] * t[5] + 6 * t[0] * t[2] * t[5] + 12 * t[1] * t[2] * t[5] + 3 * p[4] * (t[0] + 2 * t[1] + t[2])*t[5] + \
			6 * t[0] * t[3] * t[5] + 12 * t[1] * t[3] * t[5] + 6 * t[2] * t[3] * t[5] + 6 * t[0] * t[4] * t[5] + 12 * t[1] * t[4] * t[5] + 6 * t[2] * t[4] * t[5] + \
			6 * t[0] * t[5] * t[5] + 12 * t[1] * t[5] * t[5] + 6 * t[2] * t[5] * t[5] + p[0] * p[0] * t[6] + 2 * p[1] * p[1] * t[6] - 4 * p[1] * p[2] * t[6] + 2 * p[2] * p[2] * t[6] + 3 * p[1] * t[1] * t[6] + \
			3 * t[0] * t[2] * t[6] + 6 * t[1] * t[2] * t[6] + 3 * t[0] * t[3] * t[6] + 6 * t[1] * t[3] * t[6] + 3 * t[2] * t[3] * t[6] + 3 * t[0] * t[4] * t[6] + 6 * t[1] * t[4] * t[6] + \
			3 * t[2] * t[4] * t[6] + 3 * t[0] * t[5] * t[6] + 6 * t[1] * t[5] * t[6] + 3 * t[2] * t[5] * t[6] + 3 * t[0] * t[6] * t[6] + 6 * t[1] * t[6] * t[6] + 3 * t[2] * t[6] * t[6];//Thanks to mathematica...

		return dividend / divisor;
	}

	double calc_b(void)
	{
		return -a*(t[0] + 2 * t[1] + t[2]) / (t[4] + 2 * t[5] + t[6]);
	}

	void CalcBasicVars(void)
	{
		a = calc_a();
		b = calc_b();
	}

	void calc_aa(void)
	{
		aa_mat[0] = a / t[0];
		aa_mat[1] = 0;
		aa_mat[2] = -a / t[2];
		aa_mat[3] = 0;
		aa_mat[4] = b / t[4];
		aa_mat[5] = 0;
		aa_mat[6] = -b / t[6];
	}

	void calc_acceleration_matrix(void)
	{
		a_mat[0][0] = 0; a_mat[0][1] = a / t[0];
		a_mat[1][0] = a; a_mat[1][1] = 0;
		a_mat[2][0] = a / t[2] * p[2]; a_mat[2][1] = -a / t[2];
		a_mat[3][0] = 0; a_mat[3][1] = 0;
		a_mat[4][0] = -b*p[3] / t[4]; a_mat[4][1] = b / t[4];
		a_mat[5][0] = b; a_mat[5][1] = 0;
		a_mat[6][0] = b*p[6] / t[6]; a_mat[6][1] = -b / t[6];
	}

	void CalcAccelerationVars(void)
	{
		calc_aa();
		calc_acceleration_matrix();
	}

	void calc_vp(void)
	{
		vp[0] = a*0.5*t[0];
		vp[1] = vp[0] + a*t[1];
		vp[2] = vp[1] + a*0.5*t[2];
		vp[3] = vp[2];
		vp[4] = vp[3] + b*0.5*t[4];
		vp[5] = vp[4] + b*t[5];
		vp[6] = 0.0;
	}

	void calc_velocity_matrix(void)
	{
		v_mat[0][0] = 0; v_mat[0][1] = 0; v_mat[0][2] = 0.5*a / t[0];
		v_mat[1][0] = -0.5*a*p[0]; v_mat[1][1] = a; v_mat[1][2] = 0;
		v_mat[2][0] = vp[1] - a*p[1] / t[2] * (p[2] - 0.5*p[1]); v_mat[2][1] = a*p[2] / t[2]; v_mat[2][2] = -0.5*a / t[2];
		v_mat[3][0] = vp[2]; v_mat[3][1] = 0; v_mat[3][2] = 0;
		v_mat[4][0] = vp[3] + b*p[3] * p[3] / 2 / t[4]; v_mat[4][1] = -b*p[3] / t[4]; v_mat[4][2] = 0.5*b / t[4];
		v_mat[5][0] = vp[4] - b*p[4]; v_mat[5][1] = b; v_mat[5][2] = 0;
		v_mat[6][0] = vp[5] - b*(p[6] * p[5] - 0.5*p[5] * p[5]) / t[6]; v_mat[6][1] = b*p[6] / t[6]; v_mat[6][2] = -0.5*b / t[6];
	}

	void CalcVelocityVars(void)
	{
		calc_vp();
		calc_velocity_matrix();
	}

	void calc_sp(void)
	{
		sp[0] = a*pow(p[0], 2) / 6;
		for (int i = 1; i < SEGMENT_NUM; i++)
		{
			sp[i] = sp[i - 1];
			sp[i] += v_mat[i][0] * (p[i] - p[i - 1]);
			sp[i] += v_mat[i][1] * (pow(p[i], 2) - pow(p[i - 1], 2)) / 2;
			sp[i] += v_mat[i][2] * (pow(p[i], 3) - pow(p[i - 1], 3)) / 3;
		}
	}

	void calc_position_matrix(void)
	{
		s_mat[0][0] = 0; s_mat[0][1] = 0; s_mat[0][2] = 0; s_mat[0][3] = a / t[0] / 6;
		for (int i = 1; i < SEGMENT_NUM; i++)
		{
			s_mat[i][0] = sp[i - 1] - (v_mat[i][0] * p[i - 1] + v_mat[i][1] * pow(p[i - 1], 2) / 2 + v_mat[i][2] * pow(p[i - 1], 3) / 3);
			s_mat[i][1] = v_mat[i][0];
			s_mat[i][2] = v_mat[i][1] / 2;
			s_mat[i][3] = v_mat[i][2] / 3;
		}
	}

	void CalcPositionVars(void)
	{
		calc_sp();
		calc_position_matrix();
	}

	int calc_gap_index(double _t)
	{
		int i = 0;
		while (i < SEGMENT_NUM && _t > p[i])
			++i;

		assert(i < SEGMENT_NUM);

		return i;
	}

	void S_Curve_Create(double totalLen, double totalTime, double r[])
	{
		S=totalLen;
		T=totalTime;
		
		for(int i=0;i<SEGMENT_NUM;i++)
			ratio[i]=r[i];

		Time_Init();
		CalcBasicVars();
	#if(ACCELERATION_UTILITY)
		CalcAccelerationVars();
	#endif
		CalcVelocityVars();
		CalcPositionVars();
	}

	double get_aa(double _t)
	{
		int i = calc_gap_index(_t);
		return aa_mat[i];
	}

	double get_acceleration(double _t)
	{
		int i = calc_gap_index(_t);
		return a_mat[i][0] + _t*a_mat[i][1];
	}

	double get_velocity(double _t)
	{
		int i = calc_gap_index(_t);
		return v_mat[i][0] + _t*(v_mat[i][1] + _t*v_mat[i][2]);
	}

	double get_position(double _t)
	{
		int i = calc_gap_index(_t);
		return s_mat[i][0] + _t*(s_mat[i][1] + _t*(s_mat[i][2] + _t *s_mat[i][3]));//Horner's rule
	}
}s[ROBOTIC_ARM_NUM];

extern OS_FLAG_GRP  *ServoModify;

/**
 *	Details to control the servos
 */
//Semaphores
char* servo_name[ROBOTIC_ARM_NUM]={"Servo0", "Servo1", "Servo2", "Servo3", "Servo4", "Servo5", "Servo6", "Servo7", "Servo8", "Servo9"};
OS_EVENT *SemApply[ROBOTIC_ARM_NUM];
char* sem_name[ROBOTIC_ARM_NUM]={"ServoSem0", "ServoSem1", "ServoSem2", "ServoSem3", "ServoSem4", "ServoSem5", "ServoSem6", "ServoSem7", "ServoSem8", "ServoSem9"};

//Receive buffer
float angle[ROBOTIC_ARM_NUM]={45.0, 45.0, 45.0, 45.0, 45.0, 45.0, 45.0, 45.0, 90.0, 90.0};

//Degree settings
float cur_angle[ROBOTIC_ARM_NUM]={45.0, 45.0, 45.0, 45.0, 45.0, 45.0, 45.0, 45.0, 90.0, 45.0};
float target_angle[ROBOTIC_ARM_NUM]={45.0, 45.0, 45.0, 45.0, 45.0, 45.0, 45.0, 45.0, 90.0, 45.0};
const float ang_min[ROBOTIC_ARM_NUM]={0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
const float ang_max[ROBOTIC_ARM_NUM]={90.0, 90.0, 90.0, 90.0, 90.0, 90.0, 90.0, 90.0, 90.0, 90.0};

//PWM settings
int cur_pwm[ROBOTIC_ARM_NUM]={1520, 1520, 1520, 1520, 1520, 1520, 1520, 1520, 250, 150};
int target_pwm[ROBOTIC_ARM_NUM]={1520, 1520, 1520, 1520, 1520, 1520, 1520, 1520, 250, 150};
const int pwm_min[ROBOTIC_ARM_NUM]={900, 900, 900, 900, 800, 800, 900, 900, 50, 50};
const int pwm_max[ROBOTIC_ARM_NUM]={2100, 2100, 2100, 2100, 2200, 2200, 2100, 2100, 250, 250};
const uint16_t pwm_period[ROBOTIC_ARM_NUM]={3, 3, 3, 3, 3, 3, 3, 3, 3, 50};//ms

//Speed settings
int next_pwm[ROBOTIC_ARM_NUM];
const int pwm_resolution[ROBOTIC_ARM_NUM]={16, 20, 20, 16, 40, 40, 40, 16, 16, 16};// x0.1 -> degree
//const float intrinsic_speed[ROBOTIC_ARM_NUM]={2.83, 2.83, 2.83, 2.83, 1.0, 1.0, 2.83, 2.83, 16, 16};// ms/degree
const float max_speed[ROBOTIC_ARM_NUM]={60.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0};// degree/s
double time_dist[ROBOTIC_ARM_NUM][SEGMENT_NUM]={
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

/**
 *	Convert PWM count to relevant angle value.
 */
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

/**
 *	High level motion control with synchronization primitives. 
 */

extern void move(int i);
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
	if(abs(delta_pwm)<pwm_resolution[index])
	  	return;
	
	const int pwm_base=cur_pwm[index];
	const double delta_t=fabs(target_angle[index]-cur_angle[index])/max_speed[index]*1000.0;//ms
	
	s[index].S_Curve_Create(delta_pwm, delta_t, time_dist[index]);
	
	const int N=(int)(delta_t/pwm_period[index]);
	for(int i=1;i<=N;i++)
	{
	  	next_pwm[index]=(int)(s[index].get_position(i*pwm_period[index]))+pwm_base;
		advance_to_next(index);
		OSTimeDlyHMSM(0, 0, 0, pwm_period[index]);
	}
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
