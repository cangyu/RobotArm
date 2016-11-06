#include "s_curve.h"

//根据给定比例划分运动时间
void Time_Init(S_Curve *_s)
{
	for (int i = 0; i < SEGMENT_NUM; i++)
		_s->t[i] = _s->ratio[i] * _s->T;

	_s->p[0] = _s->t[0];
	for (int i = 1; i < SEGMENT_NUM; i++)
		_s->p[i] = _s->p[i - 1] + _s->t[i];
}

//加速段最大加速度
float calc_a(S_Curve *_s)
{
	float dividend = _s->S * 6 * (_s->t[4] + 2 * _s->t[5] + _s->t[6]);
	float divisor = -2 * _s->p[5] * _s->p[5] * _s->t[0] + 4 * _s->p[5] * _s->p[6] * _s->t[0] - 2 * _s->p[6] * _s->p[6] * _s->t[0] - 4 * _s->p[5] * _s->p[5] * _s->t[1] + 8 * _s->p[5] * _s->p[6] * _s->t[1] - 4 * _s->p[6] * _s->p[6] * _s->t[1] - 2 * _s->p[5] * _s->p[5] * _s->t[2] + \
		4 * _s->p[5] * _s->p[6] * _s->t[2] - 2 * _s->p[6] * _s->p[6] * _s->t[2] - _s->p[3] * _s->p[3] * (_s->t[0] + 2 * _s->t[1] + _s->t[2]) + 2 * _s->p[3] * _s->p[4] * (_s->t[0] + 2 * _s->t[1] + _s->t[2]) - \
		_s->p[4] * _s->p[4] * (_s->t[0] + 2 * _s->t[1] + _s->t[2]) + _s->p[0] * _s->p[0] * _s->t[4] + 2 * _s->p[1] * _s->p[1] * _s->t[4] - 4 * _s->p[1] * _s->p[2] * _s->t[4] + 2 * _s->p[2] * _s->p[2] * _s->t[4] + 3 * _s->p[1] * _s->t[1] * _s->t[4] + \
		3 * _s->t[0] * _s->t[2] * _s->t[4] + 6 * _s->t[1] * _s->t[2] * _s->t[4] + 3 * _s->t[0] * _s->t[3] * _s->t[4] + 6 * _s->t[1] * _s->t[3] * _s->t[4] + 3 * _s->t[2] * _s->t[3] * _s->t[4] + 3 * _s->t[0] * _s->t[4] * _s->t[4] + 6 * _s->t[1] * _s->t[4] * _s->t[4] + \
		3 * _s->t[2] * _s->t[4] * _s->t[4] + 2 * _s->p[0] * _s->p[0] * _s->t[5] + 4 * _s->p[1] * _s->p[1] * _s->t[5] - 8 * _s->p[1] * _s->p[2] * _s->t[5] + 4 * _s->p[2] * _s->p[2] * _s->t[5] - 3 * _s->p[5] * _s->t[0] * _s->t[5] + 6 * _s->p[1] * _s->t[1] * _s->t[5] - \
		6 * _s->p[5] * _s->t[1] * _s->t[5] - 3 * _s->p[5] * _s->t[2] * _s->t[5] + 6 * _s->t[0] * _s->t[2] * _s->t[5] + 12 * _s->t[1] * _s->t[2] * _s->t[5] + 3 * _s->p[4] * (_s->t[0] + 2 * _s->t[1] + _s->t[2])*_s->t[5] + \
		6 * _s->t[0] * _s->t[3] * _s->t[5] + 12 * _s->t[1] * _s->t[3] * _s->t[5] + 6 * _s->t[2] * _s->t[3] * _s->t[5] + 6 * _s->t[0] * _s->t[4] * _s->t[5] + 12 * _s->t[1] * _s->t[4] * _s->t[5] + 6 * _s->t[2] * _s->t[4] * _s->t[5] + \
		6 * _s->t[0] * _s->t[5] * _s->t[5] + 12 * _s->t[1] * _s->t[5] * _s->t[5] + 6 * _s->t[2] * _s->t[5] * _s->t[5] + _s->p[0] * _s->p[0] * _s->t[6] + 2 * _s->p[1] * _s->p[1] * _s->t[6] - 4 * _s->p[1] * _s->p[2] * _s->t[6] + 2 * _s->p[2] * _s->p[2] * _s->t[6] + 3 * _s->p[1] * _s->t[1] * _s->t[6] + \
		3 * _s->t[0] * _s->t[2] * _s->t[6] + 6 * _s->t[1] * _s->t[2] * _s->t[6] + 3 * _s->t[0] * _s->t[3] * _s->t[6] + 6 * _s->t[1] * _s->t[3] * _s->t[6] + 3 * _s->t[2] * _s->t[3] * _s->t[6] + 3 * _s->t[0] * _s->t[4] * _s->t[6] + 6 * _s->t[1] * _s->t[4] * _s->t[6] + \
		3 * _s->t[2] * _s->t[4] * _s->t[6] + 3 * _s->t[0] * _s->t[5] * _s->t[6] + 6 * _s->t[1] * _s->t[5] * _s->t[6] + 3 * _s->t[2] * _s->t[5] * _s->t[6] + 3 * _s->t[0] * _s->t[6] * _s->t[6] + 6 * _s->t[1] * _s->t[6] * _s->t[6] + 3 * _s->t[2] * _s->t[6] * _s->t[6];//Thanks to mathematica...

	return dividend / divisor;
}

//减速段最小加速度
float calc_b(S_Curve *_s)
{
	return -_s->a*(_s->t[0] + 2 * _s->t[1] + _s->t[2]) / (_s->t[4] + 2 * _s->t[5] + _s->t[6]);
}

//描述曲线的基本量
void CalcBasicVars(S_Curve *_s)
{
	_s->a = calc_a(_s);
	_s->b = calc_b(_s);
}

//加加速度
void calc_aa(S_Curve *_s)
{
	_s->aa_mat[0] = _s->a / _s->t[0];
	_s->aa_mat[1] = 0;
	_s->aa_mat[2] = -_s->a / _s->t[2];
	_s->aa_mat[3] = 0;
	_s->aa_mat[4] = _s->b / _s->t[4];
	_s->aa_mat[5] = 0;
	_s->aa_mat[6] = -_s->b / _s->t[6];
}

//加速度矩阵
void calc_acceleration_matrix(S_Curve *_s)
{
	_s->a_mat[0][0] = 0; 
	_s->a_mat[0][1] = _s->a / _s->t[0];
	
	_s->a_mat[1][0] = _s->a;
	_s->a_mat[1][1] = 0;
	
	_s->a_mat[2][0] = _s->a / _s->t[2] * _s->p[2];
	_s->a_mat[2][1] = -_s->a / _s->t[2];
	
	_s->a_mat[3][0] = 0; 
	_s->a_mat[3][1] = 0;
	
	_s->a_mat[4][0] = -_s->b*_s->p[3] / _s->t[4]; 
	_s->a_mat[4][1] = _s->b / _s->t[4];
	
	_s->a_mat[5][0] = _s->b; 
	_s->a_mat[5][1] = 0;
	
	_s->a_mat[6][0] = _s->b*_s->p[6] / _s->t[6]; 
	_s->a_mat[6][1] = -_s->b / _s->t[6];
}

//加速度相关量
void CalcAccelerationVars(S_Curve *_s)
{
	calc_aa(_s);
	calc_acceleration_matrix(_s);
}

//节点速度
void calc_vp(S_Curve *_s)
{
	_s->vp[0] = _s->a*0.5*_s->t[0];
	_s->vp[1] = _s->vp[0] + _s->a*_s->t[1];
	_s->vp[2] = _s->vp[1] + _s->a*0.5*_s->t[2];
	_s->vp[3] = _s->vp[2];
	_s->vp[4] = _s->vp[3] + _s->b*0.5*_s->t[4];
	_s->vp[5] = _s->vp[4] + _s->b*_s->t[5];
	_s->vp[6] = 0.0;
}

//速度矩阵
void calc_velocity_matrix(S_Curve *_s)
{
	_s->v_mat[0][0] = 0; 
	_s->v_mat[0][1] = 0; 
	_s->v_mat[0][2] = 0.5*_s->a / _s->t[0];
	
	_s->v_mat[1][0] = -0.5*_s->a*_s->p[0]; 
	_s->v_mat[1][1] = _s->a;
	_s->v_mat[1][2] = 0;
	
	_s->v_mat[2][0] = _s->vp[1] - _s->a*_s->p[1] / _s->t[2] * (_s->p[2] - 0.5*_s->p[1]);
	_s->v_mat[2][1] = _s->a*_s->p[2] / _s->t[2]; 
	_s->v_mat[2][2] = -0.5*_s->a / _s->t[2];
	
	_s->v_mat[3][0] = _s->vp[2]; 
	_s->v_mat[3][1] = 0; 
	_s->v_mat[3][2] = 0;
	
	_s->v_mat[4][0] = _s->vp[3] + _s->b*_s->p[3] * _s->p[3] / 2 / _s->t[4]; 
	_s->v_mat[4][1] = -_s->b*_s->p[3] / _s->t[4]; 
	_s->v_mat[4][2] = 0.5*_s->b / _s->t[4];
	
	_s->v_mat[5][0] = _s->vp[4] - _s->b*_s->p[4];
	_s->v_mat[5][1] = _s->b;
	_s->v_mat[5][2] = 0;
	
	_s->v_mat[6][0] = _s->vp[5] - _s->b*(_s->p[6] * _s->p[5] - 0.5*_s->p[5] * _s->p[5]) / _s->t[6];
	_s->v_mat[6][1] = _s->b*_s->p[6] / _s->t[6]; 
	_s->v_mat[6][2] = -0.5*_s->b / _s->t[6];
}

//速度相关量
void CalcVelocityVars(S_Curve *_s)
{
	calc_vp(_s);
	calc_velocity_matrix(_s);
}

//节点位移
void calc_sp(S_Curve *_s)
{
	_s->sp[0] = _s->a*pow(_s->p[0], 2) / 6;
	for (int i = 1; i < SEGMENT_NUM; i++)
	{
		_s->sp[i] = _s->sp[i - 1];
		_s->sp[i] += _s->v_mat[i][0] * (_s->p[i] - _s->p[i - 1]);
		_s->sp[i] += _s->v_mat[i][1] * (pow(_s->p[i], 2) - pow(_s->p[i - 1], 2)) / 2;
		_s->sp[i] += _s->v_mat[i][2] * (pow(_s->p[i], 3) - pow(_s->p[i - 1], 3)) / 3;
	}
}

//位移矩阵
void calc_position_matrix(S_Curve *_s)
{
	_s->s_mat[0][0] = 0; 
	_s->s_mat[0][1] = 0; 
	_s->s_mat[0][2] = 0; 
	_s->s_mat[0][3] = _s->a / _s->t[0] / 6;
	
	for (int i = 1; i < SEGMENT_NUM; i++)
	{
		_s->s_mat[i][0] = _s->sp[i - 1] - (_s->v_mat[i][0] * _s->p[i - 1] + _s->v_mat[i][1] * pow(_s->p[i - 1], 2) / 2 + _s->v_mat[i][2] * pow(_s->p[i - 1], 3) / 3);
		_s->s_mat[i][1] = _s->v_mat[i][0];
		_s->s_mat[i][2] = _s->v_mat[i][1] / 2;
		_s->s_mat[i][3] = _s->v_mat[i][2] / 3;
	}
}

//位移相关量
void CalcPositionVars(S_Curve *_s)
{
	calc_sp(_s);
	calc_position_matrix(_s);
}

//时间段
int calc_gap_index(S_Curve *_s, float _t)
{
	int i = 0;
	while (i < SEGMENT_NUM && _t > _s->p[i])
		++i;

	assert(i < SEGMENT_NUM);

	return i;
}

//生成曲线
void s_curve_create(S_Curve *_s, float totalLen, float totalTime, float r[])
{
	_s->S=totalLen;
	_s->T=totalTime;
	
	for(int i=0;i<SEGMENT_NUM;i++)
		_s->ratio[i]=r[i];

	Time_Init(_s);
	CalcBasicVars(_s);
#if(ACCELERATION_UTILITY)
	CalcAccelerationVars(_s);
#endif
	CalcVelocityVars(_s);
	CalcPositionVars(_s);
}

//计算加加速度
float get_aa(S_Curve *_s, float _t)
{
	int i = calc_gap_index(_s, _t);
	return _s->aa_mat[i];
}

//计算加速度
float get_acceleration(S_Curve *_s, float _t)
{
	int i = calc_gap_index(_s, _t);
	return _s->a_mat[i][0] + _t*_s->a_mat[i][1];
}

//计算速度
float get_velocity(S_Curve *_s, float _t)
{
	int i = calc_gap_index(_s, _t);
	return _s->v_mat[i][0] + _t*(_s->v_mat[i][1] + _t*_s->v_mat[i][2]);
}

//计算位移
float get_position(S_Curve *_s, float _t)
{
	int i = calc_gap_index(_s, _t);
	return _s->s_mat[i][0] + _t*(_s->s_mat[i][1] + _t*(_s->s_mat[i][2] + _t *_s->s_mat[i][3]));//Horner's rule
}
