#ifndef _S_CURVE_H_
#define _S_CURVE_H_

/* Includes */
#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <math.h>
#include <stdlib.h>

/* Defines */
#define SEGMENT_NUM 7
#define ACCELERATION_UTILITY 0

/* 7段式S曲线，用于舵机运动的平滑加减速控制 */
typedef struct
{
	float S, T;
	float a, b;
	float ratio[SEGMENT_NUM];
	float t[SEGMENT_NUM], p[SEGMENT_NUM];
	float aa_mat[SEGMENT_NUM];
	float a_mat[SEGMENT_NUM][2];
	float vp[SEGMENT_NUM], v_mat[SEGMENT_NUM][3];
	float sp[SEGMENT_NUM], s_mat[SEGMENT_NUM][4];
}S_Curve;

/* Function Prototypes */
void Time_Init(S_Curve *_s);
float calc_a(S_Curve *_s);
float calc_b(S_Curve *_s);
void CalcBasicVars(S_Curve *_s);
void calc_aa(S_Curve *_s);
void CalcAccelerationVars(S_Curve *_s);
void calc_vp(S_Curve *_s);
void calc_velocity_matrix(S_Curve *_s);
void CalcVelocityVars(S_Curve *_s);
void calc_sp(S_Curve *_s);
void calc_position_matrix(S_Curve *_s);
void CalcPositionVars(S_Curve *_s);
int calc_gap_index(S_Curve *_s, float _t);
void s_curve_create(S_Curve *_s, float totalLen, float totalTime, float r[]);
float get_aa(S_Curve *_s, float _t);
float get_acceleration(S_Curve *_s, float _t);
float get_velocity(S_Curve *_s, float _t);
float get_position(S_Curve *_s, float _t);

#endif