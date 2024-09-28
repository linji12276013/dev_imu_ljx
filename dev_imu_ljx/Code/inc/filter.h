#ifndef __FILTER_H
#define __FILTER_H

#include "main.h"
#include "matrix.h"

typedef struct kalman_filter
{
	float xk_1;		//x(k-1|k-1)
	float Pk_1;		//P(k-1|k-1)
	float xkk_1;	//x(k|k-1)
	float Pkk_1;	//P(k|k-1)
	float Kk;		//Kk
	float xk;		//x(k|k)
	float Pk;		//P(k|k)
	float uk;		//u(k)
	float A;
	float H;
	float B;
	float Q;
	float R;
}kalman;

typedef struct 
{
	Matrix xx;//4*1
	Matrix yy;//2*1
	Matrix H;//2*4			//状态矩阵
	Matrix Q;//4*4			//过程噪声
	Matrix P;//4*4			//状态协方差矩阵
	Matrix R;//2*2			//测量噪声
	Matrix I;//4*4			//单位矩阵
	Matrix T;//4*4			//状态转移矩阵

	//中间参数矩阵
	Matrix Pre_x;//4*1
	Matrix Pre_y;//2*1
	Matrix Pre_p;//4*4		//后验协方差
	Matrix x_final;//2*1	//最终预测坐标
	
	Matrix K;//2*4			//kalman增益
	Matrix S;//2*2
}kalman_matrix;

float Kalman_Filter_x(float Accel,float Gyro);
float Kalman_Filter_y(float Accel,float Gyro);
float Complementary_Filter_x(float angle_m, float gyro_m);
float Complementary_Filter_y(float angle_m, float gyro_m);

float Contemprary_filter(float bias, float last_bias, float k_filter);

void kalman_param_init(kalman *filter);
float calkalmanfilter(kalman *filter, float zk, float Q, float R);

void KalManInit(kalman_matrix *km, double X, double Y);
void KalManm_Predict(kalman_matrix *km, double X, double Y, double* _X, double* _Y);

#endif
