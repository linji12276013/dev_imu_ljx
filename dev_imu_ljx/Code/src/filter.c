#include "filter.h"
#include "matrix.h"
#define dt	0.002

float Kalman_Filter_x(float Accel,float Gyro)		
{
	static float angle_dot;
	static float angle;
	float Q_angle=0.001; // ¹ý³ÌÔëÉùµÄÐ­·½²î
	float Q_gyro=0.003;	//0.003 ¹ý³ÌÔëÉùµÄÐ­·½²î ¹ý³ÌÔëÉùµÄÐ­·½²îÎªÒ»¸öÒ»ÐÐÁ½ÁÐ¾ØÕó
	float R_angle=0.5;		// ²âÁ¿ÔëÉùµÄÐ­·½²î ¼È²âÁ¿Æ«²î
	char  C_0 = 1;
	static float Q_bias, Angle_err;
	static float PCt_0, PCt_1, E;
	static float K_0, K_1, t_0, t_1;
	static float Pdot[4] ={0,0,0,0};
	static float PP[2][2] = { { 1, 0 },{ 0, 1 } };
	angle+=(Gyro - Q_bias) * dt; //ÏÈÑé¹À¼Æ
	Pdot[0]=Q_angle - PP[0][1] - PP[1][0]; // Pk-ÏÈÑé¹À¼ÆÎó²îÐ­·½²îµÄÎ¢·Ö

	Pdot[1]=-PP[1][1];
	Pdot[2]=-PP[1][1];
	Pdot[3]=Q_gyro;
	PP[0][0] += Pdot[0] * dt;   // Pk-ÏÈÑé¹À¼ÆÎó²îÐ­·½²îÎ¢·ÖµÄ»ý·Ö
	PP[0][1] += Pdot[1] * dt;   // =ÏÈÑé¹À¼ÆÎó²îÐ­·½²î
	PP[1][0] += Pdot[2] * dt;
	PP[1][1] += Pdot[3] * dt;
		
	Angle_err = Accel - angle;	//zk-ÏÈÑé¹À¼Æ
	
	PCt_0 = C_0 * PP[0][0];
	PCt_1 = C_0 * PP[1][0];

	E = R_angle + C_0 * PCt_0;

	K_0 = PCt_0 / E;
	K_1 = PCt_1 / E;

	t_0 = PCt_0;
	t_1 = C_0 * PP[0][1];

	PP[0][0] -= K_0 * t_0;		 //ºóÑé¹À¼ÆÎó²îÐ­·½²î
	PP[0][1] -= K_0 * t_1;
	PP[1][0] -= K_1 * t_0;
	PP[1][1] -= K_1 * t_1;

	angle	+= K_0 * Angle_err;	 //ºóÑé¹À¼Æ
	Q_bias	+= K_1 * Angle_err;	 //ºóÑé¹À¼Æ
	angle_dot   = Gyro - Q_bias;	 //Êä³öÖµ(ºóÑé¹À¼Æ)µÄÎ¢·Ö=½ÇËÙ¶È
	return angle;
}
/**************************************************************************
Function: First order complementary filtering
Input   : acceleration¡¢angular velocity
Output  : none
º¯Êý¹¦ÄÜ£ºÒ»½×»¥²¹ÂË²¨
Èë¿Ú²ÎÊý£º¼ÓËÙ¶È»ñÈ¡µÄ½Ç¶È¡¢½ÇËÙ¶È
·µ»Ø  Öµ£ºxÖá½ÇËÙ¶È
**************************************************************************/
float Complementary_Filter_x(float angle_m, float gyro_m)
{
	static float angle;
	float K1 =0.02;
	angle = K1 * angle_m+ (1-K1) * (angle + gyro_m * dt);
	return angle;
}
/**************************************************************************
Function: Simple Kalman filter
Input   : acceleration¡¢angular velocity
Output  : none
º¯Êý¹¦ÄÜ£º»ñÈ¡yÖá½Ç¶È¼òÒ×¿¨¶ûÂüÂË²¨
Èë¿Ú²ÎÊý£º¼ÓËÙ¶È»ñÈ¡µÄ½Ç¶È¡¢½ÇËÙ¶È
·µ»Ø  Öµ£ºyÖá½ÇËÙ¶È
**************************************************************************/
float Kalman_Filter_y(float Accel,float Gyro)		
{
	static float angle_dot;
	static float angle;
	float Q_angle=0.001; // ¹ý³ÌÔëÉùµÄÐ­·½²î
	float Q_gyro=0.003;	//0.003 ¹ý³ÌÔëÉùµÄÐ­·½²î ¹ý³ÌÔëÉùµÄÐ­·½²îÎªÒ»¸öÒ»ÐÐÁ½ÁÐ¾ØÕó
	float R_angle=0.5;		// ²âÁ¿ÔëÉùµÄÐ­·½²î ¼È²âÁ¿Æ«²î
	char  C_0 = 1;
	static float Q_bias, Angle_err;
	static float PCt_0, PCt_1, E;
	static float K_0, K_1, t_0, t_1;
	static float Pdot[4] ={0,0,0,0};
	static float PP[2][2] = { { 1, 0 },{ 0, 1 } };
	angle+=(Gyro - Q_bias) * dt; //ÏÈÑé¹À¼Æ
	Pdot[0]=Q_angle - PP[0][1] - PP[1][0]; // Pk-ÏÈÑé¹À¼ÆÎó²îÐ­·½²îµÄÎ¢·Ö
	Pdot[1]=-PP[1][1];
	Pdot[2]=-PP[1][1];
	Pdot[3]=Q_gyro;
	PP[0][0] += Pdot[0] * dt;   // Pk-ÏÈÑé¹À¼ÆÎó²îÐ­·½²îÎ¢·ÖµÄ»ý·Ö
	PP[0][1] += Pdot[1] * dt;   // =ÏÈÑé¹À¼ÆÎó²îÐ­·½²î
	PP[1][0] += Pdot[2] * dt;
	PP[1][1] += Pdot[3] * dt;
	Angle_err = Accel - angle;	//zk-ÏÈÑé¹À¼Æ
	
	PCt_0 = C_0 * PP[0][0];
	PCt_1 = C_0 * PP[1][0];
	
	E = R_angle + C_0 * PCt_0;
	
	K_0 = PCt_0 / E;
	K_1 = PCt_1 / E;
	
	t_0 = PCt_0;
	t_1 = C_0 * PP[0][1];

	PP[0][0] -= K_0 * t_0;		 //ºóÑé¹À¼ÆÎó²îÐ­·½²î
	PP[0][1] -= K_0 * t_1;
	PP[1][0] -= K_1 * t_0;
	PP[1][1] -= K_1 * t_1;
		
	angle	+= K_0 * Angle_err;	   //ºóÑé¹À¼Æ
	Q_bias	+= K_1 * Angle_err;	 //ºóÑé¹À¼Æ
	angle_dot   = Gyro - Q_bias;	//Êä³öÖµ(ºóÑé¹À¼Æ)µÄÎ¢·Ö=½ÇËÙ¶È
	return angle;
}
/**************************************************************************
Function: First order complementary filtering
Input   : acceleration¡¢angular velocity
Output  : none
º¯Êý¹¦ÄÜ£ºÒ»½×»¥²¹ÂË²¨
Èë¿Ú²ÎÊý£º¼ÓËÙ¶È»ñÈ¡µÄ½Ç¶È¡¢½ÇËÙ¶È
·µ»Ø  Öµ£ºyÖá½ÇËÙ¶È
**************************************************************************/
float Complementary_Filter_y(float angle_m, float gyro_m)
{
	static float angle;
	float K1 =0.02; 
	angle = K1 * angle_m+ (1-K1) * (angle + gyro_m * dt);
	return angle;
}



float Contemprary_filter(float bias, float last_bias, float k_filter)
{
	return (k_filter * bias + (1 - k_filter) * last_bias);
}
void kalman_param_init(kalman *filter)
{
	filter->xk_1 = 0;
	filter->Pk_1 = 1;
	filter->xkk_1 = 0;
	filter->Pkk_1 = 0;
	filter->Kk = 0;
	filter->xk = 0;
	filter->Pk = 0;
	filter->uk = 0;
	filter->A = 1;
	filter->H = 1;
	filter->B = 1;
	filter->Q = 0.0001f;
	filter->R = 0.5f;
}

/*一维卡尔曼滤波算法实现
Time Update(prediction)公式：
  x(k|k-1) = A * x(k-1|k-1) + B * u(k) + w(k)
  P(k|k-1) = A * P(k-1|k-1) * A^T + Q
Measurement Update(correction)公式：
  K(k) = P(k|k-1) * H^T * (H * P(k|k-1) * H^T + R)^-1
  x(k) = x(k|k-1) + K(k) * (z(k) - H * x(k|k-1))
  P(k) = (I - K(k) * H) * P(k|k-1)

A，B，H大多数情况下，它们为常数，系统为一维情况下为1，对于一个卡尔曼滤波器我们需要确定以下参数（难点）：

1)R，初始值可根据传感器测量误差进行选择

2)Q，初始值可根据实际调试确定

3)x0，初始值一般可设置为0

4)P0，初始值一般可设置为1，P0=0表示完全相信测量值，P0=1表示完全相信估计值，一般是
  不能设置为0，否则Pk就一直为0了，可设置为1，在后续迭代的过程中，Pk会不断被修正

参数调节的一般方法：

根据公式，我们可以分析得出：R越大，Kk越小；Q越大，Kk越大。Kk越大我们越相信测量值，当我们使用的传
感器比较精密，测量噪声很小时，我们越相信测量值，此时应该调小R，反之则调大，另外，当我们的系统模型
越准确时，我们的预测值会越精确，此时我们越信任预测值，则应该调小Q。
*/
float calkalmanfilter(kalman *filter, float zk, float Q, float R)
{
	filter->Q = Q;
	filter->R = R;
	//prediction
	filter->xkk_1 = filter->A * filter->xk_1 + filter->B * filter->uk;
	filter->Pkk_1 = filter->A * filter->Pk_1 + filter->Q;

	//correction
	filter->Kk = filter->Pkk_1 / (filter->Pkk_1 + filter->R);
	filter->xk = filter->xkk_1 + filter->Kk * (zk - filter->H * filter->xkk_1);
	filter->Pk = (1 - filter->Kk * filter->H) * filter->Pkk_1;

	filter->xk_1 = filter->xk;
	filter->Pk_1 = filter->Pk;
	
	return filter->xk;
}

/*二维卡尔曼滤波算法实现

*/
void KalManInit(kalman_matrix *km, double X, double Y)
{
	//初始化矩阵
	InitMatrix(&km->H, 4, 2, m_H);
	InitMatrix(&km->Q, 4, 4, m_Q);
	InitMatrix(&km->P, 4, 4, m_P);
	InitMatrix(&km->R, 2, 2, m_R);
	InitMatrix(&km->I, 4, 4, m_I);
	InitMatrix(&km->T, 4, 4, m_T);

	//初始化起始预测坐标
	km->xx.data[0] = X; km->xx.data[1] = Y;
	km->xx.data[2] = 1; km->xx.data[3] = 1;
}

//通过调整测量噪声和过程噪声来改变卡尔曼的增益
void KalManm_Predict(kalman_matrix *km, double X, double Y, double* _X, double* _Y)
{
	Matrix tmp2, tmp1, tT, t_H, invS;
	//测量值赋值
	km->yy.data[0] = X, km->yy.data[1] = Y;

	//预测状态向量
	MulMatrix(km->T, km->xx, &km->Pre_x);

	//预测状态协方差矩阵
	MulMatrix(km->T, km->P, &tmp1);
	TransMatrix(km->T, &tT);
	MulMatrix(tmp1, tT, &tmp2);
	AddMatrix(tmp2, km->Q, &km->Pre_p);

	//测量值和预测值的差值
	MulMatrix(km->H, km->Pre_x, &tmp1);	//计算预测值
	SubMatrix(km->yy, tmp1, &km->Pre_y);

	//卡尔曼增益计算
	MulMatrix(km->H, km->Pre_p, &tmp1);
	TransMatrix(km->H, &t_H);
	MulMatrix(tmp1, t_H, &tmp2);
	AddMatrix(tmp2, km->R, &km->S);
	MulMatrix(km->Pre_p, t_H, &tmp1);
	InvMatrix(km->S, &invS);
	MulMatrix(tmp1, invS, &km->K);

	//更新当前状态，形成闭环
	MulMatrix(km->K, km->Pre_y, &tmp1);
	AddMatrix(km->Pre_x, tmp1, &km->x_final);
	//更新系统的协方差，形成闭环
	MulMatrix(km->K, km->H, &tmp1);
	SubMatrix(km->I, tmp1, &tmp2);
	MulMatrix(tmp2, km->Pre_p, &km->P);

	//本次状态的最终预测值，下次状态的输入值
	km->xx.data[0] = km->x_final.data[0], km->xx.data[1] = km->x_final.data[1];

	//当前状态预测值输出
	*_X = km->x_final.data[0], *_Y = km->x_final.data[1];
}
