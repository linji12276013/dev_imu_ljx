#ifndef __MATRIX_H
#define __MATRIX_H

#include "main.h"

//申请最大矩阵内存
#define MATSIZE 50
//结构体
typedef struct
{
	int row, col;		//row为行,col
//	Matrix() {};
//	Matrix(int _row,int _col) :row(_row), col(_col) {}
	double data[MATSIZE];
}Matrix;

extern double m_T[16];
extern double m_H[8];
extern double m_Q[16];
extern double m_P[16];
extern double m_R[4];
extern double m_I[16];

void InitMatrix(Matrix *matrix, int col, int row, double *array);		//初始化矩阵
void ValueMatrix(Matrix *matrix, double *array);				//给一个矩阵赋值
int SizeMatrix(Matrix *matrix);								//获得一个矩阵的大小
void CopyMatrix(Matrix *matrix_A, Matrix *matrix_B);		//复制一个矩阵的值
void PrintMatrix(Matrix *matrix);							//打印一个矩阵

															//矩阵的基本运算
void AddMatrix(Matrix matrix_A, Matrix matrix_B, Matrix *matrix_C);		//矩阵的加法
void SubMatrix(Matrix matrix_A, Matrix matrix_B, Matrix *matrix_C);		//矩阵剑法
void MulMatrix(Matrix matrix_A, Matrix matrix_B, Matrix *matrix_C);		//矩阵的乘法
void TransMatrix(Matrix matrix, Matrix *matrixTemp);					//矩阵的转置
void InvMatrix(Matrix matrix_A, Matrix *matrix_C);                      //逆矩阵

#endif
