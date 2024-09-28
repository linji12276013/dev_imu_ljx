#include "matrix.h"
#include "usart.h"
#include <string.h>

//状态转移矩阵
double m_T[16] = {
	1, 0, 1, 0,
	0, 1, 0, 1,
	0, 0, 1, 0,
	0, 0, 0, 1 };

//状态矩阵
double m_H[8] = {
	1, 0, 0, 0,
	0, 1, 0, 0 };

//过程噪声
double m_Q[16] = {
	1, 0, 1, 0,
	0, 1, 0, 1,
	0, 0, 1, 0,
	0, 0, 0, 1 };

//状态协方差矩阵
double m_P[16] = {
	1, 0, 1, 0,
	0, 1, 0, 1,
	0, 0, 1, 0,
	0, 0, 0, 1 };

//测量噪声
double m_R[4] = {
	100000, 0,
	0, 100000 };

//单位矩阵
double m_I[16] = {
	1, 0, 1, 0,
	0, 1, 0, 1,
	0, 0, 1, 0,
	0, 0, 0, 1 };

void InitMatrix(Matrix *matrix, int col, int row, double *array)
{
	assert_param(col > 0 && row > 0);
	matrix->col = col;
	matrix->row = row;
	memcpy(matrix->data, array, sizeof(double) * row * col);
}

void ValueMatrix(Matrix *matrix, double *array)
{
	if (*matrix->data != NULL)
	{
		memcpy(matrix->data, array, matrix->col * matrix->row * sizeof(double));
	}
}

int SizeMatrix(Matrix *matrix)
{
	return (matrix->row * matrix->col);
}

void CopyMatrix(Matrix *matrix_A, Matrix *matrix_B)
{
	matrix_B->col = matrix_A->col;
	matrix_B->row = matrix_A->row;
	memcpy(matrix_B->data, matrix_A->data, SizeMatrix(matrix_A) * sizeof(double));
}

void PrintMatrix(Matrix *matrix)
{
	for (int i = 0; i<SizeMatrix(matrix); i++)
	{
		printf("%lf\t", matrix->data[i]);
		if ((i + 1) % matrix->row == 0)
			printf("\n");
	}
}

void AddMatrix(Matrix matrix_A, Matrix matrix_B, Matrix *matrix_C)
{
	assert_param(matrix_A.col == matrix_B.col && matrix_A.row == matrix_B.row);
	matrix_C->row = matrix_A.row;
	matrix_C->col = matrix_A.col;
	for (int i = 0; i<matrix_A.row; i++)
	{
		for (int j = 0; j<matrix_A.col; j++)
		{
			matrix_C->data[i*matrix_C->col + j] = \
				matrix_A.data[i*matrix_A.col + j] + matrix_B.data[i*matrix_A.col + j];
		}
	}
}

void SubMatrix(Matrix matrix_A, Matrix matrix_B, Matrix *matrix_C)
{
	assert_param(matrix_A.col == matrix_B.col && matrix_A.row == matrix_B.row);
	matrix_C->row = matrix_A.row;
	matrix_C->col = matrix_A.col;
	for (int i = 0; i<matrix_A.row; i++)
	{
		for (int j = 0; j<matrix_A.col; j++)
		{
			matrix_C->data[i*matrix_C->col + j] = \
				matrix_A.data[i*matrix_A.col + j] - matrix_B.data[i*matrix_A.col + j];
		}
	}
}

void MulMatrix(Matrix matrix_A, Matrix matrix_B, Matrix *matrix_C)
{
	assert_param(matrix_A.row == matrix_B.col && matrix_A.col == matrix_B.row);
	matrix_C->row = matrix_A.row;
	matrix_C->col = matrix_B.col;
	for (int i = 0; i<matrix_A.row; i++)
	{
		for (int j = 0; j<matrix_B.col; j++)
		{
			for (int k = 0; k<matrix_A.row; k++)
			{
				matrix_C->data[i*matrix_C->col + j] += \
					matrix_A.data[i*matrix_A.col + k] * matrix_B.data[k*matrix_B.col + j];
			}
		}
	}
}

void TransMatrix(Matrix matrix, Matrix *matrixTemp)
{
	assert_param(matrix.col>0 && matrix.row>0);
	matrixTemp->col = matrix.row;
	matrixTemp->row = matrix.col;

	for (int i = 0; i<matrix.col; i++)
	{
		for (int j = 0; j<matrix.row; j++)
		{
			matrixTemp->data[i*matrixTemp->col + j] = matrix.data[j*matrix.col + i];
		}
	}
}

//2*2
void InvMatrix(Matrix matrix_A, Matrix *matrix_C)
{
	assert_param(matrix_A.col==2&&matrix_A.row==2);
	matrix_C->col = matrix_A.col;
	matrix_C->row = matrix_A.row;

	double f, k;
	f = matrix_A.data[3] * matrix_A.data[0] - matrix_A.data[1] * matrix_A.data[2];
	assert_param(f);
	k = 1 / f;
	matrix_C->data[0] = matrix_A.data[3] * k;
	matrix_C->data[1] = -matrix_A.data[1] *k;
	matrix_C->data[2] = -matrix_A.data[2] *k;
	matrix_C->data[3] = matrix_A.data[0] * k;
}

