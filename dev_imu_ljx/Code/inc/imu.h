#ifndef __IMU_H
#define __IMU_H

#include "main.h"

//void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az);
void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az);
void Get_EularAngle(float roll, float pitch, float yaw);

void calculateAttitude( float ax, float ay, float az, \
						float gx, float gy, float gz, \
						float *roll, float *pitch, float *yaw);

#endif
