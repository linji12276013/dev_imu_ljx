#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "soft_iic.h"
#include "filter.h"
#include "usart.h"
#include <math.h>

#define PRINT_ACCEL     (0x01)
#define PRINT_GYRO      (0x02)
#define PRINT_QUAT      (0x04)
#define ACCEL_ON        (0x01)
#define GYRO_ON         (0x02)
#define MOTION          (0)
#define NO_MOTION       (1)
#define DEFAULT_MPU_HZ  (200)
#define FLASH_SIZE      (512)
#define FLASH_MEM_START ((void*)0x1800)
#define q30  1073741824.0f
short gyro[3], accel[3], sensors;
kalman accelk[3], gyrok[3];
float accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z;
float angle_x, angle_y;
float Roll, Pitch, Yaw;
kalman pitchk, rollk, yawk;
float q0=1.0f,q1=0.0f,q2=0.0f,q3=0.0f;
static signed char gyro_orientation[9] = {-1, 0, 0,
                                           0,-1, 0,
                                           0, 0, 1};

static soft_iic_init_struct dmp_iic_struct = {GPIOC, GPIO_PIN_10, GPIOC, GPIO_PIN_11, 0x68, 100000, 1};
/**
  * @brief  向IIC设备的寄存器连续写入数据，带超时重试设置，供mpu接口调用
  * @param  Address: IIC设备地址
  * @param  RegisterAddr: 寄存器地址
  * @param  RegisterLen: 要写入数据的长度
  * @param  RegisterValue: 要指向写入数据的指针
  * @retval 0正常，非0异常
  */
int Sensors_I2C_WriteRegister(unsigned char slave_addr,
                              unsigned char reg_addr,
                              unsigned short len,
                              const unsigned char *data_ptr)
{
	char retries = 0;
	int ret = 0;
	unsigned short retry_in_mlsec = get_iic_retry();

tryWriteAgain:
	ret = 0;
	ret = soft_iic_write(&dmp_iic_struct, slave_addr, reg_addr, len, (unsigned char *)data_ptr);

	if(ret && retry_in_mlsec)
	{
		if (retries++ > 4)
			return ret;

		HAL_Delay(retry_in_mlsec);
		goto tryWriteAgain;
	}
	return ret;
}

/**
  * @brief  向IIC设备的寄存器连续读出数据,带超时重试设置，供mpu接口调用
  * @param  Address: IIC设备地址
  * @param  RegisterAddr: 寄存器地址
  * @param  RegisterLen: 要读取的数据长度
  * @param  RegisterValue: 指向存储读出数据的指针
  * @retval 0正常，非0异常
  */
int Sensors_I2C_ReadRegister(unsigned char slave_addr,
                             unsigned char reg_addr,
                             unsigned short len,
                             unsigned char *data_ptr)
{
	char retries=0;
	int ret = 0;
	unsigned short retry_in_mlsec = get_iic_retry();

tryReadAgain:
	ret = 0;
	ret = soft_iic_read(&dmp_iic_struct, slave_addr, reg_addr, len, (unsigned char *)data_ptr);

	if (ret && retry_in_mlsec)
	{
		if( retries++ > 4 )
			return ret;

		HAL_Delay(retry_in_mlsec);
		goto tryReadAgain;
	}
	return ret;
}										   

/**
  * @brief   写数据到MPU6050寄存器
  * @param   
  * @retval  
  */
void MPU6050_WriteReg(uint8_t reg_add,uint8_t reg_dat)
{
	assert_param(!Sensors_I2C_WriteRegister(MPU6050_SLAVE_ADDRESS >> 1, reg_add, 1, &reg_dat));
}

/**
  * @brief   从MPU6050寄存器读取数据
  * @param   
  * @retval  
  */
void MPU6050_ReadData(uint8_t reg_add, unsigned char *Read, uint8_t num)
{
	assert_param(!Sensors_I2C_ReadRegister(MPU6050_SLAVE_ADDRESS >> 1, reg_add, num, Read));
}

/**
  * @brief   读取MPU6050的ID
  * @param   
  * @retval  
  */
uint8_t MPU6050ReadID(void)
{
	unsigned char Re = 0;
    MPU6050_ReadData(MPU6050_RA_WHO_AM_I, &Re, 1);    //读器件地址
	assert_param(Re == 0x68);//检测不到MPU6050模块，请检查模块与开发板的接线
	return 1;
}

/**
  * @brief   读取MPU6050的加速度数据
  * @param   
  * @retval  
  */
void MPU6050ReadAcc(short *accData)
{
    uint8_t buf[6];
    MPU6050_ReadData(MPU6050_ACC_OUT, buf, 6);
    accData[0] = (buf[0] << 8) | buf[1];
    accData[1] = (buf[2] << 8) | buf[3];
    accData[2] = (buf[4] << 8) | buf[5];
}

/**
  * @brief   读取MPU6050的角加速度数据
  * @param   
  * @retval  
  */
void MPU6050ReadGyro(short *gyroData)
{
    uint8_t buf[6];
    MPU6050_ReadData(MPU6050_GYRO_OUT, buf, 6);
    gyroData[0] = (buf[0] << 8) | buf[1];
    gyroData[1] = (buf[2] << 8) | buf[3];
    gyroData[2] = (buf[4] << 8) | buf[5];
}

/**
  * @brief   读取MPU6050的温度数据，转化成摄氏度
  * @param   
  * @retval  
  */
void MPU6050_ReturnTemp(float *Temperature)
{
	short temp3;
	uint8_t buf[2];
	
	MPU6050_ReadData(MPU6050_RA_TEMP_OUT_H,buf,2);     //读取温度值
    temp3= (buf[0] << 8) | buf[1];	
	*Temperature=((double) temp3/340.0)+36.53;
}

void MPU6050_CorrectData(void)
{
	static short ax1,ax2,ax3,ay1,ay2,ay3,az1,az2,az3;

	gyro[0]+=95,gyro[1]+=48,gyro[2]+=-6.5;//陀螺仪零漂校准
	ax2=ax1,ay2=ay1,az2=az1;
	ax1=accel[0],ay1=accel[1],az1=accel[2];
	if((ax1-ax2)<=100&&(ax1-ax2>=-100))
	{
		ax3=(ax1-ax2),ay3=(ay1-ay2),az3=(az1-az2);//加速度计零漂校准
	}
	accel[0]-=ax3,accel[1]-=ay3,accel[2]-=az3;
}

void MPU6050_FilterData(float Q, float R)
{
	accel[0] = calkalmanfilter(&accelk[0], accel[0], Q, R);
	accel[1] = calkalmanfilter(&accelk[1], accel[1], Q, R);
	accel[2] = calkalmanfilter(&accelk[2], accel[2], Q, R);
	gyro[0] = calkalmanfilter(&gyrok[0], gyro[0], Q, R);
	gyro[0] = calkalmanfilter(&gyrok[1], gyro[1], Q, R);
	gyro[0] = calkalmanfilter(&gyrok[2], gyro[2], Q, R);
}

void MPU6050_TransData(void)
{
	accel_x = (float)accel[0] / 16384.0f / 9.8f;
	accel_y = (float)accel[1] / 16384.0f / 9.8f;
	accel_z = (float)accel[2] / 16384.0f / 9.8f;
	gyro_x = (float)gyro[0] / 131.2f / 57.3f;
	gyro_y = (float)gyro[1] / 131.2f / 57.3f;
	gyro_z = (float)gyro[2] / 131.2f / 57.3f;

	angle_x = atan2(accel_y, accel_z) * 57.3;
	angle_y = atan2(accel_x, accel_z) * 57.3;
}

/**
  * @brief   MPU6050原始数据
  * @param   
  * @retval  
  * @note    
  */
void MPU6050_GetData(void)
{	
	int id = MPU6050ReadID();
	if (id)//器件ID正确
	{
		/*获取原始数据*/
		MPU6050ReadAcc(accel);
		MPU6050ReadGyro(gyro);
	}
}

static unsigned short inv_row_2_scale(const signed char *row)
{
    unsigned short b;

    if (row[0] > 0)
        b = 0;
    else if (row[0] < 0)
        b = 4;
    else if (row[1] > 0)
        b = 1;
    else if (row[1] < 0)
        b = 5;
    else if (row[2] > 0)
        b = 2;
    else if (row[2] < 0)
        b = 6;
    else
        b = 7;      // error
    return b;
}

static unsigned short inv_orientation_matrix_to_scalar(const signed char *mtx)
{
    unsigned short scalar;
    scalar = inv_row_2_scale(mtx);
    scalar |= inv_row_2_scale(mtx + 3) << 3;
    scalar |= inv_row_2_scale(mtx + 6) << 6;

    return scalar;
}

static void run_self_test(void)
{
    int result;
    long gyro[3], accel[3];

    result = mpu_run_self_test(gyro, accel);
    if (result == 0x7) {
        /* Test passed. We can trust the gyro data here, so let's push it down
         * to the DMP.
         */
        float sens;
        unsigned short accel_sens;
        mpu_get_gyro_sens(&sens);
        gyro[0] = (long)(gyro[0] * sens);
        gyro[1] = (long)(gyro[1] * sens);
        gyro[2] = (long)(gyro[2] * sens);
        dmp_set_gyro_bias(gyro);
        mpu_get_accel_sens(&accel_sens);
        accel[0] *= accel_sens;
        accel[1] *= accel_sens;
        accel[2] *= accel_sens;
        dmp_set_accel_bias(accel);
		printf("setting bias succesfully ......\r\n");
    }
}

void MPU6050_Init(void)
{
	I2C_Bus_Init(&dmp_iic_struct);
	HAL_Delay(1000);
	MPU6050_WriteReg(MPU6050_RA_PWR_MGMT_1, 0x00);	    //解除休眠状态
	MPU6050_WriteReg(MPU6050_RA_SMPLRT_DIV , 0x07);	    //陀螺仪采样率1KHz
	MPU6050_WriteReg(MPU6050_RA_CONFIG , 0x06);	        //低通滤波器的设置，截止频率是1k，带宽是5k
	MPU6050_WriteReg(MPU6050_RA_ACCEL_CONFIG , 0x00);	  //加速度计2g模式，不自检
	MPU6050_WriteReg(MPU6050_RA_GYRO_CONFIG, 0x00);     //陀螺仪自检及测量范围：典型值0x18（不自检）
}

void DMP_Init(void)
{
	I2C_Bus_Init(&dmp_iic_struct);
	kalman_param_init(&accelk[0]);
	kalman_param_init(&accelk[1]);
	kalman_param_init(&accelk[2]);
	kalman_param_init(&gyrok[0]);
	kalman_param_init(&gyrok[1]);
	kalman_param_init(&gyrok[2]);

	uint8_t temp[1]={0};
	Sensors_I2C_ReadRegister(0x68,0x75,2,temp);
	if(temp[0]!=0x68)
		assert_param(0);
	if(!mpu_init())
	{
		if(!mpu_set_sensors(INV_XYZ_GYRO|INV_XYZ_ACCEL)){}
		if(!mpu_configure_fifo(INV_XYZ_GYRO|INV_XYZ_ACCEL)){}
		if(!mpu_set_sample_rate(DEFAULT_MPU_HZ)){}
	    if(!dmp_load_motion_driver_firmware()) {}
	    if(!dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation))) {}
	    if(!dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
	        DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
	        DMP_FEATURE_GYRO_CAL)) {}
	    if(!dmp_set_fifo_rate(DEFAULT_MPU_HZ)) {}
	    run_self_test();
	    if(!mpu_set_dmp_state(1)) {}
	}
}

int Read_DMP(void)
{
	unsigned long sensor_timestamp;
	unsigned char more;
	long quat[4];

	MPU6050_GetData();

	MPU6050_CorrectData();

	//MPU6050_FilterData(1e-2, 0.5f);

	dmp_read_fifo((short*)gyro, (short*)accel, quat, &sensor_timestamp, (short*)&sensors, &more);		//读取DMP数据
	if (sensors & INV_WXYZ_QUAT)
	{
		q0 = quat[0] / q30;
		q1 = quat[1] / q30;
		q2 = quat[2] / q30;
		q3 = quat[3] / q30; 		//四元数
		Roll = asin(-2 * q1 * q3 + 2 * q0 * q2) * 57.3; 	//计算出横滚角
		Pitch = (atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1) * 57.3); // 计算出俯仰角
		Yaw = atan2(2 * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * 57.3;	 //计算出偏航角
		return 0;
	}
	else	return 1;
}
