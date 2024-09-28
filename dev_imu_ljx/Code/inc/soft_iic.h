#ifndef __SOFT_IIC_H
#define __SOFT_IIC_H

#include "main.h"
#include "usart.h"

typedef struct
{
	GPIO_TypeDef *scl_port;
	uint32_t scl_pin;
	GPIO_TypeDef *sda_port;
	uint32_t sda_pin;
	uint8_t	 addr;		//器件地址
	uint32_t delay, nop;
}soft_iic_init_struct;

//#define HARD_IIC 

/**************************I2C参数定义，I2C1或I2C2********************************/
#define             SENSORS_I2Cx                                I2C1
#define             SENSORS_I2C_APBxClock_FUN                   RCC_APB1PeriphClockCmd
#define             SENSORS_I2C_CLK                             RCC_APB1Periph_I2C1
#define             SENSORS_I2C_GPIO_APBxClock_FUN              RCC_APB2PeriphClockCmd
#define             SENSORS_I2C_GPIO_CLK                        RCC_APB2Periph_GPIOB     
#define             SENSORS_I2C_SCL_PORT                        GPIOB   
#define             SENSORS_I2C_SCL_PIN                         GPIO_Pin_6
#define             SENSORS_I2C_SDA_PORT                        GPIOB 
#define             SENSORS_I2C_SDA_PIN                         GPIO_Pin_7


/*等待超时时间*/
#define I2CT_FLAG_TIMEOUT         ((uint32_t)0x1000)
#define I2CT_LONG_TIMEOUT         ((uint32_t)(10 * I2CT_FLAG_TIMEOUT))

//这两个是标准库硬IIC宏定义
#define I2C_Direction_Transmitter	0x00
#define I2C_Direction_Receiver		0x01

/*信息输出*/
#define MPU_DEBUG_ON         0
#define MPU_DEBUG_FUNC_ON    0

#define MPU_INFO(fmt,arg...)           printf("<<-MPU-INFO->> "fmt"\n",##arg)
#define MPU_ERROR(fmt,arg...)          printf("<<-MPU-ERROR->> "fmt"\n",##arg)
#define MPU_DEBUG(fmt,arg...)          do{\
                                          if(MPU_DEBUG_ON)\
                                          printf("<<-MPU-DEBUG->> [%d]"fmt"\n",__LINE__, ##arg);\
                                          }while(0)

#define MPU_DEBUG_FUNC()               do{\
                                         if(MPU_DEBUG_FUNC_ON)\
                                         printf("<<-MPU-FUNC->> Func:%s@Line:%d\n",__func__,__LINE__);\
                                       }while(0)

void I2C_Bus_Init(void *iic_struct);
void set_iic_retry(unsigned short ml_sec);
unsigned short get_iic_retry(void);									   
uint8_t soft_iic_write(soft_iic_init_struct *soft_iic_struct, 
						   uint8_t soft_dev_addr, 
						   uint8_t soft_reg_addr, 
						   uint8_t soft_i2c_len,
						   unsigned char *soft_i2c_data_buf);
uint8_t soft_iic_read(soft_iic_init_struct *soft_iic_struct, 
						  uint8_t soft_dev_addr, 
						  uint8_t soft_reg_addr, 
						  uint8_t soft_i2c_len, 
						  unsigned char *soft_i2c_data_buf);

#endif
