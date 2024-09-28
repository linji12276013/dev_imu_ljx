#include "soft_iic.h"
#include "gpio.h"

void delay_us(int us)
{
	while(us--)
	{
		for(int i = 0; i <= 1; i++);
	}
}

/* STM32 I2C 快速模式 */
#define I2C_Speed              400000  //

/* 这个地址只要与STM32外挂的I2C器件地址不一样即可 */
#define I2Cx_OWN_ADDRESS7      0X0A   

static __IO uint32_t  I2CTimeout = I2CT_LONG_TIMEOUT;


//使用硬件IIC时不能与液晶屏共用，因为FSMC的NADV与IIC1的SDA 是同一个引脚，互相影响了

#ifdef HARD_IIC

#define ST_Sensors_I2C_WriteRegister  ST_Hard_Sensors_I2C_WriteRegister
#define ST_Sensors_I2C_ReadRegister ST_Hard_Sensors_I2C_ReadRegister

static uint8_t I2C_TIMEOUT_UserCallback(uint8_t errorCode);

#else

#define soft_iic_ready		0x00
#define soft_iic_bus_busy	0x01	
#define soft_iic_bus_error	0x02

#define set_iic_nack	    0x00 
#define set_iic_ack			0x01

#define ST_Sensors_I2C_WriteRegister  soft_iic_write
#define ST_Sensors_I2C_ReadRegister soft_iic_read

#endif

#ifdef HARD_IIC

/**
  * @brief  I2C 工作模式配置
  * @param  无
  * @retval 无
  */
static void I2C_Mode_Configu(void)
{
  I2C_InitTypeDef  I2C_InitStructure; 

  /* I2C 配置 */
  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	
	/* 高电平数据稳定，低电平数据变化 SCL 时钟线的占空比 */
  I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	
  I2C_InitStructure.I2C_OwnAddress1 =I2Cx_OWN_ADDRESS7; 
  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable ;
	
	/* I2C的寻址模式 */
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	
	/* 通信速率 */
  I2C_InitStructure.I2C_ClockSpeed = I2C_Speed;
  
	/* I2C1 初始化 */
  I2C_Init(SENSORS_I2Cx, &I2C_InitStructure);
  
	/* 使能 I2C1 */
  I2C_Cmd(SENSORS_I2Cx, ENABLE);   
}



/**
  * @brief  向IIC设备的寄存器连续写入数据
  * @param  Address: IIC设备地址
  * @param  RegisterAddr: 寄存器地址
  * @param  RegisterLen: 要写入数据的长度
  * @param  RegisterValue: 要指向写入数据的指针
  * @retval 0正常，非0异常
  */
unsigned long ST_Hard_Sensors_I2C_WriteRegister(unsigned char Address, unsigned char RegisterAddr, unsigned short RegisterLen, const unsigned char *RegisterValue)
{
	 uint32_t i=0;
	
	 /* Send STRAT condition */
	  I2C_GenerateSTART(SENSORS_I2Cx, ENABLE);

		I2CTimeout = I2CT_FLAG_TIMEOUT;


	  /* Test on EV5 and clear it */
	  while(!I2C_CheckEvent(SENSORS_I2Cx, I2C_EVENT_MASTER_MODE_SELECT))
	  {
	    if((I2CTimeout--) == 0) return I2C_TIMEOUT_UserCallback(0);
	  }

	  /* Send slave address for write */
	  I2C_Send7bitAddress(SENSORS_I2Cx, (Address<<1), I2C_Direction_Transmitter);

		I2CTimeout = I2CT_FLAG_TIMEOUT;
	  /* Test on EV6 and clear it */
	  while(!I2C_CheckEvent(SENSORS_I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
		{
	    if((I2CTimeout--) == 0) return I2C_TIMEOUT_UserCallback(1);
	  }

	  /* Send the slave's internal address to write to */
	  I2C_SendData(SENSORS_I2Cx, RegisterAddr);

		I2CTimeout = I2CT_FLAG_TIMEOUT;
	  /* Test on EV8 and clear it */
	  while(!I2C_CheckEvent(SENSORS_I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
		{
	    if((I2CTimeout--) == 0) return I2C_TIMEOUT_UserCallback(2);
	  }

	  /* Send the byte to be written */
	  for( i=0; i<(RegisterLen); i++)
	   {
		  	  I2CTimeout = I2CT_FLAG_TIMEOUT;
		  	  /* Test on EV8 and clear it */
		  	  while(!I2C_CheckEvent(SENSORS_I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
		  		{
		  	    if((I2CTimeout--) == 0) return I2C_TIMEOUT_UserCallback(3);
		  	  }

	     /* Prepare the register value to be sent */
	     I2C_SendData(SENSORS_I2Cx, RegisterValue[i]);
	   }

		I2CTimeout = I2CT_FLAG_TIMEOUT;

	  /* Test on EV8 and clear it */
	  while(!I2C_CheckEvent(SENSORS_I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
		{
	    if((I2CTimeout--) == 0) return I2C_TIMEOUT_UserCallback(4);
	  }

	  /* Send STOP condition */
	  I2C_GenerateSTOP(SENSORS_I2Cx, ENABLE);

		return 0; //正常返回0
}


/**
  * @brief  向IIC设备的寄存器连续读出数据
  * @param  Address: IIC设备地址
  * @param  RegisterAddr: 寄存器地址
  * @param  RegisterLen: 要读取的数据长度
  * @param  RegisterValue: 指向存储读出数据的指针
  * @retval 0正常，非0异常
  */
unsigned long ST_Hard_Sensors_I2C_ReadRegister(unsigned char Address, unsigned char RegisterAddr, unsigned short RegisterLen, unsigned char *RegisterValue)
{
	 I2CTimeout = I2CT_LONG_TIMEOUT;

	  while(I2C_GetFlagStatus(SENSORS_I2Cx, I2C_FLAG_BUSY)) // Added by Najoua 27/08/2008
	  {
	    if((I2CTimeout--) == 0) return I2C_TIMEOUT_UserCallback(5);
	   }

	  I2C_GenerateSTART(SENSORS_I2Cx, ENABLE);

		I2CTimeout = I2CT_FLAG_TIMEOUT;

	  /* Test on EV5 and clear it */
	  while(!I2C_CheckEvent(SENSORS_I2Cx, I2C_EVENT_MASTER_MODE_SELECT))
		{
	    if((I2CTimeout--) == 0) return I2C_TIMEOUT_UserCallback(6);
	   }

	  /* Send slave address for write */
	  I2C_Send7bitAddress(SENSORS_I2Cx, (Address<<1), I2C_Direction_Transmitter);

		I2CTimeout = I2CT_FLAG_TIMEOUT;

	  /* Test on EV6 and clear it */
	  while(!I2C_CheckEvent(SENSORS_I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
		{
	    if((I2CTimeout--) == 0) return I2C_TIMEOUT_UserCallback(7);
	   }

	  /* Clear EV6 by setting again the PE bit */
	  I2C_Cmd(SENSORS_I2Cx, ENABLE);

	  /* Send the slave's internal address to write to */
	  I2C_SendData(SENSORS_I2Cx, RegisterAddr);

		I2CTimeout = I2CT_FLAG_TIMEOUT;

	  /* Test on EV8 and clear it */
	  while(!I2C_CheckEvent(SENSORS_I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
		{
	    if((I2CTimeout--) == 0) return I2C_TIMEOUT_UserCallback(8);
	   }

	  /* Send STRAT condition a second time */
	  I2C_GenerateSTART(SENSORS_I2Cx, ENABLE);

		I2CTimeout = I2CT_FLAG_TIMEOUT;
	  /* Test on EV5 and clear it */
	  while(!I2C_CheckEvent(SENSORS_I2Cx, I2C_EVENT_MASTER_MODE_SELECT))
		{
	    if((I2CTimeout--) == 0) return I2C_TIMEOUT_UserCallback(9);
	   }

	  /* Send slave address for read */
	  I2C_Send7bitAddress(SENSORS_I2Cx, (Address<<1), I2C_Direction_Receiver);

		I2CTimeout = I2CT_FLAG_TIMEOUT;

	  /* Test on EV6 and clear it */
	  while(!I2C_CheckEvent(SENSORS_I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
		{
	    if((I2CTimeout--) == 0) return I2C_TIMEOUT_UserCallback(10);
	   }

	  /* While there is data to be read */
	  while(RegisterLen)
	  {
	    if(RegisterLen == 1)
	    {
	      /* Disable Acknowledgement */
	      I2C_AcknowledgeConfig(SENSORS_I2Cx, DISABLE);

	      /* Send STOP Condition */
	      I2C_GenerateSTOP(SENSORS_I2Cx, ENABLE);
	    }

			I2CTimeout = I2CT_LONG_TIMEOUT;
			while(!I2C_CheckEvent(SENSORS_I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED))
			{
				if((I2CTimeout--) == 0) 
				{

						return I2C_TIMEOUT_UserCallback(10);
				}
			 }
			
	    {
	      /* Read a byte from the slave */
	      *RegisterValue = I2C_ReceiveData(SENSORS_I2Cx);

	      /* Point to the next location where the byte read will be saved */
	      RegisterValue++;

	      /* Decrement the read bytes counter */
	      RegisterLen--;
	    }

	  }

	  /* Enable Acknowledgement to be ready for another reception */
	  I2C_AcknowledgeConfig(SENSORS_I2Cx, ENABLE);

		return 0; //正常，返回0
}


/**
  * @brief  Basic management of the timeout situation.
  * @param  None.
  * @retval None.
  */
static  uint8_t I2C_TIMEOUT_UserCallback(uint8_t errorCode)
{
	/*重置IIC*/
  I2C_GenerateSTOP(SENSORS_I2Cx, ENABLE);
  I2C_SoftwareResetCmd(SENSORS_I2Cx, ENABLE);
  I2C_SoftwareResetCmd(SENSORS_I2Cx, DISABLE);
	
	I2C_Bus_Init();
	
  /* Block communication and all processes */
  MPU_ERROR("I2C Timeout error! error code = %d",errorCode);
  
  return 1;
}

#endif //endof #ifdef HARD_IIC

static unsigned short RETRY_IN_MLSEC  = 55;

/**
  * @brief  设置iic重试时间
  * @param  ml_sec：重试的时间，单位毫秒
  * @retval 重试的时间，单位毫秒
  */
void set_iic_retry(unsigned short ml_sec)
{
	RETRY_IN_MLSEC = ml_sec;
}

/**
  * @brief  获取设置的iic重试时间
  * @param  none
  * @retval none
  */
unsigned short get_iic_retry(void)
{
	return RETRY_IN_MLSEC;
}

/************************软件IIC驱动函数****************************************/

#ifndef HARD_IIC

/*	__     ______	   _
SCL	  \___/		 \____/
	_    __________	  __
SDA  \__/		   \_/
	start	trans	stop	*/

#define scl_high	gpio_high(soft_iic_struct->scl_port, soft_iic_struct->scl_pin)
#define scl_low		gpio_low(soft_iic_struct->scl_port, soft_iic_struct->scl_pin)
#define sda_high	gpio_high(soft_iic_struct->sda_port, soft_iic_struct->sda_pin)
#define sda_low		gpio_low(soft_iic_struct->sda_port, soft_iic_struct->sda_pin)
#define sda_state	HAL_GPIO_ReadPin(soft_iic_struct->sda_port, soft_iic_struct->sda_pin)	

static void soft_iic_init(soft_iic_init_struct *soft_iic_struct)
{
//	assert_param(NULL != soft_iic_struct);
//	assert_param(soft_iic_struct->scl_pin != soft_iic_struct->sda_pin);

	gpio_init(soft_iic_struct->scl_port, soft_iic_struct->scl_pin, GPIO_MODE_OUTPUT_OD);
	gpio_init(soft_iic_struct->sda_port, soft_iic_struct->sda_pin, GPIO_MODE_OUTPUT_OD);
//	soft_iic_struct->delay = 1;
//	soft_iic_struct->nop = 1;//原来是10us
	scl_high;
	sda_high;
	delay_us(soft_iic_struct->delay);
}

static uint8_t soft_iic_start(soft_iic_init_struct *soft_iic_struct)
{ 
	sda_high;
 	delay_us(soft_iic_struct->nop);

 	scl_high;
 	delay_us(soft_iic_struct->nop);    

 	if (!sda_state) return soft_iic_bus_busy;

 	sda_low;
 	delay_us(soft_iic_struct->nop);
 
 	scl_low;  
 	delay_us(soft_iic_struct->nop);

 	if (sda_state) return soft_iic_bus_error;

 	return soft_iic_ready;
}

static void soft_iic_stop(soft_iic_init_struct *soft_iic_struct)
{
 	sda_low; 
 	delay_us(soft_iic_struct->nop);

 	scl_high; 
 	delay_us(soft_iic_struct->nop);    

 	sda_high;
 	delay_us(soft_iic_struct->nop);
}

static void soft_iic_send_ack(soft_iic_init_struct *soft_iic_struct)
{
 	sda_low;
 	delay_us(soft_iic_struct->nop);
 	scl_high;
 	delay_us(soft_iic_struct->nop);
 	scl_low; 
 	delay_us(soft_iic_struct->nop);  
}

static void soft_iic_send_nack(soft_iic_init_struct *soft_iic_struct)
{
	sda_high;
	delay_us(soft_iic_struct->nop);
	scl_high;
	delay_us(soft_iic_struct->nop);
	scl_low; 
	delay_us(soft_iic_struct->nop);  
}

/**
  * @brief  等待应答信号到来
  * @retval 返回值：1，接收应答失败
  *					0，接收应答成功
  */
static uint8_t soft_iic_wait_ack(soft_iic_init_struct *soft_iic_struct)
{
	uint8_t ucErrTime = 0;

	sda_high;
	delay_us(soft_iic_struct->nop);	   
	scl_high;
	delay_us(soft_iic_struct->nop);	 

	while (sda_state)
	{
		ucErrTime++;
		if (ucErrTime > 250)
		{
			soft_iic_stop(soft_iic_struct);
			return soft_iic_bus_error;
		}
	}
	scl_low;//时钟输出0 	   
	return 0;  
} 

static uint8_t soft_iic_send_byte(soft_iic_init_struct *soft_iic_struct, uint8_t soft_i2c_data)
{
 	uint8_t i;
 	
	scl_low;
 	for (i = 0; i < 8; i++)
 	{  
  		if (soft_i2c_data & 0x80) 
			sda_high;
   		else 
			sda_low;

  		soft_i2c_data <<= 1;
  		delay_us(soft_iic_struct->nop);

  		scl_high;
  		delay_us(soft_iic_struct->nop);
  		scl_low;
  		delay_us(soft_iic_struct->nop); 
 	}
	return soft_iic_wait_ack(soft_iic_struct);  
}

static uint8_t soft_iic_read_byte(soft_iic_init_struct *soft_iic_struct)
{
	uint8_t i, soft_i2c_data;

 	sda_high;
 	scl_low; 
 	soft_i2c_data = 0;

 	for (i = 0; i < 8; i++)
 	{
  		scl_high;
  		delay_us(soft_iic_struct->nop); 
  		soft_i2c_data <<= 1;

  		if (sda_state)	
			soft_i2c_data |= 0x01; 

  		scl_low;  
  		delay_us(soft_iic_struct->nop);         
 	}
	soft_iic_send_nack(soft_iic_struct);
 	return soft_i2c_data;
}

static uint8_t soft_iic_read_byte_with_ack(soft_iic_init_struct *soft_iic_struct)
{
	uint8_t i, soft_i2c_data;

 	sda_high;
 	scl_low; 
 	soft_i2c_data = 0;

 	for (i = 0; i < 8; i++)
 	{
  		scl_high;
  		delay_us(soft_iic_struct->nop); 
  		soft_i2c_data <<= 1;

  		if (sda_state)	soft_i2c_data |= 0x01; 
  
  		scl_low;  
  		delay_us(soft_iic_struct->nop);         
 	}
	soft_iic_send_ack(soft_iic_struct);
 	return soft_i2c_data;
}

uint8_t soft_iic_write(soft_iic_init_struct *soft_iic_struct, 
					   uint8_t soft_dev_addr, 
					   uint8_t soft_reg_addr, 
					   uint8_t soft_i2c_len,
					   unsigned char *soft_i2c_data_buf)
{
	uint8_t i, result = 0;
	soft_iic_start(soft_iic_struct);

	if (soft_dev_addr != NULL)
		result = soft_iic_send_byte(soft_iic_struct, soft_dev_addr << 1 | I2C_Direction_Transmitter);	
	if (result != 0) return result;

	if (soft_reg_addr != NULL)
		result = soft_iic_send_byte(soft_iic_struct, soft_reg_addr);  
	if (result != 0) return result;

	for (i = 0; i < soft_i2c_len; i++) 
	{
		if (NULL != soft_i2c_data_buf)
			result = soft_iic_send_byte(soft_iic_struct, soft_i2c_data_buf[i]);
		else
			result = soft_iic_send_byte(soft_iic_struct, 0);
		if (result != 0) return result;
	}
	soft_iic_stop(soft_iic_struct);
	return 0;
}

uint8_t soft_iic_read(soft_iic_init_struct *soft_iic_struct, 
					  uint8_t soft_dev_addr, 
					  uint8_t soft_reg_addr, 
					  uint8_t soft_i2c_len, 
					  unsigned char *soft_i2c_data_buf)
{
	uint8_t result;

	soft_iic_start(soft_iic_struct);
	result  = soft_iic_send_byte(soft_iic_struct, soft_dev_addr << 1 | I2C_Direction_Transmitter);			
	if (result != 0) return result;

	result = soft_iic_send_byte(soft_iic_struct, soft_reg_addr); 
	if (result != 0) return result;

	soft_iic_start(soft_iic_struct);
	result = soft_iic_send_byte(soft_iic_struct, soft_dev_addr << 1 | I2C_Direction_Receiver);
	if (result != 0) return result;

    while (soft_i2c_len)
	{
		if (soft_i2c_len==1) *soft_i2c_data_buf = soft_iic_read_byte(soft_iic_struct);  
		else *soft_i2c_data_buf = soft_iic_read_byte_with_ack(soft_iic_struct);
		soft_i2c_data_buf++;
		soft_i2c_len--;
    }
	soft_iic_stop(soft_iic_struct);
    return 0;
}

#endif //endof #ifndef HARD_IIC

/**
  * @brief  I2C 外设初始化
  * @param  无
  * @retval 无
  */
void I2C_Bus_Init(void *iic_struct)
{
	set_iic_retry(5);
#ifdef HARD_IIC
    I2C_GPIO_Config(); 
    I2C_Mode_Configu();	
#else
	soft_iic_init(iic_struct);
#endif
}
