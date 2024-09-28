#include "internal_flash.h"


/*准备写入的测试数据*/
#define DATA_32                 ((uint32_t)0x87654321)
size_t Data[1024];

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* 要擦除内部FLASH的起始地址 */
#define FLASH_USER_START_ADDR   ADDR_FLASH_SECTOR_5   
/* 要擦除内部FLASH的结束地址 */
#define FLASH_USER_END_ADDR     ADDR_FLASH_SECTOR_7  


static uint32_t GetSector(uint32_t Address);

/**
  * @brief  InternalFlash_Test,对内部FLASH进行读写测试
  * @param  None
  * @retval None
  */

void flash_unlock(void)
{
	/* 使能访问FLASH控制寄存器 */
	HAL_FLASH_Unlock();
}
void flash_lock(void)
{
	/* 给FLASH上锁，防止内容被篡改*/
	HAL_FLASH_Lock();
}
void flash_erase(void)
{
	/*要擦除的起始扇区(包含)及结束扇区(不包含)，如8-12，表示擦除8、9、10、11扇区*/
	uint32_t FirstSector = 0;
	uint32_t NbOfSectors = 0;

	uint32_t SECTORError = 0;

	static FLASH_EraseInitTypeDef EraseInitStruct;
	/* FLASH 解锁 ********************************/
	HAL_FLASH_Unlock();

	FirstSector = GetSector(FLASH_USER_START_ADDR);
	NbOfSectors = GetSector(FLASH_USER_END_ADDR)- FirstSector + 1;

	/* 擦除用户区域 (用户区域指程序本身没有使用的空间，可以自定义)**/
	/* Fill EraseInit structure*/
	EraseInitStruct.TypeErase     = FLASH_TYPEERASE_SECTORS;
	EraseInitStruct.VoltageRange  = FLASH_VOLTAGE_RANGE_3;/* 以“字”的大小进行操作 */ 
	EraseInitStruct.Sector        = FirstSector;
	EraseInitStruct.NbSectors     = NbOfSectors;
	/* 开始擦除操作 */
	if (HAL_FLASHEx_Erase(&EraseInitStruct, &SECTORError) != HAL_OK)
	{
		/*擦除出错，返回，实际应用中可加入处理 */
		assert_param(0);
	}
	HAL_FLASH_Lock();
}

uint32_t STMFLASH_ReadWord(uint32_t faddr)
{
    return *(__IO uint32_t *)faddr;
}

void STMFLASH_Write(uint32_t WriteAddr, uint32_t *pBuffer, uint32_t Num)
{
    FLASH_EraseInitTypeDef FlashEraseInit;
    HAL_StatusTypeDef FlashStatus = HAL_OK;
    uint32_t SectorError = 0;
    uint32_t addrx = 0;
    uint32_t endaddr = 0;
    if (WriteAddr < FLASH_BASE || WriteAddr % 4)
        return; //非法地址

    HAL_FLASH_Unlock();            //解锁
    addrx = WriteAddr;             //写入的起始地址
    endaddr = WriteAddr + Num * 4; //写入的结束地址

    if (addrx < 0X080C1000)
    {
        while (addrx < endaddr)
        {
            if (STMFLASH_ReadWord(addrx) != 0XFFFFFFFF)
            {
                FlashEraseInit.TypeErase = FLASH_TYPEERASE_SECTORS;     //擦除类型，扇区擦除
                FlashEraseInit.Sector = GetSector(addrx); //要擦除的扇区
                FlashEraseInit.NbSectors = 1;                           //一次只擦除一个扇区
                FlashEraseInit.VoltageRange = FLASH_VOLTAGE_RANGE_3;    //电压范围，VCC=2.7~3.6V之间!!
                if (HAL_FLASHEx_Erase(&FlashEraseInit, &SectorError) != HAL_OK)
                {
                    break; //发生错误了
                }
            }
            else
                addrx += 4;
            FLASH_WaitForLastOperation(FLASH_WAITETIME); //等待上次操作完成
        }
    }
    FlashStatus = FLASH_WaitForLastOperation(FLASH_WAITETIME); //等待上次操作完成
    if (FlashStatus == HAL_OK)
    {
        while (WriteAddr < endaddr) //写数据
        {
            if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, WriteAddr, *pBuffer) != HAL_OK) //写入数据
            {
                break; //写入异常
            }
            WriteAddr += 4;
            pBuffer++;
        }
    }
    HAL_FLASH_Lock(); //上锁
}

static uint32_t Address = FLASH_USER_START_ADDR;
void flash_write_double(double data)
{
	HAL_FLASH_Unlock();
	if (Address < FLASH_USER_END_ADDR)
	{
		FLASH->SR = 0;
		if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, Address, data) == HAL_OK)
		{
		    Address = Address + 4;
		}
	}
	HAL_FLASH_Lock();
}

void flash_read(double *Data)
{
	uint32_t Address = FLASH_USER_START_ADDR;

	while (Address < FLASH_USER_END_ADDR)
	{
		*Data = *(__IO uint32_t*)Address;
		printf("%lf\n", *Data);
		HAL_Delay(500);
		Data++;
		Address += sizeof(double);
	}
}


/**
  * @brief  根据输入的地址给出它所在的sector
  *					例如：
						uwStartSector = GetSector(FLASH_USER_START_ADDR);
						uwEndSector = GetSector(FLASH_USER_END_ADDR);	
  * @param  Address：地址
  * @retval 地址所在的sector
  */
static uint32_t GetSector(uint32_t Address)
{
  uint32_t sector = 0;
  
  if((Address < ADDR_FLASH_SECTOR_1) && (Address >= ADDR_FLASH_SECTOR_0))
  {
    sector = FLASH_SECTOR_0;  
  }
  else if((Address < ADDR_FLASH_SECTOR_2) && (Address >= ADDR_FLASH_SECTOR_1))
  {
    sector = FLASH_SECTOR_1;  
  }
  else if((Address < ADDR_FLASH_SECTOR_3) && (Address >= ADDR_FLASH_SECTOR_2))
  {
    sector = FLASH_SECTOR_2;  
  }
  else if((Address < ADDR_FLASH_SECTOR_4) && (Address >= ADDR_FLASH_SECTOR_3))
  {
    sector = FLASH_SECTOR_3;  
  }
  else if((Address < ADDR_FLASH_SECTOR_5) && (Address >= ADDR_FLASH_SECTOR_4))
  {
    sector = FLASH_SECTOR_4;  
  }
  else if((Address < ADDR_FLASH_SECTOR_6) && (Address >= ADDR_FLASH_SECTOR_5))
  {
    sector = FLASH_SECTOR_5;  
  }
  else if((Address < ADDR_FLASH_SECTOR_7) && (Address >= ADDR_FLASH_SECTOR_6))
  {
    sector = FLASH_SECTOR_6;  
  }
  else/*(Address < FLASH_END_ADDR) && (Address >= ADDR_FLASH_SECTOR_23))*/
  {
    sector = FLASH_SECTOR_7;  
  }
  return sector;
}

