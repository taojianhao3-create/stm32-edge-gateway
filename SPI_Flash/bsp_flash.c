#include "stm32f4xx.h"
#include "bsp_flash.h"
#include "spi.h"

static __IO uint32_t  SPITimeout = SPIT_LONG_TIMEOUT;   
static uint16_t SPI_TIMEOUT_UserCallback(uint8_t errorCode);




void SPI_FLASH_Init(void)
{
    // 1. 初始化 CS 引脚（PB12）
    __HAL_RCC_GPIOB_CLK_ENABLE();          // 确保 GPIOB 时钟开启
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = FLASH_CS_PIN;    // PB12
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(FLASH_CS_GPIO_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(FLASH_CS_GPIO_PORT, FLASH_CS_PIN, GPIO_PIN_SET); // CS 拉高

    // 2. 使能 SPI2 外设（关键！）
    __HAL_SPI_ENABLE(&hspi2);
}

/*4KB 扇区擦除*/
void SPI_FLASH_SectorErase(uint32_t SectorAddr)
{
  SPI_FLASH_WriteEnable();
  SPI_FLASH_WaitForWriteEnd();

  SPI_FLASH_CS_LOW();
  SPI_FLASH_SendByte(W25X_SectorErase);
  SPI_FLASH_SendByte((SectorAddr & 0xFF0000) >> 16);
  SPI_FLASH_SendByte((SectorAddr & 0xFF00) >> 8);
  SPI_FLASH_SendByte(SectorAddr & 0xFF);
  SPI_FLASH_CS_HIGH();

  SPI_FLASH_WaitForWriteEnd();
}

void SPI_FLASH_BulkErase(void)
{
  SPI_FLASH_WriteEnable();

  SPI_FLASH_CS_LOW();
  SPI_FLASH_SendByte(W25X_ChipErase);
  SPI_FLASH_CS_HIGH();

  SPI_FLASH_WaitForWriteEnd();
}

//单页写入（一页 256 字节）
void SPI_FLASH_PageWrite(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite)
{
  SPI_FLASH_WriteEnable();			//发送写使能指令 0x06

  SPI_FLASH_CS_LOW();
  SPI_FLASH_SendByte(W25X_PageProgram);
  SPI_FLASH_SendByte((WriteAddr & 0xFF0000) >> 16);		//W25Q 是 24 位地址，分 3 字节依次发送：
  SPI_FLASH_SendByte((WriteAddr & 0xFF00) >> 8);
  SPI_FLASH_SendByte(WriteAddr & 0xFF);

  if(NumByteToWrite > SPI_FLASH_PerWritePageSize)
  {
     NumByteToWrite = SPI_FLASH_PerWritePageSize;
     FLASH_ERROR("SPI_FLASH_PageWrite too large!");
  }

  while (NumByteToWrite--)
  {
    SPI_FLASH_SendByte(*pBuffer);
    pBuffer++;
  }

  SPI_FLASH_CS_HIGH();
  SPI_FLASH_WaitForWriteEnd();
}

/*自动分页批量写入（上层最常用）
	pBuffer待写入数据
	WriteAddr：待写入的 Flash 绝对起始地址
	NumByteToWrite：本次总共要写入的字节数长度*/
void SPI_FLASH_BufferWrite(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite)
{
  uint8_t NumOfPage = 0, NumOfSingle = 0, Addr = 0, count = 0, temp = 0;

  Addr = WriteAddr % SPI_FLASH_PageSize;
  count = SPI_FLASH_PageSize - Addr;				//当前页剩余可写字节数
  NumOfPage =  NumByteToWrite / SPI_FLASH_PageSize;		//完整 256 字节的页数
  NumOfSingle = NumByteToWrite % SPI_FLASH_PageSize;		//最后不足 256 字节的零散字节数

  if (Addr == 0) 		//起始地址刚好对齐 256 字节页边界
  {
    if (NumOfPage == 0) 			// 场景1：总长度不足1页
    {
      SPI_FLASH_PageWrite(pBuffer, WriteAddr, NumByteToWrite);
    }
    else 
    {
			 // 循环写所有完整页
      while (NumOfPage--)
      {
        SPI_FLASH_PageWrite(pBuffer, WriteAddr, SPI_FLASH_PageSize);
        WriteAddr +=  SPI_FLASH_PageSize;
        pBuffer += SPI_FLASH_PageSize;
      }
			 // 写最后不足1页的零散数据
      SPI_FLASH_PageWrite(pBuffer, WriteAddr, NumOfSingle);
    }
  }
  else 
  {
    if (NumOfPage == 0) 			//字节数不到1页
    {
      if (NumOfSingle > count) 		 // 情况A：数据长度 > 当前页剩余空间 → 跨当前页，分两次写
      {
        temp = NumOfSingle - count;
        SPI_FLASH_PageWrite(pBuffer, WriteAddr, count); // 第一步：填满当前页剩余空间
        WriteAddr +=  count;
        pBuffer += count;
        SPI_FLASH_PageWrite(pBuffer, WriteAddr, temp);	 // 第二步：写到下一页开头
      }
      else 
      {				
        SPI_FLASH_PageWrite(pBuffer, WriteAddr, NumByteToWrite); // 情况B：数据长度 ≤ 当前页剩余空间 → 一页写完   后续没有数据地址就不需要再加了
      }
    }
    else 
    {
      NumByteToWrite -= count; // 扣除当前页剩余字节，重新计算页数、零散数
      NumOfPage =  NumByteToWrite / SPI_FLASH_PageSize;		//完整256字节的页数
      NumOfSingle = NumByteToWrite % SPI_FLASH_PageSize;	//

      SPI_FLASH_PageWrite(pBuffer, WriteAddr, count); // 第一步：先把当前页剩余空间写满
      WriteAddr +=  count;
      pBuffer += count;

      while (NumOfPage--)	  // 第二步：循环写入所有完整256字节页
      {
        SPI_FLASH_PageWrite(pBuffer, WriteAddr, SPI_FLASH_PageSize);
        WriteAddr +=  SPI_FLASH_PageSize;
        pBuffer += SPI_FLASH_PageSize;
      }

      if (NumOfSingle != 0)		 // 第三步：写入最后不足一页的零散数据
      {
        SPI_FLASH_PageWrite(pBuffer, WriteAddr, NumOfSingle);
      }
    }
  }
}

/*批量读取（上层最常用）*/
/*
写：受 256 字节页限制，必须分页拆分 → BufferWrite 拆分调用 PageWrite；
读：无任何分页限制，一次发地址连续读完 → BufferRead 单层实现，不需要拆分。
*/
void SPI_FLASH_BufferRead(uint8_t* pBuffer, uint32_t ReadAddr, uint16_t NumByteToRead)
{
  // 1. 片选拉低，选中Flash
  SPI_FLASH_CS_LOW();

  // 2. 发送读指令 + 24位物理地址
  SPI_FLASH_SendByte(W25X_ReadData);        // 读指令 0x03
  SPI_FLASH_SendByte((ReadAddr & 0xFF0000) >> 16); // 地址高8位
  SPI_FLASH_SendByte((ReadAddr& 0xFF00) >> 8);     // 地址中8位
  SPI_FLASH_SendByte(ReadAddr & 0xFF);             // 地址低8位

  // 3. 循环读取指定字节数
  while (NumByteToRead--)
  {
    // 发送哑字节 0xFF，同时接收Flash返回的数据
    *pBuffer = SPI_FLASH_SendByte(Dummy_Byte);
     pBuffer++;
  }

  // 4. 片选拉高，释放Flash
  SPI_FLASH_CS_HIGH();
}

uint32_t SPI_FLASH_ReadID(void)
{
  uint32_t Temp = 0, Temp0 = 0, Temp1 = 0, Temp2 = 0;

  SPI_FLASH_CS_LOW();
  SPI_FLASH_SendByte(W25X_JedecDeviceID);

  Temp0 = SPI_FLASH_SendByte(Dummy_Byte);
  Temp1 = SPI_FLASH_SendByte(Dummy_Byte);
  Temp2 = SPI_FLASH_SendByte(Dummy_Byte);

  SPI_FLASH_CS_HIGH();
  Temp = (Temp0 << 16) | (Temp1 << 8) | Temp2;
  return Temp;
}

uint32_t SPI_FLASH_ReadDeviceID(void)
{
  uint32_t Temp = 0;
  SPI_FLASH_CS_LOW();
  SPI_FLASH_SendByte(W25X_DeviceID);
  SPI_FLASH_SendByte(Dummy_Byte);
  SPI_FLASH_SendByte(Dummy_Byte);
  SPI_FLASH_SendByte(Dummy_Byte);
  Temp = SPI_FLASH_SendByte(Dummy_Byte);
  SPI_FLASH_CS_HIGH();
  return Temp;
}

void SPI_FLASH_StartReadSequence(uint32_t ReadAddr)
{
  SPI_FLASH_CS_LOW();
  SPI_FLASH_SendByte(W25X_ReadData);
  SPI_FLASH_SendByte((ReadAddr & 0xFF0000) >> 16);
  SPI_FLASH_SendByte((ReadAddr& 0xFF00) >> 8);
  SPI_FLASH_SendByte(ReadAddr & 0xFF);
}

uint8_t SPI_FLASH_ReadByte(void)
{
  return SPI_FLASH_SendByte(Dummy_Byte);
}

uint8_t SPI_FLASH_SendByte(uint8_t byte)
{
  SPITimeout = SPIT_FLAG_TIMEOUT;
  while (__HAL_SPI_GET_FLAG(&SpiHandle, SPI_FLAG_TXE) == RESET)
  {
    if((SPITimeout--) == 0) return SPI_TIMEOUT_UserCallback(0);
  }

  WRITE_REG(SpiHandle.Instance->DR, byte);
  SPITimeout = SPIT_FLAG_TIMEOUT;

  while (__HAL_SPI_GET_FLAG(&SpiHandle, SPI_FLAG_RXNE) == RESET)
  {
    if((SPITimeout--) == 0) return SPI_TIMEOUT_UserCallback(1);
  }

  return READ_REG(SpiHandle.Instance->DR);
}

uint16_t SPI_FLASH_SendHalfWord(uint16_t HalfWord)
{
  SPITimeout = SPIT_FLAG_TIMEOUT;
  while (__HAL_SPI_GET_FLAG(&SpiHandle, SPI_FLAG_TXE) == RESET)
  {
    if((SPITimeout--) == 0) return SPI_TIMEOUT_UserCallback(2);
  }

  WRITE_REG(SpiHandle.Instance->DR, HalfWord);
  SPITimeout = SPIT_FLAG_TIMEOUT;

  while (__HAL_SPI_GET_FLAG(&SpiHandle, SPI_FLAG_RXNE) == RESET)
  {
    if((SPITimeout--) == 0) return SPI_TIMEOUT_UserCallback(3);
  }

  return READ_REG(SpiHandle.Instance->DR);
}

void SPI_FLASH_WriteEnable(void)
{
  SPI_FLASH_CS_LOW();
  SPI_FLASH_SendByte(W25X_WriteEnable);
  SPI_FLASH_CS_HIGH();
}

void SPI_FLASH_WaitForWriteEnd(void)
{
  uint8_t FLASH_Status = 0;

  SPI_FLASH_CS_LOW();
  SPI_FLASH_SendByte(W25X_ReadStatusReg);

  SPITimeout = SPIT_FLAG_TIMEOUT;
  do
  {
    FLASH_Status = SPI_FLASH_SendByte(Dummy_Byte);
    if((SPITimeout--) == 0) 
    {
      SPI_TIMEOUT_UserCallback(4);
      return;
    }
  } while ((FLASH_Status & WIP_Flag) == SET);

  SPI_FLASH_CS_HIGH();
}

void SPI_Flash_PowerDown(void)   
{ 
  SPI_FLASH_CS_LOW();
  SPI_FLASH_SendByte(W25X_PowerDown);
  SPI_FLASH_CS_HIGH();
}   

void SPI_Flash_WAKEUP(void)   
{
  SPI_FLASH_CS_LOW();
  SPI_FLASH_SendByte(W25X_ReleasePowerDown);
  SPI_FLASH_CS_HIGH();
}   

static uint16_t SPI_TIMEOUT_UserCallback(uint8_t errorCode)
{
  FLASH_ERROR("SPI 等待超时!errorCode = %d",errorCode);
  return 0;
}