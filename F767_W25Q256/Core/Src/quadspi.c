/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    quadspi.c
  * @brief   This file provides code for the configuration
  *          of the QUADSPI instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "quadspi.h"

/* USER CODE BEGIN 0 */
#include "w25q256.h"

static uint8_t QSPI_WriteEnable(void);
static uint8_t QSPI_AutoPollingMemReady(void);
static uint8_t QSPI_Configuration(void);
static uint8_t QSPI_ResetChip(void);
/* USER CODE END 0 */

QSPI_HandleTypeDef hqspi;

/* QUADSPI init function */
void MX_QUADSPI_Init(void)
{

  /* USER CODE BEGIN QUADSPI_Init 0 */

  /* USER CODE END QUADSPI_Init 0 */

  /* USER CODE BEGIN QUADSPI_Init 1 */

  /* USER CODE END QUADSPI_Init 1 */
  hqspi.Instance = QUADSPI;
  hqspi.Init.ClockPrescaler = 1;
  hqspi.Init.FifoThreshold = 32;
  hqspi.Init.SampleShifting = QSPI_SAMPLE_SHIFTING_HALFCYCLE;
  hqspi.Init.FlashSize = 24;
  hqspi.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_5_CYCLE;
  hqspi.Init.ClockMode = QSPI_CLOCK_MODE_0;
  hqspi.Init.FlashID = QSPI_FLASH_ID_1;
  hqspi.Init.DualFlash = QSPI_DUALFLASH_DISABLE;
  if (HAL_QSPI_Init(&hqspi) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN QUADSPI_Init 2 */

  /* USER CODE END QUADSPI_Init 2 */

}

void HAL_QSPI_MspInit(QSPI_HandleTypeDef* qspiHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(qspiHandle->Instance==QUADSPI)
  {
  /* USER CODE BEGIN QUADSPI_MspInit 0 */

  /* USER CODE END QUADSPI_MspInit 0 */
    /* QUADSPI clock enable */
    __HAL_RCC_QSPI_CLK_ENABLE();

    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**QUADSPI GPIO Configuration
    PF6     ------> QUADSPI_BK1_IO3
    PF7     ------> QUADSPI_BK1_IO2
    PF8     ------> QUADSPI_BK1_IO0
    PF9     ------> QUADSPI_BK1_IO1
    PB2     ------> QUADSPI_CLK
    PB6     ------> QUADSPI_BK1_NCS
    */
    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_QUADSPI;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF10_QUADSPI;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_QUADSPI;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF10_QUADSPI;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN QUADSPI_MspInit 1 */

  /* USER CODE END QUADSPI_MspInit 1 */
  }
}

void HAL_QSPI_MspDeInit(QSPI_HandleTypeDef* qspiHandle)
{

  if(qspiHandle->Instance==QUADSPI)
  {
  /* USER CODE BEGIN QUADSPI_MspDeInit 0 */

  /* USER CODE END QUADSPI_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_QSPI_CLK_DISABLE();

    /**QUADSPI GPIO Configuration
    PF6     ------> QUADSPI_BK1_IO3
    PF7     ------> QUADSPI_BK1_IO2
    PF8     ------> QUADSPI_BK1_IO0
    PF9     ------> QUADSPI_BK1_IO1
    PB2     ------> QUADSPI_CLK
    PB6     ------> QUADSPI_BK1_NCS
    */
    HAL_GPIO_DeInit(GPIOF, GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9);

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_2|GPIO_PIN_6);

    /* QUADSPI interrupt Deinit */
    HAL_NVIC_DisableIRQ(QUADSPI_IRQn);
  /* USER CODE BEGIN QUADSPI_MspDeInit 1 */

  /* USER CODE END QUADSPI_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
//QSPI�?��?命令
//instruction:�?�?��?的指令
//address:�?��?到的目的地�?�
//dummyCycles:空指令周期数
//	instructionMode:指令模�?;QSPI_INSTRUCTION_NONE,QSPI_INSTRUCTION_1_LINE,QSPI_INSTRUCTION_2_LINE,QSPI_INSTRUCTION_4_LINE  
//	addressMode:地�?�模�?; QSPI_ADDRESS_NONE,QSPI_ADDRESS_1_LINE,QSPI_ADDRESS_2_LINE,QSPI_ADDRESS_4_LINE
//	addressSize:地�?�长度;QSPI_ADDRESS_8_BITS,QSPI_ADDRESS_16_BITS,QSPI_ADDRESS_24_BITS,QSPI_ADDRESS_32_BITS
//	dataMode:数�?�模�?; QSPI_DATA_NONE,QSPI_DATA_1_LINE,QSPI_DATA_2_LINE,QSPI_DATA_4_LINE

void QSPI_Send_CMD(uint32_t instruction,uint32_t address,uint32_t dummyCycles,uint32_t instructionMode,uint32_t addressMode,uint32_t addressSize,uint32_t dataMode)
{
    QSPI_CommandTypeDef Cmdhandler;
    
    Cmdhandler.Instruction=instruction;                 	//指令
    Cmdhandler.Address=address;                            	//地�?�
    Cmdhandler.DummyCycles=dummyCycles;                     //设置空指令周期数
    Cmdhandler.InstructionMode=instructionMode;				//指令模�?
    Cmdhandler.AddressMode=addressMode;   					//地�?�模�?
    Cmdhandler.AddressSize=addressSize;   					//地�?�长度
    Cmdhandler.DataMode=dataMode;             				//数�?�模�?
    Cmdhandler.SIOOMode=QSPI_SIOO_INST_EVERY_CMD;       	//�?次都�?��?指令
    Cmdhandler.AlternateByteMode=QSPI_ALTERNATE_BYTES_NONE; //无交替字节
    Cmdhandler.DdrMode=QSPI_DDR_MODE_DISABLE;           	//关闭DDR模�?
    Cmdhandler.DdrHoldHalfCycle=QSPI_DDR_HHC_ANALOG_DELAY;

    HAL_QSPI_Command(&hqspi,&Cmdhandler,5000);
}

//QSPI接收指定长度的数�?�
//buf:接收数�?�缓冲区首地�?�
//datalen:�?传输的数�?�长度
//返回值:0,正常
//    其他,错误代�?
uint8_t QSPI_Receive(uint8_t* buf,uint32_t datalen)
{
    hqspi.Instance->DLR=datalen-1;                           //�?置数�?�长度
    if(HAL_QSPI_Receive(&hqspi,buf,5000)==HAL_OK) return 0;  //接收数�?�
    else return 1;
}

//QSPI�?��?指定长度的数�?�
//buf:�?��?数�?�缓冲区首地�?�
//datalen:�?传输的数�?�长度
//返回值:0,正常
//    其他,错误代�?
uint8_t QSPI_Transmit(uint8_t* buf,uint32_t datalen)
{
    hqspi.Instance->DLR=datalen-1;                            //�?置数�?�长度
    if(HAL_QSPI_Transmit(&hqspi,buf,5000)==HAL_OK) return 0;  //�?��?数�?�
    else return 1;
}

/* QUADSPI init function */
uint8_t CSP_QUADSPI_Init(void) {
	MX_QUADSPI_Init();
	W25Q256_Init();

	return HAL_OK;
}


uint8_t CSP_QSPI_Erase_Chip(void) {
	W25Q256_Write_Enable();					//SET WEL 
  W25Q256_Wait_Busy();   
	QSPI_Send_CMD(W25X_ChipErase,0,0,QSPI_INSTRUCTION_4_LINES,QSPI_ADDRESS_NONE,QSPI_ADDRESS_8_BITS,QSPI_DATA_NONE);//QPI,写全片擦除指令,地址为0,无数据_8位地址_无地址_4线传输指令,无空周期,0个字节数据
	W25Q256_Wait_Busy();						//等待芯片擦除结束

	return HAL_OK;
}

uint8_t QSPI_AutoPollingMemReady(void) {

	QSPI_CommandTypeDef sCommand;
	 QSPI_AutoPollingTypeDef sConfig;

	/* Configure automatic polling mode to wait for memory ready ------ */
	sCommand.InstructionMode = QSPI_INSTRUCTION_4_LINES;
	sCommand.Instruction = READ_STATUS_REG_CMD;
	sCommand.AddressMode = QSPI_ADDRESS_NONE;
	sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
	sCommand.DataMode = QSPI_DATA_1_LINE;
	sCommand.DummyCycles = 0;
	sCommand.DdrMode = QSPI_DDR_MODE_DISABLE;
	sCommand.DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;
	sCommand.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;

	sConfig.Match = 0x00;
	sConfig.Mask = 0x01;
	sConfig.MatchMode = QSPI_MATCH_MODE_AND;
	sConfig.StatusBytesSize = 1;
	sConfig.Interval = 0x10;
	sConfig.AutomaticStop = QSPI_AUTOMATIC_STOP_ENABLE;

	if (HAL_QSPI_AutoPolling(&hqspi, &sCommand, &sConfig,
	HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
		return HAL_ERROR;
	}

	return HAL_OK;
}

static uint8_t QSPI_WriteEnable(void) {
	QSPI_CommandTypeDef sCommand;
	QSPI_AutoPollingTypeDef sConfig;

	/* Enable write operations ------------------------------------------ */
	sCommand.InstructionMode = QSPI_INSTRUCTION_4_LINES;
	sCommand.Instruction = WRITE_ENABLE_CMD;
	sCommand.AddressMode = QSPI_ADDRESS_NONE;
	sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
	sCommand.DataMode = QSPI_DATA_NONE;
	sCommand.DummyCycles = 0;
	sCommand.DdrMode = QSPI_DDR_MODE_DISABLE;
	sCommand.DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;
	sCommand.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;

	if (HAL_QSPI_Command(&hqspi, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE)
			!= HAL_OK) {
		return HAL_ERROR;
	}

	/* Configure automatic polling mode to wait for write enabling ---- */
	sConfig.Match = 0x02;
	sConfig.Mask = 0x02;
	sConfig.MatchMode = QSPI_MATCH_MODE_AND;
	sConfig.StatusBytesSize = 1;
	sConfig.Interval = 0x10;
	sConfig.AutomaticStop = QSPI_AUTOMATIC_STOP_ENABLE;

	sCommand.Instruction = READ_STATUS_REG_CMD;
	sCommand.DataMode = QSPI_DATA_1_LINE;
	if (HAL_QSPI_AutoPolling(&hqspi, &sCommand, &sConfig,
	HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
		return HAL_ERROR;
	}

	return HAL_OK;
}
/*Enable quad mode and set dummy cycles count*/
uint8_t QSPI_Configuration(void) {
	uint8_t temp; 

	temp=W25Q256_ReadSR(3);      //读取状态寄存器3，判断地址模式
        if((temp&0X01)==0)			//如果不是4字节地址模式,则进入4字节地址模式
		{ 
			W25Q256_Write_Enable();	//写使能
			QSPI_Send_CMD(W25X_Enable4ByteAddr,0,0,QSPI_INSTRUCTION_4_LINES,QSPI_ADDRESS_NONE,QSPI_ADDRESS_8_BITS,QSPI_DATA_NONE);//QPI,使能4字节地址指令,地址为0,无数据_8位地址_无地址_4线传输指令,无空周期,0个字节数据 
		}
		W25Q256_Write_Enable();		//写使能
		QSPI_Send_CMD(W25X_SetReadParam,0,0,QSPI_INSTRUCTION_4_LINES,QSPI_ADDRESS_NONE,QSPI_ADDRESS_8_BITS,QSPI_DATA_4_LINES); 		//QPI,设置读参数指令,地址为0,4线传数据_8位地址_无地址_4线传输指令,无空周期,1个字节数据
		temp=3<<4;					//设置P4&P5=11,8个dummy clocks,104M
		QSPI_Transmit(&temp,1);		//发送1个字节
	return HAL_OK;
}

uint8_t CSP_QSPI_EraseSector(uint32_t EraseStartAddress, uint32_t EraseEndAddress) {

	uint32_t start_index = EraseStartAddress/MEMORY_SECTOR_SIZE;
	uint32_t end_index = EraseEndAddress/MEMORY_SECTOR_SIZE;
	while(end_index>=start_index){
		W25Q256_Erase_Sector(start_index);
		start_index++;
	}

	return HAL_OK;
}

uint8_t CSP_QSPI_WriteMemory(uint8_t* buffer, uint32_t address,uint32_t buffer_size) {

	W25Q256_Write(buffer, address, buffer_size);
	return HAL_OK;

}

/**
 * @brief 正点原子提供
 * 
 */
//QSPI进入内存映射模式（执行QSPI代码必备前提，为了减少引入的文件，
//除了GPIO驱动外，其他的外设驱动均采用寄存器形式）
void QSPI_Enable_Memmapmode(void)
{
	uint32_t tempreg=0; 
	volatile uint32_t *data_reg=&QUADSPI->DR;
	GPIO_InitTypeDef qspi_gpio;
	
	// RCC->AHB4ENR|=1<<1;    						//使能PORTB时钟	   
	// RCC->AHB4ENR|=1<<5;    						//使能PORTF时钟	   
	RCC->AHB3ENR|=1<<14;   						//QSPI时钟使能
 
	qspi_gpio.Pin=GPIO_PIN_6;					//PB6 AF10	
	qspi_gpio.Mode=GPIO_MODE_AF_PP;
	qspi_gpio.Speed=GPIO_SPEED_FREQ_VERY_HIGH;
	qspi_gpio.Pull=GPIO_NOPULL;
	qspi_gpio.Alternate=GPIO_AF10_QUADSPI;
	HAL_GPIO_Init(GPIOB,&qspi_gpio);
	
	qspi_gpio.Pin=GPIO_PIN_2;					//PB2 AF9	
	qspi_gpio.Alternate=GPIO_AF9_QUADSPI;
	HAL_GPIO_Init(GPIOB,&qspi_gpio);
	
	qspi_gpio.Pin=GPIO_PIN_6|GPIO_PIN_7;		//PF6,7 AF9	
	qspi_gpio.Alternate=GPIO_AF9_QUADSPI;
	HAL_GPIO_Init(GPIOF,&qspi_gpio);
	
	qspi_gpio.Pin=GPIO_PIN_8|GPIO_PIN_9;		//PF8,9 AF10		
	qspi_gpio.Alternate=GPIO_AF10_QUADSPI;
	HAL_GPIO_Init(GPIOF,&qspi_gpio);
	
	//QSPI设置，参考QSPI实验的QSPI_Init函数
	RCC->AHB3RSTR|=1<<14;			//复位QSPI
	RCC->AHB3RSTR&=~(1<<14);		//停止复位QSPI
	while(QUADSPI->SR&(1<<5));		//等待BUSY位清零 
	QUADSPI->CR=0X01000310;			//设置CR寄存器,这些值怎么来的，请参考QSPI实验/看H750参考手册寄存器描述分析
	QUADSPI->DCR=0X00160401;		//设置DCR寄存器
	QUADSPI->CR|=1<<0;				//使能QSPI 
 
	//注意:QSPI QE位的使能，在QSPI烧写算法里面，就已经设置了
	//所以,这里可以不用设置QE位，否则需要加入对QE位置1的代码
	//不过,代码必须通过仿真器下载,直接烧录到外部QSPI FLASH,是不可用的
	//如果想直接烧录到外部QSPI FLASH也可以用,则需要在这里添加QE位置1的代码
	
	//W25QXX进入QPI模式（0X38指令）
	while(QUADSPI->SR&(1<<5));		//等待BUSY位清零 
	QUADSPI->CCR=0X00000138;		//发送0X38指令，W25QXX进入QPI模式
	while((QUADSPI->SR&(1<<1))==0);	//等待指令发送完成
	QUADSPI->FCR|=1<<1;				//清除发送完成标志位 	
 
	//W25QXX写使能（0X06指令）
	while(QUADSPI->SR&(1<<5));		//等待BUSY位清零 
	QUADSPI->CCR=0X00000106;		//发送0X06指令，W25QXX写使能
	while((QUADSPI->SR&(1<<1))==0);	//等待指令发送完成
	QUADSPI->FCR|=1<<1;				//清除发送完成标志位 
	
	//W25QXX设置QPI相关读参数（0XC0）
	while(QUADSPI->SR&(1<<5));		//等待BUSY位清零 
	QUADSPI->CCR=0X030003C0;		//发送0XC0指令，W25QXX读参数设置
	QUADSPI->DLR=0;
	while((QUADSPI->SR&(1<<2))==0);	//等待FTF
	*(volatile uint8_t *)data_reg=3<<4;			//设置P4&P5=11,8个dummy clocks,104M
	QUADSPI->CR|=1<<2;				//终止传输 
	while((QUADSPI->SR&(1<<1))==0);	//等待数据发送完成
	QUADSPI->FCR|=1<<1;				//清除发送完成标志位  
	while(QUADSPI->SR&(1<<5));		//等待BUSY位清零 	 
 
	//MemroyMap 模式设置
	while(QUADSPI->SR&(1<<5));		//等待BUSY位清零 
	QUADSPI->ABR=0;					//交替字节设置为0，实际上就是W25Q 0XEB指令的,M0~M7=0
	tempreg=0XEB;					//INSTRUCTION[7:0]=0XEB,发送0XEB指令（Fast Read QUAD I/O）
	tempreg|=3<<8;					//IMODE[1:0]=3,四线传输指令
	tempreg|=3<<10;					//ADDRESS[1:0]=3,四线传输地址
	tempreg|=2<<12;					//ADSIZE[1:0]=2,24位地址长度
	tempreg|=3<<14;					//ABMODE[1:0]=3,四线传输交替字节
	tempreg|=0<<16;					//ABSIZE[1:0]=0,8位交替字节(M0~M7)
	tempreg|=8<<18;					//DCYC[4:0]=6,6个dummy周期
	tempreg|=3<<24;					//DMODE[1:0]=3,四线传输数据
	tempreg|=3<<26;					//FMODE[1:0]=3,内存映射模式
	QUADSPI->CCR=tempreg;			//设置CCR寄存器
	
	//设置QSPI FLASH空间的MPU保护
	SCB->SHCSR&=~(1<<16);			//禁止MemManage 
	MPU->CTRL&=~(1<<0);				//禁止MPU
	MPU->RNR=0;						//设置保护区域编号为0(1~7可以给其他内存用)
	MPU->RBAR=0X90000000;			//基地址为0X9000 000,即QSPI的起始地址
	MPU->RASR=0X0303002D;			//设置相关保护参数(禁止共用,允许cache,允许缓冲),详见MPU实验的解析
	MPU->CTRL=(1<<2)|(1<<0);		//使能PRIVDEFENA,使能MPU 
	SCB->SHCSR|=1<<16;				//使能MemManage
}

/**
  * @brief  配置QSPI为内存映射模式，野火提供
  * @retval QSPI内存状态
  */
uint32_t QSPI_EnableMemoryMappedMode()
{
 QSPI_CommandTypeDef      s_command;
  QSPI_MemoryMappedTypeDef s_mem_mapped_cfg;
 
  /* Configure the command for the read instruction */
	s_command.InstructionMode   = QSPI_INSTRUCTION_4_LINES;
  s_command.Instruction       = 0xeb;
  s_command.AddressMode       = QSPI_ADDRESS_4_LINES;
  s_command.AddressSize       = QSPI_ADDRESS_32_BITS;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_4_LINES;
	s_command.AlternateBytesSize = 0;
  s_command.DataMode          = QSPI_DATA_4_LINES;
  s_command.DummyCycles       = 6;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_HALF_CLK_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;
 
 
 
  /* Configure the memory mapped mode */
  s_mem_mapped_cfg.TimeOutActivation = QSPI_TIMEOUT_COUNTER_DISABLE;
  s_mem_mapped_cfg.TimeOutPeriod     = 0;
 
 
 
  if (HAL_QSPI_MemoryMapped(&hqspi, &s_command, &s_mem_mapped_cfg) != HAL_OK)
  {
    return 0;
  }
	
  return 1;
}

uint8_t CSP_QSPI_EnableMemoryMappedMode(void) {

	//QSPI_Enable_Memmapmode();
	QSPI_EnableMemoryMappedMode();
    return HAL_OK;
}


uint8_t QSPI_ResetChip() {
	QSPI_CommandTypeDef sCommand;
	uint32_t temp = 0;
	/* Erasing Sequence -------------------------------------------------- */
	sCommand.InstructionMode = QSPI_INSTRUCTION_4_LINES;
	sCommand.AddressSize = QSPI_ADDRESS_24_BITS;
	sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
	sCommand.DdrMode = QSPI_DDR_MODE_DISABLE;
	sCommand.DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;
	sCommand.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;
	sCommand.Instruction = RESET_ENABLE_CMD;
	sCommand.AddressMode = QSPI_ADDRESS_NONE;
	sCommand.Address = 0;
	sCommand.DataMode = QSPI_DATA_NONE;
	sCommand.DummyCycles = 0;

	if (HAL_QSPI_Command(&hqspi, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE)
			!= HAL_OK) {
		return HAL_ERROR;
	}
	for (temp = 0; temp < 0x2f; temp++) {
		__NOP();
	}

	sCommand.InstructionMode = QSPI_INSTRUCTION_4_LINES;
	sCommand.AddressSize = QSPI_ADDRESS_24_BITS;
	sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
	sCommand.DdrMode = QSPI_DDR_MODE_DISABLE;
	sCommand.DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;
	sCommand.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;
	sCommand.Instruction = RESET_EXECUTE_CMD;
	sCommand.AddressMode = QSPI_ADDRESS_NONE;
	sCommand.Address = 0;
	sCommand.DataMode = QSPI_DATA_NONE;
	sCommand.DummyCycles = 0;

	if (HAL_QSPI_Command(&hqspi, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE)
			!= HAL_OK) {
		return HAL_ERROR;
	}
	return HAL_OK;
}
/* USER CODE END 1 */
