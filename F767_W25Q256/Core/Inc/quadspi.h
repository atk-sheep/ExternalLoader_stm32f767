/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    quadspi.h
  * @brief   This file contains all the function prototypes for
  *          the quadspi.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __QUADSPI_H__
#define __QUADSPI_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern QSPI_HandleTypeDef hqspi;

/* USER CODE BEGIN Private defines */
uint8_t CSP_QUADSPI_Init(void);
uint8_t CSP_QSPI_EraseSector(uint32_t EraseStartAddress ,uint32_t EraseEndAddress);
uint8_t CSP_QSPI_WriteMemory(uint8_t* buffer, uint32_t address, uint32_t buffer_size);
uint8_t CSP_QSPI_EnableMemoryMappedMode(void);
uint8_t CSP_QSPI_Erase_Chip (void);

void QSPI_Send_CMD(uint32_t instruction,uint32_t address,uint32_t dummyCycles,uint32_t instructionMode,uint32_t addressMode,uint32_t addressSize,uint32_t dataMode);
uint8_t QSPI_Transmit(uint8_t* buf,uint32_t datalen);
uint8_t QSPI_Receive(uint8_t* buf,uint32_t datalen);
/* USER CODE END Private defines */

void MX_QUADSPI_Init(void);

/* USER CODE BEGIN Prototypes */
/*N25Q256A memory parameters*/
#define MEMORY_FLASH_SIZE				0x2000000 /* 32 MBytes*/
#define MEMORY_SECTOR_SIZE				0x1000   /* 4kBytes */
#define MEMORY_PAGE_SIZE				0x100     /* 256 bytes */


/*N25Q256A  commands */
#define WRITE_ENABLE_CMD 0x06
#define READ_STATUS_REG_CMD 0x05
#define ENTER_4_BYTE_ADD_CMD 0xB7
#define WRITE_VOL_CFG_REG_CMD 0x81
#define SECTOR_ERASE_CMD 0xD8
#define CHIP_ERASE_CMD 0xC7
#define QUAD_IN_FAST_PROG_CMD 0x32
#define READ_CONFIGURATION_REG_CMD 0x85
#define QUAD_OUT_FAST_READ_CMD 0xEB
#define DUMMY_CLOCK_CYCLES_READ_QUAD 10
#define RESET_ENABLE_CMD 0x66
#define RESET_EXECUTE_CMD 0x99
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __QUADSPI_H__ */

