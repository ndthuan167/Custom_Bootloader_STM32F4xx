/**
 * @file Custom_Bootloader.h
 * @author Nguyen Dinh Thuan (thuan.nd.167@gmail.com)
 * @brief Definition for custom bootloader for STM32F4xx MCU
 * @date 2025-04-21
 */

/******************************************************************************
 * Include Files
 ******************************************************************************/

#include "stdio.h"
#include "stdlib.h"
#include "stdint.h"
#include "stdbool.h"

#pragma pack(1) // Ensure no padding between struct members

/******************************************************************************
 * Definition
 ******************************************************************************/

#define FIRMWARE_UPDATE_TIME_WAITING 3000 // 3s = 3000ms

#define FALSH_SECTOR2_START_ADDRESS 0x08008000UL
#define SIZE_OF_CURRENT_FW_FLASH    (uint16_t)(360448)

#define NEW_APPLICATION_START_ADDRESS 0x08060000UL
#define SIZE_OF_NEW_FW_FLASH        (uint16_t)(131072 * 5)

/******************************************************************************
 * Function definition
 ******************************************************************************/

void Bootloader_Init(void);
void JumpToApplication(void);
void New_Firmware_Update(void);
void GetDataWriterAndLoadIntoNewFwFlash(void);
bool CRCCheckingFirmware(void);

void EarseNewFirmwareSector(void);
