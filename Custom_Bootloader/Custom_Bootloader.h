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

/******************************************************************************
 * Definition
 ******************************************************************************/

#define FIRMWARE_UPDATE_TIME_WAITING 3000 // 3s = 3000ms
#define CURRENT_APPLICATION_START_ADDRESS 0x08008000UL

#define NEW_APPLICATION_START_ADDRESS 0x08060000UL

#define FALSH_SECTOR2_START_ADDRESS 0x08008000UL

typedef struct
{
    uint8_t Sector2[16384];
    uint8_t Sector3[16384];
    uint8_t Sector4[65536];
    uint8_t Sector5[131072];
    uint8_t Sector6[131072];
} Sector_crr_FW;

#define FLASH_SECTOR7_START_ADDREESS 0x08060000UL

typedef struct
{
    uint8_t Sector7[131072];
    uint8_t Sector8[131072];
    uint8_t Sector9[131072];
    uint8_t Sector10[131072];
    uint8_t Sector11[131072];

} Sector_new_FW;

/******************************************************************************
 * Function definition
 ******************************************************************************/

void Bootloader_Init(void);
void JumpToApplication(void);
void New_Firmware_Update(void);
void GetDataWriterAndLoadIntoNewFwFlash(void);
bool CRCCheckingFirmware(void);

void EarseNewFirmwareSector(void);
void EarseCurrentFirmwareSector(void);
