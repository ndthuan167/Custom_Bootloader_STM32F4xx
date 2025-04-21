/**
 * @file Custom_Bootloader.c
 * @author Nguyen Dinh Thuan (thuan.nd.167@gmail.com)
 * @brief Writing custom bootloader for STM32F4xx MCU
 * @date 2025-04-21
 */

/******************************************************************************
 * Include Files
 ******************************************************************************/

#include "Custom_Bootloader.h"
#include "../Peripheral/GPIO/GPIO.h"
#include "../Peripheral/RCC/RCC.h"
#include "../Peripheral/USART/USART.h"
#include "../Peripheral/SystemControlBlock/SCB.h"
#include "string.h"

/******************************************************************************
 * Definition
 ******************************************************************************/

#define CRC32_POLYNOMIAL 0xEDB88320UL

GPIOn *gpio_A_bl = (GPIOn *)ADDRESS_GPIO_A;
USARTn *usart2 = (USARTn *)ADDRESS_USART_2;
SCB *scb = (SCB *)ADDRESS_SCB;

/******************************************************************************
 * Variables definition
 ******************************************************************************/

uint32_t num_of_sector7_wrote = 0;
uint32_t num_of_sector8_wrote = 0;
uint32_t num_of_sector9_wrote = 0;
uint32_t num_of_sector10_wrote = 0;
uint32_t num_of_sector11_wrote = 0;

Sector_new_FW *sector_new_fw_address = (Sector_new_FW *)FLASH_SECTOR7_START_ADDREESS;
Sector_crr_FW *sector_crr_fw_address = (Sector_crr_FW *)FALSH_SECTOR2_START_ADDRESS;

/**
*******************************************************************************
* @ Name : Bootloader_Init
* @ Parameters: void
* @ Registers :
* @ Descriptions :
*		- Initialize Bootloader:
*			+ Enable Clock for GPIO A for CRC Check LED, Error Check LED
*			+ Config FW update button as Input
*			+ Config LED for CRC check as Output
*			+ Config LED for Error check as Output
*			+ Config USART2 to receive data from Writer
*			+ Set CRC Check LED, Error Check LED is OFF in Intitial
* @ Return value : void
* @ author : Nguyen Dinh Thuan(thuan.nd.167@gmail.com)
* @ date : 2025-04-21
*******************************************************************************
*/
void Bootloader_Init(void)
{
    // Enable Clock for GPIO A for CRC Check LED, Error Check LED
    RCC_EnablePeripheralClock(CLOCK_GPIO_A);

    // Config FW update button as Input
    GPIO_Configuration(gpio_A_bl, GPIO_PIN0, MODER_INPUT, OTYPER_PUSHPULL, OSPEEDR_LOW, PUPDR_NOTHING);

    // Config LED for CRC check
    GPIO_Configuration(gpio_A_bl, GPIO_PIN1, MODER_OUTPUT, OTYPER_PUSHPULL, OSPEEDR_HIGH, PUPDR_NOTHING);

    // Config LED for Error check
    GPIO_Configuration(gpio_A_bl, GPIO_PIN2, MODER_OUTPUT, OTYPER_PUSHPULL, OSPEEDR_HIGH, PUPDR_NOTHING);

    // Config USART2 to receive data from Writer
    USART_Configuration(usart2, BAUDRATE_9600, DATA_BITS_8, OVERSAMPLING_BY_16, PARITY_DISABLE, EVEN_PARITY);

    // Set CRC Check LED, Error Check LED is OFF in Intitial
    GPIO_SettingOutputDataBSRR(gpio_A_bl, GPIO_PIN1, CLEAR);
    GPIO_SettingOutputDataBSRR(gpio_A_bl, GPIO_PIN2, CLEAR);
}

/**
*******************************************************************************
* @ Name : JumpToApplication()
* @ Parameters: void
* @ Registers :
* @ Descriptions :
*		- Jump to application:
*          + Disable all interrupts
*          + Set main stack pointer to application's stack base address
*          + Update vector table offset register to point to the application's vector table
*          + Jump to the application's reset handler
* @ Return value : void
* @ author : Nguyen Dinh Thuan(thuan.nd.167@gmail.com)
* @ date : 2025-04-21
*******************************************************************************
*/
void JumpToApplication(void)
{
    // 1. Disable all interrupts
    __disable_irq();

    // 2. Set main stack pointer to application's stack base address
    uint32_t app_stack_pointer = *((volatile uint32_t*)CURRENT_APPLICATION_START_ADDRESS);
    __set_MSP(app_stack_pointer);

    // 3. Update vector table offset register to point to the application's vector table
    scb->VTOR = CURRENT_APPLICATION_START_ADDRESS;

    // 4. Jump to the application's reset handler
    void (*app_reset_handler)(void);
    app_reset_handler = (void (*)(void))(*((volatile uint32_t*)(CURRENT_APPLICATION_START_ADDRESS + 4)));
    app_reset_handler();
}

/**
*******************************************************************************
* @ Name : GetDataWriterAndLoadIntoNewFwFlash()
* @ Parameters: void
* @ Registers :
* @ Descriptions :
*		- Get data from Writer and load into new firmware flash (sector 7 - 11):
*          + Receive data from USART2 and load into new firmware flash (each sector has 131072 bytes)
* @ Return value : void
* @ author : Nguyen Dinh Thuan(thuan.nd.167@gmail.com)
* @ date : 2025-04-21
*******************************************************************************
*/
void GetDataWriterAndLoadIntoNewFwFlash(void)
{
    uint32_t data_received;

    Sector_new_FW *sector_new_fw_address = (Sector_new_FW *)FLASH_SECTOR7_START_ADDREESS;

    if (num_of_sector7_wrote < 131072)
    {
        data_received = USART_ReceiveData(usart2);
        sector_new_fw_address->Sector7[num_of_sector7_wrote] = data_received;
        num_of_sector7_wrote++;
    }

    if (num_of_sector7_wrote >= 131072 && num_of_sector8_wrote < 131072)
    {
        num_of_sector7_wrote = 131073;
        data_received = USART_ReceiveData(usart2);
        sector_new_fw_address->Sector8[num_of_sector8_wrote] = data_received;
        num_of_sector8_wrote++;
    }

    if (num_of_sector8_wrote >= 131072 && num_of_sector9_wrote < 131072)
    {
        num_of_sector8_wrote = 131073;
        data_received = USART_ReceiveData(usart2);
        sector_new_fw_address->Sector9[num_of_sector9_wrote] = data_received;
        num_of_sector9_wrote++;
    }

    if (num_of_sector9_wrote >= 131072 && num_of_sector10_wrote < 131072)
    {
        num_of_sector9_wrote = 131073;
        data_received = USART_ReceiveData(usart2);
        sector_new_fw_address->Sector10[num_of_sector10_wrote] = data_received;
        num_of_sector10_wrote++;
    }

    if (num_of_sector10_wrote >= 131072 && num_of_sector11_wrote < 131072)
    {
        num_of_sector10_wrote = 131073;
        data_received = USART_ReceiveData(usart2);
        sector_new_fw_address->Sector11[num_of_sector11_wrote] = data_received;
        num_of_sector11_wrote++;
    }

    if (num_of_sector11_wrote >= 131072)
    {
        num_of_sector11_wrote = 131073;
    }
}

/**
*******************************************************************************
* @ Name : CalculateCRC32()
* @ Parameters: void
* @ Registers :
* @ Descriptions :
*		- Calculate CRC32 checksum for the given data:
* @ Return value : void
* @ author : Nguyen Dinh Thuan(thuan.nd.167@gmail.com)
* @ date : 2025-04-21
*******************************************************************************
*/
uint32_t CalculateCRC32(const uint8_t *data, size_t length)
{
    uint32_t crc = 0xFFFFFFFFUL;
    uint16_t i;
    uint16_t j;
    for (i = 0; i < length; ++i)
    {
        crc ^= data[i];
        for (j = 0; j < 8; ++j)
        {
            if (crc & 1)
            {
                crc = (crc >> 1) ^ CRC32_POLYNOMIAL;
            }
            else
            {
                crc >>= 1;
            }
        }
    }
    return crc ^ 0xFFFFFFFFUL; // XOR cuối cùng để có giá trị cuối cùng
}

/**
*******************************************************************************
* @ Name : CRCCheckingFirmware()
* @ Parameters: void
* @ Registers :
* @ Descriptions :
*		- Check CRC of new firmware with current firmware:
*          + Calculate CRC of new firmware from sector 7 - 11 and crr firmware from sector 2 - 6
*          + Compare CRC of new firmware with current firmware
*          + If CRC is matched, return true, else return false
* @ Return value : bool
* @ author : Nguyen Dinh Thuan(thuan.nd.167@gmail.com)
* @ date : 2025-04-21
*******************************************************************************
*/
bool CRCCheckingFirmware(void)
{
    bool CRCMatched = false;

    // Calculate CRC of new FW from sector 7 - 11 and crr FW from sector 2 - 6
    uint32_t crc_new_fw = CalculateCRC32((const uint8_t *)&sector_new_fw_address->Sector7[0], sizeof(sector_new_fw_address->Sector7) + sizeof(sector_new_fw_address->Sector8) + sizeof(sector_new_fw_address->Sector9) + sizeof(sector_new_fw_address->Sector10) + sizeof(sector_new_fw_address->Sector11));
    uint32_t crc_crr_fw = CalculateCRC32((const uint8_t *)&sector_crr_fw_address->Sector2[0], sizeof(sector_crr_fw_address->Sector2) + sizeof(sector_crr_fw_address->Sector3) + sizeof(sector_crr_fw_address->Sector4) + sizeof(sector_crr_fw_address->Sector5) + sizeof(sector_crr_fw_address->Sector6));

    // Comparision
    if (crc_new_fw == crc_crr_fw)
    {
        CRCMatched = true;
    }

    return CRCMatched;
}

/**
*******************************************************************************
* @ Name : EarseNewFirmwareSector()
* @ Parameters: void
* @ Registers :
* @ Descriptions :
*		- Earse all sectors of new firmware (sector 7 - 11) by writing 0xF to all bytes
*          + Each sector has 131072 bytes
* @ Return value : void
* @ author : Nguyen Dinh Thuan(thuan.nd.167@gmail.com)
* @ date : 2025-04-21
*******************************************************************************
*/
void EarseNewFirmwareSector(void)
{
    uint32_t sector_new_fw = 0;

    for (sector_new_fw = 0; sector_new_fw < 131072; sector_new_fw++)
    {
        sector_new_fw_address->Sector7[sector_new_fw] = 0xF;
        sector_new_fw_address->Sector8[sector_new_fw] = 0xF;
        sector_new_fw_address->Sector9[sector_new_fw] = 0xF;
        sector_new_fw_address->Sector10[sector_new_fw] = 0xF;
        sector_new_fw_address->Sector11[sector_new_fw] = 0xF;
    }
}

/**
*******************************************************************************
* @ Name : EarseCurrentFirmwareSector()
* @ Parameters: void
* @ Registers :
* @ Descriptions :
*		- Earse all sectors of current firmware (sector 2 - 6) by writing 0xF to all bytes
*         + Each sector has 16384 bytes for sector 2 - 3 and 65536 bytes for sector 4 and 131072 bytes for sector 5 - 6
* @ Return value : void
* @ author : Nguyen Dinh Thuan(thuan.nd.167@gmail.com)
* @ date : 2025-04-21
*******************************************************************************
*/
void EarseCurrentFirmwareSector(void)
{
    uint16_t sector2_3_index = 0;
    uint32_t sector4_index = 0;
    uint32_t sector5_6_index = 0;

    for (sector2_3_index = 0; sector2_3_index < 16384; sector2_3_index++)
    {
        sector_crr_fw_address->Sector2[sector2_3_index] = 0xF;
        sector_crr_fw_address->Sector3[sector2_3_index] = 0xF;
    }

    for (sector4_index = 0; sector4_index < 65536; sector4_index++)
    {
        sector_crr_fw_address->Sector4[sector4_index] = 0xF;
    }

    for (sector5_6_index = 0; sector5_6_index < 131072; sector5_6_index++)
    {
        sector_crr_fw_address->Sector5[sector5_6_index] = 0xF;
        sector_crr_fw_address->Sector6[sector5_6_index] = 0xF;
    }
}

/**
*******************************************************************************
* @ Name : New_Firmware_Update()
* @ Parameters: void
* @ Registers :
* @ Descriptions :
*		- Check and update new firmware:
*          + Earse all sectors of new firmware (sector 7 - 11)
*          + Receive data from writer and write to Flash memory from Sector 7
*          + Check CRC of new firmware with current firmware in Sector2
*          + If CRC is different, turn on CRC Check LED and earse current firmware in Sector2 - 6 and copy new firmware from Sector7 - 11 to Sector2 - 6
*          + Jump to current application
*          + If CRC is same, turn on Error Check LED and jump to current application (do not update)
* @ Return value : void
* @ author : Nguyen Dinh Thuan(thuan.nd.167@gmail.com)
* @ date : 2025-04-21
*******************************************************************************
*/
void New_Firmware_Update(void)
{
    // Earse all sectors of new firmware
    EarseNewFirmwareSector();

    // Receive data from writer and write to Flash memory from Sector 7
    GetDataWriterAndLoadIntoNewFwFlash();

    // Check CRC of new firmware with current firmware in Sector2
    if (!CRCCheckingFirmware())
    {
        // Turn on CRC Check LED -> diffent FW -> update
        GPIO_SettingOutputDataBSRR(gpio_A_bl, GPIO_PIN1, SET);

        // Earse current firmware in Sector2 - 6
        EarseCurrentFirmwareSector();

        // Copy new firmware from Sector7 - 11 to Sector2 - 6
        memcpy(sector_crr_fw_address, sector_new_fw_address, sizeof(Sector_new_FW));

        // Jump to current application
        JumpToApplication();
    }
    else
    {
        // Turn on Error Check LED -> same FW -> do not update
        GPIO_SettingOutputDataBSRR(gpio_A_bl, GPIO_PIN2, SET);
        JumpToApplication();
    }
}
