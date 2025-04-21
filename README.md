# Custom_Bootloader_STM32F4xx

Config reset MCU from FLASH MEMORY (BOOT 1 = X, BOOT0 = 1). Base on the Flash module organization of STM32F4xx to seperate 12 sectors to 3 parts:
- Sector 0-1 (0x0800 0000 - 0x0800 7FFF) stored Custom Bootloader
- Sector 2-6 (0x0800 8000 - 0x0805 FFFF) stored the User Application
- Sector 7-11 (0x0806 0000 - 0x080F FFFF) stored the new FW update.

![image](https://github.com/user-attachments/assets/dc1098c2-1192-4da1-b128-14de1b2351a7)

After definition space for each part. Make the method to check and update new FW in Custom bootloader as follow:

![image](https://github.com/user-attachments/assets/c4906743-a0da-4185-855f-77b994ad419b)

=========================================================================================

1. Main method: Check firmware update button pressed util 3s, if it is pressed -> Move to Check and Update new Firmware, otherwise, move to Application immediately.

```c
	while (1)
	{
		LoopSystickTimerSetting(); // Systick timer loop

		// Check 3s to press the firmware update button
		if (GetFlagTimerSystick1ms())
		{
			firmware_update_time_waiting++;
		}

		if (firmware_update_time_waiting >= FIRMWARE_UPDATE_TIME_WAITING)  // 3000 ms = 3s
		{
			JumpToApplication(); // Jump to current application if not press the firmware update button in 3s
			firmware_update_time_waiting = 0;
		}
		else
		{
			if (firmware_update_button_pressed == SET)
			{
				New_Firmware_Update(); // Update new firmware if press the firmware update button in 3s
			}
		}
	}
```

2. Checking and Update new firmware function:
   + Earse all sectors of new firmware (sector 7 - 11)
   + Receive data from writer and write to Flash memory from Sector 7
   + Check CRC of new firmware with current firmware in Sector2
   + If CRC is different, turn on CRC Check LED and earse current firmware in Sector2 - 6 and copy new firmware from Sector7 - 11 to Sector2 - 6
   + Jump to current application
   + If CRC is same, turn on Error Check LED and jump to current application (do not update)

```c
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

```
