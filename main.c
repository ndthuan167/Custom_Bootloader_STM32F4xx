#include "Thuan_STM32F407VGTx_Driver.h"

uint8_t firmware_update_time_waiting = 0; // Time to wait for the firmware update process
bool firmware_update_button_pressed = CLEAR;

int main()
{

	RCC_EnablePeripheralClock(CLOCK_SYSCFG);

	// Systick Timer config for delay function
	SettingSystemTimer(PROCESSOR_CLKSRC, ENABLE, SYSTICK_TIMER_1MS);

	// Config Exti line for button
	EXTI_Configuration(EXTIx_PA_PIN, EXTI_LINE_0, NOT_MASKED, MASKED, RISING_TRIGGER);
	NVIC_Configuration(_EXTI1_IRQHandler, 0x05, ENABLE);

	// Bootloader initialization
	Bootloader_Init();

	while (1)
	{
		LoopSystickTimerSetting(); // Systick timer loop

		// Check 3s to press the firmware update button
		if (GetFlagTimerSystick1ms())
		{
			firmware_update_time_waiting++;
		}

		if (firmware_update_time_waiting >= FIRMWARE_UPDATE_TIME_WAITING)
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
}

void SysTick_Handler(void)
{
    systick_timer_loop_1ms_IT++;
}

void EXTI1_IRQHandler()
{
    firmware_update_button_pressed = true;
	NVIC_ClearPendingInterrupt(_EXTI1_IRQHandler); // Clear pending interrupt flag
}
