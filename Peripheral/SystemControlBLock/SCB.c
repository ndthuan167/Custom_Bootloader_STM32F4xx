/**
 * @file NVIC.c
 * @author Nguyen Dinh Thuan (thuan.nd.167@gmail.com)
 * @brief Configuration for NVIC of STM32F407VGTx (ARMCortex M4)
 * @date 2024-07-11
 *
 */

/******************************************************************************
 * Include Files
 ******************************************************************************/

#include "SCB.h"

// SCB
SCB* _SCB = (SCB*) ADDRESS_SCB;

/**
*******************************************************************************
* @ Name : SCB_SetSysTickPending
* @ Parameters: void
* @ Registers : SCB_ICSR
* @ Descriptions :
*		- Setting pending for Systick timer.
* @ Return value : void
* @ author : Nguyen Dinh Thuan(thuan.nd.167@gmail.com)
* @ date : 2024-07-14
*******************************************************************************
*/
void SCB_SetSysTickPending(void)
{
    SCB_SET_SYSTICK_PENDING(_SCB);
}

/**
*******************************************************************************
* @ Name : SCB_GetSysTickPendingStatus
* @ Parameters: void
* @ Registers : SCB_ICSR
* @ Descriptions :
*		- Get status of pending for Systick timer.
* @ Return value : uint8_t
* @ author : Nguyen Dinh Thuan(thuan.nd.167@gmail.com)
* @ date : 2024-07-14
*******************************************************************************
*/
uint8_t SCB_GetSysTickPendingStatus(void)
{
    return SCB_GET_SYSTICK_PENDING(_SCB);
}

/**
*******************************************************************************
* @ Name : SCB_ClearSysTickPending
* @ Parameters: void
* @ Registers : SCB_ICSR
* @ Descriptions :
*		- Clear pending for Systick timer.
* @ Return value : uint8_t
* @ author : Nguyen Dinh Thuan(thuan.nd.167@gmail.com)
* @ date : 2024-07-14
*******************************************************************************
*/
void SCB_ClearSysTickPending(void)
{
    SCB_CLEAR_SYSTICK_PENDING(_SCB);
}

/**
*******************************************************************************
* @ Name : SCB_SetSysTickPriority
* @ Parameters: uint8_t priority
* @ Registers : SCB_SHPR3
* @ Descriptions :
*		- Set priority for Systick timer.
* @ Return value : void
* @ author : Nguyen Dinh Thuan(thuan.nd.167@gmail.com)
* @ date : 2024-07-14
*******************************************************************************
*/
void SCB_SetSysTickPriority(uint8_t priority)
{
    SCB_SET_SYSTICK_PRIORITY(_SCB, priority);
}
