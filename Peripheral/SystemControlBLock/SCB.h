/**
 * @file SCB.h
 * @author Nguyen Dinh Thuan (thuan.nd.167@gmail.com)
 * @brief Some declaration for System control block of STM32F407VGTx (ARMCortex M4)
 * @date 2024-07-14
 *
 */

/******************************************************************************
 * Include Files
 ******************************************************************************/

#include "stdio.h"
#include "stdint.h"

/*******************************************************************************
 * SCB REGISTER STRUCTURE DEFINITION
 *******************************************************************************/

typedef struct
{
    volatile uint32_t CPUID;
    volatile uint32_t ICSR;
    volatile uint32_t VTOR;
    volatile uint32_t AIRCR;
    volatile uint32_t SCR;
    volatile uint32_t CCR;
    volatile uint32_t SHPR1;
    volatile uint32_t SHPR2;
    volatile uint32_t SHPR3;
    volatile uint32_t SHCRS;
    volatile uint32_t CFSR;
    volatile uint32_t HFSR[2];
    volatile uint32_t MMAR;
    volatile uint32_t BFAR;
    volatile uint32_t AFSR;
} SCB;

/*******************************************************************************
 * SCB ADDRESS DEFINITION
 *******************************************************************************/

#define ADDRESS_CSB_ACTLR 0xE000E008
#define ADDRESS_SCB 0xE000ED00

/*******************************************************************************
 * ENUM DEFINITION
 *******************************************************************************/
// numerical order of bit in ICSR register
enum
{
    VECTACTIVE,
    RETTOBASE = 11,
    VECTPENDING,
    ISRPENDING = 22,
    PENDSTCLR = 25,
    PENDSTSET,
    PENDSVCLR,
    PENDSVSET,
    NMIPENDSET = 31,
};

/*******************************************************************************
 * COMMON DEFINITION
 *******************************************************************************/

/*******************************************************************************
 * SCB_ICSR (Interrupt Control and State register) REGISTERS DEFINITION
 *******************************************************************************/

#define SCB_SET_SYSTICK_PENDING(_SCB)               _SCB->ICSR |= (1 << PENDSTSET)
#define SCB_GET_SYSTICK_PENDING(_SCB)               (_SCB->ICSR & (1 << PENDSTSET)) >> PENDSTSET
#define SCB_CLEAR_SYSTICK_PENDING(_SCB)              _SCB->ICSR |= (1 << PENDSTCLR)

/*******************************************************************************
 * SCB_SHPR3(System Handler Priority Register 3) REGISTERS DEFINITION
 *******************************************************************************/
#define SCB_SET_SYSTICK_PRIORITY(_SCB, value)       _SCB->SHPR3 |= (value << 24)

/*******************************************************************************
 * FUNCTIONS DEFINITION
 *******************************************************************************/
void SCB_SetSysTickPending(void);
uint8_t SCB_GetSysTickPendingStatus(void);
void SCB_ClearSysTickPending(void);

void SCB_SetSysTickPriority(uint8_t priority);
