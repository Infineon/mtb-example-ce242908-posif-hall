/*******************************************************************************
* File Name:   main.c
*
* Description: This example demonstrates a Position Interface (POSIF) module
*              in Hall sensor mode and uses the CCU40 module to determine the
*              speed of rotation of the motor.
*              Instead of hall motor, the example demonstrates the use of
*              POSIF_HALL module, using simulation via 3 PWM signals.
*
* Related Document: See README.md
*
********************************************************************************
* (c) 2026, Infineon Technologies AG, or an affiliate of Infineon
* Technologies AG. All rights reserved.
* This software, associated documentation and materials ("Software") is
* owned by Infineon Technologies AG or one of its affiliates ("Infineon")
* and is protected by and subject to worldwide patent protection, worldwide
* copyright laws, and international treaty provisions. Therefore, you may use
* this Software only as provided in the license agreement accompanying the
* software package from which you obtained this Software. If no license
* agreement applies, then any use, reproduction, modification, translation, or
* compilation of this Software is prohibited without the express written
* permission of Infineon.
*
* Disclaimer: UNLESS OTHERWISE EXPRESSLY AGREED WITH INFINEON, THIS SOFTWARE
* IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
* INCLUDING, BUT NOT LIMITED TO, ALL WARRANTIES OF NON-INFRINGEMENT OF
* THIRD-PARTY RIGHTS AND IMPLIED WARRANTIES SUCH AS WARRANTIES OF FITNESS FOR A
* SPECIFIC USE/PURPOSE OR MERCHANTABILITY.
* Infineon reserves the right to make changes to the Software without notice.
* You are responsible for properly designing, programming, and testing the
* functionality and safety of your intended application of the Software, as
* well as complying with any legal requirements related to its use. Infineon
* does not guarantee that the Software will be free from intrusion, data theft
* or loss, or other breaches ("Security Breaches"), and Infineon shall have
* no liability arising out of any Security Breaches. Unless otherwise
* explicitly approved by Infineon, the Software may not be used in any
* application where a failure of the Product or any consequences of the use
* thereof can reasonably be expected to result in personal injury.
*******************************************************************************/

#include "cybsp.h"
#include "cy_utils.h"
#include "cy_retarget_io.h"
#include <stdio.h>

/*******************************************************************************
*  Macros
*******************************************************************************/
#define TICKS_PER_SECOND                    (1000U)
#define TICKS_WAIT                          (100U)

/* Define macro to enable/disable printing of debug messages */
#define ENABLE_DEBUG_PRINT              (0)

/* Define macro to set the loop count before printing debug messages */
#if ENABLE_DEBUG_PRINT
#define DEBUG_LOOP_COUNT_MAX                (3U)
#endif

/*******************************************************************************
* Global variables
*******************************************************************************/
/* Correct hall event and wrong hall event flag variables */
uint8_t che_flag = 0, whe_flag = 0;

/* CCU8 pulse counter */
uint8_t ccu8_pulse_counter = 0;

/* Timers flag */
bool timers_started = false;

/* Hall input array */
uint8_t hall[3] = {0,0,0};

/* Hall position variable */
uint8_t hall_position = 0;

/* Correct hall event variable */
unsigned long hall_events_interval = 0;

#if ENABLE_DEBUG_PRINT
/* Initialize the current loop count to zero */
static uint32_t debug_loop_count = 0;
#endif

 /*******************************************************************************
 * Function Name: SysTick Handler
 ********************************************************************************
 * Summary:
 *  This is the interrupt handler function for the System Tick interrupt. This
 *  function print the time interval between two correct hall events and wrong
 *  hall event.
 *
 * Parameters:
 *  none
 *
 * Return:
 *  void
 *
 *******************************************************************************/
void SysTick_Handler(void)
{
    /* Ticks wait */
    static uint32_t ticks = 0;

    ticks++;

    /* Wait for 500ms delay */
    if (ticks == TICKS_WAIT)
    {
        ticks = 0;
        /* Check if correct hall event occurs */
        if((che_flag == 1) && (whe_flag == 0))
        {
            /* Set che_flag to 0 */
            che_flag = 0;
            #if ENABLE_DEBUG_PRINT
                debug_loop_count++;
                if (debug_loop_count == DEBUG_LOOP_COUNT_MAX)
                    printf("All three correct hall events occurs\r\n");
            #else
                /* Print the time interval between two correct hall events in nano seconds */
                printf("Time interval between two correct hall events: %luns\r\n", hall_events_interval);
            #endif
        }
        /* Check if wrong hall event occurs */
        else if((che_flag == 0) && (whe_flag == 1))
        {
            /* Set whe_flag to 0 */
            whe_flag = 0;
            /* Print the wrong hall event */
            printf("Wrong hall event\r\n");
        }
    }
}

/*******************************************************************************
* Function Name: POSIF0_0_IRQHandler
********************************************************************************
* Summary:
*  POSIF0_0_IRQHandler interrupt handler function will occur for every
*  correct hall pattern. Calculate the timing between two correct hall events.
*
* Parameters:
*  none
*
* Return:
*  none
*
*******************************************************************************/
void POSIF0_0_IRQHandler(void)
{
    /* Get the capture timer value */
    uint16_t captured_value = 0;

    /* Set che_flag to 1 */
    che_flag = 1;
    /* Set whe_flag to 0 */
    whe_flag = 0;

    /* Check for a rising edge of POSIF0.OUT1 signal */
    if (Cy_CCU4_SLICE_GetEvent(HALL_SPEED_TIMER_HW, CY_CCU4_SLICE_IRQ_ID_EVENT0))
    {
        /* Clear event*/
        Cy_CCU4_SLICE_ClearEvent(HALL_SPEED_TIMER_HW, CY_CCU4_SLICE_IRQ_ID_EVENT0);

        /* Get captured timer value on rising edge */
        captured_value = Cy_CCU4_SLICE_GetCaptureRegisterValue(HALL_SPEED_TIMER_HW, 1U);

        /* Calculate the time between two correct hall events
         * (captured_value * prescaler * 1000) / clock */
        hall_events_interval = captured_value * HALL_SPEED_TIMER_TICK_NS;
    }
    /* Clear pending event */
    Cy_POSIF_ClearEvent(HALL_POSIF_HW, CY_POSIF_IRQ_EVENT_CHE);
}

/*******************************************************************************
* Function Name: POSIF0_1_IRQHandler
********************************************************************************
* Summary:
*  POSIF0_1_IRQHandler interrupt handler function will occur for every
*  wrong hall pattern.
*
* Parameters:
*  none
*
* Return:
*  none
*
*******************************************************************************/
void POSIF0_1_IRQHandler(void)
{
    /* Set whe_flag to 1 */
    whe_flag = 1;
    /* Set che_flag to 0 */
    che_flag = 0;

    /* Clear pending event */
    Cy_POSIF_ClearEvent(HALL_POSIF_HW, CY_POSIF_IRQ_EVENT_WHE);
}

/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
*  This is the main function. It starts the POSIF Module in hall mode and uses
*  the CCU40 module to determine the speed of rotation of the motor. Each time a
*  correct Hall event is detected, an interrupt is generated. Timing between the
*  two correct hall events are displayed on the terminal. Each time a wrong hall
*  event is detected, an interrupt is generated and displayed on the terminal.
*
* Parameters:
*  none
*
* Return:
*  int
*
*******************************************************************************/
int main(void)
{
    cy_rslt_t result;

    /* Initialize the device and board peripherals */
    result = cybsp_init();
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Initialize retarget-io to use the debug UART port */
    result = cy_retarget_io_init(CYBSP_DEBUG_UART_HW);
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    #if ENABLE_DEBUG_PRINT
    printf("Initialization done\r\n");
    #else
    /* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
    printf("\x1b[2J\x1b[;H");
    printf("============================================================ \r\n");
    printf("PSOC™ Control C1 MCU: POSIF Hall example \r\n");
    printf("============================================================ \r\n");
    #endif


    /* Set priority */
    NVIC_SetPriority(POSIF0_0_IRQn, 0U);
    NVIC_SetPriority(POSIF0_1_IRQn, 1U);

    /* Enable IRQ */
    NVIC_EnableIRQ(POSIF0_0_IRQn);
    NVIC_EnableIRQ(POSIF0_1_IRQn);

    /* Print the CHE/WHE occurrence for every 500ms */
    SysTick_Config(SystemCoreClock / TICKS_PER_SECOND);

    /* Start HALL_1, HALL_2 and HALL_3 Timers */
    Cy_CCU8_SLICE_StartTimer(HALL_1_HW);
    Cy_CCU8_SLICE_StartTimer(HALL_2_HW);
    Cy_CCU8_SLICE_StartTimer(HALL_3_HW);

    while (1)
    {
        Cy_Delay(1);
        /* Checks if period match event has occurred */
        if (Cy_CCU8_SLICE_GetEvent(HALL_3_HW, CY_CCU8_SLICE_IRQ_ID_PERIOD_MATCH))
        {
            /* Timers are not started and CCU8 pulse counter greater than 3 */
            if ((ccu8_pulse_counter++ > 3) && (!timers_started))
            {
                /* Start the Encoder */
                Cy_POSIF_Start(HALL_POSIF_HW);

                /* Read the Hall input GPIO pins */
                hall[0] = Cy_GPIO_GetInput(HALL_INPUT_1_PORT, HALL_INPUT_1_PIN);
                hall[1] = Cy_GPIO_GetInput(HALL_INPUT_2_PORT, HALL_INPUT_2_PIN);
                hall[2] = Cy_GPIO_GetInput(HALL_INPUT_3_PORT, HALL_INPUT_3_PIN);
                hall_position = (uint8_t)((hall[0] | (hall[1] << 1) | (hall[2] << 2)));

                /* Configure current and expected hall patterns */
                Cy_POSIF_HSC_SetHallPatterns(HALL_POSIF_HW, HALL_POSIF_Hall_Pattern[hall_position ? hall_position : 1]);

                /* Update hall pattern */
                Cy_POSIF_HSC_UpdateHallPattern(HALL_POSIF_HW);

                /* Start CCU4 timers */
                Cy_CCU4_SLICE_StartTimer(HALL_DELAY_TIMER_HW);
                Cy_CCU4_SLICE_StartTimer(HALL_SPEED_TIMER_HW);

                /* Sets the timers flag to the true value */
                timers_started = true;
            }
            Cy_CCU8_SLICE_ClearEvent(HALL_3_HW, CY_CCU8_SLICE_IRQ_ID_PERIOD_MATCH);
        }

        /* Delay and Speed timers are started */
        if (timers_started)
        {
            /* Read the Hall input GPIO pins */
            hall[0] = Cy_GPIO_GetInput(HALL_INPUT_1_PORT, HALL_INPUT_1_PIN);
            hall[1] = Cy_GPIO_GetInput(HALL_INPUT_2_PORT, HALL_INPUT_2_PIN);
            hall[2] = Cy_GPIO_GetInput(HALL_INPUT_3_PORT, HALL_INPUT_3_PIN);
            hall_position = (uint8_t)((hall[0] | (hall[1] << 1) | (hall[2] << 2)));

            /* Configure current and expected hall patterns */
            Cy_POSIF_HSC_SetHallPatterns(HALL_POSIF_HW, HALL_POSIF_Hall_Pattern[hall_position ? hall_position : 1]);

            /* Update hall pattern */
            Cy_POSIF_HSC_UpdateHallPattern(HALL_POSIF_HW);
        }
    }
}
