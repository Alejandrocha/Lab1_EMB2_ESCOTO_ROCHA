/*
 *
 */
 
/**
 * @file    Lab_1Code.c
 * @brief   Application entry point.
 * TODO -
 */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MK64F12.h"
#include "fsl_debug_console.h"
/* TODO: insert other include files here. */
#include "rtos.h"

/* TODO: insert other definitions and declarations here. */
void dummy_task1(void)
{
	uint32_t counter = 0;
	for (;;)
	{

		PRINTF("IN TASK 1: %i +++++++++++++++\r\n", counter);
		counter++;
		rtos_delay(2000);
//		for(uint32_t a=0;a<0xFFFF;a++);
	}
}

void dummy_task2(void)
{
	uint32_t counter = 0;
	for (;;)
	{
		PRINTF("IN TASK 2: %i ***************\r\n", counter);
		counter++;
		rtos_delay(1000);
//		for(uint32_t a=0;a<0xFFFF;a++);
	}
}

void dummy_task3(void)
{
	uint8_t counter = 0;
	for (;;)
	{
		PRINTF("IN TASK 3: %i ---------------\r\n", counter);
		counter++;
		rtos_delay(4000);
//		for(uint32_t a=0;a<0xFFFF;a++);

	}
}

/*
 * @brief   Application entry point.
 */
int main(void) {

  	/* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
  	/* Init FSL debug console. */
    BOARD_InitDebugConsole();

    PRINTF("Hello World\n\r");

	rtos_create_task(dummy_task1, kPrio1, kAutoStart);
	rtos_create_task(dummy_task2, kPrio2, kAutoStart);
	rtos_create_task(dummy_task3, kPrio1, kAutoStart);
	rtos_start_scheduler();

	for (;;)
	{
		__asm("NOP");
	}
}
