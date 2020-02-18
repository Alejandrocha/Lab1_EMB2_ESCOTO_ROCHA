/**
 * @file rtos.c
 * @author ITESO
 * @date Feb 2018
 * @brief Implementation of rtos API
 *
 * This is the implementation of the rtos module for the
 * embedded systems II course at ITESO
 */

#include "rtos.h"
#include "rtos_config.h"
#include "clock_config.h"

#ifdef RTOS_ENABLE_IS_ALIVE
#include "fsl_gpio.h"
#include "fsl_port.h"
#endif
/**********************************************************************************/
// Module defines
/**********************************************************************************/

#define FORCE_INLINE 	__attribute__((always_inline)) inline

#define STACK_FRAME_SIZE			8
#define STACK_LR_OFFSET				2
#define STACK_PSR_OFFSET			1
#define STACK_PC_OFFSET				3
#define STACK_PSR_DEFAULT			0x01000000

/**********************************************************************************/
// IS ALIVE definitions
/**********************************************************************************/

#ifdef RTOS_ENABLE_IS_ALIVE
#define CAT_STRING(x,y)  		x##y
#define alive_GPIO(x)			CAT_STRING(GPIO,x)
#define alive_PORT(x)			CAT_STRING(PORT,x)
#define alive_CLOCK(x)			CAT_STRING(kCLOCK_Port,x)
static void init_is_alive(void);
static void refresh_is_alive(void);
#endif

/**********************************************************************************/
// Type definitions
/**********************************************************************************/

typedef enum
{
	S_READY = 0, S_RUNNING, S_WAITING, S_SUSPENDED
} task_state_e;
typedef enum
{
	kFromISR = 0, kFromNormalExec
} task_switch_type_e;

typedef struct
{
	uint8_t priority;
	task_state_e state;
	uint32_t *sp;
	void (*task_body)();
	rtos_tick_t local_tick;
	uint32_t reserved[10];
	uint32_t stack[RTOS_STACK_SIZE];
} rtos_tcb_t;

/**********************************************************************************/
// Global (static) task list
/**********************************************************************************/

struct
{
	uint8_t nTasks;
	rtos_task_handle_t current_task;
	rtos_task_handle_t next_task;
	rtos_tcb_t tasks[RTOS_MAX_NUMBER_OF_TASKS + 1];
	rtos_tick_t global_tick;
} task_list =
{ 0 };

/**********************************************************************************/
// Local methods prototypes
/**********************************************************************************/

static void reload_systick(void);
static void dispatcher(task_switch_type_e type);
static void activate_waiting_tasks();
FORCE_INLINE static void context_switch(task_switch_type_e type);
static void idle_task(void);

/**********************************************************************************/
// API implementation
/**********************************************************************************/

void rtos_start_scheduler(void)
{
#ifdef RTOS_ENABLE_IS_ALIVE
	init_is_alive();
#endif
	SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk
	        | SysTick_CTRL_ENABLE_Msk;
	reload_systick();
	task_list.global_tick = 0;
	rtos_create_task(idle_task, 0, kAutoStart);
	for (;;);
}

rtos_task_handle_t rtos_create_task(void (*task_body)(), uint8_t priority,rtos_autostart_e autostart)
{
	//register uint32_t PCR_task asm("xpsr");
	if(task_list.nTasks<=RTOS_MAX_NUMBER_OF_TASKS){
		if(autostart == kAutoStart)
			task_list.tasks[task_list.nTasks].state = S_READY;
		else
			task_list.tasks[task_list.nTasks].state = S_SUSPENDED;
		task_list.tasks[task_list.nTasks].local_tick = 0;
		task_list.tasks[task_list.nTasks].priority = priority;
		task_list.tasks[task_list.nTasks].task_body = task_body;
		task_list.tasks[task_list.nTasks].sp = &(task_list.tasks[task_list.nTasks].stack[0]);
		task_list.tasks[task_list.nTasks].sp += (RTOS_STACK_SIZE-1);
		task_list.tasks[task_list.nTasks].sp -= (STACK_FRAME_SIZE);
		task_list.tasks[task_list.nTasks].stack[RTOS_STACK_SIZE-STACK_PSR_OFFSET] = (uint32_t)STACK_PSR_DEFAULT;
		task_list.tasks[task_list.nTasks].stack[RTOS_STACK_SIZE-STACK_LR_OFFSET] = (uint32_t)task_body;
		task_list.tasks[task_list.nTasks].stack[RTOS_STACK_SIZE-STACK_PC_OFFSET] = (uint32_t)task_body;

		task_list.nTasks++;
		return task_list.nTasks-1;
	}else
		return -1;
/*!
 * IF TASK_SPACE NOT EMPTY
 * 	IF AUTOSTART
 * 		TASK READY
 *	---
 * .SUSPEND TASK
 * .MOVE SP TO TASK STACK INIT
 * .BASE CONTEXT PARAMETERS
 * .LOCTICK = 0
 */
}

rtos_tick_t rtos_get_clock(void)
{
	/*!
	 * GET SYS_CLK VALUE
	 */
	return SysTick->VAL;
}

void rtos_delay(rtos_tick_t ticks)
{

	task_list.tasks[task_list.current_task].state= S_WAITING;
	task_list.tasks[task_list.current_task].local_tick = ticks;
	dispatcher(kFromNormalExec);
	/*!
	 * SET CALLING TASK TO SLEEP ticks TICKS
	 * ASSIGN LOCAL TICKS
	 * CALL DISPATCHER
	 */

}

void rtos_suspend_task(void)
{
	task_list.tasks[task_list.current_task].state = S_SUSPENDED;
	dispatcher(kFromNormalExec);
/*!
 * SUSPEND CALLING TASK
 * CALL DISPATCHER
 */
}

void rtos_activate_task(rtos_task_handle_t task)
{
	/*!
	 * SET CALLING TASK TO ACTIVE
	 * CALL DISPATCHER
	 */
	task_list.tasks[task].state= S_READY;
	dispatcher(kFromNormalExec);

}

/**********************************************************************************/
// Local methods implementation
/**********************************************************************************/

static void reload_systick(void)
{
	SysTick->LOAD = USEC_TO_COUNT(RTOS_TIC_PERIOD_IN_US,
	        CLOCK_GetCoreSysClkFreq());
	SysTick->VAL = 0;
}

static void dispatcher(task_switch_type_e type)
{
	uint8_t highest_priority = kPrio0;
	uint8_t ltask;
	for(ltask = 0; ltask<task_list.nTasks; ltask++)
	{
		if(task_list.tasks[ltask].task_body == idle_task)
		{
			task_list.next_task = ltask;
		}
	}
	for(ltask = 0;ltask<task_list.nTasks;ltask++)
	{
		if((task_list.tasks[ltask].priority >= highest_priority)
				&& ((task_list.tasks[ltask].state == S_READY)
						||(task_list.tasks[ltask].state == S_RUNNING)))
		{
			highest_priority = task_list.tasks[ltask].priority;
			task_list.next_task = ltask;
		}
	}
	if(task_list.current_task != task_list.next_task)
		context_switch(type);

/*!
 * SET NEXT TASK TO IDLE
 * SET HIGH PRI
 *
 * FOR EACH TASK
 * 	IF HIGHEST PRI AND (ACTIVE OR RUNNING)
 * 		HIGHEST PRI = TASK PRI
 * 		NEXT TASK = TASK
 * 	END
 * 	IF NEXT TASK != CURRENT TASK
 * 		CHANGE CONTEXT
 * 	END
 *
 */
}

FORCE_INLINE static void context_switch(task_switch_type_e type)
{
	static uint8_t first_time_flag = 0;
	register uint32_t SP_Task asm("r0");
	(void)SP_Task;
	if(first_time_flag == 0)
		first_time_flag = 1;
	else
	{

		__asm ("mov r0, r7");
		if(type == kFromNormalExec){
			task_list.tasks[task_list.current_task].sp = (uint32_t *)(SP_Task);
			task_list.tasks[task_list.current_task].sp += -9;
		}
		else{
			task_list.tasks[task_list.current_task].sp = (uint32_t *)(SP_Task );
			task_list.tasks[task_list.current_task].sp -= -11;
			}
//		task_list.tasks[task_list.current_task].sp -= STACK_FRAME_SIZE;
	}
	task_list.current_task = task_list.next_task;
	task_list.tasks[task_list.next_task].state = S_RUNNING;
	SCB->ICSR |= SCB_ICSR_PENDSVSET_Msk;

/*!
 * IF NOT FIRST TASK
 *		SAVE CURRENT SP IN CURRENT TASK STACK
 * END
 * CHANGE CURRENT TASK FOR NEXT TASK
 * SET NEXT TASK TO RUNNING
 * INVOKE PendSV
 */

}

static void activate_waiting_tasks()
{
	uint8_t tasks;
	for(tasks = 0; tasks < task_list.nTasks; tasks++)
	{
		if(task_list.tasks[tasks].state == S_WAITING)
		{
			task_list.tasks[tasks].local_tick--;
			if(task_list.tasks[tasks].local_tick == 0)
				task_list.tasks[tasks].state = S_READY;
		}
	}
/*!
 * FOR ALL TASKS
 * 	IF WAITING TASK
 * 		LOCAL TICK -1
 * 		IF LOCAL TICK = 0
 * 			READY TASK
 * 		END
 * 	END
 * END
 */

}

/**********************************************************************************/
// IDLE TASK
/**********************************************************************************/

static void idle_task(void)
{
	for (;;)
	{

	}
}

/****************************************************/
// ISR implementation
/****************************************************/

void SysTick_Handler(void)
{
#ifdef RTOS_ENABLE_IS_ALIVE
	refresh_is_alive();
#endif
	task_list.global_tick++;
	activate_waiting_tasks();
	reload_systick();
	dispatcher(kFromISR);
}

void PendSV_Handler(void)
{
	register uint32_t SP_Task asm("r0");
	(void)SP_Task;
	SCB->ICSR |= SCB_ICSR_PENDSVCLR_Msk;
	SP_Task = (uint32_t)task_list.tasks[task_list.current_task].sp;

	__asm ("mov r7, r0");

/*!
 *  LOAD CPU SP WITH CURRENT TASK SP
 *
 */
}

/**********************************************************************************/
// IS ALIVE SIGNAL IMPLEMENTATION
/**********************************************************************************/

#ifdef RTOS_ENABLE_IS_ALIVE
static void init_is_alive(void)
{
	gpio_pin_config_t gpio_config =
	{ kGPIO_DigitalOutput, 1, };

	port_pin_config_t port_config =
	{ kPORT_PullDisable, kPORT_FastSlewRate, kPORT_PassiveFilterDisable,
	        kPORT_OpenDrainDisable, kPORT_LowDriveStrength, kPORT_MuxAsGpio,
	        kPORT_UnlockRegister, };
	CLOCK_EnableClock(alive_CLOCK(RTOS_IS_ALIVE_PORT));
	PORT_SetPinConfig(alive_PORT(RTOS_IS_ALIVE_PORT), RTOS_IS_ALIVE_PIN,
	        &port_config);
	GPIO_PinInit(alive_GPIO(RTOS_IS_ALIVE_PORT), RTOS_IS_ALIVE_PIN,
	        &gpio_config);
}

static void refresh_is_alive(void)
{
	static uint8_t state = 0;
	static uint32_t count = 0;
	SysTick->LOAD = USEC_TO_COUNT(RTOS_TIC_PERIOD_IN_US,
	        CLOCK_GetCoreSysClkFreq());
	SysTick->VAL = 0;
	if (RTOS_IS_ALIVE_PERIOD_IN_US / RTOS_TIC_PERIOD_IN_US - 1 == count)
	{
		GPIO_PinWrite(alive_GPIO(RTOS_IS_ALIVE_PORT), RTOS_IS_ALIVE_PIN,
		        state);
		state = state == 0 ? 1 : 0;
		count = 0;
	} else //
	{
		count++;
	}
}
#endif
///
