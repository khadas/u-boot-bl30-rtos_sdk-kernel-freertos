
/*
 * FreeRTOS Kernel V10.0.0
 * Copyright (C) 2017 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software. If you wish to use our Amazon
 * FreeRTOS name, please do so in a fair use way that does not cause confusion.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 *
 * 1 tab == 4 spaces!
 */

#include <stdlib.h>
#include <xtensa/config/core.h>
#include "xtbsp.h"
#include "xtensa_rtos.h"
#include "FreeRTOS.h"
#include "task.h"

/* Defined in xtensa_context.S */
extern void _xt_coproc_init(void);

extern BaseType_t xPortSysTickHandler( void );

/*-----------------------------------------------------------*/

/* We require the address of the pxCurrentTCB variable, but don't want to know
any details of its type. */
typedef void TCB_t;
extern volatile TCB_t * volatile pxCurrentTCB;

unsigned port_xSchedulerRunning = 0; // Duplicate of inaccessible xSchedulerRunning; needed at startup to avoid counting nesting
unsigned port_interruptNesting = 0;  // Interrupt nesting level

/*-----------------------------------------------------------*/

// User exception dispatcher when exiting
void _xt_user_exit(void);

/*
 * Stack initialization
 */
#if portUSING_MPU_WRAPPERS
StackType_t *pxPortInitialiseStack( StackType_t *pxTopOfStack, TaskFunction_t pxCode, void *pvParameters, BaseType_t xRunPrivileged )
{
	(void) xRunPrivileged;
	StackType_t *sp, *tp;
	XtExcFrame  *frame;
	#if XCHAL_CP_NUM > 0
	uint32_t *p;
	#endif

	/* Create interrupt stack frame aligned to 16 byte boundary */
	sp = (StackType_t *) (((UBaseType_t)(pxTopOfStack + 1) - XT_CP_SIZE - XT_STK_FRMSZ) & ~0xf);

	/* Clear the entire frame (do not use memset() because we don't depend on C library) */
	for (tp = sp; tp <= pxTopOfStack; ++tp)
		*tp = 0;

	frame = (XtExcFrame *) sp;

	/* Explicitly initialize certain saved registers */
	frame->pc   = (UBaseType_t) pxCode;             /* task entrypoint                */
	frame->a0   = 0;                                /* to terminate GDB backtrace     */
	frame->a1   = (UBaseType_t) sp + XT_STK_FRMSZ;  /* physical top of stack frame    */
	frame->exit = (UBaseType_t) _xt_user_exit;      /* user exception exit dispatcher */

	/* Set initial PS to int level 0, EXCM disabled ('rfe' will enable), user mode. */
	/* Also set entry point argument parameter. */
	#ifdef __XTENSA_CALL0_ABI__
	frame->a2 = (UBaseType_t) pvParameters;
	frame->ps = PS_UM | PS_EXCM;
	#else
	/* + for windowed ABI also set WOE and CALLINC (pretend task was 'call4'd). */
	frame->a6 = (UBaseType_t) pvParameters;
	frame->ps = PS_UM | PS_EXCM | PS_WOE | PS_CALLINC(1);
	#endif

	#ifdef XT_USE_SWPRI
	/* Set the initial virtual priority mask value to all 1's. */
	frame->vpri = 0xFFFFFFFF;
	#endif

	#if XCHAL_CP_NUM > 0
	/* Init the coprocessor save area (see xtensa_context.h) */
	/* No access to TCB here, so derive indirectly. Stack growth is top to bottom.
         * //p = (uint32_t *) xMPUSettings->coproc_area;
	 */
	p = (uint32_t *)(((uint32_t) pxTopOfStack - XT_CP_SIZE) & ~0xf);
	p[0] = 0;
	p[1] = 0;
	p[2] = (((uint32_t) p) + 12 + XCHAL_TOTAL_SA_ALIGN - 1) & -XCHAL_TOTAL_SA_ALIGN;
	#endif

	return sp;
}
#else
StackType_t *pxPortInitialiseStack( StackType_t *pxTopOfStack, TaskFunction_t pxCode, void *pvParameters )
{
	StackType_t *sp, *tp;
	XtExcFrame  *frame;
	#if XCHAL_CP_NUM > 0
	uint32_t *p;
	#endif

	/* Create interrupt stack frame aligned to 16 byte boundary */
	sp = (StackType_t *) (((UBaseType_t)(pxTopOfStack + 1) - XT_CP_SIZE - XT_STK_FRMSZ) & ~0xf);

	/* Clear the entire frame (do not use memset() because we don't depend on C library) */
	for (tp = sp; tp <= pxTopOfStack; ++tp)
		*tp = 0;

	frame = (XtExcFrame *) sp;

	/* Explicitly initialize certain saved registers */
	frame->pc   = (UBaseType_t) pxCode;             /* task entrypoint                */
	frame->a0   = 0;                                /* to terminate GDB backtrace     */
	frame->a1   = (UBaseType_t) sp + XT_STK_FRMSZ;  /* physical top of stack frame    */
	frame->exit = (UBaseType_t) _xt_user_exit;      /* user exception exit dispatcher */

	/* Set initial PS to int level 0, EXCM disabled ('rfe' will enable), user mode. */
	/* Also set entry point argument parameter. */
	#ifdef __XTENSA_CALL0_ABI__
	frame->a2 = (UBaseType_t) pvParameters;
	frame->ps = PS_UM | PS_EXCM;
	#else
	/* + for windowed ABI also set WOE and CALLINC (pretend task was 'call4'd). */
	frame->a6 = (UBaseType_t) pvParameters;
	frame->ps = PS_UM | PS_EXCM | PS_WOE | PS_CALLINC(1);
	#endif

	#ifdef XT_USE_SWPRI
	/* Set the initial virtual priority mask value to all 1's. */
	frame->vpri = 0xFFFFFFFF;
	#endif

	#if XCHAL_CP_NUM > 0
	/* Init the coprocessor save area (see xtensa_context.h) */
	/* No access to TCB here, so derive indirectly. Stack growth is top to bottom.
         * //p = (uint32_t *) xMPUSettings->coproc_area;
	 */
	p = (uint32_t *)(((uint32_t) pxTopOfStack - XT_CP_SIZE) & ~0xf);
	p[0] = 0;
	p[1] = 0;
	p[2] = (((uint32_t) p) + 12 + XCHAL_TOTAL_SA_ALIGN - 1) & -XCHAL_TOTAL_SA_ALIGN;
	#endif

	return sp;
}

#endif

/*-----------------------------------------------------------*/

void vPortEndScheduler( void )
{
	/* It is unlikely that the Xtensa port will get stopped.  If required simply
	disable the tick interrupt here. */
}

/*-----------------------------------------------------------*/

BaseType_t xPortStartScheduler( void )
{
	// Interrupts are disabled at this point and stack contains PS with enabled interrupts when task context is restored

	#if XCHAL_CP_NUM > 0
	/* Initialize co-processor management for tasks. Leave CPENABLE alone. */
	_xt_coproc_init();
	#endif

	/* Init the tick divisor value */
	_xt_tick_divisor_init();

	/* Setup the hardware to generate the tick. */
	_frxt_tick_timer_init();

	#if XT_USE_THREAD_SAFE_CLIB
	// Init C library
	vPortClibInit();
	#endif

	port_xSchedulerRunning = 1;

	// Cannot be directly called from C; never returns
	__asm__ volatile ("call0    _frxt_dispatch\n");

	/* Should not get here. */
	return pdTRUE;
}
/*-----------------------------------------------------------*/

BaseType_t xPortSysTickHandler( void )
{
	BaseType_t ret;
	uint32_t interruptMask;

	portbenchmarkIntLatency();

	//printf("excute sys tick hander......\n");
	/* Interrupts upto configMAX_SYSCALL_INTERRUPT_PRIORITY must be
	 * disabled before calling xTaskIncrementTick as it access the
	 * kernel lists. */
	interruptMask = portSET_INTERRUPT_MASK_FROM_ISR();
	{
		ret = xTaskIncrementTick();
	}
	portCLEAR_INTERRUPT_MASK_FROM_ISR( interruptMask );

	/*update the _xt_tick_divisor every tick handler*/
	_xt_tick_divisor = xtbsp_clock_freq_hz() / XT_TICK_PER_SEC;

	portYIELD_FROM_ISR( ret );

	return ret;
}
/*-----------------------------------------------------------*/

/*
 * Used to set coprocessor area in stack. Current hack is to reuse MPU pointer for coprocessor area.
 */
#if portUSING_MPU_WRAPPERS
void vPortStoreTaskMPUSettings( xMPU_SETTINGS *xMPUSettings, const struct xMEMORY_REGION * const xRegions, StackType_t *pxBottomOfStack, uint32_t ulStackDepth )
{
	 const struct xMEMORY_REGION*  xRegions1 ;  //delete warning
	 xRegions1 = xRegions ;
	#if XCHAL_CP_NUM > 0
	//xMPUSettings->coproc_area = (StackType_t*)((((uint32_t)(pxBottomOfStack + ulStackDepth - 1)) - XT_CP_SIZE ) & ~0xf);

	xMPUSettings->coproc_area = (StackType_t *)((uint32_t)(pxBottomOfStack + ulStackDepth - 1));
	xMPUSettings->coproc_area = (StackType_t *)
		(((portPOINTER_SIZE_TYPE)xMPUSettings->coproc_area)
		& (~((portPOINTER_SIZE_TYPE)portBYTE_ALIGNMENT_MASK)));
	xMPUSettings->coproc_area = (StackType_t *)
		(((uint32_t)xMPUSettings->coproc_area - XT_CP_SIZE) & ~0xf);

	/* NOTE: we cannot initialize the coprocessor save area here because FreeRTOS is going to
         * clear the stack area after we return. This is done in pxPortInitialiseStack().
	 */
	#endif
}
#endif

/*
 * Hifi tickless mode support
 */

#if (configUSE_TICKLESS_IDLE == 1)

uint32_t ulTimerCountsForOneTick = 1000U;

/*xMaximumPossibleSuppressedTicks get max ticks */
uint32_t xMaximumPossibleSuppressedTicks(void)
{
	//return (uint32_t)(0xffffffff/Hificlkval * configTICK_RATE_HZ);
	//return (uint32_t)((double)(0xffffffff/configCPU_CLOCK_HZ) * configTICK_RATE_HZ);
	return (uint32_t)(0xffffffff/_xt_tick_divisor);
}

void prvSleep( TickType_t xExpectedIdleTime )
{
	/* Allow the application to define some pre-sleep processing. */
	configPRE_SLEEP_PROCESSING( xExpectedIdleTime );

	/* xExpectedIdleTime being set to 0 by configPRE_SLEEP_PROCESSING()
	means the application defined code has already executed the WAIT
	instruction. */

	if ( xExpectedIdleTime > 0 ) {
		//printf("WAITI...\n");
		__asm__ volatile ("WAITI 0");
	}

	/* Allow the application to define some post sleep processing. */
	configPOST_SLEEP_PROCESSING( xExpectedIdleTime );
}

__attribute__((optimize ("-O0")))
void vPortSuppressTicksAndSleep( TickType_t xExpectedIdleTime )
{
	uint32_t ulLowPowerTimeBeforeSleep, ulLowPowerTimeAfterSleep, ulDuration;
	eSleepModeStatus eSleepStatus;
	uint32_t ulReloadValue = 0;

	if (xExpectedIdleTime > xMaximumPossibleSuppressedTicks())
		xExpectedIdleTime = xMaximumPossibleSuppressedTicks();

	/* read the current time from timere that will remain opereational
	   while the microcontroller is in a low power state. */
	//ulLowPowerTimeBeforeSleep = xHwClockSourceRead();
	//printf("ulLowPowerTimeBeforeSleep : %d\n" ,ulLowPowerTimeBeforeSleep);
	ulLowPowerTimeBeforeSleep = xthal_get_ccount();

	/* Stop the timer that is generating the tick interrupt. */
	prvStopSysTickInterruptTimer();
	__asm__ volatile ("esync ");

	/* enter a critical section that will not effect inerrupts bringing
	 the MCU out of sleep mode. */
	//portDISABLE_INTERRUPTS();
	xt_set_intlevel(XCHAL_EXCM_LEVEL);

	//printf("_xt_tick_divisor : %u\n", _xt_tick_divisor);
	ulReloadValue = xExpectedIdleTime * _xt_tick_divisor;
	//printf("now clockcycle:0x%08x,  need set time : 0x%08x\n" ,xthal_get_ccount(), ulReloadValue);

	/* ensure it is still ok to enter the sleep mode. */
	eSleepStatus = eTaskConfirmSleepModeStatus();
	if ( eSleepStatus == eAbortSleep ) {
		/* a task has been woved out of the blocked state since this
		 macro was executed, or a context siwth is being held pending.
		 do not enter a sleep state, restart the tick and exit the critical
		 section. */
		//ulLowPowerTimeAfterSleep = xHwClockSourceRead();
		prvEnableSysTickInterruptTimer();
		portENABLE_INTERRUPTS();
		__asm__ volatile ("esync ");
	}
	else {
		if ( eSleepStatus == eNoTasksWaitingTimeout ) {
			//printf(" enter task waiting timeout ==== %u\n", eNoTasksWaitingTimeout);
			/* it is not necessary to configure an interrupt to bring
			   the microcontroller out of its low power state at a fixed
			   time in the future. */
			prvSleep(xExpectedIdleTime);
		} else {
			/* configure an interrupt to bring the microcontroller
			   out of its low power state at the time the kernel next
			   needs to execute. the interrupt must be generated from
			   a source that remains operational when the microcontroller is
			   in a low power state. */
			/***
			//1) timer overflow check
			//2) need make sure the sleep time is times of _xt_tick_divisor
			if (ulReloadValue + xthal_get_ccount() < ulReloadValue)
				ulReloadValue =((uint32_t)((0xffffffff - ulReloadValue)/_xt_tick_divisor) )* _xt_tick_divisor;
			***/
			//printf("timer set time:%u\n", ulReloadValue);
			vSetWakeTimeInterrupt( ulReloadValue );
			__asm__ volatile ("esync ");
			//portENABLE_INTERRUPTS();
			ulLowPowerTimeBeforeSleep = xthal_get_ccount();
			//printf("ulLowPowerTimeBeforeSleep : %u\n" ,ulLowPowerTimeBeforeSleep);
			/* enter the low power state. */
			prvSleep(xExpectedIdleTime);
		}
		/*determine how long the microcontroller was acrually in a low power
		  state for, which will be less than xExpectedIdleTime if the
		  microcontroller was brought out of low power mode by an interrupt
		  other than that configured by vSetWakeTimeInterrupt() call. Note
		  that the scheduler is sucpended before portSUPPRESS_TICKS_AND_SLEEP()
		  is called, and resumed when portSUPPRESS_TICKS_AND_SLEEP() returns. therefore
		  no other tasks will execute until this function completes. */

		//ulLowPowerTimeAfterSleep = xHwClockSourceRead();
		ulLowPowerTimeAfterSleep = xthal_get_ccount();
		//printf("ulLowPowerTimeAfterSleep : %u\n",ulLowPowerTimeAfterSleep);
		/* correct the kernels tick count to accunt for the time the microcontroller
		   spent in its low power state. */
		if (ulLowPowerTimeAfterSleep < ulLowPowerTimeBeforeSleep)
			ulDuration =  0xffffffff - ulLowPowerTimeBeforeSleep + ulLowPowerTimeAfterSleep;
		else
			ulDuration = ulLowPowerTimeAfterSleep - ulLowPowerTimeBeforeSleep;
		//printf("ulDuration : %u\n", ulDuration );

		//vTaskStepTick( (TickType_t)((double)((long long)(ulDuration * configTICK_RATE_HZ)/1000000 )));
		vTaskStepTick( (TickType_t)(ulDuration/_xt_tick_divisor));
	}
		portENABLE_INTERRUPTS();
		/* restart the timer that is generating the tick interrupt. */
		prvStartTickInterruptTimer(_xt_tick_divisor);
		__asm__ volatile ("esync ");
}

#endif /*configUSE_TICKLESS_IDLE*/

