/*
 * Copyright (c) 2021-2022 Amlogic, Inc. All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef __AML_PORTABLE_EXT_H__
#define __AML_PORTABLE_EXT_H__

#include <stdint.h>

typedef enum Halt_Action{
	HLTACT_RUN_OS=0,
	HLTACT_SHUTDOWN_SYSTEM
}Halt_Action_e;

#ifdef CONFIG_ARM64
static inline unsigned long _irq_save(void)
{
	unsigned long flags;

	asm volatile(
		"mrs	%0, daif		// arch_local_irq_save\n"
		"msr	daifset, #2"
		: "=r" (flags)
		:
		: "memory");

	return flags;
}
static inline void _irq_restore(unsigned long flags)
{
	asm volatile(
		"msr	daif, %0		// arch_local_irq_restore"
	:
	: "r" (flags)
	: "memory");
}

#define portIRQ_SAVE(flags) 			\
	do {								\
		flags = _irq_save();			\
	} while (0)

#define portIRQ_RESTORE(flags)			\
	do {								\
		_irq_restore(flags);			\
	} while (0)
#else

#define portIRQ_SAVE(a)		(void)(a)
#define portIRQ_RESTORE(a)	(void)(a)

#define portIRQ_SAVE(a)		(void)(a)
#define portIRQ_RESTORE(a)	(void)(a)

#endif

void vLowPowerSystem(void);

uint8_t xPortIsIsrContext( void );

void vPortAddIrq(uint32_t irq_num);

void vPortRemoveIrq(uint32_t irq_num);

void vPortRtosInfoUpdateStatus(uint32_t status);

void vPortHaltSystem(Halt_Action_e act);

#endif
