/******************************************************************************
 * The MIT License
 *
 * Copyright (c) 2011 Perry Hung.
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use, copy,
 * modify, merge, publish, distribute, sublicense, and/or sell copies
 * of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *****************************************************************************/

/* STM32F1 performance line ISR weak declarations */

#include <libmaple/stm32.h>

	.thumb

/* Default handler for all non-overridden interrupts and exceptions */
	.globl	__default_handler
	.type	__default_handler, %function

__default_handler:
	b .

	.weak	__exc_nmi
	.globl	__exc_nmi
	.set	__exc_nmi, __default_handler
	.weak	__exc_hardfault
	.globl	__exc_hardfault
	.set	__exc_hardfault, __default_handler
	.weak	__exc_memmanage
	.globl	__exc_memmanage
	.set	__exc_memmanage, __default_handler
	.weak	__exc_busfault
	.globl	__exc_busfault
	.set	__exc_busfault, __default_handler
	.weak	__exc_usagefault
	.globl	__exc_usagefault
	.set	__exc_usagefault, __default_handler
	.weak	__stm32reservedexception7
	.globl	__stm32reservedexception7
	.set	__stm32reservedexception7, __default_handler
	.weak	__stm32reservedexception8
	.globl	__stm32reservedexception8
	.set	__stm32reservedexception8, __default_handler
	.weak	__stm32reservedexception9
	.globl	__stm32reservedexception9
	.set	__stm32reservedexception9, __default_handler
	.weak	__stm32reservedexception10
	.globl	__stm32reservedexception10
	.set	__stm32reservedexception10, __default_handler
	.weak	__exc_svc
	.globl	__exc_svc
	.set	__exc_svc, __default_handler
	.weak	__exc_debug_monitor
	.globl	__exc_debug_monitor
	.set	__exc_debug_monitor, __default_handler
	.weak	__stm32reservedexception13
	.globl	__stm32reservedexception13
	.set	__stm32reservedexception13, __default_handler
	.weak	__exc_pendsv
	.globl	__exc_pendsv
	.set	__exc_pendsv, __default_handler
	.weak	__irq_wwdg
	.globl	__irq_wwdg
	.set	__irq_wwdg, __default_handler
	.weak	__irq_pvd
	.globl	__irq_pvd
	.set	__irq_pvd, __default_handler
	.weak	__irq_tamper
	.globl	__irq_tamper
	.set	__irq_tamper, __default_handler
	.weak	__irq_flash
	.globl	__irq_flash
	.set	__irq_flash, __default_handler
	.weak	__irq_rcc
	.globl	__irq_rcc
	.set	__irq_rcc, __default_handler
	.weak	__irq_usb_hp_can_tx
	.globl	__irq_usb_hp_can_tx
	.set	__irq_usb_hp_can_tx, __default_handler
	.weak	__irq_can_rx1
	.globl	__irq_can_rx1
	.set	__irq_can_rx1, __default_handler
	.weak	__irq_can_sce
	.globl	__irq_can_sce
	.set	__irq_can_sce, __default_handler
	.weak	__irq_spi1
	.globl	__irq_spi1
	.set	__irq_spi1, __default_handler
	.weak	__irq_spi2
	.globl	__irq_spi2
	.set	__irq_spi2, __default_handler
	.weak	__irq_usbwakeup
	.globl	__irq_usbwakeup
	.set	__irq_usbwakeup, __default_handler
#if defined (STM32_HIGH_DENSITY)
	.weak	__irq_fsmc
	.globl	__irq_fsmc
	.set	__irq_fsmc, __default_handler
	.weak	__irq_sdio
	.globl	__irq_sdio
	.set	__irq_sdio, __default_handler
	.weak	__irq_spi3
	.globl	__irq_spi3
	.set	__irq_spi3, __default_handler
#endif /* STM32_HIGH_DENSITY */