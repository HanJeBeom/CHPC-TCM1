/******************************************************************************
 * @file hardfault.s
 * @author Lee Jinyoung (jylee1@gst-in.com)
 * @brief 
 * @version 0.1
 * @date 2023-10-05
 * 
 * @copyright Copyright (c) 2023 Global Standard Technology Co., Ltd.
 * 
 * All rights reserved.
 * This file is part of closed source software.
 * Unauthorized reproduction and redistribution prohibited.
 *****************************************************************************/

	.syntax unified	// required, otherwise the register r8-r12 are unknown
						// this directive sets the Instruction Set Syntax as described in the ARM-Instruction-Set section
	.cpu cortex-m4	// define CPU
	.thumb

/******************************************************************************
 * @brief 
 * 
 *****************************************************************************/
	.global  HardFault_Handler
	.extern  HardFault_Trampoline
	.extern  pIPStoredInException
	.extern  pBPStoredInException

	.section  .text.HardFault_Handler
	.type  HardFault_Handler, %function
HardFault_Handler:
	tst lr, #4
	ite eq
	mrseq r10, msp
	mrsne r10, psp

	// SysExceptHandler needs the following parameters:
	// pIPStoredInException: stacked_pc is stored at offset 0x18 from address in r0
	// pBPStoredInException: r7
	// pSPStoredInException: Stack Pointer

	// store r7 to pBP
	ldr r12, =pBPStoredInException
	str r7, [r12]

	// read IP from exception frame
	ldr r11,[r10,#0x18]
	ldr r12, =pIPStoredInException
	str r11, [r12]

	// set return address to trampoline function and exit HardFault context
	ldr r11,=HardFault_Trampoline
	str r11,[r10,#0x18]
	bx lr

/******************************************************************************
 * @brief 
 * 
 *****************************************************************************/
	.global  HardFault_Trampoline
	.extern  SysExceptHandler
	.extern  pSPStoredInException
	.extern  gContext
	.extern  longjmp
	.extern  giParam

	.section  .text.HardFault_Trampoline
	.type  HardFault_Trampoline, %function
HardFault_Trampoline:
	// store pSP from current stack pointer
	mov r11, sp
	ldr r12, =pSPStoredInException
	str r11, [r12]

	// call exception handler => SysExceptGenerate exception
	ldr r4,=SysExceptHandler
	blx r4

	ldr r0, =gContext
	ldr r1, =giParam

	bl.w 	longjmp

