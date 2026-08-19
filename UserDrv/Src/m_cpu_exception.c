/******************************************************************************
 * @file m_cpu_exception.c
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

/*****************************************************************************/
/** INCLUDES *****************************************************************/
/*****************************************************************************/

#include "UserDrivers.h"
#include <setjmp.h>

/*****************************************************************************/
/** MACRO DEFINITIONS ********************************************************/
/*****************************************************************************/

#define CFSR_REGISTER 0xE000ED28


/*****************************************************************************/
/** DATATYPES ****************************************************************/
/*****************************************************************************/

typedef struct RegContexttag
{
	uintptr_t IP;
	uintptr_t BP;
	uintptr_t SP;
} RegContext;

typedef struct tagSEHCOntext
{
	struct tagSEHCOntext *pNext;
	uint32_t    ui32Pattern;
	uint32_t    ui32ExceptionCode;
	jmp_buf jmpbuf;
	RegContext  context;
	bool    bHandled;
	bool    bRegistered;
	void *	fpenvbuf;
} SEHContext;

/*****************************************************************************/
/** FUNCTION DECLARATIONS ****************************************************/
/*****************************************************************************/

/*****************************************************************************/
/** EXTERNAL VARIABLES *******************************************************/
/*****************************************************************************/

/*****************************************************************************/
/** LOCAL VARIABLES **********************************************************/
/*****************************************************************************/

/*****************************************************************************/
/** GLOBAL VARIABLES *********************************************************/
/*****************************************************************************/

int* pBPStoredInException;
int* pSPStoredInException;
int* pIPStoredInException;

jmp_buf gContext;
jmp_buf giParam;

static RegContext s_RegContext;
static uint32_t exceptionCode;

/*****************************************************************************/
/** FUNCTION DEFINITIONS *****************************************************/
/*****************************************************************************/

/******************************************************************************
 * @brief 
 * 
 *****************************************************************************/
void SysExceptHandler(void)
{
	uintptr_t ulCFSRValue;
	s_RegContext.IP = (uintptr_t)pIPStoredInException;
	s_RegContext.BP = (uintptr_t)pBPStoredInException;
	s_RegContext.SP = (uintptr_t)pSPStoredInException;

	ulCFSRValue = *(uintptr_t*)CFSR_REGISTER;

	/* Following CSFR bits are not matched by below code, thus
	   they will lead to exception code EXCPT_UNKNOWN

	CSFR.USFR.INVSTATE: 16+1
	CSFR.USFR.NOCP : 16+3
	CSFR.USFR reserved : 16+4 ... 16+7
	CSFR.USFR reserved : 16+10 ... 16+15

	CSFR.BSFR.IMPRECISERR : 8+2
	CSFR.BSFR.UNSTKERR : 8+3
	CSFR.BSFR.STKERR : 8+4
	CSFR.BSFR.LSPERR : 8+5
	CSFR.BSFR reserved: 8+6
	CSFR.BSFR.BFARVALID : 8+7 //address valid, not an error code

	MMFSR reserved: 0+2
	MMFSR.MLSPERR : 0+5
	MMFSR reserved: 0+6
	MMFSR.MMARVALID : 0+7 //address valid, not an error code
	*/
	if (ulCFSRValue & (1 << (16+9)))
	{
		// CSFR.USFR.DIVBYZERO
		*(uintptr_t*)CFSR_REGISTER = 1 << (16+9); /* clear bit */
		exceptionCode = EXCPT_DIVIDEBYZERO;
	}
	else if (ulCFSRValue & (1 << (16+8)))
	{
		// CSFR.USFR.UNALIGNED
		*(uintptr_t*)CFSR_REGISTER = 1 << (16+8); /* clear bit */
		exceptionCode = EXCPT_MISALIGNMENT;
	}
	else if (ulCFSRValue & (1 << (16+2)))
	{
		// CSFR.USFR.INVPC
		*(uintptr_t*)CFSR_REGISTER = 1 << (16+2); /* clear bit */
		exceptionCode = EXCPT_ILLEGAL_INSTRUCTION;
	}
	else if (ulCFSRValue & (1 << (16+0)))
	{
		// CSFR.USFR.UNDEFINSTR
		*(uintptr_t*)CFSR_REGISTER = 1 << (16+0); /* clear bit */
		exceptionCode = EXCPT_ILLEGAL_INSTRUCTION;
	}
	else if (ulCFSRValue & (1 << (8+1)))
	{
		// CSFR.BFSR.PRECISERR
		// e.g. bus access to invalid address
		*(uintptr_t*)CFSR_REGISTER = 1 << (8+1); /* clear bit */
		exceptionCode = EXCPT_ACCESS_VIOLATION;
	}
	else if (ulCFSRValue & (1 << (8+0)))
	{
		// CSFR.BFSR.IBUSERR
		// e.g. instruction fetch bus access to invalid address
		*(uintptr_t*)CFSR_REGISTER = 1 << (8+0); /* clear bit */
		exceptionCode = EXCPT_ACCESS_VIOLATION;
	}
	else if (ulCFSRValue & (1 << 4))
	{
		// CSFR.MMFSR.MSTKERR
		*(uintptr_t*)CFSR_REGISTER = 1 << 4; /* clear bit */
		exceptionCode = EXCPT_ACCESS_VIOLATION;
	}
	else if (ulCFSRValue & (1 << 3))
	{
		// CSFR.MMFSR.MUNSTKERR
		*(uintptr_t*)CFSR_REGISTER = 1 << 3; /* clear bit */
		exceptionCode = EXCPT_ACCESS_VIOLATION;
	}
	else if (ulCFSRValue & (1 << 1))
	{
		// CSFR.MMFSR.DACCVIOL
		*(uintptr_t*)CFSR_REGISTER = 1 << 1; /* clear bit */
		exceptionCode = EXCPT_ACCESS_VIOLATION;
	}
	else if (ulCFSRValue & (1 << 0))
	{
		// CSFR.MMFSR.IACCVIOL
		*(uintptr_t*)CFSR_REGISTER = 1 << 0; /* clear bit */
		exceptionCode = EXCPT_ACCESS_VIOLATION;
	}
	else
	{
		exceptionCode = EXCPT_UNKNOWN;
	}

	if ( ( ( exceptionCode >= EXCPT_ILLEGAL_INSTRUCTION ) && ( exceptionCode <= EXCPT_NONCONTINUABLE ) ) ||
		( ( exceptionCode >= EXCPT_FPU_ERROR ) && ( exceptionCode <= EXCPT_FPU_UNDERFLOW ) ) )
	{
		/* hard fault: pass exception to potentially registered exception handlers */
		/* check if an exception frame was registered and if this frame was not overwritten */
//		(*ppSEHContext)->context = s_RegContext;
//		longjmp((*ppSEHContext)->jmpbuf, exceptionCode);

		EEPR.Write( 0x798, &exceptionCode, 4 );

	}
	NVIC_SystemReset();
}
