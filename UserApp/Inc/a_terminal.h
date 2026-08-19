/******************************************************************************
 * @file a_terminal.h
 * @author jylee1 (jylee1@gst-in.com)
 * @brief 
 * @version 0.1
 * @date 2023-06-27
 * 
 * @copyright Copyright (c) 2023 Global Standard Technology Co., Ltd.
 * 
 * All rights reserved.
 * This file is part of closed source software.
 * Unauthorized reproduction and redistribution prohibited.
 *****************************************************************************/

#ifndef _A_TERMINAL_H_
#define _A_TERMINAL_H_

#ifdef __cplusplus
extern "C" {
#endif

/*****************************************************************************/
/** INCLUDES *****************************************************************/
/*****************************************************************************/

/*****************************************************************************/
/** MACRO DEFINITIONS ********************************************************/
/*****************************************************************************/

#define VT100_CURSOR_POS_SAVE		"\033[s"
#define VT100_CURSOR_POS_RESTORE	"\033[u"

#define VT100_CURSOR_UP(x)			"\033["#x"A"
#define VT100_CURSOR_DOWN(x)		"\033["#x"B"
#define VT100_CURSOR_RIGHT(x)		"\033["#x"C"
#define VT100_CURSOR_LEFT(x)		"\033["#x"D"
#define VT100_CURSOR_TO_HOME		"\033[H"

#define VT100_ERASE_SCR_TO_END		"\033[J"
#define VT100_ERASE_SCR_TO_HERE		"\033[1J"
#define VT100_ERASE_SCR_ALL			"\033[2J"
#define VT100_ERASE_LN_TO_END		"\033[K"
#define VT100_ERASE_LN_TO_HERE		"\033[1K"
#define VT100_ERASE_LN_ALL			"\033[2K"

/*****************************************************************************/
/** DATATYPES ****************************************************************/
/*****************************************************************************/

typedef struct menu_Tag menu_st;
typedef struct menu_Tag
{
	char			cmd_char;           /* inputted character */
	const char *	title;              /* Title of current menu */
	void			(*fp)( void * );    /* function pointer */
	void *			fparam;              /* parameter passed to the next item */
	const menu_st *	next;
} menu_st;

typedef struct menu_stack_Tag
{
	const menu_st * menu;			/* Menu Item pointer */
	const char * title;				/* Title of menu */
} menu_stack_t;

/*****************************************************************************/
/** FUNCTION DECLARATIONS ****************************************************/
/*****************************************************************************/

void tprintf( const char * fmt, ... );

void AddTerminalFunction( uint8_t (*f)(void *), void * p );
void TerminalTask( void );

/*****************************************************************************/
/** EXTERNAL VARIABLES *******************************************************/
/*****************************************************************************/

/*****************************************************************************/
/** LOCAL VARIABLES **********************************************************/
/*****************************************************************************/

/*****************************************************************************/
/** GLOBAL VARIABLES *********************************************************/
/*****************************************************************************/

/*****************************************************************************/
/** FUNCTION DEFINITIONS *****************************************************/
/*****************************************************************************/


#ifdef __cplusplus
}
#endif

#endif /* _A_TERMINAL_H_ */
