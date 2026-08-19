/******************************************************************************
 * @file a_terminal.c
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


/*****************************************************************************/
/** INCLUDES *****************************************************************/
/*****************************************************************************/

#include <UserApp.h>
#include <ctype.h>

/*****************************************************************************/
/** MACRO DEFINITIONS ********************************************************/
/*****************************************************************************/

#define TERMINAL_TASK_CYCLE_TIME 0.01f

#define TERM_HEADLINE				"ENGNEERING MENU"

#define	TER_SERVICE_DATE            "230904"

#define MAX_MENU_STACK 6

#define MODBUS_SLAVE_THROUGH_USBCDC

/*****************************************************************************/
/** DATATYPES ****************************************************************/
/*****************************************************************************/

typedef struct terminal_service_Tag
{
	struct
	{
		uint8_t On:2;				/* 0: terminal service off, not 0: terminal service on */
		uint8_t Refresh:1;		/* menu print out requested  */
		uint8_t CommandIndex:5;
	};

	uint8_t TxBuf[ 512 ];
	uint8_t RxBuf[ 256 ];			/* string copied to terminal service */
	uint8_t RxCnt;
	volatile uint32_t DelayCnt;		/* delay count when terminal service starts */
	uint8_t ( *function )( void * );	/* current function pointer. if null, not run. */
	void * func_param;				/* parameter of function pointer */
} terminal_st;

/*****************************************************************************/
/** FUNCTION DECLARATIONS ****************************************************/
/*****************************************************************************/

__weak void terminal_menu_splash_screen( void );

/*****************************************************************************/
/** EXTERNAL VARIABLES *******************************************************/
/*****************************************************************************/

extern GstRingBufHandle_t rbUsbRx;

/*****************************************************************************/
/** LOCAL VARIABLES **********************************************************/
/*****************************************************************************/

static CCMRAM terminal_st terminal;
static AppTimerData_ut timerTerminalTask = { 0 };

static const char * const MenuHeader[] =
{
	VT100_ERASE_SCR_ALL VT100_CURSOR_TO_HOME "\r\n",
	" ,== %-15.15s ================= " TER_SERVICE_DATE " ==.\r\n",
	" | %-40.40s (%d) |\r\n",
	" |----------------------------------------------|\r\n",
};

static const char * const MenuBody[] =
{
	" | [%c] %-40.40s |\r\n",
	" | %-44.44s |\r\n"
};

static const char * const MenuFooter[] =
{
	" |----------------------------------------------|\r\n",
	" | [ESC] Previous              [/] Home         |\r\n",
	" | [?] Refresh                 [~] Reboot (x3)  |\r\n",
	" `=============================================='\r\n",
	"  * Select Menu : " VT100_CURSOR_POS_SAVE
};

/* DO NOT EDIT BELOW *******************************************/

/******************************************************************************
 * @brief weak array for MenuHome
 * 
 * @note DO NOT EDIT THIS WEAK VARIABLE TO MAKE MENU
 * @note This function Should not be modified, when editing is needed,
 *       it could be implemented in the a_terminal_menu.c file
 *****************************************************************************/
menu_st const MenuHome[] __weak =
{
	{ '\0', "YOU DIDN'T MAKE A 'MenuHome' VARIABLE.", NULL, NULL, NULL, },
	{ '\0', "If you saw this, you need to create a", NULL, NULL, NULL, },
	{ '\0', "  MenuHome variable at a_terminal_menu.c", NULL, NULL, NULL, },
	{ '\0', "  like this:", NULL, NULL, NULL, },
	{ '\0', "", NULL, NULL, NULL, },
	{ '\0', "menu_st const MenuHome[] = ", NULL, NULL, NULL, },
	{ '\0', "{", NULL, NULL, NULL, },
	{ '\0', "};", NULL, NULL, NULL, },
	{ '\0', NULL, NULL, NULL, NULL, },
};

/* DO NOT EDIT UPPER *******************************************/


static menu_stack_t menuStack[ MAX_MENU_STACK ];
static int menu_stack_top;

/*****************************************************************************/
/** GLOBAL VARIABLES *********************************************************/
/*****************************************************************************/

/*****************************************************************************/
/** FUNCTION DEFINITIONS *****************************************************/
/*****************************************************************************/

/******************************************************************************
 * @brief 
 * 
 * @return menu_st const* 
 *****************************************************************************/
static inline menu_st const * current_menu( void )
{
	return ( menuStack[ menu_stack_top ].menu );
}

/******************************************************************************
 * @brief 
 * 
 * @param menu 
 * @return uint8_t 
 *****************************************************************************/
static inline uint8_t menu_valid( menu_st const * menu )
{
	return menu && ( menu->cmd_char || menu->title );
}

/******************************************************************************
 * @brief get current menu title as literal
 * 
 * @return const char* 
 *****************************************************************************/
static inline const char * current_menu_title( void )
{
	return ( menuStack[ menu_stack_top ].title );
}

/******************************************************************************
 * @brief pop last menu from menu stack
 * 
 * @return menu_st const* 
 *****************************************************************************/
static menu_st const * menu_pop( void )
{
	menu_st const * menu = NULL;

	menu = menuStack[ menu_stack_top ].menu;

	if ( menu_stack_top )
	{
		menuStack[ menu_stack_top ].menu = 0;
		menuStack[ menu_stack_top ].title = 0;
		menu_stack_top--;
	}

	return ( menu );
}

/******************************************************************************
 * @brief pop all from menu stack
 * 
 * @return menu_st const* 
 *****************************************************************************/
static menu_st const * menu_pop_all( void )
{
	if( menu_stack_top < 0 ) return NULL;

	while ( menu_stack_top )
	{
		menu_pop();
	}

	return ( current_menu() );
}

/******************************************************************************
 * @brief push menu to menu stack
 * 
 * @param menu 
 * @param title 
 * @return int 
 *****************************************************************************/
static int menu_push( menu_st const * menu, const char * title )
{
	int stack_push_status = 0;

	if ( menu_stack_top >= MAX_MENU_STACK )
	{
		tprintf( "MENU THREAD ERROR : Stack is Full!!\r\n" );
	}
	else
	{
		menu_stack_top++;
		menuStack[ menu_stack_top ].menu = menu;
		menuStack[ menu_stack_top ].title = title;

		stack_push_status = 1;
	}

	return ( stack_push_status );
}

/******************************************************************************
 * @brief initialize menu stack
 * 
 *****************************************************************************/
static void menu_init(void)
{
	memset( menuStack, 0, sizeof( menuStack ) );
	menu_stack_top = -1;
	menu_push( MenuHome, (const char *)"Terminal Service" );
}

/******************************************************************************
 * @brief print out menu header
 * 
 *****************************************************************************/
void terminal_menu_header( void )
{
	tprintf( MenuHeader[ 0 ] );
	tprintf( MenuHeader[ 1 ], TERM_HEADLINE );
	for( int i = 1; i <= menu_stack_top; ++i)
	{
		tprintf( MenuHeader[ 2 ], menuStack[ i ].title, i );
	}
	tprintf( MenuHeader[ 3 ] );
}

/******************************************************************************
 * @brief print out menu body
 * 
 *****************************************************************************/
void terminal_menu_body( void )
{
	menu_st const * menu = current_menu();

	while( menu_valid( menu ) )
	{
		if( menu->cmd_char )
		{
			tprintf( MenuBody[ 0 ], menu->cmd_char, menu->title );
		}
		else
		{
			tprintf( MenuBody[ 1 ], menu->title );
		}

		menu++;
	}
}

/******************************************************************************
 * @brief print out menu footer
 * 
 *****************************************************************************/
void terminal_menu_footer( void )
{
	const uint8_t length = sizeof( MenuFooter ) / sizeof( MenuFooter[ 0 ] );

	for( uint8_t idx = 0; idx < length; ++idx )
	{
		tprintf( MenuFooter[ idx ] );
	}
}

/******************************************************************************
 * @brief print out menu if requested
 * 
 *****************************************************************************/
static void terminal_menu_display( void )
{
	if( terminal.Refresh )
	{
		terminal.Refresh = 0;

		terminal_menu_header();
		terminal_menu_body();
		terminal_menu_footer();
	}
}

/******************************************************************************
 * @brief printf out error message when error occurs
 * 
 * @param command 
 *****************************************************************************/
static inline void terminal_menu_command_error( char command )
{
	tprintf( VT100_CURSOR_POS_SAVE "\r\n" VT100_ERASE_LN_ALL "Error!!\r\n" VT100_ERASE_LN_ALL "[ %c ] is a invalid command. Please select above command again." VT100_CURSOR_POS_RESTORE , command );
}

/******************************************************************************
 * @brief Execute if there are additional functions to execute.
 * 
 * @return uint8_t 
 *****************************************************************************/
static inline uint8_t terminal_function_repeatly( void )
{
	uint8_t command_parsed = 0;

	if ( terminal.function )
	{
		command_parsed = terminal.function( terminal.func_param );

		if( 0x1B == ( uint32_t )terminal.func_param )
		{
			terminal.function = NULL;
			terminal.Refresh = 1;
		}

		if( command_parsed )
		{
			terminal.func_param = NULL;
		}
	}

	return command_parsed;
}

/******************************************************************************
 * @brief menu command parser
 * 
 * @note <3 times tilt> => NVIC System Reset
 * @note <slash>        => go menu to first
 * @note <ESC>          => go previous menu
 * @note <?>            => print current menu again
 * 
 * @param command 
 * @return uint8_t 
 *****************************************************************************/
static uint8_t common_command_parser( char command )
{
	uint8_t parsed_successfully = 0;
	static int esc_count = 2;
	static char last_command = 0;

	switch ( command )
	{
		case '~':
			{
				if( last_command == command )
				{
					--esc_count;
				}
				else
				{
					esc_count = 2;
				}

				if( esc_count )
				{
					tprintf( VT100_CURSOR_POS_SAVE "\r\n" VT100_ERASE_LN_ALL "\r\n" VT100_ERASE_LN_ALL " Input <SHIFT> + ` %d more to reboot" VT100_CURSOR_POS_RESTORE, esc_count );
					parsed_successfully = 1;
				}
				else
				{
					tprintf( "\r\n" VT100_ERASE_LN_ALL "\r\n" VT100_ERASE_LN_ALL " Reboot Now!" );

					/* Clear Screen */
					tprintf( VT100_CURSOR_TO_HOME VT100_ERASE_SCR_ALL );
					AppTimer.Delay( 0.05f );
					NVIC_SystemReset();
				}
			}
			break;
		case '/':
			menu_pop_all();
			terminal.Refresh = 1;
			parsed_successfully = 1;
			break;
		case '\x1B':
			if( menu_stack_top )
			{
				menu_pop();
				terminal.Refresh = 1;
			}
			else
			{
				terminal.On = 0;
			}
			parsed_successfully = 1;
			break;
		case '?':
			terminal.Refresh = 1;
			parsed_successfully = 1;
			break;
		case 0:
			parsed_successfully = 1;
			break;
	}
	
	last_command = command;

	return ( parsed_successfully );
}

/******************************************************************************
 * @brief current menu parser
 * 
 * @param command 
 *****************************************************************************/
static void terminal_menu_parser( char command )
{
	menu_st const * menu = current_menu();

	while( menu_valid( menu ) )
	{
		if( menu->cmd_char == command )
		{
			if ( menu->fp )
			{
				menu->fp( menu->fparam );
			}

			if( menu->next )
			{
				/* Move to next menu */
				menu_push( ( menu_st const * ) menu->next, menu->title );
				terminal.Refresh = 1;
			}
			break;
		}
		menu++;
	}

	if( !menu_valid( menu ) && !Calib.status.On )
	{
		terminal_menu_command_error( command );
	}
}

/******************************************************************************
 * @brief 
 * 
 * @param f 
 * @param p 
 *****************************************************************************/
void AddTerminalFunction( uint8_t (*f)(void *), void * p )
{
	terminal.function = f;
	terminal.func_param = p;
}

/******************************************************************************
 * @brief display splash screen (weak function)
 * 
 * @note DO NOT EDIT THIS WEAK FUNCTION TO MAKE SPLASH SCREEN
 * @note This function Should not be modified, when the edited is needed,
 *       the it could be implemented in the a_terminal_menu.c file
 *****************************************************************************/
__weak void terminal_menu_splash_screen( void )
{
	__NOP();
}

/******************************************************************************
 * @brief 
 * 
 * @param fmt 
 * @param ... 
 *****************************************************************************/
void tprintf( const char * fmt, ... )
{
#if defined( USE_LEGACY_TERMINAL )
	if( CDC.IsConnected() )
	{
		va_list args;

		int32_t sz = 0;
		va_start( args, fmt );
		sz = vsnprintf( ( char * )terminal.TxBuf, 512, fmt, args );
		va_end( args );

		if( 0 < sz && sz < 512 )
		{
			terminal.TxBuf[ sz ] = '\0';
			CDC.Write( terminal.TxBuf, sz );
		}
	}
#else
	/* Machine-readable test interface (a_test_func.c) owns the USB CDC channel.
	 * Human VT100 output (e.g. calibration_task progress) would corrupt the
	 * line protocol, so suppress it while the legacy terminal is disabled. */
	( void )fmt;
#endif
}

/******************************************************************************
 * @brief 
 * 
 *****************************************************************************/
void TerminalTask( void )
{
	static uint32_t old_length = 0;

	if( !CDC.IsConnected() )
	{
		terminal.On = 0;
		return;
	}

	if( AppTimer.IsExpired( &timerTerminalTask ) )
	{
		uint32_t rb_length = 0;

		AppTimer.Start( &timerTerminalTask, TERMINAL_TASK_CYCLE_TIME );

		if( terminal.On == 2 )
		{
			static AppTimerData_ut timerDefaultScreen = { 0 };

			if( AppTimer.IsExpired( &timerDefaultScreen ) )
			{
				AppTimer.Start( &timerDefaultScreen, 0.5f );

//				terminal_menu_splash_screen();
			}
		}

		if( ( ( rb_length = Ring.Length( &rbUsbRx ) ) == old_length ) && old_length )
		{
			const static char * terminal_magic_word[] = {"eng", "sta"};

			Ring.Get( &rbUsbRx, terminal.RxBuf, sizeof( terminal.RxBuf ) );
			uint8_t command = tolower( terminal.RxBuf[ 0 ] );

			if( terminal.On == 1 )
			{
				command = toupper( command );

				if( terminal.function )
				{
					terminal.func_param = ( void * )( uint32_t )command;
				}
				else
				{
					uint8_t command_parsed = 0;
					command_parsed = common_command_parser( command );

					if( !command_parsed )
					{
						terminal_menu_parser( command );
					}
				}
			}
			else
			{
				if( command == '\x08' )
				{
					if( terminal.CommandIndex ) terminal.CommandIndex--;
				}
				else if( terminal_magic_word[ 0 ][ terminal.CommandIndex ] == command )
				{
					terminal.CommandIndex++;
					tprintf( "%c", command );

					if( strlen( terminal_magic_word[ 0 ] ) == terminal.CommandIndex )
					{
						terminal.CommandIndex = 0;
						terminal.On = 1;
						tprintf( VT100_CURSOR_TO_HOME VT100_ERASE_SCR_ALL );
						menu_init();
						terminal.Refresh = 1;
					}
				}
				else if( terminal_magic_word[ 1 ][ terminal.CommandIndex ] == command )
				{
					terminal.CommandIndex++;
					tprintf( "%c", command );

					if( strlen( terminal_magic_word[ 1 ] ) == terminal.CommandIndex )
					{
						terminal.CommandIndex = 0;
						terminal.On = 2;
					}
				}
				else
				{
					terminal.CommandIndex = 0;
				}

#ifdef MODBUS_SLAVE_THROUGH_USBCDC
				if( !terminal.On )
				{
					extern Modbus_Server_st MBS;
					uint16_t tx_len = 0;

					MBS.UpdateInputs();
					tx_len = MBS.Run( terminal.RxBuf, rb_length, terminal.TxBuf );

					if( tx_len )
					{
						CDC.Write( terminal.TxBuf, tx_len );
					}

					old_length = 0;
					memset( terminal.RxBuf, 0, sizeof( terminal.RxBuf ) );

					MBS.UpdateFromHoldings();
				}
#endif
			}
		}
		else
		{
			old_length = rb_length;
		}

		terminal_menu_display();
		terminal_function_repeatly();
	}
}
