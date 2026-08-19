/******************************************************************************
 * @file m_datablock.c
 * @author jylee1 (jylee1@gst-in.com)
 * @brief 
 * @version 0.1
 * @date 2023-06-22
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

#include <m_datablock.h>

/*****************************************************************************/
/** MACRO DEFINITIONS ********************************************************/
/*****************************************************************************/

/*****************************************************************************/
/** DATATYPES ****************************************************************/
/*****************************************************************************/

/*****************************************************************************/
/** FUNCTION DECLARATIONS ****************************************************/
/*****************************************************************************/

/*****************************************************************************/
/** LOCAL VARIABLES **********************************************************/
/*****************************************************************************/

static uint16_t mbs_input_regs[ MBS_INPUT_REGS_CNT ] __section(".ccmram");
static uint16_t mbs_hold_regs[ MBS_HOLDING_REGS_CNT ] __section(".ccmram");
static uint16_t mbm_hold_regs[ MAX_SMPS_CMD ][ MBM_HOLDING_REGS_CNT ] __section(".ccmram");

/*****************************************************************************/
/** GLOBAL VARIABLES *********************************************************/
/*****************************************************************************/

const DataBlock_st MBSDB = {
	.DInputs = NULL,
	.DInputsCnt = 0,
	.DInputsStart = 0,

	.Coils = NULL,
	.CoilsCnt = 0,
	.CoilsStart = 0,

	.Inputs = mbs_input_regs,
	.InputsCnt = MBS_INPUT_REGS_CNT,
	.InputsStart = MBS_INPUT_START,

	.Holdings = mbs_hold_regs,
	.HoldingsCnt = MBS_HOLDING_REGS_CNT,
	.HoldingsStart = MBS_HOLDING_START,
};

const DataBlock_st MBM_READ_DB[ MAX_SMPS_ID ] = {
	[ 0 ] = {
	.Holdings = mbm_hold_regs[ BRAODCAST_ID ],
	.HoldingsCnt = MBM_HOLDING_REGS_CNT,
	.HoldingsStart = MBM_HOLDING_START,
	},
	[ 1 ] = {
	.Holdings = mbm_hold_regs[ 1 ],
	.HoldingsCnt = MBM_HOLDING_REGS_CNT,
	.HoldingsStart = MBM_HOLDING_START,
	},
	[ 2 ] = {
	.Holdings = mbm_hold_regs[ 2 ],
	.HoldingsCnt = MBM_HOLDING_REGS_CNT,
	.HoldingsStart = MBM_HOLDING_START,
	},
	[ 3 ] = {
	.Holdings = mbm_hold_regs[ 3 ],
	.HoldingsCnt = MBM_HOLDING_REGS_CNT,
	.HoldingsStart = MBM_HOLDING_START,
	},
	[ 4 ] = {
	.Holdings = mbm_hold_regs[ 4 ],
	.HoldingsCnt = MBM_HOLDING_REGS_CNT,
	.HoldingsStart = MBM_HOLDING_START,
	},
	[ 5 ] = {
	.Holdings = mbm_hold_regs[ 5 ],
	.HoldingsCnt = MBM_HOLDING_REGS_CNT,
	.HoldingsStart = MBM_HOLDING_START,
	},
	[ 6 ] = {
	.Holdings = mbm_hold_regs[ 6 ],
	.HoldingsCnt = MBM_HOLDING_REGS_CNT,
	.HoldingsStart = MBM_HOLDING_START,
	},
	[ 7 ] = {
	.Holdings = mbm_hold_regs[ 7 ],
	.HoldingsCnt = MBM_HOLDING_REGS_CNT,
	.HoldingsStart = MBM_HOLDING_START,
	},
	[ 8 ] = {
	.Holdings = mbm_hold_regs[ 8 ],
	.HoldingsCnt = MBM_HOLDING_REGS_CNT,
	.HoldingsStart = MBM_HOLDING_START,
	},
	[ 9 ] = {
	.Holdings = mbm_hold_regs[ 9 ],
	.HoldingsCnt = MBM_HOLDING_REGS_CNT,
	.HoldingsStart = MBM_HOLDING_START,
	},
	[ 10 ] = {
	.Holdings = mbm_hold_regs[ 10 ],
	.HoldingsCnt = MBM_HOLDING_REGS_CNT,
	.HoldingsStart = MBM_HOLDING_START,
	},
	[ 11 ] = {
	.Holdings = mbm_hold_regs[ 11 ],
	.HoldingsCnt = MBM_HOLDING_REGS_CNT,
	.HoldingsStart = MBM_HOLDING_START,
	},
	[ 12 ] = {
	.Holdings = mbm_hold_regs[ 12 ],
	.HoldingsCnt = MBM_HOLDING_REGS_CNT,
	.HoldingsStart = MBM_HOLDING_START,
	},
	[ 13 ] = {
	.Holdings = mbm_hold_regs[ 13 ],
	.HoldingsCnt = MBM_HOLDING_REGS_CNT,
	.HoldingsStart = MBM_HOLDING_START,
	},
	[ 14 ] = {
	.Holdings = mbm_hold_regs[ 14 ],
	.HoldingsCnt = MBM_HOLDING_REGS_CNT,
	.HoldingsStart = MBM_HOLDING_START,
	},
	[ 15 ] = {
	.Holdings = mbm_hold_regs[ 15 ],
	.HoldingsCnt = MBM_HOLDING_REGS_CNT,
	.HoldingsStart = MBM_HOLDING_START,
	},
};

const DataBlock_st MBM_WRITE_DB[ MAX_SMPS_ID ] = {
	[ 0 ] = {
	.Holdings = mbm_hold_regs[ 16 ],
	.HoldingsCnt = MBM_HOLDING_REGS_CNT,
	.HoldingsStart = MBM_HOLDING_START,
	},
	[ 1 ] = {
	.Holdings = mbm_hold_regs[ 17 ],
	.HoldingsCnt = MBM_HOLDING_REGS_CNT,
	.HoldingsStart = MBM_HOLDING_START,
	},
	[ 2 ] = {
	.Holdings = mbm_hold_regs[ 18 ],
	.HoldingsCnt = MBM_HOLDING_REGS_CNT,
	.HoldingsStart = MBM_HOLDING_START,
	},
	[ 3 ] = {
	.Holdings = mbm_hold_regs[ 19 ],
	.HoldingsCnt = MBM_HOLDING_REGS_CNT,
	.HoldingsStart = MBM_HOLDING_START,
	},
	[ 4 ] = {
	.Holdings = mbm_hold_regs[ 20 ],
	.HoldingsCnt = MBM_HOLDING_REGS_CNT,
	.HoldingsStart = MBM_HOLDING_START,
	},
	[ 5 ] = {
	.Holdings = mbm_hold_regs[ 21 ],
	.HoldingsCnt = MBM_HOLDING_REGS_CNT,
	.HoldingsStart = MBM_HOLDING_START,
	},
	[ 6 ] = {
	.Holdings = mbm_hold_regs[ 22 ],
	.HoldingsCnt = MBM_HOLDING_REGS_CNT,
	.HoldingsStart = MBM_HOLDING_START,
	},
	[ 7 ] = {
	.Holdings = mbm_hold_regs[ 23 ],
	.HoldingsCnt = MBM_HOLDING_REGS_CNT,
	.HoldingsStart = MBM_HOLDING_START,
	},
	[ 8 ] = {
	.Holdings = mbm_hold_regs[ 24 ],
	.HoldingsCnt = MBM_HOLDING_REGS_CNT,
	.HoldingsStart = MBM_HOLDING_START,
	},
	[ 9 ] = {
	.Holdings = mbm_hold_regs[ 25 ],
	.HoldingsCnt = MBM_HOLDING_REGS_CNT,
	.HoldingsStart = MBM_HOLDING_START,
	},
	[ 10 ] = {
	.Holdings = mbm_hold_regs[ 26 ],
	.HoldingsCnt = MBM_HOLDING_REGS_CNT,
	.HoldingsStart = MBM_HOLDING_START,
	},
	[ 11 ] = {
	.Holdings = mbm_hold_regs[ 27 ],
	.HoldingsCnt = MBM_HOLDING_REGS_CNT,
	.HoldingsStart = MBM_HOLDING_START,
	},
	[ 12 ] = {
	.Holdings = mbm_hold_regs[ 28 ],
	.HoldingsCnt = MBM_HOLDING_REGS_CNT,
	.HoldingsStart = MBM_HOLDING_START,
	},
	[ 13 ] = {
	.Holdings = mbm_hold_regs[ 29 ],
	.HoldingsCnt = MBM_HOLDING_REGS_CNT,
	.HoldingsStart = MBM_HOLDING_START,
	},
	[ 14 ] = {
	.Holdings = mbm_hold_regs[ 30 ],
	.HoldingsCnt = MBM_HOLDING_REGS_CNT,
	.HoldingsStart = MBM_HOLDING_START,
	},
	[ 15 ] = {
	.Holdings = mbm_hold_regs[ 31 ],
	.HoldingsCnt = MBM_HOLDING_REGS_CNT,
	.HoldingsStart = MBM_HOLDING_START,
	},
};
/*****************************************************************************/
/** FUNCTION DEFINITIONS *****************************************************/
/*****************************************************************************/



