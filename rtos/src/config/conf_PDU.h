/*
 * conf_PDU.h
 *
 * Created: 11/25/2017 6:10:15 PM
 *  Author: MizzouRacing
 */ 


#ifndef CONF_PDU_H_
#define CONF_PDU_H_

#include "../CAN/PDU_CAN_conf.h"
#include "config_block.h"


/*
 *	This is the maximum value that we can currently 
 *	support with the hardware/software setups
 *	The MoTeC ECU can only control up to a max of 32 channels
 *	Thus without some extra mucking about we cannot go over this limit
 *	To go over this limit we would need to modify software to handle 
 *	multiple ECU request
 */	
#define MAX_PDU_CHANNEL 32
//configASSERT( MAX_PDU_CHANNEL > NUM_PDU_CHANNEL );

/*
* IOPORT Definitions for PDU enables
* Assumes all channels exist on PORTD from PORTD0 to PORTD(NUM_PDM_CHANNEL)
* Individual Definitions for each channels name and offset in PORT
*/												
#define PDU_CHANNEL_0_OFFSET	24 
#define PDU_ENABLES_PORTD_MASK	((1<<NUM_LC_CHANNEL) - 1	) << PDU_CHANNEL_0_OFFSET	//generate bitmask for PORTD enable

#define PDU_MULTIPLEXER_EN		PDU_MULTIPLEXER_iopins[0]
#define PDU_MULTIPLEXER_MASK	PDU_MULTIPLEXER_iopins[1] | PDU_MULTIPLEXER_iopins[2] | PDU_MULTIPLEXER_iopins[3] 


/************************************************************************/
/* PWM Defintions                                                       */
/************************************************************************/

#define PDU_PWM								PWM
#define PWM_STARTUP_DUTY_CYCLE				5
#define PWM_DUTY_CYCLE_SATURATION_LIMIT		100
#define PWM_DUTY_CYCLE_OFFSET				100

/* 
* Voltage levels for which we are going to try to limit pwm freq 
* If battery voltage starts to droop we pull back allowable max pwm to 
* protect battery from dropping below voltage floor of lithium cells
*/

typedef enum voltage_lvl
{
	VOLTAGE_LVL_8V,
	VOLTAGE_LVL_9V,
	VOLTAGE_LVL_10V,
	VOLTAGE_LVL_11V,
	VOLTAGE_LVL_12V,
	VOLTAGE_LVL_13V,
	VOLTAGE_LVL_14V,
	NUM_VOLTAGE_LVL

}voltage_lvl_t;

static const voltage_lvl_t pwm_voltage_levels[NUM_VOLTAGE_LVL] = {
	[VOLTAGE_LVL_8V] 	= 8,
	[VOLTAGE_LVL_9V]	= 9,
	[VOLTAGE_LVL_10V]	= 10,
	[VOLTAGE_LVL_11V]	= 11,
	[VOLTAGE_LVL_12V]	= 12,
	[VOLTAGE_LVL_13V]	= 13,
	[VOLTAGE_LVL_14V]	= 14
};

static const uint8_t pwm_staturation_limits[NUM_VOLTAGE_LVL] = {
	0,
	25,
	75,
	PWM_DUTY_CYCLE_SATURATION_LIMIT,
	PWM_DUTY_CYCLE_SATURATION_LIMIT,
	PWM_DUTY_CYCLE_SATURATION_LIMIT,
	PWM_DUTY_CYCLE_SATURATION_LIMIT
};

/*
* 
*/
#define IS_AFEC					AFEC0
#define IS_AFEC2				AFEC1

	
/************************************************************************/
/* General constants for operation                                      */
/************************************************************************/
#define PDU_OVERTEMP						75							//75C, considering this value as too high of a temperature for the pdu to continue operation

#define INRUSH_AFEC_DELAY					20							//number of samples to delay fuse tripping by, to ride through inrush currents
#define OVERCURRENT_REJECTION_DELAY			5							//number of quantum that channel should be overcurrent before tripping fault
#define PRECHARGE_ATTEMPTS					5							//number of times to cycle powerfet to charge capacitors on bus
#define PRECHARGE_TIME						5							//ms to leave bus energized to attempt precharge

#define CAN_TIMEOUT_LIMIT					1000						//ms, maximum time between ECU messages

/************************************************************************/
/* Channel Status Codes                                                 */
/************************************************************************/

typedef enum {
	CHANNEL_OFF				= 0,
	CHANNEL_ON				= 1,
	CHANNEL_OVER_CURRENT	= 2,
	CHANNEL_OPEN			= 3,
	CHANNEL_FAULT			= 4,
	CHANNEL_CLEAR_FAULT		= 5
}status_code;

typedef enum {
	NO_REQUEST				= 0,
	OFF_REQUEST				,
	ON_REQUEST				,
	OVER_CURRENT_TRANSITION	,
	OPEN_TRANSITION			,
	FAULT_TRANSITION		,
	CLEAR_FAULT_TRANSITION			
}state_request;

/************************************************************************/
/*				Vehicle Definitions                                     */
/************************************************************************/

typedef enum{
	startup = 0,
	run		= 1,
	stop	= 2,	
	crank	= 3
	}engine_state;

/************************************************************************/
/*           Timer Definitions for PDU                                */
/************************************************************************/

#define PDU_TC0_MODULE						TC0
#define PDU_TC1_MODULE						TC1

#define PDU_TC_COMPARE_INT					TC_IER_CPCS

#define TC_100HZ							TC0_Handler
#define TC_10HZ								TC1_Handler
#define TC_1HZ								TC2_Handler

typedef enum{
	TC_100HZ_CHANNEL	=	0,
	TC_10HZ_CHANNEL		=	1,
	TC_1HZ_CHANNEL		=	2,
	TC_NUM_CHANNEL
	}PDU_TC0_INDEX;
	
static const uint32_t PDU_TC_DESIRED_HZ[TC_NUM_CHANNEL] = {
	100,
	10,
	1,
	};

static const IRQn_Type PDU_TC_IRQ[TC_NUM_CHANNEL] = {
	TC0_IRQn,
	TC1_IRQn,
	TC2_IRQn,
	//TC3_IRQn,
	//TC4_IRQn,
	//TC5_IRQn,
	//TC6_IRQn,
	//TC7_IRQn,
	//TC8_IRQn
};


/************************************************************************/
/* RTOS Frequency Definitions                                           */
/************************************************************************/

#define PDU_TICK_HZ						1000
#define PDU_MASTER_CLK					(unsigned long)((BOARD_FREQ_MAINCK_XTAL * CONFIG_PLL0_MUL) / CONFIG_PLL0_DIV )

#define MS_IN_SEC						1000

#define TASK_100_HZ						MS_IN_SEC / 100
#define	TASK_50_HZ						MS_IN_SEC / 50
#define	TASK_20_HZ						MS_IN_SEC / 20
#define TASK_10_HZ						MS_IN_SEC / 10
#define TASK_1_HZ						MS_IN_SEC / 1



#endif /* CONF_PDU_H_ */