/*
 * config_block.h
 *
 * Created: 7/14/2018 6:46:14 PM
 *  Author: Jason
 */ 


#ifndef CONFIG_BLOCK_H_
#define CONFIG_BLOCK_H_

#include "conf_PDU.h"

#ifndef SAM4E8E
	#define SAM4E8E
#endif

#ifndef __SAM4E8E__
	#define __SAM4E8E__
#endif

/*
* Bit position for individual enable channels
* Each value is the offset from the zero position for each channel
*/

typedef enum enable_index	{
	channel_0 = 0	,
	channel_1		,
	channel_2		,
	channel_3		,
	channel_4		,
	channel_5		,
	channel_6		,
	channel_7		,
	channel_8		,
	channel_9		,
	channel_10		,
	channel_11		,
	NUM_PDU_CHANNEL  //total number of PDM channels
}				pdu_enable_index;

/************************************************************************/
/*				Channel config for Rev2/3 PDU                             */
/* Channels 0-3 are higher current BTS-50060 chips single channel		*/
/* Channels 4-11 are multiplexed in BTS-700x chips						*/
/************************************************************************/
typedef enum {
	FAN			= channel_0,
	IGNITION	= channel_3,
	PUMP		= channel_2,
	ECU			= channel_4,
	BRAKE		= channel_6,
	HORN		= channel_7,
	DATA		= channel_8,
	GEN_5V		= channel_9,
	GEN_12V		= channel_10,
	WATER		= channel_1,
	BSPD		= channel_11
	
}channel_config;

#define NUM_HC_CHANNEL				4
#define NUM_LC_CHANNEL				8
#define NUM_DIGITAL_IO				10


/*
*---------------DEFAULT INIT VAL ------------
* The default value to be loaded into ioport for enabled channels
* This determines which channels will be enabled at startup and in
* complicated error recovery modes defined later
*/
#define PDU_DEFAULT_MASK		( (1<<ECU) | (1<<DATA) | (1<<GEN_5V) | (1<<GEN_12V) )
#define PDU_TIMEOUT_MASK		0//((1<<ECU) | (1<<DATA))
//At a later date might be able to remove ECU_MASK from this list, make a "run" relay in ECU that toggles its own PDU bit
#define PDU_NON_ECU_MASK		PDU_DEFAULT_MASK
//These channels are configured to be "restarted" if they encounter a fault or over current
#define CONF_SOFT_RESTART_MASK	0//( (1<<ECU) | (1<<FAN) | (1<<PUMP) | (1<<WATER) )
#define NULL_MASK				0

/************************************************************************/
/*			PWM Config                                             */
/************************************************************************/

/** PWM frequency in Hz */
#define PDU_PWM_FREQUENCY			5000
/** Period value of PWM output waveform */
#define PDU_PERIOD_VALUE			100
/** Initial duty cycle value */
#define PDU_DEFAULT_DUTY_VALUE		(PWM_DUTY_CYCLE_SATURATION_LIMIT - 25)

/*
* Config for Current limits for each channel
* These values are derived from bench testing devices
* and some from manufacturer datasheets, these represent the
* max current we expect a device to pull during normal operation
* NOTE: These are listed as the ceiling of what we expect
*/
#define FAN_IS_MAX				26
#define IGNITION_IS_MAX			10
#define PUMP_IS_MAX				15
#define ECU_IS_MAX				12
#define BRAKE_IS_MAX			2
#define HORN_IS_MAX				6
#define DATA_IS_MAX				6
#define GEN_5V__IS_MAX			3
#define GEN_12V_IS_MAX			5
#define WAWA_PUMP_IS_MAX		15

static const uint32_t curr_limits[NUM_PDU_CHANNEL] = {
	[FAN]=FAN_IS_MAX,
	[IGNITION]=IGNITION_IS_MAX,
	[PUMP]=PUMP_IS_MAX,
	[ECU]=ECU_IS_MAX,
	[BRAKE]=BRAKE_IS_MAX,
	[HORN]=HORN_IS_MAX,
	[DATA]=DATA_IS_MAX,
	[GEN_5V]=GEN_5V__IS_MAX,
	[GEN_12V]=GEN_12V_IS_MAX,
	[WATER]=WAWA_PUMP_IS_MAX
	
};


/*
* AFEC Symbolic Names for each channel
*/
//AFEC0 mappings
#define HA_AFEC				AFEC0
#define HA_1				AFEC_CHANNEL_0
#define HA_2				AFEC_CHANNEL_1
#define HA_3				AFEC_CHANNEL_2
#define HA_4				AFEC_CHANNEL_3
#define HA_1_TEMP			AFEC_CHANNEL_9
#define HA_2_TEMP			AFEC_CHANNEL_10
#define HA_3_TEMP			AFEC_CHANNEL_11
#define HA_4_TEMP			AFEC_CHANNEL_6


#define BATT_MONITOR		AFEC_CHANNEL_12
#define SUPPLY_MONITOR		AFEC_CHANNEL_13
#define NUM_SUPPLY_MONITOR	2

//AFEC1 mappings
#define LA_AFEC				AFEC1
#define IS_1				AFEC_CHANNEL_4
#define IS_2				AFEC_CHANNEL_5
#define IS_3				AFEC_CHANNEL_6
#define IS_4				AFEC_CHANNEL_7


//AFEC for just batt and supply monitor
#define PDU_AFEC			AFEC0
/*
* List of the AFEC channels used by the PDU
* Used for looking up AFEC channel translation for each channel
*/
static const enum afec_channel_num HC_AFEC_channel_list[NUM_HC_CHANNEL] = {
	HA_1,
	HA_2,
	HA_3,
	HA_4
};

static const enum afec_channel_num HC_temp_channel_list[NUM_HC_CHANNEL] = {
	HA_1_TEMP,
	HA_2_TEMP,
	HA_3_TEMP,
	HA_4_TEMP
};

static const enum afec_channel_num LC_AFEC_channel_list[NUM_LC_CHANNEL/ 2] = {
	IS_1,
	IS_2,
	IS_3,
	IS_4
};

static const enum afec_channel_num supply_monitor_list[NUM_SUPPLY_MONITOR] = {
	BATT_MONITOR,
	AFEC_CHANNEL_13
	};

/************************************************************************/
/* IOPin Assignments                                                    */
/************************************************************************/

/* Status LED GPIO Pins */
#define STATUS_LED1_GPIO            (PIO_PA25_IDX)
#define STATUS_LED2_GPIO            (PIO_PA26_IDX)
#define STATUS_LED3_GPIO            (PIO_PA27_IDX)

#define HC_THREAD_LED				STATUS_LED2_GPIO
#define SYS_THREAD_LED				STATUS_LED1_GPIO
#define CAN_REC_THREAD_LED			STATUS_LED3_GPIO

#define	PDU_ENABLE_PORT				IOPORT_PIOD
#define DIO_PORT					IOPORT_PIOD

#define PDU_OFF_STATE				IOPORT_PIN_LEVEL_LOW
#define PDU_ON_STATE				IOPORT_PIN_LEVEL_HIGH

#define PDU_DIO_INPUT_FLAGS			(PIO_PULLUP | PIO_DEBOUNCE | PIO_DEGLITCH)
#define PDU_DIO_SENSE_LEVEL			IOPORT_SENSE_LEVEL_LOW


static const ioport_pin_t PDU_iopins[NUM_PDU_CHANNEL]={
	[channel_0]=	PIO_PD20_IDX,
	[channel_1]= 	PIO_PD21_IDX,
	[channel_2]=	PIO_PD22_IDX,
	[channel_3]=	PIO_PD23_IDX,
	[channel_4]=	PIO_PD24_IDX,
	[channel_5]=	PIO_PD25_IDX,
	[channel_6]=	PIO_PD26_IDX,
	[channel_7]=	PIO_PD27_IDX,
	[channel_8]=	PIO_PD28_IDX,
	[channel_9]=	PIO_PD29_IDX,
	[channel_10]=	PIO_PD30_IDX,
	[channel_11]=	PIO_PD31_IDX,
};

static const ioport_pin_t PDU_DEN_iopins[NUM_LC_CHANNEL]={
	PIO_PD11_IDX,
	PIO_PD12_IDX,
	PIO_PD13_IDX,
	PIO_PD14_IDX,
};

static const ioport_pin_t PDU_DSEL_iopins[NUM_LC_CHANNEL]={
	PIO_PD16_IDX,
	PIO_PD17_IDX,
	PIO_PD18_IDX,
	PIO_PD19_IDX,
};

static const uint32_t pwm_assignment[NUM_HC_CHANNEL]={
	[channel_0]=	PWM_CHANNEL_0,
	[channel_1]= 	PWM_CHANNEL_1,
	[channel_2]=	PWM_CHANNEL_2,
	[channel_3]=	PWM_CHANNEL_3
};


static const ioport_pin_t PDU_digital_iopoins[NUM_DIGITAL_IO]={
	PIO_PD0,
	PIO_PD1,
	PIO_PD2,
	PIO_PD3,
	PIO_PD4,
	PIO_PD5,
	PIO_PD6,
	PIO_PD7,
	PIO_PD8,
	PIO_PD9
	
};

//IOPins symbolic names
#define RESET_LINE				PDU_digital_iopoins[0]
//#define ENDURANCE_SWITCH		PDU_digital_iopoins[NUM_DIGITAL_IO]


/*
* Const Array of static offsets taken by each powerfet Is
* The Fets all have a typical offset value that we could use
* But due to variances in soldering and PCB traces, all are just
* different enough to warrant an array of values gleaned from
* bench test in debug mode and are mV
*/
static const uint32_t PDU_AFEC_channel_offset[NUM_PDU_CHANNEL] = {
	47,40,40,40,1,1,1,1,1,1,1,1								
}; //this shit is why I should have put NVSRAM on board

#define AFEC_DIGITAL_MAX					(4095UL * 16)
#define AFEC_ANALOG_REF						3300UL
#define AFEC_conversion_factor_mV 			(float)((float)AFEC_ANALOG_REF / AFEC_DIGITAL_MAX)
#define AFEC_mV_to_V						(1000)
#define AFEC_ANALOG_OFFSET					0x800

/************************************************************************/
/* Thermistor constants                                                 */
/************************************************************************/
#define T_NOMINAL							10000
#define R_BALANCE							10000
#define BETA								4300
#define ROOM_TEMP							(25 + 273)

/************************************************************************/
/* Voltage Monitor constants                                                 */
/************************************************************************/
#define SUPPLY_MONITOR_R1					4700
#define SUPPLY_MONITOR_R2					4700
#define BATTERY_MONITOR_R1					16000
#define BATTERY_MONITOR_R2					4700
#define VOLTAGE_SCALE_FACTOR				100

/************************************************************************/
/* Current follower constants for high current FETs                     */
/************************************************************************/
#define PDU_SENSE_MV_TO_MA_HC				220							//this is value of resistor transforming current feedback into voltage
#define PDU_FET_DIFFERENTIAL_RATIO_HC		13							//define in BTS50060 Datasheet
#define SCALE_FACTOR_HC						10							//factor to multiple Amperage value by before stuffing into uint8 val

//Sucks that for this bts50060 the fault and open load currents are same
#define IS_FAULT_MIN_HC_MA					(7.5F)
#define IS_OPEN_MIN_HC_MA					(7.5F)

#define FAULT_MA_HC							((uint32_t)IS_FAULT_MIN_HC*PDU_SENSE_MV_TO_MA_HC)
#define IS_FAULT_MIN_A_HC					(uint16_t)((IS_FAULT_MIN_HC_MA * PDU_FET_DIFFERENTIAL_RATIO_HC) * SCALE_FACTOR_HC)
#define IS_OPEN_MIN_A_HC					(uint16_t)((IS_OPEN_MIN_HC_MA * PDU_FET_DIFFERENTIAL_RATIO_HC) * SCALE_FACTOR_HC)

#define IS_FAULT_MIN_HC						1320						//min number of mV that indicates fault condition
#define IS_OPEN_MIN_HC						1320						//min number of mV that indicates open load condition


#define FAULT_LATCH_TIME_MS_HC				85



/************************************************************************/
/* Current follower constants for low current FETs                     */
/************************************************************************/
#define PDU_SENSE_MV_TO_MA_LC				1000							//this is value of resistor transforming current feedback into voltage
#define PDU_FET_DIFFERENTIAL_RATIO_LC		(5.5F)							//define in BTS7008 Datasheet
#define SCALE_FACTOR_LC						5							//factor to multiple Amperage value by before stuffing into uint8 val

#define IS_FAULT_MIN_LC_MA					(5.5F)
#define IS_OPEN_MIN_LC_MA					(2.5F)

#define FAULT_MA_LC							((uint32_t)IS_FAULT_MIN_LC*PDU_SENSE_MV_TO_MA_LC)
#define IS_FAULT_MIN_A_LC					(uint16_t)((IS_FAULT_MIN_LC_MA * PDU_FET_DIFFERENTIAL_RATIO_LC) * SCALE_FACTOR_LC)
#define IS_OPEN_MIN_A_LC					(uint16_t)((IS_OPEN_MIN_LC_MA * PDU_FET_DIFFERENTIAL_RATIO_LC) * SCALE_FACTOR_LC)

#define IS_FAULT_MIN_LC						1320						//min number of mV that indicates fault condition
#define IS_OPEN_MIN_LC						1320						//min number of mV that indicates open load condition

#define FAULT_LATCH_TIME_MS_LC				100



/************************************************************************/
/* RTOS Configuration for task                                          */
/************************************************************************/

#define HC_THREAD_STACK_SIZE			(1024/sizeof(portSTACK_TYPE))
#define LC_THREAD_STACK_SIZE			(1024/sizeof(portSTACK_TYPE))
#define SYSTEM_THREAD_STACK_SIZE		(1024/sizeof(portSTACK_TYPE))

#define CAN_THREAD_STACK_SIZE			(256/sizeof(portSTACK_TYPE))
#define TRANSMIT_THREAD_STACK_SIZE		((4096)/sizeof(portSTACK_TYPE))

#define HC_THREAD_PRIORITY				 1
#define LC_THREAD_PRIORITY				 1
#define SYSTEM_TASK_PRIORITY			 3

#define CAN_THREAD_PRIORITY				 0
#define TRANSMIT_PRIORITY				 2



#endif /* CONFIG_BLOCK_H_ */