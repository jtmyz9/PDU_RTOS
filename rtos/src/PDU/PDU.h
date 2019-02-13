/*
 * PDU.h
 *
 * Created: 11/26/2017 5:40:22 PM
 *  Author: MizzouRacing
 */ 


#ifndef PDU_H_
#define PDU_H_

#include <asf.h>
#include <string.h>	
#include "conf_PDU.h"
#include "config_block.h"
#include "config/conf_board.h"
#include <math.h>

typedef struct PDM_state{
	uint16_t	batt_volt;
	uint16_t	rail_voltage_3_3;
	uint8_t		chip_temp;	
	uint8_t		error_flag;
	uint8_t		total_curr;	
}PDU_state_t;

typedef struct vehicle_data{
	uint32_t RPM;
	uint16_t coolant_temp;
	uint8_t  fuel_pressure;
	engine_state  eng_status;
}vehicle_data_t;

typedef struct output_data{
	uint32_t	overcurrent_time;
	uint16_t	voltage;
	uint8_t		current;
	uint8_t		last_curr;	
	uint8_t		temp;	//in celcius
	status_code state;	
}output_data_t;

typedef struct channel_config{
	uint32_t	inrush_delay;  //ms
	uint8_t		current_limit;
	
}channel_config_t;

typedef struct is_config{
	float		follower_ratio;	
	uint32_t	conversion_factor;
	uint32_t	offset;
	uint16_t	fault_min;
	uint16_t	open_load_min;
	uint8_t		scale_factor;
	
}is_config_t;

#ifndef PDU_ENABLE_PORT
	#error "PDM_ENABLE_PORT not defined"
#endif



/************************************************************************/
/* PDU Variables */
/************************************************************************/

/* 
* Mask of channels that should currently be enabled
* Inital value supplied by conf_PDU header
* Running state value retreive from ECU via CAN message
*/
volatile extern ioport_port_mask_t	enable_mask;
volatile extern	ioport_port_mask_t	soft_restart_mask;
volatile extern	ioport_port_mask_t	error_mask;
volatile extern	uint8_t				pwm_request_arr[NUM_HC_CHANNEL];
volatile extern uint8_t 			max_duty_cycle;

extern PDU_state_t					PDU;

extern vehicle_data_t				vehicle;

extern output_data_t				outputs[MAX_PDU_CHANNEL];
extern channel_config_t				config[MAX_PDU_CHANNEL];

extern uint32_t						ul_sysclk;

extern pwm_channel_t hc_pwm_channel[NUM_HC_CHANNEL];
/*
* CAN Transfer mailbox structure 
* Array of structs to be used for all the mailboxes in CAN controller
*/
can_mb_conf_t can_mailbox[CANMB_NUMBER];



extern volatile bool				update_outputs;

/**Time in ms, since last received message from ECU*/
extern volatile uint32_t 			g_recv_timeout_cnt;


/************************************************************************/
/* PDU AFEC Variables                                                   */
/************************************************************************/
struct afec_config			afec_cfg;
struct afec_config			afec_1_cfg;
struct afec_ch_config		afec_ch_cfg;
struct afec_ch_config		afec_1_ch_cfg;


// Thread synchronization for fet management/ current feedback checking
extern SemaphoreHandle_t	afec_semaphore;
extern SemaphoreHandle_t	afec2_semaphore;
extern SemaphoreHandle_t	afec_complete_buffer;
/*
* Set the supplied mask channels to supply ioport val
* Mask: is bit mask of channels 
* enable:	is the port value to write to masked channels
* Used to either enable or disable supplied channels
*/
void set_enable(ioport_port_mask_t mask, enum ioport_value enable);

/*
* Set all channels in the PDU to off state
* Used during initialization to ensure low states are asserted
* Also in error state where all outputs must be disabled
*/
void all_off(void);

void reset_line_handler(uint32_t id, uint32_t mask);

void update_pwm_request(void);
state_request output_state_machine(output_data_t* output_channel, uint32_t chan_num, is_config_t channel_is_config);
void state_machine(void);
void read_can(void);

void get_is( Afec *const afec ,enum afec_channel_num pdu_channel, is_config_t config, uint8_t* current);
void get_fet_temp(Afec *const afec, enum afec_channel_num afec_channel, uint8_t* temp);

void update_output_status(void);
void update_fault_states(void);

void get_chip_temp(uint8_t *temp);

/*
 *	Prod: soft_restart
 *	Will enact a software restart of channels
 *	will restart all the channels in the provided mask
 */
void soft_restart(pdu_enable_index channel_num, uint32_t latch_time);

/*
* Initialize the PDU
* Setup the I/O port used for enabling FET's
*		I/O Port is defined as PDU_ENABLE_PORT in config header
* Setup the ADC for reading the current values from each FET
*/
void init_PDU(void);

void pdu_afec_init(void);
void pdu_can_init(void);
void pdu_pwm_init(void);
void pdu_ioinit(void);

void ECU_timeout_callback(void);
void DIO_callback(void);

void PDU_system_startup(void);

/**
* Main application threads for managing high side drivers
* pvParameters are identifying parameters to be passed into thread
* ie. channel number for indexing
**/
void HC_thread	(void* pvParameters );
void LC_thread	(void* pvParameters );

/*
* System task to handle background task and general 
* system management that doesn't fall under domain of fet mangement
*/
void system_task(void* pvParameters);
void get_system_voltages(uint16_t* batt_volt, uint16_t* supply_volt);
//super secret cheaty sauce, no one but me should know this exists
void monitor_BSPD(uint16_t* bspd_voltage);

/*
* Can Bus Threads and functions
*/
extern void can_thread			(void* pvParameters );
extern void handle_CAN			(can_mb_conf_t mailbox);

void transmit_status			(void* pvParameters);
void transmit_output_current	(void* pvParameters);
void transmit_output_voltage	(void* pvParameters);
void transmit_output_state		(void* pvParameters);
void transmit_output_temp		(void* pvParameters);



#endif /* PDU_H_ */