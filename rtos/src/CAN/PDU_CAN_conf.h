/*
 * PDU_CAN_conf.h
 *
 * Created: 4/16/2018 7:30:37 PM
 *  Author: Jason
 */ 


#ifndef PDU_CAN_CONF_H_
#define PDU_CAN_CONF_H_

/************************************************************************/
/*           CAN BUS Definitions for PDU                                */
/************************************************************************/
#define PDU_CAN_SEL							0				//select CAN0 or CAN1 for preprocessor statements involving which can bus
#define PDU_CAN								CAN0
#define PDU_CAN_BAUD						CAN_BPS_1000K
#define PDU_CAN_DIAGNOSTIC_DELAY			50				//number of millisec until ECU can timeout
/*
 *	this is slightly higher than warning limit but below error passive limit for tx error cnt
 *	This makes it so that we don't keep waiting for a message to transmit
 *	in loops where we need contents of mailbox x to transmit before loading with next message
 */
#define PDU_CAN_TX_ERROR_CNT				100				

typedef enum{
	standard,
	extended
	}CAN_BUS_ID_TYPE;

/** CAN Bus Addresses */
#define PDU_ADDRES_TYPE						standard

//Control Messages( Request Messages)
#define PDU_ECU_REC_BLOCK_BASE				0x100
#define PDU_ECU_REC_ADDRESS					0x11A			//this is hard coded into ECU
#define PDU_PWM_UPDATE_ADD					0x120
//Vehicle and engine state messages
#define PDU_ECU_DATA_BLOCK_BASE				0x200
#define PDU_ECU_ENG_DATA_ADD				0x218
#define PDU_ECU_VEHICLE_DATA_ADD			0x219

#define PDU_BASE_TX_ADDRESS					0x500			//configured in ECU software


/** Specific definitions for CAN receive                                */
#define PDU_RECEIVE_MB						3
#define PDU_CAN_RECEIVE_IE_MASK				(CAN_IER_MB1 | CAN_IER_MB0)			//mask for interrupts to enable
#define PDU_ECU_GEN_MSG_ID_MASK				(((unsigned int)PDU_ECU_DATA_BLOCK_BASE | PDU_ECU_REC_BLOCK_BASE) << CAN_MID_MIDvA_Pos)

//TODO: add comments to explain wtf these errors represent, bit error, stuffing error, etc
#define	PDU_ALL_ERR_MASK					(CAN_SR_CERR | CAN_SR_SERR | CAN_SR_AERR |CAN_SR_FERR | CAN_SR_BERR)
// Awknowledge error or buss off error
#define PDU_CAN_IE_MASK						(CAN_IER_AERR | CAN_IER_BOFF)

// Mob assignments
#define ENABLE_MB							CAN_TCR_MB0
#define PWM_MB								CAN_TCR_MB1
#define REC_MB								CAN_TCR_MB2

// Mob assignment index(es)
#define ENABLE_MB_IDX						ctz(CAN_TCR_MB0)
#define PWM_MB_IDX							ctz(CAN_TCR_MB1)
#define REC_MB_IDX							ctz(CAN_TCR_MB2)

/** Definitions for CAN TX												*/
#define PDU_TX_MB							5
#define PDU_TX_STATUS_CNT					3

#define PDU_TX_FIRST_MB						PDU_RECEIVE_MB

#define PDU_STATUS_TX_MB					PDU_TX_FIRST_MB
#define PDU_OUTPUT_TX_MB					PDU_TX_FIRST_MB + 4
#define PDU_AMP_TX_MB						PDU_TX_FIRST_MB + 1
#define PDU_VOLT_TX_MB						PDU_TX_FIRST_MB + 3
#define PDU_TEMP_TX_MB						PDU_TX_FIRST_MB + 2
/** TODO: MAke this less hardcoded: find way to build this mask bases on how many TX messags */
#define PDU_TX_MB_MASK						( CAN_TCR_MB3 | CAN_TCR_MB4 | CAN_TCR_MB5 | CAN_TCR_MB6 | CAN_TCR_MB7 )

/*
* "Magic Numbers" for CAN data
* These are constant values used when encoding/decoding ECU can messages
* Some values are scales, others are for unit conversions
*/

#define ECU_RPM_SCALE						600


/*
* Values to be or'd with Byte 0 to set compound ID
* Detailed in Excel sheet
*/
typedef enum{
	state_zero,
	state_one		= 1 << 4,
	state_two		= 2 << 4,
	state_three		= 3 << 4
	}PDU_state_compound_id;
	
static const PDU_state_compound_id state_id[PDU_TX_MB]= {
	state_zero,
	state_one,
	state_two,
	state_three
};
	
typedef enum{
	output_zero,
	output_one		= 1 << 6,
	output_two		= 2 << 6,
	output_three	= 3 << 6,
	NUM_status		= 4
}PDU_output_compound_id;

static const PDU_output_compound_id PDU_status_id[NUM_status] = {
	output_zero, 
	output_one,
	output_two,
	output_three
};


/** CAN frame max data length: helpful def */
#define MAX_CAN_FRAME_DATA_LEN				8

//Alias for CAN0_Handler for PDU
#define	CAN_HANDLER							CAN0_Handler

typedef enum{
	no_receive, 
	rx_good,
	rx_timeout,
	rx_fault
	}can_receive_status;
	
typedef enum{
	tx_request,
	tx_good,
	tx_timeout,
	tx_fault
}can_transmit_status;

/************************************************************************/
/*					CAN BUS Dispatch Tables                             */
/************************************************************************/
/*
Dispatch tables are incredibly useful for translating a unique value, ie CANBUS
ID into the routine that needs to be run. This makes it possible to remove implementation
for handling particular messages from receive irq. 
*/

typedef void can_routine(uint64_t payload);		//typedefs in C are lowkey the worst thing imaginable 

can_routine enable_request, ecu_eng_data, ecu_vehicle_data, pwm_request, unused_id;

struct can_handler
{
	uint32_t can_id;
	can_routine *callback;
};

//void data_recieve(void);
//
static const struct can_handler  recieve_dispatch_table[] =  {
	{ CAN_MID_MIDvA(PDU_ECU_REC_ADDRESS),		enable_request		},
	{ CAN_MID_MIDvA(PDU_ECU_ENG_DATA_ADD),		ecu_eng_data		},
	{ CAN_MID_MIDvA(PDU_ECU_VEHICLE_DATA_ADD),	ecu_vehicle_data	},
	{ CAN_MID_MIDvA(PDU_PWM_UPDATE_ADD),		pwm_request			}	
	
};



/**
 * \brief Reset mailbox configure structure.
 *
 *  \param p_mailbox Pointer to mailbox configure structure.
 */
static void reset_mailbox_conf(can_mb_conf_t *p_mailbox)
{
	p_mailbox->ul_mb_idx = 0;
	p_mailbox->uc_obj_type = 0;
	p_mailbox->uc_id_ver = 0;
	p_mailbox->uc_length = 0;
	p_mailbox->uc_tx_prio = 0;
	p_mailbox->ul_status = 0;
	p_mailbox->ul_id_msk = 0;
	p_mailbox->ul_id = 0;
	p_mailbox->ul_fid = 0;
	p_mailbox->ul_datal = 0;
	p_mailbox->ul_datah = 0;
}

#endif /* PDU_CAN_CONF_H_ */