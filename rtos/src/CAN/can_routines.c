/*
 * can_routines.c
 *
 * Created: 4/21/2018 3:39:38 PM
 *  Author: Jason
 */ 

#include <asf.h>
#include "PDU/PDU.h"
#include "PDU_CAN_conf.h"


const int	num_can_handlers = sizeof(recieve_dispatch_table) / sizeof(struct can_handler);

static inline can_routine* recieve_handler_parse(uint32_t id){
	for(uint8_t i = 0; i < num_can_handlers; i++ ){
		if(recieve_dispatch_table[i].can_id == id) return recieve_dispatch_table[i].callback;
	}
	return unused_id;
}

/*
* Handle received CAN messages
* mailbox: Pointer to mailbox holding newest message
*/


void handle_CAN(can_mb_conf_t mailbox){
	Union64 payload;
	payload.u32[0] = mailbox.ul_datal;
	payload.u32[1] = mailbox.ul_datah;
	
	recieve_handler_parse( (mailbox.ul_id | CAN_MID_MIDvA(mailbox.ul_fid) )& CAN_MID_MIDvA_Msk )( payload.u64 );
}

/*
* Service a State Request from Requesting ECU
*/
void enable_request(uint64_t payload){
	enable_mask = ((payload)) & ~error_mask;
	g_recv_timeout_cnt = 0;
	//set_enable(enable_mask, PDU_ON_STATE);
	//do this to ensure off request are serviced
	//TODO: someday refactor the port write code to make more sense
	//set_enable(~enable_mask, PDU_OFF_STATE);
}

void ecu_eng_data(uint64_t payload){
	vehicle.RPM = LSB0W(payload) * ECU_RPM_SCALE;
}

void ecu_vehicle_data(uint64_t payload){
	vehicle.fuel_pressure	= LSB0D(payload);
	//PDU.batt_volt			= ( LSB1D(payload) << 8 ) | LSB2D(payload);
	vehicle.eng_status		= (Tst_bits(LSB3D( payload ), 1));
}

void pwm_request(uint64_t payload){
	
	for(int i = 0; i < NUM_HC_CHANNEL; i++)
		{
		pwm_request_arr[i] = ((uint8_t *)( &( payload ) ))[i];
		if( pwm_request_arr[i] > max_duty_cycle ) pwm_request_arr[i] = max_duty_cycle;
		pwm_request_arr[i] = PWM_DUTY_CYCLE_OFFSET - pwm_request_arr[i];
		}
	
}

void unused_id(uint64_t payload){
	UNUSED(payload);
}

void can_thread(void* pvParameters){
	
		static volatile uint32_t status = 0;
		TickType_t xLastWakeTime;
		const TickType_t xFrequency = TASK_10_HZ;
		
		xLastWakeTime = xTaskGetTickCount();
		
		for (;;)
			{
				
			g_recv_timeout_cnt += xFrequency;

			//check enable mailbox(M0)
			status = can_get_status(PDU_CAN);
			//can_mailbox_read(PDU_CAN, &can_mailbox[0]);
			//if( status & GLOBAL_MAILBOX_MASK){
			status = can_mailbox_get_status(PDU_CAN, ctz(ENABLE_MB));
			if ((status & CAN_MSR_MRDY) == CAN_MSR_MRDY)
				{
#ifdef CAN_REC_THREAD_LED
				ioport_toggle_pin_level(CAN_REC_THREAD_LED);
#endif
				can_mailbox_read(PDU_CAN, &can_mailbox[ctz(ENABLE_MB)]);
				handle_CAN(can_mailbox[ctz(ENABLE_MB)]);
				}


			//check pwm mailbox(M1)
			status = can_mailbox_get_status(PDU_CAN,  ctz(PWM_MB));
			if ((status & CAN_MSR_MRDY) == CAN_MSR_MRDY)
				{
				can_mailbox_read(PDU_CAN, &can_mailbox[ctz(PWM_MB)]);
				handle_CAN(can_mailbox[ctz(PWM_MB)]);
				}
				
			//if new messages grab data from rest of mailboxes
			status = can_mailbox_get_status(PDU_CAN,  ctz(REC_MB));
			if ((status & CAN_MSR_MRDY) == CAN_MSR_MRDY)
			{
				can_mailbox_read(PDU_CAN, &can_mailbox[ctz(REC_MB)]);
				handle_CAN(can_mailbox[ctz(REC_MB)]);
			}
			

			
			// wait
			vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(xFrequency));
			}
		
} 

/*
* Handler for generic interrupts from PDU_CAN
* PDU is connected directly to vehicle ECU that requests PDU updates
* Interrupt source is detected and then handled as needed
*/
void CAN_HANDLER(void){
	//can_disable_interrupt(PDU_CAN, CAN_DISABLE_ALL_INTERRUPT_MASK);
	
	/*
	* Local status variable for use in interrupt handler
	* Used to hold/ and store result of determining source of the 
	* Triggering interrupt
	*/
	uint32_t status;
	status = can_get_status(PDU_CAN);				//get the entire status register
	
	
	/*
	* Interrupt Triggered by CAN bus off
	*/	
	if ( status & (CAN_SR_BOFF | CAN_SR_AERR) ){
		UBaseType_t uxSavedInterruptStatus;

		
		enable_mask = PDU_TIMEOUT_MASK;
		set_enable(enable_mask, PDU_ON_STATE);
		//do this to ensure off request are serviced
		//TODO: someday refactor the port write code to make more sense
		set_enable(~enable_mask, PDU_OFF_STATE);
		PDU.error_flag = true;
		
		
		can_init(PDU_CAN, ul_sysclk, PDU_CAN_BAUD);		
				
		/* Disable all CAN interrupts. */
		can_disable_interrupt(PDU_CAN, CAN_DISABLE_ALL_INTERRUPT_MASK);
		uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();
		can_enable_autobaud_listen_mode(PDU_CAN);
		//__disable_irq();
		while( ( can_get_status(PDU_CAN) & PDU_ALL_ERR_MASK ) );
		can_disable_autobaud_listen_mode(PDU_CAN);
		//__enable_irq();
		taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
		can_enable_interrupt(PDU_CAN, CAN_IER_BOFF);
		
	}
}