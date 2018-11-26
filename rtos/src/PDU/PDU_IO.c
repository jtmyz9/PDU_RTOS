/*
 * PDU_IO.c
 *
 * Created: 11/19/2018 6:10:46 PM
 *  Author: Jason
 */ 

#include "PDU.h"


void reset_line_handler(uint32_t id, uint32_t mask){
	printf("RESET\r");
	all_off();
	for(uint8_t index = 0; index < NUM_HC_CHANNEL; index++){
		pwm_channel_disable(PDU_PWM, pwm_assignment[index]);
		}
	delay_ms( FAULT_LATCH_TIME_MS_LC * 50 );
	NVIC_SystemReset();
	}
	
	
void all_off(void){
	enable_mask = 0;
	set_enable(PDU_ENABLES_PORTD_MASK, PDU_OFF_STATE );
	}

/*
* Set the supplied mask channels to supply ioport val
* Mask: is bit mask of channels
* enable:	is the port value to write to masked channels
* Used to either enable or disable supplied channels
*
*	Function is supplied with enable_mask value that is used to bit shift
*	to index in the port
*/
void set_enable(ioport_port_mask_t mask, enum ioport_value enable){
	//bit shift to index in port, and shift over by index of PDU channel zero
	ioport_set_port_level(PDU_ENABLE_PORT, mask, enable );
	}