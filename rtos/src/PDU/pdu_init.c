/*
 * pdu_init.c
 *
 * Created: 11/10/2018 2:13:14 PM
 *  Author: Jason McLoud
 */ 

#include "PDU.h"


/**
 * \brief Set peripheral mode for IOPORT pins.
 * It will configure port mode and disable pin mode (but enable peripheral).
 * \param port IOPORT port to configure
 * \param masks IOPORT pin masks to configure
 * \param mode Mode masks to configure for the specified pin (\ref ioport_modes)
 */
#define ioport_set_port_peripheral_mode(port, masks, mode) \
	do {\
		ioport_set_port_mode(port, masks, mode);\
		ioport_disable_port(port, masks);\
	} while (0)

/**
 * \brief Set peripheral mode for one single IOPORT pin.
 * It will configure port mode and disable pin mode (but enable peripheral).
 * \param pin IOPORT pin to configure
 * \param mode Mode masks to configure for the specified pin (\ref ioport_modes)
 */
#define ioport_set_pin_peripheral_mode(pin, mode) \
	do {\
		ioport_set_pin_mode(pin, mode);\
		ioport_disable_pin(pin);\
	} while (0)

/**
 * \brief Set input mode for one single IOPORT pin.
 * It will configure port mode and disable pin mode (but enable peripheral).
 * \param pin IOPORT pin to configure
 * \param mode Mode masks to configure for the specified pin (\ref ioport_modes)
 * \param sense Sense for interrupt detection (\ref ioport_sense)
 */
#define ioport_set_pin_input_mode(pin, mode, sense) \
	do {\
		ioport_set_pin_dir(pin, IOPORT_DIR_INPUT);\
		ioport_set_pin_mode(pin, mode);\
		ioport_set_pin_sense_mode(pin, sense);\
	} while (0)



void init_PDU(void){
	
	
	
	pdu_ioinit();
	pdu_pwm_init();
	pdu_can_init();
	pdu_afec_init();
	afec_semaphore = xSemaphoreCreateMutex();
	afec2_semaphore = xSemaphoreCreateMutex();
	afec_complete_buffer = xSemaphoreCreateBinary();
	
	enable_mask = PDU_NON_ECU_MASK;
	
	for(uint8_t index = 0; index < NUM_LC_CHANNEL; index++){
		ioport_set_pin_dir(PDU_DEN_iopins[index], IOPORT_DIR_OUTPUT);
		ioport_set_pin_level(PDU_DEN_iopins[index], PDU_ON_STATE);
		
		ioport_set_pin_dir(PDU_DSEL_iopins[index], IOPORT_DIR_OUTPUT);
		ioport_set_pin_level(PDU_DSEL_iopins[index], PDU_OFF_STATE);
		}
	
	
}


/*
* PDU_IOINIT
* FUnction to intialize all the I/O pins and services for PDU
* shall setup pins as inputs/outputs and configure necessary ISR(s)
*/
void pdu_ioinit(void){
	//setup the port for the PDU
	ioport_enable_port(PDU_ENABLE_PORT, PDU_ENABLES_PORTD_MASK);
	//set all the pins in the PORT MASK as outputs
	ioport_set_port_dir(PDU_ENABLE_PORT, PDU_ENABLES_PORTD_MASK, IOPORT_DIR_OUTPUT);
	
	//TODO: make this not shit later, just wanted this here for now to have status led
	ioport_set_pin_dir(STATUS_LED1_GPIO, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(STATUS_LED1_GPIO, LED0_INACTIVE_LEVEL);
	ioport_set_pin_dir(STATUS_LED2_GPIO, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(STATUS_LED2_GPIO, LED0_INACTIVE_LEVEL);
	ioport_set_pin_dir(STATUS_LED3_GPIO, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(STATUS_LED3_GPIO, LED0_INACTIVE_LEVEL);

	for( uint8_t iopin = 0; iopin < NUM_DIGITAL_IO; iopin++){
		
		pio_set_input(PIOD, PDU_digital_iopoins[iopin], PIO_PULLUP);
		//pio_handler_set(arch_ioport_pin_to_base(PDU_digital_iopoins[iopin]), PDU_digital_iopoins[iopin], );
		//ioport_set_pin_dir(PDU_digital_iopoins[iopin], IOPORT_DIR_INPUT);
		//ioport_set_pin_input_mode(PDU_digital_iopoins[iopin], PDU_DIO_INPUT_FLAGS, PDU_DIO_SENSE_LEVEL);
		//ioport_set_pin_mode(PDU_digital_iopoins[iopin], (IOPORT_MODE_PULLUP | IOPORT_MODE_DEBOUNCE | IOPORT_MODE_GLITCH_FILTER)); 
		}
		
	pio_handler_set(PIOD, ID_PIOD, RESET_LINE, PIO_IT_LOW_LEVEL, reset_line_handler);
	pio_enable_interrupt(PIOD, RESET_LINE);
	NVIC_EnableIRQ(PIOD_IRQn);
	}

void pdu_afec_init(void){
		
	/*
	* High Current afec Enable  
	*/
	afec_enable(IS_AFEC);
	afec_get_config_defaults(&afec_cfg);	
	afec_cfg.resolution = AFEC_16_BITS;
	//afec_cfg.anach = false;
	afec_init(IS_AFEC, &afec_cfg);
	afec_set_trigger(IS_AFEC, AFEC_TRIG_SW);
	
	//get default config for a channel
	afec_ch_get_config_defaults(&afec_ch_cfg);
	for(uint8_t i = 0; i < NUM_HC_CHANNEL; i++){
		//set the config for the AFEC channel
		afec_ch_set_config(IS_AFEC, HC_AFEC_channel_list[i], &afec_ch_cfg);
		afec_ch_set_config(IS_AFEC, HC_temp_channel_list[i], &afec_ch_cfg);
		//move offset to measure voltage from 0->Vref
		afec_channel_set_analog_offset(IS_AFEC, HC_AFEC_channel_list[i],AFEC_ANALOG_OFFSET);
		afec_channel_set_analog_offset(IS_AFEC, HC_temp_channel_list[i],AFEC_ANALOG_OFFSET);
	}
	
	// Initialize System voltage sensor AFEC
	for(uint8_t index = 0; index < NUM_SUPPLY_MONITOR; index++){
		afec_ch_set_config( PDU_AFEC, supply_monitor_list[index], &afec_ch_cfg );
		afec_channel_set_analog_offset( PDU_AFEC, supply_monitor_list[index], AFEC_ANALOG_OFFSET );
		}
	
	//afec_enable_interrupt(HA_AFEC, 0xFFFF);
	/*
	* Low Current Afec Enable  
	*/
	afec_enable(LA_AFEC);
	afec_get_config_defaults(&afec_1_cfg);
	afec_1_cfg.resolution = AFEC_16_BITS;
	//afec_1_cfg.anach = false;
	afec_init(LA_AFEC, &afec_1_cfg);
	afec_set_trigger(LA_AFEC, AFEC_TRIG_SW);
	
	//get default config for a channel
	afec_ch_get_config_defaults(&afec_1_ch_cfg);
	for(uint8_t i = 0; i < NUM_LC_CHANNEL/2; i++){
		//set the config for the AFEC channel
		afec_ch_set_config(LA_AFEC, LC_AFEC_channel_list[i], &afec_1_ch_cfg);
		//move offset to measure voltage from 0->Vref
		afec_channel_set_analog_offset(LA_AFEC, LC_AFEC_channel_list[i], AFEC_ANALOG_OFFSET );
	}
	//afec_set_callback(AFEC0, AFEC_INTERRUPT_DATA_READY, afec_data_ready, 1);
	

	/*Use AFEC0 here because the temperature sensor is hardwired to AFEC0*/	 
	afec_ch_set_config(AFEC0, AFEC_TEMPERATURE_SENSOR, &afec_ch_cfg);	
	/*
	 * Because the internal ADC offset is 0x800, it should cancel it and shift
	 * down to 0.
	 */
	afec_channel_set_analog_offset(AFEC0, AFEC_TEMPERATURE_SENSOR, AFEC_ANALOG_OFFSET);
	
	struct afec_temp_sensor_config afec_temp_sensor_cfg;

	afec_temp_sensor_get_config_defaults(&afec_temp_sensor_cfg);
	afec_temp_sensor_cfg.rctc = false;
	afec_temp_sensor_set_config(AFEC0, &afec_temp_sensor_cfg);
	
	//afec_set_callback(AFEC0, AFEC_INTERRUPT_EOC_15,	get_chip_temp, 1);
	//NVIC_EnableIRQ(AFEC0_IRQn);
}

void pdu_can_init(void){
	/*
	* Initialize the CAN Bus for the PDU
	* Setup mailbox structs 
	*/
	//NOTE: CAN peripherial must first be initialized in board init()
	can_init(PDU_CAN, ul_sysclk, PDU_CAN_BAUD);	
	
	can_enable_autobaud_listen_mode(PDU_CAN);
	while( ( can_get_status(PDU_CAN) & PDU_ALL_ERR_MASK ) );
	can_disable_autobaud_listen_mode(PDU_CAN);

	//make sure all mailbox structs are zeroed
	for( int i =0; i < CANMB_NUMBER; i ++){
		reset_mailbox_conf(&can_mailbox[i]);
	}
	
	//the first mailbox is always the ECU receive mailbox
	can_mailbox[ENABLE_MB_IDX].ul_mb_idx = ENABLE_MB_IDX;
	
	can_mailbox[ENABLE_MB_IDX].uc_obj_type = CAN_MB_RX_OVER_WR_MODE;
	can_mailbox[ENABLE_MB_IDX].ul_id_msk = CAN_MAM_MIDvA_Msk | CAN_MAM_MIDvB_Msk;		//this masks all id bits
	can_mailbox[ENABLE_MB_IDX].ul_id = CAN_MID_MIDvA(PDU_ECU_REC_ADDRESS);

	can_mailbox_init(PDU_CAN, &(can_mailbox[ENABLE_MB_IDX]));
		
	///* second mailbox is always the receive mailbox for pwm request */
	can_mailbox[PWM_MB_IDX].ul_mb_idx = PWM_MB_IDX;
	can_mailbox[PWM_MB_IDX].uc_obj_type = CAN_MB_RX_OVER_WR_MODE;
	can_mailbox[PWM_MB_IDX].ul_id_msk = CAN_MAM_MIDvA_Msk | CAN_MAM_MIDvB_Msk;		//this masks all id bits
	can_mailbox[PWM_MB_IDX].ul_id = CAN_MID_MIDvA(PDU_PWM_UPDATE_ADD);
	can_mailbox_init(PDU_CAN, &can_mailbox[PWM_MB_IDX]);
	
	/* third mailbox is always the ECU receive mailbox for vehicle data */
	can_mailbox[REC_MB_IDX].ul_mb_idx = REC_MB_IDX;
	can_mailbox[REC_MB_IDX].uc_obj_type = CAN_MB_RX_OVER_WR_MODE;
	can_mailbox[REC_MB_IDX].ul_id_msk = PDU_ECU_GEN_MSG_ID_MASK;		//this will only allow all generic messages from ECU
	can_mailbox[REC_MB_IDX].ul_id = CAN_MID_MIDvA(PDU_ECU_DATA_BLOCK_BASE);
	can_mailbox_init(PDU_CAN, &can_mailbox[REC_MB_IDX]);
	

	
	/** Init CAN bus TX mailboxes */
	for( uint8_t i = PDU_TX_FIRST_MB; i < PDU_TX_MB + PDU_TX_FIRST_MB; i++){
		can_mailbox[i].ul_mb_idx = i;
		can_mailbox[i].uc_obj_type = CAN_MB_TX_MODE;
		can_mailbox[i].ul_id_msk = 0;
		can_mailbox[i].ul_id = CAN_MID_MIDvA(PDU_BASE_TX_ADDRESS + (i - PDU_TX_FIRST_MB));
		can_mailbox_init(PDU_CAN, &can_mailbox[i]);
	}
	
	//can_enable_interrupt(PDU_CAN, PDU_CAN_IE_MASK);
	//NVIC_SetPriority(CAN0_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
	//NVIC_EnableIRQ(CAN0_IRQn);
	

}

void pdu_pwm_init(void){
	
	
	pwm_clock_t clock_setting = {
		.ul_clka = PDU_PWM_FREQUENCY * PDU_PERIOD_VALUE,
		.ul_clkb = 0,
		.ul_mck = ul_sysclk
	};
	pwm_init(PWM, &clock_setting);
	
	for(int i =0; i < NUM_HC_CHANNEL; i++){
		/* Disable channel counter event interrupt */
		pwm_channel_disable_interrupt(PWM, pwm_assignment[i], 0);
		
		/* Initialize PWM channel for HCx */
		/* Period is left-aligned */
		hc_pwm_channel[i].alignment = PWM_ALIGN_LEFT;
		/* Output waveform starts at a low level */
		hc_pwm_channel[i].polarity = PWM_LOW;
		/* Use PWM clock A as source clock */
		hc_pwm_channel[i].ul_prescaler = PWM_CMR_CPRE_CLKA;
		/* Period value of output waveform */
		hc_pwm_channel[i].ul_period = PDU_PERIOD_VALUE;
		/* Duty cycle value of output waveform */
		hc_pwm_channel[i].ul_duty = PDU_DEFAULT_DUTY_VALUE;
		hc_pwm_channel[i].channel = pwm_assignment[i];
	
		pwm_channel_init(PWM, &(hc_pwm_channel[i]));
		pwm_channel_enable_interrupt(PWM, pwm_assignment[i], 0);

	}
		
}


/*
* System Startup function
* This function shall create all the necessary threads
* for PDU functionality
*/
void PDU_system_startup(void){
		for (U8 i = 0; i < NUM_HC_CHANNEL; i++)
		{
			if (xTaskCreate(HC_thread,
			"HC Thread",
			HC_THREAD_STACK_SIZE,
			(void*) i,				// channel number
			HC_THREAD_PRIORITY,
			NULL) != pdPASS)
			{printf("Failed to create a HC task %d\r\n", i); }

		}
		
		for (U8 i = NUM_HC_CHANNEL; i < NUM_PDU_CHANNEL; i++)
			{
			if (xTaskCreate(LC_thread,
				"LC Thread",
				LC_THREAD_STACK_SIZE,
				(void*) i,				// channel number
				LC_THREAD_PRIORITY,
				NULL) != pdPASS)
				{printf("Failed to create a LC task %d\r\n", i); }		
			}
		
		if (xTaskCreate(can_thread,
			"CAN Thread",
			CAN_THREAD_STACK_SIZE,
			NULL,
			1,
			NULL) != pdPASS)
			{ printf("Failed to create can task\r\n"); }
		
		if (xTaskCreate(transmit_status,
			"Status Thread",
			CAN_THREAD_STACK_SIZE,
			NULL,
			TRANSMIT_PRIORITY,
			NULL) != pdPASS)
			{ printf("Failed to create can task\r\n"); }
		
		
		if (xTaskCreate(transmit_output_current,
			"Current Thread",
			CAN_THREAD_STACK_SIZE,
			NULL,
			TRANSMIT_PRIORITY,
			NULL) != pdPASS)
			{ printf("Failed to create can task\r\n"); }
		
		
		if (xTaskCreate(transmit_output_state,
			"State Thread",
			CAN_THREAD_STACK_SIZE,
			NULL,
			TRANSMIT_PRIORITY,
			NULL) != pdPASS)
			{ printf("Failed to create can task\r\n"); }
		
		if (xTaskCreate(system_task,
			"System Thread",
			SYSTEM_THREAD_STACK_SIZE,
			NULL,
			SYSTEM_TASK_PRIORITY,
			NULL) != pdPASS)
			{ printf("Failed to create can task\r\n"); }
	}

