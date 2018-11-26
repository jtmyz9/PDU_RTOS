/*
 * PDU.c
 *
 * Created: 11/26/2017 5:50:23 PM
 *  Author: MizzouRacing
 */ 


#include "PDU.h"

/*
* Main system thread 
* Setup to perform background task and throw system faults
* when current system status requires
*/
void system_task(void* pvParameters){
	
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = TASK_1_HZ;
	xLastWakeTime = xTaskGetTickCount();

	for (;;)
		{	
#ifdef SYS_THREAD_LED
		// Toggle light to indicate system is still alive
		ioport_toggle_pin_level(SYS_THREAD_LED);
#endif
		//Check for CAN bus timeout fault, ie we haven't heard from ECU 
		if(g_recv_timeout_cnt > CAN_TIMEOUT_LIMIT){
			ECU_timeout_callback();
			}
			
		if(PDU.chip_temp > PDU_OVERTEMP){
			//this is an absolute hard fault, we want everything to stop
			vTaskSuspendAll();
			__disable_irq();
			all_off();
			for(U8 i =0; i < NUM_HC_CHANNEL; i++){pwm_channel_disable(PDU_PWM, pwm_assignment[i]);}
			for(;;){}
		}
		
		xSemaphoreTake(afec_semaphore, portMAX_DELAY);
		//afec_channel_enable(AFEC0, AFEC_TEMPERATURE_SENSOR);
		//get 12v and 3.3v status
		get_system_voltages(&PDU.batt_volt, &PDU.rail_voltage_3_3);
		get_chip_temp(&PDU.chip_temp);

		xSemaphoreGive( afec_semaphore );

		//battery voltage -> pwm max shift
		if(vehicle.eng_status == true){
			for(uint8_t voltage = 0; voltage < NUM_VOLTAGE_LVL; voltage++){
				if( PDU.batt_volt < ( pwm_voltage_levels[voltage] * VOLTAGE_SCALE_FACTOR )){
					 max_duty_cycle = pwm_staturation_limits[voltage];
					 break;
					}
				}
			}
		if(error_mask){
			PDU.error_flag = true;
			}
#ifdef ENDURANCE_SWITCH
		
		for(uint8_t index; index < NUM_PDU_CHANNEL; index++){
			config[index].current_limit = UINT32_MAX;
			}
			
		// Check BSPD Voltage and switch extra channel if needed
		if( vehicle.eng_status == run){
			//Switch on secondary channel to override the BSPD, because safety third
			enable_mask |= (1<<BSPD);
			}
			
		
#endif			
		

		// wait
		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS( xFrequency ));
		}
	
	
	
}
	
/*
* This function shall represent a generic handling
* for one of the high(er) current powerfets on the PDU
*
* This shall call functions to update current feedback,
* update current status registers, and mutual exclusion needed
* to keep sampling running
*
*/
void HC_thread(void* pvParameters){
	
	volatile UBaseType_t uxHighWaterMark;
	
	// init channel and data structure(s)
	pdu_enable_index channel_num = (uint32_t) pvParameters;
	is_config_t channel_is_config = {
		.offset				= PDU_AFEC_channel_offset[channel_num],
		.conversion_factor	= PDU_SENSE_MV_TO_MA_HC,
		.follower_ratio		= PDU_FET_DIFFERENTIAL_RATIO_HC,
		.scale_factor		= SCALE_FACTOR_HC,
		.fault_min			= IS_FAULT_MIN_A_HC,
		.open_load_min		= IS_OPEN_MIN_A_HC,
		};

	outputs[channel_num].state = CHANNEL_OFF;
	config[channel_num].current_limit = curr_limits[channel_num] * SCALE_FACTOR_HC;
	config[channel_num].inrush_delay = INRUSH_AFEC_DELAY;

	
	uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
	
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = TASK_10_HZ;
	xLastWakeTime = xTaskGetTickCount();
	
	for(;;){
		// lock mutex( I/O & ADC resource)
		xSemaphoreTake(afec_semaphore, portMAX_DELAY );
		// if on bit not set, run open load check
		//if(enable_mask && (1<<channel_num)){
			////set open load on
			////i2c command
			////delay for t-reset time to polarize output and trigger fault
			//vTaskDelay((const TickType_t) 55 / portTICK_PERIOD_MS);
			//
		//}
		afec_channel_enable(IS_AFEC, HC_AFEC_channel_list[channel_num]);
		afec_channel_enable(IS_AFEC, HC_temp_channel_list[channel_num]);
		afec_start_software_conversion(IS_AFEC);
		// check current feedback
		get_is(IS_AFEC, HC_AFEC_channel_list[channel_num], channel_is_config, &(outputs[channel_num].current));
		
		// get chip temp
		get_fet_temp(IS_AFEC, HC_temp_channel_list[channel_num], &(outputs[channel_num].temp));
		afec_channel_disable(IS_AFEC, HC_AFEC_channel_list[channel_num]);
		afec_channel_disable(IS_AFEC, HC_temp_channel_list[channel_num]);
		
		// unlock mutex( I/O & ADC resource)
		xSemaphoreGive(afec_semaphore);
		
		//update pwm duty cycle
		if ( hc_pwm_channel[channel_num].ul_duty != pwm_request_arr[channel_num] )
			{
			hc_pwm_channel[channel_num].ul_duty = pwm_request_arr[channel_num];
			pwm_channel_update_duty(PDU_PWM, &hc_pwm_channel[channel_num], hc_pwm_channel[channel_num].ul_duty);
			}		
		
		//calc state machine
		state_request req = output_state_machine(&outputs[channel_num], channel_num, channel_is_config);
		switch(req){
			case ON_REQUEST:
				pwm_channel_enable(PWM, pwm_assignment[channel_num]);
				
				for(int i = 100; i > pwm_request_arr[channel_num]; i-=10){
					pwm_channel_update_duty(PDU_PWM, &hc_pwm_channel[channel_num], i);
					vTaskDelay(pdMS_TO_TICKS(1));
					}
				pwm_channel_update_duty(PDU_PWM, &hc_pwm_channel[channel_num], hc_pwm_channel[channel_num].ul_duty);
				outputs[channel_num].state = CHANNEL_ON;
				break;
				
			case OFF_REQUEST:
				hc_pwm_channel[channel_num].ul_duty = PWM_STARTUP_DUTY_CYCLE;
				pwm_channel_disable(PWM, pwm_assignment[channel_num]);
				outputs[channel_num].state = CHANNEL_OFF;
				break;
				
			case OVER_CURRENT_TRANSITION:
				pwm_channel_disable(PWM, pwm_assignment[channel_num]);
				Set_bits(error_mask, (1<<channel_num));
				outputs[channel_num].state = CHANNEL_OVER_CURRENT;
				break;
				
			case OPEN_TRANSITION:
				outputs[channel_num].state = CHANNEL_OPEN;
				break;
				
			case FAULT_TRANSITION:
				pwm_channel_disable(PWM, pwm_assignment[channel_num]);
				Set_bits(error_mask, (1<<channel_num));
				outputs[channel_num].state = CHANNEL_FAULT;
				break;
				
			case CLEAR_FAULT_TRANSITION:
				outputs[channel_num].state = CHANNEL_CLEAR_FAULT;
				soft_restart(channel_num, FAULT_LATCH_TIME_MS_HC);
				Clr_bits(error_mask, (1<<channel_num));
				break;
				
			case NO_REQUEST:
				break;
				
			default:
				//do we throw a fault here, this would represent something fucked up
				break;
			}
		uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
#ifdef HC_THREAD_LED
		ioport_toggle_pin_level(HC_THREAD_LED);
#endif

		// wait
		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(xFrequency));
		}
}

state_request output_state_machine(output_data_t* output_channel, uint32_t chan_num, is_config_t channel_is_config){
	//init/off
	if(output_channel->state == CHANNEL_OFF){
		if(Tst_bits( enable_mask, 1<<chan_num ) & !Tst_bits( error_mask, 1<<chan_num )) return ON_REQUEST;
		else if(output_channel->current > channel_is_config.fault_min) return FAULT_TRANSITION;
		else if(output_channel->current > channel_is_config.open_load_min) return OPEN_TRANSITION;
		
		}
	//on
	else if(output_channel->state == CHANNEL_ON){
		// If an off request is recieved
		if(!Tst_bits(enable_mask, 1<<chan_num)){
			output_channel->overcurrent_time = 0;
			return OFF_REQUEST;
			}
		// If the channel's IS is above its Fault limit, indicating a short/Fault
		else if(output_channel->current > channel_is_config.fault_min){
			return FAULT_TRANSITION;
			}
		// If the channel is overcurrent 
		else if(output_channel->current				> config[chan_num].current_limit){
			output_channel->overcurrent_time++;
			}
		//
		else if(output_channel->current				> config[chan_num].current_limit &&
			    output_channel->overcurrent_time	> config[chan_num].inrush_delay){
					return OVER_CURRENT_TRANSITION;
					}
				}

	//overcurrent
	else if(output_channel->state == CHANNEL_OVER_CURRENT){
		// Reset if an off request is recieved for channel
		if(!Tst_bits(enable_mask, 1<<chan_num)) 			return OFF_REQUEST;
		// if this is a channel we want to try to turn back on after software fuse
		else if(Tst_bits(soft_restart_mask, 1<<chan_num))	return ON_REQUEST;

		}
	//fault
	else if(output_channel->state == CHANNEL_FAULT){
		if(Tst_bits(soft_restart_mask, 1<<chan_num)) return CLEAR_FAULT_TRANSITION;
		}
	//clear fault
	else if(output_channel->state == CHANNEL_CLEAR_FAULT){
		// if the channel is still short circuited 
		if(output_channel->current > IS_FAULT_MIN_HC) return FAULT_TRANSITION;
		}
	//open load
	else if(output_channel->state == CHANNEL_OPEN){
		if(output_channel->current < IS_OPEN_MIN_HC) return OFF_REQUEST;
		}
	
	return NO_REQUEST;
}

/*
* This function shall represent a generic handling
* for one channel of dual channel powerfets on the PDU
*
* This shall call functions to update current feedback,
* update current status registers, and mutual exclusion needed
* to keep sampling running
*
*/
void LC_thread(void* pvParameters){
	
	volatile UBaseType_t uxHighWaterMark;
	
	// init channel and data structure(s)
	pdu_enable_index channel_num = (uint32_t) pvParameters;
	pdu_enable_index channel_index = (channel_num - NUM_HC_CHANNEL) / 2;
	
	
	is_config_t channel_is_config = {
		.offset				= PDU_AFEC_channel_offset[channel_num],
		.conversion_factor	= PDU_SENSE_MV_TO_MA_LC,
		.follower_ratio		= PDU_FET_DIFFERENTIAL_RATIO_LC,
		.scale_factor		= SCALE_FACTOR_LC,
		.fault_min			= IS_FAULT_MIN_A_LC,
		.open_load_min		= IS_OPEN_MIN_A_LC,
		};
	
	// Instantiate the output array 	
	outputs[channel_num].state = CHANNEL_OFF;
	outputs[channel_num].current = 0xAA;
	
	config[channel_num].current_limit = curr_limits[channel_num] * SCALE_FACTOR_LC;
	config[channel_num].inrush_delay = INRUSH_AFEC_DELAY;	
	
	uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
		
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = TASK_10_HZ;
	xLastWakeTime = xTaskGetTickCount();
		
	for(;;){
		// lock mutex( I/O & ADC resource)
		xSemaphoreTake(afec2_semaphore, portMAX_DELAY );
		// if on bit not set, run open load check
		//if(enable_mask && (1<<channel_num)){
		////set open load on
		////i2c command
		////delay for t-reset time to polarize output and trigger fault
		//vTaskDelay((const TickType_t) 55 / portTICK_PERIOD_MS);
		//
		//}
		
		ioport_set_pin_level(PDU_DSEL_iopins[channel_index], channel_num % 2 ? PDU_ON_STATE : PDU_OFF_STATE);
		
		// if the Diag Enable pin is not on, the multiplexed channel is attempted a fault reset
		if( ioport_get_pin_level(PDU_DEN_iopins[channel_index]) ){
			afec_channel_enable(LA_AFEC, LC_AFEC_channel_list[channel_index]);
			afec_start_software_conversion(LA_AFEC);
			// check current feedback
			get_is( LA_AFEC, LC_AFEC_channel_list[channel_index], channel_is_config, &(outputs[channel_num].current) );			
			afec_channel_disable(LA_AFEC, LC_AFEC_channel_list[channel_index]);
		}
	
			
		// unlock mutex( I/O & ADC resource)
		xSemaphoreGive(afec2_semaphore);
		
		//calc state machine
		state_request req = output_state_machine(&outputs[channel_num], channel_num, channel_is_config);
		switch(req){
			case ON_REQUEST:
				for(U8 attempt = 0; attempt < PRECHARGE_ATTEMPTS; attempt++){
					ioport_set_pin_level(PDU_iopins[channel_num], PDU_ON_STATE);
					vTaskDelay(pdMS_TO_TICKS(PRECHARGE_TIME));
					ioport_set_pin_level(PDU_iopins[channel_num], PDU_OFF_STATE);
					}
				ioport_set_pin_level(PDU_iopins[channel_num], PDU_ON_STATE);
				outputs[channel_num].state = CHANNEL_ON;
				break;
			case OFF_REQUEST:
				ioport_set_pin_level(PDU_iopins[channel_num], PDU_OFF_STATE);
				outputs[channel_num].state = CHANNEL_OFF;
				break;
			case OVER_CURRENT_TRANSITION:
				ioport_set_pin_level(PDU_iopins[channel_num], PDU_OFF_STATE);
				Set_bits(error_mask, (1<<channel_num));
				outputs[channel_num].state = CHANNEL_OVER_CURRENT;
				break;
			case OPEN_TRANSITION:
				outputs[channel_num].state = CHANNEL_OPEN;
				break;
			case FAULT_TRANSITION:
				ioport_set_pin_level(PDU_iopins[channel_num], PDU_OFF_STATE);
				Set_bits(error_mask, (1<<channel_num));
				outputs[channel_num].state = CHANNEL_FAULT;
				break;
			case CLEAR_FAULT_TRANSITION:
				outputs[channel_num].state = CHANNEL_CLEAR_FAULT;
				soft_restart(channel_num, FAULT_LATCH_TIME_MS_LC);
				Clr_bits(error_mask, (1<<channel_num));
				break;
			case NO_REQUEST:
				break;
			default:
				//do we throw a fault here, this would represent something fucked up
				break;
			}
			
		uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
		
		// wait
		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(xFrequency));
		}
	}





/*
 *	Procedure: get_is
 *	Get the current sense feedback for the selected pdu_channel
 *	Where pdu_channel is the requested channel, offset is the analog offset of the 
 *	sense feedback for that channel and afec is the PDU AFEC
 *
 *	Returns the sense feedback value in amps, but value is also multiplied by the configured
 *	scale factor to get the desired significant digit, ie scale factor of 10 encodes 7.3 as 73 
 *	for CAN Bus transmitting
 *
 *	If the current conversion detects a fault returns uint8_t_MAX
 */
void get_is( Afec *const afec ,enum afec_channel_num pdu_channel, is_config_t channel_is_config, uint8_t* current ){	
	
	while(  !(afec_get_interrupt_status( afec ) & ( 1 << pdu_channel )));
	float conversion_result = afec_channel_get_value( afec, pdu_channel );
	
	//transform AFEC result into mV
	conversion_result = conversion_result * AFEC_conversion_factor_mV;
	conversion_result = conversion_result - channel_is_config.offset;
	///*
	 //*	If the converted amperage is at the minimun value for fault 
	 //*  indication, this is either a short circuit event or fet is over temp
	 //*/
	//if ( conversion_result > IS_FAULT_MIN_HC) *current = UINT8_MAX; // should change this behaviour because this could also be a valid returned value after value is scaled
	
	//transform into number of mA
	conversion_result = conversion_result / channel_is_config.conversion_factor;
	
	//multiply by differential ratio to find load A
	conversion_result = conversion_result * channel_is_config.follower_ratio;
	conversion_result = conversion_result * channel_is_config.scale_factor;
		
	//if the converted amperage is 0 or negative due to not perfect offset calibration
	conversion_result = max(0, conversion_result);
	//clamp value to size of data structure
	conversion_result = min(UINT8_MAX+1, conversion_result);
	/*
	 *	return the converted value
	 *	The value is multiplied by the desired scale factor
	 *	to transmit the desired significant digits
	 */
	(*current) = (uint8_t)conversion_result;
	}
	

/*
 *	Procedure: get_fet_temp
 */
void get_fet_temp( Afec *const afec ,enum afec_channel_num afec_channel, uint8_t* temp){	
	
	while(  !(afec_get_interrupt_status(afec) & (1 << afec_channel))  );
	volatile float conversion_result = afec_channel_get_value(afec, afec_channel);
	volatile float tKelvin = 0.0;
	
	// Calculate thermistor R
	conversion_result = (( AFEC_DIGITAL_MAX / conversion_result ) - 1 ) * ( R_BALANCE);
	tKelvin = (BETA * ROOM_TEMP) / 
            (BETA + (ROOM_TEMP * log10f(conversion_result / T_NOMINAL)));		

	
	*temp = (uint8_t)tKelvin - 273.15;;
	}
	
	
void get_chip_temp(uint8_t *temp)
{
	float ul_temp;
	
	afec_channel_enable(AFEC0, AFEC_TEMPERATURE_SENSOR);
	afec_start_software_conversion( AFEC0 );
	while( !(afec_get_interrupt_status( AFEC0 ) & ( 1 << AFEC_TEMPERATURE_SENSOR)) );

	ul_temp = afec_channel_get_value(AFEC0, AFEC_TEMPERATURE_SENSOR);	
	
	ul_temp  = ul_temp * 3300 / (4095 * 16);// AFEC_conversion_factor;
	/*
	* According to datasheet, The output voltage VT = 1.44V at 27C
	* and the temperature slope dVT/dT = 4.7 mV/C
	*/
	ul_temp =  (ul_temp - 1440)  * 100 / 470 + 27;
	
	//basically shit is fucked
	if( ul_temp > UINT8_MAX ) ul_temp = UINT8_MAX;
	
	*(temp) = (uint8_t)ul_temp;
	afec_channel_disable(AFEC0, AFEC_TEMPERATURE_SENSOR);
}

void get_system_voltages(uint16_t* batt_volt, uint16_t* supply_volt){
	volatile float tmp_batt, tmp_supply;

	afec_channel_enable( PDU_AFEC, BATT_MONITOR );
	afec_channel_enable( PDU_AFEC, SUPPLY_MONITOR );


	afec_start_software_conversion( PDU_AFEC );
	//TODO: make this not hardcoded, this works becuase the Supply monitor channel number is higher
	// and all afec conversions are done in numerical order of enabled channels
	while( !(afec_get_interrupt_status( PDU_AFEC ) & ( (1 << SUPPLY_MONITOR) )) );

	tmp_supply = afec_channel_get_value( PDU_AFEC, SUPPLY_MONITOR );
	tmp_batt   = afec_channel_get_value( PDU_AFEC, BATT_MONITOR );

	tmp_supply 	= ( tmp_supply *  AFEC_conversion_factor_mV ) / AFEC_mV_to_V;
	tmp_batt	= ( tmp_batt * AFEC_conversion_factor_mV ) / AFEC_mV_to_V;

	tmp_supply 	= ( tmp_supply * ( SUPPLY_MONITOR_R1 + SUPPLY_MONITOR_R2) ) / SUPPLY_MONITOR_R2;
	tmp_batt 	= ( tmp_batt * ( BATTERY_MONITOR_R1 + BATTERY_MONITOR_R2) ) / BATTERY_MONITOR_R2;

	afec_channel_disable( PDU_AFEC, BATT_MONITOR );
	afec_channel_disable( PDU_AFEC, SUPPLY_MONITOR );


	*batt_volt		= min((uint16_t)(tmp_batt * VOLTAGE_SCALE_FACTOR ), UINT16_MAX);
	*supply_volt	= min((uint16_t)(tmp_supply * VOLTAGE_SCALE_FACTOR ), UINT16_MAX);
}


void soft_restart(pdu_enable_index channel_num, uint32_t latch_time)
{
	//__disable_irq();
	if(channel_num <= NUM_HC_CHANNEL){
		ioport_set_port_level(PDU_ENABLE_PORT, (1<<channel_num), PDU_OFF_STATE );	
		}

	/*
	 *	Delay to allow latch in powerfet to reset
	 */
	
	vTaskDelay(pdMS_TO_TICKS(latch_time));

	if(channel_num <= NUM_HC_CHANNEL){
		ioport_set_port_level(PDU_ENABLE_PORT, (1<<channel_num), PDU_ON_STATE  );
		}
	
}



/*
* Handlers for transmitting required data to ECU/ Data logger
* Called By Timer/Counter Compare Match interrupt handler
*   The Timer Counter handler this is called from determines 
*	the frequency at which messages will be transmitted 
*
* CAN mailboxes and CAN bus should be first initialized by init
* This function will just load new data into mailbox and start 
* transmission to ECU and data logger
*/
void transmit_status(void* pvParameters){
	
	UNUSED(pvParameters);
	
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = TASK_20_HZ;
	xLastWakeTime = xTaskGetTickCount();

	uint8_t byte_index	= 0;
	uint8_t block_index	= 0;

	for (;;)
		{

		if( block_index < PDU_TX_STATUS_CNT){

			/*
			* PDU message block containing input states and PDU global status
			*/
			can_mailbox[PDU_STATUS_TX_MB].ul_datal = 0;
			can_mailbox[PDU_STATUS_TX_MB].ul_datah = 0;
			//TODO: MAKE THIS LESS AWFUL; Maybe make new struct that char array, and lines up when cast pointer?		
			for( uint8_t index = 0; index < 4; index++){
				if( Tst_bits(enable_mask, ( 1 << (index + byte_index) ) ) ) can_mailbox[PDU_STATUS_TX_MB].ul_datal |= (1<< (8 *index));
				}

			for( uint8_t index = 4; index < 8; index++){
				if( Tst_bits(enable_mask, ( 1 << (index + byte_index) ) ) ) can_mailbox[PDU_STATUS_TX_MB].ul_datah |= (1<< (8 * (index - 4 )));
				}
			can_mailbox[PDU_STATUS_TX_MB].ul_datal |= state_id[block_index];

			can_mailbox[PDU_STATUS_TX_MB].uc_length = MAX_CAN_FRAME_DATA_LEN;
			/* Send out the information in the mailbox. */
			can_mailbox_write(PDU_CAN, &can_mailbox[PDU_STATUS_TX_MB]);
			can_mailbox_send_transfer_cmd(PDU_CAN, &can_mailbox[PDU_STATUS_TX_MB]); 

			byte_index +=8;
			block_index++;
			}
			
		else{
			can_mailbox[PDU_STATUS_TX_MB].ul_datal = 0;
			can_mailbox[PDU_STATUS_TX_MB].ul_datah = 0;
		
			LSB1W(can_mailbox[PDU_STATUS_TX_MB].ul_datal) = PDU.chip_temp;
			LSB2W(can_mailbox[PDU_STATUS_TX_MB].ul_datal) = (uint8_t)( PDU.batt_volt / ( VOLTAGE_SCALE_FACTOR * 0.1216 ) );
			LSB3W(can_mailbox[PDU_STATUS_TX_MB].ul_datal) = PDU.error_flag;
			can_mailbox[PDU_STATUS_TX_MB].ul_datal |= state_three;
			LSB0W(can_mailbox[PDU_STATUS_TX_MB].ul_datah) = PDU.total_curr;
			LSB1W(can_mailbox[PDU_STATUS_TX_MB].ul_datah) = PDU.rail_voltage_3_3 /2;
						
			can_mailbox[PDU_STATUS_TX_MB].uc_length = MAX_CAN_FRAME_DATA_LEN;
			can_mailbox_write(PDU_CAN, &can_mailbox[PDU_STATUS_TX_MB]);
			can_mailbox_send_transfer_cmd(PDU_CAN, &can_mailbox[PDU_STATUS_TX_MB]);

			byte_index = 0;
			block_index = 0;
			}

		// wait
		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS( xFrequency ));
		}
	}
	
	
void transmit_output_current(void* pvParameters){
	
	UNUSED(pvParameters);
	
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = TASK_20_HZ;
	xLastWakeTime = xTaskGetTickCount();

	uint8_t block_index	= 0;
	
	for(;;){
		
		/*
		*	Send out the current measurements to datalogger and ECU
		*	Messages are sent in message with ID of Base Address + 1
		*	7 measurements are sent per message with compound ID in byte 0
		*	all messages send max data length
		*/	
		can_mailbox[PDU_AMP_TX_MB].uc_length = MAX_CAN_FRAME_DATA_LEN;	
		if ( block_index < NUM_PDU_CHANNEL / 7 + 1)
			{
			LSB0W(can_mailbox[PDU_AMP_TX_MB].ul_datal) = block_index;
			LSB1W(can_mailbox[PDU_AMP_TX_MB].ul_datal) = outputs[ (block_index * 7) + 0].current;
			LSB2W(can_mailbox[PDU_AMP_TX_MB].ul_datal) = outputs[ (block_index * 7) + 1].current;
			LSB3W(can_mailbox[PDU_AMP_TX_MB].ul_datal) = outputs[ (block_index * 7) + 2].current;
			LSB0W(can_mailbox[PDU_AMP_TX_MB].ul_datah) = outputs[ (block_index * 7) + 3].current;
			LSB1W(can_mailbox[PDU_AMP_TX_MB].ul_datah) = outputs[ (block_index * 7) + 4].current;
			LSB2W(can_mailbox[PDU_AMP_TX_MB].ul_datah) = outputs[ (block_index * 7) + 5].current;
			LSB3W(can_mailbox[PDU_AMP_TX_MB].ul_datah) = outputs[ (block_index * 7) + 6].current;
			
			can_mailbox_write(PDU_CAN, &can_mailbox[PDU_AMP_TX_MB]);
			can_mailbox_send_transfer_cmd(PDU_CAN, &can_mailbox[PDU_AMP_TX_MB]);
			
			block_index++;
			}
			
		else{
			block_index = 0;
		}
		
		
		// wait
		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS( xFrequency ));
		}
	}
	

/*
* Function to transmit the voltage of output channels
* The output voltages are captured during the AFEC phase
* of channel threads, we only do this 
*
* WARN: this only works on Rev3 of PDU, Rev2 is capable of this 
* but the external ADC's max V sense is 12V
*/	
void transmit_output_voltage(void* pvParameters){
	
	UNUSED(pvParameters);
	
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = TASK_20_HZ;
	xLastWakeTime = xTaskGetTickCount();

	uint8_t block_index	= 0;
	
	for(;;){
		
		/*
		*	Send out the current measurements to datalogger and ECU
		*	Messages are sent in message with ID of Base Address + 1
		*	7 measurements are sent per message with compound ID in byte 0
		*	all messages send max data length
		*/	
		can_mailbox[PDU_VOLT_TX_MB].uc_length = MAX_CAN_FRAME_DATA_LEN;	
		if ( block_index < NUM_PDU_CHANNEL / 7 + 1)
			{
			LSB0W(can_mailbox[PDU_AMP_TX_MB].ul_datal) = block_index;
			LSB1W(can_mailbox[PDU_AMP_TX_MB].ul_datal) = outputs[ (block_index * 7) + 0].voltage;
			LSB2W(can_mailbox[PDU_AMP_TX_MB].ul_datal) = outputs[ (block_index * 7) + 1].voltage;
			LSB3W(can_mailbox[PDU_AMP_TX_MB].ul_datal) = outputs[ (block_index * 7) + 2].voltage;
			LSB0W(can_mailbox[PDU_AMP_TX_MB].ul_datah) = outputs[ (block_index * 7) + 3].voltage;
			LSB1W(can_mailbox[PDU_AMP_TX_MB].ul_datah) = outputs[ (block_index * 7) + 4].voltage;
			LSB2W(can_mailbox[PDU_AMP_TX_MB].ul_datah) = outputs[ (block_index * 7) + 5].voltage;
			LSB3W(can_mailbox[PDU_AMP_TX_MB].ul_datah) = outputs[ (block_index * 7) + 6].voltage;
			
			can_mailbox_write(PDU_CAN, &can_mailbox[PDU_VOLT_TX_MB]);
			can_mailbox_send_transfer_cmd(PDU_CAN, &can_mailbox[PDU_VOLT_TX_MB]);
				
			block_index++;
			}
			
		else{
			block_index = 0;
		}
		
		
		// wait
		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS( xFrequency ));
		}
	}

void transmit_output_state(void* pvParameters){
	
	UNUSED(pvParameters);
	
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = TASK_20_HZ;
	xLastWakeTime = xTaskGetTickCount();

	uint8_t block_index	= 0;
	
	for(;;){
		
		/*
		* Send the PDU message block containing output states
		*/
		can_mailbox[PDU_OUTPUT_TX_MB].uc_length = MAX_CAN_FRAME_DATA_LEN;
		if ( block_index < NUM_PDU_CHANNEL / 7 + 1 )
			{
			LSB0W(can_mailbox[PDU_OUTPUT_TX_MB].ul_datal) = outputs[ (block_index * 8) + 0].state | PDU_status_id[block_index];
			LSB1W(can_mailbox[PDU_OUTPUT_TX_MB].ul_datal) = outputs[ (block_index * 8) + 1].state;
			LSB2W(can_mailbox[PDU_OUTPUT_TX_MB].ul_datal) = outputs[ (block_index * 8) + 2].state;
			LSB3W(can_mailbox[PDU_OUTPUT_TX_MB].ul_datal) = outputs[ (block_index * 8) + 3].state;

			LSB0W(can_mailbox[PDU_OUTPUT_TX_MB].ul_datah) = outputs[ (block_index * 8) + 4].state;
			LSB1W(can_mailbox[PDU_OUTPUT_TX_MB].ul_datah) = outputs[ (block_index * 8) + 5].state;
			LSB2W(can_mailbox[PDU_OUTPUT_TX_MB].ul_datah) = outputs[ (block_index * 8) + 6].state;
			LSB3W(can_mailbox[PDU_OUTPUT_TX_MB].ul_datah) = outputs[ (block_index * 8) + 7].state;
				
			can_mailbox_write(PDU_CAN, &can_mailbox[PDU_OUTPUT_TX_MB]);
			can_mailbox_send_transfer_cmd(PDU_CAN, &can_mailbox[PDU_OUTPUT_TX_MB]);
				
			block_index++;
			}
		else{
			block_index = 0;
		}
		
		
		// wait
		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS( xFrequency ));
		}
	}
	
void transmit_output_temp(void* pvParameters){
	
	UNUSED(pvParameters);
	
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = TASK_20_HZ;
	xLastWakeTime = xTaskGetTickCount();

	uint8_t block_index	= 0;
	
	for(;;){
		
		/*
		*	Send out the current measurements to datalogger and ECU
		*	Messages are sent in message with ID of Base Address + 1
		*	7 measurements are sent per message with compound ID in byte 0
		*	all messages send max data length
		*/	
		can_mailbox[PDU_TEMP_TX_MB].uc_length = MAX_CAN_FRAME_DATA_LEN;	
		if ( block_index < NUM_PDU_CHANNEL / 7 + 1)
			{
			LSB0W(can_mailbox[PDU_TEMP_TX_MB].ul_datal) = block_index;
			LSB1W(can_mailbox[PDU_TEMP_TX_MB].ul_datal) = outputs[ (block_index * 7) + 0].temp;
			LSB2W(can_mailbox[PDU_TEMP_TX_MB].ul_datal) = outputs[ (block_index * 7) + 1].temp;
			LSB3W(can_mailbox[PDU_TEMP_TX_MB].ul_datal) = outputs[ (block_index * 7) + 2].temp;
			LSB0W(can_mailbox[PDU_TEMP_TX_MB].ul_datah) = outputs[ (block_index * 7) + 3].temp;
			LSB1W(can_mailbox[PDU_TEMP_TX_MB].ul_datah) = outputs[ (block_index * 7) + 4].temp;
			LSB2W(can_mailbox[PDU_TEMP_TX_MB].ul_datah) = outputs[ (block_index * 7) + 5].temp;
			LSB3W(can_mailbox[PDU_TEMP_TX_MB].ul_datah) = outputs[ (block_index * 7) + 6].temp;
			
			can_mailbox_write(PDU_CAN, &can_mailbox[PDU_TEMP_TX_MB]);
			can_mailbox_send_transfer_cmd(PDU_CAN, &can_mailbox[PDU_TEMP_TX_MB]);
			
			block_index++;
			}
			
		else{
			block_index = 0;
		}
		
		
		// wait
		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS( xFrequency ));
		}
	}


/*
* Callback function for when PDU hasn't recieved control
* message from ECU for greater than timeout 
*
* Currently we only need this to set error flag and set 
* enable mask to timeout mask
* 
* Even though this function is pretty simple right now, we
* want it as a placeholder incase we need more complicated logic
*/
void ECU_timeout_callback(void){
	
	enable_mask = PDU_TIMEOUT_MASK;
	PDU.error_flag = true;
}

