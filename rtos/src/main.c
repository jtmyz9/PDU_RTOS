/**
 * \file
 *
 * \brief FreeRTOS configuration
 *
 * Copyright (c) 2012-2018 Microchip Technology Inc. and its subsidiaries.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Subject to your compliance with these terms, you may use Microchip
 * software and any derivatives exclusively with Microchip products.
 * It is your responsibility to comply with third party license terms applicable
 * to your use of third party software (including open source software) that
 * may accompany Microchip software.
 *
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES,
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE,
 * INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY,
 * AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT WILL MICROCHIP BE
 * LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, INCIDENTAL OR CONSEQUENTIAL
 * LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO THE
 * SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE
 * POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE FULLEST EXTENT
 * ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY
 * RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
 * THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * \asf_license_stop
 *
 */
/*
 * Support and FAQ: visit <a href="https://www.microchip.com/support/">Microchip Support</a>
 */


#include <asf.h>
#include "PDU.h"
#include "conf_board.h"
#include "config_block.h"

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,
		signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);

/**
 * \brief Called if stack overflow during execution
 */
extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,
		signed char *pcTaskName)
{
	printf("stack overflow %x %s\r\n", (unsigned int)pxTask, (portCHAR *)pcTaskName);
	/* If the parameters have been corrupted then inspect pxCurrentTCB to
	 * identify which task has overflowed its stack.
	 */
	for (;;) {
	}
}

/**
 * \brief This function is called by FreeRTOS idle task
 */
extern void vApplicationIdleHook(void)
{
	
}

/**
 * \brief This function is called by FreeRTOS each tick
 */
extern void vApplicationTickHook(void)
{
	//ioport_toggle_pin_level(STATUS_LED1_GPIO);
}

extern void vApplicationMallocFailedHook(void)
{
	/* Called if a call to pvPortMalloc() fails because there is insufficient
	free memory available in the FreeRTOS heap.  pvPortMalloc() is called
	internally by FreeRTOS API functions that create tasks, queues, software
	timers, and semaphores.  The size of the FreeRTOS heap is set by the
	configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */

	/* Force an assert. */
	configASSERT( ( volatile void * ) NULL );
}

/**
 * \brief This task, when activated, send every ten seconds on debug UART
 * the whole report of free heap and total tasks status
 */
static void task_monitor(void *pvParameters)
{
	static portCHAR szList[256];
	UNUSED(pvParameters);

	for (;;) {
		//printf("--- task ## %u", (unsigned int)uxTaskGetNumberOfTasks());
		//vTaskList((unsigned portCHAR *)szList);
		//printf(szList);

		//printf("Enable Mask: %u \n", &enable_mask);
		vTaskDelay(1000);
	}
}


/**
 * \brief Configure the console UART.
 */
static void configure_console(void)
{
	const usart_serial_options_t uart_serial_options = {
		.baudrate = CONF_UART_BAUDRATE,
#if (defined CONF_UART_CHAR_LENGTH)
		.charlength = CONF_UART_CHAR_LENGTH,
#endif
		.paritytype = CONF_UART_PARITY,
#if (defined CONF_UART_STOP_BITS)
		.stopbits = CONF_UART_STOP_BITS,
#endif
	};

	/* Configure console UART. */
	stdio_serial_init(CONF_UART, &uart_serial_options);

	/* Specify that stdout should not be buffered. */
#if defined(__GNUC__)
	setbuf(stdout, NULL);
#else
	/* Already the case in IAR's Normal DLIB default configuration: printf()
	 * emits one character at a time.
	 */
#endif
}

/*
* Global Variable Declarations
*/
uint32_t ul_sysclk = 0;

volatile ioport_port_mask_t	enable_mask = NULL_MASK;
volatile ioport_port_mask_t	soft_restart_mask = NULL_MASK;
volatile ioport_port_mask_t	error_mask = NULL_MASK;


PDU_state_t			PDU = {
	0,0,0,0,0
};

volatile uint8_t		pwm_request_arr[NUM_HC_CHANNEL] = {
	PDU_DEFAULT_DUTY_VALUE, PDU_DEFAULT_DUTY_VALUE,PDU_DEFAULT_DUTY_VALUE,PDU_DEFAULT_DUTY_VALUE};

// Initially the maximum duty cycle is our saturation limit
volatile uint8_t max_duty_cycle = PWM_DUTY_CYCLE_SATURATION_LIMIT;
vehicle_data_t			vehicle = {0,0,0,0};

output_data_t			outputs[MAX_PDU_CHANNEL];
channel_config_t		config[MAX_PDU_CHANNEL];

volatile bool					update_outputs		= false;
/**Time in ms, since last received message from ECU*/
volatile uint32_t g_recv_timeout_cnt = 0;

pwm_channel_t hc_pwm_channel[NUM_HC_CHANNEL];

/*
* CAN Transfer mailbox structure 
* Array of structs to be used for all the mailboxes in CAN controller
*/
// can_mb_conf_t can_mailbox[CANMB_NUMBER];
volatile can_receive_status		g_ul_recv_status    = no_receive;
volatile can_transmit_status	g_tx_status			= tx_request;


SemaphoreHandle_t afec_semaphore;
SemaphoreHandle_t afec2_semaphore;
SemaphoreHandle_t afec_complete_buffer;


/**
 *  Main application for Mizzou Racing PDU Revision 2 w/ RTOS
 *  This will call necessary functions to setup RTOS and PDU
 *  And then will spawn off threads for PDU
 *
 *  \return Unused (ANSI-C compatibility).
 */
int main(void)
{
	/* Initialize the SAM system */
	sysclk_init();
	ul_sysclk = sysclk_get_cpu_hz();
	board_init();
	
	configure_console();
	printf("Debug mode");puts("\r");
	init_PDU();
	printf("Initialized");puts("\r");
	
	PDU_system_startup();

	/*
	* Call to start system
	*/			
	vTaskStartScheduler();

	/* Will only get here if there was insufficient memory to create the idle task. */
	return 0;
}
