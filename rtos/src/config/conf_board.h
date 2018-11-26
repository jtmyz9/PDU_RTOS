/**
 * \file
 *
 * \brief Board configuration.
 *
 * Copyright (c) 2014-2018 Microchip Technology Inc. and its subsidiaries.
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

#ifndef CONF_BOARD_H
#define CONF_BOARD_H

#ifndef BOARD
#define BOARD USER_BOARD
#endif


//!------   External Clock Config
#define BOARD_FREQ_SLCK_XTAL      (32768U)
#define BOARD_FREQ_SLCK_BYPASS    (32768U)
#define BOARD_FREQ_MAINCK_XTAL    (20000000U)
#define BOARD_FREQ_MAINCK_BYPASS  (20000000U)
#define BOARD_MCK                 110000000U //CHIP_FREQ_CPU_MAX
#define BOARD_OSC_STARTUP_US      15625
/*
* define CAN1 and CAN0 peripherals,
* leverages conditional compilation from ASF init function
* in sam/boards/sam4e_ek/init
*
* The rebuilt function has preprocessor statements for configuring all 
* different combinations of peripherals on sam4e, maybe someday can 
* clean it up and make it in a more intuitive folder
*/
#define CONF_BOARD_CAN0
//#define CONF_BOARD_CAN1

// Define PWM to leverage some prebuilt ASF compiler definitions and init
#define CONF_PDU_PWM				

//I dont like this here but i need these configurations here for init
// just need to refactor that part
/** PWM0 pin definitions. */
#define PIN_PWM_0_GPIO     PIO_PD20_IDX
#define PIN_PWM_0_FLAGS    (IOPORT_MODE_MUX_A)

/** PWM1 pin definitions. */
#define PIN_PWM_1_GPIO     PIO_PD21_IDX
#define PIN_PWM_1_FLAGS    (IOPORT_MODE_MUX_A)

/** PWM2 pin definitions. */
#define PIN_PWM_2_GPIO     PIO_PD22_IDX
#define PIN_PWM_2_FLAGS    (IOPORT_MODE_MUX_A)

/** PWM3 pin definitions. */
#define PIN_PWM_3_GPIO     PIO_PD23_IDX
#define PIN_PWM_3_FLAGS    (IOPORT_MODE_MUX_A)

/** Defnitions for SPI */
#define CONF_BOARD_SPI
#define CONF_BOARD_SPI_NPCS0

/* Configure UART pins */
#define CONF_BOARD_UART_CONSOLE

#endif /* CONF_BOARD_H */
