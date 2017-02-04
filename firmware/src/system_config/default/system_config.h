/*******************************************************************************
  MPLAB Harmony System Configuration Header

  File Name:
    system_config.h

  Summary:
    Build-time configuration header for the system defined by this MPLAB Harmony
    project.

  Description:
    An MPLAB Project may have multiple configurations.  This file defines the
    build-time options for a single configuration.

  Remarks:
    This configuration header must not define any prototypes or data
    definitions (or include any files that do).  It only provides macro
    definitions for build-time configuration options that are not instantiated
    until used by another MPLAB Harmony module or application.

    Created with MPLAB Harmony Version 2.01
*******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2015 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED AS IS WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
*******************************************************************************/
// DOM-IGNORE-END

#ifndef _SYSTEM_CONFIG_H
#define _SYSTEM_CONFIG_H

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************
/*  This section Includes other configuration headers necessary to completely
    define this configuration.
*/


// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

extern "C" {

#endif
// DOM-IGNORE-END

// *****************************************************************************
// *****************************************************************************
// Section: System Service Configuration
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Clock System Service Configuration Options
*/
#define SYS_CLK_FREQ                        48000000ul
#define SYS_CLK_BUS_PERIPHERAL_1            48000000ul
#define SYS_CLK_BUS_REFERENCE_1             24000000ul
#define SYS_CLK_UPLL_BEFORE_DIV2_FREQ       192000000ul
#define SYS_CLK_CONFIG_PRIMARY_XTAL         8000000ul
#define SYS_CLK_CONFIG_SECONDARY_XTAL       32768ul
   // *****************************************************************************
/* Common System Service Configuration Options
*/
#define SYS_VERSION_STR           "2.01"
#define SYS_VERSION               20100

/*** Interrupt System Service Configuration ***/
#define SYS_INT                     true

/*** Ports System Service Configuration ***/
#define SYS_PORT_A_ANSEL        0xFFE0
#define SYS_PORT_A_TRIS         0xFFE6
#define SYS_PORT_A_LAT          0x0018
#define SYS_PORT_A_ODC          0x0000
#define SYS_PORT_A_CNPU         0x0010
#define SYS_PORT_A_CNPD         0x0000
#define SYS_PORT_A_CNEN         0x0000

#define SYS_PORT_B_ANSEL        0x0001
#define SYS_PORT_B_TRIS         0xBFFF
#define SYS_PORT_B_LAT          0x0000
#define SYS_PORT_B_ODC          0x0000
#define SYS_PORT_B_CNPU         0x0000
#define SYS_PORT_B_CNPD         0x0000
#define SYS_PORT_B_CNEN         0x0000


// *****************************************************************************
// *****************************************************************************
// Section: Driver Configuration
// *****************************************************************************
// *****************************************************************************
// *****************************************************************************
/* I2C Driver Configuration Options
*/
#define DRV_I2C_INTERRUPT_MODE                    		true
#define DRV_I2C_CLIENTS_NUMBER                    		1
#define DRV_I2C_INSTANCES_NUMBER                  		1

#define DRV_I2C_PERIPHERAL_ID_IDX0                		I2C_ID_2
#define DRV_I2C_OPERATION_MODE_IDX0               		DRV_I2C_MODE_MASTER
#define DRV_SCL_PORT_IDX0                               PORT_CHANNEL_A
#define DRV_SCL_PIN_POSITION_IDX0                       PORTS_BIT_POS_2
#define DRV_SDA_PORT_IDX0                               PORT_CHANNEL_A
#define DRV_SDA_PIN_POSITION_IDX0                       PORTS_BIT_POS_3
#define DRV_I2C_BIT_BANG_IDX0                           false
#define DRV_I2C_STOP_IN_IDLE_IDX0                       false
#define DRV_I2C_SMBus_SPECIFICATION_IDX0			    false
#define DRV_I2C_BAUD_RATE_IDX0                    		100000
#define DRV_I2C_BRG_CLOCK_IDX0	                  		48000000
#define DRV_I2C_SLEW_RATE_CONTROL_IDX0      			false
#define DRV_I2C_MASTER_INT_SRC_IDX0               		INT_SOURCE_I2C_2_MASTER
#define DRV_I2C_SLAVE_INT_SRC_IDX0                		
#define DRV_I2C_ERR_MX_INT_SRC_IDX0               		INT_SOURCE_I2C_2_ERROR
#define DRV_I2C_INT_VECTOR_IDX0                         INT_VECTOR_I2C2
#define DRV_I2C_ISR_VECTOR_IDX0                         _I2C_2_VECTOR
#define DRV_I2C_INT_PRIORITY_IDX0                 		INT_PRIORITY_LEVEL6
#define DRV_I2C_INT_SUB_PRIORITY_IDX0             		INT_SUBPRIORITY_LEVEL1
#define DRV_I2C_POWER_STATE_IDX0                  		SYS_MODULE_POWER_RUN_FULL
#define DRV_I2C_INTERRUPT_MODE                    		true


#define USE_8BIT_PMP

// *****************************************************************************
/* USART Driver Configuration Options
*/
#define DRV_USART_INTERRUPT_MODE                    true

#define DRV_USART_BYTE_MODEL_SUPPORT                true

#define DRV_USART_READ_WRITE_MODEL_SUPPORT          false

#define DRV_USART_BUFFER_QUEUE_SUPPORT              false

#define DRV_USART_CLIENTS_NUMBER                    1
#define DRV_USART_INSTANCES_NUMBER                  1

#define DRV_USART_PERIPHERAL_ID_IDX0                USART_ID_1
#define DRV_USART_OPER_MODE_IDX0                    DRV_USART_OPERATION_MODE_NORMAL
#define DRV_USART_OPER_MODE_DATA_IDX0               
#define DRV_USART_INIT_FLAG_WAKE_ON_START_IDX0      false
#define DRV_USART_INIT_FLAG_AUTO_BAUD_IDX0          false
#define DRV_USART_INIT_FLAG_STOP_IN_IDLE_IDX0       false
#define DRV_USART_INIT_FLAGS_IDX0                   0
#define DRV_USART_BRG_CLOCK_IDX0                    48000000
#define DRV_USART_BAUD_RATE_IDX0                    921600
#define DRV_USART_LINE_CNTRL_IDX0                   DRV_USART_LINE_CONTROL_8NONE1
#define DRV_USART_HANDSHAKE_MODE_IDX0               DRV_USART_HANDSHAKE_NONE
#define DRV_USART_XMIT_INT_SRC_IDX0                 INT_SOURCE_USART_1_TRANSMIT
#define DRV_USART_RCV_INT_SRC_IDX0                  INT_SOURCE_USART_1_RECEIVE
#define DRV_USART_ERR_INT_SRC_IDX0                  INT_SOURCE_USART_1_ERROR
#define DRV_USART_INT_VECTOR_IDX0                   INT_VECTOR_UART1
#define DRV_USART_INT_PRIORITY_IDX0                 INT_PRIORITY_LEVEL1
#define DRV_USART_INT_SUB_PRIORITY_IDX0             INT_SUBPRIORITY_LEVEL0


#define DRV_USART_POWER_STATE_IDX0                  SYS_MODULE_POWER_RUN_FULL


// *****************************************************************************
// *****************************************************************************
// Section: Middleware & Other Library Configuration
// *****************************************************************************
// *****************************************************************************



// *****************************************************************************
// *****************************************************************************
// Section: Application Configuration
// *****************************************************************************
// *****************************************************************************

/*** Application Defined Pins ***/

/*** Functions for CAM_EN pin ***/
#define CAM_ENToggle() PLIB_PORTS_PinToggle(PORTS_ID_0, PORT_CHANNEL_A, PORTS_BIT_POS_4)
#define CAM_ENOn() PLIB_PORTS_PinSet(PORTS_ID_0, PORT_CHANNEL_A, PORTS_BIT_POS_4)
#define CAM_ENOff() PLIB_PORTS_PinClear(PORTS_ID_0, PORT_CHANNEL_A, PORTS_BIT_POS_4)
#define CAM_ENStateGet() PLIB_PORTS_PinGetLatched(PORTS_ID_0, PORT_CHANNEL_A, PORTS_BIT_POS_4)
#define CAM_ENStateSet(Value) PLIB_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_A, PORTS_BIT_POS_4, Value)

/*** Functions for PWDN pin ***/
#define PWDNToggle() PLIB_PORTS_PinToggle(PORTS_ID_0, PORT_CHANNEL_B, PORTS_BIT_POS_14)
#define PWDNOn() PLIB_PORTS_PinSet(PORTS_ID_0, PORT_CHANNEL_B, PORTS_BIT_POS_14)
#define PWDNOff() PLIB_PORTS_PinClear(PORTS_ID_0, PORT_CHANNEL_B, PORTS_BIT_POS_14)
#define PWDNStateGet() PLIB_PORTS_PinGetLatched(PORTS_ID_0, PORT_CHANNEL_B, PORTS_BIT_POS_14)
#define PWDNStateSet(Value) PLIB_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_B, PORTS_BIT_POS_14, Value)

/*** Functions for PMP_R pin ***/
#define PMP_RToggle() PLIB_PORTS_PinToggle(PORTS_ID_0, PORT_CHANNEL_A, PORTS_BIT_POS_0)
#define PMP_ROn() PLIB_PORTS_PinSet(PORTS_ID_0, PORT_CHANNEL_A, PORTS_BIT_POS_0)
#define PMP_ROff() PLIB_PORTS_PinClear(PORTS_ID_0, PORT_CHANNEL_A, PORTS_BIT_POS_0)
#define PMP_RStateGet() PLIB_PORTS_PinGetLatched(PORTS_ID_0, PORT_CHANNEL_A, PORTS_BIT_POS_0)
#define PMP_RStateSet(Value) PLIB_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_A, PORTS_BIT_POS_0, Value)



/*** Application Instance 0 Configuration ***/

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END


#endif // _SYSTEM_CONFIG_H
/*******************************************************************************
 End of File
*/

