/*******************************************************************************
  MPLAB Harmony Application Header File

  Company:
    Microchip Technology Inc.

  File Name:
    app.h

  Summary:
    This header file provides prototypes and definitions for the application.

  Description:
    This header file provides function prototypes and data type definitions for
    the application.  Some of these are required by the system (such as the
    "APP_Initialize" and "APP_Tasks" prototypes) and some of them are only used
    internally by the application (such as the "APP_STATES" definition).  Both
    are defined here for convenience.
*******************************************************************************/

//<editor-fold defaultstate="collapsed" desc="Copyright Details">
//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
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
//DOM-IGNORE-END
//</editor-fold>

#ifndef _APP_H
#define _APP_H

//<editor-fold defaultstate="collapsed" desc="Included Files">
// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include "system_config.h"
#include "system_definitions.h"
//</editor-fold>

//<editor-fold defaultstate="collapsed" desc="C++ Compatibility">
// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

extern "C" {

#endif
// DOM-IGNORE-END 
//</editor-fold>
    
//<editor-fold defaultstate="collapsed" desc="Variable Definitions">
// *****************************************************************************
// *****************************************************************************
// Section: Variable Definitions
// *****************************************************************************
// *****************************************************************************
// *****************************************************************************
#define APP_CAMERA_WIDTH            1280 * 3
#define APP_CAMERA_HEIGHT           800
#define APP_FRAME_WIDTH             30
#define APP_FRAME_HEIGHT            32
#define APP_FRAME_HEIGHT_RGGB       APP_FRAME_HEIGHT * 2
#define APP_FRAME_LINE_DIV          ( APP_CAMERA_WIDTH / APP_FRAME_WIDTH )
#define APP_FRAME_ROW_DIV           ( APP_CAMERA_HEIGHT / APP_FRAME_HEIGHT ) 
//</editor-fold>
    
//<editor-fold defaultstate="collapsed" desc="Type Definitions">
// *****************************************************************************
// *****************************************************************************
// Section: Type Definitions
// *****************************************************************************
// *****************************************************************************
typedef enum
{
	/* Application's state machine's initial state. */
	APP_STATE_INIT=0,
            
	APP_STATE_SERVICE_TASKS,
} APP_STATES;

typedef struct
{
    /* The application's current state */
    APP_STATES                          state;
    DRV_HANDLE                          drvI2CHandle;
    DRV_HANDLE                          drvUSARTHandle;
    SYS_DMA_CHANNEL_HANDLE              sysDMAHandle;   
    uint8_t                             ramBuff[APP_CAMERA_WIDTH];

} APP_DATA;


//</editor-fold>

//<editor-fold defaultstate="collapsed" desc="Application Function Prototypes">
// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************
void APP_Initialize(                  void );
void Camera_Initalize(                void );
void APP_Tasks(                       void );
void APP_Line_Done_Interrupt_Handler( void );
void APP_VSYNC_Interrupt_Handler(     void );
void APP_HSYNC_Interrupt_Handler(     void );
void APP_PCLK_Interrupt_Handler(      void );
void delay(                            int );
void printChar(                    uint8_t );
//</editor-fold>

#endif /* _APP_H */

//<editor-fold defaultstate="collapsed" desc="C++ Compatibility">
//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END
//</editor-fold>

/*******************************************************************************
 End of File
 */

