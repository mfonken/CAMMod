/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It 
    implements the logic of the application's state machine and it may call 
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
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
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************

#include "app.h"
#include "centroid.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.
    
    Application strings and buffers are be defined outside this structure.
*/

APP_DATA appData;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
*/

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************

static void App_Frame_Event_Handler(SYS_DMA_TRANSFER_EVENT event,
        SYS_DMA_CHANNEL_HANDLE handle, uintptr_t contextHandle);

/* TODO:  Add any necessary local functions.
*/


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    appData.state = APP_STATE_INIT;
}

volatile int frame_row_count = 0;
volatile int frame_row_div_count = APP_FRAME_ROW_DIV;
bool wait_for_vsync = true;

uint8_t img_header[5] = { 0x06, 0x08, 0x07, 0x09, '\t' };
/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Tasks ( void )
{

    /* Check the application's current state. */
    switch ( appData.state )
    {
        /* Application's initial state. */
        case APP_STATE_INIT:
        {
            bool appInitialized = true;
       
            if (appInitialized)
            {
                SYS_INT_VectorPrioritySet(INT_VECTOR_CS0, INT_PRIORITY_LEVEL6);
                SYS_INT_VectorSubprioritySet(INT_VECTOR_CS0, INT_SUBPRIORITY_LEVEL0);
                SYS_INT_SourceEnable(INT_SOURCE_SOFTWARE_0);
                
                SYS_INT_VectorPrioritySet(INT_VECTOR_CS1, INT_PRIORITY_LEVEL7);
                SYS_INT_VectorSubprioritySet(INT_VECTOR_CS1, INT_SUBPRIORITY_LEVEL0);
                SYS_INT_SourceEnable(INT_SOURCE_SOFTWARE_1);
    
                delay(10000);
                appData.drvI2CHandle = DRV_I2C_Open( DRV_I2C_INDEX_0, DRV_IO_INTENT_WRITE );
                uint8_t buffer[] = {0xc2,0xb0};
                DRV_I2C_Transmit( appData.drvI2CHandle, 0x60, buffer, (sizeof(buffer)), NULL );
                
                delay(3000);
                appData.drvUSARTHandle = DRV_USART_Open( DRV_USART_INDEX_0, DRV_IO_INTENT_WRITE );
                //DRV_USART_WriteByte(appData.drvUSARTHandle, DRV_USART_ReadByte(appData.drvUSARTHandle));
                
                DRV_USART_WriteByte(appData.drvUSARTHandle , '\r');
//                DRV_USART_WriteByte(appData.drvUSARTHandle , '\n');
//                DRV_USART_WriteByte(appData.drvUSARTHandle , '^');
                
                  /* Allocate a DMA channel */
                appData.sysDMAHandle = SYS_DMA_ChannelAllocate(DMA_CHANNEL_0);
                /* Register an event handler for the channel */
                SYS_DMA_ChannelTransferEventHandlerSet(appData.sysDMAHandle, App_Frame_Event_Handler, NULL);

                /* Setup the channel */
                SYS_DMA_ChannelSetup(appData.sysDMAHandle,
                                 (SYS_DMA_CHANNEL_OP_MODE_BASIC 
                                    | SYS_DMA_CHANNEL_OP_MODE_AUTO),
                                 INT_SOURCE_EXTERNAL_4);

                /* Add the memory block transfer request. */  
            
                SYS_DMA_ChannelTransferAdd(appData.sysDMAHandle, 
                        (uint8_t *)&PMDIN, 1,                   // Set source address to Parallel Master Port Data IN of size 1 byte
                        appData.ramBuff, APP_CAMERA_WIDTH,   // Set destination address to buffer in SRAM of size APP_DMA_FRAME_BUFFER_SIZE
                        1);                                  // Transfer 1 byte every trigger
                        
                
                SYS_INT_SourceEnable(INT_SOURCE_EXTERNAL_3);
                appData.state = APP_STATE_SERVICE_TASKS;
                
                initCentroids( APP_FRAME_WIDTH, APP_CAMERA_HEIGHT );
            }
            break;
        }

        case APP_STATE_SERVICE_TASKS:
        {
             if (!DRV_USART_ReceiverBufferIsEmpty(appData.drvUSARTHandle))
            {
                uint8_t c = DRV_USART_ReadByte(appData.drvUSARTHandle);
                DRV_USART_WriteByte(appData.drvUSARTHandle, c);   
            }
            
            break;
        }

        case APP_FRAME_DONE:
        {
            appData.state = APP_STATE_SERVICE_TASKS;
            break;
        }
        /* TODO: implement your application state machine.*/
        

        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}

 static void App_Frame_Event_Handler(SYS_DMA_TRANSFER_EVENT event,
        SYS_DMA_CHANNEL_HANDLE handle, uintptr_t contextHandle)
{
    if(SYS_DMA_TRANSFER_EVENT_COMPLETE == event)
    {
        //SYS_INT_SourceStatusSet( DMA_TRIGGER_SOFTWARE_1 );
    }
}
void APP_VSYNC_Interrupt_Handler( void )
{
    if(wait_for_vsync)
    {
        SYS_INT_SourceEnable(INT_SOURCE_EXTERNAL_4);
        SYS_INT_SourceEnable(INT_SOURCE_EXTERNAL_1);
        wait_for_vsync = false;
    }
    else
    {
        frame_row_div_count = APP_FRAME_ROW_DIV;
        SYS_INT_SourceStatusSet( DMA_TRIGGER_SOFTWARE_1 );
    }
    
    DRV_USART_WriteByte( appData.drvUSARTHandle, 0xfa );
    delay(150);
    DRV_USART_WriteByte( appData.drvUSARTHandle, 0xa1 );
    delay(150);
    DRV_USART_WriteByte( appData.drvUSARTHandle, (char)centroids.numBlobs );
    
    centroids.numBlobs = 0;
    frame_row_count = 0;
}
void APP_HSYNC_Interrupt_Handler( void )
{
    frame_row_div_count--;
    
    if( frame_row_div_count ==  0)
    {
        //PMP_RToggle();
        
        SYS_DMA_ChannelTransferAdd(appData.sysDMAHandle, (uint8_t *)&PMDIN, 1, appData.ramBuff, APP_CAMERA_WIDTH, 1);
        
        SYS_INT_SourceDisable(INT_SOURCE_EXTERNAL_3);
        SYS_INT_SourceDisable(INT_SOURCE_EXTERNAL_4);
        
        volatile uint8_t i = 0;
        
        //DRV_USART_WriteByte( appData.drvUSARTHandle, 'b' );
        getCentroids( appData.ramBuff, frame_row_count, 3 );
        //DRV_USART_WriteByte( appData.drvUSARTHandle, 'e' );
        while( i < APP_FRAME_HEIGHT_RGB )
        {
            //DRV_USART_WriteByte( appData.drvUSARTHandle, appData.ramBuff[i] );
            i++;
            delay(150);
        }
        
        frame_row_div_count = APP_FRAME_ROW_DIV;
        frame_row_count++;
        SYS_INT_SourceEnable(INT_SOURCE_EXTERNAL_3);
        SYS_INT_SourceEnable(INT_SOURCE_EXTERNAL_4);
    }   
}
void APP_PCLK_Interrupt_Handler( void )
{
}
void APP_Line_Done_Interrupt_Handler( void )
{
    //PMP_RToggle();
    appData.state = APP_FRAME_DONE;
}

void delay( int count )
{
    int j = 0;
    while(j < count)
    {
        j++;
        Nop();
    }
}
/*******************************************************************************
 End of File
 */
