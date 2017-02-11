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

//<editor-fold defaultstate="collapsed" desc="Copyright Details">
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
//</editor-fold>

#define DEBUG_IMG

//<editor-fold defaultstate="collapsed" desc="Include Files">
// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************
#include "app.h"
#include "centroid.h"
#include "ov9712.h"
//</editor-fold>

//<editor-fold defaultstate="collapsed" desc="Global Data Definitions">
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

uint16_t frame_row_count = 0;
uint16_t frame_row_div_count = APP_FRAME_ROW_DIV;
bool wait_for_vsync = true;

//</editor-fold>

//<editor-fold defaultstate="collapsed" desc="System Control Functions">
// *****************************************************************************
// *****************************************************************************
// Section: System Control Functions
// *****************************************************************************
// *****************************************************************************
inline void enablePCLKINT( void )
{
    SYS_INT_SourceEnable(INT_SOURCE_EXTERNAL_4);
}
inline void disablePCLKINT( void )
{
    SYS_INT_SourceDisable(INT_SOURCE_EXTERNAL_4);
}

inline void transferAddDMA( void )
{
    SYS_DMA_ChannelTransferAdd(appData.sysDMAHandle, (uint8_t *)&PMDIN, 1, appData.ramBuff, APP_CAMERA_WIDTH, 1);
}
//</editor-fold>

//<editor-fold defaultstate="collapsed" desc="Application Initialization Functions">
// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization
// *****************************************************************************
// *****************************************************************************
 static void App_Frame_Event_Handler(SYS_DMA_TRANSFER_EVENT event,
        SYS_DMA_CHANNEL_HANDLE handle, uintptr_t contextHandle);
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

/******************************************************************************
  Function:
    void Camera_Initialize( void )

  Description:
    Initialize camera registers using OV9712_regs.h
 */
bool Camera_Initialize( void )
{
    int i = 0, j = 0;
    uint8_t buffer[2];
    for( ; ; i++ )
    {
        if( OV9712_regs[i].id == ENDR ) 
        {
            return true;
        }
        buffer[0] = OV9712_regs[i].id;
        buffer[1] = OV9712_regs[i].value;
        DRV_I2C_BUFFER_HANDLE handle = DRV_I2C_Transmit( appData.drvI2CHandle, OV9712_ADDR, buffer, (sizeof(buffer)), NULL );
        while( DRV_I2C_TransferStatusGet( appData.drvI2CHandle, handle ) != DRV_I2C_BUFFER_EVENT_COMPLETE );
    }
}
//</editor-fold>

//<editor-fold defaultstate="expanded" desc="Application Main Tasks">
// *****************************************************************************
// *****************************************************************************
// Section: Application Main Tasks
// *****************************************************************************
// *****************************************************************************
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
                delay(300);
                appData.drvUSARTHandle = DRV_USART_Open( DRV_USART_INDEX_0, DRV_IO_INTENT_WRITE );

                printChar( '>' );
                printChar( '\r' );
                printChar( '\n' );
                delay(20000);
                
                Camera_Initialize();
                delay(20000);

                /* Allocate a DMA channel */
                appData.sysDMAHandle = SYS_DMA_ChannelAllocate(DMA_CHANNEL_0);
                
                /* Register an event handler for the channel */
                SYS_DMA_ChannelTransferEventHandlerSet(appData.sysDMAHandle, App_Frame_Event_Handler, NULL);

                /* Setup the channel */
                SYS_DMA_ChannelSetup(appData.sysDMAHandle,
                                        (SYS_DMA_CHANNEL_OP_MODE_BASIC | SYS_DMA_CHANNEL_OP_MODE_AUTO),
                                        INT_SOURCE_EXTERNAL_4);

                /* Add the memory block transfer request. */  
                transferAddDMA();
                //APP_FRAME_WIDTH, APP_FRAME_HEIGHT, APP_FRAME_WIDTH_RGGB
                initCentroids( APP_FRAME_WIDTH_RGGB, APP_FRAME_HEIGHT, APP_DEFAULT_INTERVAL, APP_DEFAULT_THRESHOLD );
                appData.state = APP_STATE_SERVICE_TASKS;
                SYS_INT_SourceEnable( INT_SOURCE_EXTERNAL_3 );
            }
            break;
        }

        case APP_STATE_SERVICE_TASKS:
        {
             if ( !DRV_USART_ReceiverBufferIsEmpty( appData.drvUSARTHandle ) )
            {
                uint8_t c = DRV_USART_ReadByte( appData.drvUSARTHandle );
                printChar( c );   
            }
            break;
        }

        /* The default state should never be executed. */
        default:
        {
            break;
        }
    }
}
//</editor-fold>

//<editor-fold defaultstate="collapsed" desc="Application Event Handlers">
// *****************************************************************************
// *****************************************************************************
// Section: Application Event Handlers
// *****************************************************************************
// *****************************************************************************

 static void App_Frame_Event_Handler(SYS_DMA_TRANSFER_EVENT event,
        SYS_DMA_CHANNEL_HANDLE handle, uintptr_t contextHandle)
{
    if(SYS_DMA_TRANSFER_EVENT_COMPLETE == event)
    {
        //SYS_INT_SourceStatusSet( DMA_TRIGGER_SOFTWARE_1 );
    }
}

// *****************************************************************************
// Section: VSYNC Handlers
// *****************************************************************************
void APP_VSYNC_Interrupt_Handler( void )
{
    if(wait_for_vsync)
    {
        //enablePCLKINT();
        SYS_INT_SourceEnable( INT_SOURCE_EXTERNAL_1 ); // Enable HSYNC
        wait_for_vsync = false;
    }
    else
    {
        sendCentroidData();
#ifdef DEBUG_IMG
        printChar( 0xab );  
        printChar( 0x34 );
#endif
        frame_row_div_count = APP_FRAME_ROW_DIV;
    }
    frame_row_count = 0;
    

}

// *****************************************************************************
// Section: HSYNC Handlers
// *****************************************************************************
void APP_HSYNC_Interrupt_Handler( void )
{
    if( frame_row_div_count-- ==  0)
    {
        //disablePCLKINT();
        getCentroids( appData.ramBuff, frame_row_count );
        
#ifdef DEBUG_IMG_
        uint8_t i = 0;
        while( i < APP_FRAME_WIDTH_RGGB ) printChar( appData.ramBuff[i++] );
#endif
        frame_row_div_count = APP_FRAME_ROW_DIV;
        frame_row_count++;
        transferAddDMA();
        //enablePCLKINT();
    }   
}

void APP_PCLK_Interrupt_Handler( void );
void APP_Line_Done_Interrupt_Handler( void )
{
    appData.state = APP_STATE_SERVICE_TASKS;
}

//</editor-fold> 

//<editor-fold defaultstate="collapsed" desc="System Functions">
// *****************************************************************************
// *****************************************************************************
// Section: System Functions
// *****************************************************************************
// *****************************************************************************

void sendCentroidData(void)
{
    //<editor-fold defaultstate="collapsed" desc="Centroids"> 
    printChar( CENTROID_HEAD );
    uint8_t numBlobs = processCentroids();
    uint8_t i;
    printChar( numBlobs );
    uint16_t x, y;
    for( i = 0; i < numBlobs; i++ )
    {
        x = centroids[i].X;
        y = centroids[i].Y;
        printTwoBytes( x );
        printTwoBytes( y );
        printTwoBytes( centroids[i].mass );
    }
    
    resetBlobs();
//</editor-fold>
}

void delay( int count )
{
    int j = 0;
    while(j++ < count) Nop();
}

void printChar( uint8_t c )
{
    if( !DRV_USART_TransmitBufferIsFull( appData.drvUSARTHandle ) )
    {
        DRV_USART_WriteByte( appData.drvUSARTHandle , c );
        delay(150);
    }
}
void printTwoBytes( uint16_t d )
{
    uint8_t top = *( ( uint8_t * )&d + 1 );
    uint8_t btm = *( uint8_t * )&d;
    printChar( top );
    printChar( btm );
}
//</editor-fold>

/*******************************************************************************
 End of File
 */
