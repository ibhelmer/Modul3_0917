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

    
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
}


/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Tasks ( void )
{
    char res=10;
    /* Check the application's current state. */
    switch ( appData.state )
    {
        /* Application's initial state. */
        case APP_STATE_INIT:
        {
            bool appInitialized = true;
            LED1Off();
            LED2Off();
            LED3Off();
          
            //Write Data to M24C64 !!
            //Start I2C Bus in master mode
//            if (DRV_I2C0_MasterStart()==false)
//            {
//                LED1On(); //There is an error
//            }
//            do {} while (DRV_I2C0_WaitForStartComplete()==false);
//            
//            //device code,Write mode
//            DRV_I2C0_ByteWrite(0xA0);
//            do {} while (DRV_I2C0_WaitForByteWriteToComplete()==false);
//            
//            // send address 0x111
//            DRV_I2C0_ByteWrite(0x01);
//            do {} while (DRV_I2C0_WaitForByteWriteToComplete()==false);           
//            DRV_I2C0_ByteWrite(0x11);
//            do {} while (DRV_I2C0_WaitForByteWriteToComplete()==false);
//            DRV_I2C0_ByteWrite(0x15);
//            do {} while (DRV_I2C0_WaitForByteWriteToComplete()==false);      
//             DRV_I2C0_MasterStop();           
//            do {} while (DRV_I2C0_WaitForStopComplete()==false);
//            LED3On();
//            
            //Read Data from M24C64
            //Start I2C Bus in master mode
            if (DRV_I2C0_MasterStart()==false)
            {
                LED1On(); //There is an error
            }
            do {} while (DRV_I2C0_WaitForStartComplete()==false);
            
            //device code,Write mode
            DRV_I2C0_ByteWrite(0xA0);
            do {} while (DRV_I2C0_WaitForByteWriteToComplete()==false);
            
            // send address 0x111
            DRV_I2C0_ByteWrite(0x01);
            do {} while (DRV_I2C0_WaitForByteWriteToComplete()==false);           
            DRV_I2C0_ByteWrite(0x11);
            do {} while (DRV_I2C0_WaitForByteWriteToComplete()==false);
            //Restart
            while (!DRV_I2C0_MasterBusIdle()); 
            if (DRV_I2C0_MasterRestart()==false)
            {
                LED1On(); //There is an error
            }
            do {} while (DRV_I2C0_WaitForStartComplete()==false);
            DRV_I2C0_ByteWrite(0xA1);
            do {} while (DRV_I2C0_WaitForByteWriteToComplete()==false);
            do {} while(DRV_I2C0_SetUpByteRead()==false); 
           
            LED2On();
            do {} while (DRV_I2C0_WaitForReadByteAvailable() == false);
            //read data in address 0x0111          
            
            res=DRV_I2C0_ByteRead();            
            
            DRV_I2C0_MasterStop();           
            do {} while (DRV_I2C0_WaitForStopComplete()==false);
            LED3On();
            if (appInitialized)
            {
            
                appData.state = APP_STATE_SERVICE_TASKS;
            }
            break;
        }

        case APP_STATE_SERVICE_TASKS:
        {
        
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

 

/*******************************************************************************
 End of File
 */
