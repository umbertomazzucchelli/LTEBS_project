/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/
// Include header
#include "InterruptRoutines.h"

// Include required header files
#include "project.h"
uint8 ch_received;

CY_ISR(Custom_ISR_RX)
{
    // Non-blocking call to get the latest data recieved
    ch_received = UART_GetChar();
        
    // Set flags based on UART command
    switch(ch_received)
    {
        case 'A':
        case 'a':
            status=1;
            break;
        case 'T':
        case 't':
            UART_PutString("HR/RR sensor");
            break;
        default:
            break;    
    }
}
/* [] END OF FILE */
