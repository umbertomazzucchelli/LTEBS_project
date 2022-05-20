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
#include "LIS3DH.h"

uint8 ch_received;
uint16_t FS_received;
uint16_t So_received;

CY_ISR(Custom_ISR_RX)
{
    // Non-blocking call to get the latest data recieved
    ch_received = UART_GetChar();
    FS_received = UART_GetByte();   // byte contains FS values
    So_received = UART_GetByte();   // byte contains So values

        
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
    
    // Set flags and FS and So values on user modifications.
    switch (FS_received)
    {
        case FS2:
            FS = FS2;
            flag_FS = 1;
            break;
        
        case FS4:
            FS = FS2;
            flag_FS = 1;
            break;
        
        case FS8:
            FS = FS2;
            flag_FS = 1;
            break;
        
        case FS16:
            FS = FS2;
            flag_FS = 1;
            break;
        
        default:
        break;
    }
    
    switch (So_received)
    {
        case LP: //10
            So[0] = 0;
            So[1] = 1;
            flag_So = 1;
            break;
        
        case Normal: //00
            So[0] = 0;
            So[1] = 0;
            flag_So = 1;
            
        case HR: //01
            So[0] = 1;
            So[1] = 0;
            flag_So = 1;
            
        case NA:
            break;
            
        default:
        break;
    }
}
/* [] END OF FILE */
