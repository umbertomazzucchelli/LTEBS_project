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
#ifndef __INTERRUPT_ROUTINES_H
    
    #define __INTERRUPT_ROUTINES_H
    
    #include "project.h"
    #include "cytypes.h"
    #include "stdio.h"
    
    #define SAMPLE 1 
    #define WAIT 0
    #define HOLD 2
    
    //CY_ISR_PROTO(Custom_ISR_TIMER);
    CY_ISR_PROTO(Custom_ISR_RX);
    //CY_ISR_PROTO(Custom_ISR_read);
    CY_ISR_PROTO(Custom_ISR_LED);
    
   
    char dataReady;
    
    char status;
        
#endif

/* [] END OF FILE */
