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
#ifndef __INTERRUPT_ROUTINESBT_H
    #define __INTERRUPT_ROUTINESBT_H
    
    #include "cytypes.h"
    #include "stdio.h"
    
    uint8_t status;
    uint8_t flag_FS;
    uint8_t flag_So;
    
    uint8_t status;
    
    CY_ISR_PROTO (Custom_ISR_RX_BT);
    
    volatile uint8 PacketReadyFlag;
#endif

/* [] END OF FILE */
