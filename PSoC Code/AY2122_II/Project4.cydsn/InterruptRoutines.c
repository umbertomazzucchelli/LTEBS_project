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
#include "InterruptRoutines.h" 
#include "project.h"
#include "I2C_Interface.h"
#include "LIS3DH.h"


uint8_t ch_received;
extern ErrorCode error;
extern char problem;
char ledStatus;

CY_ISR(Custom_ISR_RX)
{
    ch_received = UART_GetChar(); //receives one byte, if we have a string we have to do it recursively
    
    switch(ch_received)
    {
        //starting case (the two notations in columns mens that the code works for both)
        case 'a':
        case 'A':
            status=SAMPLE;
            //Timer_Start();
            //flag_stop = 0;
            break;
        
        case 's':
        case 'S':
            status=HOLD;
        break;
        //default case
        default:
            break;
    }
}
/*
CY_ISR(Custom_ISR_read)
{
    Timer_read_ReadStatusRegister();
    if(status==SAMPLE && dataReady==0)
    {
        error = I2C_Peripheral_ReadRegister(LIS3DH_DEVICE_ADDRESS,LIS3DH_OUT_X_L,&dataLSB);
        error = I2C_Peripheral_ReadRegister(LIS3DH_DEVICE_ADDRESS,LIS3DH_OUT_X_H,&dataMSB);
        if (error == NO_ERROR) xData = (dataLSB | (dataMSB<<8)) >> 6;
        error = I2C_Peripheral_ReadRegister(LIS3DH_DEVICE_ADDRESS,LIS3DH_OUT_Y_L,&dataLSB);
        error = I2C_Peripheral_ReadRegister(LIS3DH_DEVICE_ADDRESS,LIS3DH_OUT_Y_H,&dataMSB);
        if (error == NO_ERROR) yData = (dataLSB | (dataMSB<<8)) >> 6;
        error = I2C_Peripheral_ReadRegister(LIS3DH_DEVICE_ADDRESS,LIS3DH_OUT_Z_L,&dataLSB);
        error = I2C_Peripheral_ReadRegister(LIS3DH_DEVICE_ADDRESS,LIS3DH_OUT_Z_H,&dataMSB);
        if (error == NO_ERROR) zData = (dataLSB | (dataMSB<<8)) >> 6;
        dataReady=1 ;
    }    
}
*/
CY_ISR(Custom_ISR_LED)
{
    Timer_LED_ReadStatusRegister();
    if(problem==1)
    {
        ledStatus = Pin_LED_Read();
        ledStatus = !ledStatus;
        Pin_LED_Write(ledStatus);
    }
}



/* [] END OF FILE */
