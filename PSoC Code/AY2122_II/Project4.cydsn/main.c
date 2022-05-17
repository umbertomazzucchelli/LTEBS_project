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
#include <stdio.h>
#include "project.h"
#include "I2C_Interface.h"
#include "LIS3DH.h"
#include "InterruptRoutines.h"

// Set this to 1 to send byte data for the Bridge Control Panel
// Otherwise set it to 0 to send temperature data as int16_t
#define USE_BRIDGECONTROLPANEL  0
uint8_t ovrnStatus = 0;
uint8_t emptyBit;
uint8_t reg;
ErrorCode error;

uint8_t regSetting;
uint8_t regStatus;

char message[50] = {'\0'};
char deviceStatus;
char oldDeviceStatus=0;
char problem;

float xAcc, yAcc, zAcc;
//Data buffer
uint8_t XData[2]; 
uint8_t YData[2]; 
uint8_t ZData[2]; 
//Output  data
int16 outZ;
int16 outY;
int16 outX;
int count;

int16 regCount = 192;
uint8_t data[192];
uint8_t fifoFull;
int16 xData[32];
int16 yData[32];
int16 zData[32];

uint8_t fifo_status;

int main(void)
{
    CyGlobalIntEnable; /* Enable global interrupts. */
    
    I2C_Peripheral_Start();
    UART_BT_Start();
    UART_Start();
    //Timer_read_Start();
    Timer_LED_Start();
    isr_LED_StartEx(Custom_ISR_LED);
    //isr_read_Start();
    isr_RX_StartEx(Custom_ISR_RX);
    
    deviceStart();
    
    
   

    for(;;)
    {
        /*
        //error = I2C_Peripheral_ReadRegister(LIS3DH_DEVICE_ADDRESS,LIS3DH_FIFO_SRC_REG,&ovrnReady);
        //sprintf(message, "ovrn register value: 0x%02X\r\n", ovrnReady);
        //UART_PutString(message);
        deviceStatus = I2C_Peripheral_IsDeviceConnected(LIS3DH_DEVICE_ADDRESS);
        if(deviceStatus != oldDeviceStatus)
        {
            switch (deviceStatus)
            {
                case 1:
                Timer_LED_Stop();
                problem = 0;
                Pin_LED_Write(1);
                break;
                
                case 0:
                Timer_LED_Start();
                problem = 1;
            }
            
        }
        
        switch (status)
        {
            case SAMPLE:

            error= I2C_Peripheral_ReadRegister(LIS3DH_DEVICE_ADDRESS, LIS3DH_CTRL_REG1, &regSetting);
            if(regSetting!=0x17)
            {
                regSetting= 0x17; //1Hz di prova
                setReg(regSetting,LIS3DH_CTRL_REG1);
            }
            error=I2C_Peripheral_ReadRegister(LIS3DH_DEVICE_ADDRESS, LIS3DH_FIFO_SRC_REG, &fifo_status);
            if(((fifo_status & 0x40)>>6)==1 && ovrnStatus==0)
            {
                ovrnStatus=1;
            }
            
            if(ovrnStatus)
            {
                error=I2C_Peripheral_ReadRegister(LIS3DH_DEVICE_ADDRESS, LIS3DH_FIFO_SRC_REG, &regStatus);
                
                //Leggo i dati e li stampo
                error=I2C_Peripheral_ReadRegister(LIS3DH_DEVICE_ADDRESS, LIS3DH_OUT_X_L, &XData[0]); 
                error=I2C_Peripheral_ReadRegister(LIS3DH_DEVICE_ADDRESS, LIS3DH_OUT_X_H, &XData[1]); 
                //CyDelay(100); 
                error=I2C_Peripheral_ReadRegister(LIS3DH_DEVICE_ADDRESS, LIS3DH_OUT_Y_L, &YData[0]); 
                error=I2C_Peripheral_ReadRegister(LIS3DH_DEVICE_ADDRESS, LIS3DH_OUT_Y_H, &YData[1]);
                //CyDelay(100); 
                error=I2C_Peripheral_ReadRegister(LIS3DH_DEVICE_ADDRESS, LIS3DH_OUT_Z_L, &ZData[0]); 
                error=I2C_Peripheral_ReadRegister(LIS3DH_DEVICE_ADDRESS, LIS3DH_OUT_Z_H, &ZData[1]);
                if (error==NO_ERROR)
                {
                    outX= (int16) (XData[0] | (XData[1] <<8)) >> 6;
                    outY= (int16) (YData[0] | (YData[1] <<8)) >> 6;
                    outZ= (int16) (ZData[0] | (ZData[1] <<8)) >> 6; 
                    sprintf(message, "%d, %d, %d \r\n", outX, outY, outZ);  
                    UART_PutString(message);
                    count++; 
                }
                
                if(regStatus == 128)
                {
                    regSetting=0x00;
                    error = I2C_Peripheral_WriteRegister(LIS3DH_DEVICE_ADDRESS,LIS3DH_FIFO_CTRL_REG,regSetting);
                    regSetting=0x40;
                    error = I2C_Peripheral_WriteRegister(LIS3DH_DEVICE_ADDRESS,LIS3DH_FIFO_CTRL_REG,regSetting);
                    ovrnStatus = 0;
                }
            }
            
            if(count==180)
            {
                status=WAIT;
                count = 0;
            }
            break;
            
            case WAIT:
            ovrnStatus = 0;
            regSetting = 0x00;
            error = I2C_Peripheral_WriteRegister(LIS3DH_DEVICE_ADDRESS,LIS3DH_CTRL_REG1,regSetting); //spegniamo il campionamento 
            count=0;
            regSetting=0x00;
            error = I2C_Peripheral_WriteRegister(LIS3DH_DEVICE_ADDRESS,LIS3DH_FIFO_CTRL_REG,regSetting);
            regSetting=0x40;
            error = I2C_Peripheral_WriteRegister(LIS3DH_DEVICE_ADDRESS,LIS3DH_FIFO_CTRL_REG,regSetting);
            break;
            
        }            
        */
        
        CyDelay(500); //to get stable data, di prova
        error=I2C_Peripheral_ReadRegister(LIS3DH_DEVICE_ADDRESS, LIS3DH_FIFO_SRC_REG, &fifoFull); 
        if(error == NO_ERROR && (fifoFull & 0x40)>>6)
        {
            I2C_Peripheral_ReadRegisterMulti(LIS3DH_DEVICE_ADDRESS,LIS3DH_OUT_X_L,regCount,data);
            for(int i = 0; i<32;i++)
            {
                xData[i] = (int16) (data[i*6] | (data[i*6+1]<<8))>>6;
                yData[i] = (int16) (data[i*6+2] | (data[i*6+3]<<8))>>6;
                zData[i] = (int16) (data[i*6+4] | (data[i*6+5]<<8))>>6;
                sprintf(message, "xData: %d",xData[i]);
                UART_PutString(message);
                sprintf(message, "yData: %d",yData[i]);
                UART_PutString(message);
                sprintf(message, "zData: %d",zData[i]);
                UART_PutString(message);
            }
            regSetting=0x00;
            error = I2C_Peripheral_WriteRegister(LIS3DH_DEVICE_ADDRESS,LIS3DH_FIFO_CTRL_REG,regSetting);
            regSetting=0x40;
            error = I2C_Peripheral_WriteRegister(LIS3DH_DEVICE_ADDRESS,LIS3DH_FIFO_CTRL_REG,regSetting);
        }
    }
        
}

/* [] END OF FILE */
