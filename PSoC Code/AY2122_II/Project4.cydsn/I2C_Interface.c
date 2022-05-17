/*
* This file includes all the required source code to interface
* the I2C peripheral.
*
* This file needs to be completed.
*/

/**
*   \brief Value returned if device present on I2C bus.
*/
#ifndef DEVICE_CONNECTED
    #define DEVICE_CONNECTED 1
#endif

/**
*   \brief Value returned if device not present on I2C bus.
*/
#ifndef DEVICE_UNCONNECTED
    #define DEVICE_UNCONNECTED 0
#endif

#include "I2C_Interface.h" 
#include "I2C_Master.h"
#include "project.h"
#include "ErrorCodes.h"
#include "LIS3DH.h"
#include "stdio.h"




ErrorCode I2C_Peripheral_Start(void) 
{
    // Start I2C peripheral
    I2C_Master_Start();  
    
    // Return no error since start function does not return any error
    return NO_ERROR;
}

// --------------------------------------------------------------------------------- //

ErrorCode I2C_Peripheral_Stop(void)
{
    // Stop I2C peripheral
    I2C_Master_Stop();
    // Return no error since stop function does not return any error
    return NO_ERROR;
}

// --------------------------------------------------------------------------------- //

ErrorCode I2C_Peripheral_ReadRegister(uint8_t device_address, 
                                      uint8_t register_address,
                                      uint8_t* data) {
    
    // Start Condition
    uint8_t error = I2C_Master_MasterSendStart(device_address, I2C_Master_WRITE_XFER_MODE);                                        
    if (error == I2C_Master_MSTR_NO_ERROR)
    {
        //Write the register address to be read
        error = I2C_Master_MasterWriteByte(register_address);
        if (error == I2C_Master_MSTR_NO_ERROR)
        {
            // Send a restart condition
            error = I2C_Master_MasterSendRestart(device_address, I2C_Master_READ_XFER_MODE);
            if (error == I2C_Master_MSTR_NO_ERROR)
            {
                *data = I2C_Master_MasterReadByte(I2C_Master_NAK_DATA);
                //I send a not aknowledgment so that the program doesn't expect any other data
            }
        }
    }
    // Stop Comunication
    I2C_Master_MasterSendStop();
    //Return
    return error ? ERROR : NO_ERROR;
    
}
                                    
// --------------------------------------------------------------------------------- //

ErrorCode I2C_Peripheral_ReadRegisterMulti(uint8_t device_address,
                                           uint8_t register_address,
                                           uint8_t register_count,
                                           uint8_t* data) {
    
    // Send start condition
    uint8_t error = I2C_Master_MasterSendStart(device_address, I2C_Master_WRITE_XFER_MODE);
    if (error == I2C_Master_MSTR_NO_ERROR)
    {
        // Write register address to be read with the MSB = 1
        register_address |= 0x80; //Datasheet indication for multi read -- autoincrement
        error = I2C_Master_MasterWriteByte(register_address);
        if (error == I2C_Master_MSTR_NO_ERROR)
        {
            // Send Restart condition
            error = I2C_Master_MasterSendRestart(device_address, I2C_Master_READ_XFER_MODE);
            if (error == I2C_Master_MSTR_NO_ERROR)
            {
                // Continue to read until we have register to be read
                uint8_t counter = register_count;
                while (counter > 1)
                {
                    data[register_count - counter] = 
                        I2C_Master_MasterReadByte(I2C_Master_ACK_DATA);
                        counter --;
                }
                // Read last data without acknolowledgment
                data[register_count - 1] = 
                    I2C_Master_MasterReadByte(I2C_Master_NAK_DATA);
            }
        }
    }
    // Send Stop Condition
    I2C_Master_MasterSendStop();
    // Return Error Code
    return error ? ERROR : NO_ERROR;
                                              
}
                                        
// --------------------------------------------------------------------------------- //

ErrorCode I2C_Peripheral_WriteRegister(uint8_t device_address,
                                       uint8_t register_address,
                                       uint8_t data) {
    
    
    // Send Start Condition
    uint8_t error = I2C_Master_MasterSendStart(device_address, I2C_Master_WRITE_XFER_MODE);
    if (error == I2C_Master_MSTR_NO_ERROR)
    {
        //Write Register Address (to be overwritten)
        error = I2C_Master_MasterWriteByte(register_address);
        if (error == I2C_Master_MSTR_NO_ERROR)
        {
            // Write Byte
            error = I2C_Master_MasterWriteByte(data);
        }
    }
    // Close Communication
    I2C_Master_MasterSendStop();
    // Return
    return error ? ERROR : NO_ERROR;
}
                                    

                                            
// --------------------------------------------------------------------------------- //

uint8_t I2C_Peripheral_IsDeviceConnected(uint8_t device_address) {
    
    // Send a start condition followed by a stop condition
    uint8_t error = I2C_Master_MasterSendStart(device_address, I2C_Master_WRITE_XFER_MODE);
    I2C_Master_MasterSendStop();
    // If no error generated during stop, device is connected
    return (error == I2C_Master_MSTR_NO_ERROR ? DEVICE_CONNECTED : DEVICE_UNCONNECTED);
    
}

// --------------------------------------------------------------------------------- //

void setReg(uint8_t setReg,uint8_t setAddress){
    UART_BT_PutString("\r\nWriting new values...\r\n");
    UART_PutString("\r\nWriting new values...\r\n");
        
    ErrorCode errorSet = I2C_Peripheral_WriteRegister(LIS3DH_DEVICE_ADDRESS,
                                         setAddress,
                                         setReg);
    
    if (errorSet == NO_ERROR)
    {
        sprintf(message, "\r\nRegister successfully written as: 0x%02X\r\n", setReg);
        UART_BT_PutString(message);
        UART_PutString(message);
    }
    else
    {
        UART_BT_PutString("\r\nError occured during I2C comm to set the register \r\n");
        UART_PutString("\r\nError occured during I2C comm to set the register\r\n");
    }
}

// --------------------------------------------------------------------------------- //

void readReg(uint8_t readReg,uint8_t readAddress){

    ErrorCode errorRead = I2C_Peripheral_ReadRegister(LIS3DH_DEVICE_ADDRESS,
                                        readAddress,
                                         &readReg);
    
    if( errorRead == NO_ERROR ) {
        sprintf(message, "Register value: 0x%02X\r\n", readReg);
        UART_BT_PutString(message);
        UART_PutString(message);
    }
    else {
        UART_BT_PutString("I2C error while reading the register\r\n");
        UART_PutString("I2C error while reading the register\r\n");
    }
}

// --------------------------------------------------------------------------------- //

void deviceStart()
{
    
    CyDelay(5); //"The boot procedure is complete about 5 ms after device power-up."
    // Check if LIS3DH is connected
    uint32_t rval = I2C_Master_MasterSendStart(LIS3DH_DEVICE_ADDRESS, I2C_Master_WRITE_XFER_MODE);
    if( rval == I2C_Master_MSTR_NO_ERROR ) {
        UART_BT_PutString("LIS3DH found @ address 0x18\r\n");
        UART_PutString("LIS3DH found @ address 0x18\r\n");
    }
    I2C_Master_MasterSendStop();
    
    UART_PutString("**************\r\n");
    UART_PutString("** I2C Scan **\r\n");
    UART_PutString("**************\r\n");
    UART_BT_PutString("**************\r\n");
    UART_BT_PutString("** I2C Scan **\r\n");
    UART_BT_PutString("**************\r\n");
    
    CyDelay(10);
    
    // Setup the screen and print the header
	UART_BT_PutString("\n\n   ");
    UART_PutString("\n\n   ");
	for(uint8_t i = 0; i<0x10; i++)
	{
        sprintf(message, "%02X ", i);
		UART_BT_PutString(message);
        UART_PutString(message);
	}
    
    // SCAN the I2C BUS for slaves
	for(uint8_t i2caddress = 0; i2caddress < 0x80; i2caddress++ ) {
        
		if(i2caddress % 0x10 == 0 ) {
            sprintf(message, "\n%02X ", i2caddress);
		    UART_PutString(message);
            UART_BT_PutString(message);
        }
 
		rval = I2C_Master_MasterSendStart(i2caddress, I2C_Master_WRITE_XFER_MODE);
        
        if( rval == I2C_Master_MSTR_NO_ERROR ) // If you get ACK then print the address
		{
            sprintf(message, "%02X ", i2caddress);
		    UART_PutString(message);
            UART_BT_PutString(message);
		}
		else //  Otherwise print a --
		{
		    UART_BT_PutString("-- ");
            UART_PutString("-- ");
		}
        I2C_Master_MasterSendStop();
	}
    
	UART_PutString("\n\n");
    UART_BT_PutString("\n\n");
    
    /******************************************/
    /*            I2C Reading                 */
    /******************************************/
          
    /*      I2C Master Read - WHOAMI Register       */
    uint8_t whoami_reg;
    ErrorCode error = I2C_Peripheral_ReadRegister(LIS3DH_DEVICE_ADDRESS, 
                                                  LIS3DH_WHO_AM_I_REG_ADDR,
                                                  &whoami_reg);
    if( error == NO_ERROR ) {
        sprintf(message, "WHOAMI register value: 0x%02X [Expected value: 0x%02X]\r\n", whoami_reg, LIS3DH_WHOAMI_RETVAL);
        UART_PutString(message);
    }
    else {
        UART_PutString("I2C error while reading LIS3DH_WHO_AM_I_REG_ADDR\r\n");
    }
    
    
    /*      I2C Master Read - STATUS Register       */
    
    uint8_t status_reg;
    error = I2C_Peripheral_ReadRegister(LIS3DH_DEVICE_ADDRESS, 
                                        LIS3DH_STATUS_REG,
                                        &status_reg);
    if( error == NO_ERROR ) {
        sprintf(message, "STATUS register value: 0x%02X\r\n", status_reg);
        UART_PutString(message);
    }
    else {
        UART_PutString("I2C error while reading LIS3DH_STATUS_REG\r\n");
    }
    
    /*      I2C Master Read - CTRL Register 1       */
    
    uint8_t control_reg_1;
    control_reg_1= 0x27; 
    error = I2C_Peripheral_WriteRegister(LIS3DH_DEVICE_ADDRESS,
                                         LIS3DH_CTRL_REG1,
                                         control_reg_1);
    
    error = I2C_Peripheral_ReadRegister(LIS3DH_DEVICE_ADDRESS,
                                        LIS3DH_CTRL_REG1,
                                        &control_reg_1);

    if( error == NO_ERROR ) {
        sprintf(message, "CTRL register 1 value: 0x%02X\r\n", control_reg_1);
        UART_PutString(message);
    }
    else {
        UART_PutString("I2C error while reading LIS3DH_CTRL_REG1\r\n");}

    
    /* CTRL REG 4 Writing from LIS3DH   */ 
    uint8_t control_reg_4;
    control_reg_4 = 0x00;
    error = I2C_Peripheral_WriteRegister(LIS3DH_DEVICE_ADDRESS,
                                         LIS3DH_CTRL_REG4,
                                         control_reg_4);
    error = I2C_Peripheral_ReadRegister(LIS3DH_DEVICE_ADDRESS,
                                        LIS3DH_CTRL_REG4,
                                        &control_reg_4);    
   
    if( error == NO_ERROR ) {
        sprintf(message, "CTRL register 4 value: 0x%02X\r\n", control_reg_4);
        UART_PutString(message);
    }
    
    else {
        UART_PutString("I2C error while reading LIS3DH_CTRL_REG4\r\n");}    
    
    //CONTROL REGISTER 5
    uint8_t control_reg_5;
    control_reg_5 = 0x40;
    error = I2C_Peripheral_WriteRegister(LIS3DH_DEVICE_ADDRESS,
                                         LIS3DH_CTRL_REG5,
                                         control_reg_5);
    error = I2C_Peripheral_ReadRegister(LIS3DH_DEVICE_ADDRESS,
                                        LIS3DH_CTRL_REG5,
                                        &control_reg_5);    
   
    if( error == NO_ERROR ) {
        sprintf(message, "CTRL register 5 value: 0x%02X\r\n", control_reg_5);
        UART_PutString(message);
    }
    else {
        UART_PutString("I2C error while reading LIS3DH_CTRL_REG5\r\n");}  
    
    //FIFO
    uint8_t fifo_control_reg;
    fifo_control_reg &= 0x40;
    error = I2C_Peripheral_WriteRegister(LIS3DH_DEVICE_ADDRESS,
                                         LIS3DH_FIFO_CTRL_REG,
                                         fifo_control_reg);
    error = I2C_Peripheral_ReadRegister(LIS3DH_DEVICE_ADDRESS,
                                        LIS3DH_FIFO_CTRL_REG,
                                        &fifo_control_reg);    
   
    if( error == NO_ERROR ) {
        sprintf(message, "CTRL register FIFO value: 0x%02X\r\n", fifo_control_reg);
        UART_PutString(message);
    }
    else {
        UART_PutString("I2C error while reading LIS3DH_CTRL_REG5\r\n");
    }  

    UART_PutString("\r\nConfiguration complete\r\n");
}
/* [] END OF FILE */
