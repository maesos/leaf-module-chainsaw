/*
 * i2c_bb.c
 *
 *  Created on: Dec. 21, 2021
 *      Author: Someone
 */
/********************************************************************************
Module      : I2C_SW
Author      : 05/04/2015, by KienLTb - https://kienltb.wordpress.com/
Description : I2C software using bit-banging.


********************************************************************************/

/*-----------------------------------------------------------------------------*/
/* Header inclusions */
/*-----------------------------------------------------------------------------*/
//#include "msp430g2553.h"
#include "i2c_bb.h"

/*-----------------------------------------------------------------------------*/
/* Local Macro definitions */
/*-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------*/
/* Function prototypes */
/*-----------------------------------------------------------------------------*/
//uint8_t Read_SCL(void);
//uint8_t Read_SDA(void);
//void Clear_SCL(void);
//void Clear_SDA(void);
//
//void i2c_bb_Init(void);
//void i2c_bb_Start(void);
//void i2c_bb_Stop(void);
//
//int i2c_bb_Writebit(uint8_t bit);
//int i2c_bb_ReadByte(void);
//
//int i2c_bb_WriteByte(uint8_t Data);
//int i2c_bb_ReadByte(void);
//
//
//int i2c_bb_WriteData(uint8_t DeviceAddr, uint8_t *Data, uint8_t nLength);
//int i2c_bb_ReadData(uint8_t DeviceAddr, uint8_t *Buff, uint8_t nLength);
//int I2C_WriteData(uint8_t DeviceAddr, uint8_t *Data, uint8_t Register, uint8_t nLength);
//int I2C_ReadData(uint8_t DeviceAddr, uint8_t *Buff, uint8_t Register, uint8_t nLength);

/*-----------------------------------------------------------------------------*/
/* Function implementations */
/*-----------------------------------------------------------------------------*/


/*--------------------------------------------------------------------------------
Function    : Read_SCL
Purpose     : Set SCL as input and return current Logic level of SCL (0 or 1)
              nomal is 1 because pullup by resistor
Parameters  : None
Return      : Logic level of SCL pin
--------------------------------------------------------------------------------*/
uint8_t Read_SCL(void)
{
    I2C_PxOUT  |= SCL;  // Set to output high
    I2C_PxDIR  &= ~SCL; // Bitmask to allow every pin except SCL
    return((I2C_PxIN & SCL) != 0);
}
/*--------------------------------------------------------------------------------
Function    : Read_SDA
Purpose     : Set SDA as input and return current Logic level of SDA (0 or 1),
              normal is 1 because pullup by resistor
Parameters  : None
Return      : Logic level of SDA pin
--------------------------------------------------------------------------------*/
uint8_t Read_SDA(void)
{
    I2C_PxOUT  |= SDA;  // Set to output high
    // Direction as 1 is output 0 is input. Clears SDA bit to zero.
    I2C_PxDIR  &= ~SDA;
    return((I2C_PxIN & SDA) != 0);
}

/*--------------------------------------------------------------------------------
Function    : Clear_SCL
Purpose     : Set SCL as output low, and output direction
Parameters  : None
Return      : None
--------------------------------------------------------------------------------*/
void Clear_SCL(void)
{
    I2C_PxOUT  &= ~SCL;
    I2C_PxDIR  |= SCL;
}
/*--------------------------------------------------------------------------------
Function    : Clear_SDA
Purpose     : Set SDA as Out put, logic LOW
Parameters  : None
Return      : None
--------------------------------------------------------------------------------*/
void Clear_SDA(void)
{
    I2C_PxOUT  &= ~SDA; // Set output state of SDA to off
    I2C_PxDIR  |= SDA;  // Set direction for SDA to 1
}
/*--------------------------------------------------------------------------------
Function    : I2C_Init
Purpose     : Initialize I2C block
Parameters  : None
Return      : None
--------------------------------------------------------------------------------*/
void i2c_bb_Init(void)
{
    // Config SCL and SDA as GPIO
    I2C_PxSEL   &= ~(SCL | SDA);
    I2C_PxSEL2  &= ~(SCL | SDA);
    // Set SCL and SDA is logic HIGH and input mode
    I2C_PxOUT   &= ~(SCL | SDA);
    I2C_PxDIR   &= ~(SCL | SDA);
}
/*--------------------------------------------------------------------------------
Function    : I2C_Reset
Purpose     : Reset the I2C Bus
Parameters  : None
Return      : None
--------------------------------------------------------------------------------*/
void i2c_bb_Reset(void)
{
    // Toggle the Clock line a bunch of times until reset
    int i;
    Read_SDA();             //set SDA to 1
    I2C_DELAY();

    for (i = 9; i > 0; i--)
    {
        Clear_SCL();            //set SCL to 0
        I2C_DELAY();
        Read_SCL();            //set SCL to 0
        I2C_DELAY();
    }
}
/*--------------------------------------------------------------------------------
Function    : I2C_Start
Purpose     : Send start signal
Parameters  : None
Return      : None
--------------------------------------------------------------------------------*/
void i2c_bb_Start(void)
{
    I2C_DELAY();
    Read_SDA();             //set SDA to 1
    Read_SCL();             //set SDA to 1
    I2C_DELAY();
    Clear_SDA();            //set SDA to 0, currently SCL is 1
    I2C_DELAY();
    Clear_SCL();            //set SCL to 0
}
/*--------------------------------------------------------------------------------
Function    : I2C_Stop
Purpose     : Send Stop signal
Parameters  : None
Return      : None
--------------------------------------------------------------------------------*/
void i2c_bb_Stop(void)
{
    Clear_SDA();            //set SDA to 0
    I2C_DELAY();
    Read_SCL();             //set SCL to 1
    I2C_DELAY();
    Read_SDA();             //set SDA to 1 (don't actually care about the values but setting to input pin configuration
    I2C_DELAY();
}
/*--------------------------------------------------------------------------------
Function    : I2C_Writebit
Purpose     : Write bit to I2C bus
Parameters  : a bit need to write
Return      : None
--------------------------------------------------------------------------------*/
void i2c_bb_Writebit(uint8_t bit)
{
    if(bit)
      Read_SDA();
    else
      Clear_SDA();
    I2C_DELAY();
    Read_SCL();
    I2C_DELAY();
    Clear_SCL();
}
/*--------------------------------------------------------------------------------
Function    : I2C_Readbit
Purpose     : Read bit to I2C bus
Parameters  : None
Return      : uint8_t
--------------------------------------------------------------------------------*/
uint8_t i2c_bb_Readbit(void)
{
    uint8_t bit;

    Read_SDA();     // Set SDA to input mode if not already
    I2C_DELAY();
    Read_SCL();     // Set SCL to HIGH
    bit = Read_SDA();   // Check SDA value (Data should be set before clock high)
    I2C_DELAY();
    Clear_SCL();    // Set SCL to LOW
    return bit;
}
/*--------------------------------------------------------------------------------
Function    : I2C_WriteByte
Purpose     : Write a Byte to I2C bus
Parameters  : uint8_t Data
Return      : None
--------------------------------------------------------------------------------*/
int i2c_bb_WriteByte(uint8_t Data)
{
    int rc = 0;
    uint8_t nBit;

    __disable_interrupt();
    for(nBit = 8; nBit > 0; nBit--)
    {
        i2c_bb_Writebit((Data & 0x80) != 0);
        Data <<= 1;
    }
    rc = i2c_bb_Readbit(); // Wait ACK
    __enable_interrupt();
    return rc;
}
/*--------------------------------------------------------------------------------
Function    : I2C_ReadByte
Purpose     : Read a Byte to I2C bus
Parameters  : None
Return      : uint8_t
--------------------------------------------------------------------------------*/
uint8_t i2c_bb_ReadByte(void)
{
    uint8_t Buff = 0;
    uint8_t nBit;

    __disable_interrupt();
    for(nBit = 8; nBit > 0; nBit--)
    {
        Buff = (Buff << 1) | i2c_bb_Readbit();
    }
    __enable_interrupt();
    return Buff;
}
/*--------------------------------------------------------------------------------
Function    : I2C_WriteData
Purpose     : Write n Byte to I2C bus
Parameters  : Data          - Pointer to Data need to write
              DeviceAddr    - Devide Address
              Register      - Register Address
              nLength       - Number of Byte need to write
Return      : None
--------------------------------------------------------------------------------*/
int i2c_bb_WriteData(uint8_t DeviceAddr, uint8_t *Data, uint8_t nLength)
{
    int rc = 0;
    uint8_t nIndex;
    i2c_bb_Start();
    rc |= i2c_bb_WriteByte(DeviceAddr << 1);
    for(nIndex = 0; (rc == 0) && nIndex < nLength; nIndex++)
    {
        rc |= i2c_bb_WriteByte(*(Data + nIndex));
    }
//    i2c_bb_Readbit();
    i2c_bb_Stop();
    return rc;
}
/*--------------------------------------------------------------------------------
Function    : I2C_ReadData
Purpose     : Read n Byte from I2C bus
Parameters  : Buff          - Pointer to Buffer store value
              DeviceAddr    - Devide Address
              Register      - Register Address
              nLength       - Number of Byte need to read
Return      : None
--------------------------------------------------------------------------------*/
int i2c_bb_ReadData(uint8_t DeviceAddr, uint8_t *Buff, uint8_t nLength)
{
    int rc = 0;
    uint8_t nIndex;                              // Short delay
    i2c_bb_Start();
    _NOP();                                 // Short delay
    rc |= i2c_bb_WriteByte((DeviceAddr << 1) | 1);
    for(nIndex = 0; (rc == 0) && nIndex < nLength; nIndex++)
    {
        *(Buff + nIndex) = i2c_bb_ReadByte();
//        _NOP();                                 // Short delay
        if(nIndex < nLength - 1)i2c_bb_Writebit(ACK);
    }
    i2c_bb_Writebit(NACK);
    i2c_bb_Stop();
    return rc;
}
/*--------------------------------------------------------------------------------
Function    : i2c_bb_WriteData
Purpose     : Write n Byte to I2C bus
Parameters  : Data          - Pointer to Data need to write
              DeviceAddr    - Devide Address
              Register      - Register Address
              nLength       - Number of Byte need to write
Return      : None
--------------------------------------------------------------------------------*/
int i2c_bb_WriteData_reg(uint8_t DeviceAddr, uint8_t *Data, uint8_t Register, uint8_t nLength)
{
    int rc = 0;
    uint8_t nIndex;
    i2c_bb_Start();
    rc |= i2c_bb_WriteByte(DeviceAddr << 1);  // byDeviceAddr is 7 bit and command is write
    if (rc == 0)
    {
        I2C_DELAY();
        rc |= i2c_bb_WriteByte(Register);
        for(nIndex = 0; (rc == 0) && nIndex < nLength; nIndex++)
        {
            I2C_DELAY();
            rc |= i2c_bb_WriteByte(*(Data + nIndex));
        }
        i2c_bb_Stop();
    }

    return rc;
}
/*--------------------------------------------------------------------------------
Function    : i2c_bb_ReadData
Purpose     : Read n Byte from I2C bus
Parameters  : Buff          - Pointer to Buffer store value
              DeviceAddr    - Devide Address
              Register      - Register Address
              nLength       - Number of Byte need to read
Return      : None
--------------------------------------------------------------------------------*/
int i2c_bb_ReadData_reg(uint8_t DeviceAddr, uint8_t *Buff, uint8_t Register, uint8_t nLength)
{
    int rc = 0;
    uint8_t nIndex;
    i2c_bb_Start();
    rc |= i2c_bb_WriteByte(DeviceAddr << 1);
    rc |= i2c_bb_WriteByte(Register);
    i2c_bb_Stop();

    if (rc == 0)
    {
        delay_usec(25);

        i2c_bb_Start();
        rc |= i2c_bb_WriteByte((DeviceAddr << 1) | 1);
        _NOP();                                 // Short delay
        for(nIndex = 0; (rc == 0) && nIndex < nLength; nIndex++)
        {
            *(Buff + nIndex) = i2c_bb_ReadByte();

            // Ensures NACK to transmit for last iteration
            if(nIndex < nLength - 1)i2c_bb_Writebit(ACK);
    //        delay_usec(20);     // 20 us is four I2C 100khz clock cycles
        }
        i2c_bb_Writebit(NACK);
        i2c_bb_Stop();
    }

    return rc;
}
