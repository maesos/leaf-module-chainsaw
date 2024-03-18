/*
 * i2c_bb.h
 *
 *  Created on: Dec. 22, 2021
 *      Author: Someone
 */

#ifndef I2C_BB_H_
#define I2C_BB_H_


#include <stdbool.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <driverlib.h>
#include "leaf_utils.h"

/********************************************************************************
Module      : I2C_SW
Author      : 05/04/2015, by KienLTb - https://kienltb.wordpress.com/
Description : I2C software using bit-banging.

********************************************************************************/
// #ifndef _I2C_SW_H_
// #define _I2C_SW_H_
/*-----------------------------------------------------------------------------*/
/* Macro definitions  */
/*-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------*/
/* Local Macro definitions */
/*-----------------------------------------------------------------------------*/

// REFER TO TABLE 6-17 IN MSP430FR2433 DATASHEET FOR PIN CONTROL BIT STATES

// Function select
// Input when function select is set to 1
// Set to output when FS is set to zero
// SEL0 and SEL1 are for defining the two bits of the selection register.
// GPIO are configured as 0,0 while X,Y are for other pin functions.
// https://www.argenox.com/library/msp430/general-purpose-input-output-gpio-chapter-5/
#define I2C_PxSEL       P1SEL0
#define I2C_PxSEL2      P1SEL1

// Direction setting of a I/O pin set 0 for input, 1 for output
#define I2C_PxDIR       P1DIR
#define I2C_PxOUT       P1OUT
#define I2C_PxIN        P1IN

// Define bit position based on
#define SCL             BIT6
#define SDA             BIT7

#define ACK             0x00
#define NACK            0x01

// SET TIME_DELAY BASED OFF CLOCK RATE
// Adjustments made based off measured frequency
#define I2C_FREQ 100000
//#define TIME_DELAY MCLK_FREQ_HZ / I2C_FREQ / 2 / 4
//#define I2C_DELAY() __delay_cycles(TIME_DELAY)
//#define I2C_DELAY() __delay_cycles(20)       // Set to fixed value for 16 MHZ
#define I2C_DELAY() __delay_cycles(1)       // Set to fixed value for 2 MHZ
/*-----------------------------------------------------------------------------*/
/* Function prototypes  */
/*-----------------------------------------------------------------------------*/

//NOTE: Need custom Read_SCL(), Read_SDA(), Clear_SCL(), Clear_SDA() to compatible Hardware.

uint8_t Read_SCL(void); // Set SCL as input and return current level of line, 0 or 1, nomal is 1 because pullup by res
uint8_t Read_SDA(void); // Set SDA as input and return current level of line, 0 or 1, nomal is 0 because pull by res

void Clear_SCL(void); // Actively drive SCL signal Low
void Clear_SDA(void); // Actively drive SDA signal Low

void i2c_bb_Reset(void);

void i2c_bb_Init(void);
void i2c_bb_Start(void);
void i2c_bb_Stop(void);

void i2c_bb_Writebit(uint8_t bit);
uint8_t i2c_bb_Readbit(void);

int i2c_bb_WriteByte(uint8_t Data);
uint8_t i2c_bb_ReadByte(void);

int i2c_bb_WriteData(uint8_t DeviceAddr, uint8_t *Data, uint8_t nLength);
int i2c_bb_ReadData(uint8_t DeviceAddr, uint8_t *Buff, uint8_t nLength);
int i2c_bb_WriteData_reg(uint8_t DeviceAddr, uint8_t *Data, uint8_t Register, uint8_t nLength);
int i2c_bb_ReadData_reg(uint8_t DeviceAddr, uint8_t *Buff, uint8_t Register, uint8_t nLength);

// #endif // _I2C_SW_H_


#endif /* I2C_BB_H_ */
