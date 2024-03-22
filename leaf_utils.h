/*
 * leaf_utils.h
 *
 *  Created on: Dec. 24, 2021
 *      Author: Someone
 */

#ifndef LEAF_UTILS_H_
#define LEAF_UTILS_H_


#include <string.h>
#include <stdio.h>
#include <stdint.h>

#include <driverlib.h>

#include "leaf_flow.h"


// Pin Definitions 



// UART Debug Port
#define NODE_DBG_PORT GPIO_PORT_P1
#define NODE_DBG_TXD_PIN GPIO_PIN4
#define NODE_DBG_RXD_PIN GPIO_PIN5

// Chainsaw Module Specifics
#define PWR_OUT_ENn_PORT GPIO_PORT_P2
#define PWR_OUT_ENn_PIN GPIO_PIN1

#define ADC_CH0_PORT GPIO_PORT_P1
#define ADC_CH0_PIN GPIO_PIN6

#define ADC_CH1_PORT GPIO_PORT_P1
#define ADC_CH1_PIN GPIO_PIN7

// RS485 Module Specifics
#define PWR_OUT_ENn_PORT GPIO_PORT_P2
#define PWR_OUT_ENn_PIN GPIO_PIN1

#define RS485_REn_PORT GPIO_PORT_P2
#define RS485_REn_PIN GPIO_PIN3
#define RS485_DE_PORT GPIO_PORT_P2
#define RS485_DE_PIN GPIO_PIN4

#define RS485_TX_PORT GPIO_PORT_P2
#define RS485_TX_PIN GPIO_PIN5
#define RS485_RXd_PORT GPIO_PORT_P2
#define RS485_RXd_PIN GPIO_PIN6

#define POKE_OUT_PORT GPIO_PORT_P2
#define POKE_OUT_PIN GPIO_PIN7

// LEAF FLOW

#define DEBUG_PRINT 0x00

#define CLOCK_NO_INIT 0

#define MCLK_FREQ_MHZ 1                     // MCLK = 8MHz
extern uint16_t mclk_freq_khz;
extern uint16_t smclk_freq_khz;
extern uint16_t aclk_freq_hz;

int clock_cfg();
int clock_cfg_slow();
void Software_Trim();                       // Software Trim to get the best DCOFTRIM value

#ifdef DEBUG_PRINT
int debug_print_init();
int debug_print(char * buf, uint8_t len);
#endif


// UART UTILITY FUNCTION

#define UART_A0_HW_BASE_ADDR __MSP430_BASEADDRESS_EUSCI_A0__     // eUSCI_A0 registers Table 6-42 in MSP430FR2433 datasheet
#define UART_A1_HW_BASE_ADDR __MSP430_BASEADDRESS_EUSCI_A1__     // eUSCI_A1 registers Table 6-42 in MSP430FR2433 datasheet

typedef struct lf_uart_s {
    // Base of UART peripheral
    uint8_t * base_address_ucaX;

    uint8_t flag_active;

    // Buffer settings
    uint8_t * buffer_pointer;
    uint8_t buffer_length;
    uint8_t buffer_count;  // If transmitting using buffer pointer, increments send_count until length reached (len-1)
    uint8_t buffer_full;
    uint8_t buffer_sent;
//    uint8_t buffer_done;

    // Desired baudrate
    uint32_t baudrate;

    // Physical pin configuration
    //    __MSP430_BASEADDRESS_PORT2_R__
    //    __MSP430_BASEADDRESS_PORT1_R__
    uint8_t * base_address_txrx_portX;
    uint8_t pin_num_tx;
    uint8_t pin_num_rx;

    uint8_t flag_compatibility_rs485;

    uint8_t * base_address_REn_port;
    uint8_t pin_num_REn;
    uint8_t * base_address_DE_port;
    uint8_t pin_num_DE;

    // Unsure
    unsigned int clk;

} lf_uart_t;

lf_uart_t uart_a0;
lf_uart_t uart_a1;

int uart_init(lf_uart_t * uart);
int uart_tx(lf_uart_t * uart, uint8_t * buf, uint8_t len);
int uart_rx(lf_uart_t * uart, uint8_t * buf, uint8_t * len);

// I2C PARAMETERS

extern EUSCI_B_I2C_initSlaveParam i2c_hw_params_slave;
extern EUSCI_B_I2C_initMasterParam i2c_hw_params_master;

extern uint8_t i2c_recv_buff[64];
extern volatile int i2c_recv_flag;
extern volatile int i2c_recv_busy;
extern volatile int i2c_recv_len;
extern volatile int i2c_tx_busy;

void i2c_hw_init();
void i2c_hw_init_intrpt();
int i2c_hw_init_slave();
int i2c_hw_WriteData(uint8_t DeviceAddr, uint8_t *Data, uint8_t nLength);
int i2c_hw_ReadData(uint8_t DeviceAddr, uint8_t *Buff, uint8_t nLength);
int i2c_hw_WriteData_reg(uint8_t DeviceAddr, uint8_t *Data, uint8_t Register, uint8_t nLength);
int i2c_hw_ReadData_reg(uint8_t DeviceAddr, uint8_t *Buff, uint8_t Register, uint8_t nLength);

// GENERAL UTILITIES

uint16_t mean(uint16_t *list, uint16_t len);
uint16_t mean_log(uint16_t *list, uint16_t len_log);
int16_t mean_log_s16(int16_t *list, uint16_t len_log);
int32_t mean_log_s32(int32_t *list, uint16_t len_log);
int16_t mean_s16(int16_t *list, uint16_t len_log);
int32_t mean_s32(int32_t *list, uint16_t len_log);

void delay_usec(uint16_t time);
void delay_msec(uint16_t time);
void delay_sec(uint16_t time);

extern volatile uint32_t tick_msec;
//extern volatile uint32_t tick_s;
extern volatile uint16_t stop_mode_done;

void timer_a0_cfg();
void stop_mode(uint32_t stop_time);
int wait_for_intrp(uint32_t stop_time);



#endif /* LEAF_UTILS_H_ */
