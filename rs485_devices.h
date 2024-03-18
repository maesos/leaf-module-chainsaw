/*
 * rs485_devices.h
 *
 *  Created on: 2022-07-11
 *      Author: Someone
 */

#ifndef RS485_DEVICES_H_
#define RS485_DEVICES_H_

#include "leaf_utils.h"


#define RS485_VARIANT_MEC10 1   // China three prong black boxed
#define RS485_VARIANT_MEC10_PAYLOAD_SIZE 12
#define RS485_VARIANT_MS20 2    // China three prong white boxed
#define RS485_VARIANT_MS20_PAYLOAD_SIZE 6
#define RS485_VARIANT_LWS10 3   //China leaf wetness
#define RS485_VARIANT_LWS10_SIZE 4

 #define RS485_VARIANT_TEROS12 100
 #define RS485_VARIANT_TEROS12_PAYLOAD_SIZE 0
// #define RS485_VARIANT_MAS1 105
// #define RS485_VARIANT_MAS1_PAYLOAD_SIZE 0


#define RS485_CMD_READ_HOLD 0x03
#define RS485_CMD_READ_INP 0x04
#define RS485_CMD_WRITE_SINGLE 0x06
#define RS485_CMD_WRITE_MULTI 0x10






int rs485_read_register(lf_uart_t *uart_dev, uint8_t dev_addr, uint16_t reg_addr, uint8_t * buf, uint8_t len);


uint16_t modbus_crc(uint8_t * buf, uint8_t len);










#endif /* LEAF_UTILS_H_ */
