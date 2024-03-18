/*
 * rs485_devices.c
 *
 *  Created on: 2022-07-11
 *      Author: Someone
 */

#include "rs485_devices.h"

int rs485_read_register(lf_uart_t *uart_dev, uint8_t dev_addr, uint16_t reg_addr, uint8_t * buf, uint8_t len)
{

    uint8_t rc = 0;

    uint8_t request_data[12] = {0};


    request_data[0] = dev_addr;    // Address
    request_data[1] = RS485_CMD_READ_HOLD;    // Command

    request_data[2] = reg_addr >> 8;    // Register location
    request_data[3] = reg_addr & 0xFF;    // Register location

    request_data[4] = 0x00;    // Read Count
    request_data[5] = len;    // Read Count

    uint16_t crc = modbus_crc(request_data, 6);
    request_data[6] = crc & 0xFF;   // Checksum
    request_data[7] = (crc>>8) & 0xFF;   // Checksum

    // Len will result in a minimum of 7 bytes and incrementing by multiple of 2
    uint8_t len_recv_expect = 5+len*2;
    uint8_t len_recv_actual = 5+len*2;

//    rc |= uart_init(&uart_a1);
    rc |= uart_tx(uart_dev, request_data, 8);
    memset(request_data, 0, sizeof(request_data));
    rc |= uart_rx(uart_dev, request_data, &len_recv_actual)<<1;
//    *(uart_dev->base_address_ucaX + OFS_UCAxCTL1) |= UCSWRST;

    if (len_recv_expect != len_recv_actual)
    {
        __no_operation();
    }
    else
    {

        uint16_t crc_recv = modbus_crc(request_data, len_recv_actual - 2);
        if (request_data[len_recv_actual-2] != (crc_recv & 0xFF) || request_data[len_recv_actual-1] != ((crc_recv >> 8) & 0xFF))
        {
            rc |= 1<<3;
        }
    }

    memcpy(buf, &request_data[3], len<<1);
//    memcpy(buf + 1, request_data, len_recv_actual);
//    memcpy(buf, &len_recv_actual, 1);

//    sprintf(uart_send, "\n\rlen_recv_actual %u\n\r    ", len_recv_actual);
//    debug_print(len_recv_actual, 26);

    // rc 0b1<<0 for tx failure
    // rc 0b1<<1 for rx none received
    // rc 0b1<<2 for rx buffer incomplete
    // rc 0b1<<3 for rx crc error
//    rc |= (len_recv_expect != len_recv_actual) << 3;

    return rc;
}


uint16_t modbus_crc(uint8_t * buf, uint8_t len)
{
    uint16_t crc = 0xFFFF;

    int pos;
    int i;
    for (pos = 0; pos < len; pos++) // Loop for bytes
    {
        crc ^= (uint16_t)buf[pos];          // XOR byte into least sig. byte of crc

        for (i = 8; i != 0; i--) // Loop over each bit
        {
            if ((crc & 0x0001) != 0)
            {      // If the LSB is set
                crc >>= 1;                    // Shift right and XOR 0xA001
                crc ^= 0xA001;
            }
            else    // Else LSB is not set
            {
                crc >>= 1;                    // Just shift right
            }
        }
    }
    // Note, this number has low and high bytes swapped, so use it accordingly (or swap bytes)
    return crc;
}
