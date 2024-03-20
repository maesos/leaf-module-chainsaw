/*
 * leaf_utils.c
 *
 *  Created on: Dec. 24, 2021
 *      Author: Someone
 */

#include "leaf_utils.h"

uint16_t mclk_freq_khz = 0;
uint16_t smclk_freq_khz = 0;
uint16_t aclk_freq_hz = 0;


int clock_cfg()
{
    // Configure ONE FRAM waitstate as required by the device datasheet for MCLK
    // operation beyond 8MHz _before_ configuring the clock system.
    FRCTL0 = FRCTLPW | NWAITS_1;

    __bis_SR_register(SCG0);                           // disable FLL
    CSCTL3 |= SELREF__REFOCLK;                         // Set REFO as FLL reference source
    CSCTL0 = 0;                                        // clear DCO and MOD registers
    CSCTL1 &= ~(DCORSEL_7);                            // Clear DCO frequency select bits first
    CSCTL1 |= DCORSEL_5;                               // Set DCORSEL_5 = 16MHz
    CSCTL2 = FLLD_0 + 487;                             // DCOCLKDIV = 16MHz
    __delay_cycles(3);
    __bic_SR_register(SCG0);                           // enable FLL
    while(CSCTL7 & (FLLUNLOCK0 | FLLUNLOCK1));         // FLL locked

    CSCTL4 = SELMS__DCOCLKDIV | SELA__REFOCLK;         // set default REFO(~32768Hz) as ACLK source, ACLK = 32768Hz
                                                       // default DCOCLKDIV as MCLK and SMCLK source

    // SMCLK operates system devices, runs at 4MHz
    CS_initClockSignal(CS_SMCLK, CS_DCOCLKDIV_SELECT, CS_CLOCK_DIVIDER_4);

    mclk_freq_khz = 16000;
    smclk_freq_khz = 4000;
    aclk_freq_hz = CS_getACLK();

    return 0;
}

// Slow clock configured for 1MHz
int clock_cfg_slow()
{
    // Configure ZERO FRAM waitstate as required by the device datasheet for MCLK
    FRCTL0 = FRCTLPW | NWAITS_0;

#if !CLOCK_NO_INIT

    __bis_SR_register(SCG0);                           // disable FLL

    CSCTL3 |= SELREF__REFOCLK;                         // Set REFO as FLL reference source (as opposed to XTAL)
                                                       // REFOCLK operates at 32768khz-ish

    CSCTL0 = 0x100;                                     // Set to 0x100 (256) for midrange tap
    CSCTL1 = DCOFTRIMEN | DCOFTRIM0 | DCOFTRIM1 | DCORSEL_1;    // DCOFTRIM=3
                                                                // DCO Range (DCOR) = 8MHz
                                                                // DCOFTRIM set to 3 because reasons???
    CSCTL2 = FLLD__2 | 30;                             // FLLD__2 divides REFO by 2
                                                       // FLLN is the constant and multiplies REFO_CLK / FLLD
                                                       // Final frequency (DCOCLK) is (FLLN + 1) * (freq_REFOCLK(32.768k) / FLLD)
                                                       // https://www.ti.com/lit/an/slaa791/slaa791.pdf

    __delay_cycles(3);                      // Did I put this here?
    __bic_SR_register(SCG0);                // enable FLL
//    Software_Trim();                        // Software Trim to get the best DCOFTRIM value

    while(CSCTL7 & (FLLUNLOCK0 | FLLUNLOCK1));         // FLL locked // Software trim does this

#endif

//    Software_Trim();                        // Software Trim to get the best DCOFTRIM value

//    CS_clearAllOscFlagsWithTimeout(10);

    CSCTL4 = SELMS__DCOCLKDIV | SELA__REFOCLK;         // Configure MCLK and SMCLK (SELMS) source to DCO_CLKDIV
                                                       // Configure ACLK (SELA) to REFCLK (32.768khz)
                                                       // ACLK unable to source from DCO_CLKDIV
    CSCTL5 = VLOAUTOOFF | (DIVS0);                     // No division on MCLK (DIVM) but divide SMCLK (DIVS) to 500khz

    // MCLK is default 1MHz??
//    CS_initClockSignal(CS_MCLK, CS_DCOCLKDIV_SELECT, CS_CLOCK_DIVIDER_1);
//    CS_initClockSignal(CS_SMCLK, CS_DCOCLKDIV_SELECT, CS_CLOCK_DIVIDER_2);
//    CS_initClockSignal(CS_ACLK, CS_REFOCLK_SELECT, CS_CLOCK_DIVIDER_1); // ACLK has no divider available


    mclk_freq_khz = 1000;
    smclk_freq_khz = 500;
    aclk_freq_hz = 32768;

    return 0;
}


void Software_Trim()
{
    unsigned int oldDcoTap = 0xffff;
    unsigned int newDcoTap = 0xffff;
    unsigned int newDcoDelta = 0xffff;
    unsigned int bestDcoDelta = 0xffff;
    unsigned int csCtl0Copy = 0;
    unsigned int csCtl1Copy = 0;
    unsigned int csCtl0Read = 0;
    unsigned int csCtl1Read = 0;
    unsigned int dcoFreqTrim = 3;
    unsigned char endLoop = 0;

    do
    {
        CSCTL0 = 0x100;                         // DCO Tap = 256
        do
        {
            CSCTL7 &= ~DCOFFG;                  // Clear DCO fault flag
        }while (CSCTL7 & DCOFFG);               // Test DCO fault flag

        __delay_cycles((unsigned int)3000 * MCLK_FREQ_MHZ);// Wait FLL lock status (FLLUNLOCK) to be stable
                                                           // Suggest to wait 24 cycles of divided FLL reference clock
        while((CSCTL7 & (FLLUNLOCK0 | FLLUNLOCK1)) && ((CSCTL7 & DCOFFG) == 0));

        csCtl0Read = CSCTL0;                   // Read CSCTL0
        csCtl1Read = CSCTL1;                   // Read CSCTL1

        oldDcoTap = newDcoTap;                 // Record DCOTAP value of last time
        newDcoTap = csCtl0Read & 0x01ff;       // Get DCOTAP value of this time
        dcoFreqTrim = (csCtl1Read & 0x0070)>>4;// Get DCOFTRIM value

        if(newDcoTap < 256)                    // DCOTAP < 256
        {
            newDcoDelta = 256 - newDcoTap;     // Delta value between DCPTAP and 256
            if((oldDcoTap != 0xffff) && (oldDcoTap >= 256)) // DCOTAP cross 256
                endLoop = 1;                   // Stop while loop
            else
            {
                dcoFreqTrim--;
                CSCTL1 = (csCtl1Read & (~(DCOFTRIM0+DCOFTRIM1+DCOFTRIM2))) | (dcoFreqTrim<<4);
            }
        }
        else                                   // DCOTAP >= 256
        {
            newDcoDelta = newDcoTap - 256;     // Delta value between DCPTAP and 256
            if(oldDcoTap < 256)                // DCOTAP cross 256
                endLoop = 1;                   // Stop while loop
            else
            {
                dcoFreqTrim++;
                CSCTL1 = (csCtl1Read & (~(DCOFTRIM0+DCOFTRIM1+DCOFTRIM2))) | (dcoFreqTrim<<4);
            }
        }

        if(newDcoDelta < bestDcoDelta)         // Record DCOTAP closest to 256
        {
            csCtl0Copy = csCtl0Read;
            csCtl1Copy = csCtl1Read;
            bestDcoDelta = newDcoDelta;
        }

    }while(endLoop == 0);                      // Poll until endLoop == 1

    CSCTL0 = csCtl0Copy;                       // Reload locked DCOTAP
    CSCTL1 = csCtl1Copy;                       // Reload locked DCOFTRIM
    while(CSCTL7 & (FLLUNLOCK0 | FLLUNLOCK1)); // Poll until FLL is locked
}


#ifdef DEBUG_PRINT
int debug_print_init()
{
    int rc = 0;
    // Configure UART parameters
    GPIO_setAsPeripheralModuleFunctionOutputPin(NODE_DBG_PORT, NODE_DBG_TXD_PIN | NODE_DBG_RXD_PIN, GPIO_PRIMARY_MODULE_FUNCTION);

    // http://software-dl.ti.com/msp430/msp430_public_sw/mcu/msp430/MSP430BaudRateConverter/index.html
    // Values configured for 4MHz clock.
    EUSCI_A_UART_initParam uart_debug = {0};
    uart_debug.selectClockSource = EUSCI_A_UART_CLOCKSOURCE_SMCLK;
    if (smclk_freq_khz == 4000)
    {
        uart_debug.clockPrescalar = 2;
        uart_debug.firstModReg = 2;
        uart_debug.secondModReg = 187;
        uart_debug.overSampling = EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION;
    }
//    else if (smclk_freq_khz == 500)
    else
    {
        uart_debug.clockPrescalar = 4;
        uart_debug.firstModReg = 0;
#if !CLOCK_NO_INIT
        uart_debug.secondModReg = 70;
#else
        uart_debug.secondModReg = 170;
#endif
        uart_debug.overSampling = EUSCI_A_UART_LOW_FREQUENCY_BAUDRATE_GENERATION;
    }
    uart_debug.msborLsbFirst = EUSCI_A_UART_LSB_FIRST;
    uart_debug.numberofStopBits = EUSCI_A_UART_ONE_STOP_BIT;
    uart_debug.uartMode = EUSCI_A_UART_MODE;

    rc |= !EUSCI_A_UART_init(EUSCI_A0_BASE, &uart_debug);
    EUSCI_A_UART_enable(EUSCI_A0_BASE);

    return rc;
}

int debug_print(char * buf, uint8_t len)
{
    int rc = 0;

    int count;

    for (count = 0; count < len || buf[count] == 0; count++)
    {
        EUSCI_A_UART_transmitData(EUSCI_A0_BASE, buf[count]);
//        __delay_cycles(250);
    }

    return rc;
}
#endif



// For references:
// MS20 tri-probe white soil sensors: https://www.made-in-china.com/showroom/dalianendeavourtech/product-detailXKVmTijUsrpx/China-Ms-20-Soil-Moisture-Sensor-Temperature-Sensor.html
// MEC10 Tri-probe orange label soil moisture sensor: https://www.aliexpress.com/item/1005002408693259.html
                                                    // https://www.infwin.com/wp-content/uploads/UM-MEC10-Soil-Moisture-EC-and-Temperature-Sensor.pdf
// LWS10 leaf wetness https://www.infwin.com/wp-content/uploads/UM-LWS10-Leaf-Wetness-Sensor.pdf
// Very generic leaf wetness ???? No product name

int uart_init(lf_uart_t * uart)
{

    uart->flag_active = 0;


    // Configure UART hardware pins
    // TODO Configure for variable depending on which base address configured
    *(uart->base_address_txrx_portX + OFS_PASEL0) |= 1<<uart->pin_num_rx | 1<<uart->pin_num_tx;                    // set 2-UART pin as second function
//    P2SEL0 |= 1<<uart->pin_num_rx | 1<<uart->pin_num_tx;                    // set 2-UART pin as second function

    if ((uart->flag_compatibility_rs485))
    {
        *(uart->base_address_DE_port + OFS_PADIR) |= 1<<uart->pin_num_DE;
        *(uart->base_address_REn_port + OFS_PADIR) |= 1<<uart->pin_num_REn;
    }

    // Configure UART
    // Reset the uart device
    *(uart->base_address_ucaX + OFS_UCAxCTLW0) |= UCSWRST;

    // Configure BRCLK (Baud rate clock generator)
    *(uart->base_address_ucaX + OFS_UCAxCTLW0) |= UCSSEL__SMCLK;
    // For SMCLK equal to 1MHZ ish?


    // Baud Rate calculation

    // Values
    // BRCLK (the clock source, in this case is SMCLK)
    // Baud rate

    if (uart->baudrate == 9600)
    {

        // 8000000/(16*9600) = 52.083
        // Fractional portion = 0.083
        // User's Guide Table 14-4: UCBRSx = 0x49
        // UCBRFx = int ( (52.083-52)*16) = 1

        if (smclk_freq_khz == 500)
        {
            /*
            // Table 22-5 in MSP430 user guide

            // UCAxBRW holds only the UCBRx value for clock prescaler (halved value fro 1MHz brclk)
            *(uart->base_address_ucaX + OFS_UCAxBRW) = 3; // Calculate required numbers for baud rate given clock rate

            // 0x20 (UCBRSx) comes from a lookup table for clock source generation for fractional portions
            // For SMCLK at 1MHZ, BRF at 8, UCOS16 is on and UCBRSx
            *(uart->base_address_ucaX + OFS_UCAxMCTLW) |= (0x20 << 8 | (UCOS16 & 1) | UCBRF_8) ;
            */

            // HAND CALCULATED (using excel) VALUES FOR CLOCK OF 1015808/2,
            // UCAxBRW holds only the UCBRx value for clock prescaler (halved value fro 1MHz brclk)
            *(uart->base_address_ucaX + OFS_UCAxBRW) = 3; // Calculate required numbers for baud rate given clock rate

            // 0x20 (UCBRSx) comes from a lookup table for clock source generation for fractional portions
            // For SMCLK at 1MHZ, BRF at 8, UCOS16 is on and UCBRSx
            *(uart->base_address_ucaX + OFS_UCAxMCTLW) |= (0xFB << 8 | (UCOS16 & 1) | UCBRF_4) ;
        }
        else if (smclk_freq_khz == 1000)
        {
            // Table 22-5 in MSP430 user guide
            // UCAxBRW holds only the UCBRx value for clock prescaler
            *(uart->base_address_ucaX + OFS_UCAxBRW) = 6; // Calculate required numbers for baud rate given clock rate

            // 0x49 comes from a lookup table for clock source generation for fractional portions
            // Table 22-5 in MSP430 user guide
            // For SMCLK at 1MHZ, BRF at 8, UCOS16 is on and UCBRSx
            *(uart->base_address_ucaX + OFS_UCAxMCTLW) |= (0x20 << 8 | (UCOS16 & 1) | UCBRF_8) ;
        }
        else
        {
            return 1;
        }
    }
    else if (uart->baudrate == 1200)
    {
        // 8000000/(16*9600) = 52.083
        // Fractional portion = 0.083
        // User's Guide Table 14-4: UCBRSx = 0x49
        // UCBRFx = int ( (52.083-52)*16) = 1

        if (smclk_freq_khz == 500)
        {
            /*
            // Table 22-5 in MSP430 user guide

            // UCAxBRW holds only the UCBRx value for clock prescaler (halved value fro 1MHz brclk)
            *(uart->base_address_ucaX + OFS_UCAxBRW) = 3; // Calculate required numbers for baud rate given clock rate

            // 0x20 (UCBRSx) comes from a lookup table for clock source generation for fractional portions
            // For SMCLK at 1MHZ, BRF at 8, UCOS16 is on and UCBRSx
            *(uart->base_address_ucaX + OFS_UCAxMCTLW) |= (0x20 << 8 | (UCOS16 & 1) | UCBRF_8) ;
            */

            // HAND CALCULATED (using excel) VALUES FOR CLOCK OF F_BRCLK of 507904/2,
            // UCAxBRW holds only the UCBRx value for clock prescaler (halved value from 1MHz brclk)
            *(uart->base_address_ucaX + OFS_UCAxBRW) = 26; // GENERATED THROUGH EXCEL

            // 0x20 (UCBRSx) comes from a lookup table for clock source generation for fractional portions
            *(uart->base_address_ucaX + OFS_UCAxMCTLW) |= (0x44 << 8 | (UCOS16 & 1) | UCBRF_7) ; // GENERATED WITH EXCEL
         }
         else
         {
             return 1;
         }
    }
    else
    {
        return 1;
    }



    // Initialize eUSCI
    *(uart->base_address_ucaX + OFS_UCAxCTLW0) &= ~UCSWRST;
    // Enable USCI_Ax RX/TX interrupt
    *(uart->base_address_ucaX + OFS_UCAxIE) |= UCRXIE | UCTXIE | UCTXCPTIE;

    return 0;
}

int uart_tx(lf_uart_t * uart, uint8_t * buf, uint8_t len)
{
    uart->buffer_full = 0;
    uart->buffer_count = 0;
    uart->buffer_length = len;
    uart->buffer_pointer = (uint8_t*)buf;

    uart->flag_active = 1;

    if ((uart->flag_compatibility_rs485))
    {
        // Output high for DE to transmit
        // Don't care condition for REn but is disabled when high
        *(uart->base_address_DE_port + OFS_PAOUT) |= 1<<uart->pin_num_DE;
        *(uart->base_address_REn_port + OFS_PAOUT) |= 1<<uart->pin_num_REn;
//        *(uart->base_address_REn_port + OFS_PAOUT) &= ~(1 << uart->pin_num_REn);
//        *(uart->base_address_REn_port + OFS_PAOUT) |= 1<<uart->pin_num_REn;
//        __delay_cycles(10);   // Possible required here if a device can't recognize the AB lines fast enough
    }

    // Needs to kickstart with first byte
    *(uart->base_address_ucaX + OFS_UCAxTXBUF) = buf[uart->buffer_count++];

    while ( uart->buffer_full == 0)
    {
//         Enter LPM0 with CPU off only and clocks running
        __bis_SR_register(LPM0_bits | GIE);
    }

//    // Kinda just have to live with this (only for when using txbufempty method?
//    __delay_cycles(3);

    while (*(uart->base_address_ucaX + OFS_UCAxSTATW) & UCBUSY && !wait_for_intrp(1));

    uart->flag_active = 0;

    if ((uart->flag_compatibility_rs485))
    {
        // Output low for DE to stop driving lines
        // Don't care condition for REn but is disabled when high
        *(uart->base_address_DE_port + OFS_PAOUT) &= ~(1 << uart->pin_num_DE);
        *(uart->base_address_REn_port + OFS_PAOUT) &= ~(1 << uart->pin_num_REn);
//        *(uart->base_address_REn_port + OFS_PAOUT) |= 1<<uart->pin_num_REn;
//        __delay_cycles(10);   // Possible required here if a device can't recognize the AB lines fast enough
    }


    return !uart->buffer_full;
}

int uart_rx(lf_uart_t * uart, uint8_t * buf, uint8_t * len)
{
    uart->buffer_full = 0;
    uart->buffer_count = 0;
    uart->buffer_length = *len;
    uart->buffer_pointer = buf;
    uint8_t timeout_flag = 0;

    uart->flag_active = 1;

    if ((uart->flag_compatibility_rs485))
    {
        // Output low for DE to stop driving lines
        // Output low for REn to activate receiving of device from A-B lines (could've tied pins together but oh well)
        *(uart->base_address_DE_port + OFS_PAOUT)   &= ~(1 << uart->pin_num_DE);
        *(uart->base_address_REn_port + OFS_PAOUT)  &= ~(1 << uart->pin_num_REn);
    }

    while ( uart->buffer_full == 0 && timeout_flag == 0)
    {
        // Interrupt USCI_A1_ISR goes off or something.
        timeout_flag = wait_for_intrp(40);
    }

    uart->flag_active = 0;

    if ((uart->flag_compatibility_rs485))
    {
        // Output low for DE to stop driving lines
        // Don't care condition for REn but is disabled when high
//        *(uart->base_address_DE_port + OFS_PAOUT) &= ~(1 << uart->pin_num_DE);
        *(uart->base_address_DE_port + OFS_PAOUT) |= 1<<uart->pin_num_DE;
        *(uart->base_address_REn_port + OFS_PAOUT) |= 1 << uart->pin_num_REn;
    }

    *len = uart->buffer_count;

    return (timeout_flag) | (!uart->buffer_full)<<1  ;
}




#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=USCI_A0_VECTOR
__interrupt void USCI_A0_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(USCI_A0_VECTOR))) USCI_A0_ISR (void)
#else
#error Compiler not supported!
#endif
{
  switch(__even_in_range(UCA0IV,USCI_UART_UCTXCPTIFG))
  {
    case USCI_NONE: break;
    case USCI_UART_UCRXIFG:
//      while(!(UCA0IFG&UCTXIFG));
//      UCA0TXBUF = UCA0RXBUF;
//      __no_operation();
      break;
    case USCI_UART_UCTXIFG: break;
    case USCI_UART_UCSTTIFG: break;
    case USCI_UART_UCTXCPTIFG: break;
    default: break;
  }
}

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=USCI_A1_VECTOR
__interrupt void USCI_A1_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(USCI_A0_VECTOR))) USCI_A1_ISR (void)
#else
#error Compiler not supported!
#endif
{

    switch(__even_in_range(UCA1IV,USCI_UART_UCTXCPTIFG))
    {
      case USCI_NONE: break;
      case USCI_UART_UCRXIFG: // For after receiving a complete character
          _nop();
          if (uart_a1.flag_active == 1 && !uart_a1.buffer_full)
          {
              uart_a1.buffer_pointer[uart_a1.buffer_count++] = UCA1RXBUF;

              uart_a1.buffer_full = uart_a1.buffer_count >= uart_a1.buffer_length;

              __bic_SR_register_on_exit(LPM4_bits);   // May not be in LPM0 mode anyways but it's probably ok to have
  //            debug_print("r", 1);
          }
        break;
      case USCI_UART_UCTXIFG: // Transmit buffer empty
          break;
      case USCI_UART_UCSTTIFG: break;
      case USCI_UART_UCTXCPTIFG:  // Case of UART TX done

          if (uart_a1.flag_active == 1 && !uart_a1.buffer_full)
          {
              UCA1TXBUF = uart_a1.buffer_pointer[uart_a1.buffer_count++];
              if (uart_a1.buffer_count == uart_a1.buffer_length)
              {
                  uart_a1.buffer_full = 1;
              }
              __bic_SR_register_on_exit(LPM4_bits);   // May not be in LPM0 mode anyways but it's probably ok to have
          }

          break;
      default: break;
    }


    // clear interrupt vector(?)
    //  UCA1IV = 0;
}


uint16_t mean(uint16_t *list, uint16_t len)
{
    uint32_t avg = 0;
    uint16_t i = 0;
    for (i = 0; i < len; i++)
    {
        avg += list[i];
    }
    avg /= len;
    return (uint16_t)avg;
}

uint16_t mean_log(uint16_t *list, uint16_t len_log)
{
    uint32_t avg = 0;
    uint16_t i;
    for (i = (1 << len_log); i > 0; i--)
    {
        avg += list[i-1];
    }
    avg = avg >> len_log;
    return (uint16_t)avg;
}

int16_t mean_log_s16(int16_t *list, uint16_t len_log)
{
    int32_t avg = 0;
    uint16_t i;
    for (i = (1 << len_log); i > 0; i--)
    {
        avg += list[i-1];
    }
    avg = avg >> len_log;
    return (int16_t)avg;
}

int32_t mean_log_s32(int32_t *list, uint16_t len_log)
{
    int64_t avg = 0;
    uint16_t i;
    for (i = (1 << len_log); i > 0; i--)
    {
        avg += list[i-1];
    }
    avg = avg >> len_log;
    return (int32_t)avg;
}

int16_t mean_s16(int16_t *list, uint16_t len)
{
    int32_t avg = 0;
    uint16_t i;
    for (i = len; i > 0; i--)
    {
        avg += list[i-1];
    }
    avg /= len;
    return (int16_t)avg;
}

int32_t mean_s32(int32_t *list, uint16_t len)
{
    int64_t avg = 0;
    uint16_t i;
    for (i = len; i > 0; i--)
    {
        avg += list[i-1];
    }
    avg /= len;
    return (int32_t)avg;
}

void delay_usec(uint16_t time)
{
    // Unsigned long defined in STDINT with uint32_t
    uint16_t count;
    for (count = time * (mclk_freq_khz >> 10) >> 3; count > 0 ; count--)
    {
        __delay_cycles(10);    // Approximate division by 1000
//        __delay_cycles(mclk_freq_khz / 1000);
    }
}

void delay_msec(uint16_t time)
{
    uint16_t count;
    for (count = time * (mclk_freq_khz >> 10); count > 0; count--)
    {
        __delay_cycles(1000);   // 1ms if 1MHz
    }
}

void delay_sec(uint16_t time)
{
    // Maximum cycle count is 4bn, around 250 seconds at 16MHz
    uint16_t count;
    for (count = time * (mclk_freq_khz >> 10) ; count > 0; count--)
        __delay_cycles(1000000);
}


// 16 bit time max ~= 1 minute
// 32 bit time max ~= 49 days
volatile uint32_t tick_msec = 0;
volatile uint16_t stop_mode_done = 0;

void timer_a0_cfg()
{
    // Timer A0 for generating timer Ticks
    // Frequency of 7.8125 kHz (128.0 usec periods) based off 32.786 khz ACLK

//        Capture compare register TA0.0 unused
//        Capture compare register TA0.1 for ticks (am I really sure about this)
//        Capture compare register TA0.2 unused

    static Timer_A_initCompareModeParam cctl_param = {0};
    cctl_param.compareInterruptEnable = TIMER_A_CAPTURECOMPARE_INTERRUPT_ENABLE;
    cctl_param.compareOutputMode = TIMER_A_OUTPUTMODE_OUTBITVALUE;
    cctl_param.compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_0;
    cctl_param.compareValue = 1000;
    Timer_A_initCompareMode(TIMER_A0_BASE, &cctl_param);

    TA0CCTL0 = CCIE;                             // TACCR0 interrupt enabled
//    TA0CCTL1 = CCIE;                             // TACCR0 interrupt enabled

//            //ACLK     COUNT  clear ctr interrupt en
//    TA0CTL = TASSEL_1 | MC__CONTINUOUS | TACLR |  TAIE;

    Timer_A_setCompareValue(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0, 10);
//    Timer_A_setCompareValue(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_1, 0);
//    Timer_A_setCompareValue(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_2, 0);

    Timer_A_initContinuousModeParam param = {0};
    param.clockSource = TIMER_A_CLOCKSOURCE_ACLK;  //ACLK 32.768kHz
    param.clockSourceDivider = TIMER_A_CLOCKSOURCE_DIVIDER_32; // 1khzish
    param.timerInterruptEnable_TAIE = TIMER_A_TAIE_INTERRUPT_ENABLE;
    param.timerClear = TIMER_A_DO_CLEAR;
    param.startTimer = 1;
    Timer_A_initContinuousMode(TIMER_A0_BASE, &param);

    return;
}

void stop_mode(uint32_t stop_time)
{

    // Driverlib version
    static Timer_A_initCompareModeParam cctl_param = {0};
    cctl_param.compareInterruptEnable = TIMER_A_CAPTURECOMPARE_INTERRUPT_ENABLE;
    cctl_param.compareOutputMode = TIMER_A_OUTPUTMODE_OUTBITVALUE;
    cctl_param.compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_1;
    cctl_param.compareValue = TA0R + stop_time;
    Timer_A_initCompareMode(TIMER_A0_BASE, &cctl_param);

    volatile uint32_t time_remaining = stop_time;
    do
    {
        if (time_remaining > 0xFFFF)
        {
            time_remaining = time_remaining - 0xFFFF;
            TA0CCR1 = TA0R + 0xFFFF;
        }
        else
        {
            TA0CCR1 = TA0R + (time_remaining & 0xFFFF) + 3;
            time_remaining = 0;
        }

        stop_mode_done = 0;
        // Sleeping after this line
        while (stop_mode_done == 0)
        {
            __bis_SR_register(LPM3_bits | GIE);

            // Should be woken up by timer onl
        }

        // MCU may wake up for other issues however it should go back to sleep.
    } while (time_remaining > 0);

    return;
}

int wait_for_intrp(uint32_t stop_time)
{


    // Driverlib version
    // Compares with CCR1 not CCR0 which is dedicated for ticks only
    static Timer_A_initCompareModeParam cctl_param = {0};
    cctl_param.compareInterruptEnable = TIMER_A_CAPTURECOMPARE_INTERRUPT_ENABLE;
    cctl_param.compareOutputMode = TIMER_A_OUTPUTMODE_OUTBITVALUE;
    cctl_param.compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_1;
    cctl_param.compareValue = (TA0R + stop_time) & 0xFFFF;  // May overflow but will be converted back to uint16_t
    Timer_A_initCompareMode(TIMER_A0_BASE, &cctl_param);

    volatile uint32_t time_remaining = stop_time;
    do
    {
        // Need to check and account for times greater than 16 bits only
        if (time_remaining > 0xFFFF)
        {
            time_remaining = time_remaining - 0xFFFF;
            TA0CCR1 = TA0R + 0xFFFF;    //https://onlinegdb.com/rLVvfecmk
        }
        else
        {
            // Get current count and add whatever time remaining.
            // Must add 1 to the counter as otherwise timer may equal or exceed desired compare value before actually entering sleep, thus preventing wakeups
            TA0CCR1 = TA0R + (time_remaining & 0xFFFF) + 1;
            time_remaining = 0;
        }

        stop_mode_done = 0;

        // Sleeping after this line
        // Only LPM0 for keeping all clocks and peripherals on
        // Woken up by TIMER0_A1_ISR
        // May be woken up by other things? Required to be in while loop until actually flagged as done through stop_mode_done
        __bis_SR_register(LPM0_bits | GIE);

        // If woken up by separate interrupt, must return
        if (!stop_mode_done)
        {
            time_remaining = 0;
        }

        // MCU may wake up for other issues however it should NOT go back to sleep.
    } while (time_remaining > 0);

    // 0 if interrupted by desired interrupt
    // 1 if timer interrupted
    return stop_mode_done;
}




// Timer0_A0 CCR0 handler (for TIMER_A_CAPTURECOMPARE_REGISTER_0 ONLY)
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=TIMER0_A0_VECTOR
__interrupt void TIMER0_A0_ISR(void)
#endif
{
    tick_msec++;
//    if (tick_ms % 1000 == 0)
//    {
//        tick_s++;
//    }

    // Add 1 to the counter for next tick
    Timer_A_setCompareValue(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0, TA0R + 1);
}



// Timer0_A1 Interrupt Vector (TAIV) handler (general version with other interrupt sources too)
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=TIMER0_A1_VECTOR
__interrupt void TIMER0_A1_ISR(void)
//#elif defined(__GNUC__)
//void __attribute__ ((interrupt(TIMER0_A1_VECTOR))) TIMER0_A1_ISR (void)
//#else
//#error Compiler not supported!
#endif
{

    switch(__even_in_range(TA0IV,TA0IV_TAIFG))
    {
        case TA0IV_NONE:
            break;                               // No interrupt
        case TA0IV_TACCR1:
            // Exit STOP mode sleep state
            stop_mode_done = 1;
            __bic_SR_register_on_exit(LPM4_bits);
            break;                               // CCR1 not used
        case TA0IV_TACCR2:
            break;                               // CCR2 not used
        case TA0IV_TAIFG:
            // Overflow conditions and others
            break;
        default:
            break;
    }
}


/* Initialize non-used ISR vectors with a trap function */

//#pragma vector=PORT2_VECTOR
//#pragma vector=PORT1_VECTOR
//#pragma vector=TIMER1_A1_VECTOR   // utilized in utils_msp.c
//#pragma vector=TIMER1_A0_VECTOR   // utilized in utils_msp.c
//#pragma vector=TIMER0_A1_VECTOR
//#pragma vector=TIMER0_A0_VECTOR
#pragma vector=ADC10_VECTOR
#pragma vector=USCIAB0TX_VECTOR
#pragma vector=WDT_VECTOR
#pragma vector=USCIAB0RX_VECTOR
#pragma vector=NMI_VECTOR
#pragma vector=COMPARATORA_VECTOR

__interrupt void ISR_trap(void)
{

}

