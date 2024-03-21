/* --COPYRIGHT--,BSD
 * Copyright (c) 2017, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/


#include <driverlib.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>


#include "utils_generic.h"
#include "leaf_utils.h"
#include "i2c_bb.h"
#include "rs485_devices.h"
#include "utils_msp.h"

void Init_GPIO();
void toggle_p2_6();
void p2_6_off();
void p2_6_on();

#ifdef DEBUG_PRINT
static char uart_send[64] = {0};
#endif

ug_pid_t pid = {0};
// Instructions: do a same exponential for proportional
// do single digits for integrals
// i lim is post exponential
ug_pid_int pid_kp = 0<<10;  // k values not affected by exponentials
ug_pid_int pid_ki = 1<<2;
ug_pid_int pid_kd = 0;
ug_pid_int command = 0;
ug_pid_int pid_exp = 8; // set for 2^-7 (1/128) decimal precision


int main(void) {

    volatile uint8_t rc = 0;

    volatile uint16_t i = 0;
    volatile uint16_t j = 0;
    volatile uint16_t k = 0;


    ug_pid_init(&pid, pid_kp, pid_ki, pid_kd, 0);
    ug_pid_set_exponential(&pid, pid_exp);
    ug_pid_set_target(&pid, 50);
    ug_pid_set_endstop(&pid, (-1<<8)<<8, (1<<8)<<8);    // 16 bit 32k max
    ug_pid_set_i_lim(&pid, -6<<8, 6<<8);                //



 // #     #  #     #                             
 // #     #  #  #  #       ####   ######   ####  
 // #     #  #  #  #      #    #  #       #    # 
 // #######  #  #  #      #       #####   #      
 // #     #  #  #  #      #       #       #  ### 
 // #     #  #  #  #      #    #  #       #    # 
 // #     #   ## ##        ####   #        ####  
                                              
    // Stop watchdog timer
    WDT_A_hold(WDT_A_BASE);
    // ~32 msec timer given a 16MHz SMCLK (if that's the actual frequency)
    WDT_A_initIntervalTimer(WDT_A_BASE, WDT_A_CLOCKSOURCE_SMCLK, WDT_A_CLOCKDIVIDER_8192K);

    __enable_interrupt(); // Enable Global Interrupts

    clock_cfg();
//    clock_cfg_slow();

    volatile uint32_t mclk = CS_getMCLK();
    volatile uint32_t smclk = CS_getSMCLK();
    volatile uint32_t aclk = CS_getACLK();

//    For the ticks
    timer_a0_cfg();
//  CLOCK CONFIGURATION END


// UART CONFIGURATION START
#ifdef DEBUG_PRINT
    memset(&uart_send, 0, 50);
    debug_print_init();
    const uint8_t * vect_sys_rst = (const uint8_t*)0x015E;
#endif
//  END UART INITIALIZATION


// ____ ___  _ ____    ____ ____ ____ 
// | __ |__] | |  |    |    |___ | __ 
// |__] |    | |__|    |___ |    |__] 
                                   

/*    For GPIO reference
P1DIR
P1OUT
P1IN
//*/
//  GPIO PIN CONFIGURATION BEGIN
    GPIO_clearInterrupt(GPIO_PORT_P1, GPIO_PIN_ALL16);
    GPIO_clearInterrupt(GPIO_PORT_P2, GPIO_PIN_ALL16);
    GPIO_disableInterrupt(GPIO_PORT_P1, GPIO_PIN_ALL16);
    GPIO_disableInterrupt(GPIO_PORT_P2, GPIO_PIN_ALL16);

    GPIO_setAsOutputPin(PWR_OUT_ENn_PORT, PWR_OUT_ENn_PIN);
    GPIO_setOutputLowOnPin(PWR_OUT_ENn_PORT, PWR_OUT_ENn_PIN);

    GPIO_setAsOutputPin(RS485_DE_PORT, RS485_DE_PIN);
    GPIO_setOutputHighOnPin(RS485_DE_PORT, RS485_DE_PIN);
    GPIO_setAsOutputPin(RS485_RX_PORT, RS485_RX_PIN);
    GPIO_setOutputLowOnPin(RS485_RX_PORT, RS485_RX_PIN);
    GPIO_setAsOutputPin(RS485_REn_PORT, RS485_REn_PIN);
    GPIO_setOutputLowOnPin(RS485_REn_PORT, RS485_REn_PIN);


    // Configure ADC
    SYSCFG2 |= ADCPCTL2 | ADCPCTL6 | ADCPCTL7;

    PMMCTL0_H = PMMPW_H;                                      // Unlock the PMM registers
    PMMCTL2 |= INTREFEN;                                      // Enable internal reference


//  GPIO PIN CONFIGURATION END

    // GPIO is by default locked to high impedance mode after boot.
    // I/O functions must be unlocked by clearing LOCKLPM5 bit
    PMM_unlockLPM5();
    WDT_A_start(WDT_A_BASE);



    int rc_uart = 0;



    // Print some messages
#ifdef DEBUG_PRINT
    sprintf( uart_send, "\n\rMSP CHAINSAW :)\n\r\r\r ");
//    sprintf( uart_send, "\n\rwind rain simulator :)\n\r       ");
    debug_print(uart_send, 25);
    sprintf( uart_send, "\n\rRST: %x\n\r\r\r\r\r\r", *vect_sys_rst);
    debug_print(uart_send, 12);
#endif



    unsigned int loops = 0;


    GPIO_setOutputLowOnPin(RS485_RX_PORT, RS485_RX_PIN);
//    With a 4MHz smclk/16=250kHz timer
//    1msec=250, 20msec=5k total cycle time
//    Sum of pwm_modes must be 5k
#define PWM_CYCLES 2
    uint16_t pwm_modes[PWM_CYCLES] = {0};
//    uint16_t pwm_modes[] = { 400 , 4600};
    pwm_modes[0] = 250;
    pwm_modes[1] = 4750;
//    pwm_modes[2] = 1750;

    um_tim1_base(TIMER_A_CLOCKSOURCE_DIVIDER_16 , pwm_modes, PWM_CYCLES, toggle_p2_6);


    // Main Loop
    while(1)
    {


        __enable_interrupt();


        WDT_A_resetTimer(WDT_A_BASE);
#ifdef DEBUG_PRINT
        sprintf(uart_send, "\n\rkick %i \t%i\n\r\r\r\r\r", loops++, tick_msec);
        debug_print(uart_send, 23);
#endif



//        stop_mode(500);



        if (1 && loops % 200 == 0)
        {
//            GPIO_setOutputLowOnPin(RS485_REn_PORT, RS485_REn_PIN);
//            GPIO_setOutputLowOnPin(RS485_RX_PORT, RS485_RX_PIN);
//            GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN7);
//            stop_mode(1);
//            GPIO_toggleOutputOnPin(PWR_OUT_ENn_PORT, PWR_OUT_ENn_PIN);
            GPIO_setOutputHighOnPin(PWR_OUT_ENn_PORT, PWR_OUT_ENn_PIN);
//            __delay_cycles(30000);
//            stop_mode(25);
        }
//        if (loops % 35 == 0)
//        {
//            GPIO_toggleOutputOnPin(GPIO_PORT_P2, GPIO_PIN7);
//        }
//        if (loops % 55 == 0)
//        {
//            GPIO_toggleOutputOnPin(RS485_REn_PORT, RS485_REn_PIN);
//        }


        pwm_modes[0] = abs( (loops) % 500 - 250 ) + 250;
        pwm_modes[1] = 5000 - pwm_modes[0];
        pwm_modes[1] = pwm_modes[1] < 4750? pwm_modes[1] : 4750;



//        UM_ADC_CH_A6 for the actual external one
        int16_t reeeeed = 0;
        int16_t rainge = UM_ADC_RANGE_LARGE;
        const int16_t inputch = ADC_INPUT_REFVOLTAGE;

        um_adc_get( inputch , UM_ADC_READ_ONE_LONG, (um_adc_range_t*)&rainge,  &reeeeed );
//        um_adc_get( ADC_INPUT_A6 , UM_ADC_READ_ONE, (um_adc_range_t*)&rainge,  &reeeeed );



        int32_t reemv = reeeeed;

        switch (rainge)
        {
        case UM_ADC_RANGE_SMALL:
            reemv *= 1500;
            reemv = reemv >> 10;
            rainge = 1;
            break;
        case UM_ADC_RANGE_LARGE:
            if (inputch == ADC_INPUT_REFVOLTAGE)
            {
                reemv = 1500;
                reemv *= 1 << 10;
                reemv /= reeeeed;
                rainge = 4;
            }
            else
            {
                reemv *= 3300;
                reemv = reemv >> 10;
                rainge = 2;
            }
            break;
        case UM_ADC_RANGE_VREFP:
            reemv = 1<<10;
            reemv *= 1500;
            reemv /= reeeeed;
            rainge = 3;
            break;
        default:
            rainge = -1;
            break;
        }



        
        int thing = loops % 1000 / 10;

        ug_pid_update(&pid, thing , &command);

#ifdef DEBUG_PRINT
        sprintf(uart_send, "\rsummary: %i \t%i\n\r\r\r\r", thing, (uint32_t)command);
        debug_print(uart_send, 38);
        sprintf(uart_send, "\r\t\t\tADC: %imV \t%i \t{%i} \n\r\r\r\r", (uint16_t)reemv, reeeeed, rainge);
        debug_print(uart_send, 38);
        sprintf(uart_send, "\r\t\t\t\t\t\ttim %u / %u | %u\n\r\r\r\r\r\r\r\r\r", TA1R , TA1CCR0, pwm_modes[0]);
        debug_print(uart_send, 50);
#endif


    }
}



void toggle_p2_6()
{
//    GPIO_toggleOutputOnPin(RS485_RX_PORT, RS485_RX_PIN);
    P2OUT ^= GPIO_PIN6;
}


void p2_6_off()
{
    P2OUT &= ~GPIO_PIN6;
}
void p2_6_on()
{
    P2OUT |= GPIO_PIN6;
}

#pragma vector=PORT1_VECTOR
__interrupt void isr_port_1(void)
{


    // Use switch for determining specific pin conditions
    switch(__even_in_range(P1IV,16))
//    switch(GPIO_getInterruptStatus(GPIO_PORT_P1, GPIO_PIN_ALL8))
    {
        // When using __even_in_range, Case is equal to (pin*2)+2
        // When using GPIO_getInterruptStatus, case is equal to pin+1
        case P1IV_NONE:
            break; // No pin
        case P1IV_P1IFG0: // P1.0
            break;
        case P1IV_P1IFG1: // P1.1
//            sprintf( uart_send, "P1.1  ");
//            debug_print(uart_send, 7);

            // Using LPM4_bits turns back on all peripherals that were off during LPM4
            // LPM4 turns off the most peripherals
            // clear status register bits
            __bic_SR_register_on_exit(LPM4_bits| GIE);

            break;
        case P1IV_P1IFG2: break; // P1.2
        case P1IV_P1IFG3: break; // P1.3
        case P1IV_P1IFG4: break; // P1.4
        case P1IV_P1IFG5: break; // P1.5
        case P1IV_P1IFG6: break; // P1.6
        case P1IV_P1IFG7: break; // P1.7
    }

    // Setting port 1 interrupt flag to zero (clear interrupts)
    GPIO_clearInterrupt(GPIO_PORT_P1, GPIO_PIN_ALL16);

}


#pragma vector=PORT2_VECTOR
__interrupt void isr_port_2(void)
{
    // Using LPM4_bits turns back on all peripherals that were off during LPM4
    // LPM4 turns off the most peripherals
    // clear status register bits
    __bic_SR_register_on_exit(LPM4_bits);

    // Use switch for determining specific pin conditions
    // even in range removes odd values
    switch(__even_in_range(P2IV,16))
//    switch(GPIO_getInterruptStatus(GPIO_PORT_P1, GPIO_PIN_ALL8))
    {
        // When using __even_in_range, Case is equal to (pin*2)+2
        // When using GPIO_getInterruptStatus, case is equal to pin+1
        case P2IV_NONE: break; // No pin
        case P2IV_P2IFG0: break; // P2.0
        case P2IV_P2IFG1: break; // P2.1
        case P2IV_P2IFG2: break; // P2.2
        case P2IV_P2IFG3: break; // P2.3
        case P2IV_P2IFG4: break; // P2.4
        case P2IV_P2IFG5: break; // P2.5
        case P2IV_P2IFG6: break; // P2.6
        case P2IV_P2IFG7: break; // P2.7
    }

    // Setting port 1 interrupt flag to zero (clear interrupts)
    GPIO_clearInterrupt(GPIO_PORT_P2, GPIO_PIN_ALL16);

}




/*
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
*/
