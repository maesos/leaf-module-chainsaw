/*
 * utils_msp.c
 *
 *  Created on: Mar. 17, 2024
 *      Author: someone
 */
#include <driverlib.h>
#include "utils_msp.h"



int um_adc_get( um_adc_ch_t channel , um_adc_samples_t samples, um_adc_range_t * range,  int16_t * readback )
{

    int rc = 0;

    uint8_t range_p;
    uint8_t range_n;

    switch (*range)
    {
    case UM_ADC_RANGE_AUTO:

        *range = UM_ADC_RANGE_SMALL;
        um_adc_get( channel , samples, range,  readback );

        range_p = *readback > UM_ADC_RANGE_THRESH_BIT ? UM_ADC_RANGE_LARGE : UM_ADC_RANGE_SMALL;
        range_n = ADC_VREFNEG_AVSS;
        break;
    case UM_ADC_RANGE_SMALL:
        range_p = ADC_VREFPOS_INT;
        range_n = ADC_VREFNEG_AVSS;
        break;
    case UM_ADC_RANGE_LARGE:
        range_p = ADC_VREFPOS_AVCC;
        range_n = ADC_VREFNEG_AVSS;
        break;
    case UM_ADC_RANGE_VREFP:
        range_p = ADC_VREFPOS_EXT_NOBUF;
        range_n = ADC_VREFNEG_AVSS;
        break;
    case UM_ADC_RANGE_VCC_VREFN:
        range_p = ADC_VREFPOS_AVCC;
        range_n = ADC_VREFNEG_EXT;
        break;
    }

    int cyclehold = samples == UM_ADC_READ_AVG ? UM_ADC_READ_ONE_LONG : samples;
    int cyclecount = samples == UM_ADC_READ_AVG ? UM_ADC_READ_AVG_CNT : 1;

    // Initialize adc with the clock settings
    // 21.3.2 (SLAU445i) sample hold source either from bit set or from a timer
    // clock source adc oscilloator and divider as required
    ADC_init(ADC_BASE, ADC_SAMPLEHOLDSOURCE_SC, ADC_CLOCKSOURCE_ADCOSC, ADC_CLOCKDIVIDER_1);
    // Get ADC module on
    ADC_enable(ADC_BASE);
    // Cycle hold time and continuous conversion on/off
    ADC_setupSamplingTimer(ADC_BASE, cyclehold, ADC_MULTIPLESAMPLESDISABLE);
    // Channel and references (ADC_INPUT_REFVOLTAGE or ADC_VREFPOS_AVCC)
    ADC_configureMemory(ADC_BASE, channel, range_p, range_n);   // Configured for reference with 1.2V internal reference


    // Knowing that using 32 samples in 10 bit format ensures total sum does not exceed 15 bits (is 15 bits) (unsigned)
    uint16_t sum = 0;

    int xfsdfsd = 0;
    for (xfsdfsd = 0; xfsdfsd < cyclecount; xfsdfsd++)
    {
        // ADCENC and ADCSC toggled for starting
        ADC_startConversion(ADC_BASE, ADC_SINGLECHANNEL);


        while( !(ADC_getInterruptStatus(ADC_BASE,ADC_COMPLETED_INTERRUPT_FLAG)) );
        *readback = ADCMEM0;
        // Clear the Interrupt Flag and start another conversion
        ADC_clearInterrupt(ADC_BASE, ADC_COMPLETED_INTERRUPT_FLAG);

//        Alternative method
//        um_adc_getnext(readback);

        sum += *readback;
    }

    ADC_disableConversions(ADC_BASE, 1);

    if (cyclecount == UM_ADC_READ_AVG_CNT)
    {
        *readback = sum >> UM_ADC_READ_AVG_LOG2;
    }


    return rc;

}

int um_adc_getnext(int16_t * readback)
{
    while( !(ADC_getInterruptStatus(ADC_BASE,ADC_COMPLETED_INTERRUPT_FLAG)) );

    *readback = ADCMEM0;

    // Clear the Interrupt Flag and start another conversion
    ADC_clearInterrupt(ADC_BASE, ADC_COMPLETED_INTERRUPT_FLAG);

    return 0;
}


int um_tim_init()
{

}

int um_tim_init_ticks()
{

    // 16 bit time max ~= 1 minute
    // 32 bit time max ~= 49 days
    volatile uint32_t tick_msec = 0;
    volatile uint16_t stop_mode_done = 0;


    // Timer A0 for generating timer Ticks
    // Frequency of 7.8125 kHz (128.0 usec periods) based off 32.786 khz ACLK

//        Capture compare register TA0.0 unused
//        Capture compare register TA0.1 for ticks
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
//    param.clockSourceDivider = 0b111;  // Timer divide by 8
    param.clockSourceDivider = TIMER_A_CLOCKSOURCE_DIVIDER_32;
    param.timerInterruptEnable_TAIE = TIMER_A_TAIE_INTERRUPT_ENABLE;
    param.timerClear = TIMER_A_DO_CLEAR;
    param.startTimer = 1;
    Timer_A_initContinuousMode(TIMER_A0_BASE, &param);

    return;
}
