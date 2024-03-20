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




int um_tim1_base(uint16_t divider, uint16_t * ccr_list, uint16_t ccr_list_len, void (*to_call)())
{

    tim1_0.ccr_list = ccr_list;
    tim1_0.ccr_list_len = ccr_list_len;
    tim1_0.ccr_list_progress = 0;
    tim1_0.ccr_call = to_call;


//        Capture compare register TA1.0 for a thing
//        Capture compare register TA1.1 for ticks
//        Capture compare register TA1.2 unused
//    Present counter value stored in [TA1R]

    static Timer_A_initCompareModeParam cctl_param = {0};
    cctl_param.compareInterruptEnable = TIMER_A_CAPTURECOMPARE_INTERRUPT_ENABLE;
    cctl_param.compareOutputMode = TIMER_A_OUTPUTMODE_OUTBITVALUE;
    cctl_param.compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_0;
    cctl_param.compareValue = 0;
    Timer_A_initCompareMode(TIMER_A1_BASE, &cctl_param);

    Timer_A_setCompareValue(TIMER_A1_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0, TA1R + tim1_0.ccr_list[tim1_0.ccr_list_progress]);

    TA1CCTL0 = CCIE;                             // TACCR0 interrupt enabled (alternatively use TA1CCTL1)

    Timer_A_initContinuousModeParam param = {0};
    param.clockSource = TIMER_A_CLOCKSOURCE_SMCLK;  //ACLK 32.768kHz
    param.clockSourceDivider = TIMER_A_CLOCKSOURCE_DIVIDER_16;
    param.timerInterruptEnable_TAIE = TIMER_A_TAIE_INTERRUPT_ENABLE;
    param.timerClear = TIMER_A_DO_CLEAR;
    param.startTimer = 1;
    Timer_A_initContinuousMode(TIMER_A1_BASE, &param);

    return 0;
}





#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=TIMER1_A0_VECTOR // THIS VECTOR IS ONLY FOR THE CCR0 FLAG, NOT ANY OTHER ONES!
__interrupt void TIMER1_A0_ISR(void)
#endif
{

    if (tim1_0.ccr_call)
    {
        tim1_0.ccr_call();
    }

//    P2OUT ^= GPIO_PIN6;

    uint16_t next = tim1_0.ccr_list[ (tim1_0.ccr_list_progress) % tim1_0.ccr_list_len ];
    next = (next + TA1CCR0) % 0xFFFF;

    Timer_A_setCompareValue(TIMER_A1_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0, next );
    tim1_0.ccr_list_progress++;

}


// Timer1_A1 Interrupt Vector (TAIV) handler (general version with other interrupt sources too)
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=TIMER1_A1_VECTOR
__interrupt void TIMER1_A1_ISR(void)
#endif
{

    uint16_t next = 0;

    switch(__even_in_range(TA1IV,TA1IV_TAIFG))
    {
        case TA1IV_NONE:
            break;                               // No interrupt
        case TA1IV_TACCR1:
            if (tim1_1.ccr_call)
            {
                tim1_1.ccr_call();
            }
            next = tim1_1.ccr_list[ (tim1_1.ccr_list_progress++) % tim1_1.ccr_list_len ];
            next = (next + TA1CCR1) % 0xFFFF;
            Timer_A_setCompareValue(TIMER_A1_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0, next );
            break;                               // CCR1 not used
        case TA1IV_TACCR2:
            if (tim1_2.ccr_call)
            {
                tim1_2.ccr_call();
            }
            next = tim1_2.ccr_list[ (tim1_2.ccr_list_progress++) % tim1_2.ccr_list_len ];
            next = (next + TA1CCR2) % 0xFFFF;
            Timer_A_setCompareValue(TIMER_A1_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0, next );
            break;                               // CCR2 not used
        case TA1IV_TAIFG:
            // Overflow conditions and others
            break;
        default:
            break;
    }

}

