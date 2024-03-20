/*
 * utils_msp.h
 *
 *  Created on: Mar. 17, 2024
 *      Author: someone
 */

#ifndef UTILS_MSP_H_
#define UTILS_MSP_H_


#define UM_ADC_READ_AVG_CNT 16
#define UM_ADC_READ_AVG_LOG2 4

typedef enum um_adc_samples_e
{
    UM_ADC_READ_ONE = ADC_CYCLEHOLD_4_CYCLES,
    UM_ADC_READ_ONE_LONG = ADC_CYCLEHOLD_64_CYCLES,
    UM_ADC_READ_AVG
} um_adc_samples_t;


#define UM_ADC_RANGE_THRESH_BIT (1 << 10) - (1 << 8)
typedef enum um_adc_range_e
{
    UM_ADC_RANGE_SMALL = ADC_VREFPOS_INT,
    UM_ADC_RANGE_LARGE = ADC_VREFPOS_AVCC,
    UM_ADC_RANGE_AUTO,
    UM_ADC_RANGE_VREFP,
    UM_ADC_RANGE_VCC_VREFN
} um_adc_range_t;

typedef enum um_adc_ch_e
{
    UM_ADC_CH_A0 = ADC_INPUT_A0,
    UM_ADC_CH_A1 = ADC_INPUT_A1,
    UM_ADC_CH_A2 = ADC_INPUT_A2,
    UM_ADC_CH_A3 = ADC_INPUT_A3,
    UM_ADC_CH_A4 = ADC_INPUT_A4,
    UM_ADC_CH_A5 = ADC_INPUT_A5,
    UM_ADC_CH_A6 = ADC_INPUT_A6,
    UM_ADC_CH_A7 = ADC_INPUT_A7,
    UM_ADC_CH_A8 = ADC_INPUT_A8,
    UM_ADC_CH_A9 = ADC_INPUT_A9,
    UM_ADC_CH_TEMPSENSOR = ADC_INPUT_TEMPSENSOR,
    UM_ADC_CH_REFVOLTAGE = ADC_INPUT_REFVOLTAGE,
    UM_ADC_CH_DVSS = ADC_INPUT_DVSS,
    UM_ADC_CH_DVCC = ADC_INPUT_DVCC
} um_adc_ch_t;


int um_adc_get( um_adc_ch_t channel , um_adc_samples_t samples, um_adc_range_t * range,  int16_t * readback );
int um_adc_getnext(int16_t * readback);



typedef struct um_tim_infos_s
{

    void (*ccr_call)();

    uint16_t * ccr_list;
    uint16_t ccr_list_len;
    uint16_t ccr_list_progress;

} um_tim_infos_t;

um_tim_infos_t tim1_0;
um_tim_infos_t tim1_1;
um_tim_infos_t tim1_2;

int um_tim1_base(uint16_t divider, uint16_t * ccr_list, uint16_t ccr_list_len, void (*to_call)());




#endif /* UTILS_MSP_H_ */
