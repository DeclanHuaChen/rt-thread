#ifndef __DRV_RIO_ADC_H__
#define __DRV_RIO_ADC_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

enum adc_range
{
    ADC_RANGE_10V = 0x0,
    ADC_RANGE_5V = 0x4,
    ADC_RANGE_2V = 0x8,
    ADC_RANGE_1V = 0x1,
    ADC_RANGE_100MV = 0x2,
    ADC_RANGE_10MV = 0xE
};

rt_err_t ADC_Reset();
void ADC_Set_Channel_Mask(rt_uint8_t mask);
rt_err_t ADC_Config_Channel_Mode(uint32_t chan, uint32_t mode);
rt_err_t ADC_Config_Channel_Range(uint32_t chan, enum adc_range Range_Value);
rt_err_t ADC_Config_Sample_Clock(uint32_t Sample_Clock);
rt_err_t ADC_Config_Trig_Type(uint32_t type);
rt_err_t ADC_Config_Sample_Num(uint32_t num);
void ADC_Acq_Start();
void ADC_Acq_Stop();
rt_err_t ADC_Read_Temperature(double *temper);
rt_err_t ADC_Write_Staic_Cal();
rt_err_t ADC_Updata_Cal(int calrange, int calvalue);
rt_err_t ADC_Read_Cal(float *data, rt_uint32_t size);
rt_err_t ADC_Read_Raw_Data(uint32_t *bData, size_t size);
rt_err_t ADC_Read_Average_Data(double *result, size_t size, double GainCoff);

#ifdef __cplusplus
}
#endif

#endif
