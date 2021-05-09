#include <rtthread.h>
#include "xil_io.h"
#include "drv_rio_adc.h"

//*********Read Address**************//
#define Addr_ID_Module 0x000
#define Addr_Self_Test 0x004
#define Addr_EPROM_ACK 0x0D4
#define Addr_EPROM_Data 0x0D8
#define Addr_AI_FIFO_Status 0x140
#define Addr_RTD_Temprature 0x150

//*********Write Address & Command**************//
#define Addr_G_nRST 0x000
#define Addr_Self_Test 0x004

#define Addr_EPROM_Control 0x0D0
#define CMD_EPROM_Disable 0x00000001
#define CMD_EPROM_Enable 0x00000000
#define CMD_EPROM_Write 0x00010001
#define CMD_EPROM_Read 0x00010002
#define CMD_EPROM_Addr 0x00020000
#define CMD_EPROM_Data 0x00030000

#define Addr_AI_Command 0x100
#define CMD_AI_nRST 0x00000000
#define CMD_AI_Work_Enable 0x00010001
#define CMD_AI_Work_Disable 0x00010000

#define Addr_AI_Channel 0x104
#define Addr_AI_Timing 0x124

#define Addr_AI_Trigger 0x160
#define CMD_AI_TrigSource 0x00000000
#define CMD_AI_nTirg 0x00010000

#define Addr_ReadytoWrite 0x02C

// Flash Changed
/*
double AI_Calibrate_Data[9][7] = {-9.0, -6.0, -3.0, 0.0, 3.0, 6.0, 9.0,
                                  -9.16379, -6.19372, -3.22463, -0.25539, 2.71274, 5.68131, 8.64949,
                                  -8.97283, -5.99162, -3.01108, -0.02998, 2.95005, 5.93018, 8.90996,
                                  -9.13472, -6.12708, -3.12038, -0.11336, 2.89242, 5.89874, 8.90447,
                                  -9.00076, -6.01461, -3.02916, -0.04388, 2.94032, 5.92521, 8.90998,
                                  -9.16198, -6.16623, -3.17073, -0.17571, 2.81841, 5.81320, 8.80776,
                                  -9.04757, -6.06513, -3.08317, -0.10115, 2.87999, 5.86143, 8.84297,
                                  -9.05817, -6.07880, -3.09972, -0.12077, 2.85711, 5.83573, 8.81360,
                                  -8.87876, -5.90814, -2.93809, 0.031804, 3.00050, 5.96997, 8.93871};
*/

static uintptr_t const dma_AddrHandle = 0xb1000000;
static uintptr_t const adc_Handle = 0xb0010000;

/**
 * @brief 查询硬件FIFO是否准备好
 * 
 * @param time 指定的等待时间，单位是操作系统时钟节拍（OS Tick），不可为负（即永久等待）
 * @retval RT_EOK FIFO已准备好
 * @retval -RT_ETIMEOUT 超时FIFO仍未准备好
 * @retval -RT_EINVAL 无效的超时时间
 */
static rt_err_t ADC_Query_FIFO_Ready(rt_int32_t time)
{
    rt_tick_t tick;
    rt_bool_t isReady;

    if (time < 0)
        return -RT_EINVAL;

    isReady = RT_FALSE;
    tick = rt_tick_get();
    do {
        if(Xil_In32(adc_Handle + Addr_AI_FIFO_Status*4) > 0x80000100)
        {
            isReady = RT_TRUE;
            break;
        }
    }
    while (rt_tick_get() - tick < time);

    if (isReady == RT_TRUE)
        return RT_EOK;
    else
        return -RT_ETIMEOUT;
}

/**
 * @brief 将BRAM中读取的原始数据转化为浮点数据
 * 
 * @param bData 输入原始数据缓存区
 * @param fData 输出浮点数据缓存区
 * @param size 数据缓存区的大小
 * @retval RT_EOK 转化成功
 * @retval -RT_EINVAL 输入缓存区不正确
 */
static rt_err_t ADC_Bdata_To_Float(uint32_t *bData, double *fData, size_t size)
{
    int Data_temp;
    double Float_Data_temp;

    if (!bData || !fData)
        return -RT_EINVAL;

    for (int i = 0; i < size; i++)
    {
        if (bData[i] < 131072)
            Data_temp = bData[i];
        else
            Data_temp = -(262143 - bData[i]);

        Float_Data_temp = Data_temp * 38.15 / 1000000;
        Float_Data_temp = (5.0 - Float_Data_temp / 0.1818) / 2.0;
        fData[i] = Float_Data_temp;
    }

    return RT_EOK;
}

/**
 * @brief 从块状Float数据中获取某个通道的数据
 * 
 * @param fData 块状Float数据缓存区
 * @param fSize 块状Float数据缓存区大小（不小于256）
 * @param chData 通道数据缓存区
 * @param chSize 通道数据缓存区大小（不小于32）
 * @param channel 获取数据的通道号（0~7）
 * @retval RT_EOK 获取成功
 * @retval -RT_EINVAL 输入参数不正确
 */
static rt_err_t ADC_Get_Channel_Float(double *fData, size_t fSize, double *chData, size_t chSize, uint32_t channel)
{
    if (fData == NULL || fSize < 256 || chData == NULL || chSize < 32 || channel >= 8)
        return -RT_EINVAL;

    for (int i = 0; i < 32; i++)
        chData[i] = fData[i * 8 + channel];

    return RT_EOK;
}

/**
 * @brief 获取数据的平均值
 * 
 * @param chData 数据缓存区
 * @param chSize 数据缓存区大小
 * @param averData 数据的平均值
 * @retval RT_EOK 获取成功
 * @retval -RT_EINVAL 输入参数不正确
 */
static rt_err_t ADC_Get_Average_Channel_Data(double *chData, size_t chSize, double *averData)
{
    double sum = 0.0;
    if (chData == NULL || averData == NULL)
        return -RT_EINVAL;

    for (int i = 0; i < chSize; i++)
        sum += chData[i];
    *averData = sum / chSize;

    return RT_EOK;
}

/**
 * @brief 设置BRAM ReadytoWrite
 * 
 */
static void ADC_Config_BRAM()
{
    Xil_Out32(adc_Handle + Addr_ReadytoWrite * 4, 0x1); //ReadytoWrite
}

/**
 * @brief 复位ADC模块
 * 
 * @return rt_err_t 返回RT_EOK
 */
rt_err_t ADC_Reset()
{
    uint32_t Addr_Reg_Write;
    uint32_t Data_Reg_Write;

    Addr_Reg_Write = Addr_G_nRST;
    Data_Reg_Write = 0;

    Xil_Out32(adc_Handle + Addr_Reg_Write*4, Data_Reg_Write);//G_nRST

    return RT_EOK;
}

/**
 * @brief 设置通道模式（input mode SI）
 * 
 * @param chan 通道号（0~7）
 * @retval RT_EOK 成功
 * @retval -RT_EINVAL 通道不存在
 */
rt_err_t ADC_Config_Channel_Mode(uint32_t chan)
{
    uint32_t Addr_Reg_Write, Data_Reg_Write;

    if (chan >= 8)
        return -RT_EINVAL;
    Addr_Reg_Write = Addr_AI_Channel;
    //Data_Reg_Write	=	0x40000000 + i*0x10000;
    Data_Reg_Write = 0x50000033 + chan * 0x10000;
    Xil_Out32(adc_Handle + Addr_Reg_Write * 4, Data_Reg_Write); //input mode SI
    return RT_EOK;
}

/**
 * @brief 设置通道采样范围
 * 
 * @param chan 通道号（0~7）
 * @param Range_Value 通道范围枚举值
 * @retval RT_EOK 成功
 * @retval -RT_EINVAL 通道不存在
 */
rt_err_t ADC_Config_Channel_Range(uint32_t chan, enum adc_range Range_Value)
{
    uint32_t Addr_Reg_Write, Data_Reg_Write;

    if (chan >= 8)
        return -RT_EINVAL;
    Addr_Reg_Write = Addr_AI_Channel;
    Data_Reg_Write = 0x30000000 + chan * 0x10000 + Range_Value;
    Xil_Out32(adc_Handle + Addr_Reg_Write * 4, Data_Reg_Write);
    return RT_EOK;
}

/**
 * @brief 设置采样频率
 * 
 * @param Sample_Clock 采样频率（必须为50M的分频数）
 * @return rt_err_t 返回RT_EOK
 */
rt_err_t ADC_Config_Sample_Clock(uint32_t Sample_Clock)
{
    uint32_t Addr_Reg_Write, Data_Reg_Write;
    Addr_Reg_Write = Addr_AI_Timing;
    Data_Reg_Write = 50000000 / Sample_Clock;                   //ADC Sample clock = 50000000/Data_Reg_Write
    Xil_Out32(adc_Handle + Addr_Reg_Write * 4, Data_Reg_Write); //AI_SampleClk_Div
    return RT_EOK;
}

/**
 * @brief 启动ADC采样
 * 
 */
void ADC_Acq_Start()
{
    uint32_t Addr_Reg_Write, Data_Reg_Write;

    //config_Work_EN
    Addr_Reg_Write = Addr_AI_Command;
    Data_Reg_Write = CMD_AI_Work_Enable;
    Xil_Out32(adc_Handle + Addr_Reg_Write * 4, Data_Reg_Write); //Work_EN

    //config_Trig
    Addr_Reg_Write = Addr_AI_Trigger;
    Data_Reg_Write = CMD_AI_nTirg;
    Xil_Out32(adc_Handle + Addr_Reg_Write * 4, Data_Reg_Write); //AI_nTirg

    //BRAM ReadytoWrite
    ADC_Config_BRAM();
}

/**
 * @brief 停止ADC采样
 * 
 */
void ADC_Acq_Stop()
{
    uint32_t Addr_Reg_Write, Data_Reg_Write;
    Addr_Reg_Write = Addr_AI_Command;
    Data_Reg_Write = CMD_AI_Work_Disable;
    Xil_Out32(adc_Handle + Addr_Reg_Write * 4, Data_Reg_Write); //Work_Disable
}

/**
 * @brief 读取BRAM数据
 * 
 *        启动采集后可直接使用此函数读取采样数据
 * 
 * @param bData 读取数据的缓存区
 * @param size 缓存区的大小（不小于256）
 * @retval RT_EOK 读取成功
 * @retval -RT_EINVAL 传入缓存区不正确
 * @retval -RT_ETIMEOUT 超时FIFO仍未准备好
 */
rt_err_t ADC_Read_BRAM_Data(uint32_t *bData, size_t size)
{
    rt_err_t ret = RT_EOK;
    uint32_t Addr_BRAM_Data;
    uint64_t data64;

    if (bData == NULL || size < 256)
        return -RT_EINVAL;

    ret = ADC_Query_FIFO_Ready(10);
    if (ret != RT_EOK)
        return ret;

    size = 256;
    for (int i = 0; i < size; i++)
    {
        Addr_BRAM_Data = i;
        data64 = Xil_In64(dma_AddrHandle + Addr_BRAM_Data * 8);
        bData[i] = 0xFFFFFFFF & data64;
    }
    ADC_Config_BRAM();

    return ret;
}

/**
 * @brief 读取各个通道数据的平均值
 * 
 *        启动采集后可直接使用此函数读取采样数据
 * 
 * @param result 输出8个通道数据平均值的缓存区
 * @param size 缓存区大小（不小于8）
 * @retval RT_EOK 成功
 * @retval -RT_EINVAL 输入参数错误
 * @retval -RT_ENOMEM 内存不足
 * @retval -RT_ETIMEOUT 超时FIFO仍未准备好
 */
rt_err_t ADC_Read_Average_Data(double *result, size_t size)
{
    rt_err_t ret = RT_EOK;
    uint32_t *bData = NULL;
    double *fData = NULL;
    double *chData = NULL;
    uint32_t ch;

    if (!result || size < 8)
    {
        ret = -RT_EINVAL;
        goto _exit;
    }
    // 获取BRAM数据
    bData = (uint32_t *)rt_malloc(256 * sizeof(uint32_t));
    if (!bData)
    {
        ret = -RT_ENOMEM;
        goto _exit;
    }
    ret = ADC_Read_BRAM_Data(bData, 256);
    if (ret != RT_EOK) goto err;

    // 将BRAM数据转化为浮点数据
    fData = (double *)rt_malloc(256 * sizeof(double));
    if (!fData)
    {
        ret = -RT_ENOMEM;
        goto err;
    }
    ADC_Bdata_To_Float(bData, fData, 256);

    // 获取各通道数据的平均值
    chData = (double *)rt_malloc(32 * sizeof(double));
    if (!chData)
    {
        ret = -RT_ENOMEM;
        goto err_1;
    }
    for (ch = 0; ch < 8; ch++)
    {
        ADC_Get_Channel_Float(fData, 256, chData, 32, ch);
        ADC_Get_Average_Channel_Data(chData, 32, &result[ch]);
    }
    ret = RT_EOK;

    rt_free(chData);
err_1:
    rt_free(fData);
err:
    rt_free(bData);
_exit:
    return ret;
}

#include <stdio.h>
void ADC_Test()
{
    double *adValues = (double *)rt_malloc(8 * sizeof(double));
    int channleNum = 8;
    ADC_Reset();
    for (int ch = 0; ch < channleNum; ch++)
        ADC_Config_Channel_Mode(ch);
    for (int ch = 0; ch < channleNum; ch++)
        ADC_Config_Channel_Range(ch, ADC_RANGE_10V);
    ADC_Config_Sample_Clock(100000);
    ADC_Acq_Start();
    ADC_Read_Average_Data(adValues, 8);
    ADC_Acq_Stop();

    printf("ad channel values:\n");
    for (int ch = 0; ch < channleNum; ch++)
    {
        printf("%.3lf", adValues[ch]);
        if (ch != channleNum-1) printf(" ");
    }
    printf("\n");
    rt_free(adValues);
}
MSH_CMD_EXPORT(ADC_Test, ADC_Test: read ADC values);