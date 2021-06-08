#include <rtthread.h>
#include "xil_io.h"
#include "xaxicdma.h"
#include "xil_cache.h"
#include "drv_rio_adc.h"

#define DBG_TAG "drv.rio.adc"
#define DBG_LVL DBG_LOG
#include "rtdbg.h"

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
#define Addr_AI_Channelsel 0x110
#define Addr_AI_Timing 0x124

#define Addr_AI_Trigger 0x160
#define CMD_AI_TrigSource 0x00000000
#define CMD_AI_nTirg 0x00010000

#define Addr_Ram_Sample_Num 0x024
#define Addr_ReadytoWrite 0x02C

#define CDMA_BUF_SIZE 4096  // 接收cdma传输的内存大小
#define RESET_LOOP_COUNT 10

// ADC模块基地址，根据插槽位置定义
static uintptr_t const adc_Handle = 0xb0020000;
// ADC模块DMA传输源地址，根据插槽位置定义
static uintptr_t const adc_Sorce_Addr = 0x81000000;

static XAxiCdma AxiCdmaInstance; /* Instance of the XAxiCdma */
volatile static rt_uint8_t CDMABuffer[CDMA_BUF_SIZE] __attribute__ ((aligned (64)));

double SAICalArray[6][7] = {  ////////电压校准AMC4324D、AMC4321D、AMC4340、AMC4321C//20130808
        {-0.018, -0.012, -0.006, 0.0, 0.006, 0.012, 0.018},
        {-0.09, -0.06, -0.03, 0.0, 0.03, 0.06, 0.09},
        {-0.9, -0.6, -0.3, 0.0, 0.3, 0.6, 0.9},
        {-1.8, -1.2, -0.6, 0.0, 0.6, 1.2, 1.8},
        {-4.5, -3.0, -1.5, 0.0, 1.5, 3.0, 4.5},
        {-9.0, -6.0, -3.0, 0.0, 3.0, 6.0, 9.0}
};

union
{
    char C_Data[4];
    float F_Data;
} DataFC;

/**
 * @brief IIC写
 * 
 * @param Addr 16bits地址
 * @param data_write 8bits数据
 * @return int 0
 */
static int IIC_Write(int Addr, int data_write)
{
    Xil_Out32(adc_Handle + Addr_EPROM_Control * 4, CMD_EPROM_Disable);           //IIC_Disable
    Xil_Out32(adc_Handle + Addr_EPROM_Control * 4, CMD_EPROM_Addr + Addr);       //IIC addr
    Xil_Out32(adc_Handle + Addr_EPROM_Control * 4, CMD_EPROM_Data + data_write); //IIC data2write
    Xil_Out32(adc_Handle + Addr_EPROM_Control * 4, CMD_EPROM_Write);             //IIC mode:write
    Xil_Out32(adc_Handle + Addr_EPROM_Control * 4, CMD_EPROM_Enable);            //Trig IIC work
    return 0;
}

/**
 * @brief IIC读
 * 
 * @param Addr 16bits地址
 * @param data_read 8bits数据
 * @return int 成功0, 失败-1
 */
static int IIC_Read(int Addr, int *data_read)
{
    int CT = 0;
    int IIC_Status;
    Xil_Out32(adc_Handle + Addr_EPROM_Control * 4, CMD_EPROM_Disable);     //IIC_Disable
    Xil_Out32(adc_Handle + Addr_EPROM_Control * 4, CMD_EPROM_Addr + Addr); //IIC addr
    Xil_Out32(adc_Handle + Addr_EPROM_Control * 4, CMD_EPROM_Read);        //IIC mode:read
    Xil_Out32(adc_Handle + Addr_EPROM_Control * 4, CMD_EPROM_Enable);      //Trig IIC work
    while (1)
    {
        CT = CT + 1;
        IIC_Status = Xil_In32(adc_Handle + Addr_EPROM_ACK * 4);
        if (IIC_Status == 1)
            break;
        if (CT == 1000000)
        {
            *data_read = 0x12345678;
            return -1;
            break;
        }
    }
    *data_read = Xil_In32(adc_Handle + Addr_EPROM_Data * 4);
    return 0;
}

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

        if(Xil_In32(adc_Handle + Addr_AI_FIFO_Status * 4) > 0x80000100)
        {
            isReady = RT_TRUE;
            break;
        }
        rt_hw_us_delay(5);
    } while (rt_tick_get() - tick < time);

    if (isReady == RT_TRUE)
        return RT_EOK;
    else
        return -RT_ETIMEOUT;
}

/**
 * @brief 完成一次ADC的CDMA传输
 * 
 * @param InstancePtr CDMA模块句柄
 * @param Length 传输的数据字节数量（>0）
 * @param Retries 重复次数（>0）
 * @retval RT_EOK 传输成功
 * @retval -RT_EINVAL 传入参数不正确
 * @retval -RT_ENOMEM 传输字节数量过大
 * @retval -RT_ERROR 传输失败
 */
static rt_err_t ADC_CDMA_Transfer(XAxiCdma *InstancePtr, int Length, int Retries)
{
    int Status;
    int Error = 0;

    if (InstancePtr == NULL || Retries <= 0 || Length <= 0) return -RT_EINVAL;
    if (Length > sizeof(CDMABuffer)) return -RT_ENOMEM;

    /* Flush the SrcBuffer before the DMA transfer, in case the Data Cache
     * is enabled
     */
    Xil_DCacheFlushRange((UINTPTR)&CDMABuffer, Length);

    /* Try to start the DMA transfer
     */
    while (Retries) {
        Retries -= 1;

        Status = XAxiCdma_SimpleTransfer(InstancePtr, (UINTPTR)adc_Sorce_Addr,
            (UINTPTR)CDMABuffer, Length, NULL, NULL);
        if (Status == XST_SUCCESS) {
            break;
        }
    }

    /* Return failure if failed to submit the transfer
     */
    if (!Retries) {
        LOG_E("Failed to submit the transfer with %d", Status);
        return -RT_ERROR;
    }


    /* Wait until the DMA transfer is done
     */
    while (XAxiCdma_IsBusy(InstancePtr)) {
        rt_thread_mdelay(1);
    }

    /* If the hardware has errors, this example fails
     * This is a poll example, no interrupt handler is involved.
     * Therefore, error conditions are not cleared by the driver.
     */
    Error = XAxiCdma_GetError(InstancePtr);
    if (Error != 0x0) {
        int TimeOut = RESET_LOOP_COUNT;

        /* Need to reset the hardware to restore to the correct state
         */
        XAxiCdma_Reset(InstancePtr);

        while (TimeOut) {
            if (XAxiCdma_ResetIsDone(InstancePtr)) {
                break;
            }
            TimeOut -= 1;
        }

        /* Reset has failed, print a message to notify the user
         */
        LOG_E("Reset done failed");
        return -RT_ERROR;
    }

    Xil_DCacheInvalidateRange((UINTPTR)CDMABuffer, Length);

    return RT_EOK;
}

/**
 * @brief 根据量程范围返回增益倍数
 * 
 * @param rng 量程范围
 * @return double 增益倍数
 */
static double ADC_Get_GainCoff(enum adc_range rng)
{
    double gain = 1.0;
    switch (rng)
    {
    case ADC_RANGE_10V: gain = 1.0; break;
    case ADC_RANGE_5V: gain = 2.0; break;
    case ADC_RANGE_2V: gain = 5.0; break;
    case ADC_RANGE_1V: gain = 10.0; break;
    case ADC_RANGE_100MV: gain = 100.0; break;
    case ADC_RANGE_10MV: gain = 1000.0; break;
    default: gain = 1.0; break;
    }

    return gain;
}
/**
 * @brief 将BRAM中读取的原始数据转化为浮点数据
 * 
 * @param bData 输入原始数据缓存区
 * @param fData 输出浮点数据缓存区
 * @param size 数据缓存区的大小
 * @param GainCoff 增益倍数
 * @retval RT_EOK 转化成功
 * @retval -RT_EINVAL 输入缓存区不正确
 */
static rt_err_t ADC_Bdata_To_Float(uint32_t *bData, double *fData, size_t size, double GainCoff)
{
    int Data_temp;
    double Float_Data_temp;

    if (!bData || !fData)
        return -RT_EINVAL;

    for (int i = 0; i < size; i++)
    {
        if (bData[i] < 32768)
            Data_temp = bData[i];
        else
            Data_temp = -(65535 - bData[i]);

        Float_Data_temp = Data_temp * 4 * 38.15 / 1000000;
        Float_Data_temp = (5.0 - Float_Data_temp / 0.1818) / 2.0 / GainCoff;
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
    uint32_t Addr_Reg_Write, Data_Reg_Write;

    Addr_Reg_Write = Addr_G_nRST;
    Data_Reg_Write = 0;
    Xil_Out32(adc_Handle + Addr_Reg_Write * 4, Data_Reg_Write);//G_nRST

    return RT_EOK;
}

/**
 * @brief 设置通道选择掩码（bit 1代表选中）
 * 
 * @param mask 通道选择掩码
 */
void ADC_Set_Channel_Mask(rt_uint8_t mask)
{
    uint32_t Addr_Reg_Write, Data_Reg_Write;
    Addr_Reg_Write = Addr_AI_Channelsel;
    Data_Reg_Write = mask;
    Xil_Out32(adc_Handle + Addr_Reg_Write * 4, Data_Reg_Write);
}

/**
 * @brief 设置通道模式（input mode SI）
 * 
 * @param chan 通道号（0~7）
 * @param chan 模式（0~3）
 * @retval RT_EOK 成功
 * @retval -RT_EINVAL 通道不存在
 */
rt_err_t ADC_Config_Channel_Mode(uint32_t chan, uint32_t mode)
{
    uint32_t Addr_Reg_Write, Data_Reg_Write;

    if ((chan >= 8) || (mode >= 4))
        return -RT_EINVAL;
    Addr_Reg_Write = Addr_AI_Channel;
    Data_Reg_Write = 0x40000000 + chan * 0x10000 + mode;
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
 * @brief 设置触发类型
 * 
 * @param type 触发类型
 * @see CMD_AI_TrigSource @see CMD_AI_nTirg
 * @return rt_err_t 返回RT_EOK
 */
rt_err_t ADC_Config_Trig_Type(uint32_t type)
{
    uint32_t Addr_Reg_Write, Data_Reg_Write;
    Addr_Reg_Write = Addr_AI_Trigger;
    Data_Reg_Write = type;
    Xil_Out32(adc_Handle + Addr_Reg_Write * 4, Data_Reg_Write); //AI_nTirg
    return RT_EOK;
}

/**
 * @brief 设置采样点数量
 * 
 * @param num 采样点数量
 * @return rt_err_t 返回RT_EOK
 */
rt_err_t ADC_Config_Sample_Num(uint32_t num)
{
    uint32_t Addr_Reg_Write, Data_Reg_Write;
    Addr_Reg_Write = Addr_Ram_Sample_Num;
    Data_Reg_Write = num; 
    Xil_Out32(adc_Handle + Addr_Reg_Write * 4, Data_Reg_Write);
    return RT_EOK;
}

int ChannelNum = 8;
int CalPointsNum = 7;
int GainNum = 6;
int CalAIRefAddr = 0x2000;
int CalAIBaseAddr = 0; 
/**
 * @brief 将SAICalArray表写入IIC
 * 
 * @return rt_err_t RT_EOK
 */
rt_err_t ADC_Write_Staic_Cal()
{
    int RangAddr = 0, PointAddr = 0, ChanAddr = 0;
    int Addr_IIC;
    int CalMode = 0;
    int j, k, m;
    char temp;
    ChanAddr = ChannelNum * 4;
    PointAddr = CalPointsNum * ChanAddr;
    RangAddr = GainNum * PointAddr;

    for (j = 0; j < GainNum; j++)
    {
        for (k = 0; k < CalPointsNum; k++)
        {
            DataFC.F_Data = (float)SAICalArray[j][k];
            Addr_IIC = CalAIRefAddr + RangAddr * CalMode + PointAddr * j + 4 * k;

            for (m = 0; m < 4; m++)
            {
                rt_thread_mdelay(10);
                temp = DataFC.C_Data[m];
                IIC_Write(Addr_IIC + m, temp);
            }
        }
    }
    return RT_EOK;
}

/**
 * @brief 更新写入校准表
 * 
 * @param calrange 校准范围
 * @param calvalue 校准值
 * @return rt_err_t RT_EOK
 */
rt_err_t ADC_Updata_Cal(int calrange, int calvalue)
{
    int RangAddr = 0, PointAddr = 0, ChanAddr = 0;
    int RangX = 0;
    double GainCoff = 1.0;
    int channleNum = 8;
    double *adValues;
    int i, j, k, m;
    char temp;
    int CalMode = 0;
    int Addr_IIC;

    if (calrange == 0) //10mv
    {
        RangX = 0xe;
        GainCoff = 1000.0;
    }
    else if (calrange == 1) //100mv
    {
        RangX = 0x2;
        GainCoff = 100.0;
    }
    else if (calrange == 2) //1v
    {
        RangX = 0x1;
        GainCoff = 10.0;
    }
    else if (calrange == 3) //2v
    {
        RangX = 0x8;
        GainCoff = 5.0;
    }
    else if (calrange == 4) //5v
    {
        RangX = 0x4;
        GainCoff = 2.0;
    }
    else if (calrange == 5) //10v
    {
        RangX = 0x0;
        GainCoff = 1.0;
    }

    ChanAddr = ChannelNum * 4;
    PointAddr = CalPointsNum * ChanAddr;
    RangAddr = GainNum * PointAddr;

    ADC_Reset();
    for (int ch = 0; ch < channleNum; ch++)
        ADC_Config_Channel_Mode(ch, 0);

    for (int ch = 0; ch < channleNum; ch++)
        ADC_Config_Channel_Range(ch, RangX);

    ADC_Config_Sample_Clock(10000);

    ADC_Acq_Start();

    adValues = (double *)rt_malloc(8 * sizeof(double));
    ADC_Read_Average_Data(adValues, 8, GainCoff);

    ADC_Acq_Stop();

    j = calrange;
    k = calvalue;

    for (i = 0; i < ChannelNum; i++)
    {
        DataFC.F_Data = (float)adValues[i];
        Addr_IIC = CalAIBaseAddr + RangAddr * CalMode + PointAddr * j + ChanAddr * k + 4 * i;
        for (m = 0; m < 4; m++)
        {
            rt_thread_mdelay(1);
            temp = DataFC.C_Data[m];
            IIC_Write(Addr_IIC + m, temp);
        }
        rt_thread_mdelay(1);
    }

    rt_free(adValues);
    
    return RT_EOK;
}

/**
 * @brief 读取校准数据
 * 
 * @param data 接收数据的缓存指针
 * @param size 缓存区大小
 * @retval RT_EOK 读取成功
 * @retval -RT_EINVAL 传入参数错误
 * @retval -RT_ENOMEM 接收缓存区过小
 */
rt_err_t ADC_Read_Cal(float *data, rt_uint32_t size)
{
    int RangAddr = 0, PointAddr = 0, ChanAddr = 0;
    int temp_data = 0;
    int j, k, m, r = 0;
    int Addr_IIC;
    rt_uint32_t count = 0;

    if (data == NULL || size ==0)
        return -RT_EINVAL;

    ChanAddr = ChannelNum * 4;
    PointAddr = CalPointsNum * ChanAddr;
    RangAddr = GainNum * PointAddr;

    for (j = 0; j < GainNum; j++)
    {
        for (k = 0; k < CalPointsNum; k++)
        {
            for (r = 0; r < ChannelNum; r++) //发读真实值的命令
            {
                Addr_IIC = CalAIBaseAddr + PointAddr * j + ChanAddr * k + 4 * r;
                for (m = 0; m < 4; m++)
                {
                    IIC_Read(Addr_IIC + m, &temp_data);
                    DataFC.C_Data[m] = (char)temp_data;
                }
                if (count >= size)
                    return -RT_ENOMEM;
                else
                    data[count++] = DataFC.F_Data;
            }
        }
    }
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
    ADC_Config_Trig_Type(CMD_AI_nTirg);

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
 * @brief 读取板卡温度
 * 
 * @param temper 返回的温度值
 * @return rt_err_t 返回RT_EOK
 */
rt_err_t ADC_Read_Temperature(double *temper)
{
    uint32_t readvalue;
    readvalue = Xil_In32(adc_Handle + Addr_RTD_Temprature * 4);
    *temper = 25.0 + ((readvalue * 0.00059604644775390625) - 106.0) / 0.36;
    return RT_EOK;
}

/**
 * @brief 读取原始数据
 * 
 *        启动采集后可直接使用此函数读取采样数据
 * 
 * @param bData 读取数据的缓存区
 * @param size 缓存区的大小（不小于256）
 * @retval RT_EOK 读取成功
 * @retval -RT_EINVAL 传入缓存区不正确
 * @retval -RT_ERROR 读取失败
 */
rt_err_t ADC_Read_Raw_Data(uint32_t *bData, size_t size)
{
    rt_err_t ret = RT_EOK;
    uint64_t data64;

    if (bData == NULL || size < 256)
        return -RT_EINVAL;

    ret = ADC_Query_FIFO_Ready(10);
    if (ret != RT_EOK)
        return ret;

    size = 256;
    // 启动并完成DMA传输
    ret = ADC_CDMA_Transfer(&AxiCdmaInstance, size*8, 3);
    if (ret != RT_EOK)
        return -RT_ERROR;

    // 传输完成后复制bData
    for (int i = 0; i < size; i++)
    {
        data64 = *((volatile rt_uint64_t *)&CDMABuffer[i*8]);
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
 * @param GainCoff 增益倍数
 * @retval RT_EOK 成功
 * @retval -RT_EINVAL 输入参数错误
 * @retval -RT_ENOMEM 内存不足
 * @retval -RT_ERROR 读取失败
 */
rt_err_t ADC_Read_Average_Data(double *result, size_t size, double GainCoff)
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
    // 获取原始数据
    bData = (uint32_t *)rt_malloc(256 * sizeof(uint32_t));
    if (!bData)
    {
        ret = -RT_ENOMEM;
        goto _exit;
    }
    ret = ADC_Read_Raw_Data(bData, 256);
    if (ret != RT_EOK) goto err;

    // 将BRAM数据转化为浮点数据
    fData = (double *)rt_malloc(256 * sizeof(double));
    if (!fData)
    {
        ret = -RT_ENOMEM;
        goto err;
    }
    ADC_Bdata_To_Float(bData, fData, 256, GainCoff);

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

int ADC_CDMA_Init(void)
{
    XAxiCdma_Config *CfgPtr;
    int Status;

    /* Initialize the XAxiCdma device.
     */
    CfgPtr = XAxiCdma_LookupConfig(XPAR_AXICDMA_0_DEVICE_ID);
    if (!CfgPtr) return -RT_ERROR;

    Status = XAxiCdma_CfgInitialize(&AxiCdmaInstance, CfgPtr,
                                    CfgPtr->BaseAddress);
    if (Status != XST_SUCCESS) return -RT_ERROR;

    /* Disable interrupts, we use polling mode
     */
    XAxiCdma_IntrDisable(&AxiCdmaInstance, XAXICDMA_XR_IRQ_ALL_MASK);

    return RT_EOK;
}

#include <stdio.h>
void ADC_Test()
{
    rt_err_t ret = RT_EOK;
    double *adValues = (double *)rt_malloc(8 * sizeof(double));
    int channleNum = 8;
    enum adc_range rng = ADC_RANGE_10V;

    ret = ADC_Reset();
    if (ret != RT_EOK) goto _exit;
    for (int ch = 0; ch < channleNum; ch++)
    {
        ret = ADC_Config_Channel_Mode(ch, 0);
        if (ret != RT_EOK) goto _exit;
    }
    for (int ch = 0; ch < channleNum; ch++)
    {
        ret = ADC_Config_Channel_Range(ch, rng);
        if (ret != RT_EOK) goto _exit;
    }
    ret = ADC_Config_Sample_Clock(100000);
    if (ret != RT_EOK) goto _exit;

    ADC_Acq_Start();
    ret = ADC_Read_Average_Data(adValues, 8, ADC_Get_GainCoff(rng));
    if (ret != RT_EOK)
    {
        printf("ADC read data fail (%ld)\n", ret);
        goto _exit;
    }
    ADC_Acq_Stop();

    printf("ad channel values:\n");
    for (int ch = 0; ch < channleNum; ch++)
    {
        printf("%.3lf", adValues[ch]);
        if (ch != channleNum-1) printf(" ");
    }
    printf("\n");

_exit:
    rt_free(adValues);
    return;
}
MSH_CMD_EXPORT(ADC_Test, ADC_Test: read ADC values);