#include <rtthread.h>
#include "xil_io.h"
#include "drv_rio_1553.h"

// 1553模块基地址，根据插槽位置定义
static uintptr_t const M1553_Handle = 0xb0010000;

static int M1553_Write_Reg(uint32_t chan, uint32_t addres, uint32_t wdata);
static int M1553_Write_Ram(uint32_t chan, uint32_t addres, uint32_t wdata);
static int M1553_Read_Reg(uint32_t chan, uint32_t addres, uint32_t *rdata);
static int M1553_Read_Ram(uint32_t chan, uint32_t addres, uint32_t *rdata);

int M1553_Reset(uint32_t chan)
{
    uint32_t Addr_Reg_Write, Data_Reg_Write;

    if (chan == 0)
        Addr_Reg_Write = 0x1025; //1553 前2个通道复位
    else if (chan == 1)
        Addr_Reg_Write = 0x1035; //1553 后2个通道复位

    Data_Reg_Write = 0;
    Xil_Out32(M1553_Handle + Addr_Reg_Write * 4, Data_Reg_Write);
    Data_Reg_Write = 1;
    Xil_Out32(M1553_Handle + Addr_Reg_Write * 4, Data_Reg_Write);

    M1553_Write_Reg(chan, 0x0c, 1); //reset IP
    M1553_Write_Reg(chan, 0x0c, 0); //reset IP

    return 0;
}

int M1553_BC_SetMessageCount(uint32_t chan, uint32_t MsgCount, uint32_t timeval[16])
{
    M1553_Write_Ram(chan, 0x00, 0x0);
    M1553_Write_Ram(chan, 0x04, 0x0);
    M1553_Write_Ram(chan, 0x08, timeval[0]);
    M1553_Write_Ram(chan, 0x0c, 0x108);
    M1553_Write_Ram(chan, 0x10, 0x0);
    M1553_Write_Ram(chan, 0x14, 0x0);
    M1553_Write_Ram(chan, 0x18, timeval[1]);
    M1553_Write_Ram(chan, 0x1c, 0x130);
    M1553_Write_Ram(chan, 0x20, 0x0);
    M1553_Write_Ram(chan, 0x24, 0x0);
    M1553_Write_Ram(chan, 0x28, timeval[2]);
    M1553_Write_Ram(chan, 0x2c, 0x158);
    M1553_Write_Ram(chan, 0x30, 0x0);
    M1553_Write_Ram(chan, 0x34, 0x0);
    M1553_Write_Ram(chan, 0x38, timeval[3]);
    M1553_Write_Ram(chan, 0x3c, 0x180);
    M1553_Write_Ram(chan, 0x40, 0x0);
    M1553_Write_Ram(chan, 0x44, 0x0);
    M1553_Write_Ram(chan, 0x48, timeval[4]);
    M1553_Write_Ram(chan, 0x4c, 0x1a8);
    M1553_Write_Ram(chan, 0x50, 0x0);
    M1553_Write_Ram(chan, 0x54, 0x0);
    M1553_Write_Ram(chan, 0x58, timeval[5]);
    M1553_Write_Ram(chan, 0x5c, 0x1d0);
    M1553_Write_Ram(chan, 0x60, 0x0);
    M1553_Write_Ram(chan, 0x64, 0x0);
    M1553_Write_Ram(chan, 0x68, timeval[6]);
    M1553_Write_Ram(chan, 0x6c, 0x1f8);

    ///以上为28个初始化数据///////////////////////////////////////////

    M1553_Write_Ram(chan, 0x400, 0x0);                          //设置消息启始地址
    M1553_Write_Ram(chan, 0x404, (0xffff - (MsgCount & 0xff))); //设置消息个数  0xfffd 为2

    return 0;
}

int M1553_BC_WriteData(uint32_t chan, uint32_t MsgNumber, uint32_t data[40], uint32_t length)
{
    uint32_t Addr_Local, i;

    //下面写入第一个消息的命令字、控制字、数据
    // (1)后面是BC->RT7 的控制字，命令字(0x38E0)和数据。 是帧的第一个消息

    Addr_Local = 0x420 + (0xa0 * MsgNumber);

    for (i = 0; i < length; i++)
    {
        M1553_Write_Ram(chan, (Addr_Local + (i * 4)), data[i]);
    }
    return 0;
}

int M1553_BC_Start(uint32_t chan, uint32_t repeat_policy)
{
    M1553_Write_Reg(chan, 0x04, repeat_policy); //设置间接高位地址到地址 0x0 启始
    M1553_Write_Reg(chan, 0x00, 0xfff7);        //设置开放帧结束位

    M1553_Write_Reg(chan, 0x0c, 0x2); //开始BC工作

    return 0;
}

int M1553_BC_Stop(uint32_t chan)
{
    M1553_Write_Reg(chan, 0x0c, 0x4); //停止BC工作并清除中断
    return 0;
}

int M1553_BC_ReceData(uint32_t chan, uint32_t MsgNumber, uint32_t reclenth, uint32_t data[40])
{
    uint32_t Data_BRAM_Read, i;

    M1553_Read_Ram(chan, ((MsgNumber * 0x10) + 0x0), &Data_BRAM_Read);
    data[0] = Data_BRAM_Read;
    M1553_Read_Ram(chan, ((MsgNumber * 0x10) + 0x04), &Data_BRAM_Read);
    data[1] = Data_BRAM_Read;

    for (i = 2; i < reclenth; i++)
    {
        M1553_Read_Ram(chan, (0x420 + (MsgNumber * 0xa0) + (i - 2) * 4), &Data_BRAM_Read);
        data[i] = Data_BRAM_Read;
    }
    for (i = reclenth; i < 40; i++)
    {
        data[i] = 0;
    }
    return 0;
}

int M1553_IRQ_Status(uint32_t chan, uint32_t *irq_sta)
{
    uint32_t Addr_Reg_Read;

    if (chan == 0)
        Addr_Reg_Read = 0x1041;
    else if (chan == 1)
        Addr_Reg_Read = 0x1051;

    //*irq_sta	=	*((u32*)M1553_Handle + Addr_Reg_Read);
    *irq_sta = Xil_In32(M1553_Handle + Addr_Reg_Read * 4);

    return 0;
}

int M1553_Clear_IRQ(uint32_t chan, uint32_t Mode)
{
    if (Mode == 0)
        M1553_Write_Reg(chan, 0x0c, 4); //BC模式清除中断寄存器状态
    else if (Mode >= 1)
        M1553_Write_Reg(chan, 0x0c, 6); //RT或BM模式清除中断寄存器状态

    return 0;
}

//// RT 函数 /////////////////////////

int M1553_RT_START(uint32_t chan, uint32_t RT_ADDRESS, uint32_t datanum, uint32_t data[32])
{
    uint32_t V_RT_ADDRE, i; //RT1  RT_ADDRE = RT_ADDRESS<<1; 因此如果设置为RT1,则V_RT_ADDRE应设置为2

    V_RT_ADDRE = 1 << (RT_ADDRESS & 0x1f);
    V_RT_ADDRE = V_RT_ADDRE | 0x0640; //RT 地址
    M1553_Write_Reg(0, 0x24, V_RT_ADDRE);

    M1553_Write_Ram(chan, 0x400, 0x0);
    for (i = 0; i < 32; i++)
    {
        M1553_Write_Ram(chan, (0x500 + (i * 4)), 0x400);
    }
    for (i = 0; i < 32; i++)
    {
        M1553_Write_Ram(chan, (0x580 + (i * 4)), 0x420);
    }
    for (i = 0; i < 32; i++)
    {
        M1553_Write_Ram(chan, (0x600 + (i * 4)), 0x400);
    }

    for (i = 0; i < 32; i++)
    {
        M1553_Write_Ram(chan, ((0x420 + i) * 4), data[i]); //写入RT待发送的数据
    }

    M1553_Write_Reg(chan, 0x04, 0x8000);
    M1553_Write_Reg(chan, 0x0, 0xfffe);

    //这里应该启动一个RT等待数据进程，进程内容参见文档
    return 0;
}

int M1553_RT_STOP(uint32_t chan)
{
    M1553_Write_Reg(chan, 0x04, 0x0);
    M1553_Write_Reg(chan, 0x0, 0xffff);
    return 0;
}

int M1553_RT_ReceData(uint32_t chan, uint32_t data[40], uint32_t *reclenth)
{
    uint32_t i, j, temp;
    uint32_t V_STACK_OFFSET, V_BLOCK_STAT, V_COMMAND_WORD, V_WORD_COUNT, V_TIME_TAG, V_STATUS, V_DATA;
    //uint32_t Result_Data[40];

    j = 0;

    M1553_Read_Reg(chan, 0x0c, &temp);
    V_STACK_OFFSET = temp - 4;
    V_STACK_OFFSET = V_STACK_OFFSET & 0xffff;

    M1553_Read_Ram(chan, V_STACK_OFFSET * 4, &V_BLOCK_STAT);
    M1553_Read_Ram(chan, (V_STACK_OFFSET + 3) * 4, &V_COMMAND_WORD);
    data[j] = 0x0;
    j++;
    data[j] = 0xffff;
    j++;
    data[j] = 0xffff;
    j++;
    data[j] = 0x0;
    j++;

    data[j] = V_COMMAND_WORD;
    j++;
    data[j] = V_BLOCK_STAT;
    j++;
    V_WORD_COUNT = V_COMMAND_WORD & 0x1f;
    if (V_WORD_COUNT == 0)
        V_WORD_COUNT = 32;

    M1553_Read_Ram(chan, (V_STACK_OFFSET + 1) * 4, &V_TIME_TAG); //获得消息的时间标签，表示每个RT接收到的消息的时间位置

    if ((V_BLOCK_STAT & 0x8000) && !(V_BLOCK_STAT & 0x1000) && !(V_BLOCK_STAT & 0x200))
    {
        if ((V_COMMAND_WORD & 0x400) == 0x0)
        { // 如果BC本个命令是要求RT接收数据
            for (i = 0; i < V_WORD_COUNT; i++)
            {
                M1553_Read_Ram(chan, (0x400 + i) * 4, &temp);
                data[j] = temp;
                j++;
            }
        }
        else
        { //如果BC本个命令是要求RT发送数据, 这里打算用其它函数跟新RT待发送的数据，在调用RT更新发送数据函数前，RT发送数据不改变

            /*M1553_Read_Reg (chan, 0x30, &V_STATUS);
            temp = (V_STATUS|0x8);
            M1553_Write_Reg (chan, 0x30, temp);	//set the busy bit

            for (i=0;i<32;i++) {      //更新RT发送数据
                V_DATA = i;
                M1553_Write_Ram (chan, ((0x420+i)*4), V_DATA);	//写入RT待发送的数据
                }

            M1553_Write_Reg (chan, 0x30, 0);	//clear the busy bit	*/
        }
    } //结束接收/发送处理程序
    *reclenth = j;
    return 0;
}

int M1553_RT_UpData(uint32_t chan, uint32_t datanum, uint32_t data[32])
{
    uint32_t V_STATUS, temp, i;

    M1553_Read_Reg(chan, 0x30, &V_STATUS);
    temp = (V_STATUS | 0x8);
    M1553_Write_Reg(chan, 0x30, temp); //set the busy bit

    for (i = 0; i < datanum; i++)
    {                                                      //更新RT发送数据
        M1553_Write_Ram(chan, ((0x420 + i) * 4), data[i]); //写入RT待发送的数据
    }

    M1553_Write_Reg(chan, 0x30, 0); //clear the busy bit
    return 0;
}

///1553 前2个通道控制函数//////////////////////////////////////////////////////////
static int M1553_Write_Reg(uint32_t chan, uint32_t addres, uint32_t wdata)
{
    unsigned int addr, data;

    addr = 0x1021 + chan * 0x10; //addre
    data = addres;
    Xil_Out32(M1553_Handle + addr * 4, data);

    addr = 0x1022 + chan * 0x10; //csn
    data = 1;
    Xil_Out32(M1553_Handle + addr * 4, data);

    addr = 0x1020 + chan * 0x10; //data
    data = wdata;
    Xil_Out32(M1553_Handle + addr * 4, data);

    addr = 0x1022 + chan * 0x10; //csn
    data = 0;
    Xil_Out32(M1553_Handle + addr * 4, data);

    return 0;
}

static int M1553_Write_Ram(uint32_t chan, uint32_t addres, uint32_t wdata)
{
    unsigned int addr, data;

    addr = 0x1021 + chan * 0x10; //addre
    data = 0x4000 + addres;
    Xil_Out32(M1553_Handle + addr * 4, data);

    addr = 0x1022 + chan * 0x10; //csn
    data = 1;
    Xil_Out32(M1553_Handle + addr * 4, data);

    addr = 0x1020 + chan * 0x10; //data
    data = wdata;
    Xil_Out32(M1553_Handle + addr * 4, data);

    addr = 0x1022 + chan * 0x10; //csn
    data = 0;
    Xil_Out32(M1553_Handle + addr * 4, data);

    return 0;
}

static int M1553_Read_Reg(uint32_t chan, uint32_t addres, uint32_t *rdata)
{
    unsigned int addr, data;

    addr = 0x1021 + chan * 0x10; //addre
    data = addres;
    Xil_Out32(M1553_Handle + addr * 4, data);

    addr = 0x1022 + chan * 0x10; //csn
    data = 1;
    Xil_Out32(M1553_Handle + addr * 4, data);

    addr = 0x1023 + chan * 0x10; //oen
    data = 1;
    Xil_Out32(M1553_Handle + addr * 4, data);

    addr = 0x1040 + chan * 0x10;
    *rdata = Xil_In32(M1553_Handle + addr * 4); //read Self_Test_Reg

    addr = 0x1023 + chan * 0x10; //oen
    data = 0;
    Xil_Out32(M1553_Handle + addr * 4, data);

    addr = 0x1022 + chan * 0x10; //csn
    data = 0;
    Xil_Out32(M1553_Handle + addr * 4, data);

    return 0;
}

static int M1553_Read_Ram(uint32_t chan, uint32_t addres, uint32_t *rdata)
{
    unsigned int addr, data;

    addr = 0x1021 + chan * 0x10; //addre
    data = 0x4000 + addres;
    Xil_Out32(M1553_Handle + addr * 4, data);

    addr = 0x1022 + chan * 0x10; //csn
    data = 1;
    Xil_Out32(M1553_Handle + addr * 4, data);

    addr = 0x1023 + chan * 0x10; //oen
    data = 1;
    Xil_Out32(M1553_Handle + addr * 4, data);

    addr = 0x1040 + chan * 0x10;
    *rdata = Xil_In32(M1553_Handle + addr * 4); //read Self_Test_Reg

    addr = 0x1023 + chan * 0x10; //oen
    data = 0;
    Xil_Out32(M1553_Handle + addr * 4, data);

    addr = 0x1022 + chan * 0x10; //csn
    data = 0;
    Xil_Out32(M1553_Handle + addr * 4, data);

    return 0;
}