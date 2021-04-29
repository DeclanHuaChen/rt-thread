#include "xil_io.h"
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <rtthread.h>

#define MAXLINE 80
#define SERV_PORT 10001
#define REG_WRITE_VALUE 0x5A5A5A5A
#define MEM_MAP_START_ADDR 0xb0010000
#define MEM_MAP_SIZE 0x10000
#define DMA_MAP_START_ADDR 0xb1000000
#define DMA_MAP_SIZE 0x1000000
#define SAMPLE_CONFIG_SUCCESS "192.168.1.100,1"
#define SAMPLE_CONFIG_FAIL "192.168.1.100,-1"

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

//	 Flash Changed
double AI_Calibrate_Data[9][7] = {-9.0, -6.0, -3.0, 0.0, 3.0, 6.0, 9.0,
                                  -9.16379, -6.19372, -3.22463, -0.25539, 2.71274, 5.68131, 8.64949,
                                  -8.97283, -5.99162, -3.01108, -0.02998, 2.95005, 5.93018, 8.90996,
                                  -9.13472, -6.12708, -3.12038, -0.11336, 2.89242, 5.89874, 8.90447,
                                  -9.00076, -6.01461, -3.02916, -0.04388, 2.94032, 5.92521, 8.90998,
                                  -9.16198, -6.16623, -3.17073, -0.17571, 2.81841, 5.81320, 8.80776,
                                  -9.04757, -6.06513, -3.08317, -0.10115, 2.87999, 5.86143, 8.84297,
                                  -9.05817, -6.07880, -3.09972, -0.12077, 2.85711, 5.83573, 8.81360,
                                  -8.87876, -5.90814, -2.93809, 0.031804, 3.00050, 5.96997, 8.93871};

void *dma_AddrHandle = 0xb0010000;
void *g_virtualAddrHandle = 0xb0010000;
void *uart_Handle = 0xb0020000;
void *can_Handle = 0xb0010000;
void *M1553_Handle = 0xb0010000;

int M1553_Write_Reg(uint32_t chan, uint32_t addres, uint32_t wdata);
int M1553_Write_Ram(uint32_t chan, uint32_t addres, uint32_t wdata);
int M1553_Read_Reg(uint32_t chan, uint32_t addres, uint32_t *rdata);
int M1553_Read_Ram(uint32_t chan, uint32_t addres, uint32_t *rdata);

int M1553_Reset(uint32_t chan);
//BC
int M1553_BC_SetMessageCount(uint32_t chan, uint32_t MsgCount, uint32_t timeval[16]);
int M1553_BC_WriteData(uint32_t chan, uint32_t MsgNumber, uint32_t data[40], uint32_t length);
int M1553_BC_Start(uint32_t chan, uint32_t repeat_policy);
int M1553_BC_Stop(uint32_t chan);
int M1553_BC_ReceData(uint32_t chan, uint32_t MsgNumber, uint32_t reclenth, uint32_t data[40]);
int M1553_IRQ_Status(uint32_t chan, uint32_t *irq_sta);
int M1553_Clear_IRQ(uint32_t chan, uint32_t Mode);
//RT
int M1553_RT_START(uint32_t chan, uint32_t RT_ADDRESS, uint32_t datanum, uint32_t data[32]);
int M1553_RT_STOP(uint32_t chan);
int M1553_RT_ReceData(uint32_t chan, uint32_t data[40], uint32_t *reclenth);
int M1553_RT_UpData(uint32_t chan, uint32_t datanum, uint32_t data[32]);

///////////////////////////////////////////////////////////////////////////////
///函数内容//////////////////////////////////////////////////////////////////////////

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
    ///////////////////////////////////////////////
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
    uint32_t Addr_Reg_Read, Data_Reg_Read;

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
    else if (chan == 0)
        Addr_Reg_Read = 0x1051;

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
int M1553_Write_Reg(uint32_t chan, uint32_t addres, uint32_t wdata)
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

int M1553_Write_Ram(uint32_t chan, uint32_t addres, uint32_t wdata)
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

int M1553_Read_Reg(uint32_t chan, uint32_t addres, uint32_t *rdata)
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

int M1553_Read_Ram(uint32_t chan, uint32_t addres, uint32_t *rdata)
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

/***************************
****************************************************
        CAN
****************************************************
****************************/
typedef struct
{
    int Chan;
    int Baud;
    int Arbitrate;
    int SuccessFlag;
    int Mode;
} CAN_CHCONFIG_PARA;

CAN_CHCONFIG_PARA gCanCfgPara = {0, 0, 0, 0, 0};

typedef struct
{
    int Chan;
    int Mask;
    int FrameId;
    int ExtFrameId;
} CAN_FLT_CHCONFIG;

CAN_FLT_CHCONFIG gCanFltCfg = {0, 0, 0, 0};

typedef struct
{
    int Chan;
    int FrameId;
    int ExtFrameId;
    int FrameFormat;
    int FrameType;
    unsigned char *MsgData;
    int MsgLen;
} CAN_MSG_DATA_WR;

CAN_MSG_DATA_WR gCanMsgDataWr = {0, 0, 0, 0, 0, NULL, 0};

typedef struct
{
    int msgNum;
    int frame_id;
    int ext_frame_id;
    int frame_format;
    int frame_type;
    unsigned char *data;
    int data_size;
} ST_CAN_FRAME_DESC;

ST_CAN_FRAME_DESC gStCanFrameDesc;

int CAN_Write_Reg(uint32_t addres, uint32_t wdata);
int CAN_Read_Reg(uint32_t addres, uint32_t *rdata);
int CAN_Reset();

//////////////////////////////////////////////////////////////////////////////
//// CAN 函数/////////////////////////////////////////////////////////////////

#define GROUP0_RX 0x0000 //channel 0 	RX
#define GROUP1_RX 0x0001 //channel 1 	RX
#define GROUP2_RX 0x0002 //channel 2 	RX
#define GROUP3_RX 0x0003 //channel 3 	RX
#define GROUP4_RX 0x0004 //channel 4 	RX
#define GROUP5_RX 0x0005 //channel 5 	RX
#define GROUP6_RX 0x0006 //channel 6		RX
#define GROUP7_RX 0x0007 //channel 7 	RX

#define GROUP0_TX 0x0008 //channel 8 	TX
#define GROUP1_TX 0x0009 //channel 9		TX
#define GROUP2_TX 0x000a //channel 10	TX
#define GROUP3_TX 0x000b //channel 11	TX
#define GROUP4_TX 0x0005 //channel 8 	TX
#define GROUP5_TX 0x0006 //channel 9		TX
#define GROUP6_TX 0x0007 //channel 10	TX
#define GROUP7_TX 0x0008 //channel 11	TX

int CAN_Reset()
{
    uint32_t addr, data;

    addr = 0x2025; //addre
    data = 0;
    Xil_Out32(can_Handle + addr * 4, data);
    addr = 0x2025; //addre
    data = 1;
    Xil_Out32(can_Handle + addr * 4, data);

    return 0;
}

int CAN_Write_Reg(uint32_t addres, uint32_t wdata)
{
    uint32_t addr, data;

    addr = 0x2021; //addre
    data = addres;
    Xil_Out32(can_Handle + addr * 4, data);

    addr = 0x2022; //csn
    data = 1;
    Xil_Out32(can_Handle + addr * 4, data);

    addr = 0x2020; //data
    data = wdata;
    Xil_Out32(can_Handle + addr * 4, data);

    addr = 0x2022; //csn
    data = 0;
    Xil_Out32(can_Handle + addr * 4, data);

    return 0;
}

int CAN_Read_Reg(uint32_t addres, uint32_t *rdata)
{
    uint32_t addr, data;

    addr = 0x2021; //addre
    data = addres;
    Xil_Out32(can_Handle + addr * 4, data);

    addr = 0x2022; //csn
    data = 1;
    Xil_Out32(can_Handle + addr * 4, data);

    addr = 0x2040; //add by xzy
    //*rdata	=	*((u32*)ARTIX_REG_BASE + addr);//read Self_Test_Reg   //add by xzy
    *rdata = Xil_In32(can_Handle + addr * 4);
    addr = 0x2023; //oen
    data = 1;
    Xil_Out32(can_Handle + addr * 4, data);
#if 0
    addr	=	0x2040;
    //*rdata	=	*((u32*)ARTIX_REG_BASE + addr);//read Self_Test_Reg
    *rdata	=	Xil_In32(can_Handle + addr*4);
#endif
    addr = 0x2023; //oen
    data = 0;
    Xil_Out32(can_Handle + addr * 4, data);

    addr = 0x2022; //csn
    data = 0;
    Xil_Out32(can_Handle + addr * 4, data);

    return 0;
}
///////////////////////////////////////
int MsgDataRd(uint32_t msgNum[], uint32_t msgDataPtr[], uint32_t bufferSize, uint32_t COUNTREG, uint32_t BUFFERREG)
{
    int i;
    int empty_cnt = 0, totol_cnt = 0;

    while (1)
    {
        while (1)
        {
            CAN_Read_Reg(COUNTREG, msgNum);

            if (*msgNum == 0)
            {
                empty_cnt++;
                if (empty_cnt > 8)
                    goto ext; //当读9次COUNTREG寄存器的值都为0时认为SDRAM的FIFO已读空
            }
            else
            {
                empty_cnt = 0; //复位为0，重新计次
                break;         //FIFO非空，继续读
            }
        }

        if (totol_cnt + *msgNum > bufferSize)
            *msgNum = bufferSize - totol_cnt;

        for (i = 0; i < *msgNum; i++)
        {
            CAN_Read_Reg(BUFFERREG, msgDataPtr + totol_cnt + i);
        }

        totol_cnt += *msgNum;
        goto ext;
    }

ext:
    *msgNum = totol_cnt;

    return 0;
}

int CAN_MsgDataRd(uint32_t channel, uint32_t *msgNum, uint32_t *frame_id, uint32_t *ext_frame_id, uint32_t *frame_format, uint32_t *frame_type, uint8_t data[], uint32_t *data_size)
{
    uint32_t msgData[4];
    uint8_t x;
    int index = 0;
    uint8_t byte_data[16] = {0};
    int m = 0;
    int i = 0;
    uint8_t tmp;

    *msgNum = 0;

    switch (channel)
    {
    case GROUP0_RX:
        MsgDataRd(msgNum, msgData, 4, 0xc80, 0xca0);
        break;
    case GROUP1_RX:
        MsgDataRd(msgNum, msgData, 4, 0xd80, 0xda0);
        break;
    case GROUP2_RX:
        MsgDataRd(msgNum, msgData, 4, 0xe80, 0xea0);
        break;
    case GROUP3_RX:
        MsgDataRd(msgNum, msgData, 4, 0xf80, 0xfa0);
        break;
    default:
        break;
    }

    if (*msgNum == 4)
    {

        for (index = 0; index < 4; index++)
        {
            byte_data[index * 4] = msgData[index] >> 24;
            byte_data[index * 4 + 1] = msgData[index] >> 16;
            byte_data[index * 4 + 2] = msgData[index] >> 8;
            byte_data[index * 4 + 3] = msgData[index];
        }

        for (index = 0; index < 16; index++)
        {
            m = 0;
            tmp = byte_data[index];
            for (m = 7; m >= 0; m--)
            {
                x = tmp >> m;

                if (((tmp >> m) & 0x01) == 0x00) // first 0 bit
                    break;
            }

            if (m >= 0)
                break;
        }

        if (index < 16)
        {
            *frame_id = 0;
            *frame_id = byte_data[index] & 0x03;
            *frame_id = (*frame_id << 8) | byte_data[index + 1];
            *frame_id = (*frame_id << 1) | ((byte_data[index + 2] >> 7) & 0x01);

            if (m == 2)
                *frame_format = 0;
            else
                *frame_format = 1;

            if (*frame_format == 0) //standard frame
            {
                *frame_type = ((byte_data[index + 2] >> 6) & 0x01);

                *data_size = byte_data[index + 2] & 0x0F;

                for (i = 0; i < *data_size; i++)
                {
                    data[i] = byte_data[index + 3 + i];
                }
            }
            else // extended format
            {
                ///////////////////////////////////////////////////////////////////////////////////
                //宋毅20180827
                *frame_id = 0;
                *frame_id = byte_data[index] & 0x3F;
                *frame_id = (*frame_id << 5) | (byte_data[index + 1] >> 3);
                *ext_frame_id = 0;
                *ext_frame_id = byte_data[index + 1] & 0x01;
                *ext_frame_id = (*ext_frame_id << 8) | byte_data[index + 2];
                *ext_frame_id = (*ext_frame_id << 8) | byte_data[index + 3];
                *ext_frame_id = (*ext_frame_id << 1) | ((byte_data[index + 4] >> 7) & 0x01);

                *frame_type = ((byte_data[index + 4] >> 6) & 0x01);

                *data_size = byte_data[index + 4] & 0x0F;
                ///////////////////////////////////////////////////////////////////////////////////

                for (i = 0; i < *data_size; i++)
                {
                    data[i] = byte_data[index + 5 + i];
                }
            }

            *msgNum = 1;
        }
    }
    else
    {
        *msgNum = 0;
    }

    return 0;
}

int CAN_ChConfig(uint32_t channel, uint32_t Baudrate, uint32_t arbitrate, uint32_t success_flag, uint32_t mode)
{
    uint32_t value = 0;

    switch (channel) //write the reg of each channel
    {
    case GROUP0_TX:
    case GROUP0_RX:
        CAN_Read_Reg(0Xc20, &value);
        value = (value & 0xFF000000) | (Baudrate & 0x00FFFFFF);          // baudrate
        value = (value & 0xBFFFFFFF) | ((mode << 30) & 0x40000000);      // mode		 high, low
        value = (value & 0xFEFFFFFF) | ((arbitrate << 24) & 0x01000000); // arbitrate

        //value = value | 0x01 << 27 ;	// /STB 结束发送
        //value = value | 0x01 << 26 ;	// 重复发送使能

        value = value | 0x01 << 31; // reset

        value = value | (success_flag & 0x01) << 29; // success_flag
        CAN_Write_Reg(0Xc20, value);
        CAN_Read_Reg(0Xc20, &value);
        value = value & 0x7fffffff;
        CAN_Write_Reg(0Xc20, value);
        break;
    case GROUP1_TX:
    case GROUP1_RX:
        CAN_Read_Reg(0Xd20, &value);

        value = (value & 0xFF000000) | (Baudrate & 0x00FFFFFF);          // baudrate
        value = (value & 0xBFFFFFFF) | ((mode << 30) & 0x40000000);      // mode		 high, low
        value = (value & 0xFEFFFFFF) | ((arbitrate << 24) & 0x01000000); // arbitrate

        //value = value | 0x01 << 27 ;	// /STB
        //value = value | 0x01 << 26 ;	// EN

        value = value | 0x01 << 31; // reset

        value = value | (success_flag & 0x01) << 29; // success_flag

        CAN_Write_Reg(0Xd20, value);
        value = value & 0x7fffffff;
        CAN_Write_Reg(0Xd20, value);
        break;
    case GROUP2_TX:
    case GROUP2_RX:
        CAN_Read_Reg(0Xe20, &value);

        value = (value & 0xFF000000) | (Baudrate & 0x00FFFFFF);          // baudrate
        value = (value & 0xBFFFFFFF) | ((mode << 30) & 0x40000000);      // mode		 high, low
        value = (value & 0xFEFFFFFF) | ((arbitrate << 24) & 0x01000000); // arbitrate

        //value = value | 0x01 << 27 ;	// /STB
        //value = value | 0x01 << 26 ;	// EN

        value = value | 0x01 << 31; // reset

        value = value | (success_flag & 0x01) << 29; // success_flag

        CAN_Write_Reg(0Xe20, value);
        value = value & 0x7fffffff;
        CAN_Write_Reg(0Xe20, value);
        break;
    case GROUP3_TX:
    case GROUP3_RX:
        CAN_Read_Reg(0Xf20, &value);

        value = (value & 0xFF000000) | (Baudrate & 0x00FFFFFF);          // baudrate
        value = (value & 0xBFFFFFFF) | ((mode << 30) & 0x40000000);      // mode		 high, low
        value = (value & 0xFEFFFFFF) | ((arbitrate << 24) & 0x01000000); // arbitrate

        //value = value | 0x01 << 27 ;	// /STB
        //value = value | 0x01 << 26 ;	// EN

        value = value | 0x01 << 31; // reset

        value = value | (success_flag & 0x01) << 29; // success_flag

        CAN_Write_Reg(0Xf20, value);
        value = value & 0x7fffffff;
        CAN_Write_Reg(0Xf20, value);
        break;

    default:
        break;
    }

    return 0;
}

int CAN_FilterConfig(uint32_t channel, uint32_t mask, uint32_t frame_id, uint32_t ext_frame_id)
{

    uint32_t value = 0;

    value = (frame_id & 0x000007FF) | (ext_frame_id << 11);

    switch (channel) //write the reg of each channel
    {
    case GROUP0_TX:
    case GROUP0_RX:

        CAN_Write_Reg(0Xc60, value);
        CAN_Write_Reg(0Xce0, mask);

        break;
    case GROUP1_TX:
    case GROUP1_RX:

        CAN_Write_Reg(0Xd60, value);
        CAN_Write_Reg(0Xde0, mask);

        break;
    case GROUP2_TX:
    case GROUP2_RX:

        CAN_Write_Reg(0Xe60, value);
        CAN_Write_Reg(0Xee0, mask);

        break;
    case GROUP3_TX:
    case GROUP3_RX:

        CAN_Write_Reg(0Xf60, value);
        CAN_Write_Reg(0Xfe0, mask);

        break;
    default:
        break;
    }

    return 0;
}

int CAN_MsgDataWr(uint32_t channel, uint32_t frame_id, uint32_t ext_frame_id, uint32_t frame_format, uint32_t frame_type, uint8_t *msgData, uint32_t msg_len)
{
    int index = 0;
    uint8_t outData_Byte[16];
    uint8_t d1, d2, d3, d4, d5;
    uint32_t outData;

    if (frame_type == 1) // remote frame
        msg_len = 0;

    if (msg_len < 0 || msgData == NULL)
        return 0;

    if (frame_format == 0) // standard format
    {
        d1 = 0;
        d1 |= (frame_id & 0x01) << 7;
        d1 |= (frame_type & 0x01) << 6;
        d1 |= (frame_format & 0x01) << 5;
        d1 |= msg_len & 0x0F;

        d2 = 0;
        d2 = frame_id >> 1;

        d3 = 0xF8 | ((frame_id >> 9) & 0x03);

        for (index = 0; index < msg_len; index++)
        {
            outData_Byte[15 - index] = msgData[msg_len - 1 - index];
        }

        outData_Byte[16 - msg_len - 1] = d1;
        outData_Byte[16 - msg_len - 2] = d2;
        outData_Byte[16 - msg_len - 3] = d3;

        for (index = 0; index < 16 - msg_len - 3; index++)
        {
            outData_Byte[index] = 0xFF;
        }
    }
    else // extend format
    {
        d1 = 0;
        d1 |= (ext_frame_id & 0x01) << 7;
        d1 |= (frame_type & 0x01) << 6;
        d1 |= (frame_format & 0x01) << 5;
        d1 |= msg_len & 0x0F;

        d2 = 0;
        d2 = ext_frame_id >> 1;

        d3 = 0;
        d3 = ext_frame_id >> 9;

        d4 = 0;
        d4 = ext_frame_id >> 17;
        d4 = d4 | ((frame_format & 0x01) << 1);
        d4 = d4 | (0x01 << 2);
        d4 = d4 | (frame_id << 3);

        d5 = 0;
        ////////////////////////////////////////////////
        //d5 = 0xB0 | ( frame_id >> 5 ) & 0x1F;   宋毅20180827
        d5 = 0x80 | ((frame_id >> 5) & 0x3f);
        ////////////////////////////////////////////////

        for (index = 0; index < msg_len; index++)
        {
            outData_Byte[15 - index] = msgData[msg_len - 1 - index];
        }

        outData_Byte[16 - msg_len - 1] = d1;
        outData_Byte[16 - msg_len - 2] = d2;
        outData_Byte[16 - msg_len - 3] = d3;
        outData_Byte[16 - msg_len - 4] = d4;
        outData_Byte[16 - msg_len - 5] = d5;

        for (index = 0; index < 16 - msg_len - 5; index++)
        {
            outData_Byte[index] = 0xFF;
        }
    }

    for (index = 0; index < 4; index++)
    {
        outData = 0;
        outData = outData_Byte[index * 4];
        outData = (outData << 8) | outData_Byte[index * 4 + 1];
        outData = (outData << 8) | outData_Byte[index * 4 + 2];
        outData = (outData << 8) | outData_Byte[index * 4 + 3];

        switch (channel) //write the reg of each channel
        {
        case GROUP0_TX:
            CAN_Write_Reg(0xca0, outData);

            break;
        case GROUP1_TX:
            CAN_Write_Reg(0xda0, outData);

            break;
        case GROUP2_TX:
            CAN_Write_Reg(0xea0, outData);

            break;
        case GROUP3_TX:
            CAN_Write_Reg(0xfa0, outData);

            break;
        default:
            break;
        }
    }
    return 0;
}

int CAN_getDataCount(uint32_t channel, uint32_t *count)
{

    *count = 0;

    switch (channel)
    {
    case GROUP0_RX:
        //Group0
        CAN_Read_Reg(0xc80, count);

        break;
    case GROUP1_RX:
        //Group1
        CAN_Read_Reg(0Xd80, count);

        break;
    case GROUP2_RX:
        //Group1
        CAN_Read_Reg(0Xe80, count);

        break;
    case GROUP3_RX:
        //Group1
        CAN_Read_Reg(0Xf80, count);

        break;
    default:
        break;
    }

    *count = *count / 4;

    return 0;
}

int CAN_Start(uint32_t channel)
{
    uint32_t value = 0;
    uint32_t count = 0;

    //加入启动发送前延时
    {
        //#define REG_CAN_GROUP0_RCV_COUNT	  0xC80
        /*CAN_Read_Reg (0xc80, &count);
        CAN_Read_Reg (0xc80, &count);
        CAN_Read_Reg (0xc80, &count);
        CAN_Read_Reg (0xc80, &count);
        CAN_Read_Reg (0xc80, &count);
        CAN_Read_Reg (0xc80, &count);
        CAN_Read_Reg (0xc80, &count);
        CAN_Read_Reg (0xc80, &count);*/
    }

    switch (channel) //write the reg of each channel
    {
    case GROUP0_TX:
    case GROUP0_RX:
        CAN_Read_Reg(0xc20, &value);

        value = value | 0x10000000; // start

        CAN_Write_Reg(0xc20, value);

        break;
    case GROUP1_TX:
    case GROUP1_RX:
        CAN_Read_Reg(0xd20, &value);

        value = value | 0x10000000; // start

        CAN_Write_Reg(0xd20, value);
        break;
    case GROUP2_TX:
    case GROUP2_RX:
        CAN_Read_Reg(0xe20, &value);

        value = value | 0x10000000; // start

        CAN_Write_Reg(0xe20, value);
        break;
    case GROUP3_TX:
    case GROUP3_RX:
        CAN_Read_Reg(0xf20, &value);

        value = value | 0x10000000; // start

        CAN_Write_Reg(0xf20, value);
        break;
    default:
        break;
    }

    return 0;
}
int get_send_stat(uint32_t channel)
{
    uint32_t value = 0;
    uint32_t offset = 0;

    switch (channel)
    {
    case GROUP0_TX:
    case GROUP0_RX:
        offset = 0xC20;

        break;
    case GROUP1_TX:
    case GROUP1_RX:
        offset = 0xD20;

        break;
    case GROUP2_TX:
    case GROUP2_RX:
        offset = 0xE20;

        break;
    case GROUP3_TX:
    case GROUP3_RX:
        offset = 0xF20;

        break;

    default:
        return 3;
        break;
    }

    CAN_Read_Reg(offset, &value);
    printf("get_send_stat value:%d\n", value);
    if (value & (0x1 << 26))
    {
        if (value & (0x1 << 25))
        {
            return 0;
        }
        else
        {
            return 1;
        }
    }
    else
    {
        return 2;
    }

    return 0;
}

/***************************
****************************************************
        uart
****************************************************
****************************/
uint32_t UART_Command_Code;
uint32_t UART_HALF[4] = {0, 0, 0, 0}; //全双工
uint32_t UART_MODE[4] = {1, 1, 1, 1}; //rs422
uint32_t UART_TERM[4] = {0, 0, 0, 0}; //无终端电阻

int UART_Reset();
int UART_TX_Disable(uint32_t channel);
int UART_TX_Enable(uint32_t channel);
int UART_Receive_Data(uint32_t channel, uint32_t amount, uint8_t Chan1_Data[]);
//int UART_Send_Data (uint32_t channel, uint32_t amount, uint8_t Se_Data[]);
int UART_Send_Data(uint32_t channel, uint32_t amount, uint8_t Se_Data[], uint32_t inter_time);
int UART_Query_FIFO_Status(uint32_t channel, uint32_t *Send_Sta, uint32_t *Rec_Sta);
int UART_Stop_Work(uint32_t channel);
int UART_OpenComConfig(uint32_t channel, uint32_t baud, uint32_t parity, uint32_t Data_bit, uint32_t Stop_bit, uint32_t IEEE_Mode, uint32_t Term_Resis, uint32_t Half_Dxp);

int UART_OpenComConfig(uint32_t channel, uint32_t baud, uint32_t parity, uint32_t Data_bit, uint32_t Stop_bit, uint32_t IEEE_Mode, uint32_t Term_Resis, uint32_t Half_Dxp)
{
    uint32_t device_code;
    uint32_t Dupx;
    uint32_t bps, para1, parament;
    uint32_t framecount = 0;
    uint32_t rec_data[2048];
    uint32_t chanencode, i, chanen;
    uint32_t presen[32];
    uint32_t Addr_Reg_Read, Data_Reg_Read, Data_Reg_Readx;
    uint32_t Addr_Reg_Write, Data_Reg_Write;

    UART_Stop_Work(channel);
    //Delay (0.01);
    rt_thread_mdelay(1);

    switch (baud)
    {

    case 4800: //right
        bps = 0xc0;
        break;
    case 7200: //right
        bps = 0x80;
        break;
    case 9600: //right
        bps = 0x60;
        break;
    case 12800: //right
        bps = 0x48;
        break;
    case 19200: //right
        bps = 0x30;
        break;
    case 28800: //right
        bps = 0x20;
        break;
    case 38400: //right
        bps = 0x18;
        break;
    case 51200: //right
        bps = 0x12;
        break;
    case 57600: //right
        bps = 0x10;
        break;
    case 61440: //right
        bps = 0xf;
        break;
    case 76800: //right
        bps = 0xc;
        break;
    case 102400: //right
        bps = 0x9;
        break;
    case 115200: //right
        bps = 0x8;
        break;
    case 153600: //right
        bps = 0x6;
        break;
    case 230400: //right
        bps = 0x4;
        break;
    case 307200: //right
        bps = 0x3;
        break;
    case 460800: //right
        bps = 0x2;
        break;
    case 921600: //right
        bps = 0x1;
        break;

    default:
        return -1;
        break;
    }

    switch (parity)
    {
    case 1: //奇校验
        if ((Data_bit == 8) & (Stop_bit == 1))
            para1 = 0x0b;
        else if ((Data_bit == 8) & (Stop_bit == 2))
            para1 = 0x0f;
        else if ((Data_bit == 7) & (Stop_bit == 1))
            para1 = 0x0a;
        else if ((Data_bit == 7) & (Stop_bit == 2))
            para1 = 0x0e;
        else if ((Data_bit == 6) & (Stop_bit == 1))
            para1 = 0x09;
        else if ((Data_bit == 6) & (Stop_bit == 2))
            para1 = 0x0d;
        else if ((Data_bit == 5) & (Stop_bit == 1))
            para1 = 0x08;
        else if ((Data_bit == 5) & (Stop_bit == 2))
            para1 = 0x0c;
        else
        {
            return -3;
        }
        break;
    case 2: //偶校验
        if ((Data_bit == 8) & (Stop_bit == 1))
            para1 = 0x1b;
        else if ((Data_bit == 8) & (Stop_bit == 2))
            para1 = 0x1f;
        else if ((Data_bit == 7) & (Stop_bit == 1))
            para1 = 0x1a;
        else if ((Data_bit == 7) & (Stop_bit == 2))
            para1 = 0x1e;
        else if ((Data_bit == 6) & (Stop_bit == 1))
            para1 = 0x19;
        else if ((Data_bit == 6) & (Stop_bit == 2))
            para1 = 0x1d;
        else if ((Data_bit == 5) & (Stop_bit == 1))
            para1 = 0x18;
        else if ((Data_bit == 5) & (Stop_bit == 2))
            para1 = 0x1c;
        else
        {
            return -3;
        }
        break;
    case 3: //总为1
        if ((Data_bit == 8) & (Stop_bit == 1))
            para1 = 0x2b;
        else if ((Data_bit == 8) & (Stop_bit == 2))
            para1 = 0x2f;
        else if ((Data_bit == 7) & (Stop_bit == 1))
            para1 = 0x2a;
        else if ((Data_bit == 7) & (Stop_bit == 2))
            para1 = 0x2e;
        else if ((Data_bit == 6) & (Stop_bit == 1))
            para1 = 0x29;
        else if ((Data_bit == 6) & (Stop_bit == 2))
            para1 = 0x2d;
        else if ((Data_bit == 5) & (Stop_bit == 1))
            para1 = 0x28;
        else if ((Data_bit == 5) & (Stop_bit == 2))
            para1 = 0x2c;
        else
        {
            return -3;
        }
        break;
    case 4: //总为0
        if ((Data_bit == 8) & (Stop_bit == 1))
            para1 = 0x3b;
        else if ((Data_bit == 8) & (Stop_bit == 2))
            para1 = 0x3f;
        else if ((Data_bit == 7) & (Stop_bit == 1))
            para1 = 0x3a;
        else if ((Data_bit == 7) & (Stop_bit == 2))
            para1 = 0x3e;
        else if ((Data_bit == 6) & (Stop_bit == 1))
            para1 = 0x39;
        else if ((Data_bit == 6) & (Stop_bit == 2))
            para1 = 0x3d;
        else if ((Data_bit == 5) & (Stop_bit == 1))
            para1 = 0x38;
        else if ((Data_bit == 5) & (Stop_bit == 2))
            para1 = 0x3c;
        else
        {
            return -3;
        }
        break;
    case 0: //无校验位
        if ((Data_bit == 8) & (Stop_bit == 1))
            para1 = 0x03;
        else if ((Data_bit == 8) & (Stop_bit == 2))
            para1 = 0x07;
        else if ((Data_bit == 7) & (Stop_bit == 1))
            para1 = 0x02;
        else if ((Data_bit == 7) & (Stop_bit == 2))
            para1 = 0x06;
        else if ((Data_bit == 6) & (Stop_bit == 1))
            para1 = 0x01;
        else if ((Data_bit == 6) & (Stop_bit == 2))
            para1 = 0x05;
        else if ((Data_bit == 5) & (Stop_bit == 1))
            para1 = 0x00;
        else if ((Data_bit == 5) & (Stop_bit == 2))
            para1 = 0x04;
        else
        {
            return -3;
        }
        break;

    default:
        return -2;
        break;
    }

    /////////////////////////////////////////////////////////
    Addr_Reg_Write = 0x22;                                                                                                   //H/F 全双工
    Data_Reg_Write = Half_Dxp;                                                                                               //0:全双工     1:半双工  D[0]管0-1通道；D[1]管2-3通道
    UART_HALF[channel] = Half_Dxp;                                                                                           //保留本通道半双工状态
    Data_Reg_Write = (UART_HALF[0] & 1) + ((UART_HALF[1] & 1) << 1) + ((UART_HALF[2] & 1) << 2) + ((UART_HALF[3] & 1) << 3); //把4个通道的半双工状态计算为控制数据
    Xil_Out32(uart_Handle + Addr_Reg_Write * 4, Data_Reg_Write);

    Addr_Reg_Write = 0x20; //485/232
    //Data_Reg_Write	=	IEEE_Mode;		//0:232     1:485/422
    UART_MODE[channel] = IEEE_Mode;                                                                                          //保留本通道422/232状态
    Data_Reg_Write = (UART_MODE[0] & 1) + ((UART_MODE[1] & 1) << 1) + ((UART_MODE[2] & 1) << 2) + ((UART_MODE[3] & 1) << 3); //把4个通道的422/232状态计算为控制数据
    Xil_Out32(uart_Handle + Addr_Reg_Write * 4, Data_Reg_Write);

    Addr_Reg_Write = 0x21; //TE485 接收A/B端之间接120欧姆终端电阻
    //Data_Reg_Write	=	Term_Resis;		//0:无终端电阻     1:A,B端120欧姆电阻    以后配合DZ1,DZ2信号
    UART_TERM[channel] = Term_Resis;                                                                                         //保留本通道姆终端电阻状态
    Data_Reg_Write = (UART_TERM[0] & 1) + ((UART_TERM[1] & 1) << 1) + ((UART_TERM[2] & 1) << 2) + ((UART_TERM[3] & 1) << 3); //把4个通道的姆终端电阻状态计算为控制数据
    Xil_Out32(uart_Handle + Addr_Reg_Write * 4, Data_Reg_Write);

    Addr_Reg_Write = 0x23; //FEN 全都采用快速模式
    Data_Reg_Write = 15;
    Xil_Out32(uart_Handle + Addr_Reg_Write * 4, Data_Reg_Write);

    Addr_Reg_Write = 0x24; //LB  不内部自循环
    Data_Reg_Write = 0;
    Xil_Out32(uart_Handle + Addr_Reg_Write * 4, Data_Reg_Write);

    if (channel == 0)
    {
        Addr_Reg_Write = 0x26; //chan1 DEN,REN
        Data_Reg_Write = 0;    //0只收不发
        Xil_Out32(uart_Handle + Addr_Reg_Write * 4, Data_Reg_Write);
    }
    else if (channel == 1)
    {
        Addr_Reg_Write = 0x27; //chan2 DEN,REN
        Data_Reg_Write = 0;    //0只收不发
        Xil_Out32(uart_Handle + Addr_Reg_Write * 4, Data_Reg_Write);
    }
    else if (channel == 2)
    {
        Addr_Reg_Write = 0x28; //chan2 DEN,REN
        Data_Reg_Write = 0;    //0只收不发
        Xil_Out32(uart_Handle + Addr_Reg_Write * 4, Data_Reg_Write);
    }
    else if (channel == 3)
    {
        Addr_Reg_Write = 0x29; //chan2 DEN,REN
        Data_Reg_Write = 0;    //0只收不发
        Xil_Out32(uart_Handle + Addr_Reg_Write * 4, Data_Reg_Write);
    }

    parament = (bps & 0xff) + (para1 << 8); //parament的D0-D7是波特率因子；D8-D13为通讯线控制寄存器值；D15为是半双工标志；

    if (channel == 0)
        UART_Command_Code = UART_Command_Code | 0x1; //Command_Code[device_code]的D0位表示命令有效；D4-D7位表示通道使能；D8位表示要设置RTS，D9位表示要读CTS状态，D12-D15是设置1-4通道RTS值。
    else if (channel == 1)
        UART_Command_Code = UART_Command_Code | 0x2;
    else if (channel == 2)
        UART_Command_Code = UART_Command_Code | 0x4;
    else if (channel == 3)
        UART_Command_Code = UART_Command_Code | 0x8;
    else
    {
        return -1;
    }

    ///////////////////////////////////////////////////////////////////
    Addr_Reg_Write = 0x2c;
    Data_Reg_Write = 1;
    Xil_Out32(uart_Handle + Addr_Reg_Write * 4, Data_Reg_Write); //收到数据时产生中断信号
    Addr_Reg_Write = 0x2d;
    Data_Reg_Write = 8;
    Xil_Out32(uart_Handle + Addr_Reg_Write * 4, Data_Reg_Write); //中断使能
    Addr_Reg_Write = 0x2e;
    Data_Reg_Write = 0;
    //Data_Reg_Write	=	1;
    Xil_Out32(uart_Handle + Addr_Reg_Write * 4, Data_Reg_Write); //不使能发送和接收FIFO

    Addr_Reg_Write = 0x2a; //parament
    Data_Reg_Write = parament;
    Xil_Out32(uart_Handle + Addr_Reg_Write * 4, Data_Reg_Write);

    Addr_Reg_Write = 0x2b;              //Command
    Data_Reg_Write = UART_Command_Code; //D0:chan1; D1:chan2; D2:chan3;  ...........
    Xil_Out32(uart_Handle + Addr_Reg_Write * 4, Data_Reg_Write);

    //Delay (0.01);
    rt_thread_mdelay(1);
    ///清空接收FIFO内数据////////////////////////////////////

    Addr_Reg_Read = 0x41 + (channel & 0xf);
    Data_Reg_Read = Xil_In32(uart_Handle + Addr_Reg_Read * 4);
    Addr_Reg_Read = 0x0;
    Data_Reg_Readx = Xil_In32(uart_Handle + Addr_Reg_Read * 4);

    for (i = 0; i < Data_Reg_Read; i++)
    {
        Addr_Reg_Read = 0x45 + (channel & 0xf);
        Data_Reg_Read = Xil_In32(uart_Handle + Addr_Reg_Read * 4);
    }

    return 0;
}

int UART_Stop_Work(uint32_t channel)
{
    uint32_t Addr_Reg_Write;
    uint32_t Data_Reg_Write;

    if (channel == 0)
        UART_Command_Code = UART_Command_Code & 0xe;
    else if (channel == 1)
        UART_Command_Code = UART_Command_Code & 0xd;
    else if (channel == 2)
        UART_Command_Code = UART_Command_Code & 0xb;
    else if (channel == 3)
        UART_Command_Code = UART_Command_Code & 0x7;
    else
    {
        return -1;
    }

    Addr_Reg_Write = 0x2b;              //Command
    Data_Reg_Write = UART_Command_Code; //D0:chan1; D1:chan2; D2:chan3;  ...........
    Xil_Out32(uart_Handle + Addr_Reg_Write * 4, Data_Reg_Write);

    return 0;
}

int UART_Query_FIFO_Status(uint32_t channel, uint32_t *Send_Sta, uint32_t *Rec_Sta)
{
    uint32_t Addr_Reg_Read, Data_Reg_Read, Data_Reg_Readx;

    Addr_Reg_Read = 0x40;
    *Send_Sta = Xil_In32(uart_Handle + Addr_Reg_Read * 4); //read Self_Test_Reg
    Addr_Reg_Read = 0x0;
    Data_Reg_Readx = Xil_In32(uart_Handle + Addr_Reg_Read * 4); //read Self_Test_Reg

    Addr_Reg_Read = 0x41 + (channel & 0xf);
    *Rec_Sta = Xil_In32(uart_Handle + Addr_Reg_Read * 4); //read Self_Test_Reg
    Addr_Reg_Read = 0x0;
    Data_Reg_Readx = Xil_In32(uart_Handle + Addr_Reg_Read * 4); //read Self_Test_Reg

    return 0;
}

int UART_Send_Data(uint32_t channel, uint32_t amount, uint8_t Se_Data[], uint32_t inter_time)
{
    uint32_t i;
    uint32_t Addr_Reg_Write, Data_Reg_Write;

    if (amount > 8192)
        return -1;
    printf("amount is %d \n", amount);
    for (i = 0; i < amount; i++)
    {
        Addr_Reg_Write = 0x30;
        Data_Reg_Write = (Se_Data[i] & 0xff) + ((channel << 8) & 0xf00);
        Xil_Out32(uart_Handle + Addr_Reg_Write * 4, Data_Reg_Write);
        rt_thread_mdelay((inter_time * 1000 + 999)/1000);
    }

    return 0;
}

int UART_Receive_Data(uint32_t channel, uint32_t amount, uint8_t Chan1_Data[])
{
    uint32_t Addr_Reg_Read, Data_Reg_Read, Data_Reg_Readx;
    uint32_t i;

    for (i = 0; i < amount; i++)
    {
        Addr_Reg_Read = 0x45 + (channel & 0xf);
        Data_Reg_Read = Xil_In32(uart_Handle + Addr_Reg_Read * 4);
        Chan1_Data[i] = Data_Reg_Read;
        Addr_Reg_Read = 0x0;
        Data_Reg_Readx = Xil_In32(uart_Handle + Addr_Reg_Read * 4);
    }

    return 0;
}

int UART_TX_Enable(uint32_t channel)
{
    uint32_t Addr_Reg_Write, Data_Reg_Write;

    if (UART_HALF[channel] == 0)
    {                       //全双工
        Data_Reg_Write = 1; //又发又收
    }
    else
    {
        Data_Reg_Write = 3; //只发不收
    }

    Addr_Reg_Write = 0x26 + (channel & 0xf); //chan1 DEN,REN

    Xil_Out32(uart_Handle + Addr_Reg_Write * 4, Data_Reg_Write);

    return 0;
}

int UART_TX_Disable(uint32_t channel)
{
    uint32_t Addr_Reg_Write, Data_Reg_Write;

    Addr_Reg_Write = 0x26 + (channel & 0xf); //chan1 DEN,REN
    Data_Reg_Write = 0;                      //只收不发
    Xil_Out32(uart_Handle + Addr_Reg_Write * 4, Data_Reg_Write);

    return 0;
}

int UART_Reset()
{
    uint32_t Addr_Reg_Write;
    uint32_t Data_Reg_Write;

    Addr_Reg_Write = 0x2f; //reset
    Data_Reg_Write = 0;    //D0为高 复位16C554 ;  D1为低复位TX_FIFO
    Xil_Out32(uart_Handle + Addr_Reg_Write * 4, Data_Reg_Write);
    Data_Reg_Write = 3;
    Xil_Out32(uart_Handle + Addr_Reg_Write * 4, Data_Reg_Write);

    return 0;
}

/**
*
*IIC Write
*Addr: 16bits,data_write:8bits
******************************************************************************/

int IIC_Write(int Addr, int data_write)
{
    Xil_Out32(g_virtualAddrHandle + Addr_EPROM_Control * 4, CMD_EPROM_Disable);           //IIC_Disable
    Xil_Out32(g_virtualAddrHandle + Addr_EPROM_Control * 4, CMD_EPROM_Addr + Addr);       //IIC addr
    Xil_Out32(g_virtualAddrHandle + Addr_EPROM_Control * 4, CMD_EPROM_Data + data_write); //IIC data2write
    Xil_Out32(g_virtualAddrHandle + Addr_EPROM_Control * 4, CMD_EPROM_Write);             //IIC mode:write
    Xil_Out32(g_virtualAddrHandle + Addr_EPROM_Control * 4, CMD_EPROM_Enable);            //Trig IIC work
    return 0;
}

/**
*
*IIC read
*Addr: 16bits,data_read:8bits
******************************************************************************/
int IIC_Read(int Addr, int *data_read)
{
    int CT = 0;
    int IIC_Status;
    Xil_Out32(g_virtualAddrHandle + Addr_EPROM_Control * 4, CMD_EPROM_Disable);     //IIC_Disable
    Xil_Out32(g_virtualAddrHandle + Addr_EPROM_Control * 4, CMD_EPROM_Addr + Addr); //IIC addr
    Xil_Out32(g_virtualAddrHandle + Addr_EPROM_Control * 4, CMD_EPROM_Read);        //IIC mode:read
    Xil_Out32(g_virtualAddrHandle + Addr_EPROM_Control * 4, CMD_EPROM_Enable);      //Trig IIC work
    while (1)
    {
        CT = CT + 1;
        IIC_Status = Xil_In32(g_virtualAddrHandle + Addr_EPROM_ACK * 4);
        if (IIC_Status == 1)
            break;
        if (CT == 1000000)
        {
            *data_read = 0x12345678;
            return -1;
            break;
        }
    }
    *data_read = Xil_In32(g_virtualAddrHandle + Addr_EPROM_Data * 4);
    return 0;
}

//queue
#define N 10000 //队列中数据元素的数据类型
typedef uint32_t data_t;
typedef struct
{
    //	data_t data[N]; //用数组作为队列的储存空间
    data_t data[N][256];
    int front, rear; //指示队头位置和队尾位置的指针
    unsigned int qlen;
} sequeue_t;

sequeue_t *CreateEmptySequeue()
{
    sequeue_t *queue;
    queue = (sequeue_t *)malloc(sizeof(sequeue_t));
    if (NULL == queue)
        return NULL;
    queue->front = queue->rear = 0;
    queue->qlen = 0;
    return queue;
}
void DestroySequeue(sequeue_t *queue)
{
    if (NULL != queue)
    {
        free(queue);
    }
}
int EmptySequeue(sequeue_t *queue)
{
    if (NULL == queue)
        return -1;
    return (queue->front == queue->rear ? 1 : 0);
}
int FullSequeue(sequeue_t *queue)
{
    if (NULL == queue)
        return -1;
    return ((queue->rear + 1) % N == queue->front ? 1 : 0);
}
void ClearSequeue(sequeue_t *queue)
{
    if (NULL == queue)
        return;
    queue->front = queue->rear = 0;
    queue->qlen = 0;
    return;
}
int EnQueue(sequeue_t *queue, data_t x[256])
{
    int i;
    if (NULL == queue)
        return -1;
    if (1 == FullSequeue(queue))
        return -2; /* full */
    queue->rear = (queue->rear + 1) % N;
    //queue->data[queue->rear] = x;
    for (i = 0; i < 256; i++)
    {
        queue->data[queue->rear][i] = x[i];
    }
    //	printf("EnQueue[0] queue->data is:[%x],,,, x is :[%x],,queue->rear=[%d]\n",queue->data[queue->rear][0], x[0],queue->rear);
    //	printf("EnQueue[1] queue->data is:[%x],,,, x is :[%x],,queue->rear=[%d]\n",queue->data[queue->rear][1], x[1],queue->rear);
    queue->qlen++;
    return 0;
}
int DeQueue(sequeue_t *queue, data_t x[256])
{
    int i;
    if (NULL == queue)
        return -1;
    if (1 == EmptySequeue(queue))
        return -2; /* empty */
    queue->front = (queue->front + 1) % N;
    queue->qlen--;
    if (NULL != x)
    {
        //*x = queue->data[queue->front];
        //printf("queue->front:[%d]!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n",queue->front);
        for (i = 0; i < 256; i++)
        {
            x[i] = queue->data[queue->front][i];
            //printf("data%d is :[%x]\n",i, queue->data[queue->front][i]);
        }
        //		printf("DeQueueDeQueueDeQueueDeQueue[0]\n queue->data is:[%x],,,, x is :[%x]\nDeQueueDeQueueDeQueue!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n",queue->data[queue->front][0], x[0]);
        //		printf("queue->front:[%d],,,, data1 is :[%x]\n",queue->front, queue->data[queue->front][1]);
        //		printf("queue->front:[%d],,,, data2 is :[%x]\n",queue->front, queue->data[queue->front][2]);
    }
    return 0;
}
//queue
sequeue_t *Aqueue;

//pthread
rt_thread_t thread_mean;
struct rt_mutex p_lock;
unsigned int *threadflag;

void func_mean(void *arg)
{
    int *id = (int *)arg;
    int pxiid = 0;
    int k = 0;
    uint32_t Addr_BRAM_Data;
    uint32_t Addr_Reg_Read;
    uint64_t data64 = 0;

    uint16_t Data16[256] = {0};
    data_t Data32[256] = {0};
    int readvalue = 0;
    int ret = 0;

    pxiid = *id;
    printf("pxiid%d\n", pxiid);
    pxiid = *threadflag;
    printf("threadflag is :[%d]\n", pxiid);
    while (1)
    {
        pxiid = *threadflag;

        if (pxiid == 1)
        {
            //pthread_testcancel();
            //数据可读
            Addr_Reg_Read = Addr_AI_FIFO_Status;
            readvalue = Xil_In32(g_virtualAddrHandle + Addr_Reg_Read * 4);
            if (readvalue > 0x80000100)
            {
                //read BRAM Data
                for (k = 0; k < 256; k++)
                {
                    Addr_BRAM_Data = k;
                    data64 = Xil_In64(dma_AddrHandle + Addr_BRAM_Data * 8);
                    Data32[k] = 0xffffffff & data64;
                    //printf("Data16[%d] is :[%x],[%x]\n",data64, Data16[k]);
                }
                //BRAM ReadytoWrite
                Xil_Out32(g_virtualAddrHandle + Addr_ReadytoWrite * 4, 0x1); //ReadytoWrite
                //printf("func_mean[0] data64 is:[%x] is :[%x]\n",data64, Data16[0]);

                //enter queue
                rt_mutex_take(&p_lock, RT_WAITING_FOREVER);
                ret = EnQueue(Aqueue, Data32);
                rt_mutex_release(&p_lock);
            }
        }
        else
        {
            rt_thread_mdelay(5); //20200507   50000-> 5000
        }
    }
    //return NULL;
}

/**
*
*Compensate ADC data
******************************************************************************/
int ADC_DataCompensate(int channel, double data, double *data_out)
{
    int line = 0, row = 0;
    int i, rowlength;
    double x1, x2, y1, y2, k, b;

    rowlength = 7;
    line = channel + 1;

    if (data <= AI_Calibrate_Data[line][0])
    {
        x1 = AI_Calibrate_Data[line][0];
        y1 = AI_Calibrate_Data[0][0];

        x2 = AI_Calibrate_Data[line][1];
        y2 = AI_Calibrate_Data[0][1];
    }
    else if (data >= AI_Calibrate_Data[line][rowlength - 1])
    {
        x1 = AI_Calibrate_Data[line][rowlength - 2];
        y1 = AI_Calibrate_Data[0][rowlength - 2];

        x2 = AI_Calibrate_Data[line][rowlength - 1];
        y2 = AI_Calibrate_Data[0][rowlength - 1];
    }
    else
    {
        for (i = 1; i < rowlength; i++)
        {
            if (data <= AI_Calibrate_Data[line][i])
            {
                x1 = AI_Calibrate_Data[line][i - 1];
                y1 = AI_Calibrate_Data[0][i - 1];

                x2 = AI_Calibrate_Data[line][i];
                y2 = AI_Calibrate_Data[0][i];
                break;
            }
        }
    }

    k = (y2 - y1) / (x2 - x1);
    b = y1 - k * x1;

    data = k * data + b;

    *data_out = data; //y = kx + b

    return 0;
}

static int has_init = 0;
int id = 0;
void myapp(void *parameter)
{
    printf("parameter0:%s\n", (char *)parameter);
    char buf[MAXLINE * 50];
    char buf1[MAXLINE];
    char str[50];
    int i, n, m;
    int k = 0;
    int writevalue = 0;
    int readvalue = 0;
    char revbuf[MAXLINE];
    char revbuf1[MAXLINE];
    char sendbuf[MAXLINE];
    char bufHead[11][50] = {"InitSystem:", "CloseSystem:", "ChannelConfig:", "SampleConfig:", "TrigConfig:", "AcqStart", "DataReady",
                            "Mean", "AcqStop", "viIn32:BAR,OFFSET", "viOut32:BAR,OFFSET,VALUE"};
    int chan = 0;
    uint32_t Addr_Reg_Read;
    uint32_t Addr_Reg_Write;
    uint32_t Data_Reg_Write;
    uint32_t Data_Reg_Read;
    uint32_t Data_BRAM_Read;
    uint32_t Data_BRAM[256] = {0};
    uint32_t Addr_BRAM_Data;
    //uint32_t Range_Value = 0;
    uint32_t Range_Value[8] = {0};
    uint32_t Channel_Range[16] = {1, 10, 100, 1000, 2, 20, 200, 2000, 5, 50, 500, 5000, 0, 0, 0, 10000};
    uint64_t data64 = 0;
    data_t fifodata[256] = {0};

    double Float_Data_CH[256] = {0.0};
    double Data_CHx[8][32] = {0.0};
    double Ave_Data_CHx[8] = {0.0};
    double Compensated_Data[8] = {0.0};
    double Float_Data_temp;
    int Data_temp;
    char *databuf = NULL;
    int datacount = 0;
    int ret = 0;

    //cal
    double SAICalArray[6][7] = {//[DevCalModeNum] // SDevCalChanNum  ////////电压校准AMC4324D、AMC4321D、AMC4340、AMC4321C//20130808
                                {-0.009, -0.006, -0.003, 0.0, 0.003, 0.006, 0.009},
                                {-0.09, -0.06, -0.03, 0.0, 0.03, 0.06, 0.09},
                                {-0.9, -0.6, -0.3, 0.0, 0.3, 0.6, 0.9},
                                {-1.8, -1.2, -0.6, 0.0, 0.6, 1.2, 1.8},
                                {-4.5, -3.0, -1.5, 0.0, 1.5, 3.0, 4.5},
                                {-9.0, -6.0, -3.0, 0.0, 3.0, 6.0, 9.0}
                                };
    int ChannelNum = 8;
    int CalPointsNum = 7;
    int GainNum = 6;
    int CalAIRefAddr = 0x2000;
    int CalAIBaseAddr = 0;

    int j, CalMode = 0, CT = 0;
    int Addr_IIC;
    double *CalDevData = NULL, AverageData[80] = {0.00}; //通道数目//2013.11.12,从32到80
    long NumRead;
    char temp, DeviceName[20];
    char CnumStr[5];
    int RangAddr = 0, PointAddr = 0, ChanAddr = 0;
    int CalPN = 1000;
    double CalRate = 1000.00;

    union
    {
        char C_Data[4];
        float F_Data;
    } DataFC;

    int calrange = 0, calvalue = 0;
    int baud = 0, parity = 0, databit = 0, stopbit = 0, mode = 0, termr = 0, half = 0, txnum = 0, rxnum = 0, intertime = 0;

    /*CAN*/
    uint32_t CurrentFrameCount = 0;
    char Candata[3];
    char *Canptr;

    CAN_CHCONFIG_PARA *CanConfigPara = NULL;
    CAN_FLT_CHCONFIG *CanFltConfig = NULL;
    CAN_MSG_DATA_WR *CanMsgDataWr = NULL;
    ST_CAN_FRAME_DESC *StCanFrameDesc = NULL;

    //创建队列
    if (!has_init)
    {
        has_init = 1;
        Aqueue = CreateEmptySequeue();
        databuf = (char *)malloc(1100 * 512 + 6);
        threadflag = (unsigned int *)malloc(sizeof(unsigned int));
        *threadflag = 0;
        ClearSequeue(Aqueue);
        rt_mutex_init(&p_lock, "queueLock", RT_IPC_FLAG_FIFO);
        thread_mean = rt_thread_create("thread_mean",
                                func_mean,
                                (void *)&id,
                                8192,
                                19,
                                5);
    }
    else
    {
        *threadflag = 0;
        ClearSequeue(Aqueue);
    }


    printf("2020058----        -----1-------------------- ...\n");
    printf("*************************************************\n");
    printf("--------------driver for CAN b001!---------------\n");
    printf("--------------driver for uart b002!--- ----------\n");
    printf("*************************************************\n");
    printf("Accepting connections ...\n");

    strcpy(buf, (char *)parameter);
    n = strlen(buf);
    memset(buf1, 0, 80);
    if (strstr(buf, "InitSystem:") != NULL) /* Command1 */
    {
        readvalue = Xil_In32(g_virtualAddrHandle + 0x0);
        printf("cmd:%s,Xil_In32 is :[%d]\n", buf, readvalue);
    }
    else if (strstr(buf, "CloseSystem:") != NULL) /* Command2 */
    {
        printf("return buf:%s\n", buf);
    }
    else if (strstr(buf, "ChannelConfig:") != NULL) /* Command3 */
    {
        //通道配置
        printf("buf:%s\n", buf);
        k = 0;
        for (i = 14; i < n; i++)
        {
            revbuf[k++] = buf[i];
            if (buf[i] == ',')
            {
                i++;
                break;
            }
        }
        revbuf[k] = '\0';
        chan = atoi(revbuf); //chan
        printf("chan is :[%d]\n", chan);

        for (j = 0; j < 8; j++)
        {
            k = 0; //range
            for (; i < n; i++)
            {
                revbuf1[k++] = buf[i];
                if ((buf[i] == '?') || (buf[i] == ','))
                {
                    i++;
                    break;
                }
            }
            revbuf1[k] = '\0';
            Range_Value[j] = atoi(revbuf1);
            printf("Range_Value[%d] is :[%d]\n", j, Range_Value[j]);
        }

        //global reset
        Addr_Reg_Write = Addr_G_nRST;
        Data_Reg_Write = 0;

        Xil_Out32(g_virtualAddrHandle + Addr_Reg_Write * 4, Data_Reg_Write); //G_nRST

        Addr_Reg_Write = Addr_AI_Channel;

        for (i = 0; i < 8; i++)
        {
            Data_Reg_Write = 0x50000033 + i * 0x10000;
            Xil_Out32(g_virtualAddrHandle + Addr_Reg_Write * 4, Data_Reg_Write); //input mode SI
        }

        printf("chan  is :[0x%x]  ,Range_Value is :[0x%x]\n", chan, Range_Value);
        Xil_Out32(g_virtualAddrHandle + 0x110 * 4, chan);

        Addr_Reg_Write = Addr_AI_Channel;
        for (i = 0; i < 8; i++) //2020.2.26
        {
            if ((chan & (1 << i)) != 0)
            {
                Data_Reg_Write = 0x30000000 + i * 0x10000 + Range_Value[i];          //Range 10V,gain x1
                                                                                     // break;
                Xil_Out32(g_virtualAddrHandle + Addr_Reg_Write * 4, Data_Reg_Write); //input mode SI
                printf("chanconfig is :[0x%x]\n", Data_Reg_Write);
            }
        }
    }
    else if (strstr(buf, "SampleConfig:") != NULL) /* Command4 */
    {
        printf("buf:%s\n", buf);
        k = 0;
        for (i = 13; i < n; i++)
        {
            revbuf[k++] = buf[i];
            if (buf[i] == '?')
            {
                i++;
                break;
            }
        }
        revbuf[k] = '\0';
        writevalue = atoi(revbuf);
        printf("writevalue is :[%d]\n", writevalue);

        Addr_Reg_Write = Addr_AI_Timing;
        Data_Reg_Write = 50000000 / writevalue;                              //ADC Sample clock = 50000000/Data_Reg_Write
        Xil_Out32(g_virtualAddrHandle + Addr_Reg_Write * 4, Data_Reg_Write); //AI_SampleClk_Div
    }
    else if (strstr(buf, "TrigConfig:") != NULL) /* Command5 */
    {
        //触发配置
        printf("buf:%s\n", buf);
        k = 0;
        for (i = 11; i < n; i++)
        {
            revbuf[k++] = buf[i];
            if (buf[i] == '?')
                break;
        }
        revbuf[k] = '\0';
        chan = atoi(revbuf);
        printf("TrigConfig Type is :[%d]\n", chan);

        Addr_Reg_Write = Addr_AI_Trigger;
        Data_Reg_Write = chan;
        Xil_Out32(g_virtualAddrHandle + Addr_Reg_Write * 4, Data_Reg_Write); //AI_nTirg
    }
    else if (strstr(buf, "AcqStart") != NULL) /* Command6 */
    {
        //启动采集
        *threadflag = 0;
        ClearSequeue(Aqueue); //20200327
        rt_thread_mdelay(10);
        printf("buf is :[%s]\n", buf);

        //config_Work_EN
        Addr_Reg_Write = Addr_AI_Command;
        Data_Reg_Write = CMD_AI_Work_Enable;
        Xil_Out32(g_virtualAddrHandle + Addr_Reg_Write * 4, Data_Reg_Write); //Work_EN

        //config_Trig
        Addr_Reg_Write = Addr_AI_Trigger;
        Data_Reg_Write = CMD_AI_nTirg;
        Xil_Out32(g_virtualAddrHandle + Addr_Reg_Write * 4, Data_Reg_Write); //AI_nTirg

        //BRAM ReadytoWrite
        Addr_Reg_Write = Addr_ReadytoWrite;
        Data_Reg_Write = 0x1;
        Xil_Out32(g_virtualAddrHandle + Addr_Reg_Write * 4, Data_Reg_Write); //ReadytoWrite

        *threadflag = 1; //20200327
        printf("start threadflag:%d \n", *threadflag);

        printf("input config success\n");
    }
    else if (strstr(buf, "DataReady") != NULL) /* Command7 */
    {
        readvalue = Aqueue->qlen; //20200327
        printf("dataready is :[%d]\n", readvalue);
    }
    else if (strstr(buf, "Mean") != NULL) /* Command8 */
    {
        strcpy(databuf, "Mean:");
        n = 5;

        datacount = Aqueue->qlen;
        printf("Aqueue->qlen!  !!!!!!!    !!!!!!!! is :[%d]\n", datacount);
        if (datacount > 5)
        {
            datacount = 5;
        }

        for (i = 0; i < datacount; i++)
        {
            rt_mutex_take(&p_lock, RT_WAITING_FOREVER);
            ret = DeQueue(Aqueue, fifodata);
            rt_mutex_release(&p_lock);
            if (ret == 0)
            {
                memcpy(databuf + 5 + i * sizeof(data_t) * 256, fifodata, sizeof(data_t) * 256);
                //n = sizeof(databuf);
                n = n + sizeof(data_t) * 256;
            }
            else
            {
                printf("queue is empty\n");
                strcpy(databuf, "Mean:NULL");
                n = strlen(databuf);
                break;
            }
        }
        databuf[n] = '\0';
        printf("%s", databuf);
    }
    else if (strstr(buf, "AcqStop") != NULL) /* Command9 */
    {
        //停止采集
        //config_Work_Disable
        Addr_Reg_Write = Addr_AI_Command;
        Data_Reg_Write = CMD_AI_Work_Disable;
        Xil_Out32(g_virtualAddrHandle + Addr_Reg_Write * 4, Data_Reg_Write); //Work_Disable

        strcpy(buf1, "AcqStop");
        strcat(buf1, ",1");

        strcat(buf1, ";\n");

        printf("%s", buf1);

        *threadflag = 0;
        ClearSequeue(Aqueue);
    }
    else if (strstr(buf, "viIn32:BAR") != NULL) /* Command10 */
    {
        //寄存器回读
        k = 0;
        for (i = 11; i < n; i++)
        {
            revbuf[k++] = buf[i];
            if (buf[i] == '?')
            {
                i++;
                break;
            }
        }
        revbuf[k] = '\0';
        chan = atoi(revbuf);
        printf("viIn32:BAR,OFFSET is :[%d]\n", chan);

        Addr_Reg_Read = chan;
        readvalue = Xil_In32(g_virtualAddrHandle + Addr_Reg_Read * 4);
        printf("viIn32:BAR is :[0x%x]\n", readvalue);
    }
    else if (strstr(buf, "viOut32:BAR,") != NULL) /* Command11 */
    {
        //寄存器配置
        /* 解析OFFSET */
        k = 0;
        for (i = 12; i < n; i++)
        {
            revbuf[k++] = buf[i];
            if (buf[i] == ',')
            {
                i++;
                break;
            }
        }
        revbuf[k] = '\0';
        chan = atoi(revbuf);
        printf("viOut32:BAR,OFFSET is :[%d]\n", chan);
        Addr_Reg_Write = chan;

        /* 解析OFFSET */
        k = 0;
        for (; i < n; i++)
        {
            revbuf1[k++] = buf[i];
            if (buf[i] == '?')
            {
                i++;
                break;
            }
        }
        revbuf1[k] = '\0';
        chan = atoi(revbuf1);
        printf("viOut32:BAR,VALUE is :[%d]\n", chan);

        Data_Reg_Write = chan;
        Xil_Out32(g_virtualAddrHandle + Addr_Reg_Write * 4, Data_Reg_Write);
    }
    /////////////////cal
    else if (strstr(buf, "CalWriteInt?") != NULL) /* CalWriteInt */
    {
        ChanAddr = ChannelNum * 4;
        PointAddr = CalPointsNum * ChanAddr;
        RangAddr = GainNum * PointAddr;

        for (j = 0; j < GainNum; j++) //	strcat(DeviceName,"/ai0:31");
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
    }
    //Cal
    else if (strstr(buf, "CalWriteRaw:") != NULL) /* CalWriteRaw */
    {
        printf("buf:%s\n", buf);
        k = 0;
        for (i = 12; i < n; i++)
        {
            revbuf[k++] = buf[i];
            if (buf[i] == ',')
            {
                i++;
                break;
            }
        }
        revbuf[k] = '\0';
        calrange = atoi(revbuf);
        printf("calrange is :[%d]\n", calrange);

        k = 0;
        for (; i < n; i++)
        {
            revbuf1[k++] = buf[i];
            if (buf[i] == '?')
            {
                break;
            }
        }
        revbuf1[k] = '\0';
        calvalue = atoi(revbuf1);
        printf("calvalue is :[%d]\n", calvalue);

        int RangX = 0;
        double GainCoff = 1.0;

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

        //j,k,下发下来的
        //通道配置 8个通道一起使能；
        //量程传进来；
        //采样频率：100k;
        //启动采集流程；

        //1024个点，取平均值，存进去。
        //global reset
        Addr_Reg_Write = Addr_G_nRST;
        Data_Reg_Write = 0;
        Xil_Out32(g_virtualAddrHandle + Addr_Reg_Write * 4, Data_Reg_Write); //G_nRST

        //config_input mode
        Addr_Reg_Write = Addr_AI_Channel;
        for (i = 0; i < 8; i++)
        {
            //Data_Reg_Write	=	0x40000000 + i*0x10000 ;
            Data_Reg_Write = 0x50000033 + i * 0x10000;
            Xil_Out32(g_virtualAddrHandle + Addr_Reg_Write * 4, Data_Reg_Write); //input mode SI
        }

        //config_input Range
        Addr_Reg_Write = Addr_AI_Channel;
        for (i = 0; i < 8; i++)
        {
            Data_Reg_Write = 0x30000000 + i * 0x10000 + RangX; //Range 10V,gain x1
            //  	Data_Reg_Write	=	0x30000033 + i*0x10000;	//Range 10mV,gain x10000
            Xil_Out32(g_virtualAddrHandle + Addr_Reg_Write * 4, Data_Reg_Write); //input mode SI
        }

        //config_Sample_CLK
        Addr_Reg_Write = Addr_AI_Timing;
        Data_Reg_Write = 50000000 / 10000;                                   //ADC Sample clock = 50000000/Data_Reg_Write
        Xil_Out32(g_virtualAddrHandle + Addr_Reg_Write * 4, Data_Reg_Write); //AI_SampleClk_Div

        rt_thread_mdelay(1000);

        //config_Work_EN
        Addr_Reg_Write = Addr_AI_Command;
        Data_Reg_Write = CMD_AI_Work_Enable;
        Xil_Out32(g_virtualAddrHandle + Addr_Reg_Write * 4, Data_Reg_Write); //Work_EN

        //config_Trig
        Addr_Reg_Write = Addr_AI_Trigger;
        Data_Reg_Write = CMD_AI_nTirg;
        Xil_Out32(g_virtualAddrHandle + Addr_Reg_Write * 4, Data_Reg_Write); //AI_nTirg

        //BRAM ReadytoWrite
        Addr_Reg_Write = Addr_ReadytoWrite;
        Data_Reg_Write = 0x1;
        Xil_Out32(g_virtualAddrHandle + Addr_Reg_Write * 4, Data_Reg_Write); //ReadytoWrite

        //read FIFO Status
        CT = 0;
        while (1)
        {
            CT = CT + 1;
            Addr_Reg_Read = Addr_AI_FIFO_Status;
            readvalue = Xil_In32(g_virtualAddrHandle + Addr_Reg_Read * 4);
            if (readvalue > 0x80000100)
                break;
        }

        uint32_t Data_BRAM[256] = {0};
        double Float_Data_CH[256] = {0.0};
        double Data_CHx[8][32] = {0.0};
        double Ave_Data_CHx[8] = {0.0};
        double Float_Data_temp;
        int Data_temp;

        //read BRAM Data
        for (k = 0; k < 256; k++)
        {
            Addr_BRAM_Data = k;
            data64 = Xil_In64(dma_AddrHandle + Addr_BRAM_Data * 8);
            Data_BRAM[k] = 0xffffffff & data64;
        }
        //BRAM ReadytoWrite
        Xil_Out32(g_virtualAddrHandle + Addr_ReadytoWrite * 4, 0x1); //ReadytoWrite

        for (k = 0; k < 256; k++)
        {
            if (Data_BRAM[k] < 131072)
                Data_temp = Data_BRAM[k];
            else
                Data_temp = -(262143 - Data_BRAM[k]);

            Float_Data_temp = Data_temp * 38.15 / 1000000;
            Float_Data_temp = (5.0 - Float_Data_temp / 0.1818) / 2.0 /*/GainCoff*/; ///20200608  V to mV
            Float_Data_CH[k] = Float_Data_temp;
        }

        for (i = 0; i < 8; i++)
        {
            for (m = 0; m < 32; m++)
                Data_CHx[i][m] = Float_Data_CH[m * 8 + i];
        }

        for (i = 0; i < 8; i++)
        {
            Float_Data_temp = 0.0;
            for (m = 0; m < 32; m++)
            {
                Float_Data_temp = Float_Data_temp + Data_CHx[i][m];
            }
            Ave_Data_CHx[i] = Float_Data_temp / 32.0; //gain=1
        }

        Addr_Reg_Write = Addr_AI_Command;
        Data_Reg_Write = CMD_AI_Work_Disable;
        Xil_Out32(g_virtualAddrHandle + Addr_Reg_Write * 4, Data_Reg_Write); //Work_Disable

        j = calrange;
        k = calvalue;

        for (i = 0; i < ChannelNum; i++)
        {
            DataFC.F_Data = (float)Ave_Data_CHx[i];
            Addr_IIC = CalAIBaseAddr + RangAddr * CalMode + PointAddr * j + ChanAddr * k + 4 * i;
            printf("CalWriteRaw Addr_IIC :[%d],Data is %f\n", Addr_IIC, DataFC.F_Data);
            for (m = 0; m < 4; m++)
            {
                rt_thread_mdelay(10);
                temp = DataFC.C_Data[m];
                IIC_Write(Addr_IIC + m, temp);
            }
            rt_thread_mdelay(10);
        }
    }

    else if (strstr(buf, "CalReadInt?") != NULL) /* CalReadInt */
    {
        int temp_data = 0;

        ChanAddr = ChannelNum * 4;
        PointAddr = CalPointsNum * ChanAddr;
        RangAddr = GainNum * PointAddr;

        memset(databuf, 0, 1100 * 512);
        strcpy(databuf, "CalReadInt:");
        n = 0;
        for (j = 0; j < GainNum; j++)
        {
            for (k = 0; k < CalPointsNum; k++)
            {
                Addr_IIC = CalAIRefAddr + /*RangAddr*i+*/ PointAddr * j + 4 * k;

                for (m = 0; m < 4; m++) //初值，发一个读初值的命令
                {
                    IIC_Read(Addr_IIC + m, &temp_data);
                    DataFC.C_Data[m] = (char)temp_data;
                }
                if (n != 0)
                {
                    strcat(databuf, ",");
                }
                else
                {
                    n++;
                }
                sprintf(str, "%.3f", DataFC.F_Data);
                strcat(databuf, str);
                printf("DataFC.F_Data is :[%.3f]\n", DataFC.F_Data);
            }
        }

        strcat(databuf, ";\n");

        n = strlen(databuf);
        printf("CalReadInt byte is :[%d]\n", n);
        databuf[n] = '\0';
        printf("send buf1 is :[%s]\n", databuf);
    }

    else if (strstr(buf, "CalReadRaw?") != NULL) /* CalReadRaw */
    {
        int temp_data = 0;
        int r = 0;

        ChanAddr = ChannelNum * 4;
        PointAddr = CalPointsNum * ChanAddr;
        RangAddr = GainNum * PointAddr;

        memset(databuf, 0, 1100 * 512);
        strcpy(databuf, "CalReadRaw:");
        n = 0;
        for (j = 0; j < GainNum; j++)
        {
            for (k = 0; k < CalPointsNum; k++)
            {

                for (r = 0; r < ChannelNum; r++) //发读真实值的命令
                {
                    Addr_IIC = CalAIBaseAddr + /*RangAddr*i+*/ PointAddr * j + ChanAddr * k + 4 * r;

                    for (m = 0; m < 4; m++)
                    {
                        IIC_Read(Addr_IIC + m, &temp_data);
                        DataFC.C_Data[m] = (char)temp_data;
                    }
                    printf("CalReadRaw Addr_IIC :[%d],data is %f\n", Addr_IIC, DataFC.F_Data);
                    if (n != 0)
                    {
                        strcat(databuf, ",");
                    }
                    else
                    {
                        n++;
                    }
                    sprintf(str, "%.3f", DataFC.F_Data);
                    strcat(databuf, str);
                }
            }
        }

        strcat(databuf, ";\n");

        n = strlen(databuf);
        printf("CalReadRaw byte is :[%d]\n", n);
    }
    /**********uart********/
    else if (strstr(buf, "UART_OpenComConfig:") != NULL) /* UART_OpenComConfig */
    {

        printf("buf:%s\n", buf);
        k = 0;
        for (i = 19; i < n; i++)
        {
            revbuf[k++] = buf[i];
            if (buf[i] == ',')
            {
                i++;
                break;
            }
        }
        revbuf[k] = '\0';
        chan = atoi(revbuf);
        printf("chan is :[%d]\n", chan);

        k = 0;
        for (; i < n; i++)
        {
            revbuf1[k++] = buf[i];
            if (buf[i] == ',')
            {
                i++;
                break;
            }
        }
        revbuf1[k] = '\0';
        baud = atoi(revbuf1);
        printf("baud is :[%d]\n", baud);

        k = 0;
        for (; i < n; i++)
        {
            revbuf1[k++] = buf[i];
            if (buf[i] == ',')
            {
                i++;
                break;
            }
        }
        revbuf1[k] = '\0';
        parity = atoi(revbuf1);
        printf("parity is :[%d]\n", parity);

        k = 0;
        for (; i < n; i++)
        {
            revbuf1[k++] = buf[i];
            if (buf[i] == ',')
            {
                i++;
                break;
            }
        }
        revbuf1[k] = '\0';
        databit = atoi(revbuf1);
        printf("databit is :[%d]\n", databit);

        k = 0;
        for (; i < n; i++)
        {
            revbuf1[k++] = buf[i];
            if (buf[i] == ',')
            {
                i++;
                break;
            }
        }
        revbuf1[k] = '\0';
        stopbit = atoi(revbuf1);
        printf("stopbit is :[%d]\n", stopbit);

        k = 0;
        for (; i < n; i++)
        {
            revbuf1[k++] = buf[i];
            if (buf[i] == ',')
            {
                i++;
                break;
            }
        }
        revbuf1[k] = '\0';
        mode = atoi(revbuf1);
        printf("mode is :[%d]\n", mode);

        k = 0;
        for (; i < n; i++)
        {
            revbuf1[k++] = buf[i];
            if (buf[i] == ',')
            {
                i++;
                break;
            }
        }
        revbuf1[k] = '\0';
        termr = atoi(revbuf1);
        printf("termr is :[%d]\n", termr);

        k = 0;
        for (; i < n; i++)
        {
            revbuf1[k++] = buf[i];
            if (buf[i] == '?')
            {
                i++;
                break;
            }
        }
        revbuf1[k] = '\0';
        half = atoi(revbuf1);
        printf("half is :[%d]\n", half);

        readvalue = UART_OpenComConfig(chan, baud, parity, databit, stopbit, mode, termr, half);

        strcpy(buf1, "UART_OpenComConfig:");
        strcat(buf1, SAMPLE_CONFIG_SUCCESS);

        strcat(buf1, ";\n");

        n = strlen(buf1);
    }
    else if (strstr(buf, "UART_Query_FIFO_Status:") != NULL) /* UART_Query_FIFO_Status */
    {
        printf("buf:%s\n", buf);
        k = 0;
        for (i = 23; i < n; i++)
        {
            revbuf[k++] = buf[i];
            if (buf[i] == '?')
            {
                i++;
                break;
            }
        }
        revbuf[k] = '\0';
        chan = atoi(revbuf);
        printf("chan is :[%d]\n", chan);

        readvalue = UART_Query_FIFO_Status(chan, &txnum, &rxnum);
        /*           
              strcpy(buf1, "UART_Query_FIFO_Status:");
              strcat(buf1, "192.168.1.100,");

              strcat(buf1, ";\n");*/

        printf("UART_Query_FIFO_Status:%d,%d,%d;\n", chan, txnum, rxnum);
    }
    else if (strstr(buf, "UART_Receive_Data:") != NULL) /* UART_Receive_Data */
    {
        uint8_t *chandata = NULL;

        printf("buf:%s\n", buf);
        k = 0;
        for (i = 18; i < n; i++)
        {
            revbuf[k++] = buf[i];
            if (buf[i] == ',')
            {
                i++;
                break;
            }
        }
        revbuf[k] = '\0';
        chan = atoi(revbuf);
        printf("chan is :[%d]\n", chan);

        k = 0;
        for (; i < n; i++)
        {
            revbuf1[k++] = buf[i];
            if (buf[i] == '?')
            {
                i++;
                break;
            }
        }
        revbuf1[k] = '\0';
        rxnum = atoi(revbuf1);
        printf("num is :[%d]\n", rxnum);

        chandata = (uint8_t *)malloc(sizeof(uint8_t) * rxnum);

        readvalue = UART_Receive_Data(chan, rxnum, chandata);

        strcpy(databuf, "UART_Receive_Data:");
        strcat(databuf, "192.168.1.100,");
        sprintf(str, "%d", chan);
        strcat(databuf, str);
        strcat(databuf, ",");
        sprintf(str, "%d", rxnum);
        strcat(databuf, str);

        strcat(databuf, ",");
        for (i = 0; i < rxnum; i++)
        {
            //strcat(buf1, ",");
            sprintf(str, "%02x", chandata[i]);
            strcat(databuf, str);
        }
        strcat(databuf, ";\n");

        free(chandata);
        chandata = NULL;
    }
    else if (strstr(buf, "UART_Send_Data:") != NULL) /* UART_Send_Data */
    {
        uint8_t *chandata = NULL;
        //			char *tmpdata=NULL;
        char udata[3] = "";
        char *ptr;

        printf("buf:%s\n", buf);
        k = 0;
        for (i = 15; i < n; i++)
        {
            revbuf[k++] = buf[i];
            if (buf[i] == ',')
            {
                i++;
                break;
            }
        }
        revbuf[k] = '\0';
        chan = atoi(revbuf);
        printf("chan is :[%d]\n", chan);

        k = 0;
        for (; i < n; i++)
        {
            revbuf1[k++] = buf[i];
            if (buf[i] == ',')
            {
                i++;
                break;
            }
        }
        revbuf1[k] = '\0';
        txnum = atoi(revbuf1);
        printf("num is :[%d]\n", txnum);

        //			tmpdata=(char *)malloc(sizeof(char)*txnum*4+1);
        k = 0;
        for (; i < n; i++)
        {
            databuf[k++] = buf[i];
            if (buf[i] == ',')
            {
                i++;
                break;
            }
        }
        databuf[k] = '\0';
        printf("sdata is :[%s]\n", databuf);

        k = 0;
        for (; i < n; i++)
        {
            revbuf[k++] = buf[i];
            if (buf[i] == '?')
            {
                i++;
                break;
            }
        }
        revbuf[k] = '\0';
        intertime = atoi(revbuf);
        printf("inter time is :[%d]\n", intertime);

        chandata = (uint8_t *)malloc(sizeof(uint8_t) * txnum);

        for (i = 0; i < txnum; i++)
        {
            udata[0] = databuf[i * 2];
            udata[1] = databuf[i * 2 + 1];
            udata[2] = '\0';
            chandata[i] = (uint8_t)strtol(udata, &ptr, 16);
            printf("chandata[%d] is :[%d]\n", i, chandata[i]);
        }

        readvalue = UART_Send_Data(chan, txnum, chandata, intertime);

        if (readvalue < 0)
            printf("UART_Send_Data:%s;\n", SAMPLE_CONFIG_FAIL);
        else
            printf("UART_Send_Data:%s;\n", SAMPLE_CONFIG_SUCCESS);

        free(chandata);
        chandata = NULL;
    }
    else if (strstr(buf, "UART_Stop_Work:") != NULL) /* UART_Stop_Work */
    {

        printf("buf:%s\n", buf);
        k = 0;
        for (i = 15; i < n; i++)
        {
            revbuf[k++] = buf[i];
            if (buf[i] == '?')
            {
                i++;
                break;
            }
        }
        revbuf[k] = '\0';
        chan = atoi(revbuf);
        printf("chan is :[%d]\n", chan);

        readvalue = UART_Stop_Work(chan);

        if (readvalue < 0)
            printf("UART_Stop_Work:%s;\n", SAMPLE_CONFIG_FAIL);
        else
            printf("UART_Stop_Work:%s;\n", SAMPLE_CONFIG_SUCCESS);
    }
    else if (strstr(buf, "UART_Reset") != NULL) /* UART_Reset */
    {
        printf("buf:%s\n", buf);
        readvalue = UART_Reset();

        if (readvalue < 0)
            printf("UART_Reset:%s;\n", SAMPLE_CONFIG_FAIL);
        else
            printf("UART_Reset:%s;\n", SAMPLE_CONFIG_SUCCESS);
    }

    else if (strstr(buf, "UART_TX_Enable:") != NULL) /* UART_TX_Enable */
    {

        printf("buf:%s\n", buf);
        k = 0;
        for (i = 15; i < n; i++)
        {
            revbuf[k++] = buf[i];
            if (buf[i] == '?')
            {
                i++;
                break;
            }
        }
        revbuf[k] = '\0';
        chan = atoi(revbuf);
        printf("chan is :[%d]\n", chan);

        readvalue = UART_TX_Enable(chan);

        if (readvalue < 0)
            printf("UART_TX_Enable:%s;\n", SAMPLE_CONFIG_FAIL);
        else
            printf("UART_TX_Enable:%s;\n", SAMPLE_CONFIG_SUCCESS);
    }
    else if (strstr(buf, "UART_TX_Disable:") != NULL) /* UART_TX_Disable */
    {
        printf("buf:%s\n", buf);
        k = 0;
        for (i = 16; i < n; i++)
        {
            revbuf[k++] = buf[i];
            if (buf[i] == '?')
            {
                i++;
                break;
            }
        }
        revbuf[k] = '\0';
        chan = atoi(revbuf);
        printf("chan is :[%d]\n", chan);

        readvalue = UART_TX_Disable(chan);

        if (readvalue < 0)
            printf("UART_TX_Disable:%s;\n", SAMPLE_CONFIG_FAIL);
        else
            printf("UART_TX_Disable:%s;\n", SAMPLE_CONFIG_SUCCESS);
    }
    ///////////////CAN
    else if (strstr(buf, "CAN_Reset") != NULL) /* CAN_Reset */
    {
        //通道配置
        printf("buf:%s\n", buf);
        k = CAN_Reset();
        sprintf(buf1, "CAN_Reset:%s;\n", SAMPLE_CONFIG_SUCCESS);
        n = strlen(buf1);
        printf("return buf:%s\n", buf1);
    }
    else if (strstr(buf, "CAN_ChConfig:") != NULL) /* CAN_ChConfig */
    {
        //通道配置
        printf("buf:%s\n", buf);
        CanConfigPara = &gCanCfgPara;

        k = 0;
        for (i = 13; i < n; i++)
        {
            revbuf[k++] = buf[i];
            if (buf[i] == ',')
            {
                i++;
                break;
            }
        }
        revbuf[k] = '\0';
        CanConfigPara->Chan = atoi(revbuf);
        printf("chan is :[%d]\n", CanConfigPara->Chan);

        k = 0;
        for (; i < n; i++)
        {
            revbuf[k++] = buf[i];
            if (buf[i] == ',')
            {
                i++;
                break;
            }
        }
        revbuf[k] = '\0';
        CanConfigPara->Baud = atoi(revbuf);
        printf("Baud is :[%d]\n", CanConfigPara->Baud);

        k = 0;
        for (; i < n; i++)
        {
            revbuf[k++] = buf[i];
            if (buf[i] == ',')
            {
                i++;
                break;
            }
        }
        revbuf[k] = '\0';
        CanConfigPara->Arbitrate = atoi(revbuf);
        printf("Arbitrate is :[%d]\n", CanConfigPara->Arbitrate);

        k = 0;
        for (; i < n; i++)
        {
            revbuf[k++] = buf[i];
            if (buf[i] == ',')
            {
                i++;
                break;
            }
        }
        revbuf[k] = '\0';
        CanConfigPara->SuccessFlag = atoi(revbuf);
        printf("SuccessFlag is :[%d]\n", CanConfigPara->SuccessFlag);

        k = 0;
        for (; i < n; i++)
        {
            revbuf[k++] = buf[i];
            if (buf[i] == '?')
            {
                i++;
                break;
            }
        }
        revbuf[k] = '\0';
        CanConfigPara->Mode = atoi(revbuf);
        printf("Mode is :[%d]\n", CanConfigPara->Mode);

        CAN_ChConfig(CanConfigPara->Chan, CanConfigPara->Baud, CanConfigPara->Arbitrate, CanConfigPara->SuccessFlag, CanConfigPara->Mode);

    }
    else if (strstr(buf, "CAN_FilterConfig:") != NULL) /* CAN_FilterConfig */
    {
        //通道配置
        printf("buf:%s\n", buf);
        CanFltConfig = &gCanFltCfg;

        k = 0;
        for (i = 17; i < n; i++)
        {
            revbuf[k++] = buf[i];
            if (buf[i] == ',')
            {
                i++;
                break;
            }
        }
        revbuf[k] = '\0';
        CanFltConfig->Chan = atoi(revbuf);
        printf("chan is :[%d]\n", CanFltConfig->Chan);

        k = 0;
        for (; i < n; i++)
        {
            revbuf[k++] = buf[i];
            if (buf[i] == ',')
            {
                i++;
                break;
            }
        }
        revbuf[k] = '\0';
        CanFltConfig->Mask = atoi(revbuf);
        printf("Mask is :[%d]\n", CanFltConfig->Mask);

        k = 0;
        for (; i < n; i++)
        {
            revbuf[k++] = buf[i];
            if (buf[i] == ',')
            {
                i++;
                break;
            }
        }
        revbuf[k] = '\0';
        CanFltConfig->FrameId = atoi(revbuf);
        printf("FrameId is :[%d]\n", CanFltConfig->FrameId);

        k = 0;
        for (; i < n; i++)
        {
            revbuf[k++] = buf[i];
            if (buf[i] == '?')
            {
                i++;
                break;
            }
        }
        revbuf[k] = '\0';
        CanFltConfig->ExtFrameId = atoi(revbuf);
        printf("ExtFrameId is :[%d]\n", CanFltConfig->ExtFrameId);

        CAN_FilterConfig(CanFltConfig->Chan, CanFltConfig->Mask, CanFltConfig->FrameId, CanFltConfig->ExtFrameId);
    }
    else if (strstr(buf, "CAN_MsgDataWr:") != NULL) /* CAN_MsgDataWr */
    {
        //通道配置
        printf("buf:%s\n", buf);
        CanMsgDataWr = &gCanMsgDataWr;

        k = 0;
        for (i = 14; i < n; i++)
        {
            revbuf[k++] = buf[i];
            if (buf[i] == ',')
            {
                i++;
                break;
            }
        }
        revbuf[k] = '\0';
        CanMsgDataWr->Chan = atoi(revbuf);
        printf("chan is :[%d]\n", CanMsgDataWr->Chan);

        k = 0;
        for (; i < n; i++)
        {
            revbuf[k++] = buf[i];
            if (buf[i] == ',')
            {
                i++;
                break;
            }
        }
        revbuf[k] = '\0';
        CanMsgDataWr->FrameId = atoi(revbuf);
        printf("FrameId is :[%d]\n", CanMsgDataWr->FrameId);

        k = 0;
        for (; i < n; i++)
        {
            revbuf[k++] = buf[i];
            if (buf[i] == ',')
            {
                i++;
                break;
            }
        }
        revbuf[k] = '\0';
        CanMsgDataWr->ExtFrameId = atoi(revbuf);
        printf("ExtFrameId is :[%d]\n", CanMsgDataWr->ExtFrameId);

        k = 0;
        for (; i < n; i++)
        {
            revbuf[k++] = buf[i];
            if (buf[i] == ',')
            {
                i++;
                break;
            }
        }
        revbuf[k] = '\0';
        CanMsgDataWr->FrameFormat = atoi(revbuf);
        printf("FrameFormat is :[%d]\n", CanMsgDataWr->FrameFormat);

        k = 0;
        for (; i < n; i++)
        {
            revbuf[k++] = buf[i];
            if (buf[i] == ',')
            {
                i++;
                break;
            }
        }
        revbuf[k] = '\0';
        CanMsgDataWr->FrameType = atoi(revbuf);
        printf("FrameType is :[%d]\n", CanMsgDataWr->FrameType);

        k = 0;
        for (; i < n; i++)
        {
            revbuf[k++] = buf[i];
            if (buf[i] == ',')
            {
                i++;
                break;
            }
        }
        revbuf[k] = '\0';
        printf("MsgData is :[%s]\n", revbuf);

        k = 0;
        for (; i < n; i++)
        {
            revbuf1[k++] = buf[i];
            if (buf[i] == '?')
            {
                i++;
                break;
            }
        }
        revbuf1[k] = '\0';
        CanMsgDataWr->MsgLen = atoi(revbuf1);
        printf("MsgLen is :[%d]\n", CanMsgDataWr->MsgLen);

        CanMsgDataWr->MsgData = (unsigned char *)malloc(CanMsgDataWr->MsgLen * sizeof(unsigned char));
        for (i = 0; i < CanMsgDataWr->MsgLen; i++)
        {
            Candata[0] = revbuf[i * 2];
            Candata[1] = revbuf[i * 2 + 1];
            Candata[2] = '\0';
            CanMsgDataWr->MsgData[i] = (uint8_t)strtol(Candata, &Canptr, 16);
            printf("Data[%d] is :[%d]\n", i, CanMsgDataWr->MsgData[i]);
        }

        CAN_MsgDataWr(CanMsgDataWr->Chan, CanMsgDataWr->FrameId, CanMsgDataWr->ExtFrameId, CanMsgDataWr->FrameFormat,
                      CanMsgDataWr->FrameType, CanMsgDataWr->MsgData, CanMsgDataWr->MsgLen);

        free(CanMsgDataWr->MsgData);
    }
    else if (strstr(buf, "CAN_Start:") != NULL) /* CAN_Start */
    {
        //通道配置
        printf("buf:%s\n", buf);

        k = 0;
        for (i = 10; i < n; i++)
        {
            revbuf[k++] = buf[i];
            if (buf[i] == '?')
            {
                i++;
                break;
            }
        }
        revbuf[k] = '\0';
        chan = atoi(revbuf);
        printf("chan is :[%d]\n", chan);

        CAN_Start(chan);
        //free(CanMsgDataWr->MsgData);
    }
    else if (strstr(buf, "CAN_getDataCount:") != NULL) /* CAN_getDataCount */
    {
        //通道配置
        printf("buf:%s\n", buf);

        k = 0;
        for (i = 17; i < n; i++)
        {
            revbuf[k++] = buf[i];
            if (buf[i] == '?')
            {
                i++;
                break;
            }
        }
        revbuf[k] = '\0';
        chan = atoi(revbuf);
        printf("chan is :[%d]\n", chan);

        CAN_getDataCount(chan, &CurrentFrameCount);
    }
    else if (strstr(buf, "CAN_MsgDataRd:") != NULL) /* CAN_MsgDataRd */
    {
        //通道配置
        printf("buf:%s\n", buf);

        StCanFrameDesc = &gStCanFrameDesc;

        k = 0;
        for (i = 14; i < n; i++)
        {
            revbuf[k++] = buf[i];
            if (buf[i] == '?')
            {
                i++;
                break;
            }
        }
        revbuf[k] = '\0';
        chan = atoi(revbuf);
        printf("chan is :[%d]\n", chan);

        StCanFrameDesc->data = (uint8_t *)malloc(sizeof(uint8_t) * 8);

        CAN_MsgDataRd(chan, &StCanFrameDesc->msgNum, &StCanFrameDesc->frame_id, &StCanFrameDesc->ext_frame_id,
                      &StCanFrameDesc->frame_format, &StCanFrameDesc->frame_type, StCanFrameDesc->data, &StCanFrameDesc->data_size);

        free(StCanFrameDesc->data);
    }
    else if (strstr(buf, "CAN_get_send_stat:") != NULL) /* get_send_stat */
    {
        //通道配置
        printf("buf:%s\n", buf);

        k = 0;
        for (i = 18; i < n; i++)
        {
            revbuf[k++] = buf[i];
            if (buf[i] == '?')
            {
                i++;
                break;
            }
        }
        revbuf[k] = '\0';
        chan = atoi(revbuf);
        printf("chan is :[%d]\n", chan);

        ret = get_send_stat(chan);
        printf("CAN_get_send_stat:192.168.1.100,%d;\n", ret);
        ///else sprintf(buf1,"CAN_get_send_stat:%s;\n",SAMPLE_CONFIG_FAIL);

        //free(CanMsgDataWr->MsgData);
    }
    /////1553b
    else if (strstr(buf, "M1553_Reset") != NULL) /* M1553_Reset */
    {

        printf("buf:%s\n", buf);
        k = 0;
        for (i = 11; i < n; i++)
        {
            revbuf[k++] = buf[i];
            if (buf[i] == '?')
            {
                i++;
                break;
            }
        }
        revbuf[k] = '\0';
        chan = atoi(revbuf);
        printf("chan is :[%d]\n", chan);

        readvalue = M1553_Reset(chan);

        if (readvalue < 0)
            printf("M1553_Reset:%s;\n", SAMPLE_CONFIG_FAIL);
        else
            printf("M1553_Reset:%s;\n", SAMPLE_CONFIG_SUCCESS);
    }
    else if (strstr(buf, "M1553_BC_SetMessageCount:") != NULL) /* M1553_BC_SetMessageCount */
    {
        int MsgCount = 0, timeval[16] = {0};
        printf("buf:%s\n", buf);
        k = 0;
        for (i = 25; i < n; i++)
        {
            revbuf[k++] = buf[i];
            if (buf[i] == ',')
            {
                i++;
                break;
            }
        }
        revbuf[k] = '\0';
        chan = atoi(revbuf);
        printf("chan is :[%d]\n", chan);

        k = 0;
        for (; i < n; i++)
        {
            revbuf1[k++] = buf[i];
            if (buf[i] == ',')
            {
                i++;
                break;
            }
        }
        revbuf1[k] = '\0';
        MsgCount = atoi(revbuf1);
        printf("MsgCount is :[%d]\n", MsgCount);

        for (j = 0; j < 16; j++)
        {
            if (i >= n)
                break;
            k = 0; //timeval
            for (; i < n; i++)
            {
                revbuf1[k++] = buf[i];
                if ((buf[i] == '?') || (buf[i] == ','))
                {
                    i++;
                    break;
                }
            }
            revbuf1[k] = '\0';
            timeval[j] = atoi(revbuf1);
            printf("timeval[%d] is :[%d]\n", j, timeval[j]);
        }

        readvalue = M1553_BC_SetMessageCount(chan, MsgCount, timeval);

        if (readvalue < 0)
            printf("M1553_BC_SetMessageCount:%s;\n", SAMPLE_CONFIG_FAIL);
        else
            printf("M1553_BC_SetMessageCount:%s;\n", SAMPLE_CONFIG_SUCCESS);
    }
    else if (strstr(buf, "M1553_BC_WriteData:") != NULL) /* M1553_BC_WriteData */
    {
        int MsgNumber = 0, length = 0, msgdata[40] = {0};
        printf("buf:%s\n", buf);
        k = 0;
        for (i = 19; i < n; i++)
        {
            revbuf[k++] = buf[i];
            if (buf[i] == ',')
            {
                i++;
                break;
            }
        }
        revbuf[k] = '\0';
        chan = atoi(revbuf);
        printf("chan is :[%d]\n", chan);

        k = 0;
        for (; i < n; i++)
        {
            revbuf1[k++] = buf[i];
            if (buf[i] == ',')
            {
                i++;
                break;
            }
        }
        revbuf1[k] = '\0';
        MsgNumber = atoi(revbuf1);
        printf("MsgNumber is :[%d]\n", MsgNumber);

        k = 0;
        for (; i < n; i++)
        {
            revbuf1[k++] = buf[i];
            if (buf[i] == ',')
            {
                i++;
                break;
            }
        }
        revbuf1[k] = '\0';
        length = atoi(revbuf1);
        printf("length is :[%d]\n", length);

        for (j = 0; j < 40; j++)
        {
            if (i >= n)
                break;
            k = 0; //msgdata
            for (; i < n; i++)
            {
                revbuf1[k++] = buf[i];
                if ((buf[i] == '?') || (buf[i] == ','))
                {
                    i++;
                    break;
                }
            }
            revbuf1[k] = '\0';
            msgdata[j] = atoi(revbuf1);
            printf("timeval[%d] is :[%d]\n", j, msgdata[j]);
        }

        readvalue = M1553_BC_WriteData(chan, MsgNumber, msgdata, length);

        if (readvalue < 0)
            printf("M1553_BC_WriteData:%s;\n", SAMPLE_CONFIG_FAIL);
        else
            printf("M1553_BC_WriteData:%s;\n", SAMPLE_CONFIG_SUCCESS);
    }
    else if (strstr(buf, "M1553_BC_Start:") != NULL) /* M1553_BC_Start */
    {
        int repeat_policy = 0;
        printf("buf:%s\n", buf);
        k = 0;
        for (i = 15; i < n; i++)
        {
            revbuf[k++] = buf[i];
            if (buf[i] == ',')
            {
                i++;
                break;
            }
        }
        revbuf[k] = '\0';
        chan = atoi(revbuf);
        printf("chan is :[%d]\n", chan);

        k = 0;
        for (; i < n; i++)
        {
            revbuf1[k++] = buf[i];
            if (buf[i] == '?')
            {
                i++;
                break;
            }
        }
        revbuf1[k] = '\0';
        repeat_policy = atoi(revbuf1);
        printf("repeat_policy is :[%d]\n", repeat_policy);

        readvalue = M1553_BC_Start(chan, repeat_policy);

        if (readvalue < 0)
            printf("M1553_BC_Start:%s;\n", SAMPLE_CONFIG_FAIL);
        else
            printf("M1553_BC_Start:%s;\n", SAMPLE_CONFIG_SUCCESS);
    }
    else if (strstr(buf, "M1553_BC_Stop:") != NULL) /* M1553_BC_Stop */
    {
        int repeat_policy = 0;
        printf("buf:%s\n", buf);
        k = 0;
        for (i = 14; i < n; i++)
        {
            revbuf[k++] = buf[i];
            if (buf[i] == '?')
            {
                i++;
                break;
            }
        }
        revbuf[k] = '\0';
        chan = atoi(revbuf);
        printf("chan is :[%d]\n", chan);

        readvalue = M1553_BC_Stop(chan);

        if (readvalue < 0)
            printf("M1553_BC_Stop:%s;\n", SAMPLE_CONFIG_FAIL);
        else
            printf("M1553_BC_Stop:%s;\n", SAMPLE_CONFIG_SUCCESS);
    }
    else if (strstr(buf, "M1553_BC_ReceData:") != NULL) /* M1553_BC_ReceData */
    {
        int MsgNumber = 0, reclenth = 0, bcdata[40] = {0};
        printf("buf:%s\n", buf);
        k = 0;
        for (i = 18; i < n; i++)
        {
            revbuf[k++] = buf[i];
            if (buf[i] == ',')
            {
                i++;
                break;
            }
        }
        revbuf[k] = '\0';
        chan = atoi(revbuf);
        printf("chan is :[%d]\n", chan);

        k = 0;
        for (; i < n; i++)
        {
            revbuf1[k++] = buf[i];
            if (buf[i] == ',')
            {
                i++;
                break;
            }
        }
        revbuf1[k] = '\0';
        MsgNumber = atoi(revbuf1);
        printf("MsgNumber is :[%d]\n", MsgNumber);

        k = 0;
        for (; i < n; i++)
        {
            revbuf1[k++] = buf[i];
            if (buf[i] == '?')
            {
                i++;
                break;
            }
        }
        revbuf1[k] = '\0';
        reclenth = atoi(revbuf1);
        printf("reclenth is :[%d]\n", reclenth);

        readvalue = M1553_BC_ReceData(chan, MsgNumber, reclenth, bcdata);

        if (readvalue < 0)
            sprintf(buf1, "M1553_BC_ReceData:%s;\n", SAMPLE_CONFIG_FAIL);
        else
        {
            strcpy(buf1, "M1553_BC_ReceData:");
            strcat(buf1, "192.168.1.100,");

            for (i = 0; i < reclenth; i++)
            {
                sprintf(str, "%04x", bcdata[i]);
                strcat(buf1, str);
            }
            strcat(buf1, ";\n");
        }
        printf("%s", buf1);
    }
    else if (strstr(buf, "M1553_IRQ_Status:") != NULL) /* M1553_IRQ_Status */
    {
        int irq_sta = 0;
        printf("buf:%s\n", buf);
        k = 0;
        for (i = 17; i < n; i++)
        {
            revbuf[k++] = buf[i];
            if (buf[i] == '?')
            {
                i++;
                break;
            }
        }
        revbuf[k] = '\0';
        chan = atoi(revbuf);
        printf("chan is :[%d]\n", chan);

        readvalue = M1553_IRQ_Status(chan, &irq_sta);

        if (readvalue < 0)
            sprintf(buf1, "M1553_IRQ_Status:%s;\n", SAMPLE_CONFIG_FAIL);
        else
        {
            strcpy(buf1, "M1553_IRQ_Status:");
            strcat(buf1, "192.168.1.100,");
            sprintf(str, "%d", irq_sta);
            strcat(buf1, str);
            strcat(buf1, ";\n");
        }
        //sprintf(buf1,"M1553_IRQ_Status:%s;\n",SAMPLE_CONFIG_SUCCESS);

        printf("%s", buf1);
    }
    else if (strstr(buf, "M1553_Clear_IRQ:") != NULL) /* M1553_Clear_IRQ */
    {
        int mode = 0;
        printf("buf:%s\n", buf);
        k = 0;
        for (i = 16; i < n; i++)
        {
            revbuf[k++] = buf[i];
            if (buf[i] == ',')
            {
                i++;
                break;
            }
        }
        revbuf[k] = '\0';
        chan = atoi(revbuf);
        printf("chan is :[%d]\n", chan);

        k = 0;
        for (; i < n; i++)
        {
            revbuf1[k++] = buf[i];
            if (buf[i] == '?')
            {
                i++;
                break;
            }
        }
        revbuf1[k] = '\0';
        mode = atoi(revbuf1);
        printf("mode is :[%d]\n", mode);

        readvalue = M1553_Clear_IRQ(chan, mode);

        if (readvalue < 0)
            printf("M1553_Clear_IRQ:%s;\n", SAMPLE_CONFIG_FAIL);
        else
            printf("M1553_Clear_IRQ:%s;\n", SAMPLE_CONFIG_SUCCESS);
    }
    else if (strstr(buf, "M1553_RT_START:") != NULL) /* M1553_RT_START */
    {
        int RT_ADDRESS = 0, datanum = 0, data[32] = {0};
        printf("buf:%s\n", buf);
        k = 0;
        for (i = 15; i < n; i++)
        {
            revbuf[k++] = buf[i];
            if (buf[i] == ',')
            {
                i++;
                break;
            }
        }
        revbuf[k] = '\0';
        chan = atoi(revbuf);
        printf("chan is :[%d]\n", chan);

        k = 0;
        for (; i < n; i++)
        {
            revbuf1[k++] = buf[i];
            if (buf[i] == ',')
            {
                i++;
                break;
            }
        }
        revbuf1[k] = '\0';
        RT_ADDRESS = atoi(revbuf1);
        printf("RT_ADDRESS is :[%d]\n", RT_ADDRESS);

        k = 0;
        for (; i < n; i++)
        {
            revbuf1[k++] = buf[i];
            if (buf[i] == ',')
            {
                i++;
                break;
            }
        }
        revbuf1[k] = '\0';
        datanum = atoi(revbuf1);
        printf("datanum is :[%d]\n", datanum);

        for (j = 0; j < 32; j++)
        {
            if (i >= n)
                break;
            k = 0; //msgdata
            for (; i < n; i++)
            {
                revbuf1[k++] = buf[i];
                if ((buf[i] == '?') || (buf[i] == ','))
                {
                    i++;
                    break;
                }
            }
            revbuf1[k] = '\0';
            data[j] = atoi(revbuf1);
            printf("data[%d] is :[%d]\n", j, data[j]);
        }

        readvalue = M1553_RT_START(chan, RT_ADDRESS, datanum, data);

        if (readvalue < 0)
            printf("M1553_RT_START:%s;\n", SAMPLE_CONFIG_FAIL);
        else
            printf("M1553_RT_START:%s;\n", SAMPLE_CONFIG_SUCCESS);
    }
    else if (strstr(buf, "M1553_RT_STOP:") != NULL) /* M1553_RT_STOP */
    {
        int mode = 0;
        printf("buf:%s\n", buf);
        k = 0;
        for (i = 14; i < n; i++)
        {
            revbuf[k++] = buf[i];
            if (buf[i] == '?')
            {
                i++;
                break;
            }
        }
        revbuf[k] = '\0';
        chan = atoi(revbuf);
        printf("chan is :[%d]\n", chan);

        readvalue = M1553_RT_STOP(chan);

        if (readvalue < 0)
            printf("M1553_RT_STOP:%s;\n", SAMPLE_CONFIG_FAIL);
        else
            printf("M1553_RT_STOP:%s;\n", SAMPLE_CONFIG_SUCCESS);
    }
    else if (strstr(buf, "M1553_RT_ReceData:") != NULL) /* M1553_RT_ReceData */
    {
        int MsgNumber = 0, reclenth = 0, data[40] = {0};
        printf("buf:%s\n", buf);
        k = 0;
        for (i = 18; i < n; i++)
        {
            revbuf[k++] = buf[i];
            if (buf[i] == '?')
            {
                i++;
                break;
            }
        }
        revbuf[k] = '\0';
        chan = atoi(revbuf);
        printf("chan is :[%d]\n", chan);

        readvalue = M1553_RT_ReceData(chan, data, &reclenth);

        if (readvalue < 0)
            sprintf(buf1, "M1553_RT_ReceData:%s;\n", SAMPLE_CONFIG_FAIL);
        else
        {
            //sprintf(buf1,"M1553_BC_ReceData:%s;\n",SAMPLE_CONFIG_SUCCESS);
            strcpy(buf1, "M1553_RT_ReceData:");
            strcat(buf1, "192.168.1.100,");

            sprintf(str, "%d,", reclenth);
            strcat(buf1, str);

            for (i = 0; i < reclenth; i++)
            {
                sprintf(str, "%04x", data[i]);
                strcat(buf1, str);
            }
            strcat(buf1, ";\n");
        }
        printf("%s", buf1);
    }
    else if (strstr(buf, "M1553_RT_UpData:") != NULL) /* M1553_RT_START */
    {
        int datanum = 0, data[32] = {0};
        printf("buf:%s\n", buf);
        k = 0;
        for (i = 16; i < n; i++)
        {
            revbuf[k++] = buf[i];
            if (buf[i] == ',')
            {
                i++;
                break;
            }
        }
        revbuf[k] = '\0';
        chan = atoi(revbuf);
        printf("chan is :[%d]\n", chan);

        k = 0;
        for (; i < n; i++)
        {
            revbuf1[k++] = buf[i];
            if (buf[i] == ',')
            {
                i++;
                break;
            }
        }
        revbuf1[k] = '\0';
        datanum = atoi(revbuf1);
        printf("datanum is :[%d]\n", datanum);

        for (j = 0; j < 32; j++)
        {
            if (i >= n)
                break;
            k = 0; //msgdata
            for (; i < n; i++)
            {
                revbuf1[k++] = buf[i];
                if ((buf[i] == '?') || (buf[i] == ','))
                {
                    i++;
                    break;
                }
            }
            revbuf1[k] = '\0';
            data[j] = atoi(revbuf1);
            printf("data[%d] is :[%d]\n", j, data[j]);
        }

        readvalue = M1553_RT_UpData(chan, datanum, data);

        if (readvalue < 0)
            printf("M1553_RT_UpData:%s;\n", SAMPLE_CONFIG_FAIL);
        else
            printf("M1553_RT_UpData:%s;\n", SAMPLE_CONFIG_SUCCESS);
    }
}

char para[200];
void myapptest(int argc, char** argv)
{
    if (argc != 2) return;
    strcpy(para, argv[1]);
    rt_thread_t tid1 = rt_thread_create("myapp",
                            myapp,
                            (void *)para,
                            4096*10,
                            20,
                            5);
    /* 如果获得线程控制块，启动这个线程 */
    if(tid1 != RT_NULL)
    {
        rt_thread_startup(tid1);
    }
    else
    {
        printf("rt_thread_create false\n");
    }
}
MSH_CMD_EXPORT(myapptest, myapp test)