#include <rtthread.h>
#include <rthw.h>
#include <stdio.h>
#include "xil_io.h"
#include "drv_rio_uart.h"

static uintptr_t const uart_Handle = 0xb0020000;

static uint32_t UART_Command_Code;
static uint32_t UART_HALF[4] = {0, 0, 0, 0}; //全双工
static uint32_t UART_MODE[4] = {1, 1, 1, 1}; //rs422
static uint32_t UART_TERM[4] = {0, 0, 0, 0}; //无终端电阻

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

int UART_OpenComConfig(uint32_t channel, uint32_t baud, uint32_t parity, uint32_t Data_bit, uint32_t Stop_bit, uint32_t IEEE_Mode, uint32_t Term_Resis, uint32_t Half_Dxp)
{
    uint32_t device_code;
    uint32_t Dupx;
    uint32_t bps, para1, parament;
    uint32_t framecount = 0;
    uint32_t chanencode, i, chanen;
    uint32_t Addr_Reg_Read, Data_Reg_Read, Data_Reg_Readx;
    uint32_t Addr_Reg_Write, Data_Reg_Write;

    UART_Stop_Work(channel);
    rt_hw_us_delay(100);

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

    rt_hw_us_delay(100);
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
    printf("amount is %lu\n", amount);
    for (i = 0; i < amount; i++)
    {
        Addr_Reg_Write = 0x30;
        Data_Reg_Write = (Se_Data[i] & 0xff) + ((channel << 8) & 0xf00);
        Xil_Out32(uart_Handle + Addr_Reg_Write * 4, Data_Reg_Write);
        rt_thread_mdelay(inter_time);
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
