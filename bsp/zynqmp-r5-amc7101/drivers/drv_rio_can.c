#include <rtthread.h>
#include <stdio.h>
#include "xil_io.h"
#include "drv_rio_can.h"

static uintptr_t const can_Handle = 0xb0010000;

CAN_CHCONFIG_PARA gCanCfgPara = {0, 0, 0, 0, 0};
CAN_FLT_CHCONFIG gCanFltCfg = {0, 0, 0, 0};
CAN_MSG_DATA_WR gCanMsgDataWr = {0, 0, 0, 0, 0, NULL, 0};
ST_CAN_FRAME_DESC gStCanFrameDesc;

static int CAN_Write_Reg(uint32_t addres, uint32_t wdata);
static int CAN_Read_Reg(uint32_t addres, uint32_t *rdata);
static int MsgDataRd(uint32_t msgNum[], uint32_t msgDataPtr[], uint32_t bufferSize, uint32_t COUNTREG, uint32_t BUFFERREG);

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

static int CAN_Write_Reg(uint32_t addres, uint32_t wdata)
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

static int CAN_Read_Reg(uint32_t addres, uint32_t *rdata)
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
static int MsgDataRd(uint32_t msgNum[], uint32_t msgDataPtr[], uint32_t bufferSize, uint32_t COUNTREG, uint32_t BUFFERREG)
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

int CAN_MsgDataRd(uint32_t channel, uint32_t *msgNum, uint32_t *frame_id, 
                    uint32_t *ext_frame_id, uint32_t *frame_format, 
                    uint32_t *frame_type, uint8_t data[], uint32_t *data_size)
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
    printf("get_send_stat value:%lu\n", value);
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
