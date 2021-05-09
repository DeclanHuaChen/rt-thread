#ifndef __DRV_RIO_CAN_H__
#define __DRV_RIO_CAN_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define GROUP0_RX 0x0000 //channel 0    RX
#define GROUP1_RX 0x0001 //channel 1    RX
#define GROUP2_RX 0x0002 //channel 2    RX
#define GROUP3_RX 0x0003 //channel 3    RX
#define GROUP4_RX 0x0004 //channel 4    RX
#define GROUP5_RX 0x0005 //channel 5    RX
#define GROUP6_RX 0x0006 //channel 6    RX
#define GROUP7_RX 0x0007 //channel 7    RX

#define GROUP0_TX 0x0008 //channel 8    TX
#define GROUP1_TX 0x0009 //channel 9    TX
#define GROUP2_TX 0x000a //channel 10   TX
#define GROUP3_TX 0x000b //channel 11   TX
#define GROUP4_TX 0x0005 //channel 8    TX
#define GROUP5_TX 0x0006 //channel 9    TX
#define GROUP6_TX 0x0007 //channel 10   TX
#define GROUP7_TX 0x0008 //channel 11   TX

typedef struct
{
    int Chan;
    int Baud;
    int Arbitrate;
    int SuccessFlag;
    int Mode;
} CAN_CHCONFIG_PARA;

typedef struct
{
    int Chan;
    int Mask;
    int FrameId;
    int ExtFrameId;
} CAN_FLT_CHCONFIG;

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

extern CAN_CHCONFIG_PARA gCanCfgPara;
extern CAN_FLT_CHCONFIG gCanFltCfg;
extern CAN_MSG_DATA_WR gCanMsgDataWr;
extern ST_CAN_FRAME_DESC gStCanFrameDesc;

int CAN_Reset();
int CAN_MsgDataRd(uint32_t channel, uint32_t *msgNum, uint32_t *frame_id, 
                  uint32_t *ext_frame_id, uint32_t *frame_format, 
                  uint32_t *frame_type, uint8_t data[], uint32_t *data_size);
int CAN_ChConfig(uint32_t channel, uint32_t Baudrate, uint32_t arbitrate, uint32_t success_flag, uint32_t mode);
int CAN_FilterConfig(uint32_t channel, uint32_t mask, uint32_t frame_id, uint32_t ext_frame_id);
int CAN_MsgDataWr(uint32_t channel, uint32_t frame_id, uint32_t ext_frame_id, uint32_t frame_format, 
                  uint32_t frame_type, uint8_t *msgData, uint32_t msg_len);
int CAN_getDataCount(uint32_t channel, uint32_t *count);
int CAN_Start(uint32_t channel);


#ifdef __cplusplus
}
#endif

#endif
