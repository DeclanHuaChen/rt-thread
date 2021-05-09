#ifndef __DRV_RIO_1553_H__
#define __DRV_RIO_1553_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

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

#ifdef __cplusplus
}
#endif

#endif
