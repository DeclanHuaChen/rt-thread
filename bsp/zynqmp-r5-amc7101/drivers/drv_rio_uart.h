#ifndef __DRV_RIO_UART_H__
#define __DRV_RIO_UART_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

int UART_Reset();
int UART_TX_Disable(uint32_t channel);
int UART_TX_Enable(uint32_t channel);
int UART_Receive_Data(uint32_t channel, uint32_t amount, uint8_t Chan1_Data[]);
int UART_Send_Data(uint32_t channel, uint32_t amount, uint8_t Se_Data[], uint32_t inter_time);
int UART_Query_FIFO_Status(uint32_t channel, uint32_t *Send_Sta, uint32_t *Rec_Sta);
int UART_Stop_Work(uint32_t channel);
int UART_OpenComConfig(uint32_t channel, uint32_t baud, uint32_t parity, uint32_t Data_bit, uint32_t Stop_bit, uint32_t IEEE_Mode, uint32_t Term_Resis, uint32_t Half_Dxp);

#ifdef __cplusplus
}
#endif

#endif
