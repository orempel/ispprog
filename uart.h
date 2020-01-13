#ifndef UART_H_
#define UART_H_

#include <stdint.h>

/* *********************************************************************** */

void    uart_send       (uint8_t data);
uint8_t uart_recv       (void);
uint8_t uart_rx_ready   (void);

void    uart_send_buf   (const uint8_t * p_data, uint16_t data_length);
void    uart_recv_buf   (uint8_t * p_data, uint16_t data_length);

void    uart_init       (void);

/* *********************************************************************** */

#endif /* UART_H_ */
