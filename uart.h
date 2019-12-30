#ifndef UART_H_
#define UART_H_

#include <stdint.h>

/* *********************************************************************** */

void    uart_send       (uint8_t data);
uint8_t uart_recv       (void);

void    uart_recv_buf   (uint8_t * p_data, uint16_t data_length);

void    uart_init       (void);

/* *********************************************************************** */

#endif /* UART_H_ */
