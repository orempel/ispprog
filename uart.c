/***************************************************************************
 *   Copyright (C) 2006 - 2020 by Olaf Rempel                              *
 *   razzor AT kopf MINUS tisch DOT de                                     *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; version 2 of the License,               *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/
#include <avr/io.h>

#include "target.h"
#include "uart.h"

#define UART_CALC_BAUDRATE(baudRate) (((uint32_t)F_CPU) / (((uint32_t)baudRate)*16) -1)

/* ***********************************************************************
 * uart_send
 * *********************************************************************** */
void uart_send(uint8_t data)
{
#if defined(__AVR_ATmega16__)
    loop_until_bit_is_set(UCSRA, UDRE);
    UDR = data;
#elif defined(__AVR_ATmega328P__)
    loop_until_bit_is_set(UCSR0A, UDRE0);
    UDR0 = data;
#endif
} /* uart_send */


/* ***********************************************************************
 * uart_recv
 * *********************************************************************** */
uint8_t uart_recv(void)
{
#if defined(__AVR_ATmega16__)
    loop_until_bit_is_set(UCSRA, RXC);
    return UDR;
#elif defined(__AVR_ATmega328P__)
    loop_until_bit_is_set(UCSR0A, RXC0);
    return UDR0;
#endif
} /* uart_recv */


/* ***********************************************************************
 * uart_rx_ready
 * *********************************************************************** */
uint8_t uart_rx_ready(void)
{
#if defined(__AVR_ATmega16__)
    return ((UCSRA & (1<<RXC)) == (1<<RXC));
#elif defined(__AVR_ATmega328P__)
    return ((UCSR0A & (1<<RXC0)) == (1<<RXC0));
#endif
} /* uart_rx_ready */


/* ***********************************************************************
 * uart_send_buf
 * *********************************************************************** */
void uart_send_buf(const uint8_t * p_data, uint16_t data_length)
{
    while (data_length--)
    {
        uart_send(*p_data++);
    }
} /* uart_send_buf */


/* ***********************************************************************
 * uart_recv_buf
 * *********************************************************************** */
void uart_recv_buf(uint8_t * p_data, uint16_t data_length)
{
    while (data_length--)
    {
        *p_data++ = uart_recv();
    }
} /* uart_recv_buf */


/* ***********************************************************************
 * uart_init
 * *********************************************************************** */
void uart_init(void)
{
#if defined(OSCCAL_VALUE)
    OSCCAL = OSCCAL_VALUE;
#endif /* defined(OSCCAL_VALUE) */

#if defined(__AVR_ATmega16__)
    /* Set baud rate */
    UBRRH = (UART_CALC_BAUDRATE(BAUDRATE)>>8) & 0xFF;
    UBRRL = (UART_CALC_BAUDRATE(BAUDRATE) & 0xFF);

    /* enable usart with 8n1 */
    UCSRB = (1<<TXEN) | (1<<RXEN);
    UCSRC = (1<<URSEL) | (1<<UCSZ1) | (1<<UCSZ0);
#elif defined(__AVR_ATmega328P__)
    /* Set baud rate */
    UBRR0H = (UART_CALC_BAUDRATE(BAUDRATE)>>8) & 0xFF;
    UBRR0L = (UART_CALC_BAUDRATE(BAUDRATE) & 0xFF);

    /* enable usart with 8n1 */
    UCSR0B = (1<<TXEN0) | (1<<RXEN0);
    UCSR0C = (1<<UCSZ01) | (1<<UCSZ00);
#endif
} /* uart_init */
