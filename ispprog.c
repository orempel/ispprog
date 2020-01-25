/***************************************************************************
 *   C based avr910 / avr109 ISP Adapter                                   *
 *                                                                         *
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
#include <avr/interrupt.h>
#include <string.h>

#include "avrdevice.h"
#include "display.h"
#include "spi_isp.h"
#include "target.h"
#include "twi_master.h"
#include "uart.h"

#define MIN(a,b)                (((a) < (b)) ? (a) : (b))

#define TIMER_IRQFREQ_MS        10

/* convert milliseconds to timer ticks */
#define TIMER_MSEC2TICKS(x)     ((x * F_CPU) / (TIMER_DIVISOR * 1000ULL))
#define TIMER_MSEC2IRQCNT(x)    (x / TIMER_IRQFREQ_MS)

#define EV_NONE                 0x00
#define EV_STATE_ENTER          0x01
#define EV_BUTTON_PRESSED       0x02
#define EV_BUTTON_RELEASED      0x04
#define EV_TIMEOUT              0x08
#define EV_PROG_ENTER           0x10
#define EV_PROG_LEAVE           0x20
#define EV_PROG_ENTER_TWI       0x40

#define STATE_IDLE              0x00    /* nothing */
#define STATE_RESET_SYNC        0x01
#define STATE_RESET_RETRY       0x02
#define STATE_RESET_PROGMODE    0x03
#define STATE_TWI_CHECK_BL      0x04
#define STATE_TWI_PROGMODE      0x05

#define LED_OFF                 0x00
#define LED_SLOW                0x20
#define LED_FAST                0x08
#define LED_ON                  0x80


static volatile uint8_t     m_led_mode = LED_OFF;
static volatile uint8_t     m_reset_timer;
static volatile uint8_t     m_events;
static uint8_t              m_state;

static uint8_t              m_page_buf[256];
static avr_device_t         m_device;
static uint16_t             m_address = 0x0000;

#if (USE_TWI_SUPPORT)
static twi_chipinfo_t       m_twi_chipinfo;
static uint8_t              m_twi_address;
#endif /* (USE_TWI_SUPPORT) */


static void reset_statemachine(uint8_t events)
{
    static uint8_t reset_retries;
    static uint8_t reset_cause;

    uint8_t oldstate;
    uint8_t timer;

    /* shortcut: there is nothing to do */
    if ((events == EV_NONE) &&
        (m_events == EV_NONE)
       )
    {
        return;
    }

    cli();
    /* get button/timer events */
    events |= m_events;
    m_events = 0x00;

    /* disable timer */
    timer = m_reset_timer;
    m_reset_timer = 0x0000;
    sei();

    do {
        oldstate = m_state;

        switch (m_state)
        {
            case STATE_IDLE:
                if (events & EV_STATE_ENTER)
                {
                    /* remove all events */
                    events = EV_NONE;

                    /* stop timer */
                    timer = TIMER_MSEC2IRQCNT(0);

                    spi_init(0);
#if (USE_TWI_SUPPORT)
                    twi_init(0);
#endif
                    /* put device in RUN mode */
                    RESET_INACTIVE();
                    m_led_mode = LED_OFF;
                }
                else if ((events & EV_PROG_ENTER) ||
#if (USE_TWI_SUPPORT)
                         ((events & EV_BUTTON_PRESSED) && (m_twi_address == 0x00))
#else
                         (events & EV_BUTTON_PRESSED)
#endif /* (USE_TWI_SUPPORT) */
                        )
                {
                    reset_cause = events;
                    events &= ~(EV_BUTTON_PRESSED | EV_PROG_ENTER);

                    reset_retries = 5;

                    /* enable SPI interface */
                    spi_init(1);

                    m_state = STATE_RESET_SYNC;
                }
#if (USE_TWI_SUPPORT)
                else if ((events & EV_PROG_ENTER_TWI) ||
                         ((events & EV_BUTTON_PRESSED) && (m_twi_address != 0x00))
                        )
                {
                    uint8_t result;

                    reset_cause = events;
                    events &= ~(EV_BUTTON_PRESSED | EV_PROG_ENTER_TWI);

                    reset_retries = 5;

                    twi_init(1);
                    result = twi_switch_application(m_twi_address, BOOTTYPE_BOOTLOADER);
                    if (result == TWI_NACK_ADDR)
                    {
                        /* no response from target, do normal reset */
                        RESET_ACTIVE();
                    }

                    m_state = STATE_TWI_CHECK_BL;
                }
#endif /* (USE_TWI_SUPPORT) */
                break;

            case STATE_RESET_SYNC:
                if (events & EV_STATE_ENTER)
                {
                    events &= ~(EV_STATE_ENTER);

                    timer = TIMER_MSEC2IRQCNT(10);

                    /* put device in ISP mode */
                    RESET_ACTIVE();
                    m_led_mode = LED_ON;
                }
                else if (events & EV_TIMEOUT)
                {
                    events &= ~(EV_TIMEOUT);

                    memset(&m_device, 0x00, sizeof(avr_device_t));

                    if (isp_enter_progmode())
                    {
                        isp_read_signature(m_device.sig);
                        avrdevice_get_by_signature(&m_device, m_device.sig);

                        m_state = STATE_RESET_PROGMODE;
                    }
                    else
                    {
                        m_state = STATE_RESET_RETRY;
                    }
                }
                break;

            case STATE_RESET_RETRY:
                if (events & EV_STATE_ENTER)
                {
                    events &= ~(EV_STATE_ENTER);

                    timer = TIMER_MSEC2IRQCNT(50);

                    /* put device in RUN mode */
                    RESET_INACTIVE();
                    m_led_mode = LED_OFF;
                }
                else if (events & EV_TIMEOUT)
                {
                    events &= ~(EV_TIMEOUT);

                    reset_retries--;
                    if (reset_retries > 0)
                    {
                        /* try lower frequency */
                        spi_set_clk(SPI_SET_CLK_DEC);

                        m_state = STATE_RESET_SYNC;
                    }
                    else
                    {
                        /* got no sync */
                        m_state = STATE_IDLE;
                    }
                }
                break;

            case STATE_RESET_PROGMODE:
                if (events & EV_STATE_ENTER)
                {
                    events &= ~(EV_STATE_ENTER);

                    if (reset_cause == EV_BUTTON_PRESSED)
                    {
                        m_state = STATE_IDLE;
                    }
                }
                else if (events & (EV_PROG_LEAVE | EV_BUTTON_PRESSED))
                {
                    events &= ~(EV_PROG_LEAVE | EV_BUTTON_PRESSED);

                    m_state = STATE_IDLE;
                }
                break;

#if (USE_TWI_SUPPORT)
            case STATE_TWI_CHECK_BL:
                if (events & EV_STATE_ENTER)
                {
                    events &= ~(EV_STATE_ENTER);

                    timer = TIMER_MSEC2IRQCNT(10);
                }
                else if (events & EV_TIMEOUT)
                {
                    uint8_t result;

                    events &= ~(EV_TIMEOUT);

                    /* put target in RUN mode */
                    RESET_INACTIVE();
                    m_led_mode = LED_ON;

                    result = twi_read_chipinfo(m_twi_address, &m_twi_chipinfo);
                    if (result == TWI_SUCCESS)
                    {
#if (USE_DISPLAY)
                        char twi_version[16 +1];

                        twi_read_version(m_twi_address, twi_version,
                                         sizeof(twi_version) -1);
                        twi_version[16] = '\0';

                        display_show_string(twi_version, 0);

                        avrdevice_get_by_signature(&m_device, m_twi_chipinfo.sig);
                        if (m_device.name[0] != '\0')
                        {
                            display_show_string(" ", 1);
                            display_show_string(m_device.name, 1);
                        }
                        else
                        {
                            display_show_string(" 0X", 1);
                            display_show_hex(m_twi_chipinfo.sig[0], 1);
                            display_show_hex(m_twi_chipinfo.sig[1], 1);
                            display_show_hex(m_twi_chipinfo.sig[2], 1);
                        }

                        display_set_mode(DISPLAY_MODE_SCROLL_ONCE);
#endif /* (USE_DISPLAY) */

                        m_state = STATE_TWI_PROGMODE;
                    }
                    else
                    {
                        reset_retries--;
                        if (reset_retries > 0)
                        {
                            timer = TIMER_MSEC2IRQCNT(10);
                        }
                        else
                        {
#if (USE_DISPLAY)
                            display_show_string("0x", 0);
                            display_show_hex(m_twi_address, 1);
                            display_show_string(":NAK", 1);
                            display_set_mode(DISPLAY_MODE_SCROLL_ONCE);
#endif /* (USE_DISPLAY) */

                            m_state = STATE_IDLE;
                        }
                    }
                }
                break;

            case STATE_TWI_PROGMODE:
                if (events & EV_STATE_ENTER)
                {
                    events &= ~(EV_STATE_ENTER);

                    if (reset_cause == EV_BUTTON_PRESSED)
                    {
                        m_state = STATE_IDLE;
                    }
                }
                else if (events & EV_PROG_LEAVE)
                {
                    events &= ~(EV_PROG_LEAVE);

                    m_state = STATE_IDLE;
                }

                if (m_state == STATE_IDLE)
                {
                    twi_switch_application(m_twi_address, BOOTTYPE_APPLICATION);
                }
                break;
#endif /* (USE_TWI_SUPPORT) */

            default:
                m_state = STATE_IDLE;
                break;
        }

#if (USE_DISPLAY)
        if ((m_state == STATE_IDLE) &&
            ((oldstate == STATE_RESET_RETRY) ||
             (oldstate == STATE_RESET_PROGMODE)
            ))
        {
            if (m_device.name[0] != '\0')
            {
                display_show_string(m_device.name, 0);
            }
            else
            {
                display_show_string("unknown 0X", 0);
                display_show_hex(m_device.sig[0], 1);
                display_show_hex(m_device.sig[1], 1);
                display_show_hex(m_device.sig[2], 1);
            }

            display_set_mode(DISPLAY_MODE_SCROLL_ONCE);
        }
#endif /* (USE_DISPLAY) */

        if (oldstate != m_state)
        {
            events |= EV_STATE_ENTER;
        }
    } while (oldstate != m_state);

    cli();
    /* start timer again */
    m_reset_timer = timer;
    sei();
} /* reset_statemachine */


static void reset_statemachine_wait(uint8_t events)
{
    reset_statemachine(events);

    /* wait while timer is running or timer elapsed */
    while (m_reset_timer || m_events)
    {
        reset_statemachine(EV_NONE);
    }
} /* reset_statemachine_wait */


static void cmd_handler_isp(uint8_t cmd)
{
    switch (cmd)
    {
        /* Write program memory, low byte */
        case 'c':
            m_led_mode = LED_FAST;
            isp_mem_write(CMD_LOAD_FLASH_LO, m_address, uart_recv());

            /* poll on byte addressed targets */
            if (m_device.pagemask == 0x00)
            {
                isp_mem_poll(&m_device);
            }

            uart_send('\r');
            break;

        /* Write program memory, high byte */
        case 'C':
            m_led_mode = LED_FAST;
            isp_mem_write(CMD_LOAD_FLASH_HI, m_address, uart_recv());

            /* poll on byte addressed targets */
            if (m_device.pagemask == 0x00)
            {
                isp_mem_poll(&m_device);
            }

            m_address++;
            uart_send('\r');
            break;

        /* Issue Page Write */
        case 'm':
            m_led_mode = LED_FAST;
            isp_mem_pagewrite();
            isp_mem_poll(&m_device);
            uart_send('\r');
            break;

        /* Read Lock Bits */
        case 'r':
            uart_send(isp_mem_read(CMD_READ_LOCK_1, CMD_READ_LOCK_2 << 8));
            uart_send('\r');
            break;

        /* Read program memory */
        case 'R':
            m_led_mode = LED_SLOW;
            uart_send(isp_mem_read(CMD_READ_FLASH_HI, m_address));
            uart_send(isp_mem_read(CMD_READ_FLASH_LO, m_address));
            m_address++;
            break;

        /* Read data memory */
        case 'd':
            m_led_mode = LED_SLOW;
            uart_send(isp_mem_read(CMD_READ_EEPROM, m_address));
            m_address++;
            break;

        /* Write data memory */
        case 'D':
            m_led_mode = LED_FAST;
            isp_mem_write(CMD_WRITE_EEPROM, m_address, uart_recv());
            isp_mem_poll(&m_device);

            m_address++;
            uart_send('\r');
            break;

        /* Chip erase */
        case 'e':
            isp_cmd4(CMD_CHIP_ERASE_1, CMD_CHIP_ERASE_2, 0x00, 0x00);
            uart_send('\r');
            break;

        /* Write lock bits */
        case 'l':
            isp_cmd4(CMD_WRITE_LOCK_1, CMD_WRITE_LOCK_2, 0x00, uart_recv());
            uart_send('\r');
            break;

        /* Read fusebits */
        case 'F':
            uart_send(isp_mem_read(CMD_READ_FUSE_1, CMD_READ_FUSE_2 << 8));
            break;

        /* Read high fusebits */
        case 'N':
            uart_send(isp_mem_read(CMD_READ_FUSE_H_1, CMD_READ_FUSE_H_2 << 8));
            break;

        /* Read extended fusebits */
        case 'Q':
            uart_send(isp_mem_read(CMD_READ_FUSE_E_1, CMD_READ_FUSE_E_2 << 8));
            break;

        /* Read signature bytes */
        case 's':
            uart_send(m_device.sig[2]);
            uart_send(m_device.sig[1]);
            uart_send(m_device.sig[0]);
            break;

        /* Block Write */
        case 'B':
        {
            uint16_t size, i;
            uint8_t type;

            m_led_mode = LED_FAST;

            size = uart_recv() << 8;
            size |= uart_recv();
            type = uart_recv();

            size = MIN(size, sizeof(m_page_buf));
            uart_recv_buf(m_page_buf, size);

            if (type == 'F')
            {
                for (i = 0; i < size; i += 2)
                {
                    isp_mem_write(CMD_LOAD_FLASH_LO, m_address, m_page_buf[i]);
                    isp_mem_write(CMD_LOAD_FLASH_HI, m_address, m_page_buf[i+1]);

                    m_address++;

                    if ((m_address & m_device.pagemask) == 0x00)
                    {
                        isp_mem_pagewrite();
                        isp_mem_poll(&m_device);
                    }
                }

                if ((m_device.pagemask != 0x00) &&
                    (size != ((m_device.pagemask +1) << 1))
                   )
                {
                    isp_mem_pagewrite();
                    isp_mem_poll(&m_device);
                }
            }
            else if (type == 'E')
            {
                for (i = 0; i < size; i++)
                {
                    isp_mem_write(CMD_WRITE_EEPROM, m_address, m_page_buf[i]);
                    isp_mem_poll(&m_device);
                    m_address++;
                }
            }

            uart_send('\r');
            break;
        }

        /* Block Read */
        case 'g':
        {
            uint16_t size, i;
            uint8_t type;

            m_led_mode = LED_SLOW;

            size = uart_recv() << 8;
            size |= uart_recv();
            type = uart_recv();

            if (type == 'F')
            {
                for (i = 0; i < size; i += 2)
                {
                    uart_send(isp_mem_read(CMD_READ_FLASH_LO, m_address));
                    uart_send(isp_mem_read(CMD_READ_FLASH_HI, m_address));
                    m_address++;
                }
            }
            else if (type == 'E')
            {
                for (i = 0; i < size; i++)
                {
                    uart_send(isp_mem_read(CMD_READ_EEPROM, m_address));
                    m_address++;
                }
            }
            break;
        }

        /* Write fusebits */
        case 'f':
            isp_cmd4(CMD_WRITE_FUSE_1, CMD_WRITE_FUSE_2, 0x00, uart_recv());
            uart_send('\r');
            break;

        /* Universial command */
        case ':':
        {
            uint8_t val[3];

            uart_recv_buf(val, sizeof(val));
            uart_send(isp_cmd3(val[0], val[1], val[2]));
            uart_send('\r');
            break;
        }

        /* New universal command */
        case '.':
        {
            uint8_t val[4];

            uart_recv_buf(val, sizeof(val));
            uart_send(isp_cmd4(val[0], val[1], val[2], val[3]));
            uart_send('\r');
            break;
        }

        default:
            uart_send('?');
            break;
    }
} /* cmd_handler_isp */


#if (USE_TWI_SUPPORT)
static void cmd_handler_twi(uint8_t cmd)
{
    switch (cmd)
    {
         /* Chip erase */
        case 'e':
            uart_send('\r');
            break;

        /* Read signature bytes */
        case 's':
            uart_send(m_twi_chipinfo.sig[2]);
            uart_send(m_twi_chipinfo.sig[1]);
            uart_send(m_twi_chipinfo.sig[0]);
            break;

        /* Block Write */
        case 'B':
        {
            uint16_t write_pos = 0;
            uint16_t size;
            uint8_t type;
            uint8_t result = TWI_SUCCESS;

            m_led_mode = LED_FAST;

            size = uart_recv() << 8;
            size |= uart_recv();
            type = uart_recv();

            size = MIN(size, sizeof(m_page_buf));
            uart_recv_buf(m_page_buf, size);

            memset(m_page_buf + size, 0xFF, sizeof(m_page_buf) - size);

            while ((write_pos < size) &&
                   (result == TWI_SUCCESS)
                  )
            {
                if (type == 'F')
                {
                    result = twi_write_memory(m_twi_address, MEMTYPE_FLASH,
                                              (m_address << 1),
                                              m_page_buf + write_pos,
                                              m_twi_chipinfo.page_size);

                    /* when accessing flash, m_address is a word address */
                    m_address += (m_twi_chipinfo.page_size >> 1);
                    write_pos += m_twi_chipinfo.page_size;
                }
                else
                {
                    uint8_t write_size;

                    /* one eeprom byte takes 3.5ms to programm */
                    write_size = MIN(size, 4);

                    result = twi_write_memory(m_twi_address, MEMTYPE_EEPROM,
                                              m_address,
                                              m_page_buf + write_pos,
                                              write_size);

                    /* when accessing eeprom, m_address is a byte address */
                    m_address += write_size;
                    write_pos += write_size;
                }
            }

            uart_send((result == TWI_SUCCESS) ? '\r' : '!');
            break;
        }

        /* Block Read */
        case 'g':
        {
            uint16_t size;
            uint8_t type;

            m_led_mode = LED_SLOW;

            size = uart_recv() << 8;
            size |= uart_recv();
            type = uart_recv();

            size = MIN(size, sizeof(m_page_buf));

            if (type == 'F')
            {
                twi_read_memory(m_twi_address, MEMTYPE_FLASH,
                                (m_address << 1),
                                m_page_buf, size);

                m_address += (size >> 1);
            }
            else
            {
                twi_read_memory(m_twi_address, MEMTYPE_EEPROM,
                                m_address,
                                m_page_buf, size);

                m_address += size;
            }

            uart_send_buf(m_page_buf, size);
            break;
        }

        default:
            uart_send('?');
            break;
    }
} /* cmd_handler_twi */
#endif /* (USE_TWI_SUPPORT) */


static void cmdloop(void) __attribute__ ((noreturn));
static void cmdloop(void)
{
    while (1)
    {
        uint8_t cmd;

        if (!uart_rx_ready())
        {
            reset_statemachine(EV_NONE);
            continue;
        }

#if (USE_DISPLAY)
        if (m_state == STATE_RESET_PROGMODE)
        {
            uint16_t byte_address;

            byte_address = (m_address << 1);
            display_show_hex(byte_address >> 8, 0);
            display_show_hex(byte_address & 0xFF, 1);

            display_set_mode(DISPLAY_MODE_STATIC);
        }
#endif /* (USE_DISPLAY) */

        cmd = uart_recv();
        switch (cmd)
        {
            /* Enter programming mode */
            case 'P':
                reset_statemachine_wait(EV_PROG_ENTER);
                uart_send((m_state == STATE_RESET_PROGMODE) ? '\r' : '!');
#if (USE_TWI_SUPPORT)
                m_twi_address = 0x00;
#endif
                break;

            /* Autoincrement address */
            case 'a':
                uart_send('Y');
                break;

            /* Set address */
            case 'A':
                m_address = (uart_recv() << 8);
                m_address |= uart_recv();
                uart_send('\r');
                break;

            /* Leave programming mode */
            case 'L':

            /* Exit Bootloader */
            case 'E':
                reset_statemachine_wait(EV_PROG_LEAVE);
                uart_send('\r');
                break;

            /* Select device type */
            case 'T':
                uart_recv(); // ignore
                uart_send('\r');
                break;

            /* Return supported device codes */
            case 't':
                avrdevice_iterate_devcodes(uart_send);
                uart_send(0x00);
                break;

            /* Return software identifier */
            case 'S':
                uart_send('A');
                uart_send('V');
                uart_send('R');
                uart_send('-');
                uart_send('I');
                uart_send('S');
                uart_send('P');
                break;

            /* Return software version */
            case 'V':
                uart_send('3');
                uart_send('8');
                break;

            /* Return hardware version */
            case 'v':
                uart_send('1');
                uart_send('2');
                break;

            /* Return programmer type */
            case 'p':
                uart_send('S');
                break;

            /* Set LED */
            case 'x':
                uart_recv();
                m_led_mode = LED_ON;
                break;

            /* Clear LED */
            case 'y':
                uart_recv();
                m_led_mode = LED_OFF;
                break;

            /* Report Block write Mode */
            case 'b':
                uart_send('Y');
                uart_send(sizeof(m_page_buf) >> 8);
                uart_send(sizeof(m_page_buf) & 0xFF);
                break;

#if (USE_TWI_SUPPORT)
            case 'I':
                m_twi_address = uart_recv() & 0x7F;

                if (m_twi_address != 0x00)
                {
                    reset_statemachine_wait(EV_PROG_ENTER_TWI);
                    if (m_state == STATE_TWI_PROGMODE)
                    {
                        uart_send('\r');
                    }
                    else
                    {
                        uart_send('!');
                    }
                }
                else
                {
                    uart_send('\r');
                }
                break;

            case 'i':
            {
                uint8_t twi_addr;
                uint8_t write_size;
                uint8_t read_size;
                uint8_t result;

                twi_addr = uart_recv();
                write_size = uart_recv();
                read_size = uart_recv();

                uart_recv_buf(m_page_buf, write_size);

                result = twi_generic(twi_addr,
                                     m_page_buf, write_size,
                                     m_page_buf, read_size);

                uart_send_buf(m_page_buf, read_size);

                uart_send((result == TWI_SUCCESS) ? '\r' : '!');
                break;
            }
#endif /* (USE_TWI_SUPPORT) */

            /* ESC */
            case 0x1B:
                break;

            default:
#if (USE_TWI_SUPPORT)
                if (m_twi_address != 0x00)
                {
                    cmd_handler_twi(cmd);
                }
                else
#endif /* (USE_TWI_SUPPORT) */
                {
                    cmd_handler_isp(cmd);
                }
                break;
        }
    }
} /* cmdloop */


/* time keeping */
ISR(TIMER0_OVF_vect)
{
    /* restart timer */
    TCNT0 = 0xFF - TIMER_MSEC2TICKS(TIMER_IRQFREQ_MS);

    static uint8_t prev_pressed;
    if (ISP_CHECK())
    {
        if (!prev_pressed)
        {
            m_events |= EV_BUTTON_PRESSED;
            prev_pressed = 1;
        }
    }
    else
    {
        if (prev_pressed)
        {
            m_events |= EV_BUTTON_RELEASED;
            prev_pressed = 0;
        }
    }

    if (m_reset_timer)
    {
        m_reset_timer--;
        if (m_reset_timer == 0)
        {
            m_events |= EV_TIMEOUT;
        }
    }

    /* update LED */
    static uint8_t led_timer;

    if (m_led_mode & ((led_timer++ & 0xFF) | 0x80))
    {
        ISP_LED_ON();
    }
    else
    {
        ISP_LED_OFF();
    }

#if (USE_DISPLAY)
    display_update();
#endif /* (USE_DISPLAY) */
} /* TIMER0_OVF_vect */


#if defined(__AVR_ATmega328P__)
/*
 * For newer devices the watchdog timer remains active even after a
 * system reset. So disable it as soon as possible.
 * automagically called on startup
 */
void disable_wdt_timer(void) __attribute__((naked, section(".init3")));
void disable_wdt_timer(void)
{
    MCUSR = 0;
    WDTCSR = (1<<WDCE) | (1<<WDE);
    WDTCSR = (0<<WDE);
} /* disable_wdt_timer */
#endif /* defined(__AVR_ATmega328P__) */


int main(void)
{
    GPIO_INIT();

    uart_init();

    spi_init(0);

#if (USE_TWI_SUPPORT)
    twi_init(0);
#endif

    TIMER_INIT();

    /* init statemachine */
    reset_statemachine(EV_BUTTON_PRESSED);

    sei();

    cmdloop();
} /* main */
