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

#include "display.h"
#include "target.h"

#if (USE_DISPLAY)
static display_mode_t   m_mode = DISPLAY_MODE_OFF;
static char             m_buffer[32];
static uint8_t          m_buffer_length = 0;
static uint8_t          m_buffer_pos = 0;

/* ***********************************************************************
 * display_putc
 * *********************************************************************** */
static void display_putc(uint8_t pos, char ch)
{
    if (ch >= 'a' && ch <= 'z')
    {
        ch &= ~0x20;
    }

    PORTD = ((ch & 0x7E) << 1);
    PORTC = ((ch & 0x01) << 3) | (3 - (pos & 0x03));
    PORTC |= (1<<DISP_WR);
} /* display_putc */


/* ***********************************************************************
 * display_put4
 * *********************************************************************** */
static void display_put4(const char * p_str)
{
    display_putc(0, *p_str++);
    display_putc(1, *p_str++);
    display_putc(2, *p_str++);
    display_putc(3, *p_str++);
} /* display_put4 */


/* ***********************************************************************
 * _hexnibble
 * *********************************************************************** */
static char _hexnibble(char value)
{
    value &= 0x0F;
    return (value < 0x0A) ? ('0' + value)
                          : ('A' + value - 0x0A);
} /* _hexnibble */


/* ***********************************************************************
 * display_show_string
 * *********************************************************************** */
void display_show_string(const char * p_string, uint8_t append)
{
    char * p_dst = m_buffer;
    uint8_t pos = 0;

    if (append)
    {
        p_dst += m_buffer_length;
        pos = m_buffer_length;
    }

    while ((pos < sizeof(m_buffer) -1) &&
           (*p_string != '\0'))
    {
        *p_dst++ = *p_string++;
        pos++;
    }

    *p_dst++ = ' ';

    m_buffer_length = pos;
    m_buffer_pos = 0;
} /* display_show_string */


/* ***********************************************************************
 * display_show_hex
 * *********************************************************************** */
void display_show_hex(uint8_t value, uint8_t append)
{
    char * p_dst = m_buffer;
    uint8_t pos = 0;

    if (append)
    {
        p_dst += m_buffer_length;
        pos = m_buffer_length;
    }

    if (pos < (sizeof(m_buffer) -3))
    {
        *p_dst++ = _hexnibble(value >> 4);
        *p_dst++ = _hexnibble(value);
        *p_dst++ = ' ';
    }

    m_buffer_length = pos +2;
    m_buffer_pos = 0;
} /* display_show_hex */


/* ***********************************************************************
 * display_set_mode
 * *********************************************************************** */
void display_set_mode(display_mode_t mode)
{
    m_mode = mode;
} /* display_set_mode */


/* ***********************************************************************
 * display_update
 * *********************************************************************** */
void display_update(void)
{
    static uint8_t update_timer;

    update_timer++;

    switch (m_mode)
    {
        default:
        case DISPLAY_MODE_OFF:
            display_put4("   ");
            break;

        case DISPLAY_MODE_RUN_ANIM:
            display_put4("RUN-");
            m_mode = DISPLAY_MODE_RUN_ANIM_LOOP;
            /* fallthrough */

        case DISPLAY_MODE_RUN_ANIM_LOOP:
            switch (update_timer & 0x18)
            {
                case 0x00:
                    display_putc(3, '-');
                    break;

                case 0x08:
                    display_putc(3, '\\');
                    break;

                case 0x10:
                    display_putc(3, '1');
                    break;

                case 0x18:
                    display_putc(3, '/');
                    break;

                default:
                    break;
            }
            break;

        case DISPLAY_MODE_STATIC:
            display_put4(m_buffer);
            break;

        case DISPLAY_MODE_SCROLL:
        case DISPLAY_MODE_SCROLL_ONCE:
            if ((m_buffer_length != 0x00) &&
                (!(update_timer & 0x1F))
               )
            {
                display_put4(m_buffer + m_buffer_pos);

                if (m_buffer_pos <= (m_buffer_length -3))
                {
                    m_buffer_pos++;
                }
                else
                {
                    m_buffer_pos = 0x00;

                    if (m_mode == DISPLAY_MODE_SCROLL_ONCE)
                    {
                        m_mode = DISPLAY_MODE_RUN_ANIM;
                    }
                }
            }
            break;
    }
} /* display_update */
#endif /* (USE_DISPLAY) */
