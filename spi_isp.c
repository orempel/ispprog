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

#include "avrdevice.h"
#include "spi_isp.h"
#include "target.h"

#include <util/delay.h>

/* F_CPU /4 (1.8432MHz) */
#define SPI_MODE4       ((1<<SPE) | (1<<MSTR))
/* F_CPU /16 (460.8kHz) */
#define SPI_MODE3       ((1<<SPE) | (1<<MSTR)             | (1<<SPR0))
/* F_CPU /64 (115.2kHz) */
#define SPI_MODE2       ((1<<SPE) | (1<<MSTR) | (1<<SPR1))
/* F_CPU /128 (57.6kHz) */
#define SPI_MODE1       ((1<<SPE) | (1<<MSTR) | (1<<SPR1) | (1<<SPR0))

static const uint8_t m_spi_modes[4] = { SPI_MODE1, SPI_MODE2, SPI_MODE3, SPI_MODE4 };

static uint8_t  m_last_cmd;
static uint8_t  m_last_val;
static uint16_t	m_last_addr;


/* ***********************************************************************
 * spi_rxtx
 * - transmit one byte to target, and return received one
 * *********************************************************************** */
static uint8_t spi_rxtx(uint8_t val)
{
    SPDR = val;
    loop_until_bit_is_set(SPSR, SPIF);
    return SPDR;
} /* spi_rxtx */


/* ***********************************************************************
 * spi_set_clk
 * *********************************************************************** */
void spi_set_clk(uint8_t mode)
{
    static uint8_t spi_speed;

    if (mode == SPI_SET_CLK_MAX)
    {
        spi_speed = (sizeof(m_spi_modes) -1);
    }
    else if ((mode == SPI_SET_CLK_DEC) &&
             (spi_speed > 0)
            )
    {
        spi_speed--;
    }

    /* enable SPI master mode */
    SPCR = m_spi_modes[spi_speed];
} /* spi_set_clk */


/* ***********************************************************************
 * spi_init
 * *********************************************************************** */
void spi_init(uint8_t enable)
{
    if (enable)
    {
        ISP_ACTIVE();
        spi_set_clk(SPI_SET_CLK_MAX);
    }
    else
    {
        ISP_INACTIVE();
        SPCR = 0x00;
    }
} /* spi_init */


/* ***********************************************************************
 * isp_enter_progmode
 * *********************************************************************** */
uint8_t isp_enter_progmode(void)
{
    uint8_t sync;
    spi_rxtx(CMD_PROG_ENABLE_1);
    spi_rxtx(CMD_PROG_ENABLE_2);
    sync = spi_rxtx(0x00);
    spi_rxtx(0x00);

    return (sync == CMD_PROG_ENABLE_2);
} /* isp_enter_progmode */


/* ***********************************************************************
 * isp_mem_write
 * - write a byte to target flash/eeprom
 * *********************************************************************** */
void isp_mem_write(uint8_t cmd, uint16_t addr, uint8_t val)
{
    spi_rxtx(cmd);
    spi_rxtx(addr >> 8);
    spi_rxtx(addr & 0xFF);
    spi_rxtx(val);

    /* remember values for polling */
    m_last_cmd = cmd;
    m_last_addr = addr;
    m_last_val = val;
} /* isp_mem_write */


/* ***********************************************************************
 * isp_mem_read
 * - read a byte from target flash/eeprom
 * *********************************************************************** */
uint8_t isp_mem_read(uint8_t cmd, uint16_t addr)
{
    spi_rxtx(cmd);
    spi_rxtx(addr >> 8);
    spi_rxtx(addr & 0xFF);
    return spi_rxtx(0x00);
} /* isp_mem_read */


/* ***********************************************************************
 * isp_mem_poll
 * - wait until byte/page is written to target memory
 * *********************************************************************** */
void isp_mem_poll(const avr_device_t * p_device)
{
    uint8_t cmd, val, poll = 0xFF;

    if (((m_last_val == 0x00) && (p_device->flags & POLL_00)) ||
        ((m_last_val == 0x7F) && (p_device->flags & POLL_7F)) ||
        ((m_last_val == 0x80) && (p_device->flags & POLL_80)) ||
        ((m_last_val == 0xFF) && (p_device->flags & POLL_FF))
       ) {
        /* wait default time */
        _delay_ms(15);
        return;
    }

    if (m_last_cmd == CMD_WRITE_EEPROM) {
        cmd = CMD_READ_EEPROM;

    } else {
        /* CMD_WRITE_FLASH -> CMD_READ_FLASH */
        cmd = (m_last_cmd & 0x08) | 0x20;
    }

    /* poll until we get correct value */
    do {
        val = isp_mem_read(cmd, m_last_addr);
    } while ((val != m_last_val) && poll--);
} /* isp_mem_poll */


/* ***********************************************************************
 * isp_mem_pagewrite
 * *********************************************************************** */
void isp_mem_pagewrite(void)
{
    spi_rxtx(CMD_WRITE_FLASH_PAGE);
    spi_rxtx(m_last_addr >> 8);
    spi_rxtx(m_last_addr & 0xFF);
    spi_rxtx(0x00);
} /* isp_mem_pagewrite */


/* ***********************************************************************
 * isp_read_signature
 * *********************************************************************** */
void isp_read_signature(uint8_t * p_signature)
{
    uint8_t i;

    for (i = 0; i < 3; i++)
    {
        p_signature[i] = isp_mem_read(CMD_READ_SIG_1, (CMD_READ_SIG_2 << 8) | i);
    }
} /* isp_read_signature */


/* ***********************************************************************
 * isp_cmd3
 * *********************************************************************** */
uint8_t isp_cmd3(uint8_t cmd1, uint8_t cmd2,
                 uint8_t cmd3)
{
    uint8_t result;

    spi_rxtx(cmd1);
    spi_rxtx(cmd2);
    result = spi_rxtx(cmd3);

    _delay_ms(10);
    return result;
} /* isp_cmd3 */


/* ***********************************************************************
 * isp_cmd4
 * *********************************************************************** */
uint8_t isp_cmd4(uint8_t cmd1, uint8_t cmd2,
                 uint8_t cmd3, uint8_t cmd4)
{
    uint8_t result;

    spi_rxtx(cmd1);
    spi_rxtx(cmd2);
    spi_rxtx(cmd3);
    result = spi_rxtx(cmd4);

    _delay_ms(10);
    return result;
} /* isp_cmd3 */
