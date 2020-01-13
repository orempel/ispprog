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
#include <string.h>

#include "target.h"
#include "twi_master.h"

#if (USE_TWI_SUPPORT)

#define TWI_SLA_W(addr)         (addr << 1)
#define TWI_SLA_R(addr)         ((addr << 1) | 0x01)


/* ***********************************************************************
 * twi_start
 * *********************************************************************** */
static uint8_t twi_start(void)
{
    TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);
    loop_until_bit_is_set(TWCR, TWINT);

    switch (TWSR & 0xF8)
    {
        case 0x08:  /* START transmitted */
        case 0x10:  /* repeated START transmitted */
            return TWI_SUCCESS;

        default:
            return TWI_ERROR;
    }
} /* twi_start */


/* ***********************************************************************
 * twi_stop
 * *********************************************************************** */
static void twi_stop(void)
{
    TWCR = (1<<TWINT) | (1<<TWSTO) | (1<<TWEN);
} /* twi_stop */


/* ***********************************************************************
 * twi_master_tx
 * *********************************************************************** */
static uint8_t twi_master_tx(uint8_t value)
{
    TWDR = value;
    TWCR = (1<<TWINT) | (1<<TWEN);
    loop_until_bit_is_set(TWCR, TWINT);

    switch (TWSR & 0xF8)
    {
        case 0x18:  /* SLA+W transmitted, ACK received */
        case 0x28:  /* Data transmitted, ACK received */
        case 0x40:  /* SLA+R transmitted, ACK received */
            return TWI_SUCCESS;

        default:
            return TWI_ERROR;
    }
} /* twi_master_tx */


/* ***********************************************************************
 * twi_master_rx
 * *********************************************************************** */
static uint8_t twi_master_rx(uint8_t * p_value, uint8_t ack)
{
    TWCR = (1<<TWINT) | (1<<TWEN) | ((ack) ? (1<<TWEA) : 0x00);
    loop_until_bit_is_set(TWCR, TWINT);
    *p_value = TWDR;

    switch (TWSR & 0xF8)
    {
        case 0x50:  /* Data received, ACK returned */
        case 0x58:  /* Data received, NAK returned */
            return TWI_SUCCESS;

        default:
            return TWI_ERROR;
    }
} /* twi_master_rx */


/* ***********************************************************************
 * twi_master_start
 * *********************************************************************** */
static uint8_t twi_master_start(uint8_t twi_addr)
{
    if (twi_start() == TWI_SUCCESS)
    {
        if (twi_master_tx(twi_addr) == TWI_SUCCESS)
        {
            return TWI_SUCCESS;
        }
    }

    twi_stop();
    return TWI_ERROR;
} /* twi_master_start */


/* ***********************************************************************
 * twi_generic
 * *********************************************************************** */
uint8_t twi_generic(uint8_t twi_addr,
                    const uint8_t * p_wr_data, uint16_t write_size,
                    uint8_t * p_rd_data, uint16_t read_size)
{
    uint8_t result = TWI_ERROR;

    if (write_size > 0)
    {
        result = twi_master_start(TWI_SLA_W(twi_addr));
        if (result == TWI_SUCCESS)
        {
            uint16_t i;

            for (i = 0; i < write_size; i++)
            {
                result = twi_master_tx(*p_wr_data++);
                if (result != TWI_SUCCESS)
                {
                    break;
                }
            }
        }
    }

    if ((read_size > 0) &&
        (result == TWI_SUCCESS)
       )
    {
        result = twi_master_start(TWI_SLA_R(twi_addr));
        if (result == TWI_SUCCESS)
        {
            uint16_t i;

            for (i = 0; i < read_size; i++)
            {
                uint8_t ack;

                ack = (i != (read_size -1));
                result = twi_master_rx(p_rd_data++, ack);
                if (result != TWI_SUCCESS)
                {
                    break;
                }
            }
        }
    }

    twi_stop();
    return result;
} /* twi_generic */


/* ***********************************************************************
 * twi_switch_application
 * *********************************************************************** */
uint8_t twi_switch_application(uint8_t twi_addr, uint8_t app)
{
    uint8_t cmd[2] = { CMD_SWITCH_APPLICATION, app };

    return twi_generic(twi_addr, cmd, sizeof(cmd), NULL, 0);
} /* twi_switch_application */


/* ***********************************************************************
 * twi_read_version
 * *********************************************************************** */
uint8_t twi_read_version(uint8_t twi_addr, char * p_version,
                         uint8_t version_length)
{
    uint8_t cmd[1] = { CMD_READ_VERSION };

    return twi_generic(twi_addr, cmd, sizeof(cmd),
                       (uint8_t *)p_version, version_length);
} /* twi_read_version */


/* ***********************************************************************
 * twi_read_chipinfo
 * *********************************************************************** */
uint8_t twi_read_chipinfo(uint8_t twi_addr, twi_chipinfo_t * p_chipinfo)
{
    uint8_t cmd[4] = { CMD_READ_MEMORY, MEMTYPE_CHIPINFO, 0x00, 0x00 };

    return twi_generic(twi_addr, cmd, sizeof(cmd),
                       (uint8_t *)p_chipinfo, sizeof(twi_chipinfo_t));
} /* twi_read_chipinfo */


/* ***********************************************************************
 * twi_read_memory
 * *********************************************************************** */
uint8_t twi_read_memory(uint8_t twi_addr, uint8_t memory_type, uint16_t memory_addr,
                        uint8_t * p_data, uint16_t data_length)
{
    uint8_t cmd[4] = { CMD_READ_MEMORY, memory_type,
                       (memory_addr >> 8) & 0xFF,
                       (memory_addr & 0xFF) };

    return twi_generic(twi_addr, cmd, sizeof(cmd), p_data, data_length);
} /* twi_read_memory */


/* ***********************************************************************
 * twi_write_memory
 * *********************************************************************** */
uint8_t twi_write_memory(uint8_t twi_addr, uint8_t memory_type, uint16_t memory_addr,
                         const uint8_t * p_data, uint16_t data_length)
{
    uint8_t cmd[4] = { CMD_WRITE_MEMORY, memory_type,
                       (memory_addr >> 8) & 0xFF,
                       (memory_addr & 0xFF) };
    uint8_t result;

    result = twi_master_start(TWI_SLA_W(twi_addr));
    if (result == TWI_SUCCESS)
    {
        uint16_t i;

        for (i = 0; i < sizeof(cmd); i++)
        {
            result = twi_master_tx(cmd[i]);
            if (result != TWI_SUCCESS)
            {
                break;
            }
        }
    }

    if (result == TWI_SUCCESS)
    {
        uint16_t i;

        for (i = 0; i < data_length; i++)
        {
            result = twi_master_tx(*p_data++);
            if (result != TWI_SUCCESS)
            {
                break;
            }
        }
    }

    twi_stop();
    return result;
} /* twi_read_memory */


/* ***********************************************************************
 * twi_init
 * *********************************************************************** */
void twi_init(uint8_t enable)
{
    if (enable)
    {
        TWBR = ((F_CPU / 100000) -16) / 2;
        TWCR = (1<<TWSTO) | (1<<TWEN);
    }
    else
    {
        TWCR = 0x00;
    }
} /* twi_init */
#endif /* (USE_TWI_SUPPORT) */
