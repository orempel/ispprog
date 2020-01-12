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
#include <avr/pgmspace.h>
#include <string.h>

#include "avrdevice.h"

/* *********************************************************************** */

#define ARRAY_SIZE(x) (sizeof(x) / sizeof(*x))

/* *********************************************************************** */

static const avr_device_t devices[] PROGMEM = {
    { "at90s1200",  { 0x1E, 0x90, 0x01 }, 0x13, 0x00, POLL_00 | POLL_FF },

    { "at90s2313",  { 0x1E, 0x91, 0x01 }, 0x20, 0x00, POLL_7F | POLL_80 | POLL_FF },
    { "tiny25",     { 0x1E, 0x91, 0x08 }, 0x20, 0x0F, POLL_FF },                        /* at90s2313 devcode */
    { "tiny26",     { 0x1E, 0x91, 0x09 }, 0x5E, 0x0F, POLL_FF },
    { "tiny2313",   { 0x1E, 0x91, 0x0A }, 0x5E, 0x0F, POLL_FF },                        /* tiny26 devcode */
    { "tiny24",     { 0x1E, 0x91, 0x0B }, 0x20, 0x0F, POLL_FF },                        /* at90s2313 devcode */

    { "tiny45",     { 0x1E, 0x92, 0x06 }, 0x20, 0x1F, POLL_FF },                        /* at90s2313 devcode */
    { "tiny44",     { 0x1E, 0x92, 0x07 }, 0x20, 0x1F, POLL_FF },                        /* at90s2313 devcode */

    { "at90s8515",  { 0x1E, 0x93, 0x01 }, 0x38, 0x00, POLL_7F | POLL_80 | POLL_FF },
    { "mega8",      { 0x1E, 0x93, 0x07 }, 0x76, 0x1F, POLL_FF },
    { "mega88",     { 0x1E, 0x93, 0x0A }, 0xFF, 0x1F, POLL_FF },
    { "tiny85",     { 0x1E, 0x93, 0x0B }, 0x20, 0x1F, POLL_FF },                        /* at90s2313 devcode */
    { "tiny84",     { 0x1E, 0x93, 0x0C }, 0x20, 0x1F, POLL_FF },                        /* at90s2313 devcode */

    { "mega16",     { 0x1E, 0x94, 0x03 }, 0x74, 0x3F, POLL_FF },
    { "mega168",    { 0x1E, 0x94, 0x06 }, 0xFF, 0x3F, POLL_FF },

    { "mega32",     { 0x1E, 0x95, 0x02 }, 0x72, 0x3F, POLL_FF },
    { "mega328p",   { 0x1E, 0x95, 0x0F }, 0xFF, 0x3F, POLL_FF },
    { "mega32u4",   { 0x1E, 0x95, 0x87 }, 0xFF, 0x3F, POLL_FF },
    { "mega32u2",   { 0x1E, 0x95, 0x8A }, 0xFF, 0x3F, POLL_FF },

    { "mega64",     { 0x1E, 0x96, 0x02 }, 0x45, 0x7F, POLL_FF },
    { "mega644a",   { 0x1E, 0x96, 0x09 }, 0x74, 0x7F, POLL_FF },                        /* mega16 devcode */
    { "mega644p",   { 0x1E, 0x96, 0x0A }, 0x74, 0x7F, POLL_FF },                        /* mega16 devcode */

    { "mega103",    { 0x1E, 0x97, 0x01 }, 0x41, 0x7F, POLL_7F | POLL_80 | POLL_FF },
    { "mega128",    { 0x1E, 0x97, 0x02 }, 0x43, 0x7F, POLL_FF },
};


/* ***********************************************************************
 * avrdevice_iterate_devcodes
 * *********************************************************************** */
void avrdevice_iterate_devcodes(avrdevice_callback_t callback)
{
    uint8_t limit = 0x00;

    while (1)
    {
        uint8_t i;
        uint8_t search = 0xFF;

        for (i = 0; i < ARRAY_SIZE(devices); i++)
        {
            uint8_t devcode = pgm_read_byte(&devices[i].devcode);

            if ((devcode > limit) && (devcode < search))
            {
                search = devcode;
            }
        }

        if (search == 0xFF)
        {
            break;
        }

        callback(search);
        limit = search;
    }
} /* avrdevice_iterate_devcodes */


/* ***********************************************************************
 * avrdevice_get_by_signature
 * *********************************************************************** */
void avrdevice_get_by_signature(avr_device_t * p_device,
                                const uint8_t * p_signature)
{
    uint8_t i;

    for (i = 0; i < ARRAY_SIZE(devices); i++)
    {
        if (memcmp_P(p_signature, devices[i].sig, sizeof(p_device->sig)) == 0)
        {
            memcpy_P(p_device, &devices[i], sizeof(avr_device_t));
            break;
        }
    }
} /* avrdevice_get_by_signature */
