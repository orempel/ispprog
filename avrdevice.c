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
    { "tiny12",     { 0x1E, 0x90, 0x05 }, 0x55, 0x00, POLL_UNTESTED },
    { "tiny15",     { 0x1E, 0x90, 0x06 }, 0x56, 0x00, POLL_UNTESTED },
    { "tiny13",     { 0x1E, 0x90, 0x07 }, 0xFF, 0x00, POLL_UNTESTED },

    { "at90s2313",  { 0x1E, 0x91, 0x01 }, 0x20, 0x00, POLL_7F | POLL_80 | POLL_FF },
    { "at90s2323",  { 0x1E, 0x91, 0x02 }, 0x48, 0x00, POLL_UNTESTED },
    { "at90s2343",  { 0x1E, 0x91, 0x03 }, 0x4C, 0x00, POLL_UNTESTED },
    { "at90s2333",  { 0x1E, 0x91, 0x05 }, 0x34, 0x00, POLL_UNTESTED },
    { "tiny25",     { 0x1E, 0x91, 0x08 }, 0x20, 0x0F, POLL_FF },                        /* at90s2313 devcode */
    { "tiny26",     { 0x1E, 0x91, 0x09 }, 0x5E, 0x0F, POLL_FF },
    { "tiny2313",   { 0x1E, 0x91, 0x0A }, 0x5E, 0x0F, POLL_FF },                        /* tiny26 devcode */
    { "tiny24",     { 0x1E, 0x91, 0x0B }, 0x20, 0x0F, POLL_FF },                        /* at90s2313 devcode */
    { "tiny261a",   { 0x1E, 0x91, 0x0C }, 0xFF, 0x0F, POLL_FF | POLL_UNTESTED },

    { "at90s4414",  { 0x1E, 0x92, 0x01 }, 0x28, 0x00, POLL_UNTESTED },
    { "at90s4434",  { 0x1E, 0x92, 0x02 }, 0x6C, 0x00, POLL_UNTESTED },
    { "at90s4433",  { 0x1E, 0x92, 0x03 }, 0x30, 0x00, POLL_UNTESTED },
    { "mega48",     { 0x1E, 0x92, 0x05 }, 0xFF, 0x1F, POLL_FF | POLL_UNTESTED},
    { "tiny45",     { 0x1E, 0x92, 0x06 }, 0x20, 0x1F, POLL_FF },                        /* at90s2313 devcode */
    { "tiny44",     { 0x1E, 0x92, 0x07 }, 0x20, 0x1F, POLL_FF },                        /* at90s2313 devcode */
    { "mega461a",   { 0x1E, 0x92, 0x08 }, 0xFF, 0x1F, POLL_FF | POLL_UNTESTED },
    { "mega48pa",   { 0x1E, 0x92, 0x0A }, 0xFF, 0x1F, POLL_FF | POLL_UNTESTED },
    { "tiny4313",   { 0x1E, 0x92, 0x0D }, 0x5E, 0x1F, POLL_FF | POLL_UNTESTED },        /* tiny26 devcode */

    { "at90s8515",  { 0x1E, 0x93, 0x01 }, 0x38, 0x00, POLL_7F | POLL_80 | POLL_FF },
    { "at90s8535",  { 0x1E, 0x93, 0x03 }, 0x68, 0x00, POLL_UNTESTED },
    { "mega83",     { 0x1E, 0x93, 0x05 }, 0x65, 0x00, POLL_UNTESTED },
    { "mega8515",   { 0x1E, 0x93, 0x06 }, 0x3A, 0x1F, POLL_FF | POLL_UNTESTED },
    { "mega8",      { 0x1E, 0x93, 0x07 }, 0x76, 0x1F, POLL_FF },
    { "mega8535",   { 0x1E, 0x93, 0x08 }, 0x69, 0x1F, POLL_FF | POLL_UNTESTED },
    { "mega88",     { 0x1E, 0x93, 0x0A }, 0xFF, 0x1F, POLL_FF },
    { "tiny85",     { 0x1E, 0x93, 0x0B }, 0x20, 0x1F, POLL_FF },                        /* at90s2313 devcode */
    { "tiny84",     { 0x1E, 0x93, 0x0C }, 0x20, 0x1F, POLL_FF },                        /* at90s2313 devcode */
    { "tiny861a",   { 0x1E, 0x93, 0x0D }, 0xFF, 0x1F, POLL_FF | POLL_UNTESTED },
    { "mega88pa",   { 0x1E, 0x93, 0x0F }, 0xFF, 0x1F, POLL_FF | POLL_UNTESTED },
    { "tiny88",     { 0x1E, 0x93, 0x11 }, 0xFF, 0x1F, POLL_FF | POLL_UNTESTED },
    { "at90pwm3",   { 0x1E, 0x93, 0x81 }, 0xFF, 0x1F, POLL_FF | POLL_UNTESTED },        /* same: at90pwm2 */
    { "at90usb82",  { 0x1E, 0x93, 0x82 }, 0xFF, 0x1F, POLL_FF | POLL_UNTESTED },
    { "at90pwm3b",  { 0x1E, 0x93, 0x83 }, 0xFF, 0x1F, POLL_FF | POLL_UNTESTED },        /* same: at90pwm2b */
    { "mega8u2",    { 0x1E, 0x93, 0x89 }, 0xFF, 0x1F, POLL_FF | POLL_UNTESTED },

    { "mega161",    { 0x1E, 0x94, 0x01 }, 0x60, 0x00, POLL_UNTESTED },
    { "mega163",    { 0x1E, 0x94, 0x02 }, 0x64, 0x00, POLL_UNTESTED },
    { "mega16",     { 0x1E, 0x94, 0x03 }, 0x74, 0x3F, POLL_FF },
    { "mega162",    { 0x1E, 0x94, 0x04 }, 0x63, 0x3F, POLL_FF | POLL_UNTESTED },
    { "mega169",    { 0x1E, 0x94, 0x05 }, 0x78, 0x3F, POLL_FF | POLL_UNTESTED },
    { "mega168",    { 0x1E, 0x94, 0x06 }, 0xFF, 0x3F, POLL_FF },
    { "mega164pa",  { 0x1E, 0x94, 0x0A }, 0x74, 0x3F, POLL_FF | POLL_UNTESTED },        /* mega16 devcode */
    { "mega168pa",  { 0x1E, 0x94, 0x0B }, 0xFF, 0x3F, POLL_FF | POLL_UNTESTED },
    { "mega164a",   { 0x1E, 0x94, 0x0F }, 0xFF, 0x3F, POLL_FF | POLL_UNTESTED },
    { "at90usb162", { 0x1E, 0x94, 0x82 }, 0xFF, 0x3F, POLL_FF | POLL_UNTESTED },
    { "mega16u4",   { 0x1E, 0x94, 0x88 }, 0xFF, 0x3F, POLL_FF | POLL_UNTESTED },
    { "mega16u2",   { 0x1E, 0x94, 0x89 }, 0xFF, 0x3F, POLL_FF | POLL_UNTESTED },

    { "mega32",     { 0x1E, 0x95, 0x02 }, 0x72, 0x3F, POLL_FF },
    { "mega329",    { 0x1E, 0x95, 0x03 }, 0x75, 0x3F, POLL_FF | POLL_UNTESTED },        /* mega169 devcode */
    { "mega3290",   { 0x1E, 0x95, 0x04 }, 0x75, 0x3F, POLL_FF | POLL_UNTESTED },        /* mega169 devcode */
    { "mega325",    { 0x1E, 0x95, 0x05 }, 0x74, 0x3F, POLL_FF | POLL_UNTESTED },        /* mega16 devcode */
    { "mega3250",   { 0x1E, 0x95, 0x06 }, 0x74, 0x3F, POLL_FF | POLL_UNTESTED },        /* mega16 devcode */
    { "mega324p",   { 0x1E, 0x95, 0x08 }, 0x74, 0x3F, POLL_FF | POLL_UNTESTED },        /* mega16 devcode */
    { "mega329p",   { 0x1E, 0x95, 0x0B }, 0x75, 0x3F, POLL_FF | POLL_UNTESTED },        /* mega169 devcode */
    { "mega3290p",  { 0x1E, 0x95, 0x0C }, 0x75, 0x3F, POLL_FF | POLL_UNTESTED },        /* mega169 devcode */
    { "mega328p",   { 0x1E, 0x95, 0x0F }, 0xFF, 0x3F, POLL_FF },
    { "mega324pa",  { 0x1E, 0x95, 0x11 }, 0x74, 0x3F, POLL_FF | POLL_UNTESTED },        /* mega16 devcode */
    { "mega324a",   { 0x1E, 0x95, 0x15 }, 0xFF, 0x3F, POLL_FF | POLL_UNTESTED },
    { "at90can32",  { 0x1E, 0x95, 0x81 }, 0xFF, 0x3F, POLL_FF | POLL_UNTESTED },
    { "mega32u4",   { 0x1E, 0x95, 0x87 }, 0xFF, 0x3F, POLL_FF },
    { "mega32u2",   { 0x1E, 0x95, 0x8A }, 0xFF, 0x3F, POLL_FF },

    { "mega64",     { 0x1E, 0x96, 0x02 }, 0x45, 0x7F, POLL_FF },
    { "mega649",    { 0x1E, 0x96, 0x03 }, 0x75, 0x7F, POLL_FF | POLL_UNTESTED },        /* mega169 devcode */
    { "mega6490",   { 0x1E, 0x96, 0x04 }, 0x75, 0x7F, POLL_FF | POLL_UNTESTED },        /* mega169 devcode */
    { "mega645",    { 0x1E, 0x96, 0x05 }, 0x74, 0x7F, POLL_FF | POLL_UNTESTED },        /* mega16 devcode */
    { "mega6450",   { 0x1E, 0x96, 0x06 }, 0x74, 0x7F, POLL_FF | POLL_UNTESTED },        /* mega16 devcode */
    { "mega640",    { 0x1E, 0x96, 0x08 }, 0xFF, 0x7F, POLL_FF | POLL_UNTESTED },
    { "mega644a",   { 0x1E, 0x96, 0x09 }, 0x74, 0x7F, POLL_FF },                        /* mega16 devcode */
    { "mega644p",   { 0x1E, 0x96, 0x0A }, 0x74, 0x7F, POLL_FF },                        /* mega16 devcode */
    { "at90can64",  { 0x1E, 0x96, 0x81 }, 0xFF, 0x7F, POLL_FF | POLL_UNTESTED },
    { "at90usb646", { 0x1E, 0x96, 0x82 }, 0xFF, 0x7F, POLL_FF | POLL_UNTESTED },        /* same: at90usb647 */

    { "mega103",    { 0x1E, 0x97, 0x01 }, 0x41, 0x7F, POLL_7F | POLL_80 | POLL_FF },
    { "mega128",    { 0x1E, 0x97, 0x02 }, 0x43, 0x7F, POLL_FF },
    { "mega1280",   { 0x1E, 0x97, 0x03 }, 0xFF, 0x7F, POLL_FF | POLL_UNTESTED },
    { "mega1281",   { 0x1E, 0x97, 0x04 }, 0xFF, 0x7F, POLL_FF | POLL_UNTESTED },
    { "mega1284p",  { 0x1E, 0x97, 0x05 }, 0x74, 0x7F, POLL_FF | POLL_UNTESTED },        /* mega16 devcode) */
    { "mega1284",   { 0x1E, 0x97, 0x06 }, 0xFF, 0x7F, POLL_FF | POLL_UNTESTED },
    { "at90can128", { 0x1E, 0x97, 0x81 }, 0xFF, 0x7F, POLL_FF | POLL_UNTESTED },
    { "at90usb1286",{ 0x1E, 0x97, 0x82 }, 0xFF, 0x7F, POLL_FF | POLL_UNTESTED },        /* same: at90usb1287 */

    { "mega2560",   { 0x1E, 0x98, 0x01 }, 0xFF, 0x00, POLL_UNTESTED },
    { "mega2561",   { 0x1E, 0x98, 0x02 }, 0xFF, 0x00, POLL_UNTESTED },

    { "mega128rfa1",{ 0x1E, 0xA7, 0x01 }, 0xFF, 0x00, POLL_UNTESTED },
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

    /* unknown devices are untested */
    if (p_device->name[0] == '\0')
    {
        p_device->flags |= POLL_UNTESTED;
    }
} /* avrdevice_get_by_signature */
