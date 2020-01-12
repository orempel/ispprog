#ifndef AVRDEVICE_H_
#define AVRDEVICE_H_

#include <stdint.h>

/* *********************************************************************** */

#define POLL_00         0x01    /* value 0x00 can not be polled from flash/eeprom */
#define POLL_7F         0x02    /* value 0x7F can not be polled from flash/eeprom */
#define POLL_80         0x04    /* value 0x80 can not be polled from flash/eeprom */
#define POLL_FF         0x08    /* value 0xFF can not be polled from flash/eeprom */

typedef struct avr_device_s
{
    char        name[12];
    uint8_t     sig[3];     /* device signature */
    uint8_t     devcode;    /* avr910 device code */
    uint16_t    pagemask;   /* pagemask (pagesize in words!) */
    uint16_t    flags;      /* quirks for this device */
} avr_device_t;

typedef void (* avrdevice_callback_t)(uint8_t value);

/* *********************************************************************** */

void avrdevice_iterate_devcodes     (avrdevice_callback_t callback);

void avrdevice_get_by_signature     (avr_device_t * p_device,
                                     const uint8_t * p_signature);

/* *********************************************************************** */

#endif /* AVRDEVICE_H_ */
