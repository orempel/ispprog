/***************************************************************************
 *   C based avr910 / avr109 ISP Adapter                                   *
 *                                                                         *
 *   Copyright (C) 2006 - 20011 by Olaf Rempel                             *
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
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <string.h>

#define ARRAY_SIZE(x) (sizeof(x) / sizeof(*x))

/* *********************************************************************** */
#if defined(CONFIG_ispprog)
/*
 * using ATmega16 @7.3728MHz:
 * Fuse H: 0xDA (512 words bootloader, jtag disabled)
 * Fuse L: 0xFF (ext. Crystal)
 */
#define F_CPU           7372800
#define BAUDRATE        115200
#define TIMER_RELOAD    (0xFF - 72)    /* 10ms @7.3728MHz */

#define ISP_RESET       PORTB1        /* to target */
#define ISP_LED         PORTB3        /* low active */
#define ISP_MOSI        PORTB5        /* to target */
#define ISP_MISO        PORTB6        /* to target */
#define ISP_SCK         PORTB7        /* to target */
#define RESET_IN        PORTD3        /* high active */

#define ISP_INACTIVE()  {   /* ISP_SCK, ISP_MOSI and ISP_RESET are inputs */ \
                            DDRB &= ~((1<<ISP_SCK) | (1<<ISP_MOSI) | (1<<ISP_RESET)); \
                            PORTB |= (1<<ISP_RESET); \
                        };

#define ISP_ACTIVE()    {   /* ISP_SCK, ISP_MOSI and ISP_RESET are outputs, set ISP_RESET low */ \
                            DDRB |= ((1<<ISP_SCK) | (1<<ISP_MOSI) | (1<<ISP_RESET)); \
                            PORTB &= ~(1<<ISP_RESET); \
                        };

#define ISP_LED_ON()    { PORTB &= ~(1<<ISP_LED); };
#define ISP_LED_OFF()   { PORTB |= (1<<ISP_LED); };
#define ISP_CHECK()     (PIND & (1<<RESET_IN))

#define GPIO_INIT()     {   /* ISP_RESET and ISP_LED are outputs, pullup SlaveSelect */ \
                            PORTB = (1<<ISP_RESET) | (1<<ISP_LED) | (1<<PORTB4); \
                            DDRB = (1<<ISP_RESET) | (1<<ISP_LED); \
                        };

/* *********************************************************************** */
#elif defined(CONFIG_ispprog2)
/*
 * using ATmega328P @8MHz:
 * Fuse E: 0xFA (2.7V BOD)
 * Fuse H: 0xDC (512 words bootloader)
 * Fuse L: 0xE2 (internal osc)
 */
#define F_CPU           8000000
#define BAUDRATE        115200
#define TIMER_RELOAD    (0xFF - 78)     /* 10ms @8MHz */

/* trim internal oscillator to get "good" baudrate */
#define OSCCAL_VALUE    0x80

#define ISP_RESET       PORTB2          /* to target */
#define ISP_LED         PORTB0          /* high active */
#define ISP_MOSI        PORTB3          /* to target */
#define ISP_MISO        PORTB4          /* to target */
#define ISP_SCK         PORTB5          /* to target */
#define RESET_IN        PORTB1          /* low active */

#define ISP_INACTIVE()  {   /* ISP_SCK, ISP_MOSI are inputs, set ISP_RESET high */ \
                            DDRB &= ~((1<<ISP_SCK) | (1<<ISP_MOSI)); \
                            PORTB |= (1<<ISP_RESET); \
                        };

#define ISP_ACTIVE()    {   /* ISP_SCK, ISP_MOSI and ISP_RESET are outputs, set ISP_RESET low */ \
                            DDRB |= ((1<<ISP_SCK) | (1<<ISP_MOSI)); \
                            PORTB &= ~(1<<ISP_RESET); \
                        };

#define ISP_LED_ON()    { PORTB |= (1<<ISP_LED); };
#define ISP_LED_OFF()   { PORTB &= ~(1<<ISP_LED); };
#define ISP_CHECK()     !(PINB & (1<<RESET_IN))

#define GPIO_INIT()     {   /* ISP_RESET and ISP_LED are outputs, pullup RESET_IN and SlaveSelect */ \
                            PORTB = (1<<ISP_RESET) | (1<<RESET_IN) | (1<<PORTB2); \
                            DDRB = (1<<ISP_RESET) | (1<<ISP_LED); \
                        };

/* *********************************************************************** */
#else
#error "unknown CONFIG"
#endif
/* *********************************************************************** */

#include <util/delay.h>
#define UART_CALC_BAUDRATE(baudRate) (((uint32_t)F_CPU) / (((uint32_t)baudRate)*16) -1)

/* F_CPU /4 (1.8432MHz) */
#define SPI_MODE4       ((1<<SPE) | (1<<MSTR))
/* F_CPU /16 (460.8kHz) */
#define SPI_MODE3       ((1<<SPE) | (1<<MSTR)             | (1<<SPR0))
/* F_CPU /64 (115.2kHz) */
#define SPI_MODE2       ((1<<SPE) | (1<<MSTR) | (1<<SPR1))
/* F_CPU /128 (57.6kHz) */
#define SPI_MODE1       ((1<<SPE) | (1<<MSTR) | (1<<SPR1) | (1<<SPR0))

static const uint8_t spi_modes[4] = { SPI_MODE1, SPI_MODE2, SPI_MODE3, SPI_MODE4 };

#define SPI_SPEED_PROBE 0xFF

static uint8_t spi_speed = SPI_SPEED_PROBE;

#define POLL_00         0x01    /* value 0x00 can not be polled from flash/eeprom */
#define POLL_7F         0x02    /* value 0x7F can not be polled from flash/eeprom */
#define POLL_80         0x04    /* value 0x80 can not be polled from flash/eeprom */
#define POLL_FF         0x08    /* value 0xFF can not be polled from flash/eeprom */

struct _device {
    uint8_t sig[3];     /* device signature */
    uint8_t devcode;    /* avr910 device code */
    uint16_t pagemask;  /* pagemask (pagesize in words!) */
    uint16_t flags;     /* quirks for this device */
};

static struct _device device;

static const struct _device devices[] PROGMEM = {
    { { 0x1E, 0x90, 0x01 }, 0x13, 0x00, POLL_00 | POLL_FF },            /* at90s1200 */
//  { { 0x1E, 0x90, 0x05 }, 0x55, ???, ??? },                           /* tiny12 */
//  { { 0x1E, 0x90, 0x06 }, 0x56, ???, ??? },                           /* tiny15 */
//  { { 0x1E, 0x90, 0x07 }, 0xFF, ???, ??? },                           /* tiny13 */

    { { 0x1E, 0x91, 0x01 }, 0x20, 0x00, POLL_7F | POLL_80 | POLL_FF },  /* at90s2313 */
//  { { 0x1E, 0x91, 0x02 }, 0x48, ???, ??? },                           /* at90s2323 */
//  { { 0x1E, 0x91, 0x03 }, 0x4C, ???, ??? },                           /* at90s2343 */
//  { { 0x1E, 0x91, 0x05 }, 0x34, ???, ??? },                           /* at90s2333 */
    { { 0x1E, 0x91, 0x08 }, 0x20, 0x0F, POLL_FF },                      /* tiny25 (at90s2313 devcode) */
    { { 0x1E, 0x91, 0x09 }, 0x5E, 0x0F, POLL_FF },                      /* tiny26 */
    { { 0x1E, 0x91, 0x0A }, 0x5E, 0x0F, POLL_FF },                      /* tiny2313 (tiny26 devcode) */
    { { 0x1E, 0x91, 0x0B }, 0x20, 0x0F, POLL_FF },                      /* tiny24 (at90s2313 devcode) */
//  { { 0x1E, 0x91, 0x0C }, 0xFF, 0x0F, POLL_FF },                      /* tiny261a */

//  { { 0x1E, 0x92, 0x01 }, 0x28, ???, ??? },                           /* at90s4414 */
//  { { 0x1E, 0x92, 0x02 }, 0x6C, ???, ??? },                           /* at90s4434 */
//  { { 0x1E, 0x92, 0x03 }, 0x30, ???, ??? },                           /* at90s4433 */
//  { { 0x1E, 0x92, 0x05 }, 0xFF, 0x1F, POLL_FF },                      /* mega48 */
    { { 0x1E, 0x92, 0x06 }, 0x20, 0x1F, POLL_FF },                      /* tiny45 (at90s2313 devcode) */
    { { 0x1E, 0x92, 0x07 }, 0x20, 0x1F, POLL_FF },                      /* tiny44 (at90s2313 devcode) */
//  { { 0x1E, 0x92, 0x08 }, 0xFF, 0x1F, POLL_FF },                      /* mega461a */
//  { { 0x1E, 0x92, 0x0A }, 0xFF, 0x1F, POLL_FF },                      /* mega48pa */
//  { { 0x1E, 0x92, 0x0D }, 0x5E, 0x1F, POLL_FF },                      /* tiny4313 (tiny26 devcode) */

    { { 0x1E, 0x93, 0x01 }, 0x38, 0x00, POLL_7F | POLL_80 | POLL_FF },  /* at90s8515 */
//  { { 0x1E, 0x93, 0x03 }, 0x68, ???, ??? },                           /* at90s8535 */
//  { { 0x1E, 0x93, 0x05 }, 0x65, ???, ??? },                           /* mega83 */
//  { { 0x1E, 0x93, 0x06 }, 0x3A, 0x1F, POLL_FF },                      /* mega8515 */
    { { 0x1E, 0x93, 0x07 }, 0x76, 0x1F, POLL_00 | POLL_FF },            /* mega8 */
//  { { 0x1E, 0x93, 0x08 }, 0x69, 0x1F, POLL_FF },                      /* mega8535 */
    { { 0x1E, 0x93, 0x0A }, 0xFF, 0x1F, POLL_FF },                      /* mega88 (no devcode) */
    { { 0x1E, 0x93, 0x0B }, 0x20, 0x1F, POLL_FF },                      /* tiny85 (at90s2313 devcode) */
    { { 0x1E, 0x93, 0x0C }, 0x20, 0x1F, POLL_FF },                      /* tiny84 (at90s2313 devcode) */
//  { { 0x1E, 0x93, 0x0D }, 0xFF, 0x1F, POLL_FF },                      /* tiny861a */
//  { { 0x1E, 0x93, 0x0F }, 0xFF, 0x1F, POLL_FF },                      /* mega88pa */
//  { { 0x1E, 0x93, 0x11 }, 0xFF, 0x1F, POLL_FF },                      /* tiny88 */
//  { { 0x1E, 0x93, 0x81 }, 0xFF, 0x1F, POLL_FF },                      /* at90pwm2/3 */
//  { { 0x1E, 0x93, 0x82 }, 0xFF, 0x3F, POLL_FF },                      /* at90usb82 */
//  { { 0x1E, 0x93, 0x83 }, 0xFF, 0x1F, POLL_FF },                      /* at90pwm2b/3b */
//  { { 0x1E, 0x93, 0x89 }, 0xFF, 0x1F, POLL_FF },                      /* mega8u2 */

//  { { 0x1E, 0x94, 0x01 }, 0x60, ???, ??? },                           /* mega161 */
//  { { 0x1E, 0x94, 0x02 }, 0x64, ???, ??? },                           /* mega163 */
    { { 0x1E, 0x94, 0x03 }, 0x74, 0x3F, POLL_FF },                      /* mega16 */
//  { { 0x1E, 0x94, 0x04 }, 0x63, 0x3F, POLL_FF },                      /* mega162 */
//  { { 0x1E, 0x94, 0x05 }, 0x78, 0x3F, POLL_FF },                      /* mega169 */
    { { 0x1E, 0x94, 0x06 }, 0xFF, 0x3F, POLL_FF },                      /* mega168 (no devcode) */
//  { { 0x1E, 0x94, 0x0A }, 0x74, 0x3F, POLL_FF },                      /* mega164pa (mega16 devcode) */
//  { { 0x1E, 0x94, 0x0B }, 0xFF, 0x3F, POLL_FF },                      /* mega168pa */
//  { { 0x1E, 0x94, 0x0F }, 0xFF, 0x3F, POLL_FF },                      /* mega164a */
//  { { 0x1E, 0x94, 0x82 }, 0xFF, 0x3F, POLL_FF },                      /* at90usb162 */
//  { { 0x1E, 0x94, 0x88 }, 0xFF, 0x3F, POLL_FF },                      /* mega16u4 */
//  { { 0x1E, 0x94, 0x89 }, 0xFF, 0x3F, POLL_FF },                      /* mega16u2 */

    { { 0x1E, 0x95, 0x02 }, 0x72, 0x3F, POLL_FF },                      /* mega32 */
//  { { 0x1E, 0x95, 0x03 }, 0x75, 0x3F, POLL_FF },                      /* mega329 (mega169 devcode) */
//  { { 0x1E, 0x95, 0x04 }, 0x75, 0x3F, POLL_FF },                      /* mega3290 (mega169 devcode) */
//  { { 0x1E, 0x95, 0x05 }, 0x74, 0x3F, POLL_FF },                      /* mega325 (mega16 devcode) */
//  { { 0x1E, 0x95, 0x06 }, 0x74, 0x3F, POLL_FF },                      /* mega3250 (mega16 devcode) */
//  { { 0x1E, 0x95, 0x08 }, 0x74, 0x3F, POLL_FF },                      /* mega324p (mega16 devcode) */
//  { { 0x1E, 0x95, 0x0B }, 0x75, 0x3F, POLL_FF },                      /* mega329p (mega169 devcode) */
//  { { 0x1E, 0x95, 0x0C }, 0x75, 0x3F, POLL_FF },                      /* mega3290p (mega169 devcode) */
    { { 0x1E, 0x95, 0x0F }, 0xFF, 0x3F, POLL_FF },                      /* mega328p (no devcode) */
//  { { 0x1E, 0x95, 0x11 }, 0x74, 0x3F, POLL_FF },                      /* mega324pa (mega16 devcode) */
//  { { 0x1E, 0x95, 0x15 }, 0xFF, 0x3F, POLL_FF },                      /* mega324a */
//  { { 0x1E, 0x95, 0x81 }, 0xFF, 0x3F, POLL_FF },                      /* at90can32 */
    { { 0x1E, 0x95, 0x87 }, 0xFF, 0x3F, POLL_FF },                      /* mega32u4 (no devcode) */
//  { { 0x1E, 0x95, 0x8A }, 0xFF, 0x3F, POLL_FF },                      /* mega32u2 */

    { { 0x1E, 0x96, 0x02 }, 0x45, 0x7F, POLL_FF },                      /* mega64 */
//  { { 0x1E, 0x96, 0x03 }, 0x75, 0x7F, POLL_FF },                      /* mega649 (mega169 devcode) */
//  { { 0x1E, 0x96, 0x04 }, 0x75, 0x7F, POLL_FF },                      /* mega6490 (mega169 devcode) */
//  { { 0x1E, 0x96, 0x05 }, 0x74, 0x7F, POLL_FF },                      /* mega645 (mega16 devcode) */
//  { { 0x1E, 0x96, 0x06 }, 0x74, 0x7F, POLL_FF },                      /* mega6450 (mega16 devcode) */
//  { { 0x1E, 0x96, 0x08 }, 0xFF, 0x7F, POLL_FF },                      /* mega640 */
    { { 0x1E, 0x96, 0x09 }, 0x74, 0x7F, POLL_FF },                      /* mega644a (mega16 devcode) */
    { { 0x1E, 0x96, 0x0A }, 0x74, 0x7F, POLL_FF },                      /* mega644pa (mega16 devcode) */
//  { { 0x1E, 0x96, 0x81 }, 0xFF, 0x7F, POLL_FF },                      /* at90can64 */
//  { { 0x1E, 0x96, 0x82 }, 0xFF, 0x7F, POLL_FF },                      /* at90usb646/647 */

    { { 0x1E, 0x97, 0x01 }, 0x41, 0x7F, POLL_7F | POLL_80 | POLL_FF },  /* mega103 */
    { { 0x1E, 0x97, 0x02 }, 0x43, 0x7F, POLL_FF },                      /* mega128 */
//  { { 0x1E, 0x97, 0x03 }, 0xFF, 0x7F, POLL_FF },                      /* mega1280 */
//  { { 0x1E, 0x97, 0x04 }, 0xFF, 0x7F, POLL_FF },                      /* mega1281 */
//  { { 0x1E, 0x97, 0x05 }, 0x74, 0x7F, POLL_FF },                      /* mega1284p (mega16 devcode) */
//  { { 0x1E, 0x97, 0x06 }, 0xFF, 0x7F, POLL_FF },                      /* mega1284 */
//  { { 0x1E, 0x97, 0x81 }, 0xFF, 0x7F, POLL_FF },                      /* at90can128 */
//  { { 0x1E, 0x97, 0x82 }, 0xFF, 0x7F, POLL_FF },                      /* at90usb1286/1287 */

//  { { 0x1E, 0x98, 0x01 }, 0xFF, ???, ??? },                           /* mega2560 */
//  { { 0x1E, 0x98, 0x02 }, 0xFF, ???, ??? },                           /* mega2561 */

//  { { 0x1E, 0xA7, 0x01 }, 0xFF, ???, ??? },                           /* mega128rfa1 */
};

#define EV_NONE                 0x00
#define EV_STATE_ENTER          0x01
#define EV_BUTTON_PRESSED       0x02
#define EV_BUTTON_RELEASED      0x03
#define EV_TIMEOUT              0x04
#define EV_PROG_ENTER           0x11
#define EV_PROG_LEAVE           0x12

#define STATE_IDLE              0x00    /* nothing */
#define STATE_RESET_SYNC        0x01
#define STATE_RESET_RETRY       0x02
#define STATE_RESET_PROGMODE    0x03


#define LED_OFF                 0x00
#define LED_SLOW                0x20
#define LED_FAST                0x08
#define LED_ON                  0x80
#define LED_SPEED1              0x20
#define LED_SPEED2              0x10
#define LED_SPEED3              0x08
#define LED_SPEED4              0x04

#define CMD_PROG_ENABLE_1       0xAC
#define CMD_PROG_ENABLE_2       0x53
#define CMD_CHIP_ERASE_1        0xAC
#define CMD_CHIP_ERASE_2        0x80
#define CMD_POLL_BUSY_1         0xF0 /* not used */
#define CMD_POLL_BUSY_2         0x00 /* not used */

#define CMD_LOAD_EADDR_1        0x4D /* not used */
#define CMD_LOAD_EADDR_2        0x00 /* not used */
#define CMD_LOAD_FLASH_HI       0x48
#define CMD_LOAD_FLASH_LO       0x40
#define CMD_LOAD_EEPROM_PAGE    0xC1 /* not used */

#define CMD_READ_FLASH_LO       0x20
#define CMD_READ_FLASH_HI       0x28
#define CMD_READ_EEPROM         0xA0
#define CMD_READ_LOCK_1         0x58
#define CMD_READ_LOCK_2         0x00
#define CMD_READ_SIG_1          0x30
#define CMD_READ_SIG_2          0x00
#define CMD_READ_FUSE_1         0x50
#define CMD_READ_FUSE_2         0x00
#define CMD_READ_FUSE_H_1       0x58
#define CMD_READ_FUSE_H_2       0x08
#define CMD_READ_FUSE_E_1       0x50
#define CMD_READ_FUSE_E_2       0x08
#define CMD_READ_CAL            0x38 /* not used */

#define CMD_WRITE_FLASH_PAGE    0x4C
#define CMD_WRITE_EEPROM        0xC0
#define CMD_WRITE_EEPROM_PAGE   0xC2 /* not used */
#define CMD_WRITE_LOCK_1        0xAC
#define CMD_WRITE_LOCK_2        0xE0
#define CMD_WRITE_FUSE_1        0xAC
#define CMD_WRITE_FUSE_2        0xA0
#define CMD_WRITE_FUSE_H_1      0xAC /* not used */
#define CMD_WRITE_FUSE_H_2      0xA8 /* not used */
#define CMD_WRITE_FUSE_E_1      0xAC /* not used */
#define CMD_WRITE_FUSE_E_2      0xA4 /* not used */


static volatile uint8_t led_mode = LED_OFF;

static uint8_t last_cmd;
static uint8_t last_val;
static uint16_t last_addr;

/* Send one byte to PC */
static void ser_send(uint8_t data)
{
#if defined(__AVR_ATmega16__)
    loop_until_bit_is_set(UCSRA, UDRE);
    UDR = data;
#elif defined(__AVR_ATmega328P__)
    loop_until_bit_is_set(UCSR0A, UDRE0);
    UDR0 = data;
#endif
}

/* Receive one byte from PC */
static uint8_t ser_recv(void)
{
#if defined(__AVR_ATmega16__)
    loop_until_bit_is_set(UCSRA, RXC);
    return UDR;
#elif defined(__AVR_ATmega328P__)
    loop_until_bit_is_set(UCSR0A, RXC0);
    return UDR0;
#endif
}

/* Send one byte to target, and return received one */
static uint8_t spi_rxtx(uint8_t val)
{
    SPDR = val;
    loop_until_bit_is_set(SPSR, SPIF);
    return SPDR;
}

/* Control reset and SPI lines */
static void set_reset(uint8_t mode)
{
    if (mode) {
        ISP_INACTIVE();
    } else {
        ISP_ACTIVE();
    }
}

/* writes a byte to target flash/eeprom */
static void mem_write(uint8_t cmd, uint16_t addr, uint8_t val)
{
    spi_rxtx(cmd);
    spi_rxtx(addr >> 8);
    spi_rxtx(addr & 0xFF);
    spi_rxtx(val);

    /* remember values for polling */
    last_cmd = cmd;
    last_addr = addr;
    last_val = val;
}

/* read a byte from target flash/eeprom */
static uint8_t mem_read(uint8_t cmd, uint16_t addr)
{
    spi_rxtx(cmd);
    spi_rxtx(addr >> 8);
    spi_rxtx(addr & 0xFF);
    return spi_rxtx(0x00);
}

/* wait until byte/page is written to target memory */
static void poll(void)
{
    uint8_t cmd, val, poll = 0xFF;

    if (((last_val == 0x00) && (device.flags & POLL_00)) ||
        ((last_val == 0x7F) && (device.flags & POLL_7F)) ||
        ((last_val == 0x80) && (device.flags & POLL_80)) ||
        ((last_val == 0xFF) && (device.flags & POLL_FF))
       ) {
        /* wait default time */
        _delay_ms(15);
        return;
    }

    if (last_cmd == CMD_WRITE_EEPROM) {
        cmd = CMD_READ_EEPROM;

    } else {
        /* CMD_WRITE_FLASH -> CMD_READ_FLASH */
        cmd = (last_cmd & 0x08) | 0x20;
    }

    /* poll until we get correct value */
    do {
        val = mem_read(cmd, last_addr);
    } while ((val != last_val) && poll--);
}


static void mem_pagewrite(uint16_t addr)
{
    spi_rxtx(CMD_WRITE_FLASH_PAGE);
    spi_rxtx(addr >> 8);
    spi_rxtx(addr & 0xFF);
    spi_rxtx(0x00);

    poll();
}

static void reset_statemachine(uint8_t event);
static volatile uint16_t reset_timer = 0x0000;
static volatile uint8_t reset_state;

static void cmdloop(void) __attribute__ ((noreturn));
static void cmdloop(void)
{
    static uint8_t page_buf[256];
    uint16_t addr = 0;

    while (1) {
        switch (ser_recv()) {
        /* Enter programming mode */
        case 'P': {
            reset_statemachine(EV_PROG_ENTER);

            while (1) {
                if (reset_state == STATE_IDLE) {
                    ser_send('!');
                    break;

                } else if (reset_state == STATE_RESET_PROGMODE) {
                    /* device not supported */
                    ser_send('\r');
                    break;
                }
            }
            break;
        }

        /* Autoincrement address */
        case 'a':
            ser_send('Y');
            break;

        /* Set address */
        case 'A':
            addr = (ser_recv() << 8);
            addr |= ser_recv();
            ser_send('\r');
            break;

        /* Write program memory, low byte */
        case 'c':
            led_mode = LED_FAST;
            mem_write(CMD_LOAD_FLASH_LO, addr, ser_recv());

            /* poll on byte addressed targets */
            if (device.pagemask == 0x00) {
                poll();
            }

            ser_send('\r');
            break;

        /* Write program memory, high byte */
        case 'C':
            led_mode = LED_FAST;
            mem_write(CMD_LOAD_FLASH_HI, addr, ser_recv());

            /* poll on byte addressed targets */
            if (device.pagemask == 0x00) {
                poll();
            }

            addr++;
            ser_send('\r');
            break;

        /* Issue Page Write */
        case 'm':
            led_mode = LED_FAST;
            mem_pagewrite(last_addr);
            ser_send('\r');
            break;

        /* Read Lock Bits */
        case 'r':
            ser_send(mem_read(CMD_READ_LOCK_1, CMD_READ_LOCK_2 << 8));
            ser_send('\r');
            break;

        /* Read program memory */
        case 'R':
            led_mode = LED_SLOW;
            ser_send(mem_read(CMD_READ_FLASH_HI, addr));
            ser_send(mem_read(CMD_READ_FLASH_LO, addr));
            addr++;
            break;

        /* Read data memory */
        case 'd':
            led_mode = LED_SLOW;
            ser_send(mem_read(CMD_READ_EEPROM, addr));
            addr++;
            break;

        /* Write data memory */
        case 'D':
            led_mode = LED_FAST;
            mem_write(CMD_WRITE_EEPROM, addr, ser_recv());
            poll();

            addr++;
            ser_send('\r');
            break;

        /* Chip erase */
        case 'e':
            spi_rxtx(CMD_CHIP_ERASE_1);
            spi_rxtx(CMD_CHIP_ERASE_2);
            spi_rxtx(0x00);
            spi_rxtx(0x00);

            _delay_ms(10);
            ser_send('\r');
            break;

        /* Write lock bits */
        case 'l': {
            uint8_t val = ser_recv();
            spi_rxtx(CMD_WRITE_LOCK_1);
            spi_rxtx(CMD_WRITE_LOCK_2);
            spi_rxtx(0x00);
            spi_rxtx(val);

            _delay_ms(10);
            ser_send('\r');
            break;
        }

        /* Read fusebits */
        case 'F':
            ser_send(mem_read(CMD_READ_FUSE_1, CMD_READ_FUSE_2 << 8));
            break;

        /* Read high fusebits */
        case 'N':
            ser_send(mem_read(CMD_READ_FUSE_H_1, CMD_READ_FUSE_H_2 << 8));
            break;

        /* Read extended fusebits */
        case 'Q':
            ser_send(mem_read(CMD_READ_FUSE_E_1, CMD_READ_FUSE_E_2 << 8));
            break;

        /* Leave programming mode */
        case 'L':

        /* Exit Bootloader */
        case 'E':
            reset_statemachine(EV_PROG_LEAVE);
            ser_send('\r');
            break;

        /* Select device type */
        case 'T': {
            ser_recv(); // ignore
            ser_send('\r');
            break;
        }

        /* Read signature bytes */
        case 's': {
            uint8_t i = 2;
            do {
                ser_send(device.sig[i]);
            } while (i--);
            break;
        }

        /* Return supported device codes */
        case 't': {
            uint8_t limit = 0x00;
            while (1) {
                uint8_t i;
                uint8_t search = 0xFF;
                for (i = 0; i < ARRAY_SIZE(devices); i++) {
                    uint8_t devcode = pgm_read_byte(&devices[i].devcode);
                    if ((devcode > limit) && (devcode < search)) {
                        search = devcode;
                    }
                }

                if (search == 0xFF)
                    break;

                ser_send(search);
                limit = search;
            }
            ser_send(0x00);
            break;
        }

        /* Return software identifier */
        case 'S':
            ser_send('A');
            ser_send('V');
            ser_send('R');
            ser_send('-');
            ser_send('I');
            ser_send('S');
            ser_send('P');
            break;

        /* Return software version */
        case 'V':
            ser_send('3');
            ser_send('8');
            break;

        /* Return hardware version */
        case 'v':
            ser_send('1');
            ser_send('2');
            break;

        /* Return programmer type */
        case 'p':
            ser_send('S');
            break;

        /* Set LED */
        case 'x':
            ser_recv();
            led_mode = LED_ON;
            break;

        /* Clear LED */
        case 'y':
            ser_recv();
            led_mode = LED_OFF;
            break;

        /* Report Block write Mode */
        case 'b': {
            ser_send('Y');
            ser_send(sizeof(page_buf) >> 8);
            ser_send(sizeof(page_buf) & 0xFF);
            break;
        }

        /* Block Write */
        case 'B': {
            uint16_t size, i;
            uint8_t type;

            led_mode = LED_FAST;

            size = ser_recv() << 8;
            size |= ser_recv();
            type = ser_recv();

            for (i = 0; i < size; i++)
                page_buf[i] = ser_recv();

            if (type == 'F') {
                for (i = 0; i < size; i += 2) {
                    mem_write(CMD_LOAD_FLASH_LO, addr, page_buf[i]);
                    mem_write(CMD_LOAD_FLASH_HI, addr, page_buf[i+1]);

                    addr++;

                    if ((addr & device.pagemask) == 0x00) {
                        mem_pagewrite(last_addr);
                    }
                }

                if (size != sizeof(page_buf)) {
                    mem_pagewrite(last_addr);
                }

            } else if (type == 'E') {
                for (i = 0; i < size; i++) {
                    mem_write(CMD_WRITE_EEPROM, addr, page_buf[i]);
                    poll();
                    addr++;
                }
            }
            ser_send('\r');
            break;
        }

        /* Block Read */
        case 'g': {
            uint16_t size, i;
            uint8_t type;

            led_mode = LED_SLOW;

            size = ser_recv() << 8;
            size |= ser_recv();
            type = ser_recv();

            if (type == 'F') {
                for (i = 0; i < size; i += 2) {
                    ser_send(mem_read(CMD_READ_FLASH_LO, addr));
                    ser_send(mem_read(CMD_READ_FLASH_HI, addr));
                    addr++;
                }

            } else if (type == 'E') {
                for (i = 0; i < size; i++) {
                    ser_send(mem_read(CMD_READ_EEPROM, addr));
                    addr++;
                }
            }
            break;
        }

        /* Write fusebits */
        case 'f': {
            uint8_t val = ser_recv();
            spi_rxtx(CMD_WRITE_FUSE_1);
            spi_rxtx(CMD_WRITE_FUSE_2);
            spi_rxtx(0x00);
            spi_rxtx(val);

            _delay_ms(10);
            ser_send('\r');
            break;
        }

        /* Universial command */
        case ':': {
            uint8_t val[3];
            val[0] = ser_recv();
            val[1] = ser_recv();
            val[2] = ser_recv();

            spi_rxtx(val[0]);
            spi_rxtx(val[1]);
            ser_send(spi_rxtx(val[2]));

            _delay_ms(10);
            ser_send('\r');
            break;
        }

        /* New universal command */
        case '.': {
            uint8_t val[4];
            val[0] = ser_recv();
            val[1] = ser_recv();
            val[2] = ser_recv();
            val[3] = ser_recv();

            spi_rxtx(val[0]);
            spi_rxtx(val[1]);
            spi_rxtx(val[2]);
            ser_send(spi_rxtx(val[3]));

            _delay_ms(10);
            ser_send('\r');
            break;
        }

        /* ESC */
        case 0x1B:
            break;

        default:
            ser_send('?');
            break;
        }
    }
}

static void reset_statemachine(uint8_t event)
{
    static uint8_t reset_retries;
    static uint8_t reset_cause;

    uint8_t state;
    uint8_t oldstate;
    uint16_t timer;

    cli();
    /* copy state, disable timer */
    state = reset_state;
    timer = reset_timer;
    reset_timer = 0x0000;
    sei();

    do {
        oldstate = state;

        switch (state) {
            case STATE_IDLE:
                if (event == EV_STATE_ENTER) {
                    led_mode = LED_OFF;
                    timer = 0; /* stop timer */

                    /* put device in RUN mode */
                    set_reset(1);

                } else if ((event == EV_BUTTON_PRESSED) || (event == EV_PROG_ENTER)) {
                    memset(&device, 0x00, sizeof(struct _device));
                    reset_retries = 5;
                    reset_cause = event;

                    /* probe SPI speed of device */
                    if (spi_speed == SPI_SPEED_PROBE) {
                        spi_speed = 3;
                    }

                    state = STATE_RESET_SYNC;
                }
                break;

            case STATE_RESET_SYNC:
                if (event == EV_STATE_ENTER) {
                    led_mode = LED_ON;
                    timer = 5; /* timeout 50ms */

                    /* set SPI speed */
                    SPCR = spi_modes[spi_speed];

                    /* put device in ISP mode */
                    set_reset(0);

                } else if (event == EV_TIMEOUT) {
                    uint8_t sync;
                    spi_rxtx(CMD_PROG_ENABLE_1);
                    spi_rxtx(CMD_PROG_ENABLE_2);
                    sync = spi_rxtx(0x00);
                    spi_rxtx(0x00);

                    if (sync == CMD_PROG_ENABLE_2) {
                        uint8_t i;
                        uint8_t sig[3];

                        for (i = 0; i < 3; i++) {
                            sig[i] = mem_read(CMD_READ_SIG_1, (CMD_READ_SIG_2 << 8) | i);
                        }

                        for (i = 0; i < ARRAY_SIZE(devices); i++) {
                            if (memcmp_P(sig, devices[i].sig, sizeof(device.sig)) == 0) {
                                memcpy_P(&device, &devices[i], sizeof(struct _device));
                                break;
                            }
                        }

                        state = (reset_cause == EV_PROG_ENTER) ? STATE_RESET_PROGMODE
                                                               : STATE_IDLE;

                    } else {
                        state = STATE_RESET_RETRY;
                    }
                }
                break;

            case STATE_RESET_RETRY:
                if (event == EV_STATE_ENTER) {
                    led_mode = LED_OFF;
                    timer = 5; /* timeout 50ms */

                    /* put device in RUN mode */
                    set_reset(1);

                } else if (event == EV_TIMEOUT) {
                    reset_retries--;
                    if (reset_retries > 0) {
                        /* try lower frequency */
                        if (spi_speed > 0) {
                            spi_speed--;
                        }

                        state = STATE_RESET_SYNC;

                    } else {
                        /* got no sync, probe speed again next time */
                        spi_speed = SPI_SPEED_PROBE;
                        state = STATE_IDLE;
                    }
                }
                break;

            case STATE_RESET_PROGMODE:
                if (event == EV_STATE_ENTER) {

                } else if (event == EV_PROG_LEAVE) {
                    /* was in prog mode (osc changed?), probe speed next time */
                    spi_speed = SPI_SPEED_PROBE;
                    state = STATE_IDLE;

                } else if (event == EV_BUTTON_PRESSED) {
                    state = STATE_IDLE;
                }
                break;

            default:
                state = STATE_IDLE;
                break;
        }

        event = (oldstate != state) ? EV_STATE_ENTER
                                    : EV_NONE;

    } while (oldstate != state);

    cli();
    /* copy state back */
    reset_timer = timer;
    reset_state = state;
    sei();
}

/* time keeping */
ISR(TIMER0_OVF_vect)
{
    uint8_t event = EV_NONE;

    /* restart timer */
    TCNT0 = TIMER_RELOAD;

    static uint8_t prev_pressed;
    if (ISP_CHECK()) {
        if (!prev_pressed) {
            event = EV_BUTTON_PRESSED;
            prev_pressed = 1;
        }

    } else {
        if (prev_pressed) {
            event = EV_BUTTON_RELEASED;
            prev_pressed = 0;
        }
    }

    if (reset_timer) {
        reset_timer--;
        if (reset_timer == 0) {
            event = EV_TIMEOUT;
        }
    }

    if (event != EV_NONE) {
        reset_statemachine(event);
    }

    /* update LED */
    static uint8_t led_timer;

    if (led_mode & ((led_timer++ & 0xFF) | 0x80)) {
        ISP_LED_ON();
    } else {
        ISP_LED_OFF();
    }
}

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
}
#endif

int main(void) __attribute__ ((noreturn));
int main(void)
{
    GPIO_INIT();

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

    /* enable SPI master mode */
    SPCR = SPI_MODE4;

#if defined(__AVR_ATmega16__)
    /* timer0, FCPU/1024, overflow interrupt */
    TCCR0 = (1<<CS02) | (1<<CS00);
    TIMSK = (1<<TOIE0);
#elif defined(__AVR_ATmega328P__)
    TCCR0B = (1<<CS02) | (1<<CS00);
    TIMSK0 = (1<<TOIE0);
#endif

    /* init statemachine */
    reset_statemachine(EV_STATE_ENTER);

    sei();

    cmdloop();
}
