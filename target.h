#ifndef TARGET_H_
#define TARGET_H_

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

#define USE_DISPLAY     0

#define TIMER_INIT()    {   /* timer0, FCPU/1024, overflow interrupt */ \
                            TCCR0 = (1<<CS02) | (1<<CS00); \
                            TIMSK = (1<<TOIE0); \
                        }

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

/* DL1414 display */
#define USE_DISPLAY     1
#define DISP_WR         PORTC2          /* low active */
#define DISP_A0         PORTC0
#define DISP_A1         PORTC1
#define DISP_D0         PORTC3
#define DISP_D1         PORTD2
#define DISP_D2         PORTD3
#define DISP_D3         PORTD4
#define DISP_D4         PORTD5
#define DISP_D5         PORTD6
#define DISP_D6         PORTD7

#define TIMER_INIT()    {   /* timer0, FCPU/1024, overflow interrupt */ \
                            TCCR0B = (1<<CS02) | (1<<CS00); \
                            TIMSK0 = (1<<TOIE0); \
                        }

#define GPIO_INIT()     {   /* ISP_RESET and ISP_LED are outputs, pullup RESET_IN and SlaveSelect */ \
                            PORTB = (1<<ISP_RESET) | (1<<RESET_IN) | (1<<PORTB2); \
                            DDRB = (1<<ISP_RESET) | (1<<ISP_LED); \
                            \
                            /* all DISP_* pins are outputs, DISP_WR is high */ \
                            DDRC = (1<<DISP_WR) | (1<<DISP_A0) | (1<<DISP_A1) | (1<<DISP_D0); \
                            PORTC = (1<<DISP_WR); \
                            DDRD = 0xFC; \
                        };

/* *********************************************************************** */
#else
#error "unknown CONFIG"
#endif
/* *********************************************************************** */

#endif /* TARGET_H_ */