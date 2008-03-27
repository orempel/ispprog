/*
 * C based avr910 / avr109 ISP Adapter
 *
 * (c) 2006-2008 by Olaf Rempel
 * <razzor at kopf minus tisch dot de>
 *
 * using ATmega16 @7.3728MHz:
 * PB1 = /Reset (to target)
 * PB3 = /LED
 * PB5 = MOSI (to target)
 * PB6 = MISO (to target)
 * PB7 = SCK  (to target)
 * PD3 = reset-in
 *
 */
#include <avr/io.h>
#include <avr/interrupt.h>

#define F_CPU		7372800
#define BAUDRATE	115200

/* SPI Clock: F_CPU /4 (1.8432MHz) or /128 (57.6kHz) */
#define SPI_MODE	((1<<SPE) | (1<<MSTR))
//#define SPI_MODE	((1<<SPE) | (1<<MSTR) | (1<<SPR1) | (1<<SPR0))

#define ISP_RESET	PORTB1
#define ISP_LED		PORTB3
#define ISP_MOSI	PORTB5
#define ISP_MISO	PORTB6
#define ISP_SCK		PORTB7
#define RESET_IN	PORTD3

struct {
	uint8_t id;		/* device id */
	uint8_t pagemask;	/* bitmask of one flash-page */
	uint8_t pollcode;	/* value of an empty flash-cell */

} devices[] = {

//	{ 0x20, 0x00, 0x7F },	/* at90s2313 (no paging, reads 0x7F back) */
	{ 0x20, 0x0F, 0xFF },	/* tiny24 (16 words/page) */
	{ 0x38, 0x00, 0x7F },	/* at90s8515 (no paging, reads 0x7F back) */
	{ 0x43, 0x7F, 0xFF },	/* mega128 (128 words/page) */
	{ 0x72,	0x3F, 0xFF },	/* mega32 (64 words/page) */
	{ 0x74,	0x3F, 0xFF },	/* mega16 (64 words/page) */
	{ 0x76,	0x1F, 0xFF },	/* mega8 (32 words/page) */
	{ 0x00, 0x00, 0x00 },
};

#include <util/delay.h>
#define UART_CALC_BAUDRATE(baudRate) (((uint32_t)F_CPU) / (((uint32_t)baudRate)*16) -1)

#define LED_OFF		0x00
#define LED_ON		0x01
#define LED_FAST	0x02
#define LED_SLOW	0x03

#define CMD_PROG_ENABLE_1	0xAC
#define CMD_PROG_ENABLE_2	0x53
#define CMD_CHIP_ERASE_1	0xAC
#define CMD_CHIP_ERASE_2	0x80
#define CMD_POLL_BUSY_1		0xF0 /* not used */
#define CMD_POLL_BUSY_2		0x00 /* not used */

#define CMD_LOAD_EADDR_1	0x4D /* not used */
#define CMD_LOAD_EADDR_2	0x00 /* not used */
#define CMD_LOAD_FLASH_HI	0x48
#define CMD_LOAD_FLASH_LO	0x40
#define CMD_LOAD_EEPROM_PAGE	0xC1 /* not used */

#define CMD_READ_FLASH_LO	0x20
#define CMD_READ_FLASH_HI	0x28
#define CMD_READ_EEPROM		0xA0
#define CMD_READ_LOCK_1		0x58
#define CMD_READ_LOCK_2		0x00
#define CMD_READ_SIG_1		0x30
#define CMD_READ_SIG_2		0x00
#define CMD_READ_FUSE_1		0x50
#define CMD_READ_FUSE_2		0x00
#define CMD_READ_FUSE_H_1	0x58
#define CMD_READ_FUSE_H_2	0x08
#define CMD_READ_FUSE_E_1	0x50
#define CMD_READ_FUSE_E_2	0x08
#define CMD_READ_CAL		0x38 /* not used */

#define CMD_WRITE_FLASH_PAGE	0x4C
#define CMD_WRITE_EEPROM	0xC0
#define CMD_WRITE_EEPROM_PAGE	0xC2 /* not used */
#define CMD_WRITE_LOCK_1	0xAC
#define CMD_WRITE_LOCK_2	0xE0
#define CMD_WRITE_FUSE_1	0xAC
#define CMD_WRITE_FUSE_2	0xA0
#define CMD_WRITE_FUSE_H_1	0xAC /* not used */
#define CMD_WRITE_FUSE_H_2	0xA8 /* not used */
#define CMD_WRITE_FUSE_E_1	0xAC /* not used */
#define CMD_WRITE_FUSE_E_2	0xA4 /* not used */

static uint16_t lastaddr;
static uint8_t lastcmd;
static uint8_t lastval;
static uint8_t pollcode = 0xFF;

/* toggle LED */
ISR(SIG_OUTPUT_COMPARE1A)
{
	PORTB ^= (1<<ISP_LED);
}

/* Send one byte to PC */
static void ser_send(uint8_t data)
{
	loop_until_bit_is_set(UCSRA, UDRIE);
	UDR = data;
}

/* Receive one byte from PC */
static uint8_t ser_recv(void)
{
	loop_until_bit_is_set(UCSRA, RXC);
	return UDR;
}

/* Check if receiver ready */
static uint8_t ser_recv_ready(void)
{
	return bit_is_set(UCSRA, RXC);
}

/* Send one byte to target, and return received one */
static uint8_t spi_rxtx(uint8_t val)
{
	SPDR = val;
	loop_until_bit_is_set(SPSR, SPIF);
	return SPDR;
}

/* Set LED mode */
static void led_mode(uint8_t mode)
{
	static uint8_t oldmode = LED_OFF;

	if (mode == oldmode) {
		return;

	} else if (mode == LED_ON) {
		TCCR1B = 0x00;
		PORTB &= ~(1<<ISP_LED);

	} else if (mode == LED_OFF) {
		TCCR1B = 0x00;
		PORTB |= (1<<ISP_LED);

	} else if (mode == LED_FAST) {
		/* timer1: F_CPU /64, CTC mode via OutputCompare A, 10Hz */
		OCR1A = (F_CPU / (64 * 10));
		TCCR1B = (1<<WGM12) | (1<<CS11) | (1<<CS10);

	} else if (mode == LED_SLOW) {
		/* timer1: FCPU /64, CTC mode via OutputCompare A, 4Hz */
		OCR1A = (F_CPU / (64 * 4));
		TCCR1B = (1<<WGM12) | (1<<CS11) | (1<<CS10);
	}
	oldmode = mode;
}

/* Control reset and SPI lines */
static void set_reset(uint8_t mode)
{
	if (mode) {
		/* ISP_SCK, ISP_MOSI and ISP_RESET are inputs */
		DDRB &= ~((1<<ISP_SCK) | (1<<ISP_MOSI) | (1<<ISP_RESET));
		PORTB |= (1<<ISP_RESET);

	} else {
		/*ISP_SCK, ISP_MOSI and ISP_RESET are outputs, set ISP_RESET low */
		DDRB |= ((1<<ISP_SCK) | (1<<ISP_MOSI) | (1<<ISP_RESET));
		PORTB &= ~(1<<ISP_RESET);
	}

	_delay_ms(25);
	_delay_ms(25);
}

/* writes a byte to target flash/eeprom */
static void mem_write(uint8_t cmd, uint16_t addr, uint8_t val)
{
	spi_rxtx(cmd);
	spi_rxtx(addr >> 8);
	spi_rxtx(addr & 0xFF);
	spi_rxtx(val);

	/* remember values for polling */
	lastcmd = cmd;
	lastaddr = addr;
	lastval = val;
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

	if (lastcmd == CMD_WRITE_EEPROM) {
		/* check if we can poll */
		if (lastval == pollcode || lastval == 0x7F || lastval == 0x80) {
			/* wait default time */
			_delay_ms(10);
			return;
		}
		cmd = CMD_READ_EEPROM;

	} else {
		/* check if we can poll */
		if (lastval == pollcode) {
			/* wait default time */
			_delay_ms(10);
			return;
		}
		/* CMD_WRITE_FLASH -> CMD_READ_FLASH */
		cmd = (lastcmd & 0x08) | 0x20;
	}

	/* poll until we get correct value */
	do {
		val = mem_read(cmd, lastaddr);
	} while ((val != lastval) && poll--);
}

int main(void)
{
	static uint8_t page_buf[256];
	uint16_t addr = 0;
	uint8_t device = 0, pagemask = 0;

	/* ISP_RESET and ISP_LED are outputs, pullup SlaveSelect */
	PORTB = (1<<ISP_RESET) | (1<<ISP_LED) | (1<<PORTB4);
	DDRB = (1<<ISP_RESET) | (1<<ISP_LED);

	/* Set baud rate */
	UBRRH = (UART_CALC_BAUDRATE(BAUDRATE)>>8) & 0xFF;
	UBRRL = (UART_CALC_BAUDRATE(BAUDRATE) & 0xFF);

	/* enable usart with 8n1 */
	UCSRB = (1<<TXEN) | (1<<RXEN);
	UCSRC = (1<<URSEL) | (1<<UCSZ1) | (1<<UCSZ0);

	/* enable SPI master mode */
	SPCR = SPI_MODE;

	/* enable timer1 OutputCompare A interrupt (CTC hit) */
	TIMSK = (1<<OCIE1A);
	sei();

	/* disable ISP_RESET */
	set_reset(1);

	while (1) {
		uint8_t pulse = 0;

		while (!ser_recv_ready()) {
			/* reset the target */
			if (PIND & (1<<RESET_IN)) {
				if (!pulse) {
					led_mode(LED_ON);
					set_reset(0);
					set_reset(1);
					led_mode(LED_OFF);
					pulse = 1;
				}

			} else {
				pulse = 0;
			}
		}

		switch (ser_recv()) {
		/* Enter programming mode */
		case 'P': {
			uint8_t sync, count = 0x20;
			led_mode(LED_ON);
			do {
				set_reset(1);
				set_reset(0);

				spi_rxtx(CMD_PROG_ENABLE_1);
				spi_rxtx(CMD_PROG_ENABLE_2);
				sync = spi_rxtx(0x00);
				spi_rxtx(0x00);

			} while (sync != 0x53 && count--);

			ser_send('\r');
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
			led_mode(LED_FAST);
			mem_write(CMD_LOAD_FLASH_LO, addr, ser_recv());

			/* poll on byte addressed targets */
			if (!pagemask)
				poll();

			ser_send('\r');
			break;

		/* Write program memory, high byte */
		case 'C':
			led_mode(LED_FAST);
			mem_write(CMD_LOAD_FLASH_HI, addr, ser_recv());

			/* poll on byte addressed targets */
			if (!pagemask)
				poll();

			addr++;
			ser_send('\r');
			break;

		/* Issue Page Write */
		case 'm':
			led_mode(LED_FAST);
			spi_rxtx(CMD_WRITE_FLASH_PAGE);
			spi_rxtx(lastaddr >> 8);
			spi_rxtx(lastaddr & 0xFF);
			spi_rxtx(0x00);

			poll();

			ser_send('\r');
			break;

		/* Read Lock Bits */
		case 'r':
			ser_send(mem_read(CMD_READ_LOCK_1, CMD_READ_LOCK_2 << 8));
			ser_send('\r');
			break;

		/* Read program memory */
		case 'R':
			led_mode(LED_SLOW);
			ser_send(mem_read(CMD_READ_FLASH_HI, addr));
			ser_send(mem_read(CMD_READ_FLASH_LO, addr));
			addr++;
			break;

		/* Read data memory */
		case 'd':
			led_mode(LED_SLOW);
			ser_send(mem_read(CMD_READ_EEPROM, addr));
			addr++;
			break;

		/* Write data memory */
		case 'D':
			led_mode(LED_FAST);
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

			_delay_ms(25);
			ser_send('\r');
			break;

		/* Write lock bits */
		case 'l': {
			uint8_t val = ser_recv();
			spi_rxtx(CMD_WRITE_LOCK_1);
			spi_rxtx(CMD_WRITE_LOCK_2);
			spi_rxtx(0x00);
			spi_rxtx(val);

			_delay_ms(25);
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
			set_reset(1);
			led_mode(LED_OFF);
			ser_send('\r');
			break;

		/* Select device type */
		case 'T': {
			uint8_t val, i = 0;
			val = ser_recv();

			do {
				if (val == devices[i].id) {
					device = val;
					pagemask = devices[i].pagemask;
					pollcode = devices[i].pollcode;
					break;
				}

			} while (devices[i++].id);

			ser_send('\r');
			break;
		}

		/* Read signature bytes */
		case 's': {
			uint8_t i = 2;
			do {
				ser_send(mem_read(CMD_READ_SIG_1, (CMD_READ_SIG_2 << 8) | i));
			} while (i--);
			break;
		}

		/* Return supported device codes */
		case 't': {
			uint8_t val, i = 0;
			do {
				val = devices[i++].id;
				ser_send(val);
			} while (val);
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
			led_mode(LED_ON);
			break;

		/* Clear LED */
		case 'y':
			ser_recv();
			led_mode(LED_OFF);
			break;

		/* Report Block write Mode */
		case 'b':
			ser_send('Y');
			ser_send(sizeof(page_buf) >> 8);
			ser_send(sizeof(page_buf) & 0xFF);
			break;

		/* Block Write */
		case 'B': {
			uint16_t size, i;
			uint8_t type;

			led_mode(LED_FAST);

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

					/* page write on page-boundry */
					if ((addr & pagemask) == 0x00) {
						spi_rxtx(CMD_WRITE_FLASH_PAGE);
						spi_rxtx(lastaddr >> 8);
						spi_rxtx(lastaddr & 0xFF);
						spi_rxtx(0x00);

						poll();
					}
				}

				/* last page */
				if (size != sizeof(page_buf)) {
					spi_rxtx(CMD_WRITE_FLASH_PAGE);
					spi_rxtx(lastaddr >> 8);
					spi_rxtx(lastaddr & 0xFF);
					spi_rxtx(0x00);

					poll();
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

			led_mode(LED_SLOW);

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

			_delay_ms(25);
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

			_delay_ms(25);
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

			_delay_ms(25);
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
	return 0;
}
