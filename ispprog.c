/*
 * C based avr910 / avr109 ISP Adapter
 *
 * (c) 05/2006 by Olaf Rempel
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
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <avr/interrupt.h>

#define F_CPU		7372800
#define BAUDRATE	115200

#define UART_CALC_BAUDRATE(baudRate) (((uint32_t)F_CPU) / (((uint32_t)baudRate)*16) -1)

// needs F_CPU
#include <util/delay.h>

#define LED_OFF		0x00
#define LED_ON		0x01
#define LED_FAST	0x02
#define LED_SLOW	0x03

struct {
	uint8_t id;			// device id
	uint8_t pagemask;		// bitmask of one flash-page
	uint8_t pollcode;		// value of an empty flash-cell

} devices[] = {

	{ 0x20, 0x00, 0x7F },		// at90s2313 (no paging, reads 0x7F back)
	{ 0x38, 0x00, 0x7F },		// at90s8515 (no paging, reads 0x7F back)
	{ 0x43, 0x7F, 0xFF },		// mega128 (128 words/page)
	{ 0x74,	0x3F, 0xFF },		// mega16 (64 words/page)
	{ 0x00, 0x00, 0x00 },
};

uint16_t lastaddr = 0;
uint8_t lastcmd = 0, lastval = 0, pollcode = 0xFF;

uint8_t page_buf[256];

ISR(SIG_INPUT_CAPTURE1)
{
	// toggle LED
	PORTB ^= (1<<PORTB3);
}

/* Send one byte to PC */
void ser_send(uint8_t data)
{
	loop_until_bit_is_set(UCSRA, UDRIE);
	UDR = data;
}

/* Receive one byte from PC */
uint8_t ser_recv(void)
{
	loop_until_bit_is_set(UCSRA, RXC);
	return UDR;
}

/* Check if receiver ready */
uint8_t ser_recv_ready(void)
{
	return bit_is_set(UCSRA, RXC);
}

/* Send one byte to target, and return received one */
uint8_t spi_rxtx(uint8_t val)
{
	SPDR = val;
	loop_until_bit_is_set(SPSR, SPIF);
	return SPDR;
}

/* Set LED mode */
void led_mode(uint8_t mode)
{
	static uint8_t oldmode = LED_OFF;

	if (mode == oldmode) {
		return;

	} else if (mode == LED_ON) {
		TCCR1B = 0x00;
		PORTB &= ~(1<<PORTB3);

	} else if (mode == LED_OFF) {
		TCCR1B = 0x00;
		PORTB |= (1<<PORTB3);

	} else if (mode == LED_FAST) {
		// 100ms reload
		ICR1H = 0x2D;
		ICR1L = 0x00;

		// timer1 prescaler 64, CTC mode via Input Capture
		TCCR1B = (1<<WGM13) | (1<<WGM12) | (1<<CS11) | (1<<CS10);

	} else if (mode == LED_SLOW) {
		// 250ms reload
		ICR1H = 0x70;
		ICR1L = 0x80;

		// timer1 prescaler 64, CTC mode via Input Capture
		TCCR1B = (1<<WGM13) | (1<<WGM12) | (1<<CS11) | (1<<CS10);
	}
	oldmode = mode;
}

/* Control reset and SPI lines */
void set_reset(uint8_t mode)
{
	// make RESET, SCK and MOSI inputs
	if (mode) {
		DDRB &= ~((1<<PORTB7) | (1<<PORTB5) | (1<<PORTB1));
		PORTB |= (1<<PORTB1);

	// make RESET, SCK & MOSI outputs, set RESET low
	} else {
		DDRB |= ((1<<PORTB7) | (1<<PORTB5) | (1<<PORTB1));
		PORTB &= ~(1<<PORTB1);
	}

	// wait 50ms
	_delay_ms(25);
	_delay_ms(25);
}

/*
 * writes a byte to target flash/eeprom
 * remeber the values for polling
 */
void mem_write(uint8_t cmd, uint16_t addr, uint8_t val)
{
	spi_rxtx(cmd);
	spi_rxtx(addr >> 8);
	spi_rxtx(addr & 0xFF);
	spi_rxtx(val);

	lastcmd = cmd;
	lastaddr = addr;
	lastval = val;
}

/* read a byte from target flash/eeprom */
uint8_t mem_read(uint8_t cmd, uint16_t addr)
{
	spi_rxtx(cmd);
	spi_rxtx(addr >> 8);
	spi_rxtx(addr & 0xFF);
	return spi_rxtx(0x00);
}

/* wait until byte/page is written to target memory */
void poll(void)
{
	uint8_t cmd, val, poll = 0xFF;

	if (lastcmd == 0xC0) {
		// we can not poll, wait default value
		if (lastval == pollcode || lastval == 0x7F || lastval == 0x80) {
			_delay_ms(10);
			return;
		}
		cmd = 0xA0;

	} else {
		// we can not poll, wait default value
		if (lastval == pollcode) {
			_delay_ms(10);
			return;
		}
		cmd = (lastcmd & 0x08) | 0x20;
	}

	/* read until we get correct value */
	do {
		val = mem_read(cmd, lastaddr);
	} while ((val != lastval) && poll--);
}

int main(void)
{
	uint16_t addr = 0;
	uint8_t device = 0, pagemask = 0;

	// reset & activity as outputs, pullup SlaveSelect
	PORTB = (1<<PORTB1) | (1<<PORTB3) | (1<<PORTB4);
	DDRB = (1<<PORTB1) | (1<<PORTB3);

	// Set baud rate
	UBRRH = (UART_CALC_BAUDRATE(BAUDRATE)>>8) & 0xFF;
	UBRRL = (UART_CALC_BAUDRATE(BAUDRATE) & 0xFF);

	// enable rx/tx, 8n1
	UCSRB = (1<<TXEN) | (1<<RXEN);
	UCSRC = (1<<URSEL) | (1<<UCSZ1) | (1<<UCSZ0);

	// SPI enabled, Master mode, F_OSC/4
	SPCR = (1<<SPE) | (1<<MSTR);

	// enable timer1 input capture interrupt (CTC hit)
	TIMSK = (1<<TICIE1);
	sei();

	// disable reset
	set_reset(1);

	while (1) {
		uint8_t pulse = 0;

		while (!ser_recv_ready()) {
			// reset the target
			if (PIND & (1<<PIND3)) {
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
		// Enter programming mode
		case 'P': {
			uint8_t sync, count = 0x20;
			led_mode(LED_ON);
			do {
				set_reset(1);
				set_reset(0);

				spi_rxtx(0xAC);
				spi_rxtx(0x53);
				sync = spi_rxtx(0x00);
				spi_rxtx(0x00);

			} while (sync != 0x53 && count--);

			ser_send('\r');
			break;
		}

		// Autoincrement address
		case 'a':
			ser_send('Y');
			break;

		// Set address
		case 'A':
			addr = (ser_recv() << 8);
			addr |= ser_recv();
			ser_send('\r');
			break;

		// Write program memory, low byte
		case 'c':
			led_mode(LED_FAST);
			mem_write(0x40, addr, ser_recv());

			// poll on byte addressed targets
			if (!pagemask)
				poll();

			ser_send('\r');
			break;

		// Write program memory, high byte
		case 'C':
			led_mode(LED_FAST);
			mem_write(0x48, addr, ser_recv());

			// poll on byte addressed targets
			if (!pagemask)
				poll();

			addr++;
			ser_send('\r');
			break;

		// Issue Page Write
		case 'm':
			led_mode(LED_FAST);
			spi_rxtx(0x4C);
			spi_rxtx(lastaddr >> 8);
			spi_rxtx(lastaddr & 0xFF);
			spi_rxtx(0x00);

			poll();

			ser_send('\r');
			break;

		// Read Lock Bits
		case 'r':
			ser_send(mem_read(0x58, 0x0000));
			ser_send('\r');
			break;

		// Read program memory
		case 'R':
			led_mode(LED_SLOW);
			ser_send(mem_read(0x28, addr));
			ser_send(mem_read(0x20, addr));
			addr++;
			break;

		// Read data memory
		case 'd':
			led_mode(LED_SLOW);
			ser_send(mem_read(0xA0, addr));
			addr++;
			break;

		// Write data memory
		case 'D':
			led_mode(LED_FAST);
			mem_write(0xC0, addr, ser_recv());
			poll();

			addr++;
			ser_send('\r');
			break;

		// Chip erase
		case 'e':
			spi_rxtx(0xAC);
			spi_rxtx(0x80);
			spi_rxtx(0x00);
			spi_rxtx(0x00);

			_delay_ms(25);
			ser_send('\r');
			break;

		// Write lock bits
		// TODO: not implemented
		case 'l':
			ser_recv();
			ser_send('\r');
			break;

		// Read fuse bits
		case 'F':
			spi_rxtx(0x50);
			spi_rxtx(0x00);
			spi_rxtx(0x00);
			ser_send(spi_rxtx(0x00));
			break;

		// Return High Fusebits
		case 'N':
			spi_rxtx(0x58);
			spi_rxtx(0x08);
			spi_rxtx(0x00);
			ser_send(spi_rxtx(0x00));
			break;

		// Return extendet Fusebits
		case 'Q':
			spi_rxtx(0x50);
			spi_rxtx(0x08);
			spi_rxtx(0x00);
			ser_send(spi_rxtx(0x00));
			break;

		// Leave programming mode
		case 'L':

		// Exit
		case 'E':
			set_reset(1);
			led_mode(LED_OFF);
			ser_send('\r');
			break;

		// Select device type
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

		// Read signature bytes
		case 's': {
			uint8_t i = 2;
			do {
				spi_rxtx(0x30);
				spi_rxtx(0x00);
				spi_rxtx(i);
				ser_send(spi_rxtx(0x00));
			} while (i--);
			break;
		}

		// Return supported device codes
		case 't': {
			uint8_t val, i = 0;
			do {
				val = devices[i++].id;
				ser_send(val);
			} while (val);
			break;
		}

		// Return software identifier
		case 'S':
			ser_send('A');
			ser_send('V');
			ser_send('R');
			ser_send('-');
			ser_send('I');
			ser_send('S');
			ser_send('P');
			break;

		// Return software version
		case 'V':
			ser_send('3');
			ser_send('8');
			break;

		// Return hardware version
		case 'v':
			ser_send('1');
			ser_send('2');
			break;

		// Return programmer type
		case 'p':
			ser_send('S');
			break;

		// Set LED
		case 'x':
			ser_recv();
			led_mode(LED_ON);
			break;

		// Clear LED
		case 'y':
			ser_recv();
			led_mode(LED_OFF);
			break;

		// Report Block write Mode
		case 'b':
			ser_send('Y');
			ser_send(sizeof(page_buf) >> 8);
			ser_send(sizeof(page_buf) & 0xFF);
			break;

		// Block Write
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
					mem_write(0x40, addr, page_buf[i]);
					mem_write(0x48, addr, page_buf[i+1]);

					addr++;

					// page write on page-boundry
					if ((addr & pagemask) == 0x00) {
						spi_rxtx(0x4C);
						spi_rxtx(lastaddr >> 8);
						spi_rxtx(lastaddr & 0xFF);
						spi_rxtx(0x00);

						poll();
					}
				}

				// last page
				if (size != sizeof(page_buf)) {
					spi_rxtx(0x4C);
					spi_rxtx(lastaddr >> 8);
					spi_rxtx(lastaddr & 0xFF);
					spi_rxtx(0x00);

					poll();
				}

			} else if (type == 'E') {
				for (i = 0; i < size; i++) {
					mem_write(0xC0, addr, page_buf[i]);
					poll();
					addr++;
				}
			}
			ser_send('\r');
			break;
		}

		// Block Read
		case 'g': {
			uint16_t size, i;
			uint8_t type;

			led_mode(LED_SLOW);

			size = ser_recv() << 8;
			size |= ser_recv();
			type = ser_recv();

			if (type == 'F') {
				for (i = 0; i < size; i += 2) {
					ser_send(mem_read(0x20, addr));
					ser_send(mem_read(0x28, addr));
					addr++;
				}

			} else if (type == 'E') {
				for (i = 0; i < size; i++) {
					ser_send(mem_read(0xA0, addr));
					addr++;
				}
			}
			break;
		}

		// Write fuse bits
		// TODO: implement
		case 'f':
			ser_recv();
			ser_send('\r');
			break;

		// Universial command
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

		// New universal command
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

		// ESC
		case 0x1B:
			break;

		default:
			ser_send('?');
			break;
		}
	}
	return 0;
}
