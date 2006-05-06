#include <avr/io.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#define F_CPU		7372800
#define BAUDRATE	115200

#define UART_CALC_BAUDRATE(baudRate) (((uint32_t)F_CPU) / (((uint32_t)baudRate)*16) -1)

// needs F_CPU
#include <util/delay.h>

#define LED_OFF		0x00
#define LED_ON		0x01

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

void ser_send(uint8_t data)
{
	loop_until_bit_is_set(UCSRA, UDRIE);
	UDR = data;
}

uint8_t ser_recv(void)
{
	loop_until_bit_is_set(UCSRA, RXC);
	return UDR;
}

uint8_t ser_recv_ready(void)
{
	return bit_is_set(UCSRA, RXC);
}

uint8_t spi_rxtx(uint8_t val)
{
	SPDR = val;
	loop_until_bit_is_set(SPSR, SPIF);
	return SPDR;
}

void led_mode(uint8_t mode)
{
	static uint8_t oldmode = LED_OFF;

	if (mode == oldmode) {
		return;

	} else if (mode == LED_ON) {
		PORTB &= ~(1<<PORTB3);

	} else if (mode == LED_OFF) {
		PORTB |= (1<<PORTB3);
	}
	oldmode = mode;
}

void set_reset(uint8_t mode)
{
	// set reset high, make SCK & MOSI inputs
	if (mode) {
		PORTB |= (1<<PORTB1);
		DDRB &= ~((1<<PORTB7) | (1<<PORTB5));

	// set reset low, make SCK & MOSI outputs
	} else {
		PORTB &= ~(1<<PORTB1);
		DDRB |= ((1<<PORTB7) | (1<<PORTB5));
	}

	_delay_ms(25);
	_delay_ms(25);
}

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

uint8_t mem_read(uint8_t cmd, uint16_t addr)
{
	spi_rxtx(cmd);
	spi_rxtx(addr >> 8);
	spi_rxtx(addr & 0xFF);
	return spi_rxtx(0x00);
}

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

	// SPI enabled, Master mode, F_OSC/2
	SPCR = (1<<SPE) | (1<<MSTR);

	// disable reset
	set_reset(1);

	while (1) {
		uint8_t pulse = 0;
		while (!ser_recv_ready()) {
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
		case 'P': {				// done
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
		case 'a':				// done
			ser_send('Y');
			break;

		// Set address
		case 'A':				// done
			addr = (ser_recv() << 8);
			addr |= ser_recv();
			ser_send('\r');
			break;

		// Write program memory, low byte
		case 'c':				// done
			mem_write(0x40, addr, ser_recv());

			if (!pagemask)
				poll();

			ser_send('\r');
			break;

		// Write program memory, high byte
		case 'C':				// done
			mem_write(0x48, addr, ser_recv());

			if (!pagemask)
				poll();

			addr++;
			ser_send('\r');
			break;

		// Issue Page Write
		case 'm':				// done
			spi_rxtx(0x4C);
//			spi_rxtx(addr >> 8);
//			spi_rxtx(addr & 0xFF);
			spi_rxtx(lastaddr >> 8);
			spi_rxtx(lastaddr & 0xFF);
			spi_rxtx(0x00);

			poll();

			ser_send('\r');
			break;

		// Read Lock Bits
		case 'r':				// done
			ser_send(mem_read(0x58, 0x0000));
			ser_send('\r');
			break;

		// Read program memory
		case 'R':				// done
			ser_send(mem_read(0x28, addr));
			ser_send(mem_read(0x20, addr));
			addr++;
			break;

		// Read data memory
		case 'd':				// done
			ser_send(mem_read(0xA0, addr));
			addr++;
			break;

		// Write data memory
		case 'D':				// done
			mem_write(0xC0, addr, ser_recv());
			poll();

			addr++;
			ser_send('\r');
			break;

		// Chip erase
		case 'e':				// done
			spi_rxtx(0xAC);
			spi_rxtx(0x80);
			spi_rxtx(0x00);
			spi_rxtx(0x00);

			_delay_ms(25);
			ser_send('\r');
			break;

		// Write lock bits
		case 'l':
			ser_recv();

			// TODO: implement?

			ser_send('\r');
			break;

		// Read fuse bits
		case 'F':				// done
			spi_rxtx(0x50);
			spi_rxtx(0x00);
			spi_rxtx(0x00);
			ser_send(spi_rxtx(0x00));
			break;

		// Return High Fusebits
		case 'N':				// done
			spi_rxtx(0x58);
			spi_rxtx(0x08);
			spi_rxtx(0x00);
			ser_send(spi_rxtx(0x00));
			break;

		// Return extendet Fusebits
		case 'Q':				// done
			spi_rxtx(0x50);
			spi_rxtx(0x08);
			spi_rxtx(0x00);
			ser_send(spi_rxtx(0x00));
			break;

		// Leave programming mode
		case 'L':				// done

		// Exit
		case 'E':				// done
			set_reset(1);
			led_mode(LED_OFF);
			ser_send('\r');
			break;

		// Select device type
		case 'T': {				// done
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
		case 's': {				// done
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
		case 't': {				// done
			uint8_t val, i = 0;
			do {
				val = devices[i++].id;
				ser_send(val);
			} while (val);
			break;
		}

		// Return software identifier
		case 'S':				// done
			ser_send('A');
			ser_send('V');
			ser_send('R');
			ser_send('-');
			ser_send('I');
			ser_send('S');
			ser_send('P');
			break;

		// Return software version
		case 'V':				// done
			ser_send('3');
			ser_send('8');
			break;

		// Return hardware version
		case 'v':				// done
			ser_send('1');
			ser_send('2');
			break;

		// Return programmer type
		case 'p':				// done
			ser_send('S');
			break;

		// Set LED
		case 'x':				// done
			ser_recv();
			led_mode(LED_ON);
			break;

		// Clear LED
		case 'y':				// done
			ser_recv();
			led_mode(LED_OFF);
			break;

		// Report Block write Mode
		case 'b':				// done
			ser_send('Y');
			ser_send(sizeof(page_buf) >> 8);
			ser_send(sizeof(page_buf) & 0xFF);
			break;

		// Block Write
		case 'B': {				// done
			uint16_t size, i;
			uint8_t type;

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
		case 'g': {				// done
			uint16_t size, i;
			uint8_t type;

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
		case 'f':
			ser_recv();
			// TODO: implement?
			ser_send('\r');
			break;

		// Universial command
		case ':': {				// done
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
		case '.': {				// done
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

		case 0x1B:
			break;

		default:
			ser_send('?');
			break;
		}
	}

	return 0;
}
