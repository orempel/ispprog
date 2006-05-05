#include <avr/io.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#define F_CPU		7372800
#define BAUDRATE	115200

#define UART_CALC_BAUDRATE(baudRate) (((uint32_t)F_CPU) / (((uint32_t)baudRate)*16) -1)

void sendchar(uint8_t data)
{
	loop_until_bit_is_set(UCSRA, UDRE);
	UDR = data;
}

uint8_t recvchar(void)
{
	loop_until_bit_is_set(UCSRA, RXC);
	return UDR;
}

int main(void)
{
	// reset & activity as outputs
	PORTB = (1<<PORTB1) | (1<<PORTB3);
	DDRB = (1<<PORTB1) | (1<<PORTB3);

	// Set baud rate
	UBRRH = (UART_CALC_BAUDRATE(BAUDRATE)>>8) & 0xFF;
	UBRRL = (UART_CALC_BAUDRATE(BAUDRATE) & 0xFF);

	// enable rx & tx
	UCSRB = (1<<TXEN) | (1<<RXEN);
	UCSRC = (1<<URSEL) | (1<<UCSZ1) | (1<<UCSZ0);

	while (1) {
		switch (recvchar()) {
		// Enter programming mode
		case 'P':
			sendchar('\r');
			break;

		// Report autoincrement address
		case 'a':
			sendchar('Y');
			break;

		// Set address
		case 'A':
			recvchar();
			recvchar();
			sendchar('\r');
			break;

		// Write program memory, low byte
		case 'c':
			recvchar();
			sendchar('\r');
			break;

		// Write program memory, high byte
		case 'C':
			recvchar();
			sendchar('\r');
			break;

		// Issue Page Write
		case 'm':
			sendchar('\r');
			break;

		// Read program memory
		case 'R':
			sendchar(0x00);
			sendchar(0x00);
			break;

		// Write data memory
		case 'D':
			recvchar();
			sendchar('\r');
			break;

		// Read data memory
		case 'd':
			sendchar(0x00);
			break;

		// Chip erase
		case 'e':
			sendchar('\r');
			break;

		// Write lock bits
		case 'l':
			recvchar();
			sendchar('\r');
			break;
		// Leave programming mode
		case 'L':
			sendchar('\r');
			break;

		// Select device type
		case 'T':
			recvchar();
			sendchar('\r');
			break;

		// Read signature bytes
		case 's':
			sendchar(0x00);
			sendchar(0x00);
			sendchar(0x00);
			break;

		// Return supported device codes
		case 't':
			sendchar(0x00);
			break;

		// Return software identifier
		case 'S':
			sendchar('I');
			sendchar('S');
			sendchar('P');
			sendchar('P');
			sendchar('R');
			sendchar('O');
			sendchar('G');
			break;

		// Return software version
		case 'V':
			sendchar('0');
			sendchar('0');
			break;

		// Return hardware version
		case 'v':
			sendchar('0');
			sendchar('0');
			break;

		// Return programmer type
		case 'p':
			sendchar('S');
			break;

		// Set/Clear LED
		case 'x':
		case 'y':
			recvchar();
			break;

		// Universial command
		case ':':
			recvchar();
			recvchar();
			recvchar();
			sendchar(0x00);
			break;

		// New universal command
		case '.':
			recvchar();
			recvchar();
			recvchar();
			recvchar();
			sendchar(0x00);
			break;

		// Exit (AVR109, Bootloader)
		case 'E':
			sendchar('\r');
			break;

		// Return Chip ID (Terminalmode only)
		case 'i':
			break;

		// Report Block write Mode (AVR109)
		case 'b':
			sendchar('Y');
			sendchar(0x00);
			sendchar(0x00);
			sendchar('\r');
			break;

		// Block Write (AVR109)
		case 'B':
			recvchar();
			recvchar();
			recvchar();
			sendchar('\n');
			break;

		// Block Read (AVR109)
		case 'g':
			recvchar();
			recvchar();
			recvchar();
			sendchar('\n');
			break;

		// Return Lockbits
		case 'r':
			sendchar(0x00);
			sendchar('\r');
			break;

		// Return High Fusebits
		case 'N':
			sendchar(0x00);
			sendchar('\r');
			break;

		// Return extendet Fusebits
		case 'Q':
			sendchar(0x00);
			sendchar('\r');
			break;

		// Write fuse bits
		case 'f':
			recvchar();
			sendchar('\r');
			break;

		// Read fuse and lock bits
		case 'F':
			sendchar(0x00);
			break;

		case 0x1B:
			break;

		default:
			sendchar('?');
			break;
		}
	}

	return 0;
}
