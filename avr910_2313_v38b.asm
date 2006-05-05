;***************************************************************************
;*
;* Title		: AVR ISP (Auto adr inc, 115200bps, supports AVR109 commands)
;* Version		: 3.8
;* Last updated	: Feb 26 2006 
;* Target		: AT90S2313
;* File			: avr910_2313_v38b.asm 
;* Author(s)	: klaus@mikrocontroller-projekte.de
;* last Filename: avr910_2313_V38a.asm
;* initial File	: avr910_31.asm, Author: wubblick@yahoo.com, source: www.mit8.ru/~mars/ )
;*
;* This Code is free for private, noncommercial use only
;*
;* DESCRIPTION
;*	The firmware on all programmers now support a unified protocol for 
;*	program and data memory programming. The host computer do not need
;*	to know if the programmer operates in serial or parallel mode.
;*
;*	The following commands are supported. All commands start with a
;*	single letter. The programmer returns 13d (carriage return) or the
;*	data read after the command is finished.
;*
;*                                     +-------------------+-------------+------+
;*  Commands                           |    Host  writes   | Host reads  |      |
;*  --------                           +-----------+-------+-------+-----+      |
;*                                     | ID (hex ) | data  | data  |     | Note |
;* +-----------------------------------+-----------+-------+-------+-----+------+
;* | Enter programming mode            | 'P'(0x50) |       |       | 13d |   1  |
;* | Report autoincrement address      | 'a'(0x61) |       |       | 'Y' |      |
;* | Set address                       | 'A'(0x41) | ah al |       | 13d |   2  |
;* | Write program memory, low byte    | 'c'(0x63) |    dd |       | 13d |   3  |
;* | Write program memory, high byte   | 'C'(0x43) |    dd |       | 13d |   3  |
;* | Issue Page Write                  | 'm'(0x6d) |       |       | 13d |  13  |
;* | Read program memory               | 'R'(0x52) |       | dd(dd)|     |   4  |
;* | Write data memory                 | 'D'(0x44) |    dd |       | 13d |      |
;* | Read data memory                  | 'd'(0x64) |       |   dd  |     |      |
;* | Chip erase                        | 'e'(0x65) |       |       | 13d |      |
;* | Write lock bits                   | 'l'(0x6c) |    dd |       | 13d |      |
;* | Leave programming mode            | 'L'(0x4c) |       |       | 13d |   5  |
;* | Select device type                | 'T'(0x54) |    dd |       | 13d |   6  |
;* | Read signature bytes              | 's'(0x73) |       |  3*dd |     |      |
;* | Return supported device codes     | 't'(0x74) |       |  n*dd | 00d |   7  |
;* | Return software identifier        | 'S'(0x53) |       |  s[7] |     |   8  |
;* | Return sofware version            | 'V'(0x56) |       | dd dd |     |   9  |
;* | Return hardware version           | 'v'(0x76) |       | dd dd |     |   9  |
;* | Return programmer type            | 'p'(0x70) |       |   dd  |     |  10  |
;* | Set LED                           | 'x'(0x78) |    dd |       | 13d |  12  |
;* | Clear LED                         | 'y'(0x79) |    dd |       | 13d |  12  |
;* | Universial command                | ':'(0x3a) |  3*dd |   dd  | 13d |  15  |
;* | New universal command             | '.'(0x2E) |  4*dd |   dd  | 13d |  15  |
;* |-----------------------------------+-----------+-------+-------+-----+------|
;* | New Commands since Version 3.3    |           |       |       |     |      |
;* | Exit (AVR109, Bootloader)         | 'E'(0x45) |       |       | 13d |  15  |
;* | Return Chip ID (Terminalmode only)| 'i'(0x69) |       |  s[n] |     |  14  |
;* +-----------------------------------+-----------+-------+-------+-----+------+
;* | New Commands since Version 3.5    |           |       |       |     |      |
;* | Implemented Atmel Bootloader commands (Atmel Appl. Note 109)  |     |      |
;* | Report Block write Mode           | 'b'(0x62) |       |'Y'2*nn| 13d |  16  |
;* | Block Write                       | 'B'(0x42) |2*nn'M'|  n*dd | 13d |  16  |
;* | Block Read                        | 'g'(0x67) |2*nn'M'|  n*dd | 13d |  16  |
;* +-----------------------------------+-----------+-------+-------+-----+------+
;* | Commands to test (fully implemented since V3.8, Unverified)   |     |      |      |
;* | Return Lockbits                   | 'r'(0x72) |       |   dd  | 13d |  17  |
;* | Return High Fusebits              | 'N'(0x4E) |       |   dd  | 13d |  17  |
;* | Return extendet Fusebits          | 'Q'(0x51) |       |   dd  | 13d |  17  |
;* | Write fuse bits (reserved)        | 'f'(0x66) |    dd |       | 13d |11,17 |
;* | Read fuse and lock bits (reserved)| 'F'(0x46) |       |   dd  |     |11,17 |
;* +-----------------------------------+-----------+-------+-------+-----+------+
;*'F
;* NOTE 1
;*	The Enter programming mode command MUST be sent one time prior to
;*	the other commands, with the exception of the 't', 'S', 'V', 'v'
;*	and 'T' commands. The 'T' command must be sent before this command
;*	(see note 6).
;*
;*	For programmers supporting both parallel and serial programming
;*	mode this command enters parallel programming mode. For programmers
;*	supporting only serial programming mode, this command enters serial
;*	programming mode.
;*
;* NOTE 2
;*	The ah and al are the high and low order bytes of the address. For
;*	parallel programmers this command issues the Load Address Low/High
;*	Byte command. For serial programmers the address byte is stored for
;*	use by the Read/Write commands.
;*
;* NOTE 3
;*	For parallel programmers this command issues the Program Flash
;*	command. For serial programmers this command iussues the Write
;*	Program Memory Command. For devices with byte-wide program memories
;*	only the low byte command should be used.
;*
;* NOTE 4
;*	The contents of the program memory at the address given by the 'A'
;*	command are written to the serial port in binary form. For byte
;*	wide memories one byte is written. For 16 bit memories two bytes
;*	are written,MSB first.
;*
;* NOTE 5
;*	This command must be executed after the programming is finished.
;*
;* NOTE 6
;*	The select device type command must be sent before the enter
;*	programming command
;*
;* NOTE 7
;*	The supported device codes are returned in binary form terminated
;*	by 0x00.
;*
;* NOTE 8
;*	This return a 7 character ASCII string identifying the programmer.
;*	For the development board it is "AVR DEV", for the parallel
;*	programmer it is "AVR PPR" and for the in-curcuit programmer it is
;*	"AVR ISP".
;*
;* NOTE 9
;*	The software/hardware version are returned as two ASCII numbers.
;*
;* NOTE 10
;*	This command should be used to identify the programmer type. The
;*	return value is 'S' for serial (or SPI) programmers or 'P' for
;*	parallel programmers.
;*
;* NOTE 11
;*	The write fuse bits command are available only on parallel
;*	programmers and only for AVR devices (device code < 0x80). The host
;*	should use the return programmer type command to determine the
;*	programmer type, do not use the  "AVR PPR" idenifier because other
;*	programmers may be available in the future.
;*
;* NOTE 12
;*	Currently only the AVR development board has LEDs. The other boards
;*	must implement this commands as NOPs.
;*
;* NOTE 13
;*  	Devices using Page Mode Programming write one page of flash memory
;*  	before issuing a Page Mode Write Pulse.
;*
;* NOTE 14
;*  	The 'i' command is only used with a Terminal, for debug reasons. 
;*  	Not used in any Programmer so far
;*
;* NOTE 15
;*  	Lock and Fuse Bits are written using the "universal command" (:) and
;*  	"new universal command" (.) in AVRProg and AVRDUDE.
;*  	AVRProg has an Exit (E) command implemented, which switches AVRProg offline
;*
;* NOTE 16
;*  	Atmel Bootloader commands described in Atmel Application Note 109 implemented. 
;*  	Very useful if you use USB to serial converter.
;*  	AVRProg since Version 1.33 checks if Programmer supports Block Mode,
;*  	IF  Programmer responds 'Y' and the size of the internal Buffer (nn nn), Bootloader commands
;*  	are used automatically from AVRProg. (faster Protocoll)
;*  	Block commands send size Buffersize (nn nn) and Type of Memory ('F'=Flash, 'E'=Data)
;*  	Compatibility to AVR Butterfly command set and Bootloader. Works with AVR911 Open Source
;*  	Programmer.
;*  	For avrdude use -c butterfly instead of -c avr910 and you get same behaviour.
;*		Page Write for Data Memory (EEPROM) not implemented
;*
;* NOTE 17
;*  	Commands implemented in V3.8 but not completely tested yet.
;*
;* NOTE 18
;*  	Device 89S8252 was tested to programm with avrprog. Shows a avrprog bug in "block Mode". Works in "standart"
;*  	avr910 Programming Mode. To program this Device, disable the "Block Mode" by changing "w53:	rjmp w6"	
;*
;* HISTORY
;* V3.8b 26.02.06 (Klaus)       Matched new Devices to avrdude.conf v5.1
;* V3.8a 30.12.05 (Klaus)       Corrected Devic entry for ATTiny13
;* V3.8  06.11.05 (Klaus)       New Devicecodes (inoffiacal) for Mega48/88/168 and others,
;*				Lockbyte write enhanced for newer Devices. Commands in 
;*				Note 17 implemented. Not comletely tested.
;*				Code cleanup to save some Bytes.
;* V3.7e 11.01.05 (Klaus)	Matched Devicecodes to the new Atmel AVRProgV1.40
;* V3.7d 05.01.05 (Klaus)	Removed 1200A/B/C devicecodes, Polling 0x7F for AT90S4414, default PollcodeF = 0xFF
;* V3.7c 04.01.05 (Klaus/Micha)	Some devicecodes changed (Bootloader and "default" mode).
;* V3.7b 03.01.05 (Klaus/Leo)	Bugfix for Block read of Flash Mamory for AT89Snnnn Controllers.
;* V3.7a 29.12.04 (Klaus/Micha)	addet devicecodes for m64, m162 and m169.
;* V3.7	 15.12.04 (Klaus)	Rewriting polled timing for "non Page Mode" Chips, reorganised Table for 
;*				different polling Codes. Skip 0xFF Byte polling. (was buggy for some Controllers)
;*				Added ATTiny2313 (device Code 0x23), and ATMega8535 (0x6A)Support.
;*				Not Supported from avrprog, avrdude Support possible.
;*				Reorganised Table for Page Mode Chips with Pagesize. Adding different Pagesizes
;* 				Try to fix S89 write, BUG in avrprog detected. Firmware should be OK now. (NOTE 18)
;* 				After 256 unsuccesful polls for Flash wait additional standart time.
;* 				Modified "release_ports" so that RESET is released also. 
;* V3.6 10.07.04  (Klaus)	Changed Block Mode Commands to AVR109 compatibility.
;*				Additional AVR109 Commands, not testet yet.
;* V3.4-V3.5 unreleased (Klaus) cleaning up, tests for Block Modes
;* V3.3 25.06.04 (Klaus, Thomas) added enhanced Block write commands
;                           	with own Protocol.
;                           	Add 0xFF, 0x7F (2313) check for polling Mode
;* V3.2.1 18.11.03  (Klaus) 	Commented out some Controller Types, which are
;				not really Supported by the hardware, but from
;				AVRProg Software. See Comments at the end.
;* V3.2 16.11.03    (Klaus) 	Correctet typo in Chrystal frequency. 
;* V3.2 13.10.03    (Klaus)	7,3728 Mhz chrystal for 115.200 Baud.
;*				            Swap MOSI <-> MISO Pinnumbers.
;*				            Add dual color LED Support.
;*				            Add some new Dewices.
;* V3.0	 02.01.23 (wubble)	Ported from 1200 to 2313.
;*				            Serial IO modified to use hardware UART.
;*				            Added support for multiple new devices.
;*				            Used data polling to short write process.
;*				            Added LED support to indicate R/W process.
;* V2.2	 00.03.10 (pkastnes) Added support for multiple new devices.
;* V2.1	 98.10.26 (mlund)	New date marking.
;*				            Removed support for AT90S1200C.
;*				            Added support for AT90S4433A.
;* V2.0	 98.01.06 (mlund)	ATmega103 support.
;* V1.7	 97.11.06 (mlund)	Universial command (':') implemented.
;*				            Releases all pins when not in 
;*				            programming mode.
;* V1.6e 97.11.04 (mlund)	mega103 rev D support
;* V1.6c 97.10.30 (mlund)	Auto incrementing / SPI sync
;*				            also works for mega103.
;* V1.6	 97.09.09 (hskar)	Created Page Mode Version (mega103)
;* V1.5	 97.08.21 (mlund)	Modified / Bugfix / Major cleanup
;* ...	 ...			    (no records)
;* V?.?	 97.03.15 (OS)		Created
;* 
;*		
;* Device Support List: see table "Devices" at the end of this file.
 
;***************************************************************************

;**** includes ****

.include "2313def.inc"

;***************************************************************************
;*
;* CONSTANTES
;*
;***************************************************************************

;**** Constant declarations Data Rate ****

;******** Chrystals for maximum Baudrates
.equ	XTAL = 7373			; XTAL frequency, Khz (7.3728 Mhz)
;********
.equ	BAUD = 115200		; Data rate, bauds
.equ	N = 3				; for 7.3728 Mhz/115.200Baud
;********
;.equ	BAUD = 38400		; Data rate, bauds
;.equ	N = 11				; for 7.3728 Mhz/38.400Baud
;********
;.equ	BAUD = 19200		; Data rate, bauds
;.equ	N = 23				; for 7.3728 Mhz/19.200 Baud
;*********

;**** definitions for different chrystals and 115.200 baud ****
;.equ	XTAL = 3686			; XTAL frequency, Khz (3.6864 Mhz)
;*********
;.equ	BAUD = 115200		; Data rate, bauds
;.equ	N = 1				; for 3.6864 Mhz/115.200Baud
;*********
;
;********* Baudrates for 8 Mhz Chrystal
;.equ	XTAL = 8000			; XTAL frequency, Khz (8.000 Mhz)
;*********
;.equ	BAUD = 38400		; Data rate, bauds
;.equ	N = 12				; for 8.00 Mhz/38.400 Baud
;*********
;.equ	BAUD = 19200		;Data rate, bauds
;.equ	N = 25				; for 8.00 Mhz/19.200 Baud
;*********
;.equ	BAUD = 9600			; Data rate, bauds ! Not valid for AVRProg, use for avrdude only !
;.equ	N = 51				; for 8.00 Mhz/9.600 Baud
;*********

;********* Baudrates for 4 Mhz Chrystal
;.equ	XTAL = 4000			; XTAL frequency, Khz (4.000 Mhz)
;*********
;.equ	BAUD = 19200		; Data rate, bauds
;.equ	N = 12				; for 4.00 Mhz/19.200 Baud
;*********
;.equ	BAUD = 9600			; Data rate, bauds ! Not valid for AVRProg, use for avrdude only !
;.equ	N = 25				; for 4.00 Mhz/9.600 Baud
;*********

.equ RAMSTART = 0x60		; first SRAM Adress of 2313
.equ PAGESIZE = 0x10		; default Page size for Programming
.equ BUFSIZE  = 0x40		; 64 bytes internal RAM for Buffer

;***************************************************************************
;*
;* PORTS
;*	Ports Definitions
;*
;* DESCRIPTION
;*	Change the following definitions if the RESET pin to the
;*	target moves and/or if the SCK/MISO/MISO/LED moves.
;*
;***************************************************************************

.equ	LEDH	= PB3		; dual color LED output, anode green  (output)
.equ	LED		= PB0		; LED output, active low, dual color LED Kathode green  (output)
.equ	MISO	= PB6		; MISO  pin of the target (input)
.equ	MOSI	= PB5		; MOSI  pin of the target (output)

.equ	RESET	= PB4		; RESET pin of the target (output)
.equ	SCK		= PB7		; SCK   pin of the target (output)
.equ	RXD		= PD0		; UART RXD line
.equ	TXD		= PD1		; UART TXD line

;***************************************************************************
;*
;* MACROS
;*	Program Macros
;*
;* DESCRIPTION
;*	Change the following macros if the RESET pin to the
;*	target moves and/or if the SCK/MISO/MISO/LED moves.
;*
;***************************************************************************

.macro	init_ports			; init ports
	ldi	temp2,0xff	
	out	PORTD,temp2
	ldi	temp2,0xff
	out	PORTB,temp2
.endm

.macro	catch_ports			; catch ports
	ldi	temp2,(1<<TXD)
	out	DDRD,temp2
	ldi	temp2,(1<<RESET) | (1<<MOSI) | (1<<SCK) | (1<<LED) | (1<<LEDH)
	out	DDRB,temp2
.endm

.macro	release_ports		; release ports
	ldi	temp2,(1<<TXD)
	out	DDRD,temp2
	ldi	temp2,(1<<LED) | (1<<LEDH)
	out	DDRB,temp2
.endm

.macro	pas_RESET			; set RESET passive
	tst	device
	brmi	m2				; S89 device
	sbi	PORTB,RESET
	rjmp	m3
m2:	cbi	PORTB,RESET
m3:		
.endm

.macro	act_RESET			; set RESET active
	tst	device
	brmi	m4				; S89 device
	cbi	PORTB,RESET
	rjmp	m5
m4:	sbi	PORTB,RESET
m5:		
.endm

.macro	set_LED				; set LED Port -> Standart LED off, dual LED green
	sbi PORTB,LED
	cbi PORTB,LEDH			; added for dual color support
.endm

.macro	clr_LED				; clear LED Port --> Standart LED on, dual LED red
	cbi PORTB,LED
	sbi PORTB,LEDH			; added for dual color support
.endm

.macro LED_off				; for dual color LED, not in use
	cbi PORTB,LED
	cbi PORTB,LEDH
.endm

.macro	sbic_MISO			; skip if MISO cleared
	sbic	PINB,MISO
.endm

.macro	set_MOSI			; set MOSI
	sbi	PORTB,MOSI
.endm

.macro	clr_MOSI			; clear MOSI
	cbi	PORTB,MOSI
.endm

.macro	clr_SCK				; clear SCK
	cbi	PORTB,SCK	
.endm

.macro	pulse_SCK			; pulse SCK
	sbi	PORTB,SCK
	ldi	temp2,(XTAL/750+1)	; had to slow down for ATTiny2313 (internal clock) default was (XTAL/1500+1)	
m0:	dec	temp2
	brne	m0
	cbi	PORTB,SCK
	ldi	temp2,(XTAL/1500+1)	; had to slow down for ATTiny2313 (internal clock) default was (XTAL/3000+1)	
m1:	dec	temp2
	brne	m1
.endm

.macro	table				; load Z pointer
	ldi	ZL,low(@0*2)		; low
	ldi	ZH,high(@0*2)		; high
.endm

;***************************************************************************
;*
;* Global Register Variables
;*
;***************************************************************************

;r0 used with lpm instruction
.def	cmd1		= r1	; Universal commands params
.def	cmd2		= r2	; ..
.def	cmd3		= r3	; ..
.def	pol_al		= r4	; Polling address low
.def	pol_ah		= r5	; Polling address high
.def	Bcnt1		= r6	; Block Mode Counter1
.def	Bcnt2		= r7	; Block Mode Counter2
.def	Bcnt3		= r8	; Block Mode Counter2
.def    B_Flag		= r9	; Flag for ws_del Routine
.def    B_Mode		= r10	; Marks Block Mode commands (See Note 16)
.def 	Memtype		= r11	; Flag for Memorytype for Block Commands (See Note 16)
.def	Pagewords	= r12	; Size of Controller page to program in Page Mode (Words!) 
.def 	PollcodeF	= r13	; Code for Polling Flash


.def	temp1		= r16	; Temporary register 1
.def	temp2		= r17	; Temporary register 2
.def	temp3		= r18	; Temporary register 3
.def	s_data		= r19	; SPI data
.def	u_data		= r20	; UART data
.def	device		= r21	; Device code
;.def	lockmode	= r22	; Param to set correct lockbit read/write mode 
.def	rd_s_data	= r23	; Read data on SPI
.def	pol_cmd		= r24	; Polling command
.def	p_data		= r25	; Polling data
;r26,r27 used as X register
.def	addrl		= r28	; (YL) Low order byte of address
.def	addrh		= r29	; (YH) High order byte of address
;r30,r31 used as Z register

;***************************************************************************
;*
;* Interrupt Vectors
;*
;***************************************************************************

.CSEG
	rjmp	INIT			; Reset Handle

;***************************************************************************
;*
;* FUNCTION
;*	u_init
;*
;* DESCRIPTION
;*	Initialize UART.
;*
;***************************************************************************

u_init:	
	ldi	temp1,N				; set baud rate
	out	UBRR,temp1
	ldi	temp1,(1<<TXEN)|(1<<RXEN) ; initialize UART for TX and RX
	out	UCR,temp1
	ret

;***************************************************************************
;*
;* FUNCTION
;*	getc
;*
;* DESCRIPTION
;*	Wait for start bit and receive a character on the UART Rx line.
;*
;***************************************************************************

getc:
	sbis	USR,RXC			; wait until a character has been received
	rjmp	getc
	in	u_data,UDR			; Read byte from the UART
	ret

;***************************************************************************
;*
;* FUNCTION
;*	putc
;*
;* DESCRIPTION
;*	Send a character on the UART Tx line.
;*
;***************************************************************************

putc:
	sbis	USR,UDRE		; test for TX register empty
	rjmp	putc			; loop until TX empty
	out	UDR,u_data			; send the byte
	ret

;***************************************************************************
;*
;* FUNCTION
;*	put_string
;*
;* DESCRIPTION
;*	Send Z - pointed null-terminated string on the UART Tx line.
;*
;***************************************************************************
	
put_string:
	lpm
	tst	r0
	breq	ps_ret			; check for end of string (0x00)
	mov	u_data,r0
	rcall	putc			; putc(char)
	adiw	ZL,1			; next char
	rjmp	put_string
ps_ret:	ret	

;***************************************************************************
;*
;* FUNCTION
;*	put_table
;*
;* DESCRIPTION
;*	Send Z - pointed table on the UART Tx line.
;*
;***************************************************************************
	
put_table:
	lpm
	tst	r0
	breq	pt_ret			; check for end of table (0x00)
	mov	u_data,r0	
	rcall	putc			; putc(Byte)
	adiw	ZL,2			; skip MSB
	rjmp	put_table
pt_ret:	ret	

;***************************************************************************
;*
;* FUNCTION
;*	bel_table
;*
;* DESCRIPTION
;*	C=0 if device belongs to table.
;*
;***************************************************************************

bel_table:
	lpm						; read table
	tst	r0					; check for end of table
	breq	c1_ret
	cp	device,r0
	breq	c0_ret			; C=0
	adiw	ZL,2			; skip MSB
	rjmp	bel_table	
c1_ret:	sec					; C=1
c0_ret:	ret		

;***************************************************************************
;*
;* FUNCTION
;*	set_pagesize
;*
;* DESCRIPTION
;*	sets programming Page size for selected Device.
;*
;***************************************************************************

set_pagesize:
	lpm						; read table
	tst	r0					; check for end of table
	breq	spa_end			; no Pagesize to set
	cp	device,r0
	breq	spa_set			; C=0
	adiw	ZL,2			; skip MSB
	rjmp	set_pagesize	
spa_set:
	adiw	ZL,1			; Point to high Byte of Word
	lpm						; get Pagesize to R0
	mov Pagewords,r0
	mov Bcnt3,Pagewords		; initiate Counter
spa_end:
	ret		

;***************************************************************************
;*
;* FUNCTION
;*	set_pollcode
;*
;* DESCRIPTION
;*	sets Code for Polling Flash for selected Device.
;*
;***************************************************************************

set_pollcode:
	lpm						; read table
	tst	r0					; check for end of table
	breq	spo_end			; no Pollcode to set
	cp	device,r0
	breq	spo_set			; C=0
	adiw	ZL,2			; skip MSB
	rjmp	set_pollcode	
spo_set:
	adiw	ZL,1			; Point to high Byte of Word
	lpm						; get Pagesize to R0
	mov PollcodeF,r0		; Set Pollcode for Flash Rom
spo_end:
	ret		

;***************************************************************************
;*
;* FUNCTION
;*	delay
;*
;* DESCRIPTION
;*	 Make delay 1mS (x temp1).
;*
;***************************************************************************

delay:	
	ldi	temp2,40
dl2:	ldi	temp3,(XTAL/120)
dl1:	dec	temp3
	brne	dl1
	dec	temp2
	brne	dl2
	dec	temp1
	brne	delay
	ret

;***************************************************************************
;*
;* FUNCTION
;*	 spi123
;*
;* DESCRIPTION
;*	 Write bytes 1 to 3 on the SPI. Byte 1 must be loadet into s_data
;*       Byte 2 ist addrh, Byte 3 ist addrl
;*
;***************************************************************************

spi123:
	rcall	wrser			; wrser(s_data) SPI write (byte 1)
	mov	s_data,addrh
	rcall	wrser			; wrser(addrh)  SPI write (byte 2)
	mov	s_data,addrl	
	rcall	wrser			; wrser(addrl)  SPI write (byte 3)
	ret

;***************************************************************************
;*
;* FUNCTION
;*	 w1234 (for Code simplification,)
;*
;* DESCRIPTION
;*	 Write SPI bytes 1 to 4.
;            Byte 1 must be loadet into cmd1
;            Byte 2 must be loadet into cmd2
;            Byte 3 must be loadet into cmd3
;            Byte 4 must be loadet into u_data
;*
;***************************************************************************
w1234:
	mov s_data,cmd1
	rcall wrser
	mov s_data,cmd2
	rcall wrser
	mov s_data,cmd3
	rcall wrser
	mov s_data,u_data		; get Byte 4 from serial
	rcall wrser
	ret

;***************************************************************************
;*
;* FUNCTION
;*	 w123r4 (for Code simplification)
;*
;* DESCRIPTION
;*	 Write SPI bytes 1 to 3, read Byte 4 to serial.
;            Byte 1 must be loadet into cmd1
;            Byte 2 must be loadet into cmd2
;            Byte 3 must be loadet into cmd3
;*
;***************************************************************************
w123r4:
	mov s_data,cmd1
	rcall wrser
	mov s_data,cmd2
	rcall wrser
	mov s_data,cmd3
	rcall wrser
	rcall rdser
	mov u_data,s_data		; put Byte 4 to serial
	rcall putc
	ret


;***************************************************************************
;*
;* FUNCTION
;*	rdser, wrser
;*
;* DESCRIPTION
;*	 Write and read bytes on the SPI.
;*
;***************************************************************************

rdser:	
	clr	s_data
wrser:	
	ldi	temp1,8			; load bit counter
	ldi	rd_s_data,0
wrs0:
	rol	s_data
	brcc	wrs1
	set_MOSI			; MOSI = 1
	rjmp	wrs2
wrs1:
	clr_MOSI			; MOSI = 0
wrs2:
	lsl	rd_s_data
	sbic_MISO			; read MISO
	ori	rd_s_data,1
	pulse_SCK			; pulse SCK
	dec	temp1			; advance bit counter
	brne	wrs0			; loop
	mov	s_data,rd_s_data
	ret



;***************************************************************************
;*
;* FUNCTION
;*	read_send_progmem
;*
;* DESCRIPTION
;*	 Read one adress (2 Byte) from Program Memory and send it through UART
;*
;***************************************************************************
read_send_progmem:
	tst	device
	brmi	rsp1			; S89 device
	ldi	s_data,0x20		; read low Byte
	rcall	wrser			; wrser(0x28)  SPI write (byte 1)
	mov	s_data,addrh
	rjmp	rsp2
rsp1:
	mov	s_data,addrh		; s_data = (addrh << 3) | 0x01;
	rcall	shift_s_data3
	ori	s_data,0x01
rsp2:
	rcall	wrser			; wrser(addrh) SPI write (byte 2) (S89=byte1)
	mov	s_data,addrl	
	rcall	wrser			; wrser(addrl) SPI write (byte 3)
	rcall	rdser			;              SPI read  (byte 4)
	mov	u_data,s_data
	rcall	putc			; send data
	tst	device
	brmi	rsp3			; S89 device
	ldi	s_data,0x28			; read High Byte
	rcall	wrser			; wrser(0x20)  SPI write (byte 1)
	mov	s_data,addrh	
	rcall	wrser			; wrser(addrh) SPI write (byte 2)
	mov	s_data,addrl	
	rcall	wrser			; wrser(addrl) SPI write (byte 3)
	rcall	rdser			;              SPI read  (byte 4)
	mov	u_data,s_data
	rcall	putc			; send data
rsp3:
	adiw	addrl,1			; Auto increment address
	ret

;***************************************************************************
;*
;* FUNCTION
;*	read_send_datamem
;*
;* DESCRIPTION
;*	 Read one Byte from Data Memory (eeprom) and send it through UART
;*
;***************************************************************************
read_send_datamem:			; Subroutine to read one eeprom Address
	tst	device
	brmi	rsd1			; S89 device
	ldi	s_data,0xa0
	rcall	wrser			; wrser(0xa0)   SPI write (byte 1)
	mov	s_data,addrh
	rjmp	rsd2
rsd1:
	cpi	device,0x87		; if (device == S53)
	breq	rsd3			; no Support for 89S53 device due to Bug in AVRProg V1.37
	mov	s_data,addrh
	rcall	shift_s_data3
	ori	s_data,0x05
rsd2:
	rcall	wrser			; wrser(addrh)  SPI write (byte 2)
	mov	s_data,addrl	
	rcall	wrser			; wrser(addrl)  SPI write (byte 3)
	rcall	rdser			;               SPI read  (byte 4)
	mov	u_data,s_data
	rcall	putc			; send data
	adiw	addrl,1			; Auto increment address
	ret
rsd3:
	pop	temp1			; remove return Adress from Stack in case of Error
	pop	temp1
	rjmp	put_err



;***************************************************************************
;*
;* FUNCTION
;*	eeprom_write
;*
;* DESCRIPTION
;*	 Write u_data to Data Memory (eeprom)
;*
;***************************************************************************
eeprom_write:
	tst	device
	brmi	eew1			; S89 device
	ldi	s_data,0xc0
	rcall	wrser			; wrser(0xc0)   SPI write (byte 1)
	mov	s_data,addrh
	rjmp	eew2	
eew1:
	cpi	device,0x87		; if (device == S53)
	breq	eew3
	mov	s_data,addrh
	rcall	shift_s_data3
	ori	s_data,0x06
eew2:
	rcall	wrser			; wrser(addrh)  SPI write (byte 2)
	mov	s_data,addrl
	rcall	wrser			; wrser(addrl)  SPI write (byte 3)
	mov	s_data,u_data
	rcall	wrser			; wrser(u_data) SPI write (byte 4)
	ldi	temp1,10		; delay 10mS
	rcall	delay

	adiw	addrl,1			; Auto increment address
	ret
eew3:
	pop temp1			; remove return Adress from Stack in case of failure
	pop temp1
	rjmp	put_err

;***************************************************************************
;*
;* FUNCTION
;*	shift_s_data3
;*
;* DESCRIPTION
;*	 Shift s_data 3 times left for S89 device.
;*
;***************************************************************************

shift_s_data3:
	lsl	s_data
	lsl	s_data
	lsl	s_data
	brcc	s3_ret
	cpi	device,0x87		; if (device != S53)
	brne	s3_ret	
	sbr	s_data, 4		; a13 +
s3_ret:	ret


;***************************************************************************
;*
;* FUNCTION
;*	healthcheck
;*
;* DESCRIPTION
;*	 changes color of dual color led.
;*
;***************************************************************************

healthcheck:				; for dual color LED
	clr_LED					; LED red
	ldi temp1,200			; 200 ms
	rcall delay		
	ldi temp1,200			; again 200 ms
	rcall delay
	ldi s_data,200			; counter Register

LEDloop:					; Pulse for yellow LED
	set_LED					; LED green
	ldi temp1,2				; 2 ms
	rcall delay
	clr_LED					; LED red
	ldi temp1,1				; 1 ms
	rcall delay
	dec s_data				; dec. counter
	brne LEDloop
	set_LED					; LED green
	ret

;***************************************************************************
;***************************************************************************
;***************************************************************************
;*
;* INIT
;*
;* DESCRIPTION
;*	Initialization
;*
;***************************************************************************
;***************************************************************************
;***************************************************************************

INIT:
	ldi	temp1,RAMEND
	out SPL,temp1			; Locate stack
	ldi	temp1,PAGESIZE		; default Pagesize
	mov Pagewords,temp1
	mov Bcnt3,Pagewords		; set counter for Pagesize
	ldi	device,0x20			; S2313 as default
	clr B_Mode				; Flag for Block Modes (see Note 16)
	clr B_Flag				; Flag for eNhanced Block write
	init_ports				; Initialize ports
	release_ports			; Release ports
	rcall	u_init			; Initialize UART
	rcall healthcheck		; show that Prog is working after Powerup (LED test)

	 	
;***************************************************************************
;*
;* PROGRAM
;*	waitcmd -> main
;*
;* DESCRIPTION
;*	Wait for and execute commands.
;*
;***************************************************************************

waitcmd:
	rcall	getc			; while (getc() == ESC) {};
	cpi	u_data,0x1b
	breq	waitcmd

;====== 'T' - Device Type ==================================================

	cpi	u_data,'T'		; 'T' Device type
	brne	w0
	rcall	getc
	mov	device,u_data		; get device type
	table Dev_M				; prepare to set Pagesize
	rcall set_pagesize		; If device has Page Mode support, set Pagesize
	ldi	temp1,0xFF
	mov	PollcodeF,temp1		; preset PollcodeF with 0xFF, will be overwritten if Pollcode known
	table Dev_S				; prepare to set Polling Code
	rcall set_pollcode		; If device has no Page Size support, set code for Polling Flash
	rjmp	put_ret

;====== 'S' - Return Software Identifier ===================================

w0:	
	cpi	u_data,'S'		; 'S' Return software identifier
	brne	w1
	table	ID_Str
	rcall	put_string		; put string "AVR ISP"
	rjmp	waitcmd

;====== 'V' - Return Software Version ======================================

w1:	
	cpi	u_data,'V'		; 'V' Return software version
	brne	w2
	table	SW_Ver
	rcall	put_string		; put software version
	rjmp	waitcmd


;====== 'v' - Return Hardware Version ======================================

w2:	
	cpi	u_data,'v'		; 'v' Return hardware version
	brne	w3
	table	HW_Ver
	rcall	put_string		; put hardware version
	rjmp	waitcmd

;====== 't' - Show Supported Devices =======================================

w3:	
	cpi	u_data,'t'		; 't' Show supported devices
	brne	w4
	table	Dev_S
	rcall	put_table		; put supported devices codes
	table	Dev_M
	rcall	put_table		; put supported devices codes
	ldi	u_data,0x00		; putc(0x00) - end of device list
	rcall	putc
	rjmp	waitcmd

;====== 'p' - Return Programmer Type =======================================

w4:	
	cpi	u_data,'p'		; 'p' Return programmer type
	brne	w5
	ldi	u_data,'S'		; putc('S') - serial programmer
	rcall	putc
	rjmp	waitcmd

;====== 'a' - Return autoincrement address support =========================

w5:	
	cpi	u_data,'a'		; 'a' Return address auto increment
	brne	w51
	ldi	u_data,'Y'		; putc('Y') - supports autoinc
	rcall	putc
	rjmp	waitcmd

;====== 'M' - Return enhanced Mode support (Note 14) ========================

w51:	
;	cpi	u_data,'M'		; 'M' Return enhanced mode Support
;	brne	w52
;	ldi u_data,'Y'			; putc('Y') - supports enhanced Mode
;	rcall putc
;	rjmp	waitcmd

;====== 'i' - Return Chip ID (Note 14) ======================================

w52:
	cpi	u_data,'i'		; 'i' Return Chip ID
	brne	w53
	table	ChipID
	rcall	put_string		; put Chip ID string 
	ldi u_data,0x0a			; putc(LF)
	rcall 	putc
	rjmp	put_ret

;====== 'b' - Return Block write Mode support (Note 17) ========================

w53:	
	cpi	u_data,'b'		; 'b' Return enhanced mode Support
	brne	w6
	ldi u_data,'Y'			; putc('Y') - supports enhanced Mode
	rcall putc
	ldi	u_data,high(BUFSIZE); putc((BUFSIZE>>8) & 0xff);
	rcall putc
	ldi	u_data,low(BUFSIZE)	; putc(BUFSIZE&0xff);
	rcall putc
	rjmp	waitcmd

;====== 'x' - Set LED ======================================================

w6:	
	cpi	u_data,'x'		; 'x' Set LED (LED off or green)
	brne	w61
	rcall	getc			; get parameter
	set_LED
	rjmp	put_ret

;====== 'y' - Clear LED ====================================================

w61:	
	cpi	u_data,'y'		; 'y' Clear LED (LED on or red)
	brne	w7
	rcall	getc			; get parameter
	clr_LED
	rjmp	put_ret


;===========================================================================

; We require that the device code be selected before any of the other commands

w7:
	table	Dev_S			; load pointer
	rcall	bel_table
	brcc	w71				; device belongs to table
	table	Dev_M	
	rcall	bel_table
	brcc	w71				; device belongs to table
	rjmp	put_err			; not match, goto put_err();

;====== 'P' - Enter Programming Mode =======================================

w71:	
	cpi	u_data,'P'		; 'P' Enter programming mode
	breq	w70
	rjmp	w8
w70:	
	clr_LED					; LED on
	catch_ports				; catch ports
	clr_SCK					; clear SCK
	pas_RESET				; set RESET passive
	ldi	temp1,50			; delay 50mS;
	rcall	delay
	act_RESET				; set RESET active
	ldi	temp1,50			; delay 50mS;
	rcall	delay	
	ldi	s_data,0xac	
	rcall	wrser			; wrser(0xac) SPI write (byte 1)
	ldi	s_data,0x53	
	rcall	wrser			; wrser(0x53) SPI write (byte 2)
							; SPI Synchronization (fix!)
	cpi	device,0x20			; if ( (device >= 0x20) && (device <= 0x7F) )
	brlo	s2
	tst	device
	brmi	s2	
	ldi	temp3,32			; count = 32;
s1:	rcall	rdser			; SPI read  (byte 3)
	cpi	s_data,0x53			; if (rdser == 0x53)
	breq	s3				; break
	ldi	s_data,0x00					
	rcall	wrser			; wrser(0x00) SPI write (byte 4)
	pulse_SCK				; pulse SCK
	ldi	s_data,0xac		
	rcall	wrser			; wrser(0xac) SPI write (byte 1)
	ldi	s_data,0x53		
	rcall	wrser			; wrser(0x53) SPI write (byte 2)
	dec	temp3				; count-1
	brne	s1				; loop
	rjmp	s3				; else
s2:	ldi	s_data,0x00	
	rcall	wrser			; wrser(0x00) SPI write (byte 3)
s3:	tst	device
	brmi	s4				; S89 device
	ldi	s_data,0x00	
	rcall	wrser			; wrser(0x00) SPI write (byte 4)
s4:	ldi	temp1,4				; delay 4mS;
	rcall	delay
	rjmp	put_ret

;====== 'c' - Write Program Memory, Low Byte ===============================

w8:	
	cpi	u_data,'c'		; 'c' Write program memory, low byte
	brne	w9
	rcall	getc			; get data byte
w8b: 	ldi	s_data,0x40
	mov	pol_cmd,s_data		; save command for polling	
	tst	device
	brmi	w81			; S89 device
	rcall	wrser			; wrser(0x40)   SPI write (byte 1)
	mov	s_data,addrh
	rjmp	w82
w81:
	mov	s_data,addrh		; s_data = (addrh << 3) | 0x02;
	rcall	shift_s_data3
	ori	s_data,0x02
w82:
	rcall	wrser			; wrser(addrh)  SPI write (byte 2)
	mov	s_data,addrl	
	rcall	wrser			; wrser(addrl)  SPI write (byte 3)
	mov	s_data,u_data	
	rcall	wrser			; wrser(u_data) SPI write (byte 4)
	mov	p_data,u_data		; save data for polling
	mov	pol_al,addrl		; save address for polling
	mov	pol_ah,addrh
	tst	device
	brpl	w83 
	adiw	addrl,1			; Auto increment address for S89 device
w83:
	rjmp	wait_S			; write FLASH delay

;====== 'C' - Write Program Memory, High Byte ==============================

w9:	
	cpi	u_data,'C'		; 'C' Write program memory, high byte
	brne	w92
	rcall	getc			; get data byte
w9a:	tst	device
	brmi	w91			; S89 device
	ldi	s_data,0x48	
	mov	pol_cmd,s_data		; save command for polling	
	rcall	wrser			; wrser(0x48)   SPI write (byte 1)
	mov	s_data,addrh	
	rcall	wrser			; wrser(addrh)  SPI write (byte 2)
	mov	s_data,addrl	
	rcall	wrser			; wrser(addrl)  SPI write (byte 3)
	mov	s_data,u_data		; wrser(u_data) SPI write (byte 4)
	rcall	wrser
	mov	p_data,u_data		; save data for polling
	mov	pol_al,addrl		; save address for polling
	mov	pol_ah,addrh
	adiw	addrl,1			; Auto increment address
	rjmp	wait_S			; write FLASH delay
w91:
	rjmp	put_err			; S89 device have byte wide program memory!	

;====== 'B' Block Write Memory (see Note 17) ======================

w92:	
	cpi	u_data,'B'		; 'B' Block Write Program Memory
	breq w92a
	rjmp w10
w92a:
	rcall	getc			; get count High Byte
	tst u_data
	breq w92b
	ldi u_data,1
	rjmp put_err	
w92b:
	rcall	getc			; get count Low Byte
	cpi u_data,BUFSIZE+1		; check maximum count 
	brlo w92c
	ldi u_data,2
	rjmp put_err
w92c:
	mov Bcnt1, u_data		; ignore BUFSIZE high Byte (must be 0 here)
	mov Bcnt2, u_data
	rcall	getc			; get Memory type to write
	mov Memtype,u_data		; Flag for Memtype
	clr XH
	ldi XL, RAMSTART		; set X pointer to SRAM begin
w93:
	rcall	getc			; get data until Bcnt1 is reached	
	st X+, u_data			; store data to SRAM
	dec Bcnt1
	brne w93
	clr XH
	ldi XL, RAMSTART		; set X pointer to SRAM begin
	ldi temp3,1
	mov B_Mode,temp3		; B_Mode != 0
	mov u_data,Memtype		; restore Memtype 
	cpi	u_data,'F'		; 'F' Flash Memory ist to write
	breq	w94a
	cpi	u_data,'E'		; 'E' eeprom Memory ist to write
	breq	w94				; Entry Point for Data Memory Block write
	ldi u_data,3
	rjmp put_err

w94:						; write Data (EEPROM) Memory
	ld u_data,X+
	rcall eeprom_write
	dec Bcnt2
	brne w94
	rjmp put_ret

;Entry Point for Flash Block write
w94a:
	table	Dev_M	
	rcall	bel_table
	brcc w95				; device belongs to table
	rjmp	w96				; not match, goto "non Paged" Mode
w95:						; Entry Point for AVRProg Block Flash write
	ldi	s_data,0x40			; write low byte
	rcall spi123
	ld	s_data, X+
	rcall	wrser			; wrser(s_data) SPI write (byte 4)
	ldi	s_data,0x48			; write high byte
	mov	pol_cmd,s_data		; save command for polling	
	rcall spi123
	ld	s_data, X+
	mov	p_data,s_data		; save data for polling
	mov	pol_al,addrl		; save address for polling
	mov	pol_ah,addrh
	rcall	wrser			; wrser(u_data) SPI write (byte 4)
	adiw	addrl,1			; Auto increment address
	dec Bcnt2
	breq w95b
	dec Bcnt2
	breq w95b
	tst B_Mode
	breq w95				; Loop for "Block Mode Commands" see Note 16
	dec Bcnt3				; count Bytes for Pagesize
	brne w95
w95b:
	tst B_Mode
	brne w95c
	rjmp put_ret
w95c:
	dec Bcnt3				; Bcnt3 will be set correct after write
	ldi	s_data,0x4c			; write Memory Page
	rcall	wrser			; wrser(0x4c)  SPI write (byte 1)
	mov	s_data,pol_ah		; last written Adress is in Page to be programmed!
	rcall	wrser			; wrser(addrh) SPI write (byte 2)
	mov	s_data,pol_al		; last written Adress is in Page to be programmed!
	rcall	wrser			; wrser(addrl) SPI write (byte 3)
	ldi	s_data,0x00
	rcall	wrser			; wrser(0x00)  SPI write (byte 4)
	mov Bcnt3,Pagewords		; reload Counter für Pagesize
	rjmp wait_M


;#@KL4 test for 89S8252 Device
w96:						; Non Page Mode Flash write
	tst	device
	brmi	w96b			; S89 device
	ld	p_data, X+			; load byte to Poll data
	cpi	p_data,0xFF			; if (p_data == 0xFF) 
	breq	w961			; skip burning
	ldi	s_data,0x40			; write low byte
	mov	pol_cmd,s_data		; save command for polling	
	rcall spi123
	mov	s_data,p_data		; reload data from Poll data
	rcall	wrser			; wrser(s_data) SPI write (byte 4)
	mov	pol_al,addrl		; save address for polling
	mov	pol_ah,addrh
	rcall Bws_pol
w961:
	dec Bcnt2
	ld	p_data, X+			; load byte to Poll data
	cpi	p_data,0xFF			; if (p_data == 0xFF) 
	breq	w962
	ldi	s_data,0x48			; write high byte
	mov	pol_cmd,s_data		; save command for polling	
	rcall spi123
	mov	s_data,p_data		; reload data from Poll data
	rcall	wrser			; wrser(u_data) SPI write (byte 4)
w96a:	mov	pol_al,addrl	; save address for polling
	mov	pol_ah,addrh
	rcall Bws_pol
w962:
	adiw	addrl,1			; Auto increment address
	dec Bcnt2
	brne w96
	rjmp put_ret			; reply

w96b:					; Code for 89S8252
;	ldi	s_data,0x40
;	mov	pol_cmd,s_data		; save command for polling	
	mov	s_data,addrh		; s_data = (addrh << 3) | 0x02;
	rcall	shift_s_data3
	ori	s_data,0x02
	rcall	wrser			; wrser(addrh)  SPI write (byte 1)
	mov	s_data,addrl	
	rcall	wrser			; wrser(addrl)  SPI write (byte 2)
	ld	s_data, X+	
	mov	p_data,s_data		; save data for polling
	rcall	wrser			; wrser(u_data) SPI write (byte 3)
	rjmp	w96a

;====== 'R' - Read Program Memory ==========================================

w10:	
	cpi	u_data,'R'		; 'R' Read program memory
	brne	w10B
	tst	device
	brmi	rpm1			; S89 device
	ldi	s_data,0x28			; read high Byte (order is different from Block Read!)
	rcall	wrser			; wrser(0x28)  SPI write (byte 1)
	mov	s_data,addrh
	rjmp	rpm2
rpm1:
	mov	s_data,addrh		; s_data = (addrh << 3) | 0x01;
	rcall	shift_s_data3
	ori	s_data,0x01
rpm2:
	rcall	wrser			; wrser(addrh) SPI write (byte 2)
	mov	s_data,addrl	
	rcall	wrser			; wrser(addrl) SPI write (byte 3)
	rcall	rdser			;              SPI read  (byte 4)
	mov	u_data,s_data
	rcall	putc			; send data
	tst	device
	brmi	rpm3			; S89 device
	ldi	s_data,0x20			; read Low Byte
	rcall	wrser			; wrser(0x20)  SPI write (byte 1)
	mov	s_data,addrh	
	rcall	wrser			; wrser(addrh) SPI write (byte 2)
	mov	s_data,addrl	
	rcall	wrser			; wrser(addrl) SPI write (byte 3)
	rcall	rdser			;              SPI read  (byte 4)
	mov	u_data,s_data
	rcall	putc			; send data
rpm3:
	adiw	addrl,1			; Auto increment address
	rjmp	waitcmd			; goto waitcmd();


;====== 'g' - Block Read Memory (See Note 17) ======================

w10B:	
	cpi	u_data,'g'		; 'g' Block Read Program Memory
	brne	w11
	rcall	getc			; XH = getc();
	mov	XH,u_data
	rcall	getc			; XL = getc();
	mov	XL,u_data
	rcall	getc			; getc(Memorytype);
	cpi u_data,'F'
	breq w10B2
	cpi u_data,'E'
	breq w10B1
	rjmp put_err
w10B1:
	rcall read_send_datamem
	sbiw XL, 1
   	brne w10B1
	rjmp waitcmd			; goto waitcmd();

w10B2:
 	rcall read_send_progmem
	tst device
	brmi w10B3
	sbiw XL, 2
	brne w10B2
	rjmp waitcmd
w10B3:
	sbiw XL, 1
   	brne w10B2
	rjmp waitcmd			; goto waitcmd();

;====== 'A' - Load Address =================================================

w11:	
	cpi	u_data,'A'		; 'A' Load address
	brne	w12
	rcall	getc			; addrh = getc();
	mov	addrh,u_data
	rcall	getc			; addrl = getc();
	mov	addrl,u_data
	rjmp	put_ret			; goto reply();

;====== 'D' - Write Data Memory ============================================

w12:	
	cpi	u_data,'D'		; 'D' Write data memory
	brne	w13
	rcall	getc			; get data
	rcall eeprom_write
	rjmp	put_ret



;====== 'd' - Read Data Memory =============================================

w13:	
	cpi	u_data,'d'		; 'd' Read data memory
	brne	w14
	rcall   read_send_datamem
	rjmp	waitcmd			; goto waitcmd();


;====== 'L' - Leave Programming Mode =======================================

w14:	
	cpi	u_data,'L'		; 'L' Leave programming mode
	brne	w142
w141:
	pas_RESET				; set RESET passive
	release_ports			; release ports
	set_LED					; LED off
	rjmp    put_ret

;====== 'E' - Exit , release all Ports, inhibit AVR910 =====================

w142:	
	cpi	u_data,'E'		; 'E' xit
	brne	w15
	rjmp w141				; exit command for AVR Prog
	
;====== 'e' - Chip Erase ===================================================

w15:	
	cpi	u_data,'e'     		; 'e' Chip erase
	brne	w16
	ldi	s_data,0xac
	rcall	wrser			; wrser(0xac) SPI write (byte 1)
	tst	device
	brmi	w151			; S89 device
	ldi	s_data,0x80
	rcall	wrser			; wrser(0x80) SPI write (byte 2)
w151:
	ldi	s_data,0x04
	rcall	wrser			; wrser(0x04) SPI write (byte 3)
	ldi	s_data,0x00
	rcall	wrser			; wrser(0x00) SPI write (byte 4)
	ldi	temp1,40			; delay 40mS
	rcall	delay
	rjmp	put_ret

;====== 'l' - Write Lockbits ==============================================

; writes Lockbits in SPI Byte 2 for old "classic" AVR Chips, and in addition
; writes Lockbits in SPI byte 4 for new Mega and Tiny Devices. 
; One Type does not care about the other Byte...

; Some more different Adresses for other devices?

w16:	
	cpi	u_data,'l'		; 'l' Write Lockbits
	brne	w161
	rcall	getc			; get data
	ldi	s_data,0xac
	rcall	wrser			; wrser(0xac)   SPI write (byte 1)
	tst	device
	brmi	w163			; S89 device
	mov	s_data,u_data 
	ori	s_data,0xe0
	rcall	wrser			; wrser(u_data) SPI write (byte 2)
	rjmp 	w162

;====== 'f' - Write Fusebits ==============================================

w161:
	cpi	u_data,'f'		; 'f' Write Fusebits
	brne	w17
	rcall	getc			; get data
	ldi	s_data,0xac
	rcall	wrser			; wrser(0xac)   SPI write (byte 1)
	ldi	s_data,0xa0
	rcall	wrser			; wrser(0xa0)   SPI write (byte 2)
w162:
	clr s_data
	rcall	wrser			; wrser(0x00)   SPI write (byte 3)
	mov	s_data,u_data 
	rcall	wrser			; wrser(u_data) SPI write (byte 4)
	rjmp	w164
w163:
	mov	s_data,u_data 
	andi	s_data,0xe0		; S89 device
	ori	s_data,0x07
	rcall	wrser			; SPI write (byte3)
	ldi	s_data,0x00
	rcall	wrser			; wrser(0x00)   SPI write (byte 4)
w164:
	ldi	temp1,10			; delay 10mS
	rcall	delay
	rjmp	put_ret


;====== 's' - Read Signature Bytes =========================================

w17:	
	cpi	u_data,'s'		; 's' Read signature bytes
	brne	w18
	tst		device
	brmi	w171			; S89 device
	ldi 	temp3,0x30
	mov 	cmd1,temp3
	clr 	cmd2
	ldi 	temp3,0x02
	mov 	cmd3,temp3
	rcall 	w123r4
	dec 	cmd3
	rcall 	w123r4
	clr 	cmd3
	rcall 	w123r4
	rjmp 	waitcmd
w171:
	rjmp	put_err


;====== 'm' - Write Program Memory Page ====================================

w18:	
	cpi	u_data,'m'		; 'm' Write Program Memory Page
	brne	w19
	ldi	s_data,0x4c	
	rcall	wrser			; wrser(0x4c)  SPI write (byte 1)
;	mov	s_data,addrh		; original, reload address
	mov	s_data,pol_ah		; for speeding up transfer
	rcall	wrser			; wrser(addrh) SPI write (byte 2)
;	mov	s_data,addrl		; original, reload address
	mov	s_data,pol_al		; for speeding up transfer
	rcall	wrser			; wrser(addrl) SPI write (byte 3)
	ldi	s_data,0x00
	rcall	wrser			; wrser(0x00)  SPI write (byte 4)
	rjmp	wait_M			; write FLASH delay

;====== ':' - Universal Command ============================================

w19:	
	cpi	u_data,':'		; ':' Universal Command
	brne	w20
	rcall	getc			; get data1
	mov	cmd1,u_data		; cmd1 = data1
	rcall	getc			; get data2
	mov	cmd2,u_data		; cmd2 = data2
	rcall	getc			; get data3
	mov	cmd3,u_data		; cmd3 = data3
	rcall w123r4
	ldi	temp1,50			; delay 50mS
	rcall	delay
	rjmp	put_ret

;====== '.' - New Universal Command ========================================

w20:	
	cpi	u_data,'.'		; '.' New Universal Command
	brne	w21
	rcall	getc			; get data1
	mov	cmd1,u_data			; cmd1 = data1
	rcall	getc			; get data2
	mov	cmd2,u_data			; cmd2 = data2
	rcall	getc			; get data3
	mov	cmd3,u_data			; cmd3 = data3
	rcall	getc			; get data4
	rcall w1234
	mov	u_data,rd_s_data
	rcall	putc			; send data
	ldi	temp1,50			; delay 50mS
	rcall	delay
	rjmp	put_ret
	
;====== 'r' - Read Lock Bits ==============================================

w21:	
	cpi	u_data,'r'		; 'r' Read Lockbits
	brne	w22
	ldi	s_data,0x58			; SPI load (byte 1)
	rjmp	w22a

;====== 'F' - Read Fusebits (lfuse) =========================================

w22:
	cpi	u_data,'F'		; 'F' Read Fusebits
	brne	w23
	ldi	s_data,0x50			; SPI load (byte 1)
w22a:
	rcall	wrser			; wrser(0x5n)   SPI write (byte 1)
	clr s_data				; SPI load (byte 2)
	rjmp w26

;====== 'N' - Read high Fusebits ==========================================

w23:
	cpi	u_data,'N'		; 'N' Read high Fusebits
	brne	w24
	ldi	s_data,0x58
	rcall	wrser			; wrser(0x58)   SPI write (byte 1)
	rjmp	w25

;====== 'Q' - Read extendet Fusebits ==============================================

w24:
	cpi	u_data,'Q'		; 'Q' Read extendet Fusebits
	brne	w30
	ldi	s_data,0x50			; SPI load (byte 1)
	rcall	wrser			; wrser(0x50)   SPI write (byte 1)
w25:
	ldi	s_data,0x08			; SPI load (byte 2)
w26:
	rcall	wrser			; wrser(0x00)   SPI write (byte 2)
	clr s_data
	rcall	wrser			; wrser(0x00)   SPI write (byte 3)
	rcall	rdser			;               SPI read  (byte 4)
	mov	u_data,s_data
	rcall	putc			; send data
	rjmp waitcmd

;====== Unknown Command ====================================================

w30:	rjmp	put_err

;====== Wait for FLASH write in avr910 mode ==================================

wait_S:
	table	Dev_M	
	rcall	bel_table
	brcs	ws_pol			; No Page Mode, poll last Byte
	rjmp	put_ret			; in Byte Mode time is long enougt to get next Byte...
Bws_pol:				; entry adress for Block mode delay
	ldi temp3,1
	mov B_Flag,temp3		; set Flag that ws_poll comes back with ret ...

ws_pol:
	tst	PollcodeF		; if polling not applicable, standart delay
	breq	ws_del			; polling not used
ws_def:
	cp 	p_data,PollcodeF	; if (data == PollcodeF)
	breq 	ws_del			; wait default delay
	andi	pol_cmd,0x0f		; write command: 0x48, 0x40
	ori	pol_cmd,0x20		; read  command: 0x28, 0x20
	clr	temp3				; clear polling counter
ws_cy:
	tst	device
	brmi	ws_89			; S89 device
	mov	s_data,pol_cmd	
	rcall	wrser			; wrser(pol_cmd) SPI write (byte 1)
	mov	s_data,pol_ah
	rjmp	ws_90
ws_89:
	mov	s_data,pol_ah		; s_data = (pol_ah << 3) | 0x01;
	rcall	shift_s_data3
	ori	s_data,0x01
ws_90:
	rcall	wrser			; wrser(pol_ah) SPI write (byte 2) (S89=byte1)
	mov	s_data,pol_al	
	rcall	wrser			; wrser(pol_al) SPI write (byte 3) (S89=byte2)
	rcall	rdser			;               SPI read  (byte 4) (S89=byte3)
	tst	device
	brpl	ws_cb
	andi	s_data,0x80		; compare only MSB for S89 device
	andi	p_data,0x80
ws_cb:					
	cp	s_data,p_data
	breq	ws_ok			; s_data = p_data
	dec	temp3
	brne	ws_cy			; loop
ws_del:						; 256 polling cycles are over, give additional standart Time
	ldi	temp1,10			; delay 10mS
	rcall	delay
ws_ok:
	tst B_Flag
	breq put_ret
	clr B_Flag				; Reset B_Flag for normal operation 
	ret

;====== Wait for FLASH write in page mode ==================================

wait_M:
	cpi	device,0x41
	breq	wm_del			; polling inapplicable for m103
	cpi	device,0x23
	breq	poll_t2313		; polling different for t2313
	cpi	p_data, 0xFF		; if last Byte was 0xFF give standart delay
	breq	wm_del
	andi	pol_cmd,0x0f		; write command: 0x48, 0x40
	ori	pol_cmd,0x20		; read  command: 0x28, 0x20
	clr	temp3				; clear polling counter
wm_cy:
	mov	s_data,pol_cmd	
	rcall	wrser			; wrser(pol_cmd) SPI write (byte 1)
	mov	s_data,pol_ah
	rcall	wrser			; wrser(pol_ah) SPI write (byte 2)
	mov	s_data,pol_al	
	rcall	wrser			; wrser(pol_al) SPI write (byte 3)
	rcall	rdser			;               SPI read  (byte 4)
	cp	s_data,p_data
	breq	wm_ok			; s_data = p_data
	dec	temp3
	breq	wm_del			; 256 polling cycles are over, give another 50ms delay...
	rjmp	wm_cy			; loop
wm_del:
	ldi	temp1,50			; delay 50mS
	rcall	delay
wm_ok:
	tst B_Mode
	breq wm_end
	tst Bcnt2
	breq wm_end
	rjmp w95
wm_end:
	clr B_Mode				; Reset Block Mode Flag
	rjmp put_ret

poll_t2313:					; Polling ATTiny2313 is different
	ldi	pol_cmd,0xf0		; Byte1= 0xF0
	clr	temp3				; clear polling counter
	mov	pol_ah,temp3		; Byte2= 0x00
p2313_1:
	mov	s_data,pol_cmd	
	rcall	wrser			; wrser(pol_cmd) SPI write (byte 1)
	mov	s_data,pol_ah
	rcall	wrser			; wrser(pol_ah) SPI write (byte 2)
	mov	s_data,pol_al	
	rcall	wrser			; wrser(pol_al) SPI write (byte 3)
	rcall	rdser			;               SPI read  (byte 4)
	andi	s_data,0x01		; last Bit tells if polling OK
	breq	wm_ok			; s_data = p_data
	dec	temp3
	breq	wm_del			; 256 polling cycles are over, give another 50ms delay...
	rjmp	p2313_1			; loop


;====== Command Error ======================================================

put_debug:					; shows u_data on serial for debug reason				
	rcall putc
put_err:
	ldi	u_data,'?'			; putc('?')
	rcall	putc			; send '?'
	rjmp	waitcmd		
	
;====== Reply Command ======================================================

put_ret:
	ldi	u_data,0x0d			; putc(0x0D)
	rcall	putc			; send CR
	rjmp	waitcmd	

;***************************************************************************
;*
;* TABLE
;*	device codes
;*
;* DESCRIPTION
;*	The following device codes must be used by the host computer. Note
;*	that the device codes are arbitrary selected, they do not have any
;*	thing in common with the signature bytes stored in the device.
;* 	This are the device Codes recognized by the AVRprog Software. Some
;*	Devices may require special hardware or a different, not yet 
;*	implemented Protocol! Use at your own risk.
;*
;***************************************************************************

Dev_S:	; byte write
;      avr910-devcode   ,Code during polling Flash. 0x00: no polling supported, program default time
;	.db	0x10	,0x00	;AT90S1200A No support for this type
;	.db	0x11	,0x00	;AT90S1200B No support for this type
;	.db	0x12	,0x00	;AT90S1200C No support for this type
	.db	0x13	,0x00	;AT90S1200
	.db	0x20	,0x7F	;AT90S2313A
	.db	0x28	,0x7F	;AT90S4414A
	.db	0x30	,0xFF	;AT90S4433A
	.db	0x34	,0xFF	;AT90S2333A
	.db	0x38	,0x7F	;AT90S8515A
	.db	0x48	,0xFF	;AT90S2323A
	.db	0x4c	,0xFF	;AT90S2343A
	.db	0x51	,0xFF	;tn10
	.db	0x55	,0xFF	;tn12
	.db	0x56	,0xFF	;tn15
	.db	0x68	,0xFF	;AT90S8535
	.db	0x6c	,0xFF	;AT90S4434
	.db	0x86	,0xFF	;AT89S8252  bug in avrprog in Block write Mode!(See Note 18 for Workaround)
	.db	0x87	,0xFF	;AT89S53    bug in avrprog
	.dw	0		;End of table
	
Dev_M:	; Devices which support Page Programming. Dont forget the Page Size 
		; of different Devices.
		; Maximum ist 0x40 due to limitated RAM in 2313 Chip. 
		; (Pages with bigger Pages are programmed multiple times)


; unofficial Device Codes. Match avrdude.conf with tis "avr910-devcode"

; avr910-devcode ,Pagesite in Words
	.db 0x01	,0x40	;m640 	avr910-Devicecode not official!
;	.db 0x02	,0x40	;m644 	avr910-Devicecode not official!
;	.db 0x03	,0x40	;m645 	avr910-Devicecode not official!
	.db 0x04	,0x40	;m649 	avr910-Devicecode not official!
	.db 0x05	,0x40	;m6490 	avr910-Devicecode not official!
	.db 0x06	,0x40	;90PWM2	avr910-Devicecode not official!
	.db 0x07	,0x40	;90PWM3	avr910-Devicecode not official!
	.db 0x08	,0x40	;m1280 	avr910-Devicecode not official!
	.db 0x09	,0x40	;m1281	avr910-Devicecode not official!
;	.db 0x0a	,0x40	;m2560 	avr910-Devicecode not official!
;	.db 0x0b	,0x40	;m2561	avr910-Devicecode not official!
;	.db 0x0c	,0x40	;m3250 	avr910-Devicecode not official!
;	.db 0x0d	,0x40	;m6450	avr910-Devicecode not official!
	.db 0x0e	,0x10	;tn24 	avr910-Devicecode not official!
	.db 0x1a	,0x10	;tn25 	avr910-Devicecode not official!
	.db 0x0f	,0x20	;tn44	avr910-Devicecode not official!
	.db 0x1b	,0x20	;tn45	avr910-Devicecode not official!
	.db 0x14	,0x20	;tn84	avr910-Devicecode not official!
	.db 0x1c	,0x20	;tn85	avr910-Devicecode not official!
	.db 0x1d	,0x40	;CAN128	avr910-Devicecode not official!
	.db 0x23	,0x10	;tn2313 avr910-Devicecode not official! (STK500 Code used)
	.db 0x31	,0x20	;m48	avr910-Devicecode not official!
	.db 0x33	,0x20	;m88 	avr910-Devicecode not official!
	.db 0x35	,0x40	;m168 	avr910-Devicecode not official!
;	.db 0x36	,0x40	;m165 	avr910-Devicecode not official!
	.db 0x37	,0x40	;m164 	avr910-Devicecode not official!
	.db 0x39	,0x40	;m324 	avr910-Devicecode not official!
;	.db 0x3c	,0x40	;m325 	avr910-Devicecode not official!
	.db 0x3d	,0x40	;m329 	avr910-Devicecode not official!
	.db 0x3e	,0x40	;m3290 	avr910-Devicecode not official!
	.db 0x57	,0x10	;tn13 	avr910-Devicecode not official! 

;Appearance of this Devicecodes does not mean this Devices are or will be fully supported !



; official Devicecodes as matched in AVRProg V1.40
	.db 0x3a	,0x20	;m8515, Pagesize 32 words (0x20)
	.db 0x3b	,0x20	;m8515boot  Bootloader Mode untested!
	.db	0x41	,0x40	;m103	
	.db	0x43	,0x40	;m128
	.db	0x44	,0x40	;m128boot  Bootloader Mode untested!
	.db	0x45	,0x40	;m64
	.db	0x46	,0x40	;m64boot   Bootloader Mode untested!
	.db	0x5e	,0x10	;tn26
	.db	0x60	,0x40	;m161
	.db	0x61	,0x40	;m161boot  Bootloader Mode untested!
	.db	0x62	,0x40	;m162
	.db	0x63	,0x40	;m162boot  Bootloader Mode untested!
	.db	0x64	,0x40	;m163
	.db	0x66	,0x40	;m163boot  Bootloader Mode untested!
	.db 0x69	,0x20	;m8535
	.db 0x6a	,0x20	;m8535boot Bootloader Mode untested!
	.db	0x72	,0x40	;m32
	.db	0x73	,0x40	;m32boot   Bootloader Mode untested!
	.db	0x74	,0x40	;m16
	.db	0x75	,0x40	;m16boot   Bootloader Mode untested!
	.db	0x76	,0x20	;m8
	.db	0x77	,0x20	;m8boot    Bootloader Mode untested!
	.db	0x78	,0x40	;m169
	.db	0x79	,0x40	;m169boot  Bootloader Mode untested!
	.dw 	0		;End of Table


; Devices with known avr910 Devicecodes, but not supported with this Programmer
;	.db	0x42	,0x40	;m603	obsolete
;	.db	0x50			;tn11 Needs additional High Voltage Hardware and uses different Protocoll! No Support!
;	.db	0x58			;tn19 Obsolete
;	.db	0x5c			;tn28 Only supported in parallel Programming Mode!
;	.db	0x65	,0x20	;m83	obsolete 
;	.db	0x67	,0x20	;m83boot  obsolete
;	.db	0x70			;AT90C8534  unknown Hardware, untested!
;	.db	0x71			;AT90C8544  unknown Hardware, untested!
;	.db	0x80			;AT89C1051  unknown Hardware, untested!
;	.db	0x81			;AT89C2051  unknown Hardware, untested!



;***************************************************************************
;*
;* TABLE
;*	revision codes
;*
;***************************************************************************

SW_Ver:
	.db "38",0,0
HW_Ver:
	.db "12",0,0

;***************************************************************************
;*
;* TABLE
;*	ID string "AVR ISP"
;*
;***************************************************************************

ID_Str:
	.db "AVR ISP",0

;***************************************************************************
;*
;* TABLE
;*	Chip ID string to identify the Firmware
;*
;***************************************************************************

ChipID:
	.db "Ver.3.8b (AVR109 Mode, 7.3728Mhz, 115.200 baud) for AN910, AT90S2313"
	.db "www.mikrocontroller-projekte.de 26.Feb.2006",0

;**** End of File ****
