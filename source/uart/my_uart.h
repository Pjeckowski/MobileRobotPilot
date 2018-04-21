/*
 * my_uart.h
 *
 *  Created on: 9 maj 2017
 *      Author: Lapek
 */

#ifndef MY_UART_H_
#define MY_UART_H_



#include<avr/io.h>
#include<avr/interrupt.h>
#include<util/delay.h>
#include<avr/pgmspace.h>
#include<string.h>
#include "..\robot\protocol.h"
#include "..\memops\memops.h"

#define USETX 'X'
#define USETY 'Y'
#define ULFOL 'L'
#define UPFOL 'P'
#define USTOP 'S'
#define UENGS 'E'
#define UGETB 'B' //get base parameters
#define USETB 'C' //set base parameters
#define UMAXE 'R'
#define USETP 'N' // set robot positioN
#define PAEND '\r'


void uart_sendEngPacket(uint8_t header, uint8_t lEngine, uint8_t rEngine);
void uart_sendPosPacket(uint8_t header, float data);
void uart_sendPacket(uint8_t* data, uint8_t count);
void uart_sendBasePacket(uint8_t header, float data);
void uart_init()
{
	uint16_t baud=51;
	// sets baudrate to 9600 bps
	UBRRH = (baud >> 8);
	UBRRL = baud;

	UCSRC |= (1 << URSEL)|(1 << UCSZ0)|(1 << UCSZ1)|(1 << USBS)|(1 << UPM1); //8bit data frame, 2 stop bits, even parity

	UCSRB |= (1 << TXEN)|(1 << RXEN)|(1 << RXCIE); //enable transmitter, receiver, and data received interrupt

}

void uart_send(uint8_t data)
{
    while (!( UCSRA & (1<<UDRE))); // wait while register is free
    UDR = data;                    // load data in the register
}

uint8_t uart_receive()
{
	while(!((UCSRA) & (1<<RXC)));     // wait while data is being received
	return UDR;                     // return data
}

void uart_sendByteAsChar(uint8_t data)
{
	int i;
	for(i = 0; i < 8; i++)
	{
		if((data >> (7-i)) & 1)
			uart_send('1');
		else
			uart_send('0');
	}
	uart_send('\r');

}

void uart_sendTable(uint8_t* table, uint8_t count)
{
	int i;
	for(i = 0; i < count; i++)
	{
		uart_send(table[i]);
	}
	uart_send('\r');
}


void uart_sendString(char* table)
{
	int i;
	for(i = 0; i < strlen(table); i++)
	{
		uart_send(table[i]);
	}
	uart_send('\r');
}

void uart_sendValueAsChar(int32_t data)
{
	if(data == 0)
	{
		uart_send('0');
	}
	else
	{
		uint8_t tab[20];
		uint8_t counter = 0;

		if(data < 0)
		{
			data = -data;
			uart_send('-');
		}
		while(0 != data)
		{
			tab[counter] = data % 10;
			data /= 10;
			counter ++;
		}
		int i;
		for(i = counter - 1; i >= 0; i--)
		{
			uart_send(tab[i] + 48);
		}
	}
}

int intToCharTable(int32_t data, uint8_t* table)
{
	uint8_t count = 0;
	if(data == 0)
		{
			table[0] = 48;
			return 1;
		}
		else
		{
			uint8_t tab[20];
			uint8_t counter = 0;


			if(data < 0)
			{
				data = -data;
				table[0] = ('-');
				count ++;
			}
			while(0 != data)
			{
				tab[counter] = data % 10;
				data /= 10;
				counter ++;
			}
			int i;
			for(i = counter - 1; i >= 0; i--)
			{
				table[count] = tab[i] + 48;
				count ++;
			}
		}
	return count;
}

void uart_sendPacket(uint8_t data[], uint8_t count)
{
	switch (data[0])
	{
		case GETPOS:
		{
			float temp = getValFromBytes(data + 1);
			uart_sendPosPacket('X', temp);
			temp = getValFromBytes(data + 5);
			uart_sendPosPacket('Y', temp);
			temp = getValFromBytes(data + 9);
			uart_sendPosPacket('A', temp);
			uart_sendEngPacket('E', data[13], data[14]);
			break;
		}
		case GETBA:
		{
			float temp = getValFromBytes(data + 1);
			uart_sendBasePacket('W', temp);
			temp = getValFromBytes(data + 5);
			uart_sendBasePacket('S', temp);
			break;
		}
		default:
		{
			break;
		}
	}
}

void uart_sendPosPacket(uint8_t header, float data)
{
	uart_send('P');
	uart_send(header);
	uart_sendValueAsChar((int32_t)round(data * 10000.0));
	uart_send('\r');
}

void uart_sendBasePacket(uint8_t header, float data)
{
	uart_send('P');
	uart_send(header);
	uart_sendValueAsChar((int32_t)round(data * 10000.0));
	uart_send('\r');
}

void uart_sendEngPacket(uint8_t header,uint8_t lEngine, uint8_t rEngine)
{
	uart_send('P');
	uart_send(header);
	uart_sendValueAsChar((int32_t) lEngine);
	uart_send(',');
	uart_sendValueAsChar((int32_t) rEngine);
	uart_send('\r');
}

#endif /* MY_UART_H_ */
