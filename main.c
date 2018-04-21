/*
 * radio.c
 *
 *  Created on: 2 mar 2017
 *      Author: Patryk
 */

#include<avr/io.h>
#include<util/delay.h>
#include <util/atomic.h>
#include "source\nrf24l01\rf24l01.h"
#include "source\uart\my_uart.h"
#include "source\memops\memops.h"
#include "source\robot\protocol.h"

#define CONT_LED PC5
#define LED_PORT PORTC
#define LED_DDR DDRC
#define time1ms 255 -31
#define UART 	1
#define POSUB	0
#define D_WIDTH 15

inline void ledON();
inline void ledOFF();
inline void ledToggle();

void radioTransmit();
void radioPrepareNextTransmission();
void processDataFromRadio(uint8_t data[], uint8_t count);

void uartRequestCollect(uint8_t data);
void uartPacketWorkout(uint8_t* packet, uint8_t count);
int getCharPos(char character, uint8_t *table, uint8_t size);
int32_t getValueFromTable(uint8_t *table, uint8_t size);

float getXYValue(uint8_t* packet, uint8_t count);

volatile uint8_t irCounter;
enum nrfState {REC, WFBT, WFBR, TRA1, TRA2, WFTR, WFRE};
enum nrfState RadioState = TRA1;
volatile uint8_t radio_actionTimer = 0;
volatile uint8_t transmitTrigger = 1;
volatile uint8_t radioBusy = 0, lastSent = POSUB, uPCount = 0;
uint8_t nextCPTimer = 0;

//radio data
uint8_t radioSendBufor[D_WIDTH];
uint8_t radioRecBufor[D_WIDTH];

//robot positions
double posX = 0, posY = 0, angle = 0;
double goalPosX = 0, goalPosY = 0;

//uart connection
volatile uint8_t uartHeaderReceived = 0;
uint8_t uartSendBufor[30];
uint8_t uartRecBufor[30];
uint8_t uartRecCounter = 0, uartTransmitTrigger = 0, uartFlushTimer = 0, isRobotPositionSubscribed = 1;
volatile uint8_t positionRequest = 0;

volatile uint16_t counter=0;


int main()
{
	LED_DDR |= (1 << CONT_LED);

	TCCR0 |= (1 << CS02);
	TIMSK |= (1 << TOIE0);
	sei();

	spi_Init(1, 0);
	radio_Init(0, 1, D_WIDTH);
	_delay_ms(1000);
	if(radio_ReadRegister(STATUS) == 0b00001110)
		ledON();

	_delay_ms(2000);
		ledOFF();
	radioRecBufor[0] = 0;

	uart_init();

	uart_sendString("Initialization");
	uart_sendByteAsChar(radio_ReadRegister(STATUS));

	while(1)
	{
		radioPrepareNextTransmission();
		radioTransmit();
	}
}


ISR(TIMER0_OVF_vect)
{
	TCNT0 = time1ms;
	irCounter ++;
	counter ++;
	if(counter == 20)
	{
		counter = 0;
	}
	if(0 != uartFlushTimer && irCounter == uartFlushTimer)
	{
		uartRecCounter = 0;
		uartFlushTimer = 0;
		uartHeaderReceived = 0;
	}
}

ISR(USART_RXC_vect)
{
	uartRequestCollect(uart_receive());
}

inline void ledON()
{
	LED_PORT |= 1 << CONT_LED;
}
inline void ledOFF()
{
	LED_PORT &= ~(1 << CONT_LED);
}

inline void ledToggle()
{
	LED_PORT ^= (1 << CONT_LED);
}

void uartRequestCollect(uint8_t data)
{
	if(uartHeaderReceived)
	{
	     if(data == '\r')
	     {
	    	 uartPacketWorkout(uartRecBufor, uartRecCounter);
	    	 uartRecCounter = 0;
	    	 uartHeaderReceived = 0;
	     }
	     else
	     {
	    	 uartRecBufor[uartRecCounter] = data;
	    	 uartRecCounter ++;
	     }
	}
	else
	{
		if(data == 'P')
			uartHeaderReceived = 1;
	}
}

void uartPacketWorkout(uint8_t* packet, uint8_t count)
{
	switch (packet[0])
	{
		case USETX:
		{
			float val;
			val = getXYValue(packet, count);
			if(val == 40)
				ledToggle();
			uartSendBufor[0] = SETGX;
			getBytes(val, uartSendBufor + 1);
			uartTransmitTrigger = 1;
			break;
		}
		case USETY:
		{
			float val;
			val = getXYValue(packet, count);
			uartSendBufor[0] = SETGY;
			getBytes(val, uartSendBufor + 1);
			uartTransmitTrigger = 1;
			break;
		}
		case UENGS:
		{
			ledToggle();
			uartSendBufor[0] = SETEF;
			uartSendBufor[1] = packet[2];
			uartSendBufor[2] = packet[4];
			if(packet[1] != 0)
				uartSendBufor[1] |= 0x80;
			if(packet[3] != 0)
				uartSendBufor[2] |= 0x80;
			uartTransmitTrigger = 1;
			break;
		}
		case ULFOL:
		{
			uartSendBufor[0] = FOLIN;
			uartTransmitTrigger = 1;
			break;
		}
		case UPFOL:
		{
			uartSendBufor[0] = FOTOP;
			uartTransmitTrigger = 1;
			break;
		}
		case USTOP:
		{
			uartSendBufor[0] = RSTOP;
			uartTransmitTrigger = 1;
			break;
		}
		case USETP:
		{
			uartSendBufor[0] = SETPO;
			int32_t pos, x, y, a;
			count--;
			pos = getCharPos('X', packet, count);
			x = getValueFromTable(packet + pos + 1, count - pos - 1);
			pos = getCharPos('Y', packet, count);
			y = getValueFromTable(packet + pos + 1, count - pos - 1);
			pos = getCharPos('A', packet, count);
			a = getValueFromTable(packet + pos + 1, count - pos - 1);

			double val = x / 1000.0;
			getBytes(val, uartSendBufor + 1);
			val = y / 1000.0;
			getBytes(val, uartSendBufor + 5);
			val = a / 100000.0;
			getBytes(val, uartSendBufor + 9);
			uartTransmitTrigger = 1;
			break;
		}
		case USETB:
		{
			uartSendBufor[0] = SETBA;
			int pos, wheelSize, wheelSpacing;
			count--;

			pos = getCharPos('W', packet, count);
			wheelSize = getValueFromTable(packet + pos + 1, count - pos - 1);
			pos = getCharPos('S', packet, count);
			wheelSpacing = getValueFromTable(packet + pos + 1, count - pos - 1);

			double val = wheelSize / 1000.0;
			getBytes(val, uartSendBufor + 1);
			val = wheelSpacing / 1000.0;
			getBytes(val, uartSendBufor + 5);
			uartTransmitTrigger = 1;
			break;
		}
		case UGETB:
		{
			uartSendBufor[0] = GETBA;
			uartTransmitTrigger = 1;
			break;
		}
		case USETW:
		{
			uartSendBufor[0] = SETWE;
			int pos, nSensor, mSensor, fSensor;

			pos = getCharPos('N', packet, count);
			nSensor = getValueFromTable(packet + pos + 1, count - pos - 1);
			pos = getCharPos('M', packet, count);
			mSensor = getValueFromTable(packet + pos + 1, count - pos - 1);

			pos = getCharPos('F', packet, count);
			fSensor = getValueFromTable(packet + pos + 1, count - pos - 1);

			double val = nSensor / 1000.0;
			getBytes(val, uartSendBufor + 1);
			val = mSensor / 1000.0;
			getBytes(val, uartSendBufor + 5);
			val = fSensor / 1000.0;
			getBytes(val, uartSendBufor + 9);
			uartTransmitTrigger = 1;
			break;
		}
		case UGETW:
		{
			uartSendBufor[0] = GETWE;
			uartTransmitTrigger = 1;
			break;
		}
		case USETL:
		{
			uartSendBufor[0] = SETLF;
			int pos, kp, tp;
			count--;

			pos = getCharPos('K', packet, count);
			kp = getValueFromTable(packet + pos + 1, count - pos - 1);
			pos = getCharPos('T', packet, count);
			tp = getValueFromTable(packet + pos + 1, count - pos - 1);

			double val = kp / 1000.0;
			getBytes(val, uartSendBufor + 1);
			val = tp / 1000.0;
			getBytes(val, uartSendBufor + 5);
			uartTransmitTrigger = 1;
			break;
		}
		case UGETL:
		{
			uartSendBufor[0] = GETLF;
			uartTransmitTrigger = 1;
			break;
		}
		default:
		{
			break;
		}
	}
}

int getCharPos(char character, uint8_t *table, uint8_t size)
{
    int i;
    for(i = 0; i <= size; i++)
    {
        if(table[i] == character)
            return i;
    }
    return -1;
}

int32_t getValueFromTable(uint8_t *table, uint8_t size)
{
    uint8_t temp[20];
    int i = 0;
    char isMinus = 0;

    if(table[0] == '-')
    {
        table++;
        size--;
        isMinus = 1;
    }

    while(table[i] >= '0' && table[i] <= '9' && i <= size)
    {
        temp[i] = table[i];
        i++;
    }
    if(i == 0)
        return -1;

    int32_t value = 0;
    int j = 0;
    for(; i > 0; i--)
    {
        value += round(temp[j] - 48) * round(pow(10, (i-1)));
        j++;
    }

    if(isMinus)
        return -value;
    return value;
}

void radioPrepareNextTransmission()
{
	if(radioBusy == 0)
	{
		if(1 == uartTransmitTrigger && 1 == isRobotPositionSubscribed)
		{
			if(lastSent == UART)
			{
				radioSendBufor[0] = GETPOS;
				lastSent = POSUB;
				transmitTrigger = 1;
			}
			else
			{
				int i;
				ATOMIC_BLOCK(ATOMIC_FORCEON)
				{
					for(i = 0; i < D_WIDTH; i++)
					{
						radioSendBufor[i] = uartSendBufor[i];
						transmitTrigger = 1;
					}
				}
				uartTransmitTrigger = 0;
				uPCount ++;
				if(uPCount > 5)
				{
					lastSent = UART;
					uPCount = 0;
				}
			}
		}
		else
			if(uartTransmitTrigger)
			{
				int i;
				ATOMIC_BLOCK(ATOMIC_FORCEON)
				{
					for(i = 0; i < D_WIDTH; i++)
					{
						radioSendBufor[i] = uartSendBufor[i];
						transmitTrigger = 1;
					}
				}
				uartTransmitTrigger = 0;
				lastSent = UART;
			}
			else
				if(isRobotPositionSubscribed)
				{
					radioSendBufor[0] = GETPOS;
					lastSent = POSUB;
					transmitTrigger = 1;
				}
				else
				{
					if(irCounter == nextCPTimer)
					{
						radioSendBufor[0] = COPAC;
						transmitTrigger = 1;
						nextCPTimer = irCounter + 200;
					}
				}
		}
}

void radioTransmit()
{
	switch (RadioState)
	{
		case TRA1:
		{
			if(transmitTrigger == 1)
			{
				radioBusy = 1;
				radio_PreparePayload(radioSendBufor, D_WIDTH);
				radio_actionTimer = irCounter + 10;
				transmitTrigger = 0;
				RadioState = TRA2;
			}
			else
			{
				radioBusy = 0;
			}
			break;
		}
		case TRA2:
		{
			if(irCounter == radio_actionTimer)
			{
				radio_Transmit();
				radio_actionTimer = irCounter + 10;
				RadioState = WFTR;
			}
			break;
		}
		case WFTR:
		{
			if(radio_IsInterruptRequest() || irCounter == radio_actionTimer)
			{
				if(radio_WasTransmissionSuccessfull())
				{
					if(isRequest(radioSendBufor[0]))
					{
						radio_Reset();
						radio_SwitchReceiver();
						radio_actionTimer = irCounter + 30;
						RadioState = WFBR;
						ledToggle();
					}
					else
					{
						RadioState = TRA1;
					}
				}
				else
				{
					radio_Reset();
					radio_Init(0, 1, D_WIDTH);
					radio_actionTimer = irCounter + 10;
					RadioState = WFBT;
				}
			}
			break;
		}
		case WFBR:
				{
					if(irCounter == radio_actionTimer)
					{
						radio_StartListenning();
						radio_actionTimer = irCounter + 30;
						RadioState = WFRE;
					}
					break;
				}
		case WFRE:
				{
					if(radio_IsInterruptRequest() || irCounter == radio_actionTimer)
					{
						radio_StopListenning();
						if(radio_WasDataReceived())//if receiving succesfull
						{
							while(!radio_IsReceivingBuforEmpty())
							{
								radio_Receive(radioRecBufor, D_WIDTH);
							}
							processDataFromRadio(radioRecBufor, D_WIDTH);
							ledToggle();
						}
						else
						{
							//errorInform();
						}
						radio_Reset();
						radio_SwitchTransmiter();
						RadioState = WFBT;
						radio_actionTimer = irCounter + 25;
					}
					break;
				}
		case WFBT:
		{
			if(irCounter == radio_actionTimer)
			{
				RadioState = TRA1;
			}
			break;
		}
		case REC:
		{
			break;
		}

	}
}

void processDataFromRadio(uint8_t data[], uint8_t count )
{
	uart_sendPacket(data, count);
}

float getXYValue(uint8_t* packet, uint8_t count)
{
	float value = 0;
	int multiply = 1;

	if(packet[1] == '-')
	{
		int i;
		for(i = count - 1; i > 1; i--)
		{
			value += round((packet[i] - 48) * multiply);
			multiply *= 10;
		}

		return -(value/100);
	}
	else
	{
		int i;
		for(i = count - 1; i > 0; i--)
		{
			value += round((packet[i] - 48) * multiply);
			multiply *= 10;
		}

		return (value/100);
	}
}


