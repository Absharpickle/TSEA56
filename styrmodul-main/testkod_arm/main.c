#include <avr/io.h>
#include <avr/interrupt.h>
#define F_CPU 16000000UL
#include <util/delay.h>
#include "UART.h"



int main()
{
	uint16_t position;
	init_UART();
	for (uint8_t i = 0; i<10; i++)
	{
		move_joint(1, INC);
	}
	
	
	_delay_ms(500);
	return 0;
}



