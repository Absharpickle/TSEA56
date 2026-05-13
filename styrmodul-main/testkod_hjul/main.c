#define F_CPU 16000000UL
#include <util/delay.h>
#include "hjul.h"
#include <avr/io.h>


int main(void)
{
    motor_init();
	start_motor_hard(FORWARD);
	_delay_ms(5000);
	stop_motor_hard(FORWARD, BRAKE_NORMAL);
    while (1) 
    {
    }
}

