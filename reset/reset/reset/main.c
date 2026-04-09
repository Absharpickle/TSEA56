#include <avr/io.h>


int main(void)
{
	DDRD |= (1 << PD6);
    while (1) 
    {
		PORTD ^= (1 << PD6);
    }
}

