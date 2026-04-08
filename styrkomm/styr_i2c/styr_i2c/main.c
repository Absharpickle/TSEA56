#include <avr/io.h>
#include <avr/interrupt.h>

#define SLAVE_ADDRESS 0x12
volatile uint8_t rx_buffer[8];
volatile uint8_t rx_index = 0;

void TWI_init_slave(void) {
	TWAR = (SLAVE_ADDRESS << 1);
	TWCR = (1<<TWEN) | (1<<TWEA) | (1<<TWIE);
	sei();
}

ISR(TWI_vect) {
	uint8_t status = TWSR & 0xF8;

	if (status == 0x60) { // Master vill SKRIVA
		rx_index = 0;
		} else if (status == 0x80) { // Data mottagen
		if (rx_index < 8) rx_buffer[rx_index++] = TWDR;
	}
	// Vid STOP (0xA0) eller andra tillstånd, återställ flaggan
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA) | (1<<TWIE);
}

int main(void) {
	TWI_init_slave();
	while (1);
}