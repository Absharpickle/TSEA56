#include <avr/io.h>
#include <avr/interrupt.h> // Krävs för ISR

#define SLAVE_ADDRESS 0x12

volatile uint8_t rx_buffer[8];
volatile uint8_t rx_index = 0;
volatile uint8_t data_ready = 0;

void TWI_init_slave(void) {
	TWAR = (SLAVE_ADDRESS << 1);
	// TWIE = TWI Interrupt Enable. Detta gör att processorn sköter detta i bakgrunden.
	TWCR = (1<<TWEN) | (1<<TWEA) | (1<<TWIE);
	sei(); // Aktivera globala interrupts
}

// Denna funktion (Interrupt Service Routine) triggas AUTOMATISKT
// på mikrosekunden när Raspberry Pi pratar med AVR:en.
ISR(TWI_vect) {
	uint8_t status = TWSR & 0xF8;

	if (status == 0x60) { // Pi vill skriva
		rx_index = 0;
		data_ready = 0;
		TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA) | (1<<TWIE);
	}
	else if (status == 0x80) { // Data mottagen
		if (rx_index < 8) {
			rx_buffer[rx_index++] = TWDR;
			if (rx_index == 8) data_ready = 1;
		}
		TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA) | (1<<TWIE);
	}
	else if (status == 0xA0) { // STOP-signal
		TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA) | (1<<TWIE);
	}
	else { // Felhantering
		TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA) | (1<<TWIE);
	}
}

int main(void) {
	TWI_init_slave();

	while (1) {
		// Huvudloopen är nu helt ledig!
		// Du behöver inte anropa någon kommunikationsfunktion här längre.
		
		if (data_ready) {
			// Här kan du i lugn och ro hantera din rx_buffer.
			// Datan fylls på automatiskt av Interruptet.
			
			data_ready = 0; // Återställ flaggan när du läst klart
		}
	}
	return 0;
}