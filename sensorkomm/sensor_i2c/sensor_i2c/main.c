#include <avr/io.h>

#define SLAVE_ADDRESS 0x11

// Datan vi simulerar från tabellen (8 bytes)
volatile uint8_t tx_buffer[8] = {0x05, 0xF1, 0xFF, 0x14, 0x00, 35, 0x01, 0xFF};
volatile uint8_t tx_index = 0;

void TWI_init_slave(void) {
	TWAR = (SLAVE_ADDRESS << 1);
	TWCR = (1<<TWEN) | (1<<TWEA); // Aktivera TWI och svara med ACK
}

void TWI_handle_communication(void) {
	if (!(TWCR & (1<<TWINT))) return;

	uint8_t status = TWSR & 0xF8;

	if (status == 0xA8) {
		// 0xA8: Master vill LÄSA från oss.
		tx_index = 0;
		TWDR = tx_buffer[tx_index++]; // Ladda första byten
		TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);
	}
	else if (status == 0xB8) {
		// 0xB8: Data skickad, Master svarade ACK och vill ha mer data
		if (tx_index < 8) {
			TWDR = tx_buffer[tx_index++];
			} else {
			TWDR = 0x00; // Skicka nollor om Master begär för mycket
		}
		TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);
	}
	else {
		// NACK (0xC0) eller annat tillstånd. Återställ.
		TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);
	}
}

int main(void) {
	TWI_init_slave();
	while (1) {
		TWI_handle_communication();
	}
	return 0;
}