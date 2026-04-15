#define F_CPU 8000000UL
#include <avr/io.h>
#include <avr/interrupt.h>

#define SLAVE_ADDRESS 0x12

volatile uint8_t rx_buffer[8];
volatile uint8_t tx_buffer[8] = {0x00};
volatile uint8_t rx_index = 0;
volatile uint8_t tx_index = 0;
volatile uint8_t nytt_paket_mottaget = 0;

void TWI_init_slave(void) {
	TWAR = (SLAVE_ADDRESS << 1);
	TWCR = (1<<TWEN) | (1<<TWEA) | (1<<TWIE);
	sei();
}

ISR(TWI_vect) {
	uint8_t status = TWSR & 0xF8;

	if (status == 0x60) {
		rx_index = 0;
		} else if (status == 0x80) {
		if (rx_index < 8) rx_buffer[rx_index++] = TWDR;
		} else if (status == 0xA0) {
		nytt_paket_mottaget = 1;
	}
	else if (status == 0xA8) {
		tx_index = 0;
		TWDR = tx_buffer[tx_index++];
		} else if (status == 0xB8) {
		TWDR = (tx_index < 8) ? tx_buffer[tx_index++] : 0xFF;
	}

	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA) | (1<<TWIE);
}

int main(void) {
	TWI_init_slave();

	while (1) {
		if (nytt_paket_mottaget) {
			
			// Kolla så det är ett giltigt paket (Börjar på 0x05, slutar på 0xFF)
			if (rx_buffer[0] == 0x05 && rx_buffer[7] == 0xFF) {
				
				// Kopiera direkt till tx_buffer för verifikationen (Echo)
				for(int i = 0; i < 8; i++) {
					tx_buffer[i] = rx_buffer[i];
				}

				// --- PARSA PAKETET ---
				uint8_t state  = rx_buffer[1];       // 1 = Auto, 2 = Manuell
				char    cmd    = (char)rx_buffer[2]; // 'h' = hjul, 'a' = arm
				char    action = (char)rx_buffer[3]; // 'f', 'b', 'l', 'r', 'p', 'd'
				
				// Återställ linjefelet från offset binary. 128 blir 0.
				// 129 blir +1, 127 blir -1 osv.
				int8_t linje_fel = (int8_t)(rx_buffer[4] - 128);
				
				// Pussla ihop 16-bitars gyrovärde
				int16_t gyro_data = (int16_t)(rx_buffer[5] | (rx_buffer[6] << 8));


				// (Göm kompilatorvarning)
				(void)state;
				(void)linje_fel;
				(void)gyro_data;

				// --- MOTORSTYRNING LOGIK ---
				if (cmd == 'h') {
					// Vi ska styra hjulen
					switch (action) {
						case 'f':
						// kor_framat_med_linjefoljning(linje_fel);
						break;
						case 'b':
						// backa();
						break;
						case 'l':
						// svang_vanster_med_gyro(gyro_data);
						break;
						case 'r':
						// svang_hoger_med_gyro(gyro_data);
						break;
						case 's':
						// stanna();
						break;
					}
				}
				else if (cmd == 'a') {
					// Vi ska styra armen/klon
					switch (action) {
						case 'p':
						// plocka_upp();
						break;
						case 'd':
						// droppa_vara();
						break;
					}
				}
			}
			
			nytt_paket_mottaget = 0;
		}
	}
}