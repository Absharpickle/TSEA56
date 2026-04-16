#define F_CPU 8000000UL // Simulatorn körs i 8 MHz
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdint.h>

#define SLAVE_ADDRESS 0x11
#define PKT_LEN 8

volatile uint8_t tx_buf[PKT_LEN] = {0};
volatile uint8_t tx_idx = 0;

void TWI_init_slave(void) {
	TWAR = (SLAVE_ADDRESS << 1);
	TWCR = (1<<TWEN) | (1<<TWEA) | (1<<TWIE);
	sei();
}

ISR(TWI_vect) {
	uint8_t status = TWSR & 0xF8;
	if (status == 0xA8) {
		tx_idx = 0;
		TWDR = tx_buf[tx_idx++];
		} else if (status == 0xB8) {
		TWDR = (tx_idx < PKT_LEN) ? tx_buf[tx_idx++] : 0xFF;
	}
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA) | (1<<TWIE);
}

static void pkt_write_i16(uint8_t idx, int16_t v) {
	tx_buf[idx] = (uint8_t)(v & 0xFF);
	tx_buf[idx + 1] = (uint8_t)((v >> 8) & 0xFF);
}

int main(void) {
	TWI_init_slave();
	
	uint16_t timer = 0;
	int16_t dev0 = 0, dev1 = 0;
	int8_t svaj_riktning = 1;

	while (1) {
		timer++;

		// 1. Simulerar svaj på linjen
		dev0 += (svaj_riktning * 2);
		dev1 = dev0 - 5;
		if (dev0 > 25 || dev0 < -25) svaj_riktning = -svaj_riktning;

		// 2. Bygg grundpaketet (Enligt sensorbois.c format)
		uint8_t stat = 0x03; // Båda linjerna hittade
		uint8_t dist = 50;   // Inget hinder
		int16_t omega = 0;   // Kör rakt

		// 3. Simulerar Korsning EXAKT var 5:e sekund (Tick 50, 100...)
		if ((timer % 50) < 5) {
			stat |= (1 << 3); // Sätt Korsnings-biten
		}

		if (timer > 1000) timer = 0;

		cli();
		tx_buf[0] = stat;
		pkt_write_i16(1, dev0);
		pkt_write_i16(3, dev1);
		tx_buf[5] = dist;
		pkt_write_i16(6, omega);
		sei();

		_delay_ms(100); // 100ms * 50 ticks = 5 sekunder
	}
}