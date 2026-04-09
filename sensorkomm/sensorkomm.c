#define F_CPU 8000000UL // Körs i 8 MHz (Glöm inte bocka ur CKDIV8!)
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define SLAVE_ADDRESS 0x11

volatile uint8_t tx_buffer[8] = {0x05, 0, 0, 0, 0, 0, 0, 0xFF};
volatile uint8_t tx_index = 0;

void TWI_init_slave(void) {
    TWAR = (SLAVE_ADDRESS << 1); 
    TWCR = (1<<TWEN) | (1<<TWEA) | (1<<TWIE); 
    sei(); 
}

ISR(TWI_vect) {
    uint8_t status = TWSR & 0xF8;

    if (status == 0xA8) { // Master (Pi) vill LÄSA
        tx_index = 0;
        TWDR = tx_buffer[tx_index++];
    } else if (status == 0xB8) { // Master vill ha NÄSTA byte
        TWDR = (tx_index < 8) ? tx_buffer[tx_index++] : 0x00;
    } 
    // Nollställ I2C-hårdvaran för att vänta på nästa klockpuls
    TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA) | (1<<TWIE);
}

int main(void) {
    TWI_init_slave();
    
    int timer = 0;
    int8_t svaj = 0; 
    int8_t svaj_riktning = 1;

    while (1) {
        timer++;

        // --- 1. SIMULERA LINJEFÖLJNING ---
        svaj += svaj_riktning;
        if (svaj > 4 || svaj < -4) svaj_riktning = -svaj_riktning; 
        
        tx_buffer[0] = 0x05;       // Startbyte
        tx_buffer[1] = 0x00;       // Status (Inga flaggor = 0)
        tx_buffer[2] = svaj;       // Byte 2: Avvikelse fram
        tx_buffer[3] = -svaj;      // Byte 3: Avvikelse bak
        tx_buffer[4] = 12;         // Byte 4: Låtsas att gyrot säger 12 rad/s
        tx_buffer[5] = 0x00;       // Tom
        tx_buffer[6] = 2;          // Byte 6: Antal aktiva lampor (2 = normalt)
        tx_buffer[7] = 0xFF;       // Stoppbyte

        // --- 2. SIMULERA KORSNING ---
        // Efter ca 5 sekunder, tänd 7 lampor för att Pi ska reagera
        if (timer >= 50 && timer <= 55) {
            tx_buffer[6] = 7; 
        }
        
        // --- 3. SIMULERA HINDER ---
        // Efter ca 10 sekunder, sätt bit 2 i statusflaggorna
        if (timer >= 100 && timer <= 105) {
            tx_buffer[1] = (1 << 2); 
        }

        if (timer > 120) timer = 0; // Börja om

        _delay_ms(100); 
    } 
}