#define F_CPU 8000000UL // Viktigt för _delay_ms om du kör 8 MHz
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define SLAVE_ADDRESS 0x11

// Initialt standardpaket
volatile uint8_t tx_buffer[8] = {0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0xFF};
volatile uint8_t tx_index = 0;

void TWI_init_slave(void) {
    TWAR = (SLAVE_ADDRESS << 1); 
    TWCR = (1<<TWEN) | (1<<TWEA) | (1<<TWIE); 
    sei(); 
}

ISR(TWI_vect) {
    uint8_t status = TWSR & 0xF8;

    if (status == 0xA8) { // Master vill LÄSA
        tx_index = 0;
        TWDR = tx_buffer[tx_index++];
    } else if (status == 0xB8) { // Master vill ha NÄSTA byte
        TWDR = (tx_index < 8) ? tx_buffer[tx_index++] : 0x00;
    } 
    TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA) | (1<<TWIE);
}

int main(void) {
    TWI_init_slave();
    
    int timer = 0;
    int8_t svaj = 0; // Används för att simulera att roboten rör sig på linjen
    int8_t svaj_riktning = 1;

    while (1) {
        timer++;

        // --- 1. SIMULERA LINJEFÖLJNING (Kontinuerligt) ---
        // Får värdet att pendla mellan -3 och +3
        svaj += svaj_riktning;
        if (svaj > 3 || svaj < -3) svaj_riktning = -svaj_riktning; 
        
        tx_buffer[2] = svaj;       // Avvikelse fram
        tx_buffer[3] = -svaj;      // Avvikelse bak (skapar en vinkel)

        // Återställ till "normal" körning (2 lampor ser svart, inget hinder)
        tx_buffer[1] = 0x00; 
        tx_buffer[6] = 2;    

        // --- 2. SIMULERA KORSNING (Kort puls varje ~5 sek) ---
        if (timer >= 50 && timer <= 55) {
            tx_buffer[6] = 7; // 7 lampor ser svart -> Korsning!
        }
        
        // --- 3. SIMULERA HINDER (Kort puls varje ~10 sek) ---
        if (timer >= 100 && timer <= 105) {
            tx_buffer[1] = (1 << 2); // Sätter bit 2 hög -> Hinder!
        }

        // --- 4. BÖRJA OM ---
        if (timer > 120) {
            timer = 0;
        }

        _delay_ms(100); // AVR väntar 0.1 sekunder, Pi:n läser när den vill
    } 
}