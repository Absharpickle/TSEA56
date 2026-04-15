#define F_CPU 8000000UL 
#include <avr/io.h>
#include <avr/interrupt.h>

#define SLAVE_ADDRESS 0x12

volatile uint8_t rx_buffer[8];
volatile uint8_t tx_buffer[8] = {0x00}; // Ny buffer för att skicka tillbaka (Echo)
volatile uint8_t rx_index = 0;
volatile uint8_t tx_index = 0;          // Index för sändning
volatile uint8_t nytt_paket_mottaget = 0;

// =================================================================
// MOTOR- OCH STYRFUNKTIONER (Fyll i er PWM-kod här!)
// =================================================================
void kor_framat() { /* PWM kod */ }
void svang_hoger() { /* PWM kod */ }
void svang_vanster() { /* PWM kod */ }
void rotera_180_eller_backa() { /* PWM kod */ }
void stanna_motorer() { /* PWM kod */ }
void plocka_upp_vara() { /* Servo kod */ }
void avlamna_vara() { /* Servo kod */ }
void pd_reglering_linje(int8_t vinkel_fel) { (void)vinkel_fel; }
void reglera_rotationshastighet(int8_t gyro_varde) { (void)gyro_varde; }

// =================================================================
// I2C-KOMMUNIKATION (SLAVE RECEIVER & TRANSMITTER)
// =================================================================
void TWI_init_slave(void) {
    TWAR = (SLAVE_ADDRESS << 1); 
    TWCR = (1<<TWEN) | (1<<TWEA) | (1<<TWIE);
    sei(); 
}

ISR(TWI_vect) {
    uint8_t status = TWSR & 0xF8;

    // --- SLAVE RECEIVER (Pi skickar till oss) ---
    if (status == 0x60) { 
        rx_index = 0;
    } else if (status == 0x80) { 
        if (rx_index < 8) rx_buffer[rx_index++] = TWDR;
    } else if (status == 0xA0) { 
        nytt_paket_mottaget = 1; 
    }
    
    // --- SLAVE TRANSMITTER (Pi läser tillbaka för Echo) ---
    else if (status == 0xA8) { // Master vill läsa
        tx_index = 0;
        TWDR = tx_buffer[tx_index++];
    } else if (status == 0xB8) { // Master vill ha nästa byte
        TWDR = (tx_index < 8) ? tx_buffer[tx_index++] : 0x00;
    }

    TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA) | (1<<TWIE);
}

// =================================================================
// HUVUDPROGRAM
// =================================================================
int main(void) {
    TWI_init_slave();
    
    uint8_t fall;
    int8_t data_1, data_2, data_3;

    while (1) {
        if (nytt_paket_mottaget) {
            
            if (rx_buffer[0] == 0x05 && rx_buffer[7] == 0xFF) {
                
                // 1. KOPIERA TILL TX_BUFFER FÖR VERIFIKATION (ECHO)
                // Pi:n kommer snart att begära att få läsa detta
                for(int i = 0; i < 8; i++) {
                    tx_buffer[i] = rx_buffer[i];
                }

                fall   = rx_buffer[1];
                data_1 = (int8_t)rx_buffer[2]; 
                data_2 = (int8_t)rx_buffer[3]; 
                data_3 = (int8_t)rx_buffer[4]; 

                // 2. TOLKA PAKETET (Samma logik som förut)
                if (fall == 1) {
                    char cmd = (char)data_1;
                    switch (cmd) {
                        case 'f': kor_framat(); break;
                        case 'r': svang_hoger(); break;
                        case 'l': svang_vanster(); break;
                        case 'b': rotera_180_eller_backa(); break;
                        case 'v': plocka_upp_vara(); break;
                        case 'a': avlamna_vara(); break;
                        case 's': stanna_motorer(); break;
                        default:  break; 
                    }
                } else if (fall == 2) {
                    pd_reglering_linje(data_1);
                } else if (fall == 3) {
                    reglera_rotationshastighet(data_1);
                }
            }
            nytt_paket_mottaget = 0; 
        }
    }
}