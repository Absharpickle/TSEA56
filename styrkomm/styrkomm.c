#define F_CPU 8000000UL // Körs i 8 MHz
#include <avr/io.h>
#include <avr/interrupt.h>

#define SLAVE_ADDRESS 0x12

volatile uint8_t rx_buffer[8];
volatile uint8_t rx_index = 0;
volatile uint8_t nytt_paket_mottaget = 0; // Flagga för main-loopen

void TWI_init_slave(void) {
    TWAR = (SLAVE_ADDRESS << 1); 
    TWCR = (1<<TWEN) | (1<<TWEA) | (1<<TWIE);
    sei(); 
}

ISR(TWI_vect) {
    uint8_t status = TWSR & 0xF8;

    if (status == 0x60) { // Master (Pi) vill SKRIVA
        rx_index = 0;
    } else if (status == 0x80) { // Master har skickat en byte
        if (rx_index < 8) {
            rx_buffer[rx_index++] = TWDR;
        }
    } else if (status == 0xA0) { // STOPP-signal från Master
        nytt_paket_mottaget = 1; // Nu har vi ett helt paket att analysera!
    }
    
    // Nollställ I2C-hårdvaran
    TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA) | (1<<TWIE);
}

int main(void) {
    TWI_init_slave();
    
    // Kopior av datan så vi inte läser dem medan ISR försöker skriva över dem
    uint8_t fall;
    int8_t  data_1, data_2, data_3;

    while (1) {
        if (nytt_paket_mottaget) {
            
            // Kolla så det faktiskt är ett giltigt paket med Start- och Stoppbyte
            if (rx_buffer[0] == 0x05 && rx_buffer[7] == 0xFF) {
                
                fall   = rx_buffer[1];
                data_1 = (int8_t)rx_buffer[2]; 
                data_2 = (int8_t)rx_buffer[3]; 
                data_3 = (int8_t)rx_buffer[4]; 

                // --- TOLKA PAKETET ---
                
                if (fall == 1) {
                    // FALL 1: NYTT KOMMANDO (Byte 2 är ett char)
                    char cmd = (char)data_1;
                    
                    if (cmd == 'b') {
                        // Kalla på funktion: backa_robot();
                    } else if (cmd == 'v') {
                        // Kalla på funktion: plocka_vara_med_klo();
                    } else if (cmd == 'a') {
                        // Kalla på funktion: slapp_vara();
                    } else if (cmd == 'r' || cmd == 'l' || cmd == 'h') {
                        // Kalla på sväng-funktioner
                    } else if (cmd == 'f') {
                        // Kalla på funktion: kor_framat();
                    } else if (cmd == 's') {
                        // Kalla på funktion: stanna_motorer();
                    }
                } 
                
                else if (fall == 2) {
                    // FALL 2: LINJEFÖLJNING
                    int8_t vinkel_fel = data_1;
                    int8_t fram_fel   = data_2;
                    int8_t bak_fel    = data_3;
                    
                    // Exempel: 
                    // pd_reglering(vinkel_fel);
                } 
                
                else if (fall == 3) {
                    // FALL 3: ROTATION PÅGÅR
                    int8_t gyro_rotationshastighet = data_1;
                    
                    // Används t.ex. för att bromsa motorerna om den snurrar för fort:
                    // reglera_svang_hastighet(gyro_rotationshastighet);
                }
            }
            
            // Markera att vi har läst datan
            nytt_paket_mottaget = 0; 
        }
    }
}