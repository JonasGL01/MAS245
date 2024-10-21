#include <avr/io.h>
#include <util/delay.h>

// Funksjon for å sette opp PWM
void setupPWM() {
	// Sett PB1 (OC1A) som utgang
	DDRB |= (1 << PB1);

	// Sett opp Timer1 i Fast PWM-modus med 8-bit oppløsning
	TCCR1A |= (1 << WGM10) | (1 << COM1A1);  // Fast PWM, 8-bit, ikke-invertert PWM på OC1A (PB1)
	TCCR1B |= (1 << WGM12) | (1 << CS11);    // Fast PWM-modus, prescaler på 8

	// Initialiser duty cycle til 0 (LED av)
	OCR1A = 0;
}

// Funksjon for å oppdatere PWM duty cycle
void setPWMDutyCycle(uint8_t duty) {
	OCR1A = duty;  // Sett duty cycle, verdi fra 0 (av) til 255 (maks lysstyrke)
}

int main(void) {
	// Sett opp PWM
	setupPWM();

	// Variabler for pulserende lys
	uint8_t brightness = 0;
	int8_t fadeAmount = 1;  // Bestemmer retning og steg for lysstyrken

	while (1) {
		// Oppdater lysstyrken til LED-en
		setPWMDutyCycle(brightness);

		// Endre lysstyrken for neste iterasjon
		brightness += fadeAmount;

		// Snu retningen når vi når maksimal/minimal lysstyrke
		if (brightness == 0 || brightness == 255) {
			fadeAmount = -fadeAmount;  // Bytt retning på fading
		}

		// Delay for å kontrollere hastigheten på pulseringen
		_delay_ms(10);  // Juster denne for å endre pulseringshastigheten
	}

	return 0;
}
