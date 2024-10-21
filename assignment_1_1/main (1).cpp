#include <avr/io.h>  // Inkluderer registerdefinisjoner for AVR
#include <util/delay.h>  // For delay funksjoner

int main(void)
{
	// Sett PB0 (pin 8 på ATmega168) som utgang
	DDRB |= (1 << PB0);  // DDRB er Data Direction Register for PORTB

	while (1)  // Uendelig løkke
	{
		// Slå på PB0 (setter den til høy)
		PORTB |= (1 << PB0);  // Setter bit PB0 i PORTB til 1 (høy)
		_delay_ms(1000);  // Vent 1 sekund

		// Slå av PB0 (setter den til lav)
		PORTB &= ~(1 << PB0);  // Setter bit PB0 i PORTB til 0 (lav)
		_delay_ms(1000);  // Vent 1 sekund
	}

	return 0;  // Selv om dette aldri vil nås
}
