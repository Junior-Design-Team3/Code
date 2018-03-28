#define F_CPU 1000000UL
#include <avr/io.h>
#include <avr/delay.h>
#include <stdint.h>

#define pwm_coma1 7
#define pwm_coma0 6
#define pwm_comb1 5
#define pwm_comb0 4

#define pwm_wgm0 0
#define pwm_wgm1 1
#define pwm_wgm2 3

#define pwm_cs0 0
#define pwm_cs1 1
#define pwm_cs2 2



int main(void)
{
    DDRD |= (1 << 6) ; // PD6 is now an output
	DDRC |= (1 << 5) ; // PC5 is output

    OCR0A = 128; // set PWM for 50% duty cycle


    TCCR0A |= (1 << COM0A1); // set none-inverting mode

    TCCR0A |= (1 << WGM01) | (1 << WGM00); // set fast PWM Mode

    TCCR0B |= (1 << CS01); // set prescaler to 8 and starts PWM


    while (1)
    {
        // we have a working Fast PWM
		PORTC = 0;
		_delay_ms (2000);
		PORTC = (1 << 5);
		_delay_ms (2000);
    }
}
