#include <avr/io.h>
#include <stdlib.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "usart.h"
#include "bit.h"

#define pinPWM PB2
#define pinDirection1 PD3
#define pinDirection2 PD2

int main (void) {
    usart_init(103) ;
    bitSet(DDRB, pinPWM) ;
    bitSet(DDRD, pinDirection1) ;
    bitSet(DDRD, pinDirection2) ;
    
    bitSet(TCCR1B, WGM13) ;     // Phase correct mode 11
    bitSet(TCCR1A, WGM11) ;     // Phase correct mode 11
    bitSet(TCCR1A, WGM10) ;     // Phase correct mode 11
    bitSet(TCCR1A, COM1B1) ;    // Non inverting mode on match compare
    bitSet(TCCR1B, CS11) ;      // Prescaler 8

    while (1) {  
        OCR1A = 10000 ;                     // Sets top value            
        bitSet(PORTD, pinDirection2) ;      // Configs direction of motor
        bitClear(PORTD, pinDirection1) ;    // Configs direction of motor
        OCR1B = 3500 ;                      // Set compare 3000 - duty cycle of 30%
        _delay_ms(2000) ;                   // Keep 35% duty cycle for 2 seconds
        OCR1B = 5000 ;                      // Set compare 5000 - duty cycle of 50%
        _delay_ms(2000) ;                   // Keep 50% duty cycle for 2 seconds
        OCR1B = 10000 ;                     // Set compare 10000 - duty cycle of 100%
        _delay_ms(2000) ;                   // Keep 100% duty cycle for 2 seconds
        bitSet(PORTD, pinDirection1) ;      // Configs direction of motor - changed from before
        bitClear(PORTD, pinDirection2) ;    // Configs direction of motor - changed from before
        OCR1B = 3500 ;                      // Set compare 3000 - duty cycle of 30%
        _delay_ms(2000) ;                   // Keep 35% duty cycle for 2 seconds
        OCR1B = 5000 ;                      // Set compare 5000 - duty cycle of 50%
        _delay_ms(2000) ;                   // Keep 50% duty cycle for 2 seconds
        OCR1B = 10000 ;                     // Set compare 10000 - duty cycle of 100%
        _delay_ms(2000) ;                   // Keep 100% duty cycle for 2 seconds
    }
}


