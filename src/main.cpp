#include <avr/io.h>
#include <stdlib.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#include "usart.h"
#include "bit.h"

#define pinPWM PIND5
#define pinLED PD4
#define pinPostive PD6
#define pinNegative PD7

volatile unsigned long numOV = 0 ;
char array[10] ;
unsigned int icr1 ;
unsigned int icr2 ;
float Ton ;
float Toff ;
float dutyCycle ;
float dutyCycleReal ;

ISR(TIMER0_OVF_vect) { 
    numOV++ ;
    if(numOV == 26) {
        if(OCR0B >= 216) {
            OCR0B = 24 ;
        } else if(OCR0B < 216) {
            OCR0B += 24 ;
        }
        numOV = 0 ;
    }
} 

ISR(TIMER1_CAPT_vect) {
    if(bitRead(ACSR, ACO)) {
        bitSet(PORTD, pinLED) ;
        bitClear(ACSR, ACIS0) ;
        bitClear(TCCR1B, ICES1) ;
    } if(!bitRead(ACSR, ACO)) {
        bitClear(PORTD, pinLED) ;
        bitSet(ACSR, ACIS0) ;
        bitSet(TCCR1B, ICES1) ;
    }
    if(!bitRead(TCCR1B, ICES1)) {
        icr1 = ICR1 ;
        Toff = (icr1 - icr2) ;
        dutyCycle = Ton / (Ton + Toff) * 100 ;
    } if(bitRead(TCCR1B, ICES1)) {
        icr2 = ICR1 ;
        Ton = (icr2 - icr1) ;
    }
}

int main(void) {

    usart_init(103) ;          
    bitSet(DDRD, pinLED) ;
    bitClear(DDRD, pinNegative) ;
    bitClear(DDRD, pinPostive) ;

    bitSet(DDRD, pinPWM) ;
    bitSet(TIMSK0, TOIE0) ;    
    bitSet(TCCR0A, COM0B1) ; 
    bitSet(TCCR0A, WGM00) ;
    bitSet(TCCR0A, WGM01) ;
    bitSet(TCCR0B, WGM02) ;
    bitSet(TCCR0B, CS02) ;
    OCR0A = 240 ;
    OCR0B = 24 ;

    bitSet(TCCR1B, CS12) ;

    bitSet(ACSR, ACIC) ;   
    bitSet(ACSR, ACIS0) ;
    bitSet(ACSR, ACIS1) ;
    bitSet(TIMSK1, ICIE1) ;
    bitSet(TCCR1B, ICNC1) ; 
    bitSet(TCCR1B, ICES1) ;  

    sei() ;  

    while(1) {
        
        float ocra = OCR0A ;
        float orcb = OCR0B ;
        dutyCycleReal = (orcb / ocra) * 100 ;
        dtostrf(dutyCycleReal, 3, 0, array) ;
        usart_tx_string(">Duty Actual: ") ;
        usart_tx_string(array) ;
        usart_transmit('\n') ;
        
        dtostrf(dutyCycle, 3, 4, array) ;
        usart_tx_string(">Duty Measured: ") ;
        usart_tx_string(array) ;
        usart_transmit('\n') ;

        
    }
}

