#include <avr/io.h>
#include <stdlib.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#include "usart.h"
#include "bit.h"

#define pinTrig PIND4
#define pinEcho PIND3
#define pinbutton PIND2

int setPrescaler_tc0(char option) ;
void set_tc0_mode(char mode) ;

float echoTime ;
float distance ;
char valueDistance[9] ;
int echoHighLow ;
int buttonCheck ;
int onOrOff = 1 ;

ISR(INT0_vect) {
    buttonCheck = bitRead(PIND, 2) ;
    if(buttonCheck == 1) {
        _delay_ms(60) ;
        buttonCheck = bitRead(PIND, 2) ;
        if(buttonCheck == 1) {
            onOrOff++ ;
            if(onOrOff == 3) {
                onOrOff = 1 ;
            }
        } 
    }
}

ISR(INT1_vect) {
    if(echoHighLow == 0) {
        TCNT0 = 0 ;
        setPrescaler_tc0(4) ;   
    } if(echoHighLow == 1) {
        setPrescaler_tc0(0) ;
        echoTime = TCNT0 ;
        distance = (echoTime * 1/(16000000/256)) * 343 / 2 * 100 ;
       
        dtostrf(distance, 7, 3, valueDistance) ;
        valueDistance[7] = '\0' ;

        usart_tx_string(">a:") ;
        usart_tx_string(valueDistance) ;
        usart_transmit('\n') ;
    }     
}

int main (void) {

    usart_init(103) ;

    set_tc0_mode(0) ;
    
    bitSet(DDRD, pinTrig) ;
    bitClear(DDRD, pinEcho) ;
    bitClear(PORTD, pinbutton) ;
    bitSet(PORTD, pinbutton) ;

    bitSet(EIMSK, INT1) ;
    bitSet(EICRA, ISC10) ;

    bitSet(EIMSK, INT0) ;
    bitSet(EICRA, ISC00) ;
    bitSet(EICRA, ISC01) ;

    sei() ;

    while(1) {

        if(onOrOff == 1) {
        bitSet(DDRD, pinTrig) ;
        bitClear(PORTD, pinTrig) ;               
        bitClear(PORTD, pinEcho) ; 

        bitSet(PORTD, pinTrig) ;
        _delay_us(10) ;
        bitClear(PORTD, pinTrig) ;

        echoHighLow = 0 ;
        while(!bitRead(PIND, pinEcho)) ;
        echoHighLow = 1 ;
        while(bitRead(PIND, pinEcho)) ;

        } if(onOrOff == 2) {
            bitClear(DDRD, pinTrig) ;
        }
        _delay_ms(100) ;
    }
}

int setPrescaler_tc0(char option) {
    
    if(option == 0) {
        bitClear(TCCR0B, CS00) ;
        bitClear(TCCR0B, CS01) ;
        bitClear(TCCR0B, CS02) ;
        return 0 ;
    } if(option == 1) {          // T = 1/(16000000/1) = 6.25e-8
        bitSet(TCCR0B, CS00) ;
        return 1 ;
    } if(option == 2) {          // T = 1/(16000000/8) = 5e-7
        bitSet(TCCR0B, CS01) ;
        return 8 ;
    } if(option == 3) {          // T = 1/(16000000/64) = 4e-6
        bitSet(TCCR0B, CS00) ;
        bitSet(TCCR0B, CS01) ;
        return 64 ;
    } if(option == 4) {          // T = 1/(16000000/256) = 16e-6
        bitSet(TCCR0B, CS02) ;
        return 256 ;
    } if(option == 5) {          // T = 1/(16000000/1024) = 64e-6
        bitSet(TCCR0B, CS00) ;
        bitSet(TCCR0B, CS02) ;
        return 1024 ;
    } else {
        return 0 ;
    }
}

void set_tc0_mode(char mode) {
    
    if(mode == 0) {                // normal, 0xFF, immediate, Max
        bitClear(TCCR0A, WGM00) ;
        bitClear(TCCR0A, WGM01) ;
        bitClear(TCCR0B, WGM02) ;
    } if(mode == 1) {             // PWM, phase correct, 0xFF, TOP, BOTTOM
        bitSet(TCCR0A, WGM00) ;
    } if(mode == 2) {             // CTC, OCRA, immediate, max
        bitSet(TCCR0A, WGM01) ;
    } if(mode == 3) {             // Fast PWM, 0xFF, BOTTOM, MAX
        bitSet(TCCR0A, WGM00) ;
        bitSet(TCCR0A, WGM01) ;
    } if(mode == 4) {             // Reserved, -, -, -
        bitSet(TCCR0B, WGM02) ;
    } if(mode == 5) {             // PWM, phase correct, OCRA, TOP, BOTTOM
        bitSet(TCCR0B, WGM02) ;
        bitSet(TCCR0A, WGM00) ;
    } if(mode == 6) {             // Reserved, -, -, -
        bitSet(TCCR0B, WGM02) ;
        bitSet(TCCR0A, WGM01) ;
    } if(mode == 7) {             // Fast PWM, OCRA, BOTTOM, TOP
        bitSet(TCCR0A, WGM00) ;
        bitSet(TCCR0A, WGM01) ;
        bitSet(TCCR0B, WGM02) ;
    }
}

