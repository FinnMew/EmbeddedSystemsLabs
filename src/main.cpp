#include <avr/io.h>
#include <stdlib.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#include "usart.h"
#include "bit.h"

#define pinPhotocell PC0 

int readADC(unsigned char pin) ;
void ledControl(float value_ADC) ;

int setPrescaler_tc0(char option) ;
void set_tc0_mode(char mode) ;

float value_ADC ;

ISR (ADC_vect) {
    value_ADC = readADC(pinPhotocell) ;
    bitSet(ADCSRA, ADSC) ;    // Start next conversion
}

int main () {

    char PhotoResValArray[11] ;
    usart_init(103) ;
    bitSet(DDRD, PIND6) ;

    bitSet(ADCSRA, ADPS2) ;   // prescalar
    bitSet(ADCSRA, ADPS1) ;   // prescalar
    bitSet(ADCSRA, ADPS0) ;   // prescalar
    bitClear(ADMUX, ADLAR) ;  // setting 0 for 10 bit
    bitSet(ADMUX, REFS0) ;    // Aref settings
    bitSet(ADCSRA, ADIE) ;    // Enable intterupt
    sei() ;                    
    bitSet(ADCSRA, ADEN) ;    // Enables ADC
    bitSet(ADCSRA, ADSC) ;    // Start conversion

    set_tc0_mode(3) ;
    bitClear(TCCR0A, COM0A0) ;
    bitClear(TCCR0A, COM0A1) ;
    bitClear(TCCR0A, COM0B0) ;
    bitClear(TCCR0A, COM0B1) ;

    bitSet(TCCR0A, COM0A1) ;
    bitSet(TCCR0A, COM0B1) ;

    setPrescaler_tc0(3) ;

    while (1) {

        dtostrf(value_ADC, 4, 3, PhotoResValArray) ;
        PhotoResValArray[11] = '\n' ;

        usart_tx_string(">a") ;
        usart_tx_string(PhotoResValArray) ;
        usart_transmit('\n') ;

       ledControl(value_ADC) ;
    }
} 

int readADC(unsigned char pin) {
    int volt = ADCL ;
    volt |= (ADCH << 8) ;
    return volt ;
}

void ledControl(float value_ADC) {

    float voltage = (value_ADC * 5) / 1023 ;
    float resistance = (1000 * (5 - voltage)) / voltage ;

    OCR0A = resistance / 255 ;
}

int setPrescaler_tc0(char option) {
   if(option == 0) {
       bitClear(TCCR0B, CS00) ;
       bitClear(TCCR0B, CS01) ;
       bitClear(TCCR0B, CS02) ;
       return 0 ;
   } if (option == 1) {          // T = 1/(16000000/1) = 6.25e-8
       bitSet(TCCR0B, CS00) ;
       return 1 ;
   } if (option == 2) {          // T = 1/(16000000/8) = 5e-7
       bitSet(TCCR0B, CS01) ;
       return 8 ;
   } if (option == 3) {          // T = 1/(16000000/64) = 4e-6
       bitSet(TCCR0B, CS00) ;
       bitSet(TCCR0B, CS01) ;
       return 64 ;
   } if (option == 4) {          // T = 1/(16000000/256) = 16e-6
       bitSet(TCCR0B, CS02) ;
       return 256 ;
   } if (option == 5) {          // T = 1/(16000000/1024) = 64e-6
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
       } if (mode == 1) {             // PWM, phase correct, 0xFF, TOP, BOTTOM
           bitSet(TCCR0A, WGM00) ;
       } if (mode == 2) {             // CTC, OCRA, immediate, max
           bitSet(TCCR0A, WGM01) ;
       } if (mode == 3) {             // Fast PWM, 0xFF, BOTTOM, MAX
           bitSet(TCCR0A, WGM00) ;
           bitSet(TCCR0A, WGM01) ;
       } if (mode == 4) {             // Reserved, -, -, -
           bitSet(TCCR0B, WGM02) ;
       } if (mode == 5) {             // PWM, phase correct, OCRA, TOP, BOTTOM
           bitSet(TCCR0B, WGM02) ;
           bitSet(TCCR0A, WGM00) ;
       } if (mode == 6) {             // Reserved, -, -, -
           bitSet(TCCR0B, WGM02) ;
           bitSet(TCCR0A, WGM01) ;
       } if (mode == 7) {             // Fast PWM, OCRA, BOTTOM, TOP
           bitSet(TCCR0A, WGM00) ;
           bitSet(TCCR0A, WGM01) ;
           bitSet(TCCR0B, WGM02) ;
       }
   }

