#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include <math.h>

#include <avr/interrupt.h>

#include "usart.h"
#include "bit.h"

int setPrescaler_tc0(char option) ;
void my_delay_us(unsigned long x) ;
void set_tc0_mode(char mode) ;

void my_delay_1e6us() ;
void my_delay_2s_ctc() ;
void my_delay_us_ctc(float x, unsigned char top, char prescaler_option) ;

volatile unsigned long numOV = 0 ;

ISR(TIMER0_OVF_vect) {
    numOV++ ;
}

ISR(TIMER0_COMPA_vect) {
    numOV++ ;
}

ISR(TIMER0_COMPB_vect) {
    numOV++ ;
}

int main(void) {

    usart_init(103) ; // set baud rate to 9600 : B = clock frequecny/16 x (ubrr +1) = 16000/16(104) = 9615.4

    bitSet(DDRD, PIND6) ; // set PIND6 as output
    bitSet(DDRD, PIND5) ; // set PIND5 as output

    bitSet(TIMSK0, TOIE0) ; // timer/counter0 overflow interrupt enabled 
    bitSet(TCCR0A, WGM01) ;// CTC mode
    sei() ; // interupt enabled

    while (1) {
        bitSet(PORTD, PIND6) ;   // turn PIND6 ON
        usart_tx_string(">b:") ;
        usart_transmit('1') ;
        usart_transmit('\n') ;
        my_delay_2s_ctc() ;
        
        bitSet(PORTD, PIND6) ;
        usart_tx_string(">b:") ;
        usart_transmit('0') ;
        usart_transmit('\n') ;
        my_delay_us_ctc(1.23, 124, 5) ;

    }
    return 0 ;
}


void my_delay_1e6us() {
    
    set_tc0_mode(0) ; // set normal mode
    bitSet(TIMSK0, TOIE0) ; // timer/counter0 overflow interrupt enable

    unsigned long numOV_max = 62500 ; // for 1 second delay overflows needed = (time of delay) / (256 x T) = 1/(256/16000000) = 62500

    numOV = 0 ; // overflows count
    TCNT0 = 0 ; // timer starts at 0
    bitSet(TCCR0B, CS00) ; // set no prescalar
    while (numOV < numOV_max - 1) ; // wait until overflows reaches required amount, 62500 in this case for 1s
    bitClear(TCCR0B, CS00) ; // clear CS00, stopping timer0
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




void my_delay_us(unsigned long x) {

    int prescalerRet = setPrescaler_tc0(5) ;
    float Fscaled = 16e6/prescalerRet ;
    float Tov = ((prescalerRet/16e6) * 256) * 1e6  ;

    unsigned long numOV_max = x/Tov ;
    unsigned char tcnt0_max = ((float)x - numOV_max * Tov) / Fscaled ;
   
    numOV = 0 ;
    TCNT0 = 0 ;
    while (numOV < numOV_max - 1) ;
    setPrescaler_tc0(0) ;

    TCNT0 = 0 ;
    while (TCNT0 < tcnt0_max - 1) ;
    setPrescaler_tc0(0) ; 
}

void my_delay_2s_ctc() {
    
    set_tc0_mode(2) ; // select CTC mode
    OCR0A = 124 ; // set top (instead of default 255)
    bitSet(TIMSK0, OCIE0A) ; // timer/counter0 output compare match A interrupt

    unsigned long numOV_max = 125 * 2 ;

    numOV = 0 ; // overflow count reset to 0
    TCNT0 = 0 ; // start timer0
    setPrescaler_tc0(5) ; // set prescaler to 1024
    while (numOV < numOV_max) ;  // wait 2s
    setPrescaler_tc0(0) ; // stop timer 
}

void my_delay_us_ctc(float x, unsigned char top, char prescaler_option) {

    set_tc0_mode(2) ;
    OCR0A = top ;
    bitSet(TIMSK0, OCIE0A) ;

    unsigned long numOV_max = (top + 1) * x ;

    setPrescaler_tc0(prescaler_option) ;
    numOV = 0 ;
    TCNT0 = 0 ;
    
    while (numOV < numOV_max) ;
    setPrescaler_tc0(0) ;

}


