#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include <math.h>

#include <avr/interrupt.h>

#include "usart.h"
#include "bit.h"


int setPrescaler_tc0(char option) ;
void my_delay_us(unsigned long x) ;
void my_delay_1e6us() ;


volatile unsigned long numOV = 0 ;


ISR(TIMER0_OVF_vect) {
    numOV++ ;
}


int main(void) {


    bitSet(DDRD, PIND6) ; //set PIND6 as output
    usart_init(103) ; // set baud rate to 9600 : B = clock frequecny/16 x (ubrr +1) = 16000/16(104) = 9615.4
    bitSet(TIMSK0, TOIE0) ; // timer/counter0 overflow interrupt enabled 
    sei() ;

    while (1) {
        bitSet(PORTD, PIND6) ;   // turn PIND6 ON
        usart_tx_string(">a:") ;
        usart_transmit('1') ;
        usart_transmit('\n') ;
        my_delay_1e6us() ;

        bitClear(PORTD, PIND6) ;
        usart_tx_string(">a:") ;
        usart_transmit('0') ;
        usart_transmit('\n') ;
        my_delay_us(3e6) ;
    }
    return 0 ;
}


void my_delay_1e6us() {
   
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
        bitSet(TCCR0B, CS01) ;
        bitSet(TCCR0B, CS02) ;
        return 1024 ;
    } else {
        return 0 ;
    }
}


void my_delay_us(unsigned long x) {

    int prescalerRet = setPrescaler_tc0(4) ;
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
