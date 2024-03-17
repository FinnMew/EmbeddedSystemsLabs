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

int main() {

    while (1) {
        
        usart_tx_string("<a:") ;
        usart_transmit('1') ;
        usart_transmit('\n') ;
        my_delay_1e6() ;

        usart_tx_string("<a:") ;
        usart_transmit('0') ;
        usart_transmit('\n') ;
        _delay_us(1e6) ;
    }
    return 0 ;
}

void my_delay_1e6us() {
    unsigned long numOV_max = 62500 ;

    numOV = 0 ;
    TCNT0 = 0 ;
    bitSet(TCCR0B, CS00) ;
    while (numOV < numOV_max - 1) ;
    bitClear(TCCR0B, CS00) ;
}


