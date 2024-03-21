#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include <avr/interrupt.h>

#include "usart.h"
#include "bit.h"

int setPrescaler_tc0(char option);
void my_delay_us(unsigned long x);
void my_delay_1e6us();

volatile unsigned long numOV = 0;

ISR(TIMER0_OVF_vect){
    numOV++;
}

int main(void)
{
        // Usart must be initialised
        bitSet(DDRD, PD3);
        usart_init(103); // 103-9600 bps; 8-115200
       // bitSet(TIMSK0, TOIE0);
        sei();

        while(1) {
            //for (prescaler=1; prescaler<6; prescaler++) {
            bitSet(PORTD, PD3); // ON OSC pin
            usart_tx_string(">a:") ;
            usart_transmit('1') ;
            usart_transmit('\n') ;
            //setPrescaler_tc0(prescaler);
            my_delay_1e6us() ;

            bitClear(PORTD, PD3) ;  // close OSC pin
            usart_tx_string(">a:") ;
            usart_transmit('0') ;
            usart_transmit('\n') ;
            my_delay_us(1e6) ;
            //}; 
        };
    return 0;
}
int setPrescaler_tc0(char option) {
    switch(option) {
        case 0:
            TCCR0B &= ~((1 << CS02) | (1 << CS01) | (1 << CS00));  // No clock
            return 0;
        case 1: // 62.5x10-9
            TCCR0B |= (1 << CS00);   // Set prescaler to 1: 001
            return 1;
        case 2: // 500x10-9
            TCCR0B |= (1 << CS01);   // Set prescaler to 8: 010
            return 8;
        case 3: // 4x10-6
            TCCR0B |= (1 << CS00) | (1 << CS01);  // Set prescaler to 64: 011
            return 64;
        case 4: // 16x10-6
            TCCR0B |= (1 << CS02);   // Set prescaler to 256: 100
            return 256;
        case 5: // 64x10-6
            TCCR0B |= (1 << CS00) | (1 << CS02);  // Set prescaler to 1024: 101
            return 1024;
        default:
            return -1;
    }
}

void my_delay_1e6us()
{
    unsigned long numOV_max = 62500; 

    numOV = 0;
    TCNT0 = 0;
    bitSet(TCCR0B, CS00);
    while (numOV < numOV_max - 1);
    bitClear(TCCR0B, CS00); // Clear active clock
}

void my_delay_us(unsigned long x) {
    // F = 16Mhz
    char option = 4; // P = 256

    int prescaler;
    prescaler = setPrescaler_tc0(option);
    float clock_tc0 = 16e6/prescaler ;
    float Tov = 1 / clock_tc0 * 256 * 1e6 ;

    unsigned long numOV_max = 62500; // F/P, 16Mhz/256
    unsigned char tcnt0_max = ((float)x - numOV_max * Tov) / clock_tc0 ;
    
    numOV = 0;
    TCNT0 = 0;
    numOV_max = numOV_max / prescaler;
    while (numOV < numOV_max - 1);
    setPrescaler_tc0(0); // Clear clock
    bitClear(TIFR0, TOV0) ;

    TCNT0 = 0;
    while (TCNT0 < tcnt0_max - 1) ;
    setPrescaler_tc0(0);

}