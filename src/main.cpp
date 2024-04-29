#include <avr/io.h>
#include <stdlib.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "usart.h"
#include "bit.h"

#define pinPWM PB2
#define pinDirection1 PD4
#define pinDirection2 PD3

int setPrescaler_tc1(int option) ;
void set_tc1_mode(char mode) ;
void calcuateTopAndComp(int Hz, float dutycycle, int PrescalerMode) ;

int main (void) {
    
    usart_init(103) ;
    bitSet(DDRB, pinPWM) ;
    bitSet(DDRD, pinDirection1) ;
    bitSet(DDRD, pinDirection2) ;
    bitSet(TCCR1A, COM1B1) ;
    set_tc1_mode(11) ;
    setPrescaler_tc1(2) ;

    while (1) {  
        bitSet(PORTD, pinDirection1) ;      
        bitClear(PORTD, pinDirection2) ;
        calcuateTopAndComp(100, 0.3, 2) ;
        _delay_ms(3000) ;
        bitSet(PORTD, pinDirection2) ;      
        bitClear(PORTD, pinDirection1) ;
        calcuateTopAndComp(100, 0.45, 2) ;
        _delay_ms(3000) ;
        bitSet(PORTD, pinDirection1) ;      
        bitClear(PORTD, pinDirection2) ;
        calcuateTopAndComp(100, 0.6, 2) ;
        _delay_ms(3000) ;
        bitSet(PORTD, pinDirection2) ;      
        bitClear(PORTD, pinDirection1) ;
        calcuateTopAndComp(100, 1, 2) ;
        _delay_ms(3000) ;
    }
}

void calcuateTopAndComp(int Hz, float dutycycle, int PrescalerMode) {
    int P = setPrescaler_tc1(PrescalerMode) ;
    int Max = 16000000 / 2 * Hz * P ;
    OCR1A = Max ;
    int Comp = Max * dutycycle ;
    OCR1B = Comp ;
}

int setPrescaler_tc1(int option) {

    if(option == 0) {
        bitClear(TCCR1B, CS10) ;
        bitClear(TCCR1B, CS11) ;
        bitClear(TCCR1B, CS12) ;
        return 0 ;
    } if (option == 1) {          // T = 1/(16000000/1) = 6.25e-8
        bitSet(TCCR1B, CS10) ;
        return 1 ;
    } if (option == 2) {          // T = 1/(16000000/8) = 5e-7
        bitSet(TCCR1B, CS11) ;
        return 8 ;
    } if (option == 3) {          // T = 1/(16000000/64) = 4e-6
        bitSet(TCCR1B, CS10) ;
        bitSet(TCCR1B, CS11) ;
        return 64 ;
    } if (option == 4) {          // T = 1/(16000000/256) = 16e-6
        bitSet(TCCR1B, CS12) ;
        return 256 ;
    } if (option == 5) {          // T = 1/(16000000/1024) = 64e-6
        bitSet(TCCR1B, CS10) ;
        bitSet(TCCR1B, CS12) ;
        return 1024 ;
    } else {
        return 0 ;
    }
}
  
void set_tc1_mode(char mode) {

    if(mode == 0) {                
        bitClear(TCCR1A, WGM10) ;
        bitClear(TCCR1A, WGM11) ;
        bitClear(TCCR1B, WGM12) ;
        bitClear(TCCR1B, WGM13) ;
    } if (mode == 1) {            
        bitSet(TCCR1A, WGM10) ;
    } if (mode == 2) {             
        bitSet(TCCR1A, WGM11) ;
    } if (mode == 3) {            
        bitSet(TCCR1A, WGM10) ;
        bitSet(TCCR1A, WGM11) ;
    } if (mode == 4) {            
        bitSet(TCCR1B, WGM12) ;
    } if (mode == 5) {            
        bitSet(TCCR1A, WGM10) ;
        bitSet(TCCR1B, WGM12) ;
    } if (mode == 6) {            
        bitSet(TCCR1A, WGM11) ;
        bitSet(TCCR1B, WGM12) ;
    } if (mode == 7) {            
        bitSet(TCCR1A, WGM10) ;
        bitSet(TCCR1A, WGM11) ;
        bitSet(TCCR1B, WGM12) ;
    } if(mode == 8) {                
        bitSet(TCCR1B, WGM13) ;
    } if (mode == 9) {            
        bitSet(TCCR1B, WGM13) ;
        bitSet(TCCR1A, WGM10) ;
    } if (mode == 10) {   
        bitSet(TCCR1B, WGM13) ;
        bitSet(TCCR1A, WGM11) ;
    } if (mode == 11) {            
        bitSet(TCCR1A, WGM10) ;
        bitSet(TCCR1A, WGM11) ;
        bitSet(TCCR1B, WGM13) ;
    } if (mode == 12) {            
        bitSet(TCCR1B, WGM12) ;
        bitSet(TCCR1B, WGM13) ;
    } if (mode == 13) {            
        bitSet(TCCR1B, WGM12) ;
        bitSet(TCCR1B, WGM13) ;
        bitSet(TCCR1A, WGM10) ;
    } if (mode == 14) {   
        bitSet(TCCR1A, WGM11) ;
        bitSet(TCCR1B, WGM12) ;
        bitSet(TCCR1B, WGM13) ;         
    } if (mode == 15) {            
        bitSet(TCCR1A, WGM10) ;
        bitSet(TCCR1A, WGM11) ;
        bitSet(TCCR1B, WGM12) ;
        bitSet(TCCR1B, WGM13) ;
    }      
}   




