#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#define bitSet(reg, n) (reg |= 1 << n)
#define bitRead(reg,n) (reg & (1 << n))
#define bitClear(reg, n) (reg &= ~(1 << n))

#define FOSC 16000000
#define BAUD 9600
#define MYUBRR FOSC / 16 / BAUD - 1

void USART_Init(unsigned int ubrr);
void USART_Transmit(unsigned char data);
void txString(char *pStr);

#define Trig PINB0
#define Echo PINB1

int main(void) {

    USART_Init(MYUBRR);                       // ??
    char txBuffer[9]; 
    
    DDRB = DDRB | (1<<Trig);                  // sets PINB0 as output
    DDRB = DDRB & ~(1<<Echo);                 // set PINB1 as input

    double EchoTime ;
    double distance ;
    
    while (1) {
        
        bitClear(PORTB, Trig) ;               // clear/reset Triger values
        bitClear(PORTB, Echo) ;               // clear/reset Echo values

        bitSet(PORTB, Trig) ;                 // set Triger to 1, turing sonic sensor on
        _delay_us(60);                        // keep sensor on for 60us
        bitClear(PORTB, Trig) ;               // clear/reset triger value, turn sonic sensor off

        while(!bitRead(PINB, Echo)) ;         // while Echo value 0 wait, continue once echo returns 1

        TCNT1 = 0;                            // Sets time/counter 1 to 0s
        TCCR1B |= (1 << CS10);                // start time/counter 1 with no prescaling, so will count at same frequency as cpu clock

        while(bitRead(PINB, Echo)) ;          // wait here while Echo is 1, timer is still counting in background
        TCCR1B = 0;                           // stops timer/counter 1
        EchoTime = TCNT1;                     // make EchoTime the value of the echo pluse that was just timed
        distance = (EchoTime / 343 ) / 2;     // 343 is speed of sound

        dtostrf(distance, 7, 3, txBuffer);    // converts distance (number) into a tring with 7 characters, 3 characters are after the decimal point and then stores this sting in txBuffer
        
        txString(">a:") ;                     // sends >a: over USART as message
        txString(txBuffer) ;                  // sends contents of txBuffer over USART
        USART_Transmit('\n') ;                // sends new line over USART

        // everything repeats in while loop
    }   
        }
    

void USART_Init(unsigned int ubrr) {
    
    UBRR0H = (unsigned char) (ubrr >> 8);
    UBRR0L = (unsigned char)ubrr;

    UCSR0B = (1 << RXEN0) | (1 << TXEN0);

    UCSR0C = (1 << USBS0) | (3 << UCSZ00) ;
}

void USART_Transmit(unsigned char data) {

    while (!(UCSR0A & (1 << UDRE0)));

    UDR0 = data;
}

void txString(char *pStr) {

    while (*pStr != '\0') {
        USART_Transmit(*pStr);
        pStr++;
    }
}


