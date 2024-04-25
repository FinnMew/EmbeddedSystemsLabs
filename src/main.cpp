// master

#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include <math.h>
#include <avr/interrupt.h>

#include "usart.h"
#include "bit.h"    

#define SS PB2
#define MOSI PB3
#define MISO PB4
#define SCK PB5

#define pinTemp PC0

int readADC(unsigned char pin) ;

volatile float tempC ;
float value_ADC ;
volatile unsigned char tempReturned ;

ISR(SPI_STC_vect) {
    bitSet(PORTB, SS) ;
    tempReturned = SPDR ;
}

ISR(ADC_vect) {
    value_ADC = readADC(pinTemp) ; // Reads how much light on photocell from conversion that just triggerd interupt
    bitSet(ADCSRA, ADSC) ;   
}

unsigned char FtoC(float) ;
float CtoF(unsigned char tempReturned) ;

int main() {
    usart_init(103) ; 
   
    bitSet(DDRB, MOSI) ;
    bitClear(DDRB, MISO) ;
    bitSet(DDRB, SCK) ;
    bitSet(DDRB, SS) ;

    bitClear(DDRC, pinTemp) ;

    bitSet(SPCR, MSTR) ;
    bitSet(SPCR, SPE) ;
    bitSet(SPCR, SPIE) ;

    bitSet(ADCSRA, ADPS2) ;     // Prescalar - 128
    bitSet(ADCSRA, ADPS1) ;     // Prescalar - 128
    bitSet(ADCSRA, ADPS0) ;     // Prescalar - 128
    bitClear(ADMUX, ADLAR) ;    // setting 0 for 10 bit res - Result is right adjusted
    bitSet(ADMUX, REFS0) ;      // Voltage reference selection - Avcc at AREFF pin
    bitSet(ADCSRA, ADIE) ;      // Enable ADC intterupt
    bitSet(ADCSRA, ADEN) ;      // Enables ADC
    bitSet(ADCSRA, ADSC) ;      // Starts first conversion
    sei() ;                     // Enable global intterupts

    while(1) {
       
        double tempK = log(10000.0 * ((1023.0 / value_ADC - 1))) ;
        tempK = 1 / (0.001129148 + (0.000234125 + (0.0000000876741 * tempK * tempK)) * tempK) ;
        tempC = tempK - 273.15 ;

        unsigned char chartempC = FtoC(tempC) ;

        bitClear(PORTB, SS) ;
        SPDR = chartempC ;
        
        usart_tx_string(">Temp Direct:") ;
        usart_tx_float(tempC, 2, 3) ;
        usart_transmit('    ') ;

        float floatTempReturned = CtoF(tempReturned) ;
        usart_tx_string(">Temp Returned:") ;
        usart_tx_float(floatTempReturned, 2, 3) ;
        usart_transmit('\n') ;

        _delay_ms(100) ;
    }
    return 0 ;
}

int readADC(unsigned char pin) {
    int wholeADC = ADCL ;       // Reads least significant 8 bits of photocell data but we want 10 bits so
    wholeADC |= (ADCH << 8) ;   // We combine ADCL with the ADCH to get the last 2 making 10 bit resltuion with a 16 bit integer
    return wholeADC ;
}

unsigned char FtoC(float x) {
    
    float scaled = 255.0 / (20) * (x - 20) ;
    unsigned char charscaled = scaled ;
    float justDecimals = scaled - charscaled ;

    if(justDecimals>= 0.5) {
        return charscaled + 1 ; // round up
    } else {
        return charscaled ; // round down
    }
}

float CtoF(unsigned char tempReturned) {
    return (float)((float)tempReturned * (20) / 255.0 + 20) ;
}



// slave

#include <avr/io.h>
#include <stdlib.h>
#include <util/delay.h>
#include <avr/interrupt.h>


#include "usart.h"
#include "bit.h"


#define SS PB2
#define MOSI PB3
#define MISO PB4
#define SCK PB5


volatile unsigned char tempReceived ;


ISR(SPI_STC_vect) {
   tempReceived = SPDR ;
   SPDR = tempReceived ;
}


float CtoF(unsigned char tempReceived) ;


int main() {


  usart_init(103) ;


  bitSet(DDRB, MISO) ;  
  bitSet(SPCR, SPE) ;
  bitSet(SPCR, SPIE) ;


  sei() ;


   while(1) {


       float floatTempReceived = CtoF(tempReceived) ;
       usart_tx_string(">Temp Received:") ;
       usart_tx_float(floatTempReceived, 2, 3) ;
       usart_transmit('\n') ;


      _delay_ms(100) ;
  }
  return 0 ;
}


float CtoF(unsigned char tempReceived) {
  return (float)((float)tempReceived * (20) / 255.0 + 20) ;
}



