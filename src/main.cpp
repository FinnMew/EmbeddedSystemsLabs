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

void playNotesSoundLevel(float dutyCycle, float Notes) ;

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
   bitSet(DDRD, PIND5) ;
 
   bitSet(TIMSK0, TOIE0) ; // timer/counter0 overflow interrupt enabled

   float notes7[] = {1046.5, 1174.66, 1318.51, 1396.91, 1567.98, 1760, 1975.53} ;
   float *pNotes7 = notes7 ;

   sei() ; // interupt enabled

   set_tc0_mode(7) ; // PWM fast mode selected
   bitClear(TCCR0A, COM0A0) ;
   bitClear(TCCR0A, COM0A1) ;
   bitClear(TCCR0A, COM0B0) ;
   bitClear(TCCR0A, COM0B1) ;

   bitSet(TCCR0A, COM0A1) ;
   bitSet(TCCR0A, COM0B1) ;

   setPrescaler_tc0(3) ;

   while (1) {
    

       for(char i = 0; i < sizeof(notes7)/sizeof(float); i++) {
           playNotesSoundLevel(0.1, *(pNotes7 + i)) ;
           _delay_ms(1000) ;
       }
      
       for(char j = 0; j < sizeof(notes7)/sizeof(float); j++) {
           playNotesSoundLevel(0.2, *(pNotes7 + j)) ;
           _delay_ms(1000) ;
       }
      
       for(char k = 0; k < sizeof(notes7)/sizeof(float); k++) {
           playNotesSoundLevel(0.3, *(pNotes7 + k)) ;
           _delay_ms(1000) ;
       }
      
      
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


void playNotesSoundLevel(float dutyCycle, float Notes) {

   int P = 64 ;
   
   int TOP ;
   int COMP ;
   float A = 16e6/Notes/P ;
   float B = A - int(A) ;
   float C = A - B ;
   if(B >= 0.5) {
       TOP = C ;
   } else if (B < 0.5) {
       TOP = C - 1 ;
   }
   float D = TOP * dutyCycle ;
   float E = D - int(D) ;
   float F = D - E ;
   if(E >= 0.5) {
       COMP = F ;
   } else if (E < 0.5) {
       COMP = F - 1 ;
   }

   OCR0A = TOP ;
   OCR0B = COMP ;

   }



