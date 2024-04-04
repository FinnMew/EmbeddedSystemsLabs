#include <avr/io.h>
#include <stdlib.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#include "usart.h"
#include "bit.h"

#define pinTrig PINB0
#define pinEcho PINB1

#define pinPhotocell PC0
#define pinLED PIND6

#define pinBuzzer PIND5

float UltraSonic(float distance, volatile register unsigned int num_us) ;

int setPrescaler_tc0(char option) ;
void set_tc0_mode(char mode) ;
void my_delay(uint16_t milliseconds) ;

int readADC(unsigned char pin) ;
void ADCandTimer0settings(void) ;

void ledControl(float value_ADC, float distance) ;

void playNoteSoundLevel(float dutyCycle, float Note) ;

int buttonModeCheck(int *buttonPressCount) ;
int buttonOnOrOff(int *onOrOff) ;

float value_ADC ;

ISR(TIMER0_OVF_vect) {
}

ISR (ADC_vect) {
    value_ADC = readADC(pinPhotocell) ;
    bitSet(ADCSRA, ADSC) ;    // Start next conversion
}

int main(void) {

    usart_init(103);
    
    bitSet(DDRB, pinTrig) ;       // sets PINB0 as output
    bitClear(DDRB, pinEcho) ;     // set PINB1 as input

    bitSet(DDRD, pinLED) ;
    bitSet(DDRD, pinBuzzer) ;

    float distance = 0 ;
    volatile register unsigned int num_us = 0 ;

    bitClear(DDRD, PIND7) ;
    bitSet(PORTD, PIND7) ;

    bitClear(DDRD, PIND4) ;
    bitSet(PORTD, PIND4) ;

    int buttonPressCount = 1 ;
    int onOrOff = 1 ;

    bitSet(TIMSK0, TOIE0) ;

    ADCandTimer0settings() ;
    
    while(1) {
        
        int buzzerMode = buttonModeCheck(&buttonPressCount) ;
        int onOffSwitch = buttonOnOrOff(&onOrOff) ;
        
        if (onOffSwitch == 2) {
            bitClear(DDRD, pinLED) ;
            bitClear(DDRD, pinBuzzer) ;
        } if (onOffSwitch == 1) {
            bitSet(DDRD, pinLED) ;
            bitSet(DDRD, pinBuzzer) ;
        }

        if(buzzerMode ==  1) {
            playNoteSoundLevel(0, 245) ;
            _delay_ms(10) ;
            bitClear(DDRD, pinBuzzer) ;
        } if(buzzerMode == 2) {
            playNoteSoundLevel(0.4, 245) ;
            _delay_ms(10) ;
            bitClear(DDRD, pinBuzzer) ;
        } if(buzzerMode == 3) {
            playNoteSoundLevel(1, 245) ;
            _delay_ms(10) ;
            bitClear(DDRD, pinBuzzer) ;
        }

        distance = UltraSonic(distance, num_us) ;
        ledControl(value_ADC, distance) ;
    }
}

float UltraSonic(float distance, volatile register unsigned int num_us) {

    num_us = 0 ;
    bitSet(PORTB, pinTrig) ;
    _delay_us(10) ;
    bitClear(PORTB, pinTrig) ;

    while(!bitRead(PINB, pinEcho)) ;

    while(bitRead(PINB, pinEcho)) {
        num_us++ ;
        _delay_us(0.5) ;
    }

    distance = ((float)num_us / 1.0e6) * 343 / 2.0 * 100.0 ;
    
    return distance ;
}

int readADC(unsigned char pin) {
    int wholeADC = ADCL ;
    wholeADC |= (ADCH << 8) ;
    return wholeADC ;
}

void ledControl(float value_ADC, float distance) {

    float voltage = (value_ADC * 5) / 1023 ;
    float resistance = (1000 * (5 - voltage)) / voltage ;
    uint16_t halfT = distance / 400 * 1000 ;
    
    if (value_ADC <= 50) {
        OCR0A = 255 ;
    } else {
        OCR0A = resistance / 255 ;
    }

    my_delay(halfT) ;
    bitClear(DDRD, pinLED) ;
    my_delay(halfT) ;
}

void ADCandTimer0settings(void) {
    
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
}



void my_delay(uint16_t milliseconds) {
    for (uint16_t i = 0; i < milliseconds; ++i) {
        _delay_ms(1) ;
    }
}

int buttonModeCheck(int *buttonPressCount) {

    static bool button_status_old = true;
    bool button_status;

    button_status = bitRead(PIND, 4);

    if (button_status != button_status_old ) {
        _delay_ms(10);

        button_status = bitRead(PIND, 4);

        if (button_status == 0 && button_status_old != button_status) {
            (*buttonPressCount)++;
            
            if (*buttonPressCount == 4) {
                *buttonPressCount = 1;
            }
        }
    }

    button_status_old = button_status;

    return *buttonPressCount ;
}

int buttonOnOrOff(int *onOrOff) {

    static bool button_status_old = true;
    bool button_status;

    button_status = bitRead(PIND, 7);

    if (button_status != button_status_old ) {
        _delay_ms(10);

        button_status = bitRead(PIND, 7);
        if (button_status == 0 && button_status_old != button_status) {
            
            (*onOrOff)++;
            
            if (*onOrOff == 3) {
                *onOrOff = 1;
            }
        }
    }

    button_status_old = button_status; // current button status to compare with next cycle

    return *onOrOff ;
}

void playNoteSoundLevel(float dutyCycle, float Note) {
   
   int P = 256 ;
   int TOP = 16000000 / (P * Note) - 1 ;
   int COMP = dutyCycle * TOP ;
   OCR0B = COMP ;
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