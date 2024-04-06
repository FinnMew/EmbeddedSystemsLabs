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

#define pinPowerButton PIND7
#define pinBuzzerButton PIND4

void activatePins(void) ;

int setPrescaler_tc0(char option) ;
void set_tc0_mode(char mode) ;
void my_delay(uint16_t milliseconds) ;

void ADCandTimer0settings(void) ;
int readADC(unsigned char pin) ;

float UltraSonic(float distance, volatile register unsigned int num_us) ;

void ledControl(float value_ADC, float distance) ;

void playNoteSoundLevel(float dutyCycle) ;

int buttonMode_Check(int *buttonPressCount) ;
int buttonOnOrOff_Check(int *onOrOff) ;
void buttonMode_Do(int buzzerMode) ;
void buttonOnOrOff_Do(int onOffSwitch) ;

float value_ADC ;
int buzzerMode  ;
int onOffSwitch ;
int buttonPressCount = 1 ;
int onOrOff = 1 ;

ISR(TIMER0_OVF_vect) { 

} 

ISR(ADC_vect) {                         // Interupt is triggered every time an ADC conversion is completed
    value_ADC = readADC(pinPhotocell) ; // Reads how much light on photocell from conversion that just triggerd interupt
    bitSet(ADCSRA, ADSC) ;              // Starts next ADC conversion
}

int main(void) {

    float distance = 0 ;                        // Distance detected by ultrasonic
    volatile register unsigned int num_us = 0 ; // For delay function that is used to calculate ultrasonic distance

    usart_init(103) ;           // Enable usart and set Baurd rate at 9600
    activatePins() ;            // Sets all pins used as outputs/inputs
    ADCandTimer0settings() ;    // Configures ADC and Timer0 settings and starts clock

    while(1) {
        
        buttonOnOrOff_Do(onOffSwitch) ; // Depending on onOffSwitch sets or clears led and buzzer pins 
        buttonMode_Do(buzzerMode) ;     // Depending on buzzerMode changes buzzer loundness by changing dutycyle of PWM

        distance = UltraSonic(distance, num_us) ;   // Activates ultrasonic and returns distance it detects
        ledControl(value_ADC, distance) ;  // Depending on distance changes flash/buzzer rate of led/buzzer by adjusting OCR0A
    }
}

 void activatePins(void) {
    
    bitSet(DDRB, pinTrig) ;     // Sets PINB0 as output which is Trigger pin of ultrasonic
    bitClear(DDRB, pinEcho) ;   // Sets PINB1 as input which is Echo pin of ultrasonic

    bitSet(DDRD, pinLED) ;      // Sets PIND6 as output which is connected to blue and green LEDs
    bitSet(DDRD, pinBuzzer) ;   // Sets PIND5 as output which is connected to buzzer

    bitClear(DDRD, pinPowerButton) ;    // Clears PIND7 to reset it
    bitSet(PORTD, pinPowerButton) ;     // Sets PIND7 as output which is on/off button
    bitClear(DDRD, pinBuzzerButton) ;   // Clears PIND4 to reset it
    bitSet(PORTD, pinBuzzerButton) ;    // Sets PIND4 as output which is buzzer loundness control button
 }

 void ADCandTimer0settings(void) {
    
    bitSet(ADCSRA, ADPS2) ;     // Prescalar - 128
    bitSet(ADCSRA, ADPS1) ;     // Prescalar - 128
    bitSet(ADCSRA, ADPS0) ;     // Prescalar - 128
    bitClear(ADMUX, ADLAR) ;    // setting 0 for 10 bit res - Result is right adjusted
    bitSet(ADMUX, REFS0) ;      // Voltage reference selection - Avcc at AREFF pin
    bitSet(ADCSRA, ADIE) ;      // Enable ADC intterupt
    sei() ;                     // Enable global intterupts
    bitSet(ADCSRA, ADEN) ;      // Enables ADC
    bitSet(ADCSRA, ADSC) ;      // Start ADC conversion

    set_tc0_mode(3) ;   // Sets Timer0 mode - Mode 3 is Fast PWM with TOP at 255

    bitSet(TIMSK0, TOIE0) ;     // Enable Timer/Counter0 overflow interrupts
    bitClear(TCCR0A, COM0A0) ;  // Fast PWM compare mode settings for OC0A - Clearing/Resetting         
    bitClear(TCCR0A, COM0A1) ;  // Fast PWM compare mode settings for OC0A - Clearing/Resetting 
    bitClear(TCCR0A, COM0B0) ;  // Fast PWM compare mode settings for OC0B - Clearing/Resetting 
    bitClear(TCCR0A, COM0B1) ;  // Fast PWM compare mode settings for OC0B - Clearing/Resetting 

    bitSet(TCCR0A, COM0A1) ;    // Fast PWM compare mode settings for OC0A - Clear OC0A on match, set at BOTTOM
    bitSet(TCCR0A, COM0B1) ;    // Fast PWM compare mode settings for OC0B - Clear OC0B on match, set at BOTTOM

    setPrescaler_tc0(3) ;   // Sets prescaler for timer0 - option 3 is 64
}

int buttonOnOrOff_Check(int *onOrOff) {

    static bool button_status_old = 1 ; // Old button status stored as static bool so it retains value between function calls
    bool button_status ;                

    button_status = bitRead(PIND, 7) ;  // Reads status of On/Off button, if its being pressed or not

    if(button_status != button_status_old ) {   // Checks if current state is different to previous state
        
        _delay_ms(10) ;                     // If it is different wait 10ms 
        button_status = bitRead(PIND, 7) ;  // After waiting check again to make sure its actually being pressed (debouncing)

        if(button_status == 0 && button_status_old != button_status) {  // If button is actually being pressed enter if statement
            
            (*onOrOff)++ ;  // Changes status of button between 1 or 2
            
            if(*onOrOff == 3) { // Makes sure button status is only ever 1 or 2
                *onOrOff = 1 ; 
            }
        }
    }

    button_status_old = button_status ; // Current button status to compare with next cycle

    return *onOrOff ;   // Returns updated button status
}

int buttonMode_Check(int *buttonPressCount) {

    static bool button_status_old = 1 ; // Old button status stored as static bool so it retains value between function calls
    bool button_status ;                

    button_status = bitRead(PIND, 4) ;  // Reads status of buzzer mode button, if its being pressed or not

    if(button_status != button_status_old ) {   // Checks if current state is different to previous state
        
        _delay_ms(10) ;                     // If it is different wait 10ms 
        button_status = bitRead(PIND, 4) ;  // After waiting check again to make sure its actually being pressed (debouncing)

        if(button_status == 0 && button_status_old != button_status) {  // If button is actually being pressed enter if statement
            
            (*buttonPressCount)++ ;  // Changes status of button between 1, 2 or 3
            
            if(*buttonPressCount == 4) { // Makes sure button status is only ever 1, 2 or 3
                *buttonPressCount = 1 ; 
            }
        }
    }

    button_status_old = button_status ; // Current button status to compare with next cycle

    return *buttonPressCount ;   // Returns updated button status

}

void buttonOnOrOff_Do(int onOffSwitch) {

    if(onOffSwitch == 1) {
        bitSet(DDRD, pinLED) ;      // If onOffSwitch is 1 set LED pin (allow power)
        bitSet(DDRD, pinBuzzer) ;   // If onOffSwitch is 1 set Buzzer pin (allow power)
    } if(onOffSwitch == 2) {
        bitClear(DDRD, pinLED) ;    // If onOffSwitch is 2 set LED pin (NO power)
        bitClear(DDRD, pinBuzzer) ; // If onOffSwitch is 2 set Buzzer pin (NO power)
    }
}

void buttonMode_Do(int buzzerMode) {

    if(buzzerMode ==  1) {              
        playNoteSoundLevel(0) ;    // Mode 1 plays sound with dutyCyle 0 - quiet sound
        _delay_ms(10) ;                 // Keeps Buzzer on for 10ms
        bitClear(DDRD, pinBuzzer) ;     // Turns Buzzer off
    } if(buzzerMode == 2) {
        playNoteSoundLevel(0.3) ;  // Mode 2 plays sound with dutyCyle 0.3 - moderate sound
        _delay_ms(10) ;                 // Keeps Buzzer on for 10ms
        bitClear(DDRD, pinBuzzer) ;     // Turns Buzzer off
    } if(buzzerMode == 3) {
        playNoteSoundLevel(1) ;    // Mode 3 plays sound with dutyCyle 1 - loud sound
        _delay_ms(10) ;                 // Keeps Buzzer on for 10ms
        bitClear(DDRD, pinBuzzer) ;     // Turns Buzzer off
    }
}

float UltraSonic(float distance, volatile register unsigned int num_us) {

    num_us = 0 ;
    bitSet(PORTB, pinTrig) ;    // Send out sonic signal
    _delay_us(10) ;             // Keep signal on for 10us
    bitClear(PORTB, pinTrig) ;  // Stop sonic signal

    while(!bitRead(PINB, pinEcho)) ;    // Wait until sonic signal returns

    while(bitRead(PINB, pinEcho)) {
        num_us++ ;          // Increment num_us every 0.5us while signal is being returned
        _delay_us(0.5) ;    // 0.5 not 1 so the time between flashes is longer
    }

    distance = ((float)num_us / 1.0e6) * 343 / 2.0 * 100.0 ;    // Calculate distance in cm
    
    return distance ;  // Return distance (cm) to main while loop
}

void ledControl(float value_ADC, float distance) {

    float voltage = (value_ADC * 5) / 1023 ;                // Calculate voltage of photocell   
    float resistance = (1000 * (5 - voltage)) / voltage ;   // Calculate resistance of photocell
    uint16_t halfT = distance / 400 * 1000 ;                // Calculate time between flashes/buzzes based on distance ratio out of 4m

    if (value_ADC <= 100) {         // If less than 100 is detected from photocell make max brightness of LED 
        OCR0A = 255 ;               // Makes duty cycle 100% so max brightness
    }  if (value_ADC >= 800) {      // If less more than 800 detected turn LED off (doesnt really work still gets residual light)
        bitClear(DDRD, pinLED) ;    // Is meant to turn LED off
    } else {
        OCR0A = resistance / 255 ;  // Set LED brightness on scale where - high light turns LED down, low light turns LED up
    }                              

    my_delay(halfT) ;           // Keep LED on at x brightness
    bitClear(DDRD, pinLED) ;    // Turn LED off
    my_delay(halfT) ;           // Keep LED off, function in while so repeats quickly and is turned back on after this delay 
}

void playNoteSoundLevel(float dutyCycle) {
   
   int TOP = 255 ;
   int COMP = dutyCycle * TOP ; // Changes compare value depending on dutycycle
   OCR0B = COMP ;               // Set compare value - this changes how much of each period the buzzer is on - OCR0B = 0 = quiet - OCR0B = 255 = loud
   }

int readADC(unsigned char pin) {
    int wholeADC = ADCL ;       // Reads least significant 8 bits of photocell data but we want 10 bits so
    wholeADC |= (ADCH << 8) ;   // We combine ADCL with the ADCH to get the last 2 making 10 bit resltuion with a 16 bit integer
    return wholeADC ;
}

void my_delay(uint16_t milliseconds) {
    for(uint16_t i = 0; i < milliseconds; ++i) {            // Takes haltT which is x many milliseconds and wait x amount of time by cycling for loop
        onOffSwitch = buttonOnOrOff_Check(&onOrOff) ;       // Finds state of switch, 1 for on 2 for off - Here make button effective at all times
        buzzerMode = buttonMode_Check(&buttonPressCount) ;  // Finds state of buzzer mode - Here make button effective at all times
        _delay_ms(1) ;                                      // Wait 1ms
    }
}

int setPrescaler_tc0(char option) {
   
    if(option == 0) {
        bitClear(TCCR0B, CS00) ;
        bitClear(TCCR0B, CS01) ;
        bitClear(TCCR0B, CS02) ;
        return 0 ;
    } if(option == 1) {          // T = 1/(16000000/1) = 6.25e-8
        bitSet(TCCR0B, CS00) ;
        return 1 ;
    } if(option == 2) {          // T = 1/(16000000/8) = 5e-7
        bitSet(TCCR0B, CS01) ;
        return 8 ;
    } if(option == 3) {          // T = 1/(16000000/64) = 4e-6
        bitSet(TCCR0B, CS00) ;
        bitSet(TCCR0B, CS01) ;
        return 64 ;
    } if(option == 4) {          // T = 1/(16000000/256) = 16e-6
        bitSet(TCCR0B, CS02) ;
        return 256 ;
    } if(option == 5) {          // T = 1/(16000000/1024) = 64e-6
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
    } if(mode == 1) {             // PWM, phase correct, 0xFF, TOP, BOTTOM
        bitSet(TCCR0A, WGM00) ;
    } if(mode == 2) {             // CTC, OCRA, immediate, max
        bitSet(TCCR0A, WGM01) ;
    } if(mode == 3) {             // Fast PWM, 0xFF, BOTTOM, MAX
        bitSet(TCCR0A, WGM00) ;
        bitSet(TCCR0A, WGM01) ;
    } if(mode == 4) {             // Reserved, -, -, -
        bitSet(TCCR0B, WGM02) ;
    } if(mode == 5) {             // PWM, phase correct, OCRA, TOP, BOTTOM
        bitSet(TCCR0B, WGM02) ;
        bitSet(TCCR0A, WGM00) ;
    } if(mode == 6) {             // Reserved, -, -, -
        bitSet(TCCR0B, WGM02) ;
        bitSet(TCCR0A, WGM01) ;
    } if(mode == 7) {             // Fast PWM, OCRA, BOTTOM, TOP
        bitSet(TCCR0A, WGM00) ;
        bitSet(TCCR0A, WGM01) ;
        bitSet(TCCR0B, WGM02) ;
    }
}

