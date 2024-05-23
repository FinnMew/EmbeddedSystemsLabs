// most recent version 4.31am 24/5

#include <avr/io.h>
#include <stdlib.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <math.h>

#include <DHT.h>
#include <Adafruit_Sensor.h>

#include "usart.h"
#include "bit.h"

#define pinTrig PIND4
#define pinEcho PIND3

#define pinPhotocell PC0 

#define pinRed PC1
#define pinYellow PC2
#define pinBlue PC3
#define pinWhite PB0
#define pinBuzzer PC4

#define pinFanPWM PB3
#define pinDirection1 PB4
#define pinDirection2 PB5
#define pinPostive PD6
#define pinNegative PD7

#define pinServoPWM PB2
#define pinServoButton PIND2

#define BUFFER_SIZE 50
#define Type DHT11

int setPrescaler_tc0(char option) ;
void set_tc0_mode(char mode) ;
int setPrescaler_tc1(int option) ;
void set_tc1_mode(char mode) ;
int setPrescaler_tc2(char option) ;
void set_tc2_mode(char mode) ;

void ADCsetup(void) ;
int readADC(unsigned char pin) ;
void photosensorActive(float value_ADC) ; 

void ultraSonicSetup(void) ;
void ultraSonicActive(void) ;
void intruderCheck(int mode, float radius, float front, float left, float right) ;

void fanSetup(void) ;
void fanActive(int power) ;

void buzzerSetup(void) ;
void buzzerActive(int power) ;

void dutyCycleSetup(void) ;

void servoSetup(int prescalar, char mode) ;
void servoActive(int rotationMode) ;

void dht11Setup(void) ;
void checkEnvironment(float humidity, float temperature) ;
void sendDHT11values(void) ;

float value_ADC;
char PhotoResValArray[11];

volatile unsigned long numOV1 = 0 ;
volatile unsigned long numOV2 = 0 ;

char array[10] ;
unsigned int icr1 ;
unsigned int icr2 ;
float Ton ;
float Toff ;
float dutyCycle ;
float dutyCycleReal ;

int buzzerPower = 0 ;
int buzzerloop ;

float echoTime ;
float distance ;
char valueDistance[9] ;
int echoHighLow ;

int UltraPOS = 140 ;
int UltraDirection = 0 ;
int rotationMode = 0 ;
int loopCount = 0 ;
int buttonCheck ;

unsigned char data_buffer[BUFFER_SIZE];
unsigned char *pRx = data_buffer;
unsigned char *pTx = data_buffer;

int sensePin = 19 ;
DHT HT(sensePin, Type) ;
float humidity ;
float temperature ;
char valueHumidity[10] ;
char valueTemperature[10] ;


int main (void) {
    init() ;
    usart_init(103) ;
    ADCsetup() ;
    fanSetup() ;
    buzzerSetup() ;
    dutyCycleSetup() ;
    ultraSonicSetup() ;
    servoSetup(3, 15) ;
    dht11Setup() ;

    sei() ;


    while(1) {
       
        servoActive(rotationMode) ;
        ultraSonicActive() ;


        humidity = HT.readHumidity() ;
        temperature = HT.readTemperature() ;
        checkEnvironment(humidity, temperature) ;
        sendDHT11values() ;
        photosensorActive(value_ADC) ;

        intruderCheck(1, 100.0, 13.0, 12.0, 13.0) ;
        
        if(buzzerPower == 0) {
            buzzerActive(0) ;
        } if(buzzerPower == 1) {
            buzzerActive(1) ;
        }

        float ocra = OCR2A ;
        float top = 255 ;
        dutyCycleReal = (ocra / top) * 100 ;
        dtostrf(dutyCycleReal, 3, 0, array) ;
        usart_tx_string(">Duty Actual: ") ;
        usart_tx_string(array) ;
        usart_transmit('\n') ;
        
        dtostrf(dutyCycle, 3, 4, array) ;
        usart_tx_string(">Duty Measured: ") ;
        usart_tx_string(array) ;
        usart_transmit('\n') ;

        loopCount++ ;
        _delay_ms(300) ;
       
    }
}

void intruderCheck(int mode, float radius, float front, float left, float right) {
    bitClear(PORTC, pinRed) ;
    float upperDist ;
    float lowerDist ;
    if(mode == 0) {
        upperDist = radius + 15.0 ;
        lowerDist = radius - 15.0 ;
        if(distance >= upperDist || distance <= lowerDist) {
            bitSet(PORTC, pinRed) ;
        }
    }if(mode == 1) {
        float expectedDist ;
        int ocr1b = OCR1B ;
        int theta = 180 * ((50 * (10 - ((645 - ocr1b) / 50))) / 500) ;
        if(theta >= 0 && theta < 90) {
            expectedDist = right / cos(theta) ;
        } if(theta == 90) {
            expectedDist = front ;
        } if(theta > 90 && theta <= 180) {
            expectedDist = (-1.0) * (left / cos(theta)) ;
        }
        //upperDist = expectedDist + 15.0 ;
        //lowerDist = expectedDist - 15.0 ;
        upperDist = expectedDist + 8.0 ;
        lowerDist = expectedDist - 8.0 ;
        if(distance >= upperDist || distance <= lowerDist) {
            bitSet(PORTC, pinRed) ;
        }
    }

}

void dutyCycleSetup(void) {

    bitClear(DDRD, pinNegative) ;
    bitClear(DDRD, pinPostive) ;

    bitSet(ACSR, ACIC) ;   
    bitSet(ACSR, ACIS0) ;
    bitSet(ACSR, ACIS1) ;
    bitSet(TIMSK1, ICIE1) ;
    bitSet(TCCR1B, ICNC1) ; 
    bitSet(TCCR1B, ICES1) ;  
}

ISR(TIMER1_CAPT_vect) {
    if(bitRead(ACSR, ACO)) {
        bitClear(ACSR, ACIS0) ;
        bitClear(TCCR1B, ICES1) ;
    } if(!bitRead(ACSR, ACO)) {
        bitSet(ACSR, ACIS0) ;
        bitSet(TCCR1B, ICES1) ;
    }
    if(!bitRead(TCCR1B, ICES1)) {
        icr1 = ICR1 ;
        Toff = (icr1 - icr2) ;
        dutyCycle = Ton / (Ton + Toff) * 100 ;
        if(dutyCycle >= 90) {
            buzzerloop++ ;
            if(buzzerloop == 3) {
                buzzerPower = 1 ;
            }
        } else {
            buzzerPower = 0 ;
            buzzerloop = 0 ;
        }
    } if(bitRead(TCCR1B, ICES1)) {
        icr2 = ICR1 ;
        Ton = (icr2 - icr1) ;
    }

}

ISR(TIMER1_OVF_vect) {
   numOV1++ ;
}

void ADCsetup(void) {

    bitSet(DDRB, pinWhite) ;

    bitSet(ADCSRA, ADPS2);   // prescaler
    bitSet(ADCSRA, ADPS1);   // prescaler
    bitSet(ADCSRA, ADPS0);   // prescaler
    bitClear(ADMUX, ADLAR);  // setting 0 for 10 bit
    bitSet(ADMUX, REFS0);    // Aref settings
    bitSet(ADCSRA, ADIE);    // Enable interrupt
                     
    bitSet(ADCSRA, ADEN);    // Enables ADC
    bitSet(ADCSRA, ADSC);    // Start conversion
}

int readADC(unsigned char pin) {
    int volt = ADCL;
    volt |= (ADCH << 8);
    return volt;
}

void photosensorActive(float value_ADC) {
    if(value_ADC >= 500) { 
        bitSet(PORTB, pinWhite) ;
    } else {
        bitClear(PORTB, pinWhite) ;
    }
}

ISR (ADC_vect) {
    value_ADC = readADC(pinPhotocell);
    bitSet(ADCSRA, ADSC);    // Start next conversion if needed
}

void buzzerSetup(void) {
    bitSet(DDRC, pinBuzzer) ;
}
void buzzerActive(int power) {
    if(power == 0) {
        bitClear(PORTC, pinBuzzer) ;
        bitClear(DDRC, pinBuzzer) ;
    } if(power == 1) {
        bitSet(DDRC, pinBuzzer) ;
        bitSet(PORTC, pinBuzzer) ;
        
    }
}

void fanSetup(void) {
    bitSet(DDRB, pinFanPWM) ;
    bitSet(PORTB, pinDirection1) ;      
    bitClear(PORTB, pinDirection2) ;
    bitSet(DDRB, pinDirection1) ;
    bitSet(DDRB, pinDirection2) ;
    bitSet(TCCR2A, COM2A1) ;
}


void fanActive(int power) {
    if(power == 0) {
        OCR2A = 1 ;
    } if(power == 1) {
        OCR2A = 254 ;
    }
}


void dht11Setup() {
    bitSet(DDRC, pinBlue) ;
    bitSet(DDRC, pinYellow) ;
    HT.begin() ;
    bitSet(UCSR0B, RXCIE0);
}


void checkEnvironment(float humidity, float temperature) {


    if(temperature >= 18 && temperature <= 25) {
        bitSet(PORTC, pinYellow) ;
    } if(humidity >= 75 && humidity <= 85) {
        bitSet(PORTC, pinBlue) ;
    } if(humidity > 85 || temperature > 25) {
        fanActive(1) ;
    } if(humidity < 75){
        bitClear(PORTC, pinBlue) ;
    } if(temperature < 18) {
        bitClear(PORTC, pinYellow) ;
    } if(humidity < 80 && temperature < 25) {
        fanActive(0) ;
    }
}


void sendDHT11values() {


    dtostrf(humidity, 7, 3, valueHumidity);
    valueHumidity[7] = '\0';  
    dtostrf(temperature, 7, 3, valueTemperature);
    valueTemperature[7] = '\0';
   
    snprintf((char *)data_buffer, BUFFER_SIZE, ">Humidity:%s\n>Temperature:%s\n", valueHumidity, valueTemperature);
   
    pTx = data_buffer;


    bitClear(UCSR0B, RXCIE0);
    bitSet(UCSR0B, UDRIE0);
}


ISR(USART_RX_vect) {
    *pRx = UDR0;


    if (*pRx == '\n') {
        *(pRx + 1) = '\0';
        pRx = data_buffer;


        bitClear(UCSR0B, RXCIE0);
        bitSet(UCSR0B, UDRIE0);
        while (!bitRead(UCSR0A, UDRE0));
        UDR0 = '\0';
    } else {
        pRx++;
    }
}


ISR(USART_UDRE_vect) {
    if (*pTx == '\0') {
        pTx = data_buffer;
        bitClear(UCSR0B, UDRIE0);
        bitSet(UCSR0B, RXCIE0);
    } else {
        UDR0 = *pTx;
        pTx++;
    }
}


ISR(TIMER2_OVF_vect) {
    numOV2++ ;
}


ISR(INT0_vect) {
    buttonCheck = bitRead(PIND, 2) ;
    if(buttonCheck == 1) {
        _delay_ms(60) ;
        buttonCheck = bitRead(PIND, 2) ;
        if(buttonCheck == 1) {
            rotationMode++ ;
            if(rotationMode == 10) {
                rotationMode = 0 ;
            }
        }
    }
}


ISR(INT1_vect) {
    if(echoHighLow == 0) {
        TCNT2 = 0 ;
        numOV2 = 0 ;
        setPrescaler_tc2(6) ;  
    } if(echoHighLow == 1) {
        setPrescaler_tc2(0) ;
        echoTime = (TCNT2 + (numOV2 * 256)) * (1.0 / (16000000.0 / 256.0));
        distance = (echoTime * 343.0) / 2.0 * 100.0;
         setPrescaler_tc2(6) ;
        
        dtostrf(distance, 7, 3, valueDistance) ;
        valueDistance[7] = '\0' ;
        usart_tx_string(">a:") ;
        usart_tx_string(valueDistance) ;
        usart_transmit('\n') ;

        dtostrf(value_ADC, 4, 3, PhotoResValArray);
        PhotoResValArray[11] = '\0';
        usart_tx_string(">adc:");
        usart_tx_string(PhotoResValArray);
        usart_transmit('\n');
       
    }    
}


void ultraSonicSetup(void) {
    bitSet(DDRC, pinRed) ;
    set_tc2_mode(3) ;
    bitSet(TIMSK2, TOIE2) ;
    bitSet(DDRD, pinTrig) ;
    bitClear(DDRD, pinEcho) ;
    bitSet(EIMSK, INT1) ;
    bitSet(EICRA, ISC10) ;
}


void ultraSonicActive(void) {


    bitSet(DDRD, pinTrig) ;
    bitClear(PORTD, pinTrig) ;              
    bitClear(PORTD, pinEcho) ;


    bitSet(PORTD, pinTrig) ;
    _delay_us(10) ;
    bitClear(PORTD, pinTrig) ;


    echoHighLow = 0 ;
    while(!bitRead(PIND, pinEcho)) ;
    echoHighLow = 1 ;
    while(bitRead(PIND, pinEcho)) ;
}


void servoSetup(int prescalar, char mode) {
   
    bitSet(DDRB, pinServoPWM) ;
    bitSet(PORTD, pinServoButton) ;
    bitSet(TIMSK1, TOIE1) ;
    bitSet(TCCR1A, COM1B1) ;
    bitSet(EIMSK, INT0) ;
    bitSet(EICRA, ISC00) ;
    bitSet(EICRA, ISC01) ;
    set_tc1_mode(mode) ;
    setPrescaler_tc1(prescalar) ;
    OCR1A = 5000 ;
}


void servoActive(int rotationMode) {
    setPrescaler_tc1(3) ;
    if(rotationMode == 0) {
        if(loopCount == 8) {
            if(UltraDirection == 0 && UltraPOS <= 645) {
                OCR1B = UltraPOS;
                UltraPOS += 50;
                if (UltraPOS > 645) {
                    UltraPOS = 645;
                    UltraDirection = 1;
                }
            }else if(UltraDirection == 1 && UltraPOS >= 145) {
                OCR1B = UltraPOS;
                UltraPOS -= 50;
                if (UltraPOS < 145) {
                    UltraPOS = 145;
                    UltraDirection = 0;
                }
            }
            loopCount = 0 ;
        }
    }
   
    if(rotationMode == 1) {
        OCR1B = 145 ;
    }if(rotationMode == 2) {
        OCR1B = 270 ;
    }if(rotationMode == 3) {
        OCR1B = 395 ;
    }if(rotationMode == 4) {
        OCR1B = 520 ;
    }if(rotationMode == 5) {
        OCR1B = 645 ;
    }if(rotationMode == 6) {
        OCR1B = 520 ;
    }if(rotationMode == 7) {
        OCR1B = 395 ;
    }if(rotationMode == 8) {
        OCR1B = 270 ;
    }if(rotationMode == 9) {
        OCR1B = 145 ;
        loopCount = 0 ;
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


int setPrescaler_tc2(char option) {
   
    if(option == 0) {
        bitClear(TCCR2B, CS20) ;
        bitClear(TCCR2B, CS21) ;
        bitClear(TCCR2B, CS22) ;
        return 0 ;
    } if(option == 1) {          // T = 1/(16000000/1) = 6.25e-8
        bitSet(TCCR2B, CS20) ;
        return 1 ;
    } if(option == 2) {          // T = 1/(16000000/8) = 5e-7
        bitSet(TCCR2B, CS21) ;
        return 8 ;
    } if(option == 3) {
        bitSet(TCCR2B, CS20) ;
        bitSet(TCCR2B, CS21) ;
        return 32 ;
    } if(option == 4) {          // T = 1/(16000000/64) = 4e-6
        bitSet(TCCR2B, CS22) ;
        return 64 ;
    } if(option == 5) {
        bitSet(TCCR2B, CS20) ;
        bitSet(TCCR2B, CS22) ;
        return 128 ;
    } if(option == 6) {          // T = 1/(16000000/256) = 16e-6
        bitSet(TCCR2B, CS21) ;
        bitSet(TCCR2B, CS22) ;
        return 256 ;
    } if(option == 7) {          // T = 1/(16000000/1024) = 64e-6
        bitSet(TCCR2B, CS20) ;
        bitSet(TCCR2B, CS21) ;
        bitSet(TCCR2B, CS22) ;
        return 1024 ;
    } else {
        return 0 ;
    }
}


void set_tc2_mode(char mode) {
   
    if(mode == 0) {                // normal, 0xFF, immediate, Max
        bitClear(TCCR2A, WGM20) ;
        bitClear(TCCR2A, WGM21) ;
        bitClear(TCCR2B, WGM22) ;
    } if(mode == 1) {             // PWM, phase correct, 0xFF, TOP, BOTTOM
        bitSet(TCCR2A, WGM20) ;
    } if(mode == 2) {             // CTC, OCRA, immediate, max
        bitSet(TCCR2A, WGM21) ;
    } if(mode == 3) {             // Fast PWM, 0xFF, BOTTOM, MAX
        bitSet(TCCR2A, WGM20) ;
        bitSet(TCCR2A, WGM21) ;
    } if(mode == 4) {             // Reserved, -, -, -
        bitSet(TCCR2B, WGM22) ;
    } if(mode == 5) {             // PWM, phase correct, OCRA, TOP, BOTTOM
        bitSet(TCCR2B, WGM22) ;
        bitSet(TCCR0A, WGM20) ;
    } if(mode == 6) {             // Reserved, -, -, -
        bitSet(TCCR2B, WGM22) ;
        bitSet(TCCR2A, WGM21) ;
    } if(mode == 7) {             // Fast PWM, OCRA, BOTTOM, TOP
        bitSet(TCCR2A, WGM20) ;
        bitSet(TCCR2A, WGM21) ;
        bitSet(TCCR2B, WGM22) ;
    }
}

















#include <avr/io.h>
#include <stdlib.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <math.h>

#include <DHT.h>
#include <Adafruit_Sensor.h>

#include "usart.h"
#include "bit.h"

#define pinTrig PIND4
#define pinEcho PIND3

#define pinPhotocell PC0 

#define pinRed PC1
#define pinYellow PC2
#define pinBlue PC3
#define pinWhite PD5
#define pinBuzzer PC4

#define pinFanPWM PB3
#define pinDirection1 PB4
#define pinDirection2 PB5
#define pinPostive PD6
#define pinNegative PD7

#define pinServoPWM PB2
#define pinServoButton PIND2

#define BUFFER_SIZE 50
#define Type DHT11

int setPrescaler_tc0(char option) ;
void set_tc0_mode(char mode) ;
int setPrescaler_tc1(int option) ;
void set_tc1_mode(char mode) ;
int setPrescaler_tc2(char option) ;
void set_tc2_mode(char mode) ;

void ADCsetup(void) ;
int readADC(unsigned char pin) ;
void photosensorActive(float value_ADC) ; 

void ultraSonicSetup(void) ;
void ultraSonicActive(void) ;
void intruderCheck(int mode, float radius, float front, float left, float right) ;

void fanSetup(void) ;
void fanActive(int power) ;

void buzzerSetup(void) ;
void buzzerActive(int power) ;

void dutyCycleSetup(void) ;

void servoSetup(int prescalar, char mode) ;
void servoActive(int rotationMode) ;

void dht11Setup(void) ;
void checkEnvironment(float humidity, float temperature) ;
void sendDHT11values(void) ;

float value_ADC;
char PhotoResValArray[11];

volatile unsigned long numOV1 = 0 ;
volatile unsigned long numOV2 = 0 ;

char array[10] ;
unsigned int icr1 ;
unsigned int icr2 ;
float Ton ;
float Toff ;
float dutyCycle ;
float dutyCycleReal ;

int buzzerPower = 0 ;

float echoTime ;
float distance ;
char valueDistance[9] ;
int echoHighLow ;

int UltraPOS = 140 ;
int UltraDirection = 0 ;
int rotationMode = 0 ;
int loopCount = 0 ;
int buttonCheck ;

unsigned char data_buffer[BUFFER_SIZE];
unsigned char *pRx = data_buffer;
unsigned char *pTx = data_buffer;

int diditgethere = 0 ;

int sensePin = 19 ;
DHT HT(sensePin, Type) ;
float humidity ;
float temperature ;
char valueHumidity[10] ;
char valueTemperature[10] ;


int main (void) {
    init() ;
    usart_init(103) ;
    ADCsetup() ;
    fanSetup() ;
    buzzerSetup() ;
    dutyCycleSetup() ;
    ultraSonicSetup() ;
    servoSetup(3, 15) ;
    dht11Setup() ;
   


    sei() ;


    while(1) {
       
        servoActive(rotationMode) ;
        ultraSonicActive() ;


        humidity = HT.readHumidity() ;
        temperature = HT.readTemperature() ;
        checkEnvironment(humidity, temperature) ;
        sendDHT11values() ;
        photosensorActive(value_ADC) ;

        intruderCheck(1, 100.0, 13.0, 12.0, 13.0) ;
        
        if(buzzerPower == 0) {
            buzzerActive(0) ;
        } if(buzzerPower == 1) {
            buzzerActive(1) ;
        }

        float ocra = OCR2A ;
        float top = 255 ;
        dutyCycleReal = (ocra / top) * 100 ;
        dtostrf(dutyCycleReal, 3, 0, array) ;
        usart_tx_string(">Duty Actual: ") ;
        usart_tx_string(array) ;
        usart_transmit('\n') ;
        
        dtostrf(dutyCycle, 3, 4, array) ;
        usart_tx_string(">Duty Measured: ") ;
        usart_tx_string(array) ;
        usart_transmit('\n') ;

        loopCount++ ;
        _delay_ms(300) ;
       
    }
}

void intruderCheck(int mode, float radius, float front, float left, float right) {
    bitClear(PORTC, pinRed) ;
    float upperDist ;
    float lowerDist ;
    if(mode == 0) {
        upperDist = radius + 15.0 ;
        lowerDist = radius - 15.0 ;
        if(distance >= upperDist || distance <= lowerDist) {
            bitSet(PORTC, pinRed) ;
        }
    }if(mode == 1) {
        float expectedDist ;
        int ocr1b = OCR1B ;
        int theta = 180 * ((50 * (10 - ((645 - ocr1b) / 50))) / 500) ;
        if(theta >= 0 && theta < 90) {
            expectedDist = right / cos(theta) ;
        } if(theta == 90) {
            expectedDist = front ;
        } if(theta > 90 && theta <= 180) {
            expectedDist = (-1.0) * (left / cos(theta)) ;
        }
        //upperDist = expectedDist + 15.0 ;
        //lowerDist = expectedDist - 15.0 ;
        upperDist = expectedDist + 8.0 ;
        lowerDist = expectedDist - 8.0 ;
        if(distance >= upperDist || distance <= lowerDist) {
            bitSet(PORTC, pinRed) ;
        }
    }

}

void dutyCycleSetup(void) {

    bitClear(DDRD, pinNegative) ;
    bitClear(DDRD, pinPostive) ;

    bitSet(ACSR, ACIC) ;   
    bitSet(ACSR, ACIS0) ;
    bitSet(ACSR, ACIS1) ;
    bitSet(TIMSK1, ICIE1) ;
    bitSet(TCCR1B, ICNC1) ; 
    bitSet(TCCR1B, ICES1) ;  
}

ISR(TIMER1_CAPT_vect) {
    if(bitRead(ACSR, ACO)) {
        bitClear(ACSR, ACIS0) ;
        bitClear(TCCR1B, ICES1) ;
    } if(!bitRead(ACSR, ACO)) {
        bitSet(ACSR, ACIS0) ;
        bitSet(TCCR1B, ICES1) ;
    }
    if(!bitRead(TCCR1B, ICES1)) {
        icr1 = ICR1 ;
        Toff = (icr1 - icr2) ;
        dutyCycle = Ton / (Ton + Toff) * 100 ;
        if(dutyCycle >= 80) {
            buzzerPower = 1 ;
        } else {
            buzzerPower = 0 ;
        }
    } if(bitRead(TCCR1B, ICES1)) {
        icr2 = ICR1 ;
        Ton = (icr2 - icr1) ;
    }

}

ISR(TIMER1_OVF_vect) {
   numOV1++ ;
}

void ADCsetup(void) {

    bitSet(DDRD, pinWhite) ;

    bitSet(ADCSRA, ADPS2);   // prescaler
    bitSet(ADCSRA, ADPS1);   // prescaler
    bitSet(ADCSRA, ADPS0);   // prescaler
    bitClear(ADMUX, ADLAR);  // setting 0 for 10 bit
    bitSet(ADMUX, REFS0);    // Aref settings
    bitSet(ADCSRA, ADIE);    // Enable interrupt
                     
    bitSet(ADCSRA, ADEN);    // Enables ADC
    bitSet(ADCSRA, ADSC);    // Start conversion
}

int readADC(unsigned char pin) {
    int volt = ADCL;
    volt |= (ADCH << 8);
    return volt;
}

void photosensorActive(float value_ADC) {
    if(value_ADC >= 500) { 
        bitSet(PORTD, pinWhite) ;
    } else {
        bitClear(PORTD, pinWhite) ;
    }
}

ISR (ADC_vect) {
    value_ADC = readADC(pinPhotocell);
    bitSet(ADCSRA, ADSC);    // Start next conversion if needed
}

void buzzerSetup(void) {
    bitSet(DDRC, pinBuzzer) ;
}
void buzzerActive(int power) {
    if(power == 0) {
        bitClear(PORTC, pinBuzzer) ;
        bitClear(DDRC, pinBuzzer) ;
    } if(power == 1) {
        bitSet(DDRC, pinBuzzer) ;
        bitSet(PORTC, pinBuzzer) ;
        
    }
}

void fanSetup(void) {
    bitSet(DDRB, pinFanPWM) ;
    bitSet(PORTB, pinDirection1) ;      
    bitClear(PORTB, pinDirection2) ;
    bitSet(DDRB, pinDirection1) ;
    bitSet(DDRB, pinDirection2) ;
    bitSet(TCCR2A, COM2A1) ;
}


void fanActive(int power) {
    if(power == 0) {
        OCR2A = 1 ;
    } if(power == 1) {
        OCR2A = 254 ;
    }
}


void dht11Setup() {
    bitSet(DDRC, pinBlue) ;
    bitSet(DDRC, pinYellow) ;
    bitSet(DDRC, pinBuzzer) ;
    HT.begin() ;
    bitSet(UCSR0B, RXCIE0);
}


void checkEnvironment(float humidity, float temperature) {


    if(temperature >= 18 && temperature <= 25) {
        bitSet(PORTC, pinYellow) ;
    } if(humidity >= 75 && humidity <= 85) {
        bitSet(PORTC, pinBlue) ;
    } if(humidity > 85 || temperature > 25) {
        fanActive(1) ;
    } if(humidity < 75){
        bitClear(PORTC, pinBlue) ;
    } if(temperature < 18) {
        bitClear(PORTC, pinYellow) ;
    } if(humidity < 80 && temperature < 25) {
        fanActive(0) ;
    }
}


void sendDHT11values() {


    dtostrf(humidity, 7, 3, valueHumidity);
    valueHumidity[7] = '\0';  
    dtostrf(temperature, 7, 3, valueTemperature);
    valueTemperature[7] = '\0';
   
    snprintf((char *)data_buffer, BUFFER_SIZE, ">Humidity:%s\n>Temperature:%s\n", valueHumidity, valueTemperature);
   
    pTx = data_buffer;


    bitClear(UCSR0B, RXCIE0);
    bitSet(UCSR0B, UDRIE0);
}


ISR(USART_RX_vect) {
    *pRx = UDR0;


    if (*pRx == '\n') {
        *(pRx + 1) = '\0';
        pRx = data_buffer;


        bitClear(UCSR0B, RXCIE0);
        bitSet(UCSR0B, UDRIE0);
        while (!bitRead(UCSR0A, UDRE0));
        UDR0 = '\0';
    } else {
        pRx++;
    }
}


ISR(USART_UDRE_vect) {
    if (*pTx == '\0') {
        pTx = data_buffer;
        bitClear(UCSR0B, UDRIE0);
        bitSet(UCSR0B, RXCIE0);
    } else {
        UDR0 = *pTx;
        pTx++;
    }
}


ISR(TIMER2_OVF_vect) {
    numOV2++ ;
}


ISR(INT0_vect) {
    buttonCheck = bitRead(PIND, 2) ;
    if(buttonCheck == 1) {
        _delay_ms(60) ;
        buttonCheck = bitRead(PIND, 2) ;
        if(buttonCheck == 1) {
            rotationMode++ ;
            if(rotationMode == 10) {
                rotationMode = 0 ;
            }
        }
    }
}


ISR(INT1_vect) {
    if(echoHighLow == 0) {
        TCNT2 = 0 ;
        numOV2 = 0 ;
        setPrescaler_tc2(6) ;  
    } if(echoHighLow == 1) {
        setPrescaler_tc2(0) ;
        echoTime = (TCNT2 + (numOV2 * 256)) * (1.0 / (16000000.0 / 256.0));
        distance = (echoTime * 343.0) / 2.0 * 100.0;
         setPrescaler_tc2(6) ;
        
        dtostrf(distance, 7, 3, valueDistance) ;
        valueDistance[7] = '\0' ;
        usart_tx_string(">a:") ;
        usart_tx_string(valueDistance) ;
        usart_transmit('\n') ;

        dtostrf(value_ADC, 4, 3, PhotoResValArray);
        PhotoResValArray[11] = '\0';
        usart_tx_string(">adc:");
        usart_tx_string(PhotoResValArray);
        usart_transmit('\n');
       
    }    
}


void ultraSonicSetup(void) {
    bitSet(DDRC, pinRed) ;
    set_tc2_mode(3) ;
    bitSet(TIMSK2, TOIE2) ;
    bitSet(DDRD, pinTrig) ;
    bitClear(DDRD, pinEcho) ;
    bitSet(EIMSK, INT1) ;
    bitSet(EICRA, ISC10) ;
}


void ultraSonicActive(void) {


    bitSet(DDRD, pinTrig) ;
    bitClear(PORTD, pinTrig) ;              
    bitClear(PORTD, pinEcho) ;


    bitSet(PORTD, pinTrig) ;
    _delay_us(10) ;
    bitClear(PORTD, pinTrig) ;


    echoHighLow = 0 ;
    while(!bitRead(PIND, pinEcho)) ;
    echoHighLow = 1 ;
    while(bitRead(PIND, pinEcho)) ;
}


void servoSetup(int prescalar, char mode) {
   
    bitSet(DDRB, pinServoPWM) ;
    bitSet(PORTD, pinServoButton) ;
    bitSet(TIMSK1, TOIE1) ;
    bitSet(TCCR1A, COM1B1) ;
    bitSet(EIMSK, INT0) ;
    bitSet(EICRA, ISC00) ;
    bitSet(EICRA, ISC01) ;
    set_tc1_mode(mode) ;
    setPrescaler_tc1(prescalar) ;
    OCR1A = 5000 ;
}


void servoActive(int rotationMode) {
    setPrescaler_tc1(3) ;
    if(rotationMode == 0) {
        if(loopCount == 8) {
            if(UltraDirection == 0 && UltraPOS <= 645) {
                OCR1B = UltraPOS;
                UltraPOS += 50;
                if (UltraPOS > 645) {
                    UltraPOS = 645;
                    UltraDirection = 1;
                }
            }else if(UltraDirection == 1 && UltraPOS >= 145) {
                OCR1B = UltraPOS;
                UltraPOS -= 50;
                if (UltraPOS < 145) {
                    UltraPOS = 145;
                    UltraDirection = 0;
                }
            }
            loopCount = 0 ;
        }
    }
   
    if(rotationMode == 1) {
        OCR1B = 145 ;
    }if(rotationMode == 2) {
        OCR1B = 270 ;
    }if(rotationMode == 3) {
        OCR1B = 395 ;
    }if(rotationMode == 4) {
        OCR1B = 520 ;
    }if(rotationMode == 5) {
        OCR1B = 645 ;
    }if(rotationMode == 6) {
        OCR1B = 520 ;
    }if(rotationMode == 7) {
        OCR1B = 395 ;
    }if(rotationMode == 8) {
        OCR1B = 270 ;
    }if(rotationMode == 9) {
        OCR1B = 145 ;
        loopCount = 0 ;
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


int setPrescaler_tc2(char option) {
   
    if(option == 0) {
        bitClear(TCCR2B, CS20) ;
        bitClear(TCCR2B, CS21) ;
        bitClear(TCCR2B, CS22) ;
        return 0 ;
    } if(option == 1) {          // T = 1/(16000000/1) = 6.25e-8
        bitSet(TCCR2B, CS20) ;
        return 1 ;
    } if(option == 2) {          // T = 1/(16000000/8) = 5e-7
        bitSet(TCCR2B, CS21) ;
        return 8 ;
    } if(option == 3) {
        bitSet(TCCR2B, CS20) ;
        bitSet(TCCR2B, CS21) ;
        return 32 ;
    } if(option == 4) {          // T = 1/(16000000/64) = 4e-6
        bitSet(TCCR2B, CS22) ;
        return 64 ;
    } if(option == 5) {
        bitSet(TCCR2B, CS20) ;
        bitSet(TCCR2B, CS22) ;
        return 128 ;
    } if(option == 6) {          // T = 1/(16000000/256) = 16e-6
        bitSet(TCCR2B, CS21) ;
        bitSet(TCCR2B, CS22) ;
        return 256 ;
    } if(option == 7) {          // T = 1/(16000000/1024) = 64e-6
        bitSet(TCCR2B, CS20) ;
        bitSet(TCCR2B, CS21) ;
        bitSet(TCCR2B, CS22) ;
        return 1024 ;
    } else {
        return 0 ;
    }
}


void set_tc2_mode(char mode) {
   
    if(mode == 0) {                // normal, 0xFF, immediate, Max
        bitClear(TCCR2A, WGM20) ;
        bitClear(TCCR2A, WGM21) ;
        bitClear(TCCR2B, WGM22) ;
    } if(mode == 1) {             // PWM, phase correct, 0xFF, TOP, BOTTOM
        bitSet(TCCR2A, WGM20) ;
    } if(mode == 2) {             // CTC, OCRA, immediate, max
        bitSet(TCCR2A, WGM21) ;
    } if(mode == 3) {             // Fast PWM, 0xFF, BOTTOM, MAX
        bitSet(TCCR2A, WGM20) ;
        bitSet(TCCR2A, WGM21) ;
    } if(mode == 4) {             // Reserved, -, -, -
        bitSet(TCCR2B, WGM22) ;
    } if(mode == 5) {             // PWM, phase correct, OCRA, TOP, BOTTOM
        bitSet(TCCR2B, WGM22) ;
        bitSet(TCCR0A, WGM20) ;
    } if(mode == 6) {             // Reserved, -, -, -
        bitSet(TCCR2B, WGM22) ;
        bitSet(TCCR2A, WGM21) ;
    } if(mode == 7) {             // Fast PWM, OCRA, BOTTOM, TOP
        bitSet(TCCR2A, WGM20) ;
        bitSet(TCCR2A, WGM21) ;
        bitSet(TCCR2B, WGM22) ;
    }
}




/*

#include <avr/io.h>
#include <stdlib.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#include <DHT.h>
#include <Adafruit_Sensor.h>

#include "usart.h"
#include "bit.h"

#define pinTrig PIND4
#define pinEcho PIND3

#define pinPhotocell PC0 

#define pinRed PC1
#define pinYellow PC2
#define pinBlue PC3
#define pinWhite PD5
#define pinBuzzer PC4

#define pinFanPWM PB3
#define pinDirection1 PB4
#define pinDirection2 PB5

#define pinServoPWM PB2
#define pinServoButton PIND2

#define BUFFER_SIZE 50
#define Type DHT11

int setPrescaler_tc0(char option) ;
void set_tc0_mode(char mode) ;
int setPrescaler_tc1(int option) ;
void set_tc1_mode(char mode) ;
int setPrescaler_tc2(char option) ;
void set_tc2_mode(char mode) ;

void ADCsetup(void) ;
int readADC(unsigned char pin) ;
void photosensorActive(float value_ADC) ; 

void ultraSonicSetup(void) ;
void ultraSonicActive(void) ;

void fanSetup(void) ;
void fanActive(int power) ;

void servoSetup(int prescalar, char mode) ;
void servoActive(int rotationMode) ;

void dht11Setup(void) ;
void checkEnvironment(float humidity, float temperature) ;
void sendDHT11values(void) ;

float value_ADC;
char PhotoResValArray[11];

volatile unsigned long numOV1 = 0 ;
volatile unsigned long numOV2 = 0 ;

float echoTime ;
float distance ;
char valueDistance[9] ;
int echoHighLow ;

int UltraPOS = 140 ;
int UltraDirection = 0 ;
int rotationMode = 0 ;
int loopCount = 0 ;
int buttonCheck ;

unsigned char data_buffer[BUFFER_SIZE];
unsigned char *pRx = data_buffer;
unsigned char *pTx = data_buffer;

int diditgethere = 0 ;

int sensePin = 19 ;
DHT HT(sensePin, Type) ;
float humidity ;
float temperature ;
char valueHumidity[10] ;
char valueTemperature[10] ;


ISR(TIMER1_OVF_vect) {
   numOV1++ ;
}


int main (void) {
    init() ;
    usart_init(103) ;
    ADCsetup() ;
    fanSetup() ;
    ultraSonicSetup() ;
    servoSetup(3, 15) ;
    dht11Setup() ;
   


    sei() ;


    while(1) {
       
        servoActive(rotationMode) ;
        ultraSonicActive() ;


        humidity = HT.readHumidity() ;
        temperature = HT.readTemperature() ;
        checkEnvironment(humidity, temperature) ;
        sendDHT11values() ;
        photosensorActive(value_ADC) ;
        
        loopCount++ ;
        _delay_ms(300) ;
       
    }
}

void ADCsetup(void) {

    bitSet(DDRD, pinWhite) ;

    bitSet(ADCSRA, ADPS2);   // prescaler
    bitSet(ADCSRA, ADPS1);   // prescaler
    bitSet(ADCSRA, ADPS0);   // prescaler
    bitClear(ADMUX, ADLAR);  // setting 0 for 10 bit
    bitSet(ADMUX, REFS0);    // Aref settings
    bitSet(ADCSRA, ADIE);    // Enable interrupt
                     
    bitSet(ADCSRA, ADEN);    // Enables ADC
    bitSet(ADCSRA, ADSC);    // Start conversion
}

int readADC(unsigned char pin) {
    int volt = ADCL;
    volt |= (ADCH << 8);
    return volt;
}

void photosensorActive(float value_ADC) {
    if(value_ADC >= 500) { 
        bitSet(PORTD, pinWhite) ;
    } else {
        bitClear(PORTD, pinWhite) ;
    }
}

ISR (ADC_vect) {
    value_ADC = readADC(pinPhotocell);
    bitSet(ADCSRA, ADSC);    // Start next conversion if needed
}


void fanSetup(void) {
    bitSet(DDRB, pinFanPWM) ;
    bitSet(PORTB, pinDirection1) ;      
    bitClear(PORTB, pinDirection2) ;
    bitSet(DDRB, pinDirection1) ;
    bitSet(DDRB, pinDirection2) ;
    bitSet(TCCR2A, COM2A1) ;
}


void fanActive(int power) {
    if(power == 0) {
        OCR2A = 0 ;
    } if(power == 1) {
        OCR2A = 255 ;
    }
}


void dht11Setup() {
    bitSet(DDRC, pinBlue) ;
    bitSet(DDRC, pinYellow) ;
    bitSet(DDRC, pinBuzzer) ;
    HT.begin() ;
    bitSet(UCSR0B, RXCIE0);
}


void checkEnvironment(float humidity, float temperature) {


    if(temperature >= 18 && temperature <= 25) {
        bitSet(PORTC, pinYellow) ;
    } if(humidity >= 75 && humidity <= 85) {
        bitSet(PORTC, pinBlue) ;
    } if(humidity > 85 || temperature > 25) {
        fanActive(1) ;
    } if(humidity < 75){
        bitClear(PORTC, pinBlue) ;
    } if(temperature < 18) {
        bitClear(PORTC, pinYellow) ;
    } if(humidity < 80 && temperature < 25) {
        fanActive(0) ;
    }
}


void sendDHT11values() {


    dtostrf(humidity, 7, 3, valueHumidity);
    valueHumidity[7] = '\0';  
    dtostrf(temperature, 7, 3, valueTemperature);
    valueTemperature[7] = '\0';
   
    snprintf((char *)data_buffer, BUFFER_SIZE, ">Humidity:%s\n>Temperature:%s\n", valueHumidity, valueTemperature);
   
    pTx = data_buffer;


    bitClear(UCSR0B, RXCIE0);
    bitSet(UCSR0B, UDRIE0);
}


ISR(USART_RX_vect) {
    *pRx = UDR0;


    if (*pRx == '\n') {
        *(pRx + 1) = '\0';
        pRx = data_buffer;


        bitClear(UCSR0B, RXCIE0);
        bitSet(UCSR0B, UDRIE0);
        while (!bitRead(UCSR0A, UDRE0));
        UDR0 = '\0';
    } else {
        pRx++;
    }
}


ISR(USART_UDRE_vect) {
    if (*pTx == '\0') {
        pTx = data_buffer;
        bitClear(UCSR0B, UDRIE0);
        bitSet(UCSR0B, RXCIE0);
    } else {
        UDR0 = *pTx;
        pTx++;
    }
}


ISR(TIMER2_OVF_vect) {
    numOV2++ ;
}


ISR(INT0_vect) {
    buttonCheck = bitRead(PIND, 2) ;
    if(buttonCheck == 1) {
        _delay_ms(60) ;
        buttonCheck = bitRead(PIND, 2) ;
        if(buttonCheck == 1) {
            rotationMode++ ;
            if(rotationMode == 10) {
                rotationMode = 0 ;
            }
        }
    }
}


ISR(INT1_vect) {
    if(echoHighLow == 0) {
        TCNT2 = 0 ;
        numOV2 = 0 ;
        setPrescaler_tc2(6) ;  
    } if(echoHighLow == 1) {
        setPrescaler_tc2(0) ;
        echoTime = (TCNT2 + (numOV2 * 256)) * (1.0 / (16000000.0 / 256.0));
        distance = (echoTime * 343.0) / 2.0 * 100.0;
         setPrescaler_tc2(6) ;
        
        dtostrf(distance, 7, 3, valueDistance) ;
        valueDistance[7] = '\0' ;
        usart_tx_string(">a:") ;
        usart_tx_string(valueDistance) ;
        usart_transmit('\n') ;

        dtostrf(value_ADC, 4, 3, PhotoResValArray);
        PhotoResValArray[11] = '\0';
        usart_tx_string(">adc:");
        usart_tx_string(PhotoResValArray);
        usart_transmit('\n');
       
    }    
}


void ultraSonicSetup(void) {
    bitSet(DDRC, pinRed) ;
    set_tc2_mode(3) ;
    bitSet(TIMSK2, TOIE2) ;
    bitSet(DDRD, pinTrig) ;
    bitClear(DDRD, pinEcho) ;
    bitSet(EIMSK, INT1) ;
    bitSet(EICRA, ISC10) ;
}


void ultraSonicActive(void) {


    bitSet(DDRD, pinTrig) ;
    bitClear(PORTD, pinTrig) ;              
    bitClear(PORTD, pinEcho) ;


    bitSet(PORTD, pinTrig) ;
    _delay_us(10) ;
    bitClear(PORTD, pinTrig) ;


    echoHighLow = 0 ;
    while(!bitRead(PIND, pinEcho)) ;
    echoHighLow = 1 ;
    while(bitRead(PIND, pinEcho)) ;
}


void servoSetup(int prescalar, char mode) {
   
    bitSet(DDRB, pinServoPWM) ;
    bitSet(PORTD, pinServoButton) ;
    bitSet(TIMSK1, TOIE1) ;
    bitSet(TCCR1A, COM1B1) ;
    bitSet(EIMSK, INT0) ;
    bitSet(EICRA, ISC00) ;
    bitSet(EICRA, ISC01) ;
    set_tc1_mode(mode) ;
    setPrescaler_tc1(prescalar) ;
    OCR1A = 5000 ;
}


void servoActive(int rotationMode) {
    setPrescaler_tc1(3) ;
    if(rotationMode == 0) {
        if(loopCount == 8) {
            if(UltraDirection == 0 && UltraPOS <= 645) {
                OCR1B = UltraPOS;
                UltraPOS += 25;
                if (UltraPOS > 645) {
                    UltraPOS = 645;
                    UltraDirection = 1;
                }
            }else if(UltraDirection == 1 && UltraPOS >= 145) {
                OCR1B = UltraPOS;
                UltraPOS -= 25;
                if (UltraPOS < 145) {
                    UltraPOS = 145;
                    UltraDirection = 0;
                }
            }
            loopCount = 0 ;
        }
    }
   
    if(rotationMode == 1) {
        OCR1B = 145 ;
    }if(rotationMode == 2) {
        OCR1B = 270 ;
    }if(rotationMode == 3) {
        OCR1B = 395 ;
    }if(rotationMode == 4) {
        OCR1B = 520 ;
    }if(rotationMode == 5) {
        OCR1B = 645 ;
    }if(rotationMode == 6) {
        OCR1B = 520 ;
    }if(rotationMode == 7) {
        OCR1B = 395 ;
    }if(rotationMode == 8) {
        OCR1B = 270 ;
    }if(rotationMode == 9) {
        OCR1B = 145 ;
        loopCount = 0 ;
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


int setPrescaler_tc2(char option) {
   
    if(option == 0) {
        bitClear(TCCR2B, CS20) ;
        bitClear(TCCR2B, CS21) ;
        bitClear(TCCR2B, CS22) ;
        return 0 ;
    } if(option == 1) {          // T = 1/(16000000/1) = 6.25e-8
        bitSet(TCCR2B, CS20) ;
        return 1 ;
    } if(option == 2) {          // T = 1/(16000000/8) = 5e-7
        bitSet(TCCR2B, CS21) ;
        return 8 ;
    } if(option == 3) {
        bitSet(TCCR2B, CS20) ;
        bitSet(TCCR2B, CS21) ;
        return 32 ;
    } if(option == 4) {          // T = 1/(16000000/64) = 4e-6
        bitSet(TCCR2B, CS22) ;
        return 64 ;
    } if(option == 5) {
        bitSet(TCCR2B, CS20) ;
        bitSet(TCCR2B, CS22) ;
        return 128 ;
    } if(option == 6) {          // T = 1/(16000000/256) = 16e-6
        bitSet(TCCR2B, CS21) ;
        bitSet(TCCR2B, CS22) ;
        return 256 ;
    } if(option == 7) {          // T = 1/(16000000/1024) = 64e-6
        bitSet(TCCR2B, CS20) ;
        bitSet(TCCR2B, CS21) ;
        bitSet(TCCR2B, CS22) ;
        return 1024 ;
    } else {
        return 0 ;
    }
}


void set_tc2_mode(char mode) {
   
    if(mode == 0) {                // normal, 0xFF, immediate, Max
        bitClear(TCCR2A, WGM20) ;
        bitClear(TCCR2A, WGM21) ;
        bitClear(TCCR2B, WGM22) ;
    } if(mode == 1) {             // PWM, phase correct, 0xFF, TOP, BOTTOM
        bitSet(TCCR2A, WGM20) ;
    } if(mode == 2) {             // CTC, OCRA, immediate, max
        bitSet(TCCR2A, WGM21) ;
    } if(mode == 3) {             // Fast PWM, 0xFF, BOTTOM, MAX
        bitSet(TCCR2A, WGM20) ;
        bitSet(TCCR2A, WGM21) ;
    } if(mode == 4) {             // Reserved, -, -, -
        bitSet(TCCR2B, WGM22) ;
    } if(mode == 5) {             // PWM, phase correct, OCRA, TOP, BOTTOM
        bitSet(TCCR2B, WGM22) ;
        bitSet(TCCR0A, WGM20) ;
    } if(mode == 6) {             // Reserved, -, -, -
        bitSet(TCCR2B, WGM22) ;
        bitSet(TCCR2A, WGM21) ;
    } if(mode == 7) {             // Fast PWM, OCRA, BOTTOM, TOP
        bitSet(TCCR2A, WGM20) ;
        bitSet(TCCR2A, WGM21) ;
        bitSet(TCCR2B, WGM22) ;
    }
}


*/

