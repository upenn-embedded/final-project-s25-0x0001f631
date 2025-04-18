
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
////    main.c
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
/*
#define F_CPU 16000000UL

#include <stdio.h>
#include <stdlib.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "./lib/uart.h"
#include <stdbool.h>

#define SOUND_SPEED_CM_PER_US 0.0343 // Speed of sound in cm/?s
#define TIMER_PRESCALER 8

#define US_TRIGGER PORTB1

volatile uint32_t end_time;
volatile bool capturing = false;
volatile uint8_t discrete = 0;
volatile uint8_t duty;

void US_init() {
    //trigger output
    DDRB |= (1 << DDB1);
    
    //Echo input
    DDRB &= ~(1 << DDB0);
    
    // prescale 8 
    TCCR1B |= (1 << ICES1) | (1 << CS11); // Rising edge
    TIMSK1 |= (1 << ICIE1); //ICIE
}

void PWM_init() {
    //pwm signal. We will keep frequency the same, so we will use fast pwm mode.
    DDRD |= (1 << PD6);
    
    TCCR0A = (1 << WGM01) | (1 << WGM00) |
             (1 << COM0A1);
    //no prescaling, we will get 16MHz/255 = 62kHz
    TCCR0B |= (1 << CS00);
     
    TIFR0 |= (1 << OCF0A);
    //OCR0A will be adjusted based on US measurements
    OCR0A = 200;

    TCCR0A |= (1 << COM0B1);
}

void IR_init() {
    
    //ADC setup
    PRR0 &= ~(1 << PRADC);
    ADMUX |= (1 << REFS0);
    ADMUX &= ~(1 << REFS1);
    ADCSRA |= (1 << ADPS0) | (1 << ADPS1) | (1 << ADPS2);
    ADMUX &= ~(1 << MUX0) & ~(1 << MUX1) & ~(1 << MUX2) & ~(1 << MUX3);
    ADCSRA |= (1 << ADATE);
    ADCSRB &= ~(1 << ADTS0) & ~(1 << ADTS1) & ~(1 << ADTS2);
    DIDR0 |= (1 << ADC0D);
    ADCSRA |= (1 << ADEN) | (1 << ADSC);
    
    //mini servo timer 3
    DDRD |= PD0;
    //PWM mode Phase correct OCR3A TOP BOTTOM
    TCCR3B |= (1 << WGM33);
    TCCR3B &= ~(1 << WGM32);
    TCCR3A |= (1 << WGM31) | (1 << WGM30);
    //prescale 64
    TCCR3B &= ~(1 << CS32); 
    TCCR3B |= (1 << CS31) | (1 << CS30);
    TCCR3A |= (1 << COM3A0);
}

void servo_init() {
    DDRD |= (1 << PD1);
    TCCR4A = (1 << COM4A1) | (1 << WGM41);
    TCCR4B = (1 << WGM43) | (1 << WGM42);
    TCCR4B |= (1 << CS41);
    ICR4 = 4000;
    OCR4A = 2000;
}

void mini_servo_move(int degrees) {
    float desired_ms = 1.f + 4*degrees/180.f;
    int val = (int) ((250 * desired_ms - 1)/4.f);
    val *= 256; //we have 16bit reg now i think
    OCR3A = min(val, 65535);
}

#define SS_PIN    PB2
#define MOSI_PIN  PB3
#define MISO_PIN  PB4
#define SCK_PIN   PB5

void SPI_MasterInit(void) {
    // Set MOSI, SCK, and SS as output; MISO as input
    DDRB |= (1 << MOSI_PIN) | (1 << SCK_PIN) | (1 << SS_PIN);
    DDRB &= ~(1 << MISO_PIN); // MISO is input

    // Enable SPI, set as master, and set clock rate fosc/16
    SPCR0 = (1 << SPE) | (1 << MSTR) | (1 << SPR0);
}

void trigger_sensor() {
   // For PB1 as trigger output
PORTB |= (1 << US_TRIGGER);
_delay_us(10);
PORTB &= ~(1 << US_TRIGGER);
}

void move_vert_servo(int degrees) {
    float desired_ms = 1.f + 4*degrees/180.f;
    ICR4 = (int) (2000*desired_ms);
    OCR4A = ICR4/2;
}

ISR(TIMER1_CAPT_vect) {
    if (!capturing) {
        TCNT1 = 0; //restart timer here, we know the starting time is 0
        TCCR1B &= ~(1 << ICES1);  //falling edge detection
        capturing = true;
    } else {
        end_time = ICR1;  // Capture falling edge time
        TCCR1B |= (1 << ICES1);  // Switch back to rising edge detection
        uint32_t pulse_width = end_time;  // end_time - 0 = end_time
        uint32_t distance = (pulse_width * SOUND_SPEED_CM_PER_US) / (2 * (16000000 / TIMER_PRESCALER / 1000000)); 
        OCR0A = 30 + (28 * distance) / 50;
        printf("Distance: %u cm |  ", distance);
        printf("OCR0A: %u | \n", OCR0A);
        capturing = false;
    }
}


int main(void) {
    uart_init(); 
    US_init();
    PWM_init();
    SPI_MasterInit();
    
    sei();

    while (1) {
      trigger_sensor();
    }
    return 0;
}*/


#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "./lib/uart.h"
#include <stdlib.h>

void initialize_ir() {
    //initialize ADC for IR sensor
    PRR0 &= ~(1 << PRADC);
    ADMUX |= (1 << REFS0);
    ADMUX &= ~(1 << REFS1);
    ADCSRA |= (1 << ADPS0) | (1 << ADPS1) | (1 << ADPS2);
    ADMUX &= ~(1 << MUX0) & ~(1 << MUX1) & ~(1 << MUX2) & ~(1 << MUX3);
    ADCSRA |= (1 << ADATE);
    ADCSRB &= ~(1 << ADTS0) & ~(1 << ADTS1) & ~(1 << ADTS2);
    DIDR0 |= (1 << ADC0D);
    ADCSRA |= (1 << ADEN) | (1 << ADSC);
    //initialize LED for IR sensor
    DDRB |= (1 << PB0);
    PORTB |= (1 << PB0);
    //mini servo on timer 0
    DDRD |= (1 << DDD6);
    TCCR0B |= (1 << WGM02);
    TCCR0A &= ~(1 << WGM01);
    TCCR0A |= (1 << WGM00);
    TCCR0B &= ~(1 << CS02); // 64 prescale
    TCCR0B |= (1 << CS01) | (1 << CS00);
    OCR0A = 0;
    TCCR0A |= (1 << COM0A0);
}
void initialize_servos() {
    DDRB |= (1 << PB1); // Set OC1A (PB1) as output
    TCCR1A = (1 << COM1A1) | (1 << WGM11);
    TCCR1B = (1 << WGM13) | (1 << WGM12);
    TCCR1B |= (1 << CS11); //prescale 8
    ICR1 = 4000;     // TOP value for 2ms period
    OCR1A = 2000;    // 50% duty cycle
}
void move_mini_servo(int degrees) {
    float desired_ms = 1.f + 4*degrees/180.f;
    //16000000/prescale/(4*OCR0A + 1) = freq
    int val = (int) ((250 * desired_ms - 1)/4.f);
    if (val > 255) {
        OCR0A = 255;
    } else {
        OCR0A = val;
    }
}

volatile uint8_t received_byte = 0;
volatile int receiving_first = 1;
volatile uint8_t current_middle = 0;
volatile uint8_t current_height = 0;

void SPI_init_slave()
{
    DDRB &= ~((1<<DDB3)|(1<<DDB5)|(1<<DDB2));  // MOSI (PB3), SCK (PB5), ~SS (PB2) as input
    DDRB |= (1<<DDB4);                         // MISO as output	
    SPCR0 = (1<<SPE) | (1<<SPIE);  // Enable SPI and SPI interrupt (slave mode)
    sei();                         // Enable global interrupts
    SPDR0 = 0x00;  // Initial response
}
void SPI_init_master()
{
    // Set MOSI, SCK, and SS as output; MISO as input
    DDRB |= (1<<DDB3) | (1<<DDB5) | (1<<DDB2);  // MOSI, SCK, SS
    DDRB &= ~(1<<DDB4);  // MISO as input
    // Enable SPI, Master mode, set clock rate fosc/16 (adjustable)
    SPCR0 = (1<<SPE) | (1<<MSTR) | (1<<SPR0);  // SPE: SPI Enable, MSTR: Master, SPR0: clk/16
    SPSR0 &= ~(1<<SPI2X);  // Clear SPI2X for fosc/16 (set it for fosc/8 if desired)
}
uint8_t SPI_ControllerRx()
{
    SPDR0 = 0xFE;  // Send dummy byte to generate clock
    while (!(SPSR0 & (1<<SPIF)));  // Wait for transmission complete
    return SPDR0;  // Read received byte from slave
}
int main(int argc, char** argv) {
    SPI_init_slave();
//    initialize_ir();
//    initialize_servos();
//
    uart_init();
    sei();
    while(1) {
//        received_byte = SPI_ControllerRx();
//        printf("Received: %d \n", received_byte);
//        _delay_ms(100);
    }
//
//    while (1) {
//        move_vert_servo(0);
////        if (ADC < 450) {
////            printf("CAR PLACED\n");
////            move_mini_servo(0);
////            _delay_ms(2000);
////        } else {
////            move_mini_servo(90);
////            printf("%d\n", ADC);
////        }
//        _delay_ms(1000);
//        move_vert_servo(90);
//        _delay_ms(1000);
//        move_vert_servo(180);
//        _delay_ms(1000);
//    }
    return 0;
}
//ISR(SPI0_STC_vect)
//{
//    received_byte = SPDR0;  // Read byte from master
//    SPDR0 = 0x1E;  // Load response for next transfer
//    printf("SENT 0x1E, received: %d", received_byte);
//}
ISR(SPI0_STC_vect)
{
    received_byte = SPDR0;       // Read the byte from the master
    SPDR0 = 0x01;   // Load response for next transmission
    if (receiving_first) {
        current_middle = received_byte;
        printf("MIDDLE RECEIVED: %x\n", current_middle);
        receiving_first = 0;
    } else {
        current_height = received_byte;
        printf("HEIGHT RECEIVED: %x \n", current_height);
        receiving_first = 1;
    }
}