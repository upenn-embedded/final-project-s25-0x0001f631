/* 
 * File:   newmain.c
 * Author: emil
 *
 * Created on March 31, 2025, 11:26 PM
 */

 #define F_CPU 16000000UL 
 #include <avr/io.h>
 #include <util/delay.h>
 #include <avr/interrupt.h>
 #include "uart.h"
 #include <stdlib.h>
 #include <stdbool.h>
 #include <math.h>
 
 #define IR_ADC_PIN PC0
 #define MINI_SERVO_PIN PC3
 #define ROT_SERVO_PIN PC4
 #define RAMP_SERVO_PIN PC5
 
 #define SS_PIN    PB2
 #define MOSI_PIN  PB3
 #define MISO_PIN  PB4
 #define SCK_PIN   PB5
 
 #define SOUND_SPEED_CM_PER_US 0.0343 // Speed of sound in cm/?s
 #define TIMER_PRESCALER 8
 #define US_TRIGGER PORTB1
 
 int state = 2;
 //0 = waiting for car placement
 //1 = spi communication and rotational servo
 //2 = calculate distance and ramp servo
 //3 = spinning up motors
 //4 = release car and wait
 
 //SERVO STUFF
 //current state of rotational servo
 float current_rot_degrees = 90.f;
 int current_servo;
 
 void disable_servos() {
     TCCR0A = 0;
     TCCR0B = 0;
     TIMSK0 &= ~(1 << OCIE0A);
     DDRC &= ~(1 << MINI_SERVO_PIN) & ~(1 << ROT_SERVO_PIN) & ~(1 << RAMP_SERVO_PIN);
 }
 void enable_timer0_pwm() {
     disable_servos();
 
     TCCR0B |= (1 << WGM02);
     TCCR0A &= ~(1 << WGM01);
     TCCR0A |= (1 << WGM00);
 
     TCCR0B |= (1 << CS02) | (1 << CS00); // 1024 prescale
     TCCR0B &= ~(1 << CS01) ;
 
     TCCR0A |= (1 << COM0A0);
     
     TIMSK0 |= (1 << OCIE0B);
     OCR0A = 156;
 }
 void enable_servo(int pin) {
     enable_timer0_pwm();
     DDRC |= (1 << pin);
     current_servo = pin;
 }
 ISR(TIMER0_COMPB_vect) {
     if (TCNT0 >= OCR0B) {
         PORTC |= (1 << current_servo);
     } else {
         PORTC &= ~(1 << current_servo);
     }
 //    PORTC ^= (1 << current_servo);
 }
 
 void servo_move(float degrees) {
     float desired_duty = 0.05f + 0.05f * degrees/180.f;
     OCR0B = OCR0A - (int) (OCR0A * desired_duty);
 }
 
 //SPI STUFF
 volatile uint8_t received_byte = 0;
 volatile int receiving_first = 1;
 volatile uint8_t current_middle = 0xFF;
 volatile uint8_t current_height = 0;
 
 void enable_spi_slave()
 {
     DDRB &= ~((1<<DDB3)|(1<<DDB5)|(1<<DDB2));  // MOSI (PB3), SCK (PB5), ~SS (PB2) as input
     DDRB |= (1<<DDB4);                         // MISO as output	
     SPCR0 = (1<<SPE) | (1<<SPIE);  // Enable SPI and SPI interrupt (slave mode)
     SPDR0 = 0x00;  // Initial response
 }
 
 void disable_spi_slave() {
     SPCR0 &= ~(1 << SPE) & ~(1 << SPIE);
     DDRB &= ~((1 << DDB2) | (1 << DDB3) | (1 << DDB4) | (1 << DDB5));
 }
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
 
 //IR SENSOR
 void IR_init() {
     DDRC &= ~(1 << PC0);
     ADMUX = (1 << REFS0);
     ADCSRA |= (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
 }
 
 //ULTRASONIC
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
 
 void US_deinit() {
     TIMSK1 &= ~(1 << ICIE1);
 }
 
 void trigger_sensor() {
    // For PB1 as trigger output
     PORTB |= (1 << US_TRIGGER);
     _delay_us(10);
     PORTB &= ~(1 << US_TRIGGER);
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
 
 //motor
 void spin_motors(float duty) {
     disable_servos();
     DDRD |= (1 << PD6);
     TCCR2B |= (1 << WGM22);
     TCCR2A &= ~(1 << WGM21);
     TCCR2A |= (1 << WGM20);
 
     TCCR2B &= ~(1 << CS22) & ~(1 << CS20);
     TCCR2B |= (1 << CS21) ;
 
     TCCR2A |= (1 << COM2A0);
     
     TIMSK2 |= (1 << OCIE2B);
     OCR2A = 255;
     
     OCR2B = (uint8_t)((1 - duty) * 255.f);
     sei();
 }
 
 ISR(TIMER2_COMPB_vect) {
     PORTD ^= (1 << PD6);
 }
 
 void kill_motors() {
     TCCR2A = 0;
     TCCR2B = 0;
     TIMSK2 &= ~(1 << OCIE2B);
     DDRD &= (1 << PD6);
 }
 
 
 int main(int argc, char** argv) {
     uart_init();
     IR_init();
     enable_timer0_pwm();
     sei();
     
     printf("Loop started.\n");
 
     //move servos to initial positions
     enable_servo(ROT_SERVO_PIN);
     servo_move(90);
     _delay_ms(1000);
     
     
     while (1) {
         if (state == 0) { //waiting for car to be placed
             ADCSRA |= (1 << ADSC);
             while (ADCSRA & (1 << ADSC));
             if (ADC == 0) {
               printf("ERR: NO POWER\n");  
             } else if (ADC < 650) {
                 //CAR IS PLACED
                 state = 1;
                 
                 enable_spi_slave();
                 enable_servo(ROT_SERVO_PIN);
                 printf("Car has been placed.\n");
                 printf("Waiting for spi signal...\n");
             } else {
 //                servo_move(-45);
                 _delay_ms(100);
             }
         } else if (state == 1) {
             if (current_middle == 0xFF) {
                 //pass
                 
             } else if (abs(current_middle - 128) <= 2) {
                 printf("Got to middle.\n");
                 disable_spi_slave();
                 disable_servos();
                 US_init();
                 state = 2;
             } else {
                 float rot_amount = ((int)current_middle - 128)/2.5f;
                 current_rot_degrees = current_rot_degrees - rot_amount;
                 servo_move(current_rot_degrees); 
                 current_middle = 0xFF;
                 printf("Waiting for spi signal...\n");
             }
         } else if (state == 2) {
 //            trigger_sensor();
             //TODO: ultrasonic
             US_deinit();
             enable_servo(RAMP_SERVO_PIN);
             //somewhere between -20 and 40
             servo_move(-20);
             state = 3;
         } else if (state == 3) {
             spin_motors(0.3f);
             printf("Spinning up motor...\n");
             _delay_ms(500);
             enable_timer0_pwm();
             enable_servo(MINI_SERVO_PIN);
             servo_move(90);
             _delay_ms(100);
             disable_servos();
             _delay_ms(500);
             kill_motors();
             state = 4;
             printf("Launched.\n");
             enable_timer0_pwm();
         } else if (state == 4) {
             enable_servo(MINI_SERVO_PIN);
             servo_move(-45);
              _delay_ms(1000);
              printf("Finished straightening servos.\n");
             state = 0;
             printf("Waiting for car placement.\n");
         }
         
     }
     return 0;
 }