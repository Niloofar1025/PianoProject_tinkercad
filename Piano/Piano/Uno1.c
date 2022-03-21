//
//  Uno1.c
//  Piano
#include "Uno1.h"

#include <stdint.h>
#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>

#define MYF_CPU (16000000UL)
#define BAUD (9600)
#define MYUBRR (MYF_CPU/16/BAUD-1)

// include the library code:
#include <LiquidCrystal.h>
#define PB1 9
#define PB0 8
#define PB4 10
#define PB5 11
#define PB6  12
#define PB7 13

/*
 *  Setting data directions in a data direction register (DDR)
 *
 *
 *  Setting, clearing, and reading bits in registers.
 *    reg is the name of a register; pin is the index (0..7)
 *  of the bit to set, clear or read.
 *  (WRITE_BIT is a combination of CLEAR_BIT & SET_BIT)
 */
#define SET_BIT(reg, pin)            (reg) |= (1 << (pin))
#define CLEAR_BIT(reg, pin)            (reg) &= ~(1 << (pin))
#define WRITE_BIT(reg, pin, value)    (reg) = (((reg) & ~(1 << (pin))) | ((value) << (pin)))
#define BIT_VALUE(reg, pin)            (((reg) >> (pin)) & 1)
#define BIT_IS_SET(reg, pin)        (BIT_VALUE((reg),(pin))==1)

//threshold used for debouncing
#define THRESHOLD (1000)

//declare time out
const double time_out = 5;

// initialize the library by associating any needed LCD interface pin with the Arduino pin number it is connected to
uint8_t rs = 9, en = 8;
LiquidCrystal lcd(rs, en, 10, 11, 12, 13);

//Functions declaration
void setup(void);
void process(void);

//uart functions
void uart_init(unsigned int ubrr);
unsigned char uart_getchar(void);
void uart_putchar(unsigned char data);
void uart_putstring(unsigned char* s);

//to store button press count
uint16_t counter = 0;

// END function declarations

/*  ****** SET UP************ */

void setup(void)
{
  // initialise uart
    uart_init(MYUBRR);
  //================================//
  // Switch (Input) / LED (output)
  //================================//
  // Initalising the buttons as inputs
    DDRD &=   ~(1<<2) | ~(1<<3) |  ~(1<<4)| ~(1<<6) | ~(1<<7);
    DDRC &= ~(1<<2);
    // Initialising the LEDs as outputs
    DDRD |= (1<<2) | (1<<3)| (1<<4) | (1<<6) | (1<<7) | (1<<8) | (1<<9) | (1<<10);
    DDRC |= (1<<2) | (1 << 5);
  
  //================================//
  // Internal Pull Up RESISTOR
  //================================//
    DDRC &= ~(1 << DDC1);
    PORTC |= (1 << PC1);
  
  //===============
  //  set up ADC
  //===============
  //                      MUX channel 3
    ADMUX = (1 << REFS0) | 0b0011;
  //                     prescaler = 128
    ADCSRA = (1 << ADEN) | 0b111;
  
  //========================================
  // SET UP Implementing Timer Overflow ISR
  //=========================================

    // Timer 2 in normal mode, with pre-scaler 1024 ==> ~60Hz overflow.
    // Timer overflow on.
    TCCR2A = 0;
    TCCR2B = 5;
    TIMSK2 = 1;
  
  // Enable timer overflow, and turn on interrupts.
    sei();
  
      // Timer 1 in normal mode, with pre-scaler 1024 ==> ~60Hz overflow.
    // Timer overflow on.
    TCCR1A = 0;
    TCCR1B = 5;
    TIMSK1 = 1;

    // Enable timer overflow, and turn on interrupts.
    sei();
}

//=================================
// Implementing Timer Overflow ISR
//==================================
// count of overflows
volatile int overflow_counter = 0;
volatile uint8_t switch_counter = 0;
volatile uint8_t is_pressed = 0;
volatile int overflowCounter = 0;

ISR(TIMER2_OVF_vect)
{
  
    uint8_t b =  (((PINC) >> (3)) & 1);
    uint8_t mask = 0b00000011;
    switch_counter = ((switch_counter << 1) & mask) | b;
    if (switch_counter == mask) is_pressed = 1;
    else if (switch_counter == 0) is_pressed = 0;
    overflow_counter ++;
}



ISR(TIMER1_OVF_vect)
{
    overflowCounter ++;
}


void setup_lcd(void) {
  
  // Print a message to the LCD
  lcd.print("Press the button!");
  lcd.setCursor(0,1);
  _delay_ms(1000);
  lcd.clear();
   
}

/*  ****** PROCESS************ */

void process(void)
{
    void Caudio();
    void Daudio();
    void Eaudio();
    void Faudio();
    void Gaudio();
    void Aaudio();
    void C5audio();

 
   //start conversion
    ADCSRA |= (1 << ADSC);

    //wait for conversion to complete
    while(ADCSRA & (1 << ADSC)){
        _delay_us(10);
    }
  
  
  
    //======================//
    // Serial Communication
    //======================//
  
    //receiving buffer
    //unsigned char rx_buf;

    //define a character to sent
    unsigned char sent_char_C = 'C';
    unsigned char sent_char_D = 'D';
    unsigned char sent_char_E = 'E';
    unsigned char sent_char_F = 'F';
    unsigned char sent_char_G = 'G';
    unsigned char sent_char_A = 'A';
    
    //detect pressed switch on C5
    if (BIT_IS_SET(PINC,2))
    {
        //send serial data
        uart_putchar(sent_char_C);
    }

    //detect presed switch on D2
    if (BIT_IS_SET(PIND,2))
    {
        //send serial data
        uart_putchar(sent_char_D);
    }
  
    //detect presed switch on D3
    if (BIT_IS_SET(PIND,3))
    {
        //send serial data
        uart_putchar(sent_char_E);
    }
    
    //detect presed switch on D4
    if (BIT_IS_SET(PIND,4))
    {
   
        //send serial data
        uart_putchar(sent_char_F);
    }

    //detect presed switch on D5
    if (BIT_IS_SET(PIND,5))
    {
        //send serial data
        uart_putchar(sent_char_G);
    }
  
    //detect presed switch on D6
    if (BIT_IS_SET(PIND,6))
    {
        //send serial data
        uart_putchar(sent_char_A);
    }
  
    //detect presed switch on D7
    if (BIT_IS_SET(PIND,7))
    {
        //send serial data
        uart_putchar(sent_char_C);
    }
  
    //======//
    // LCD
    //======//
  
  
    // set up the LCD in 4-pin or 8-pin mode
      lcd.begin(16,2);

    if( (PINC & (1<<PC5))==(1<<PC5) )
    {
        //    lcd.clear();
         //   overflowCounter = 0;
            // lcd.clear();
            //  _delay_ms(100);
            // Print a message to the LCD
            
        lcd.print("Play piano!");
              //lcd.clear();
           
        _delay_ms(1000);
        _delay_ms(1000);
       
           
        
    }

    if( (PINC & (1<<PC2))==(1<<PC2) ) //if the Button is pressed
    {
        Caudio();
        _delay_ms(50);
             //print this
        lcd.setCursor(0, 1);
        lcd.print("DO");
        lcd.setCursor(15, 1);
        lcd.print("C");
        lcd.setCursor(7, 1);
        lcd.print("1");
             //delay for milliseconds
        _delay_ms(1000);
             //clear screen
        lcd.clear();
    }

    else if( (PIND & (1<<PD2))==(1<<PD2) ) //if the Button is pressed
    {
        Daudio();
        _delay_ms(50);
             //print this
        lcd.setCursor(0, 1);
        lcd.print("RE");
        lcd.setCursor(15, 1);
        lcd.print("D");
        lcd.setCursor(7, 1);
        lcd.print("2");
             //delay for milliseconds
        _delay_ms(1000);
             //clear screen
        lcd.clear();
    }

    else if( (PIND & (1<<PD3))==(1<<PD3) ) //if the Button is pressed
    {
        Eaudio();
        _delay_ms(1);
             //print this
        lcd.setCursor(0, 1);
        lcd.print("Mi");
        lcd.setCursor(15, 1);
        lcd.print("E");
        lcd.setCursor(7, 1);
        lcd.print("3");
             //delay for milliseconds
        _delay_ms(1000);
             //clear screen
        lcd.clear();
    }

    else if( (PIND & (1<<PD4))==(1<<PD4) ) //if the Button is pressed
    {
        Faudio();
        _delay_ms(1);
            //print this
        lcd.setCursor(0, 1);
        lcd.print("Fa");
        lcd.setCursor(15, 1);
        lcd.print("F");
        lcd.setCursor(7, 1);
        lcd.print("4");
            //delay for milliseconds
        _delay_ms(1000);
            //clear screen
        lcd.clear();
    }
    else if( (PIND & (1<<PD5))==(1<<PD5) ) //if the Button is pressed
    {
        Gaudio();
        _delay_ms(1);
            //print this
        lcd.setCursor(0, 1);
        lcd.print("So");
        lcd.setCursor(15, 1);
        lcd.print("G");
        lcd.setCursor(7, 1);
        lcd.print("5");
            //delay for milliseconds
        _delay_ms(1000);
            //clear screen
        lcd.clear();
    }
    else if( (PIND & (1<<PD6))==(1<<PD6) ) //if the Button is pressed
    {
        Aaudio();
        _delay_ms(1);
            //print this
        lcd.setCursor(0, 1);
        lcd.print("La");
        lcd.setCursor(15, 1);
        lcd.print("A");
        lcd.setCursor(7, 1);
        lcd.print("6");
            //delay for milliseconds
        _delay_ms(1000);
            //clear screen
        lcd.clear();
    }

    else if( (PIND & (1<<PD7))==(1<<PD7) ) //if the Button is pressed
    {
        C5audio();
        _delay_ms(1);
            //print this
        lcd.setCursor(0, 1);
        lcd.print("Si");
        lcd.setCursor(15, 1);
        lcd.print("B");
        lcd.setCursor(7, 1);
        lcd.print("7");
            //delay for milliseconds
        _delay_ms(1000);
        lcd.clear();
    }
            //END IF ELSE FOR LCD SCREEN

            //============================//
            // Interrupt-based debouncing
            //============================//

            //define a buffer to be sent
    char timer_buf[64];

            //compute elapsed time
    double time = ( overflowCounter * 256.0 + TCNT0 ) * 1024.0  / 16000000;

            //convert float to a string
    dtostrf(time, 7,3,timer_buf);

            // DEBUG SEND TIME_BUF (TIMER) TO CONSOLE
   //uart_putstring((unsigned char *) timer_buf);

    uart_putchar('\n');
    
            //==============================================================//
            // Polling - Counting the beat of each note (Metronome counter)
            //===============================================================/

            //define a buffer to be sent
            unsigned char speed_buf[64];

            //convert number to string

            itoa(counter, (char *)speed_buf,10);

            //detect pressed switch on C3
    if (BIT_IS_SET(PINC,3))
    {

        while ( BIT_IS_SET(PINC, 3) )
        {
            // Block until switch released.
        }
                //int counter_limit = 10;

                //increment count
            counter++;
    }
            //if (counter > counter_limit)
            //{
            //    break;
            //}

            //detect presed switch on C4
    if (BIT_IS_SET(PINC,4))
    {
            //send serial data
        uart_putstring(speed_buf);
        while ( BIT_IS_SET(PINC, 4) )
        {
            // Block until switch released.
        }
    }

    if (time > time_out)
    {
        overflowCounter = 0;
        process();
    }
    
        
       
}
  

  /*  ****** Unique Audio for each button ************ */

  //============================================//
  //  Software based PWM by using 50% duty cycle
  //============================================//
void playTone(long duration, int freq) {
    duration *= 1000;
    int period = (1.0 / freq) * 1000000;
    long elapsed_time = 0;
        while (elapsed_time < duration)
        {
          SET_BIT(PORTC,0); // BIT BANGING - Software based PWM
          _delay_us(period / 2);//delay for period/2 // DUTY cycle of 50%
          CLEAR_BIT(PORTC,0);
          _delay_us(period / 2);//delay for period/2 // DUTY cycle of 50%
          elapsed_time += (period);
        }
}
void Caudio(){
      SET_BIT(PORTC,2);
      playTone(500,261.63);
      _delay_ms(500);
      CLEAR_BIT(PORTC,2);
  
}
void Daudio(){
    SET_BIT(PORTD,2);
    playTone(500,293.66);
    _delay_ms(500);
    CLEAR_BIT(PORTD,2);
}
void Eaudio(){
      SET_BIT(PORTD,3);
      playTone(500,329.63);
      _delay_ms(500);
      CLEAR_BIT(PORTD,3);
}
void Faudio(){
      SET_BIT(PORTD,4);
      playTone(500,349.23);
      _delay_ms(500);
      CLEAR_BIT(PORTD,4);
}
void Gaudio(){
      SET_BIT(PORTD,5);
      playTone(500,392.0);
      _delay_ms(500);
      CLEAR_BIT(PORTD,5);
}
void Aaudio(){
      SET_BIT(PORTD,6);
      playTone(500,440.0);
      _delay_ms(500);
      CLEAR_BIT(PORTD,6);
}
void C5audio(){
      SET_BIT(PORTD,7);
      playTone(500,523.25);
      _delay_ms(500);
      CLEAR_BIT(PORTD,7);
}

/*  ****** serial definitions ************ */

void uart_init(unsigned int ubrr){
  UBRR0H = (unsigned char)(ubrr>>8);
  UBRR0L = (unsigned char)(ubrr);
  UCSR0B = (1 << RXEN0) | (1 << TXEN0);
  UCSR0C = (3 << UCSZ00);
}

void uart_putchar(unsigned char data){
    
    while (!( UCSR0A & (1 <<UDRE0))); /* Wait for empty transmit buffer */
    UDR0 = data;            /* Put data into buffer, sends the data */
}

// Transmit a string
void uart_putstring(unsigned char* s)
{
    // transmit character until NULL is reached
    while(*s > 0) uart_putchar(*s++);
}

int main(void)
{

  setup();
  setup_lcd();
  uint8_t prevState = 0;
  for (; ;){
    if (is_pressed != prevState) {
       prevState = is_pressed;
      
      }
     _delay_ms(100);
      process();
   
  }

}
