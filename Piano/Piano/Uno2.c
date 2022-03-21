//
//  Uno2.c
//  Piano
//
//  Created by niloo gorji on 21/3/2022.
//

#include "Uno2.h"

#include <stdint.h>
#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
//
volatile uint8_t a = PIND;

void setup() {
  
  DDRD |= (1 << 2);
  DDRD |= (1 << 3);
  DDRD |= (1 << 4);
  DDRD |= (1 << 5);

  DDRB |= (1 <<0);
  DDRB |= (1 <<1);
  DDRB |= (1 << 2);
  DDRB |= (1 << 3);
  
  // PIN CHANGE
     
  //set PD6 to input
  DDRD &= ~(3<<6);

  //set PB4 to output and write initial to LOW
   DDRB |= (3<<4);
   PORTB &= ~(3<<4);

  //enable pin change interrupts on port D
  PCICR |= (1<<2);

  //enable PD6 as interrupts in PCMSK2
  PCMSK2 |= (3<<6);

  //set button pins initial value to LOW
  PORTD &= ~(3<<6);
  
  sei();
  
 
 
}



ISR(PCINT2_vect){
  
  if( (PIND - a) == 32 )
  {
    PORTB ^= (1<<5);
    PORTB &= ~(1<<4);
  }
  else if( (PIND - a) == 64 )
  {
    PORTB ^= (1<<4);
    PORTB &= ~(1<<5);
  }
 
}


void process(void) {

  PORTB |= (1 << 2);
  PORTB |= (1 << 3);
  _delay_ms(130);
  // changing
  PORTB |= (1 <<0);
  PORTB |= (1 <<1);
  PORTD |= (1 << 2);
  PORTD |= (1 << 3);
   _delay_ms(130);
  // resume
  PORTB &= ~(1 <<0);
  PORTB &= ~(1 <<1);
  PORTD &= ~(1 << 2);
  PORTD &= ~(1 << 3);
  
}

int main(){
  setup();
  for(;;)
  {
    process();
  }

}
