/*
Based on 
https://www.youtube.com/user/DrakerDG

Overclockers version:
Removed pot. buttons, 3 lights
Added ttp223 capicitive touch sensor

*/


// Requires headers for AVR defines and ISR function
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>    // Sleep Modes
#include <avr/power.h>    // Power management


// Define pins for switch the LED, plus the chosen interrupt for reacting to
#define INTERRUPT_PIN PCINT3  // This is PB1 per the schematic
#define INT_PIN PB3           // Interrupt pin of choice: PB1 (same as PCINT1) - Pin 6

#define  LED_RED PB2
#define  LED_YLW PB1
#define  LED_GRN PB0
#define BUZZER PB4

/*
   Alias for the ISR: "PCINT_VECTOR" (Note: There is only one PCINT ISR.
   PCINT0 in the name for the ISR was confusing to me at first,
   hence the Alias, but it's how the datasheet refers to it)
*/
#define PCINT_VECTOR PCINT0_vect      // This step is not necessary - it's a naming thing for clarity

// FYI: Variables used within ISR must be declared Volatile.
// static volatile byte LEDState;

// The setup function runs only once when the ALU boots
void setup() {

  for (int i = 0; i < 5; i++) {
    pinMode(i, OUTPUT);
    digitalWrite(i, LOW);
  }

  // Code here is the key piece of configuring and enabling the interrupt
  cli();                            // Disable interrupts during setup

  PCMSK |= (1 << INTERRUPT_PIN);    // Enable interrupt handler (ISR) for our chosen interrupt pin (PCINT1/PB1/pin 6)

  GIMSK |= (1 << PCIE);             // Enable PCINT interrupt in the general interrupt mask

  pinMode(INT_PIN, INPUT_PULLUP);   // Set our interrupt pin as input with a pullup to keep it stable

  sei();                            //last line of setup - enable interrupts after setup

}


void loop() {
  melodyY(1, 0, 15);
  melodyY(2, 15, 22);
  melodyY(1, 4, 15);
  melodyY(2, 15, 22);
  melodyY(1, 4, 15);
  melodyY(2, 15, 22);
  melodyY(1, 22, 50);
}


const int MeSderM[] [3] = {{72 , 6, LED_YLW}, {70 , 6, LED_GRN}, {69 , 6, LED_RED}, {65 , 6, LED_GRN}, {67 , 3, LED_GRN}, {67 , 6, LED_GRN}, {74 , 6, LED_YLW}, {72 , 3, LED_GRN}, {70 , 3, LED_RED}, {69 , 3, LED_GRN}, {69 , 6, LED_GRN}, {69 , 6, LED_GRN}, {72 , 3, LED_YLW}, {70 , 6, LED_GRN}, {69 , 6, LED_RED}, {67 , 3, LED_GRN}, {67 , 6, LED_GRN}, {82 , 6, LED_YLW}, {81 , 6, LED_GRN}, {82 , 6, LED_YLW}, {81 , 6, LED_RED}, {82 , 6, LED_YLW}, {70 , 6, LED_GRN}, {70 , 6, LED_GRN}, {70 , 6, LED_GRN}, {70 , 6, LED_GRN}, {74 , 6, LED_YLW}, {74 , 6, LED_YLW}, {74 , 6, LED_YLW}, {74 , 6, LED_YLW}, {72 , 6, LED_RED}, {72 , 6, LED_RED}, {72 , 6, LED_RED}, {72 , 6, LED_RED}, {77 , 6, LED_GRN}, {77 , 6, LED_GRN}, {77 , 6, LED_YLW}, {77 , 6, LED_RED}, {79 , 6, LED_YLW}, {79 , 6, LED_YLW}, {79 , 6, LED_YLW}, {79 , 6, LED_YLW}, {79 , 6, LED_YLW}, {79 , 6, LED_YLW}, {79 , 6, LED_YLW}, {79 , 6, LED_YLW}, {79 , 6, LED_YLW}, {79 , 6, LED_YLW}, {79 , 6, LED_YLW}, {79 , 6, LED_YLW}};


int OctX = 1.5;

void melodyY(int RepX, int iniX, int endX) {
  for (int RepI = 0; RepI < RepX; RepI++) {
    for (int thisNote = iniX; thisNote < endX; thisNote++) {
      int noteDuration = 1000 / MeSderM[thisNote][1];
      digitalWrite(MeSderM[thisNote][2], HIGH);
      float NteX = MeSderM[thisNote][0] + OctX * 12 - 69;
      NteX = pow(2, NteX / 12) * 440;
      String str1 = "Eighth: " + String(4 + OctX) + "; Freq: " + String(NteX, 3);
      tone(BUZZER, NteX, noteDuration);
      int pauseBetweenNotes = noteDuration * 1.30;
      delay(pauseBetweenNotes * 2 / 3);
      digitalWrite(MeSderM[thisNote][2], LOW);
      delay(pauseBetweenNotes / 3);
      noTone(BUZZER);
    }
  }
}

// This is the interrupt handler called when there is any change on the INT_PIN
// ISR is defined in the headers - the ATtiny85 only has one handler
ISR(PCINT_VECTOR)
{
  if ( digitalRead(INT_PIN) == HIGH ) {

  } else if ( digitalRead(INT_PIN) == LOW ) {
    for (int i = 0; i < 5; i++) {
      digitalWrite(i, LOW);
    }
    noTone(BUZZER);
    goToSleep ();
  }
}

void goToSleep () {

  GIMSK |= _BV(PCIE);                     // Enable Pin Change Interrupts
  PCMSK |= _BV(PCINT3);                   // Use PB3 as interrupt pin
  ADCSRA &= ~_BV(ADEN);                   // ADC off
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);    // replaces above statement

  sleep_enable();                         // Sets the Sleep Enable bit in the MCUCR Register (SE BIT)
  sei();                                  // Enable interrupts
  sleep_cpu();                            // sleep

  cli();                                  // Disable interrupts
  //  PCMSK &= ~_BV(PCINT3);                  // Turn off PB3 as interrupt pin: disabled to keep it on
  sleep_disable();                        // Clear SE bit
  ADCSRA |= _BV(ADEN);                    // ADC on

  sei();                                  // Enable interrupts
} // sleep
