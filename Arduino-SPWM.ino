#include <stdlib.h>

#define OUTPUT_FREQ 50
#define SWITCHING_FREQ OUTPUT_FREQ * 400

// Structure for SPWM interrupt state and info 
struct SPWM_Data {
  unsigned int steps;
  unsigned int *pwm_values = NULL;
  unsigned int quadrant = 0;
  unsigned int step = 0;
};

// Globals
volatile SPWM_Data spwm_data;

// PWM Setup
void set_SPWM1(float switching_frequency, float output_frequency, float scale_factor) {
  // switch outputs to off
  digitalWrite(9, LOW);
  digitalWrite(10, LOW);
  DDRB |= _BV(DDB1) | _BV(DDB2);
  // disable interrupts while we setup...
  noInterrupts();
  // Calculate top value for required frequency assuming no prescaling
  float count = 16000000 / switching_frequency - 1;
  // Work out if we need to set the prescaler
  byte cs12 = 0;
  if (count > 65535) {
    count = count / 1024;
    cs12 = 1;
  }
  // Initialise SPWM state
  spwm_data.step = 0;
  spwm_data.quadrant = 0;
  // Work out 'on' count array for ocr (only need to do it for 1 quadrant)
  spwm_data.steps = switching_frequency/output_frequency/4;
  free(spwm_data.pwm_values); 
  spwm_data.pwm_values = (unsigned int*) calloc(spwm_data.steps, sizeof(unsigned int));
  for(int i = 0; i < spwm_data.steps; i++){
    spwm_data.pwm_values[i] = count * sin(2 * M_PI * i / spwm_data.steps / 4)  * scale_factor ;
  }
  // Initialise control registers (always do this first)
  TCCR1A = 0;
  TCCR1B = 0;
  // set TOP
  ICR1 = count;
  // Set On time for output A
  OCR1A = 0;
  // Set On time for output B
  OCR1B = 0;
  // set Fast PWM mode using ICR1 as TOP
  TCCR1A |= _BV(WGM11);
  TCCR1B |= _BV(WGM12) | _BV(WGM13);
  // START the timer with no prescaler or 1024 prescaler
  TCCR1B |= _BV(CS10) | (cs12 << CS12);
  // switch on outputs
  DDRB |= _BV(DDB1) | _BV(DDB2); 
  // Enable overflow interrupt
  TIMSK1 |= _BV(TOIE1);
  // enable all interrupts
  interrupts();
  // output debug info if required
  Serial.println(count);
  Serial.println(spwm_data.steps);
  for(int i = 0; i < spwm_data.steps; i++){
    Serial.println(spwm_data.pwm_values[i]);
  }    
}

// Interrupt routine
ISR(TIMER1_OVF_vect){
  //increment step..
  spwm_data.step++;
  // if it overflows...
  if(spwm_data.step >= spwm_data.steps){
    // reset    
    spwm_data.step = 0;
    // increment quadrant and reset if it overflows
    spwm_data.quadrant++;
    if(spwm_data.quadrant >= 4){
      spwm_data.quadrant = 0;      
    }
    // Switch off one of the comparators depending on quadrant
    byte tcccr1a = TCCR1A;  
    if(spwm_data.quadrant < 2){
      OCR1B = 0;
      tcccr1a &= ~_BV(COM1B1);
      tcccr1a |= _BV(COM1A1);
    } else {
      OCR1A = 0;
      tcccr1a &= ~_BV(COM1A1);
      tcccr1a |= _BV(COM1B1);      
    }
    TCCR1A = tcccr1a;
  }
  // get index for PWM value depending on quadrant
  unsigned int index;
  if(spwm_data.quadrant % 2){
    index = spwm_data.steps - spwm_data.step - 1;
  } else {
    index = spwm_data.step;
  }
  // Set appropriate comparator depending on quadrant
  if(spwm_data.quadrant < 2){
    OCR1A = spwm_data.pwm_values[index];
  } else {
    OCR1B = spwm_data.pwm_values[index];
  }    
}

void setup() {
  // Initialise serial port
  Serial.begin(9600);
  // Setup SPWM...
  set_SPWM1(SWITCHING_FREQ, OUTPUT_FREQ, 1);
}

void loop() {
  // Nothing to do...
}
