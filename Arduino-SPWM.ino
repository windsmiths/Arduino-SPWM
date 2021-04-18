#include <stdlib.h>
#include <EEPROM.h>
#include <LiquidCrystal_I2C.h>

// Configuration Options
#define NOMINAL_FREQ 50
#define STROBE_FREQ 50
#define STROBE_ON_MS 0.2
#define SWITCHING_FREQ 20000
#define SCALE_FACTOR 1.0

// Board related Constants
#define TIMER1PINA 9
#define TIMER1PINB 10
#define STROBEPIN 3
#define UP_PIN 4
#define DOWN_PIN 5
#define RPM_TOGGLE_PIN 2
#define LED_45RPM 13

// Other constants
#define APP_ID "SPWM"
#define SPEED_CORR_DPS 4

// Structure for SPWM interrupt state and info 
struct SPWM_Data {
  unsigned int steps;
  int *pwm_values = NULL;
  unsigned int step = 0;
  unsigned int strobe_counter = 0;  
  unsigned int strobe_off_value = 0;
  unsigned int strobe_limit = 0;
};

// Structure for EEProm Data
struct EEPromData{
  char app_id[10] = {APP_ID};
  byte version = 0;  
  byte rpm = 33;
  float speed_correction = 1.0;
};

struct PinStates{
  byte up_pin = 1;
  byte down_pin = 1;
  byte rpm_pin = 1;
};

// Globals
EEPromData settings;
PinStates pin_states;
volatile SPWM_Data spwm_data;
LiquidCrystal_I2C lcd(0x27,16,2);

//EEPROM
EEPromData load_settings(){
  EEPromData default_settings;
  EEPromData settings;
  EEPROM.get(0, settings);
  if (!strcmp(settings.app_id, APP_ID)) {
    Serial.println("Loading settings from EPROM...");
    // EEPROM has a structure we recognise
    if (settings.version < default_settings.version){
      // do any version specific updates here
    }  
  } else {
    // EEPROM is either uninitialised or has been used for something else...
    Serial.println("Using Default Settings...");
    settings = default_settings;
    save_settings(settings);
  }
  Serial.print("App ID: "); Serial.println(settings.app_id);
  Serial.print("Version: "); Serial.println(settings.version);
  return settings;
}

void save_settings(EEPromData settings){
  EEPROM.put(0, settings);
}

// PWM Setup
float set_SPWM1(float switching_frequency, float nominal_frequency, float speed_correction, float scale_factor) {
  // switch outputs to off
  digitalWrite(TIMER1PINA, LOW);
  digitalWrite(TIMER1PINB, LOW);
  digitalWrite(STROBEPIN, LOW); 
  // set directions   
  pinMode(STROBEPIN, OUTPUT);  
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
  // Work out 'on' count array for ocr 
  unsigned int nominal_steps = floor(switching_frequency / nominal_frequency + 0.5);
  spwm_data.steps = floor(switching_frequency / (nominal_frequency * speed_correction) + 0.5);
  float actual_correction = (float) nominal_steps / (float) spwm_data.steps;
  free(spwm_data.pwm_values); 
  spwm_data.pwm_values = (int*) calloc(spwm_data.steps, sizeof(int));
  for(int i = 0; i < spwm_data.steps; i++){
    spwm_data.pwm_values[i] = floor(0.5 + count * (0.5 + sin(2.0 * M_PI * i / spwm_data.steps)  * scale_factor / 2.0)) ;
  }
  // Work out strobe values
  spwm_data.strobe_limit = SWITCHING_FREQ/STROBE_FREQ;
  spwm_data.strobe_off_value = STROBE_ON_MS / 1000.0 * SWITCHING_FREQ;
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
  // set A and B as complementary comparators
  TCCR1A |= _BV(COM1A1);
  TCCR1A |= _BV(COM1B1);
  TCCR1A |= _BV(COM1B0);
  // START the timer with no prescaler or 1024 prescaler
  TCCR1B |= _BV(CS10) | (cs12 << CS12);
  // switch on outputs
  DDRB |= _BV(DDB1) | _BV(DDB2); 
  // Enable overflow interrupt
  TIMSK1 |= _BV(TOIE1);
  // enable all interrupts
  interrupts();
  // output debug info if required
  Serial.print("count: "); Serial.println(count);
  Serial.print("steps: "); Serial.println(spwm_data.steps);
  Serial.print("strobe limit: "); Serial.println(spwm_data.strobe_limit);
  Serial.print("strobe count: "); Serial.println(spwm_data.strobe_off_value);
//   for(int i = 0; i < spwm_data.steps; i++){
//     Serial.print("Value "); Serial.print(i); Serial.print(": "); Serial.println(spwm_data.pwm_values[i]);
//   }  
  return actual_correction;  
}

// Interrupt routine
ISR(TIMER1_OVF_vect){
  // strobe...
  spwm_data.strobe_counter++;
  if (spwm_data.strobe_counter >= spwm_data.strobe_limit) {
    spwm_data.strobe_counter = 0; 
    digitalWrite(STROBEPIN, HIGH); 
  } else {
    if (spwm_data.strobe_counter == spwm_data.strobe_off_value)
      digitalWrite(STROBEPIN, LOW); 
  }
  // SPWM
  //increment step..
  spwm_data.step++;
  // check for overflow...
  if(spwm_data.step >= spwm_data.steps){
    spwm_data.step = 0;
  }
  // Set comparators
  OCR1A = spwm_data.pwm_values[spwm_data.step];
  OCR1B = spwm_data.pwm_values[spwm_data.step];
}

void update_SPWM1(byte rpm, int delta){
  float nominal_frequency = NOMINAL_FREQ;
  float speed_correction = settings.speed_correction;
  bool save = false;
  // Only apply speed correction deltas in 33 rpm mode  
  if (delta != 0 and rpm == 33){
    speed_correction = speed_correction * (float) (spwm_data.steps + delta) / (float) spwm_data.steps;
  }
  // Adjust nominal frequency for 45 rpm
  if (rpm == 45){
    nominal_frequency = nominal_frequency * 45.0 / (100.0 / 3.0);
  } 
  // Apply settings
  float actual_correction = set_SPWM1(SWITCHING_FREQ, nominal_frequency, speed_correction, SCALE_FACTOR);

  if (actual_correction != settings.speed_correction and rpm == 33){
    settings.speed_correction = actual_correction; 
    save = true;
  }  
  if (rpm != settings.rpm){
    settings.rpm = rpm;
    save = true;
  }   
  if (save) save_settings(settings);
  Serial.print("rpm: "); Serial.println(settings.rpm);  
  Serial.print("speed_correction: "); Serial.println(settings.speed_correction, SPEED_CORR_DPS);   
  lcd.setCursor(0,0);
  lcd.print("rpm: "); 
  if (settings.rpm == 33) {
    lcd.print("33 1/3");
  } else {
    lcd.print("45    ");
  }
  lcd.setCursor(0,1);
  lcd.print("xSpd: "); lcd.print(settings.speed_correction, SPEED_CORR_DPS); 
}


void setup() {
  // Set Input Pins
  pinMode(UP_PIN, INPUT_PULLUP); 
  pinMode(DOWN_PIN, INPUT_PULLUP); 
  pinMode(RPM_TOGGLE_PIN, INPUT_PULLUP); 
  // Initialise serial port and lcd
  Serial.begin(115200);
  lcd.init();  
  lcd.backlight();
  // Load settings
  settings = load_settings();
  // Setup SPWM...
  update_SPWM1(settings.rpm, 0);
}

void loop() {
  // Read inputs
  byte up_pin = digitalRead(UP_PIN);
  byte down_pin = digitalRead(DOWN_PIN);
  byte rpm_pin = digitalRead(RPM_TOGGLE_PIN);
  // Wait a bit and then read again to 'debounce'
  delay(10);
  up_pin = up_pin or digitalRead(UP_PIN);
  down_pin = down_pin or digitalRead(DOWN_PIN);
  rpm_pin = rpm_pin or digitalRead(RPM_TOGGLE_PIN);
  // Do any actions 
  if (pin_states.up_pin and !up_pin){
    // increase speed
    update_SPWM1(settings.rpm, 1);
  }
  if (pin_states.down_pin and !down_pin){
    // decrease speed
    update_SPWM1(settings.rpm, -1);
  }  
  if (pin_states.rpm_pin and !rpm_pin){
    // toggle speed setting...
    byte rpm = 33;
    if (settings.rpm == 33){
      rpm = 45;
    }
    // and update
    update_SPWM1(rpm, 0);    
  }   
  digitalWrite(LED_45RPM, settings.rpm == 45);
  // save pin states
  pin_states.up_pin = up_pin;
  pin_states.down_pin = down_pin;  
  pin_states.rpm_pin = rpm_pin;  
  //  and wait a bit ...
  delay(100);
}
