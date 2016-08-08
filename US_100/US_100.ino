#include <Servo.h>
#include <TimerOne.h>
#include <avr/sleep.h>

#define sonar_output_pin 36  // ultrasonic sensor, TRIG -> digital write
#define sonar_input_pin 40  // ultrasonic sensor, ECHO -> digital read
#define servo_output_pin 7  // servo motor -> pwm out
#define battery_check_pin 3  // input pull-up
#define power_switch_pin 19  // pull-up pin - on/off switch - ground
#define power_led_pin 13

#define sonar_min_reading 160L    // ultrasonic sensor minimum reading 
#define sonar_max_reading 22600L  // ultrasonic sensor maximum reading

#define servo_min_angle 0   // servo motor minimum angle
#define servo_max_angle 180  // servo motor maximum angle

#define servo_init_angle servo_min_angle  // initial angle after turning on device
#define bootup_time 1000  // ultrasonic sensor start-up time
#define sonar_samples_n 5  // number of samples to get for each IO cycle
#define vcc_samples_n 20  // number of vcc samples to consider

const long sonar_reading_range = sonar_max_reading - sonar_min_reading;
long sonar_samples[sonar_samples_n];   // array to hold sonar readings
long sonar_old_reading = 0;
Servo servo;  // servo object
volatile boolean check_battery = false;
long vcc_samples[vcc_samples_n];
volatile boolean check_vcc = false;
int vcc_sample_pos = 0;
volatile boolean sleepy = false;

void resetDevice(){
  // reset variables after sleep
  sonar_old_reading = 0;
  check_battery = false;
  check_vcc = false;
  vcc_sample_pos = 0;
  
  long vcc = readVcc();
  for(int i = 0; i < vcc_samples_n; i++){
    vcc_samples[i] = vcc;
  }
  
  servo.write(servo_init_angle);
  delay(bootup_time);  // wait for sensor to initialize
}

void gotoSleep(){
  // sleep
  digitalWrite(power_led_pin, LOW);
  detachInterrupt(digitalPinToInterrupt(battery_check_pin));
  Timer1.detachInterrupt();
  attachInterrupt(digitalPinToInterrupt(power_switch_pin), onPowerUp, LOW);
  sleep_mode();
  
  // wake up
  digitalWrite(power_led_pin, HIGH);
  resetDevice();
  Timer1.attachInterrupt(checkVcc);
  attachInterrupt(digitalPinToInterrupt(battery_check_pin), onBatteryCheckButtonPressed, FALLING);
  attachInterrupt(digitalPinToInterrupt(power_switch_pin), onPowerDown, RISING);
}

void onPowerUp(){
  detachInterrupt(digitalPinToInterrupt(power_switch_pin));
  sleepy = false;
}

void onPowerDown(){
  detachInterrupt(digitalPinToInterrupt(power_switch_pin));
  sleepy = true;
}

void initPowerSwitch(){
  pinMode(power_led_pin, OUTPUT);
  digitalWrite(power_led_pin, HIGH);
  pinMode(power_switch_pin, INPUT_PULLUP);
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  attachInterrupt(digitalPinToInterrupt(power_switch_pin), onPowerDown, RISING);
}

void initSonar(){
  delay(bootup_time);  // wait for sensor to initialize
  pinMode(sonar_output_pin, OUTPUT);  // set TRIG pin as output
  digitalWrite(sonar_output_pin, LOW);
  pinMode(sonar_input_pin, INPUT); // set ECHO pin as input
}

void initServo(){
  servo.attach(servo_output_pin);
  servo.write(servo_init_angle);
}

void initBatteryCheck(){
  long vcc = readVcc();
  for(int i = 0; i < vcc_samples_n; i++){
    vcc_samples[i] = vcc;
  }
  
  // timer interrupt to read vcc every 10s
  Timer1.initialize(10000000UL);
  Timer1.attachInterrupt(checkVcc);
  
  // external interrupt for check battery button presses
  pinMode(battery_check_pin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(battery_check_pin), onBatteryCheckButtonPressed, FALLING);
}

void takeSonarSamples(){
  int i = 0;
  while(i < sonar_samples_n){
    delay(70);  // wait time between readings
    
    // send >10uS pulse to TRIG
    digitalWrite(sonar_output_pin, HIGH);
    delayMicroseconds(11);
    digitalWrite(sonar_output_pin, LOW);
    
    // listen for pulse on ECHO
    sonar_samples[i] = pulseIn(sonar_input_pin, HIGH);
    
    // check if sensor reading is valid
    if(sonar_samples[i] > sonar_min_reading && sonar_samples[i] < sonar_max_reading){
      sonar_old_reading = sonar_samples[i];
      i++;
    }
    else if(sonar_samples[i] == 0){
      sonar_old_reading = 0;
    }
    else if(sonar_old_reading != 0) {
      sonar_samples[i] = sonar_old_reading;
      i++;
    }
  }
}

long medianSample(long samples[], int nSamples){
  // insertion sort samples
  long currVal;
  int i, j;
  for(i = 1; i < nSamples; i++){
    currVal = samples[i];
    for(j = i-1; j >= 0 && samples[j] > currVal; j--){
      samples[j+1] = samples[j];
    }
    samples[j+1] = currVal;
  }
  // return median reading
  return samples[nSamples / 2];
}

long getSonarReading(){
  takeSonarSamples();
  return medianSample(sonar_samples, sonar_samples_n);
}

void turnServo(int angle){
  servo.write(angle);
  delay(15);  // wait for servo to finish
}

void convertSonar2Servo(long sonarReading){
  int angle = (sonar_max_reading - sonarReading) * 100 / sonar_reading_range * servo_max_angle / 100;
  turnServo(angle);
}

void checkVcc(){
  check_vcc = true;
}

// Returns actual value of Vcc (x 1000)
long readVcc(){
  // measure internal 1.1V reference against AVcc
  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
    ADMUX = _BV(MUX5) | _BV(MUX0);
  #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
    ADMUX = _BV(MUX3) | _BV(MUX2);
  #else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif
  
  delay(2);  // Let mux settle a little to get a more stable A/D conversion
  ADCSRA |= _BV( ADSC );  // Start a conversion
  while( ( (ADCSRA & (1<<ADSC)) != 0 ) );  // Wait for it to complete
  uint8_t low = ADCL;  // must read ADCL first - it then locks ADCH
  uint8_t high = ADCH;  // unlocks both
  long result = (high<<8) | low;
  return result == 0L ? 0L : 1126400L / result;  // 1100L * 1024L = 1126400L
}

void sampleVcc(){
  vcc_samples[vcc_sample_pos] = readVcc();
  vcc_sample_pos = (vcc_sample_pos + 1) % vcc_samples_n;
}

void onBatteryCheckButtonPressed(){
  check_battery = true;
}

void showCharge(){
  // begin routine by doing a full sweep
  int step = 10;
  int angle = servo_min_angle;
  while(angle < servo_max_angle){
    turnServo(angle);
    angle += step;
  }
  while(angle >= servo_min_angle){
    turnServo(angle);
    angle -= step;
  }
  
  /******** Notes on Battery ************
    Xiaomi power bank, charge vs voltage
    100% : 5190
     75% : 5190
     50% : 5190
     25% : 5190

    This power bank's output voltage does NOT decrease with charge level.
    At low charge levels, its output voltage becomes less consistent.

    Based on this observation, we'll assume its charge is low if recent samples of voltage readings show inconsistencies.
  ***********************************/
  
  int currPos = 1;
  while(currPos < vcc_samples_n && vcc_samples[currPos] == vcc_samples[0]){
    currPos++;
  }
  // then move to indicate charge
  if(currPos != vcc_samples_n){
    // low charge
    turnServo(servo_max_angle);
  } else {
    // OK charge
    turnServo(servo_min_angle);
  }
  delay(3000);
  
  // end routine by moving to middle position
  turnServo(servo_max_angle / 2);
}

void setup() {
  initServo();
  initSonar();
  initBatteryCheck();
  initPowerSwitch();
}

void loop() {
  // power switch status
  if(sleepy){
    gotoSleep();
  }
  
  // do battery check if flag is ON
  if(check_battery){
    showCharge();
    check_battery = false;  // turn flag OFF
  }
  
  // convert sonar median reading to servo output angle
  convertSonar2Servo(getSonarReading());
  
  // time to check vcc
  if(check_vcc){
    sampleVcc();
    check_vcc = false;
  }
}



