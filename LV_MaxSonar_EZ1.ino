#include <Adafruit_SoftServo.h>  // SoftwareServo (works on non PWM pins)

#define sensor_input_pin 2  // ultrasonic sensor -> #2, digital read
#define servo_output_pin 0  // servo motor -> #0, pwm out

#define min_reading 880L    // ultrasonic sensor minimum reading 
#define max_reading 37500L  // ultrasonic sensor maximum reading

#define min_angle 0L    // servo motor minimum angle
#define max_angle 180L  // servo motor maximum angle

#define init_angle 90   // initial angle after turning on device
#define bootup_time 448  // ultrasonic sensor start-up time
#define n_samples 5  // number of samples to get for each IO cycle

long samples[n_samples];   // array to hold sample readings
Adafruit_SoftServo servo;  // servo object

void setup() {
  servo.attach(servo_output_pin);
  
  // Set up the interrupt that will refresh the servo for us automagically
  OCR0A = 0xAF;            // any number is OK
  TIMSK |= _BV(OCIE0A);    // Turn on the compare interrupt (below!)
  
  servo.write(init_angle);

  delay(bootup_time);  // wait for sensor to initialize
  
  pinMode(sensor_input_pin, INPUT); // Set ultrasonic sensor pin as input
}

void loop() {
  takeSamples();  // take samples from sensor readings
  sortSamples();  // sort samples
  // convert median reading to servo output angle
  int angle = map(samples[n_samples / 2], min_reading, max_reading, min_angle, max_angle);
  servo.write(angle); 
  delay(15);
}

void takeSamples(){
  long reading;  // pulse reading from sensor
  int i = 0;
  while(i < n_samples){
    reading = pulseIn(sensor_input_pin, HIGH);
    // check if sensor reading is valid
    if(reading > min_reading && reading < max_reading){
      samples[i] = reading;
      i++;
    }
  }
}

void sortSamples(){
  long currVal;
  int i, j;
  for(i = 1; i < n_samples; i++){
    currVal = samples[i];
    for(j = i-1; j >= 0 && samples[j] > currVal; j--){
      samples[j+1] = samples[j];
    }
    samples[j+1] = currVal;
  }
}


// We'll take advantage of the built in millis() timer that goes off
// to keep track of time, and refresh the servo every 20 milliseconds
// The SIGNAL(TIMER0_COMPA_vect) function is the interrupt that will be
// Called by the microcontroller every 2 milliseconds
volatile uint8_t counter = 0;
SIGNAL(TIMER0_COMPA_vect) {
  // this gets called every 2 milliseconds
  counter += 2;
  // every 20 milliseconds, refresh the servos!
  if (counter >= 20) {
    counter = 0;
    servo.refresh();
  }
}

