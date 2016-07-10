#include <Servo.h>
#include <arduino.h>

const int pinRange = 3;
bool highRange = true;

Servo servo;
int pos = 0;
int value = 0;

int lastLength = -1;

void setup() {
	//confiigure serial for debug
	Serial.begin(9600);

	//configure Range of ultrasonic sensor (US-016)
	if (highRange)
	{
		pinMode(pinRange, OUTPUT);
		digitalWrite(pinRange, HIGH);
	}

	//configure servo
	servo.attach(2);

	Serial.println("====================");
	Serial.println("LAUNCHED");
	Serial.print("Range: ");

	if (highRange)
		Serial.println("3m (high)");
	else
		Serial.println("1m (low)");

	Serial.println("====================");
}

/*
long readVcc() {
	long result; // Read 1.1V reference against AVcc 
	ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
	delay(2); // Wait for Vref to settle 
	ADCSRA |= _BV(ADSC); // Convert 
	while (bit_is_set(ADCSRA, ADSC));
	result = ADCL;
	result |= ADCH << 8;
	result = 1126400L / result; // Back-calculate AVcc in mV 
	return result;
}
*/

void showDistance(int distance)
{
	distance = ((int)(distance / 10.0)) * 10.0;

	if (distance < 20) // malfunction of servo
		distance = 20;

	if (servo.read() != distance)
		servo.write(distance);
}

void loop() {
	float k = (float)analogRead(A0) / 1023;

	float multiplier = highRange ? 3096 : 1024;
	float length = k * multiplier;
	float angle = k * 180;

	// print out the value you read:
	if (lastLength != length)
	{
		lastLength = length;
		showDistance(angle);

		Serial.print("k: ");
		Serial.println(k);

		Serial.print("length(mm): ");
		Serial.println(length);

		Serial.print("angle: ");
		Serial.println(angle);
	}

	delay(100);
}

