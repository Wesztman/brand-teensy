#include "brandsensor.h"


/*
Triggers Ultrasonic sensor and measures the travel time  of the returning sound wave.
Returns the distance in mm. Min 20mm, Max 2000mm.
Max can be increased to 4000 if timeout is increased.
Input: triggerpin and echopin for the sensor
*/
int readUltraDist(int trig, int echo){
   float duration; // variable for the duration of sound wave travel
   float distance; // variable for the distance measurement
   float prevUltraDist = 0.0;  //  stores the old value of ultrasonic sensor
   // Clears the trigPin condition
   digitalWrite(trig, LOW);
   delayMicroseconds(2);
   // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
   digitalWrite(trig, HIGH);
   delayMicroseconds(10);
   digitalWrite(trig, LOW);
   // Reads the echoPin, returns the sound wave travel time in microseconds
   duration = pulseIn(echo, HIGH, 11600);
   // Calculating the distance
   distance = duration * 0.343 / 2; // Speed of sound wave divided by 2 (go and back)
   
   if(distance == 0){
      distance = prevUltraDist;
   }
   prevUltraDist = distance;
   // Return distance
   return (int)distance;
}

//Serial prints pixel values from IR Camera
void printIRCamera(float pixels[]){

 Serial.print("[");
  for(int i=1; i<=AMG88xx_PIXEL_ARRAY_SIZE; i++){
    Serial.print(pixels[i-1]);
    Serial.print(", ");
    if( i%8 == 0 ) Serial.println();
  }
  Serial.println("]");
  Serial.println();
}

//Return value from line sensor 0-1024
int readLineSensor(int linePin){
     int lineValue = analogRead(linePin);
     return lineValue;
}

//Encoder_OP* Encoder_OP::sEncoder = 0;

Encoder_OP::Encoder_OP(int pin1, int pin2)
{
   pinMode(pin1, INPUT_PULLUP);
   pinMode(pin2, INPUT_PULLUP);

   _pin1 = pin1;
   _pin2 = pin2;

   //sEncoder = this;
   attachInterrupt(digitalPinToInterrupt(pin1), encISR, CHANGE);
   attachInterrupt(digitalPinToInterrupt(pin2), encISR, CHANGE);
}

int32_t Encoder_OP::readEnc()
{
   return position;
}

void Encoder_OP::updateEnc()
{
   newState = state & 3;
   if (digitalRead(_pin1)) newState |= 4;
	if (digitalRead(_pin2)) newState |= 8;
		switch (newState) {
			case 0: case 5: case 10: case 15:
				break;
			case 1: case 7: case 8: case 14:
				position++; break;
			case 2: case 4: case 11: case 13:
				position--; break;
			case 3: case 12:
				position += 2; break;
			default:
				position -= 2; break;
		}
		state = (newState >> 2);
}

void Encoder_OP::resetEnc()
{
   position = 0;
}

/*
void Encoder_OP::updateEncoderISR()
{
   if (sEncoder != 0)
        sEncoder->updateEnc();
}
*/

void encISR(Encoder_OP* enc)
{
   enc->updateEnc();
}