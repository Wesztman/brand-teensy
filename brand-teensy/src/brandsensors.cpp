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
