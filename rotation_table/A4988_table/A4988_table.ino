#include "A4988.h"

#define MOTOR_PIN 9
#define BUTTON_PIN 10
#define DIRECTION_PIN 8
#define MS1 10
#define MS2 11
#define MS3 12

unsigned int initial_time;
A4988 stepper(400,0, MOTOR_PIN, MS1, MS2, MS3);

void setup() {
  Serial.begin(9600);
  stepper.begin(1,1); //set RPM 
  

}
void RecordData(int b){
Serial.print("TIME: ");
 Serial.print(millis()-initial_time);       
 Serial.print(",\t");
 Serial.println(b*10);
       
 }
  



void loop() {
    initial_time = millis();
    static int count = 1;
    Serial.print("Entering revolution number:");
    Serial.println(count);
    for(int b = 0; b<36; b++){ //360 degrees
      stepper.rotate(20); //rotate 10 degrees (2:1 ratio)
      delay(10000); //delay 10 sec
      RecordData(b);
     }
      count++;

}
