#include "A4988.h"

#define MOTOR_PIN 9
#define BUTTON_PIN 10
#define DIRECTION_PIN 8
#define MS1 10
#define MS2 11
#define MS3 12

unsigned int initial_time;
unsigned int log_time;
unsigned int counter_time;
A4988 stepper(400, DIRECTION_PIN, MOTOR_PIN, MS1, MS2, MS3);

void setup() {
  Serial.begin(9600);
  pinMode(BUTTON_PIN,INPUT);
  stepper.begin(20,1); //set RPM 
  

}
void RecordData(int b){
 Serial.print(millis()-initial_time);       
 Serial.print(",\t");
 Serial.println(b*5);
       
 }
  



void loop() {
//  if(digitalRead(BUTTON_PIN)==HIGH){ 
    initial_time = millis();
    log_time = millis();
    static int count = 1;
    Serial.print("Entering revolution number:");
    Serial.println(count);
    for(int b = 0; b<72; b++){
    Serial.print("Entering iteration:");
    Serial.println(b);
    
    stepper.rotate(5);
    RecordData(b);
    delay(2000);
   
   }
      count++;
  
 //}

}
