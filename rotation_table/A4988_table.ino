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

A4988 stepper(200, DIRECTION_PIN, MOTOR_PIN, MS1, MS2, MS3);

void setup() {
  Serial.begin(9600);
  pinMode(BUTTON_PIN,INPUT);
  stepper.begin(1/6,1);
  

}
void RecordData(int b){
  if(millis()- log_time >= 50){ //log every 50 ms
          Serial.print(millis()-initial_time);       
          Serial.print(",\t");
          Serial.println(b*5);
          log_time=millis();
       
        }
  }
  



void loop() {
  if(digitalRead(BUTTON_PIN)==HIGH){ //if button is pressed, move 5 degrees every 5 seconds 
    initial_time = millis();
    log_time = millis();
    
    for(int b = 0; b<72; b++){
    Serial.print("Entering new Iteration:");
    Serial.println(b);
    stepper.rotate(5);
   
      counter_time = millis(); 
      while(millis()-counter_time<5000){ //wait for 5 seconds after 5 degrees
       RecordData(b+1);
     }
   }
  
 }

}
