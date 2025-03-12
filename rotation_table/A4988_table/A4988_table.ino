#include "A4988.h"

#define MOTOR_PIN 9
#define BUTTON_PIN 10
#define DIRECTION_PIN 8
#define MS1 10
#define MS2 11
#define MS3 12

// unsigned int initial_time;
// unsigned int log_time;
// unsigned int counter_time;

A4988 stepper(400, 0, MOTOR_PIN, MS1, MS2, MS3);

int initial_time = millis();
int log_time = millis();

void setup() {
  Serial.begin(9600);
  pinMode(BUTTON_PIN,INPUT);
  Serial.println("Starting stepper motor");
  stepper.begin(5,1); //NOTE: Gear ratio 36:72 motor:table
  Serial.println("motor started");
}

void RecordData(int b){
  if(millis()- log_time >= 50){ //log every 50 ms
          Serial.print(millis()-initial_time);       
          Serial.print(",\t");
          Serial.println(b*5);
          log_time=millis();
       
        }
  }
  

static int count = 0;
int b = 0;

void loop() {
  // if(digitalRead(BUTTON_PIN)==HIGH){ //if button is pressed, move 5 degrees every 5 seconds 
  // initial_time = millis();
  // log_time = millis();
  // int b = 0;
  Serial.print("Entering new Iteration:");
  Serial.println(b);
  stepper.rotate(720); // x * pi/180 degrees at 2 rad/s
  delay(10000);

  // counter_time = millis(); 
  // while(millis()-counter_time<2000){ //wait for 5 seconds after 5 degrees
  //   RecordData(b+1);
  // }
  b++;

  // }
  // Serial.println("Rotating motor");
  // stepper.rotate(360);
  // Serial.println(count);
  // Serial.println("motor rotated");
  // ++count;
  // delay(1000);

}
