#include <SD.h>

File rotdata;

#define MOTOR_PIN 9
#define BUTTON_PIN 10
#define DIRECTION_PIN 8

unsigned int initial_time;
unsigned int log_time;
unsigned int counter_time;


void setup() {
  pinMode(MOTOR_PIN,OUTPUT);
  pinMode(BUTTON_PIN,INPUT);
  pinMode(DIRECTION_PIN,OUTPUT);
  digitalWrite(DIRECTION_PIN, HIGH);

  rotdata= SD.open("Rotation_Table.csv", FILE_WRITE);
  if(rotdata){
  rotdata.println("Time | Angle");
  rotdata.close();
  }
  else{
    Serial.println("Error opening");
  }
}

void RecordData(int b, int i){
  if(millis()- log_time >= 50){ //log every 50 ms
      rotdata= SD.open("RotationTable.csv", FILE_WRITE);
      if(rotdata){
          rotdata.print(millis()-initial_time);       
          rotdata.print(",\t");
          rotdata.println((b*5) +(i*1.8));
          log_time=millis();
          rotdata.close();
        }
       else{
        Serial.println("Error opening");
       }
       
  }
  

}

void loop() {
  if(digitalRead(BUTTON_PIN)==HIGH){    //if button is pressed, move 5 degrees every 5 seconds 
    initial_time = millis();
    log_time = millis();
    
   
   for(int b = 0; b<72; b++){
    Serial.print("Entering new Iteration:");
    Serial.println(b);
    for(int i = 0; i<3; i++){ //rotate 5 degrees assuming we are using a 200 step/revolution stepper motor, 3 steps is approx 5 degrees  
      digitalWrite(MOTOR_PIN,HIGH); //10hz
      delay(50);
      RecordData(b,i);
      digitalWrite(MOTOR_PIN, LOW);
      delay(50);
      RecordData(b,i);
      
}
      counter_time = millis(); 
      
      while(millis()-counter_time<5000){ //5 degrees every 5 seconds
        RecordData(b,0);
     }
   }
  

}

  

  

  



}
