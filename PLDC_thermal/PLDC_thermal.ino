//Constants
const float VCC = 5.0; // fixed arduino volage supply (5V)
const float Rfixed = 1000; //fixed resistance value
const float R0 = 10000; //fixed thermistor resistance
const float Beta = 3492.0; //beta constant (used in temperature equation)
const float temp = 25; //reference temperature in degree Celcuis
const float T0_k = 273.15 + temp; //reference temperature in Kelvin
//Thermsistor Inputs
const int numTherm = 6; //number of resistors
const int thermPins[numTherm] =  {A0, A1, A2, A3, A4, A5};
const int numSamples = 10; //number of measuremnts from each thermsistor
const int sampleDelay = 1; //# of ms between each measurement takes place
void setup() {
  Serial.begin(9600);
  while(!Serial) { }
  Serial.println("time_ms,T1_C,T2_C,T3_C,T4_C,T5_C,T6_C\n");
}
void loop() {
  unsigned long timeNow = millis(); // tracks time since start
  Serial.print(timeNow);
  Serial.print(",");
  // loop through each thermistor
  for (int i = 0; i < numTherm; i++)
  {
    long sum = 0;
    //adding and averaging the total ADC
    for(int n = 0; n < numSamples; n++)
    {
      sum += analogRead(thermPins[i]);
      delay(sampleDelay);
    }
    float adc_avg = sum / (float)numSamples;
    // calculates the total voltage out, based on this relationship to convert the ADC number back into volts
    float Vout = (adc_avg / 1023.0) * VCC; // use 1023.0 (float division)
    // calculating thermsistor resistance
    float RNTC = Rfixed * (Vout / (VCC - Vout));
    //temperature calculation
    float invT = (1.0 / T0_k) + (1.0 / Beta) * log(RNTC / R0);
    float tempK = 1.0 / invT;
    float tempC = tempK - 273.15;
    Serial.print(tempC, 2);
    if (i < numTherm - 1) Serial.print(",");
  }
  Serial.println();
  delay(1000);
}