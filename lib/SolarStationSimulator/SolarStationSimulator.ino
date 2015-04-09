/*
  Output simulator for Solar Station
  Julio Ferrer 2015
*/

#include <ArduinoJson.h>

StaticJsonBuffer<200> jsonBuffer;
JsonObject& jSend = jsonBuffer.createObject();

int lower = 20; //define lower and upper limits of random numbers
int upper = 25;

void setup() {
  Serial.begin(9600);
}

void loop() {

  randomSeed(analogRead(0)); //Initialize random number generator by reading an unconnected pin
  
  //Temperature
  jSend["T_mono"] = float(random(lower,upper))+float(random(1,1000))/1000;
  jSend["T_thin"] = float(random(lower,upper))+float(random(1,1000))/1000;
  jSend["T_poly"] = float(random(lower,upper))+float(random(1,1000))/1000;
  
  //Voltages
  jSend["U_mono"] = float(random(lower,upper))+float(random(1,1000))/1000;
  jSend["U_thin"] = float(random(lower,upper))+float(random(1,1000))/1000;
  jSend["U_poly"] = float(random(lower,upper))+float(random(1,1000))/1000;
  
  //Currents
  jSend["I_mono"] = float(random(lower,upper))+float(random(1,1000))/1000;
  jSend["I_thin"] = float(random(lower,upper))+float(random(1,1000))/1000;
  jSend["I_poly"] = float(random(lower,upper))+float(random(1,1000))/1000;
  
  //Powers
  jSend["P_mono"] = float(random(lower,upper))+float(random(1,1000))/1000;
  jSend["P_thin"] = float(random(lower,upper))+float(random(1,1000))/1000;
  jSend["P_poly"] = float(random(lower,upper))+float(random(1,1000))/1000;
  
  jSend.prettyPrintTo(Serial);
  Serial.println("EOL");
  //jSend.printTo(Serial); //Prints everything in one line
  Serial.println();  
  
  
  //jSend.printTo(Serial);
  //jSend.prettyPrintTo(Serial);
  //Serial.println();
}
