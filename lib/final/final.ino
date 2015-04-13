#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_TSL2561_U.h>
#include <Adafruit_ADXL345_U.h>
#include <ArduinoJson.h>

/* Assign a unique ID to the Light Intensity and Accelerometer sensors*/
Adafruit_TSL2561_Unified tsl = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345);
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
/**************************************************************************/
/*
    Displays some basic information on the Lux sensor from the unified
    sensor API sensor_t type (see Adafruit_Sensor for more information)
*/
/**************************************************************************/
void displayLightSensorDetails(void)
{
 sensor_t sensor;
 tsl.getSensor(&sensor);
 Serial.println("------------------------------------");
 Serial.print  ("Sensor:       "); Serial.println(sensor.name);
 Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
 Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
 Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" lux");
 Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" lux");
 Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" lux");  
 Serial.println("------------------------------------");
 Serial.println("");
 delay(500);
}

/**************************************************************************/
/*
    Configures the gain and integration time for the TSL2561
*/
/**************************************************************************/
void configureSensor(void)
{
 /* You can also manually set the gain or enable auto-gain support */
 // tsl.setGain(TSL2561_GAIN_1X);      /* No gain ... use in bright light to avoid sensor saturation */
 // tsl.setGain(TSL2561_GAIN_16X);     /* 16x gain ... use in low light to boost sensitivity */
 tsl.enableAutoRange(true);            /* Auto-gain ... switches automatically between 1x and 16x */
  
 /* Changing the integration time gives you better sensor resolution (402ms = 16-bit data) */
 tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_13MS);      /* fast but low resolution */
 // tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_101MS);  /* medium resolution and speed   */
 // tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_402MS);  /* 16-bit data but slowest conversions */

 /* Update these values depending on what you've set above! */  
// Serial.print  ("Gain:         "); Serial.println("Auto");
// Serial.print  ("Timing:       "); Serial.println("13 ms");
// Serial.println("------------------------------------");
// Serial.println("------------------------------------");
}

/**************************************************************************/
/*
    Displays some basic information on the Accel sensor from the unified
    sensor API sensor_t type (see Adafruit_Sensor for more information)
*/
/**************************************************************************/

void displayAccelSensorDetails(void)
{
  sensor_t sensor;
  accel.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" m/s^2");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" m/s^2");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" m/s^2");  
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

void displayDataRate(void)
{
  Serial.print  ("Data Rate:    "); 
  
  switch(accel.getDataRate())
  {
    case ADXL345_DATARATE_3200_HZ:
      Serial.print  ("3200 "); 
      break;
    case ADXL345_DATARATE_1600_HZ:
      Serial.print  ("1600 "); 
      break;
    case ADXL345_DATARATE_800_HZ:
      Serial.print  ("800 "); 
      break;
    case ADXL345_DATARATE_400_HZ:
      Serial.print  ("400 "); 
      break;
    case ADXL345_DATARATE_200_HZ:
      Serial.print  ("200 "); 
      break;
    case ADXL345_DATARATE_100_HZ:
      Serial.print  ("100 "); 
      break;
    case ADXL345_DATARATE_50_HZ:
      Serial.print  ("50 "); 
      break;
    case ADXL345_DATARATE_25_HZ:
      Serial.print  ("25 "); 
      break;
    case ADXL345_DATARATE_12_5_HZ:
      Serial.print  ("12.5 "); 
      break;
    case ADXL345_DATARATE_6_25HZ:
      Serial.print  ("6.25 "); 
      break;
    case ADXL345_DATARATE_3_13_HZ:
      Serial.print  ("3.13 "); 
      break;
    case ADXL345_DATARATE_1_56_HZ:
      Serial.print  ("1.56 "); 
      break;
    case ADXL345_DATARATE_0_78_HZ:
      Serial.print  ("0.78 "); 
      break;
    case ADXL345_DATARATE_0_39_HZ:
      Serial.print  ("0.39 "); 
      break;
    case ADXL345_DATARATE_0_20_HZ:
      Serial.print  ("0.20 "); 
      break;
    case ADXL345_DATARATE_0_10_HZ:
      Serial.print  ("0.10 "); 
      break;
    default:
      Serial.print  ("???? "); 
      break;
  }  
  Serial.println(" Hz");  
}

void displayRange(void)
{
  Serial.print  ("Range:         +/- "); 
  
  switch(accel.getRange())
  {
    case ADXL345_RANGE_16_G:
      Serial.print  ("16 "); 
      break;
    case ADXL345_RANGE_8_G:
      Serial.print  ("8 "); 
      break;
    case ADXL345_RANGE_4_G:
      Serial.print  ("4 "); 
      break;
    case ADXL345_RANGE_2_G:
      Serial.print  ("2 "); 
      break;
    default:
      Serial.print  ("?? "); 
      break;
  }  
  Serial.println(" g");  
}

// Data wire is plugged into port 2 on the Arduino
#define ONE_WIRE_BUS 2
#define TEMPERATURE_PRECISION 12

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

int numberOfDevices; // Number of temperature devices found

DeviceAddress tempDeviceAddress; // We'll use this variable to store a found device address

StaticJsonBuffer<250> jsonBuffer; //Used for JSON formatting
JsonObject& jSend = jsonBuffer.createObject();

boolean showSettings = false;

// function to print the temperature for a device
void printTemperature(DeviceAddress deviceAddress)
{
 float tempC = sensors.getTempC(deviceAddress);
 Serial.print(tempC);
 Serial.print(" C");
 Serial.print("\t");
 Serial.print("\t");
}

// function to print a device address
void printAddress(DeviceAddress deviceAddress)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
}

float sensorValue0;
float sensorValue1;
float sensorValue2;
float sensorValue3;
float sensorValue4;
float sensorValue5;


float voltage0;
float voltage1;
float voltage2;
  
float current0;
float current1;
float current2;
  
float power0;
float power1;
float power2;

boolean ADCread = true;

String content = "";
char floatbuf[32];

const float alpha = 0.5;
double fXg = 0;
double fYg = 0;
double fZg = 0;



void setup(void)
{
  // start serial port
  Serial.begin(9600);
  Serial1.begin(19200);
  if(showSettings){
    Serial.println("Light Sensor Settings");
  }
  
  if(!tsl.begin())
  {
    /* There was a problem detecting the ADXL345 ... check your connections */
    Serial.print("Ooops, no TSL2561 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  
  /* Display some basic information on this sensor */
  if(showSettings){
    displayLightSensorDetails();
  }
  
  /* Setup the sensor gain and integration time */
  configureSensor();
  
  /* We're ready to go! */
  
  if(showSettings){
  //Serial.println("Inclination Test"); Serial.println("");
  }
  /* Initialise the sensor */
  if(!accel.begin())
  {
    /* There was a problem detecting the ADXL345 ... check your connections */
    Serial.println("Ooops, no ADXL345 detected ... Check your wiring!");
    while(1);
  }

  /* Set the range to whatever is appropriate for your project */
  accel.setRange(ADXL345_RANGE_2_G);
  // displaySetRange(ADXL345_RANGE_8_G);
  // displaySetRange(ADXL345_RANGE_4_G);
  // displaySetRange(ADXL345_RANGE_2_G);
  
  /* Display some basic information on this sensor */
  if(showSettings){
    displayAccelSensorDetails();
  }
  
  /* Display additional settings (outside the scope of sensor_t) */
  if(showSettings){
    displayDataRate();
    displayRange();
    Serial.println("");
  }
  /* We're ready to go! */

  // Start up the temperature library
  sensors.begin();
  
  // Grab a count of temperature devices on the wire
  numberOfDevices = sensors.getDeviceCount();
  if(showSettings){
    Serial.println("");
    Serial.println("Temperature Sensor Settings");
    Serial.println("------------------------------------");
    
    
    // locate devices on the bus
    Serial.print("Locating temperature devices...");
    delay(500);
    
    Serial.print("Found ");
    Serial.print(numberOfDevices, DEC);
    Serial.println(" devices.");
    Serial.println("");
    delay(500);
  
    // report parasite power requirements
    Serial.print("Parasite power is: "); 
    if (sensors.isParasitePowerMode()) Serial.println("ON");
    else Serial.println("OFF");
    delay(500);
  }
 
  
  // Loop through each device, print out address
  for(int i=0;i<numberOfDevices; i++)
  {
    // Search the wire for address
    if(sensors.getAddress(tempDeviceAddress, i))
	{
          if(showSettings){
                Serial.println("");
		Serial.print("Found device ");
		Serial.print(i, DEC);
		Serial.print(" with address: ");
		printAddress(tempDeviceAddress);
                Serial.println("");
                delay(500);
		
		Serial.print("Setting resolution to ");
		Serial.println(TEMPERATURE_PRECISION, DEC);
	        }
		// set the resolution to TEMPERATURE_PRECISION bit (Each Dallas/Maxim device is capable of several different resolutions)
		sensors.setResolution(tempDeviceAddress, TEMPERATURE_PRECISION);
	if(showSettings){
		Serial.print("Resolution actually set to: ");
		Serial.println(sensors.getResolution(tempDeviceAddress), DEC); 
                delay(500);
                }
	}else{
		Serial.print("Found ghost device at ");
		Serial.print(i, DEC);
		Serial.println(" but could not detect address. Check power and cabling");
	}
  
  }
  if(showSettings){
    Serial.println("---------------------------------------------------------------");
    Serial.println("---------------------------------------------------------------");
  }
}




void loop() {
  
        
while(ADCread){
  
  /* Get a new sensor event */ 
 sensors_event_t eventLight;
 tsl.getEvent(&eventLight);
 
 /* Display the results (light is measured in lux) */
 if(showSettings){
   Serial.println();
   Serial.println("Test Conditions");
   Serial.println("------------------");
 }
 
 if (eventLight.light)
 {
    jSend["L_lux"] = eventLight.light;
    jSend["L_w/m2"] = eventLight.light*0.0079;
//    Serial.print(eventLight.light); 
//    Serial.print(" lux");
//    Serial.print(" ----> for sunlight: ");
//    Serial.print(eventLight.light*0.0079); 
//    Serial.println(" W/m2");
 }
 else
 {
    /* If event.light = 0 lux the sensor is probably saturated
       and no reliable data could be generated! */
    Serial.println("Sensor overload");
 }

   double pitch, roll, Xg, Yg, Zg;
  
  /* Get a new sensor event */ 
  sensors_event_t eventAccel; 
  accel.getEvent(&eventAccel);
 
  /* Display the results (acceleration is measured in m/s^2) */
  Xg = eventAccel.acceleration.x; 
  Yg = eventAccel.acceleration.y; 
  Zg = eventAccel.acceleration.z; 
  
  //Low Pass Filter
  fXg = Xg * alpha + (fXg * (1.0 - alpha));
  fYg = Yg * alpha + (fYg * (1.0 - alpha));
  fZg = Zg * alpha + (fZg * (1.0 - alpha));
  
  //Roll & Pitch Equations
  roll  = (atan2(-fYg, fZg)*180.0)/M_PI;
  pitch = (atan2(fXg, sqrt(fYg*fYg + fZg*fZg))*180.0)/M_PI;

  jSend["Pitch"] = pitch;
  jSend["Roll"] = roll; 
//  Serial.print("Pitch: ");
//  Serial.print(pitch);
//  Serial.print("deg");
//  Serial.print("\t");
//  Serial.print(", Roll: ");
//  Serial.print(roll);
//  Serial.println("deg");
  
 // call sensors.requestTemperatures() to issue a global temperature 
 // request to all devices on the bus

 sensors.requestTemperatures(); // Send the command to get temperatures
  
 // Loop through each device, print out temperature data
 for(int i=0;i<numberOfDevices; i++)
  {
    // Search the wire for address
    if(sensors.getAddress(tempDeviceAddress, i))
	{
//                Serial.print("T");
                switch (i) 
                  {
                      case 0:
                          jSend["T_mono"] =  float(sensors.getTempC(tempDeviceAddress));
//                          Serial.print("_mono");
                          break;
                      case 1:
                          jSend["T_thin"] =  float(sensors.getTempC(tempDeviceAddress));
//                          Serial.print("_thin");
                          break;
                      case 2:
                          jSend["T_poly"] =  float(sensors.getTempC(tempDeviceAddress));
//                          Serial.print("_poly");
                          break;
                  }
             
		//Serial.print(": ");
		// It responds almost immediately. Let's print out the data
		//printTemperature(tempDeviceAddress); // Use a simple function to print out the data
         } 
	//else ghost device! Check your power requirements and cabling
	
  }
  
  
 sensorValue0 = analogRead(A0);
 sensorValue1 = analogRead(A1);
 sensorValue2 = analogRead(A2);
 sensorValue3 = analogRead(A3);
 sensorValue4 = analogRead(A4);
 sensorValue5 = analogRead(A5);
  
 //Serial.println(sensorValue0);
  
 // Convert the analog readings (which goes from 0 - 1023) to voltages (0 - 20V) or (0 - 65V):
 voltage0 = 4 * sensorValue0 * (5.0 / 1023);
 voltage1 = 14 * sensorValue1 * (5.0 / 1023);
 voltage2 = 4 * sensorValue2 * (5.0 / 1023);
  
 // Convert the analog readings (which goes from 0 - 1023) to currents (0 - 6A):
 current0 = (sensorValue3 / 0.79)  * (5.0 / 1023);
 current1 = (sensorValue4 / 0.79) * (5.0 / 1023);
 current2 = (sensorValue5 / 0.79) * (5.0 / 1023);
  
  
  // Calculate the power
 power0 = voltage0 * current0;
 power1 = voltage1 * current1;
 power2 = voltage2 * current2;
  
// print out the values you read:
jSend["U_mono"] =  voltage0;
jSend["U_thin"] =  voltage1;
jSend["U_poly"] =  voltage2;

jSend["I_mono"] =  current0;
jSend["I_thin"] =  current1;
jSend["I_poly"] =  current2;

jSend["P_mono"] =  power0;
jSend["P_thin"] =  power1;
jSend["P_poly"] =  power2;

 if(showSettings){
   Serial.println("");
  
   Serial.println();
   Serial.println("PV Modules");
   Serial.println("----------");
   Serial.print("U_mono: ");
   Serial.print(voltage0, 3);
   Serial.print(" V");
   Serial.print("\t");
   
    
   Serial.print("U_thin: ");
   Serial.print(voltage1, 3);
   Serial.print(" V");
   Serial.print("\t");
   
    
   Serial.print("U_poly: ");
   Serial.print(voltage2, 3);
   Serial.print(" V");
   Serial.println("\t");
    
    
   Serial.print("I_mono: ");
   Serial.print(current0, 3);
   Serial.print(" A");
   Serial.print("\t");
   Serial.print("\t");
    
   Serial.print("I_thin: ");
   Serial.print(current1, 3);
   Serial.print(" A");
   Serial.print("\t");
   Serial.print("\t");
    
   Serial.print("I_poly: ");
   Serial.print(current2, 3);
   Serial.println(" A");
   
    
   Serial.print("P_mono: ");
   Serial.print(power0);
   Serial.print(" W");
   Serial.print("\t");
   Serial.print("\t");
    
   Serial.print("P_thin: ");
   Serial.print(power1);
   Serial.print(" W");
   Serial.print("\t");
   Serial.print("\t");
    
   Serial.print("P_poly: ");
   Serial.print(power2);
   Serial.println(" W");
  
   Serial.println();
 }

 if(showSettings){
   Serial.println("Battery Bank");
   Serial.println("------------");
 }
 ADCread = false;
          
 } 
 
 while(Serial1.available()) {
    
     content.concat(Serial1.readStringUntil('\n'));

     if (content.indexOf('V') == 0) {
       content.substring(2).toCharArray(floatbuf, sizeof(floatbuf));
       //Serial.print(content.substring(0,2)); 
       jSend["B_volts"] =  float(atof(floatbuf))/1000;
       //float number_0 = atof(floatbuf);
       //Serial.print(number_0/1000);
       //Serial.println(" V"); 
       //number_0 = 0;
     }
     
     if (content.indexOf('I') == 0) {
       content.substring(2).toCharArray(floatbuf, sizeof(floatbuf));
       //Serial.print(content.substring(0,2)); 
       jSend["B_amps"] =  float(atof(floatbuf))/1000 - 0.21;
       //float number_0 = atof(floatbuf);
       //Serial.print(number_0/1000 - 0.21);
       //Serial.println(" A"); 
       //number_0 = 0;
     }
     
     if ((content.indexOf('P') == 0) && (content.indexOf('I')) != 1) {
       content.substring(2).toCharArray(floatbuf, sizeof(floatbuf));
       jSend["B_power"] = atoi(floatbuf);
       //Serial.print(content);  
       //Serial.println(" W");
     } 
     
     if ((content.indexOf('C') == 0) && (content.indexOf('h') != 1)) {
       content.substring(3).toCharArray(floatbuf, sizeof(floatbuf));
       //Serial.print(content.substring(0,3)); 
       jSend["B_ah"] = float(atof(floatbuf))/1000;
       //float number_0 = atof(floatbuf);
       //Serial.print(number_0/1000);
       //Serial.println(" Ah"); 
       //number_0 = 0;
     }
     
     if (content.indexOf('S') == 0) {
       content.substring(4).toCharArray(floatbuf, sizeof(floatbuf));
       //Serial.print(content.substring(0,4)); 
       jSend["B_soc"] = float(atof(floatbuf))/10;
       //float number_0 = atof(floatbuf);
       //Serial.print(number_0/10);
       //Serial.println(" %"); 
       //number_0 = 0;
     }
       
     if (content.indexOf('T') == 0) {
     content.substring(4).toCharArray(floatbuf, sizeof(floatbuf));
     jSend["B_ttg"] = atoi(floatbuf);  
//     Serial.print(content);  
//     Serial.println(" min");
     ADCread = true;
     } 
       
     
  content = "";
 
}
  jSend.prettyPrintTo(Serial);
  Serial.println("EOL");
}
