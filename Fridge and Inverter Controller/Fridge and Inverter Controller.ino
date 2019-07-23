/*------------------------------------------------------------------------------
Arduino sketch for ESP8285 board to turn an AC fridge on/off - will cycle inverter as needed to conserve power
------------------------------------------------------------------------------*/

#include <ESP8266WiFi.h>
#include <Ticker.h>
#include <OneWire.h> // for temp sensor
#include <DallasTemperature.h> //for temp sensor
#include <PubSubClient.h> // FOR MQTT Update

// Data wire is plugged into port 2 on the Arduino
#define ONE_WIRE_BUS 13 //temp sensor is hooked up to pin 13 

//Temp sensor power pin - provides power to temp sensor. Can by cycled to stop from heating fridge
#define Temp_Sensor_Power_Pin 15



// MQTT Setup stuff
#define MQTT_TOPIC_READINGS_TEMP "Fridge/Temp"
#define MQTT_TOPIC_READINGS_AC_STATUS "Inverter/AC"
#define MQTT_TOPIC_READINGS_AC_OVERRIDE_STATUS "Inverter/AC_Override"
#define MQTT_TOPIC_STATE "Fridge/Status"
#define MQTT_TEMP_DELAY 10 //temp reading delay in seconds - delay for how often to get the temp and how often to publish an update
#define MQTT_CLIENT_ID "FridgeController"

//Define Pins
#define INVERTER_RELAY_PIN 12
#define CompressorRelayPin 14
#define Inverter_Override_Pin 5




// Connecting to the Internet
char * ssid = "MooseS5";
char * password = "potatoes";

//MQTT Stuff
const char *MQTT_SERVER = "192.168.43.120"; //192.168.143.122 is what it should be
const char *MQTT_USER = ""; // NULL for no authentication
const char *MQTT_PASSWORD = ""; // NULL for no authentication
WiFiClient espClient;
PubSubClient mqttClient(espClient);
Ticker MQTT_Publish_Updates_Timer;
#define MQTT_Publish_Updates_Timer_Delay 10 //sets the timer to try and reconnect to MQTT if connection is lost every 20 seconds
Ticker MQTT_Reconnect_Timer;
#define MQTT_Reconnect_Timer_Delay 10
bool Mqtt_Reconnect_Timer = 0;

//Temperature Sensor Stuff
byte Temp_Sensor_Counter = 0; //a counter to keep track of the temp sensor reading progress
float temperature = 0; //start of with a low value so we don't accidentally cycle the whole system

Ticker Temp_Sensor_Timer;
#define Temp_Sensor_Timer_Delay 10 //sets the system to check temperature every X seconds 

OneWire oneWire(ONE_WIRE_BUS); // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
DallasTemperature sensors(&oneWire); // Pass our oneWire reference to Dallas Temperature.

//Fridge Control Stuff
int TempSetting = 90 ;
int Hysteresis = 5 ;
byte CompressorCoolDown = 0; //bool to store whether the compressor is in cool down mode
Ticker Compressor_CoolDown_Timer;
bool CoolingActive = 0;
int CompressorDelay = 5*60; //(5*60); 5 minute cooldown timer in seconds

//AC Detection Stuff
  bool ACPowerStatus = 0;
  byte WirelessACReading = 130; //set to defaultish value at startup
  byte WirelessACReadingMax = WirelessACReading;
  byte WirelessACReadingMin = WirelessACReading;
  byte WirelessACReadingDifferential1 = 0;
  byte WirelessACReadingDifferential2 = 0;
  byte WirelessACReadingDifferential3 = 0;
  byte WirelessACReadingDifferentialAVG = 0;

//Inverter Override
bool InverterOverride = 0; //set override to off upon startup just in case
Ticker Inverter_Override_Check_Timer;

//Inverter Power Cycling
bool Inverter_Power_Switch = 0; //bool to store whether the status of the power switch timer
Ticker Inverter_Power_Switch_Timer;
#define Inverter_Power_Switch_Delay 500; //define the power switch delay as 500 milliseconds

void setupWifi() {
              float TimeToConnect =millis();
              Serial.print("Connecting to ");
              Serial.println(ssid);
            
            
              WiFi.begin(ssid, password);
            
              while (WiFi.status() != WL_CONNECTED) {
                delay(500);
                Serial.print(".");
              }
              Serial.println("");
              Serial.print("Time To Connect: ");
              Serial.println(millis()-TimeToConnect);
              Serial.println("WiFi connected");
              Serial.print("IP address: ");
              Serial.println(WiFi.localIP());
            }

void MQTT_Reconnect_Timer_Flip() {
	Mqtt_Reconnect_Timer = 0; // reset timer to zero
	Serial.println("MQTT Reconnect timeout expired, allowing system to try to reconnect");
	MQTT_Reconnect_Timer.detach(); // detaches timer so it doesn't happen again....

}

void mqttReconnect() {
  Serial.print("Attempting MQTT connection...");

    // Attempt to connect
    
    if (mqttClient.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASSWORD, MQTT_TOPIC_STATE, 1, true, "disconnected", false)) {
      Serial.println("MQTT reconnection successfull");

      // Once connected, publish an announcement...
      mqttClient.publish(MQTT_TOPIC_STATE, "connected", true);
    } else {
     // Serial.println("Connection Failed - will try again upon next scheduled mqtt broadcast");
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
	  Serial.print(" try again in ");
	  Serial.print(MQTT_Reconnect_Timer_Delay);
	  Serial.println(" seconds");
      Mqtt_Reconnect_Timer = 1;
	  MQTT_Reconnect_Timer.attach(MQTT_Reconnect_Timer_Delay, MQTT_Reconnect_Timer_Flip);
	  Serial.println("Failed to reconnect, starting MQTT Reconnect timeout");
	  } 
    
 }
  


void mqttPublish(char *topic, float payload) {
  Serial.print(topic);
  Serial.print(": ");
  Serial.println(payload);

  mqttClient.publish(topic, String(payload).c_str(), true);
}

          void CyclePower() {
            Serial.println("Cycling invter power relay");
            digitalWrite(INVERTER_RELAY_PIN, HIGH);
            delay(800);
            digitalWrite(INVERTER_RELAY_PIN, LOW);
            delay(800);
                
          }

          void Compressor_CoolDown() { //Compressor Cooldown Function  - cooldown must be set to 0 before initializing so it knows to start the timer.
            Serial.println("Compressor_CoolDown Function beginnnig");
            Serial.print("             Timestamp: ");
            Serial.println(millis()/1000);
          
            CompressorCoolDown = 0; //set to zero
            Serial.println(" Cooldown set to 0");
          
            Compressor_CoolDown_Timer.detach(); //detaches timer
            Serial.println("Cool Down Timer Detached");
            Serial.println("Compressor_CoolDown Function end");
          
          }

void MQTT_Publish_Updates() {   //--------------------------------------------------------------------------------------
  Serial.println("MQTT_Publish_Updates initiated");
  
  if (!mqttClient.connected()) { //if connection to MQTT client is lost, reconnect
    
	Serial.println("MQTT Connection lost - skipping mqtt publish");
   }
  else {
  Serial.println("MQTT Connected - Publishing MQTT Updates");
  mqttPublish(MQTT_TOPIC_READINGS_TEMP, temperature);
    mqttPublish(MQTT_TOPIC_READINGS_AC_STATUS, ACPowerStatus);
    mqttPublish(MQTT_TOPIC_READINGS_AC_OVERRIDE_STATUS, InverterOverride);

	// Just for debugging, remove when done
	//mqtt
  }
}

void Temp_Sensor() {     //--------------------------------------------------------------------------------------------------------
   if (Temp_Sensor_Counter == 0) {
    //Serial.println("Turning on Temp Sensor to take reading");
     digitalWrite(Temp_Sensor_Power_Pin, HIGH); // turn on the sensor - then wait 500ms before taking the reading, then shut if off
     Temp_Sensor_Counter = 1;
     Temp_Sensor_Timer.attach(0.5,Temp_Sensor); // set this to rerun in half a second
     }
   if (Temp_Sensor_Counter == 1) {
    Temp_Sensor_Counter = 0;
    //Serial.println("Getting reading from Temp Sensor");
     sensors.requestTemperatures(); // Send the command to get temperatures
     temperature = sensors.getTempFByIndex(0);
    // Serial.print("Temperature:  ");
    // Serial.print(temperature);
    // Serial.println("   Temp Reading aquired, powering down temp sensor");
     digitalWrite(Temp_Sensor_Power_Pin, LOW); //turn off power to sensor to keep from heating up sensor
     Temp_Sensor_Timer.attach(Temp_Sensor_Timer_Delay,Temp_Sensor); // set this to rerun in half a second
   }
       // Publishing sensor data
    
   if (!mqttClient.connected()) {
	   Serial.println("Mqtt disconnected so publishing here - ");
	   Serial.print("Temp reading: ");
	   Serial.print(temperature);
	   Serial.println
	   (" F");
   }
 
}



              void CheckPowerStatus(){ 
                //For Wireless power detection
                //checks power status by comparing the reading differentials of 100 readings (max-min) on three separate attempts and averages them
                //if it is over a certain threshold it will say power is on or OFF
              
              
                WirelessACReading = analogRead(A0);
                WirelessACReadingMax = WirelessACReading;
                WirelessACReadingMin = WirelessACReading;
                WirelessACReadingDifferential1 = 0;
                WirelessACReadingDifferential2 = 0;
                WirelessACReadingDifferential3 = 0;
                WirelessACReadingDifferentialAVG = 0;
                 
                for (byte i = 0; i <= 100; i++) {
                  WirelessACReading = analogRead(A0);
                      if (WirelessACReading > WirelessACReadingMax)
                      { WirelessACReadingMax = WirelessACReading;}
                    else if (WirelessACReading < WirelessACReadingMin) 
                     { WirelessACReadingMin = WirelessACReading;}
                  }
                  
                WirelessACReadingDifferential1 = WirelessACReadingMax - WirelessACReadingMin;  
              
                WirelessACReading = analogRead(A0);
                WirelessACReadingMax = WirelessACReading;
                WirelessACReadingMin = WirelessACReading;
                for (byte i = 0; i <= 100; i++) {
                  WirelessACReading = analogRead(A0);
                      if (WirelessACReading > WirelessACReadingMax)
                      { WirelessACReadingMax = WirelessACReading;}
                    else if (WirelessACReading < WirelessACReadingMin) 
                     { WirelessACReadingMin = WirelessACReading;}
                  }
              WirelessACReadingDifferential2 = WirelessACReadingMax - WirelessACReadingMin;  
              
                WirelessACReading = analogRead(A0);
                WirelessACReadingMax = WirelessACReading;
                WirelessACReadingMin = WirelessACReading;
                  for (byte i = 0; i <= 100; i++) {
                  WirelessACReading = analogRead(A0);
                      if (WirelessACReading > WirelessACReadingMax)
                      { WirelessACReadingMax = WirelessACReading;}
                    else if (WirelessACReading < WirelessACReadingMin) 
                     { WirelessACReadingMin = WirelessACReading;}
                  }
              WirelessACReadingDifferential3 = WirelessACReadingMax - WirelessACReadingMin;  
                
                 //Sets the reading as the average of two differential readings 
                WirelessACReadingDifferentialAVG = (WirelessACReadingDifferential1+WirelessACReadingDifferential2+WirelessACReadingDifferential3)/3;
                if ((WirelessACReadingDifferentialAVG) > 70) {
                    ACPowerStatus = HIGH;}
                  else {
                    ACPowerStatus = LOW;}
                    
                 // Serial.print("Low Reading = ");
                 // Serial.print(WirelessACReadingMin);
                 // Serial.print("       High Reading = ");
                 // Serial.print(WirelessACReadingMax);
                  Serial.print("     Differential = ");
                  Serial.println(WirelessACReadingDifferentialAVG);
                  
                Serial.print("Current Power Status = ");
                Serial.println(ACPowerStatus);
              }

        void StartCooling(){ // function that starts the fridge cooling cycle
          Serial.println("Starting Cooling Cycle");
         CheckPowerStatus();
          if (ACPowerStatus == 0){ //if no AC power, try to turn it on
            Serial.println("AC Power off, turning on");
            for (byte i = 0; i < 4; i++) { // will try to turn on the AC Inverter up to 4 times
              CyclePower();
              CheckPowerStatus();
              if(ACPowerStatus == 1){ //Check if power cycling worked. If it did, exit loop
                  i = 0; //reset i for some reason?
                  Serial.println("AC Power succesfully turned ON - turning on Compressor");
                  CoolingActive = 1; //set cooling to active and begin cooling ops
                  digitalWrite(CompressorRelayPin, HIGH);
                  Serial.println("Compressor ON");
                  break; //exits for loop
                }   
              if (i == 3){
                Serial.println("Unable to turn on AC Power, aborting cooling cycle");
                CoolingActive = 0; //turn off the cooling active flag
                CompressorCoolDown = 1; //prime the cooldown flag to keep it from trying again for a couple minutes
                Serial.print("CompressorCoolDown = ");
                Serial.println(CompressorCoolDown);
                Compressor_CoolDown_Timer.attach(CompressorDelay,Compressor_CoolDown); // start the cooldown timer. Since it flips the current digit, you start with 0 not 1
                Serial.print("             Timestamp: ");
                Serial.println(millis()/1000);
                Serial.println("attached compressor cooldown timer");
                break; //exit for loop
                }
            }
          }
          if (ACPowerStatus == 1){
            Serial.println("Inverter on, initializing compressor");
            CoolingActive = 1;
            digitalWrite(CompressorRelayPin, HIGH);
          }
        }

        void StopCooling() { //function to turn off compressor and AC Power
         Serial.println("Ending Cooling Cycle");
          
          CoolingActive = 0;
          CompressorCoolDown = 1; //prime the cooldown timer
          Compressor_CoolDown_Timer.attach(CompressorDelay,Compressor_CoolDown);
          Serial.print("             Timestamp: ");
          Serial.println(millis()/1000);
          Serial.println("attached compressor cooldown timer");
          CheckPowerStatus(); //check to see if inverter is on
          if (ACPowerStatus == 1 && InverterOverride == 0){ // if inverter is on, turn it off now that compressor is off
            Serial.println("Turning AC Off");
            for (byte i = 0; i <= 4; i++) { // will try to turn off the AC Inverter up to 4 times
              CyclePower();
              CheckPowerStatus();
              if(ACPowerStatus == 0){ //Check if power cycling worked. If it did, exit loop
                Serial.println("AC Power Off");
                break; //exit for loop and stop trying to turn the inverter off since it's already off
                }   
              }
            }
		  digitalWrite(CompressorRelayPin, LOW); // turn off compressor AFTER turning off AC power - otherwise ac voltage will fluctuate  and it won't properly sense if it turned off the power
        }

        void Inverter_Override_Check(){ //--- Timer that is run once every X seconds to check if inverter override switch is on
          if (InverterOverride != digitalRead(Inverter_Override_Pin)) {
              Serial.print("Inverter Override = ");
              InverterOverride = !InverterOverride ; //inverts current reading
              Serial.println(InverterOverride);
          }
        }

void setup() {
//Pin Setup
pinMode(CompressorRelayPin, OUTPUT);
pinMode(Temp_Sensor_Power_Pin, OUTPUT);
pinMode(Inverter_Override_Pin, INPUT);
//pinMode(LED_BUILTIN, OUTPUT);  // I don't use this led so ..


/************************ Inverter relay pin ***************/
pinMode(INVERTER_RELAY_PIN, OUTPUT);
digitalWrite(INVERTER_RELAY_PIN, LOW);
digitalWrite(CompressorRelayPin, LOW);
digitalWrite(Temp_Sensor_Power_Pin, HIGH);  // gives power to temp sensor to take temp readings
  // put your setup code here, to run once:
  WiFi.softAPdisconnect (true);
  //WiFi.persistent( false );
  Serial.begin(115200);
  setupWifi(); //calls setup function below

//MQTT Stuff
mqttClient.setServer(MQTT_SERVER, 1883);
MQTT_Publish_Updates_Timer.attach(MQTT_Publish_Updates_Timer_Delay,MQTT_Publish_Updates);

//startup the inverter override switch check
Inverter_Override_Check_Timer.attach(2,Inverter_Override_Check);

   
// Start up the temp sensor library
  sensors.begin();
  Temp_Sensor_Timer.attach(Temp_Sensor_Timer_Delay,Temp_Sensor);  //----------------------------------------------------------------------

//initiate connection to mqtt
Serial.println("initiating mqtt connection before beginning loop");
mqttReconnect();


}


void loop() {


if (!mqttClient.connected() && Mqtt_Reconnect_Timer == 0) { //if connection to MQTT client is lost AND the reconnect timeout hasn't initiated, reconnect
	Serial.println("MQTT disconnected - Timeout not engaged - calling mqttReconnect() function");
	mqttReconnect();
  }
  

if (mqttClient.connected()) { //if connection to MQTT client is lost, reconnect
   // Serial.println (" connection enabled, running mqtt loop");
    mqttClient.loop();
  }

  

if (temperature > TempSetting && CompressorCoolDown == 0 && CoolingActive == 0) { // if it gets too hot and it isn't already cooling or in timeout, start cooling
    Serial.println("Temp too high, begin cooling cycle");
    StartCooling();
    
  }

if (temperature <= (TempSetting-Hysteresis) && CoolingActive == 1){ // turn off cooling and AC Inverter
  Serial.println("Cooling succesful, ending cooling cycle");
    StopCooling();
  }
  
}