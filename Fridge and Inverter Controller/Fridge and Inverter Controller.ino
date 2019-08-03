/*------------------------------------------------------------------------------
Arduino sketch for ESP8285 board to turn an AC fridge on/off - will cycle inverter as needed to conserve power
------------------------------------------------------------------------------*/

#include <ESP8266WiFi.h>
#include <Ticker.h>
#include <OneWire.h> // for temp sensor
#include <DallasTemperature.h> //for temp sensor
#include <PubSubClient.h> // FOR MQTT Update



// MQTT Setup stuff
#define MQTT_TOPIC_READINGS_TEMP "Fridge/Temp"         // Use DEFINE instead of CONST because of how the mqtt function works
#define MQTT_TOPIC_READINGS_AC_STATUS "Inverter/AC"
#define MQTT_TOPIC_READINGS_AC_OVERRIDE_STATUS "Inverter/AC_Override"
#define MQTT_TOPIC_STATE "Fridge/Status"
#define MQTT_TEMP_DELAY 10 //temp reading delay in seconds - delay for how often to get the temp and how often to publish an update
#define MQTT_CLIENT_ID "FridgeController"

#define kMqttIncomingMessage  "Inverter/AC_Override/Switch"

//Define Pins
const int kOneWireBus = 13;          //temp sensor is hooked up to pin 13 
const int kTempSensorPowerPin = 15;  //Power to the temp sensor is sent via this pin so it can be turned on/off
const int kInverterRelayPin = 12;
const int kCompressorRelayPin = 14;
const int kInverterOverridePin = 5;




// Connecting to the Internet
char * ssid = "MooseS5";
char * password = "potatoes";

//MQTT Stuff
const char *MQTT_SERVER = "192.168.43.120"; //192.168.143.122 is what it should be
const char *MQTT_USER = ""; // NULL for no authentication
const char *MQTT_PASSWORD = ""; // NULL for no authentication
WiFiClient espClient;
PubSubClient mqttClient(espClient);  //we are calling the client we will connect to with pubsubclient as "mqttclient"
Ticker MQTT_Publish_Updates_Timer;
const int kMqttPublishUpdatesTimerDelaySeconds = 10; //sets the timer to try and reconnect to MQTT if connection is lost every 20 seconds
Ticker MQTT_Reconnect_Timer;
const int kMqttReconnectTimerDelaySeconds = 10;
bool mqtt_reconnect_timer_seconds = 0;

//Temperature Sensor Stuff
bool temp_sensor_counter = 0; //a counter to keep track of the temp sensor reading progress
float temperature = 0;        //start of with a low value so we don't accidentally cycle the whole system

Ticker Temp_Sensor_Timer;
const int kTempSensorTimerDelaySeconds = 10; //sets the system to check temperature every X seconds 

OneWire oneWire(kOneWireBus);        // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
DallasTemperature sensors(&oneWire); // Pass our oneWire reference to Dallas Temperature.

//Fridge Control Stuff
byte temp_setting = 90 ;
byte hysteresis = 5 ;
bool compressor_cooldown = 0; //bool to store whether the compressor is in cool down mode
Ticker Compressor_CoolDown_Timer;
bool cooling_active = 0;
int compressor_delay = 5*60; //(5*60); 5 minute cooldown timer in seconds

//AC Detection Stuff
  bool ac_power_status = 0;
  byte wireless_ac_reading = 130; //set to defaultish value at startup
  byte wireless_ac_reading_max = wireless_ac_reading;
  byte wireless_ac_reading_min = wireless_ac_reading;
  byte wireless_ac_reading_differential_1 = 0;
  byte wireless_ac_reading_differential_2 = 0;
  byte wireless_ac_reading_differential_3 = 0;
  byte wireless_ac_reading_differential_avg = 0;

//Inverter Override
bool inverter_override = 0; //set override to off upon startup just in case
Ticker Inverter_Override_Check_Timer;

//Inverter Power Cycling
bool inverter_power_switch = 0; //bool to store whether the status of the power switch timer
Ticker Inverter_Power_Switch_Timer;
const int kInverterPowerSwitchDelayMilliSeconds = 800; //define the power switch delay as 500 milliseconds

void SetupWifi() {
              float TimeToConnect =millis();
              Serial.print("Connecting to ");
              Serial.println(ssid);
            
            
              WiFi.begin(ssid, password);
			  byte i = 0;
              while (WiFi.status() != WL_CONNECTED) {
                delay(500);
                Serial.print(".");
				i++;
				if (i > 60) { 
					Serial.println("Failed to connect to wifi"); 
					break; }  //if it still hasn't connected to wifi in 30 seconds, give up 
              }
              Serial.println("");
              Serial.print("Time To Connect: ");
              Serial.println(millis()-TimeToConnect);
              Serial.println("WiFi connected");
              Serial.print("IP address: ");
              Serial.println(WiFi.localIP());
            }

void MqttReconnectTimerFlip() {
	mqtt_reconnect_timer_seconds = 0; // reset timer to zero
	Serial.println("MQTT Reconnect timeout expired, allowing system to try to reconnect");
	MQTT_Reconnect_Timer.detach(); // detaches timer so it doesn't happen again....

}

void MqttReconnect() {
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
	  Serial.print(kMqttReconnectTimerDelaySeconds);
	  Serial.println(" seconds");
      mqtt_reconnect_timer_seconds = 1;
	  MQTT_Reconnect_Timer.attach(kMqttReconnectTimerDelaySeconds, MqttReconnectTimerFlip);
	  Serial.println("Failed to reconnect, starting MQTT Reconnect timeout");
	  } 
    
 }
  

void MqttPublish(char *topic, float payload) {
  Serial.print(topic);
  Serial.print(": ");
  Serial.println(payload);

  mqttClient.publish(topic, String(payload).c_str(), true);
}

void CyclePower() {
	Serial.println("Cycling invter power relay");
	digitalWrite(kInverterRelayPin, HIGH);
	delay(kInverterPowerSwitchDelayMilliSeconds);
	digitalWrite(kInverterRelayPin, LOW);
	delay(kInverterPowerSwitchDelayMilliSeconds);
             
 }

void CompressorCoolDownTimer() { //Compressor Cooldown Function  - cooldown must be set to 0 before initializing so it knows to start the timer.
            Serial.println("Compressor_CoolDown Function beginnnig");
            Serial.print("             Timestamp: ");
            Serial.println(millis()/1000);
          
            compressor_cooldown = 0; //set to zero
            Serial.println(" Cooldown set to 0");
          
            Compressor_CoolDown_Timer.detach(); //detaches timer
            Serial.println("Cool Down Timer Detached");
            Serial.println("Compressor_CoolDown Function end");
          
          }

void MqttPublishUpdates() {   //--------------------------------------------------------------------------------------
  Serial.println("MQTT_Publish_Updates initiated");
  
  if (!mqttClient.connected()) { //if connection to MQTT client is lost, reconnect
    
	Serial.println("MQTT Connection lost - skipping mqtt publish");
   }
  else {
  Serial.println("MQTT Connected - Publishing MQTT Updates");
  MqttPublish(MQTT_TOPIC_READINGS_TEMP, temperature);
    MqttPublish(MQTT_TOPIC_READINGS_AC_STATUS, ac_power_status);
    MqttPublish(MQTT_TOPIC_READINGS_AC_OVERRIDE_STATUS, inverter_override);

	// Just for debugging, remove when done
	//mqtt
  }
}

void GetTempReading() {     // Function to get temperature reading
   if (temp_sensor_counter == 0) {              // first step is to power on the sensor, wait half a second for it to boot up
     digitalWrite(kTempSensorPowerPin, HIGH);   // turn on the sensor 
     temp_sensor_counter = 1;				    // sets counter HIGH so we now power is now on so proceed to 2nd step upon next timer call
     Temp_Sensor_Timer.attach(0.5,GetTempReading); // set this to rerun in half a second
     }
   if (temp_sensor_counter == 1) {            //Sensor has now had time to initialize, so this will take the reading and shutoff the timer
    temp_sensor_counter = 0;                  //Reset counter so upon next call of this function it'll start over again
    sensors.requestTemperatures();            // Request temperature reading from sensor
    temperature = sensors.getTempFByIndex(0);
    digitalWrite(kTempSensorPowerPin, LOW);    //turn off power to sensor to keep from heating up sensor
    Temp_Sensor_Timer.attach(kTempSensorTimerDelaySeconds,GetTempReading); // Restart the timer to run at the standard temp reading interval
   }
   
   
   // Publishing sensor data to serial if MQTT disconneted
    
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
              
              
                wireless_ac_reading = analogRead(A0);
                wireless_ac_reading_max = wireless_ac_reading;
                wireless_ac_reading_min = wireless_ac_reading;
                wireless_ac_reading_differential_1 = 0;
                wireless_ac_reading_differential_2 = 0;
                wireless_ac_reading_differential_3 = 0;
                wireless_ac_reading_differential_avg = 0;
                 
                for (byte i = 0; i <= 100; i++) {
                  wireless_ac_reading = analogRead(A0);
                      if (wireless_ac_reading > wireless_ac_reading_max)
                      { wireless_ac_reading_max = wireless_ac_reading;}
                    else if (wireless_ac_reading < wireless_ac_reading_min) 
                     { wireless_ac_reading_min = wireless_ac_reading;}
                  }
                  
                wireless_ac_reading_differential_1 = wireless_ac_reading_max - wireless_ac_reading_min;  
              
                wireless_ac_reading = analogRead(A0);
                wireless_ac_reading_max = wireless_ac_reading;
                wireless_ac_reading_min = wireless_ac_reading;
                for (byte i = 0; i <= 100; i++) {
                  wireless_ac_reading = analogRead(A0);
                      if (wireless_ac_reading > wireless_ac_reading_max)
                      { wireless_ac_reading_max = wireless_ac_reading;}
                    else if (wireless_ac_reading < wireless_ac_reading_min) 
                     { wireless_ac_reading_min = wireless_ac_reading;}
                  }
              wireless_ac_reading_differential_2 = wireless_ac_reading_max - wireless_ac_reading_min;  
              
                wireless_ac_reading = analogRead(A0);
                wireless_ac_reading_max = wireless_ac_reading;
                wireless_ac_reading_min = wireless_ac_reading;
                  for (byte i = 0; i <= 100; i++) {
                  wireless_ac_reading = analogRead(A0);
                      if (wireless_ac_reading > wireless_ac_reading_max)
                      { wireless_ac_reading_max = wireless_ac_reading;}
                    else if (wireless_ac_reading < wireless_ac_reading_min) 
                     { wireless_ac_reading_min = wireless_ac_reading;}
                  }
              wireless_ac_reading_differential_3 = wireless_ac_reading_max - wireless_ac_reading_min;  
                
                 //Sets the reading as the average of two differential readings 
                wireless_ac_reading_differential_avg = (wireless_ac_reading_differential_1+wireless_ac_reading_differential_2+wireless_ac_reading_differential_3)/3;
                if ((wireless_ac_reading_differential_avg) > 70) {
                    ac_power_status = HIGH;}
                  else {
                    ac_power_status = LOW;}
                    
                 // Serial.print("Low Reading = ");
                 // Serial.print(WirelessACReadingMin);
                 // Serial.print("       High Reading = ");
                 // Serial.print(WirelessACReadingMax);
                  Serial.print("     Differential = ");
                  Serial.println(wireless_ac_reading_differential_avg);
                  
                Serial.print("Current Power Status = ");
                Serial.println(ac_power_status);
              }

void StartCooling(){ // function that starts the fridge cooling cycle
          Serial.println("Starting Cooling Cycle");
         CheckPowerStatus();
          if (ac_power_status == 0){ //if no AC power, try to turn it on
            Serial.println("AC Power off, turning on");
            for (byte i = 0; i < 4; i++) { // will try to turn on the AC Inverter up to 4 times
              CyclePower();
              CheckPowerStatus();
              if(ac_power_status == 1){ //Check if power cycling worked. If it did, exit loop
                  i = 0; //reset i for some reason?
                  Serial.println("AC Power succesfully turned ON - turning on Compressor");
                  cooling_active = 1; //set cooling to active and begin cooling ops
                  digitalWrite(kCompressorRelayPin, HIGH);
                  Serial.println("Compressor ON");
                  break; //exits for loop
                }   
              if (i == 3){
                Serial.println("Unable to turn on AC Power, aborting cooling cycle");
                cooling_active = 0; //turn off the cooling active flag
                compressor_cooldown = 1; //prime the cooldown flag to keep it from trying again for a couple minutes
                Serial.print("CompressorCoolDown = ");
                Serial.println(compressor_cooldown);
                Compressor_CoolDown_Timer.attach(compressor_delay,CompressorCoolDownTimer); // start the cooldown timer. Since it flips the current digit, you start with 0 not 1
                Serial.print("             Timestamp: ");
                Serial.println(millis()/1000);
                Serial.println("attached compressor cooldown timer");
                break; //exit for loop
                }
            }
          }
          if (ac_power_status == 1){
            Serial.println("Inverter on, initializing compressor");
            cooling_active = 1;
            digitalWrite(kCompressorRelayPin, HIGH);
          }
        }

void StopCooling() { //function to turn off compressor and AC Power
         Serial.println("Ending Cooling Cycle");
          
          cooling_active = 0;
          compressor_cooldown = 1; //prime the cooldown timer
          Compressor_CoolDown_Timer.attach(compressor_delay,CompressorCoolDownTimer);
          Serial.print("             Timestamp: ");
          Serial.println(millis()/1000);
          Serial.println("attached compressor cooldown timer");
          CheckPowerStatus(); //check to see if inverter is on
          if (ac_power_status == 1 && inverter_override == 0){ // if inverter is on, turn it off now that compressor is off
            Serial.println("Turning AC Off");
            for (byte i = 0; i <= 4; i++) { // will try to turn off the AC Inverter up to 4 times
              CyclePower();
              CheckPowerStatus();
              if(ac_power_status == 0){ //Check if power cycling worked. If it did, exit loop
                Serial.println("AC Power Off");
                break; //exit for loop and stop trying to turn the inverter off since it's already off
                }   
              }
            }
		  digitalWrite(kCompressorRelayPin, LOW); // turn off compressor AFTER turning off AC power - otherwise ac voltage will fluctuate  and it won't properly sense if it turned off the power
        }

void InverterOverrideCheck(){ //--- Timer that is run once every X seconds to check if inverter override switch is on
          if (inverter_override != digitalRead(kInverterOverridePin)) {
              Serial.print("Inverter Override = ");
              inverter_override = !inverter_override ; //inverts current reading
              Serial.println(inverter_override);
          }
        }

void callback(char* topic, byte* payload, unsigned int length) {

	Serial.print("Message arrived in topic: ");
	Serial.println(topic);

	Serial.print("Message:");
	for (int i = 0; i < length; i++) {
		Serial.print((char)payload[i]);
	}

	Serial.println();
	Serial.println("-----------------------");

} 

void setup() {
//Pin Setup
pinMode(kCompressorRelayPin, OUTPUT);
pinMode(kTempSensorPowerPin, OUTPUT);
pinMode(kInverterOverridePin, INPUT);
//pinMode(LED_BUILTIN, OUTPUT);  // I don't use this led so ..


/************************ Inverter relay pin ***************/
pinMode(kInverterRelayPin, OUTPUT);
digitalWrite(kInverterRelayPin, LOW);
digitalWrite(kCompressorRelayPin, LOW);
digitalWrite(kTempSensorPowerPin, HIGH);  // gives power to temp sensor to take temp readings
  // put your setup code here, to run once:
  WiFi.softAPdisconnect (true);
  //WiFi.persistent( false );
  Serial.begin(115200);
  SetupWifi(); //calls setup function below

//MQTT Stuff
mqttClient.setServer(MQTT_SERVER, 1883);  //sets server location for PubSubClient library will be used to connect
mqttClient.subscribe(kMqttIncomingMessage);
MQTT_Publish_Updates_Timer.attach(kMqttPublishUpdatesTimerDelaySeconds,MqttPublishUpdates);
mqttClient.setCallback(callback);

//startup the inverter override switch check
Inverter_Override_Check_Timer.attach(2,InverterOverrideCheck);

   
// Start up the temp sensor library
  sensors.begin();
  Temp_Sensor_Timer.attach(kTempSensorTimerDelaySeconds,GetTempReading);  

//initiate connection to mqtt
Serial.println("initiating mqtt connection before beginning loop");
MqttReconnect();


}


void loop() {


if (!mqttClient.connected() && mqtt_reconnect_timer_seconds == 0) { //if connection to MQTT client is lost AND the reconnect timeout hasn't initiated, reconnect
	Serial.println("MQTT disconnected - Timeout not engaged - calling mqttReconnect() function");
	MqttReconnect();
  }
  

if (mqttClient.connected()) { //if connection to MQTT client is lost, reconnect
   // Serial.println (" connection enabled, running mqtt loop");
    mqttClient.loop();
  }

  

if (temperature > temp_setting && compressor_cooldown == 0 && cooling_active == 0) { // if it gets too hot and it isn't already cooling or in timeout, start cooling
    Serial.println("Temp too high, begin cooling cycle");
    StartCooling();
    
  }

if (temperature <= (temp_setting-hysteresis) && cooling_active == 1){ // turn off cooling and AC Inverter
  Serial.println("Cooling succesful, ending cooling cycle");
    StopCooling();
  }
  
}