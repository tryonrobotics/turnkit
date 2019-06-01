#include <FS.h>                   //this needs to be first, or it all crashes and burns...
//#define BLYNK_DEBUG           // Comment this out to disable debug and save space
#define BLYNK_PRINT Serial    // Comment this out to disable prints and save space
#include <ESP8266WiFi.h>
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager
#include <SPI.h>
//for LED status
#include <Ticker.h>
Ticker ticker;
#include <BlynkSimpleEsp8266.h>
#include <ArduinoJson.h>          //https://github.com/bblanchon/ArduinoJson
//#include <BlynkSimpleEthernet.h>

/////////////////////////////
//  VARIABLE DECLARATIONS
/////////////////////////////

int powerStatus = 0;
int ignitionTimer = 0;
int shutdownTimer = 0;
int currentSetpoint = 0;
int grillTemp = 0;
int meatTemp = 0;

//  DEFAULT BLYK TOKEN  
char blynk_token[34] = "43d1f01c97d640f689cea2022aafc3e5";  // Blynk Token for TurnKit3.0
// Josh 43d1f01c97d640f689cea2022aafc3e5
// Keith 9c2d18810bbd4dedb4e6240e25256ca9

//////////////////////////////
//  END VARIABLE DECLARATIONS
//////////////////////////////

/////////////////////////
//  TurnKit Functions  //
/////////////////////////

// if serial is available, the following function is called
// Read the serial data, and parse it out into different variables
// powerStatus, ignitionTimer, shutdownTimer, currentSetpoint, grillTemp, meatTemp
// 
// Every second, the above variables will be trasmitted out to the app for display on phone
//
void readSerialUpdate() {
   if (Serial.find('C')) {
 //  powerStatus = Serial.parseInt();
  // ignitionTimer = Serial.parseInt();
 //  shutdownTimer = Serial.parseInt();
   currentSetpoint = Serial.parseInt();
   grillTemp = Serial.parseInt();
 //  meatTemp = Serial.parseInt();

  }
}


// This function is called automatically by the timer routine, once every second
// DO NOT SEND MORE THAN 10 VALUES PER SECOND - OR WILL BE BANNED FROM BLYNK SERVER
//
void sendUpdateStatus(){
  readSerialUpdate();
      Blynk.virtualWrite(V10, 255);
  switch (powerStatus) {
    case 0:
 //     Blynk.setProperty(V10, "color", "#FF5733");
 //     Blynk.setProperty(V10, "label", "OFF");
 //     Blynk.setProperty(V11, "label", " ");
 //     Blynk.virtualWrite(V11, " ");
      break;
    case 1:
 //     Blynk.setProperty(V10, "color", "#FFF033");
 //     Blynk.setProperty(V10, "label", "Ignition");
 //     Blynk.setProperty(V11, "label", "Startup Seconds");
 //     Blynk.virtualWrite(V11, ignitionTimer);
      break;
    case 2:
  //    Blynk.setProperty(V10, "color", "#00E607");
  //    Blynk.setProperty(V10, "label", "Running");
  //    Blynk.setProperty(V11, "label", " ");
  //    Blynk.virtualWrite(V11, " ");
      break;
    case 3:
 //     Blynk.setProperty(V10, "color", "#46a4fc");
  //    Blynk.setProperty(V10, "label", "Shutdown");
  //    Blynk.setProperty(V11, "label", "Cool Down Seconds");
  //    Blynk.virtualWrite(V11, shutdownTimer);
      break;
  }

  Blynk.virtualWrite(V13, currentSetpoint);   // sends the current Grill Setpoint to the App

  Blynk.virtualWrite(V14, grillTemp);    // sends  value (grill temp) up to the App

 // Blynk.virtualWrite(V15, meatTemp);    // sends TempB value (meat temp) up to the App
  
}


//  the following BLYNK_WRITE functions take incoming Virtual Pin data from app and send to serial
BLYNK_WRITE(V1)
{
  int pinValue = param.asInt();  // incoming FROM APP V1 assigned to local variable pinValue
  Serial.print(pinValue);        // sends local variable out serial to main controller
}
BLYNK_WRITE(V2)
{
  int pinValue = param.asInt();  // incoming FROM APP V2 assigned to local variable pinValue
  Serial.print(pinValue);        // sends local variable out serial to main controller
}

BLYNK_WRITE(V3)
{
  int pinValue = param.asInt();  // incoming FROM APP V3 assigned to local variable pinValue
  Serial.print(pinValue);        // sends local variable out serial to main controller
}




/////////////////////////////
//  END TurnKit Functions  //
/////////////////////////////

//////////////////////////////////////////
//////////////////////////////////////////
// BLYNK CODE - no need to update this  //
//////////////////////////////////////////
//////////////////////////////////////////
bool shouldSaveConfig = false; //flag for saving data

BlynkTimer timer;
WidgetTerminal terminal(V0);
BLYNK_WRITE(V0)
{

  // if you type "Marco" into Terminal Widget - it will respond: "Polo:"
  if (String("Marco") == param.asStr()) {
    terminal.println("You said: 'Marco'") ;
    terminal.println("I said: 'Polo'") ;
    Serial.println("1");
  } else {

    // Send it back
    terminal.print("You said:");
    terminal.write(param.getBuffer(), param.getLength());
    terminal.println();
  }

  // Ensure everything is sent
  terminal.flush();
}
void tick()
{
  //toggle state
  int state = digitalRead(BUILTIN_LED);  // get the current state of GPIO1 pin
  digitalWrite(BUILTIN_LED, !state);     // set pin to the opposite state
}
void saveConfigCallback () {  //callback notifying us of the need to save config
  Serial.println("Should save config");
  shouldSaveConfig = true;
  ticker.attach(0.2, tick);  // led toggle faster
}

/////////////////////
/////////////////////
// END BLYNK CODE  //
/////////////////////
/////////////////////

void setup()
{

  Serial.begin(9600);
  Serial.println();
  
  //set led pin as output
  pinMode(BUILTIN_LED, OUTPUT);
  // start ticker with 0.5 because we start in AP mode and try to connect
  ticker.attach(0.6, tick);

  //SPIFFS.format();    //clean FS, for testing
  Serial.println("Mounting FS...");    //read configuration from FS json

  if (SPIFFS.begin()) {
    Serial.println("Mounted file system");
    if (SPIFFS.exists("/config.json")) {
      //file exists, reading and loading
      Serial.println("Reading config file");
      File configFile = SPIFFS.open("/config.json", "r");
      if (configFile) {
        Serial.println("Opened config file");
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);
        DynamicJsonBuffer jsonBuffer;
        JsonObject& json = jsonBuffer.parseObject(buf.get());
        json.printTo(Serial);
        if (json.success()) {
          Serial.println("\nparsed json");

          strcpy(blynk_token, json["blynk_token"]);

        } else {
          Serial.println("Failed to load json config");
        }
      }
    }
  } else {
    Serial.println("Failed to mount FS");
  }
  //end read

  // The extra parameters to be configured (can be either global or just in the setup)
  // After connecting, parameter.getValue() will get you the configured value
  // id/name placeholder/prompt default length
  WiFiManagerParameter custom_blynk_token("blynk", "blynk token", blynk_token, 33);   // was 32 length
  
  Serial.println(blynk_token);

  //WiFiManager
  //Local intialization. Once its business is done, there is no need to keep it around
  WiFiManager wifiManager;

  wifiManager.setSaveConfigCallback(saveConfigCallback);   //set config save notify callback

  //set static ip
  // this is for connecting to Office router not GargoyleTest but it can be changed in AP mode at 192.168.4.1
  //wifiManager.setSTAStaticIPConfig(IPAddress(192,168,10,111), IPAddress(192,168,10,90), IPAddress(255,255,255,0));
  
  wifiManager.addParameter(&custom_blynk_token);   //add all your parameters here

 // wifiManager.resetSettings();  //reset settings - for testing

  //set minimu quality of signal so it ignores AP's under that quality
  //defaults to 8%
  //wifiManager.setMinimumSignalQuality();
  
  //sets timeout until configuration portal gets turned off
  //useful to make it all retry or go to sleep, in seconds
  wifiManager.setTimeout(600);   // 10 minutes to enter data and then Wemos resets to try again.

  //fetches ssid and pass and tries to connect, if it does not connect it starts an access point with the specified name
  //and goes into a blocking loop awaiting configuration
  if (!wifiManager.autoConnect("CentralHeatingAP", "MY123PWD")) {
    Serial.println("Failed to connect and hit timeout");
    delay(3000);
    //reset and try again, or maybe put it to deep sleep
    ESP.reset();
    delay(5000);
  }
  Serial.println("Connected Central Heating System :)");   //if you get here you have connected to the WiFi
  ticker.detach();
  //turn LED off
  digitalWrite(BUILTIN_LED, HIGH);

  strcpy(blynk_token, custom_blynk_token.getValue());    //read updated parameters

  if (shouldSaveConfig) {      //save the custom parameters to FS
    Serial.println("saving config");
    DynamicJsonBuffer jsonBuffer;
    JsonObject& json = jsonBuffer.createObject();
    json["blynk_token"] = blynk_token;

    File configFile = SPIFFS.open("/config.json", "w");
    if (!configFile) {
      Serial.println("Failed to open config file for writing");
    }

    json.printTo(Serial);
    json.printTo(configFile);
    configFile.close();
    //end save
  }

  Serial.println("local ip");
  Serial.println(WiFi.localIP());
  
  Blynk.config(blynk_token);
  Blynk.connect();
terminal.clear();
 // This will print Blynk Software version to the Terminal Widget when
  // your hardware gets connected to Blynk Server
  terminal.println(F("Blynk v" BLYNK_VERSION ": Device started"));
  terminal.println(F("-------------"));
  terminal.println(F("Type 'Marco' and get a reply, or type"));
  terminal.println(F("anything else and get it printed back."));
  terminal.flush();

  // runs the sendUpdateStatus function every second
  timer.setInterval(1000L, sendUpdateStatus);  
  
}

///////////////////////////////////////////////////////////////////
//
// WARNING - Keep this loop clean
//           don't call any blynk read/write commands in here
//           they should all be done in the timer, or Blynk will
//           kick us off their server for spamming data
//
////////////////////////////////////////////////////////////////////

  void loop()
{
  Blynk.run(); // Initiates Blynk
  timer.run(); // Initiates BlynkTimer

// if (Serial.available()) {
 // readSerialUpdate();   // if serial is available, lets read it
 //char inByte = Serial.read();
 //Serial.print(inByte);
// }



}
