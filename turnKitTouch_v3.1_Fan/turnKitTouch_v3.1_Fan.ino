//  TRYONROBOTICS.COM
//  Josh Tryon's 2.8" Touch Screen Pellet Grill Controller
//  For TurnKit Pellet Grill
//  VERSION 3.1
//  Date last edited:  April 5, 2019  
//
//  Changelog
//  3.1 custom controller hardware.  Note the Analog Touch pins are different (reversed A0-A4) and relays 
//    are normally LOW and triggered HIGH, which is opposite of previous hardware version
//  2.3added variable fan timings
//
//
//
//  shout out to http://www.educ8s.tv
//  following your youtube demo helped me figure out what's going on here

//  also, used code from the tutorial example, for the MAX31856 thermocouple IC chip.  Pieces of that code from Peter Easton is below.

// Library Implementation Details
// ==============================
// DRDY and FAULT lines are not used in this driver. DRDY is useful for low-power mode so samples are only taken when
// needed; this driver assumes power isn't an issue.  The FAULT line can be used to generate an interrupt in the host
// processor when a fault occurs.  This library reads the fault register every time a reading is taken, and will
// return a fault error if there is one.  The MAX31856 has sophisticated usage scenarios involving FAULT.  For
// example, low and high temperature limits can be set, and the FAULT line triggers when these temperatures are
// breached. This is beyond the scope of this sample library.  The assumption is that most applications will be
// polling for temperature readings - but it is good to know these features are supported by the hardware.
//
// The MAX31856 differs from earlier thermocouple IC's in that it has registers that must be configured before
// readings can be taken.  This makes it very flexible and powerful, but one concern is power loss to the IC.  The IC
// should be as close to the cold junction as possible, which might mean there is a cable connecting the breakout
// board to the host processor.  If this cable is disconnected and reconnected (MAX31856 loses power) then the
// registers must be reinitialized.  This library detects this condition and will automatically reconfigure the
// registers.  This simplifies the software running on the host.
//
// A lot of configuration options appear in the .H file.  Of particular note is the line frequency filtering, which
// defaults to 60Hz (USA and others).  If your line voltage is 50Hz you should set CR0_NOISE_FILTER_50HZ.
//
// This library handles the full range of temperatures, including negative temperatures.
//
// When connecting the thermocouple, remember the 2 wires are polarized.  If temperatures go up when you expect
// them to go down just reverse the wires.  No damage will be done to the MAX31856.
//
// Change History:
// v1.4 - Added Skip Button to ignition timing.
// 26 December 2016    Initial Version (this is from Peter's code)

#include <Adafruit_TFTLCD.h>
#include <Adafruit_GFX.h>
#include <TouchScreen.h>
#include <EEPROM.h>
#include <MAX31856.h>

#define led 13
#define led1 0
#define led2 0
#define led3 0
#define led4 0

#define igniter 22      // igniter output on digital pin 50
#define fan 24          // fan output on digital pin 53
#define auger 26        // auger output on digital pin 52

#define LCD_CS A3 //  A1     //A3    //new A1
#define LCD_CD A2 //  A2     //A2    //new A2
#define LCD_WR A1 //  A3     //A1    //
#define LCD_RD A0 //  A4     //A0
#define LCD_RESET A4 //  A0  //A4

#define TS_MINX 126
#define TS_MINY 74
#define TS_MAXX 908
#define TS_MAXY 915

#define YP A3  // must be an analog pin, use "An" notation!   //WAS A1
#define XM A2  // must be an analog pin, use "An" notation!   //WAS A2
#define YM 9   // can be a digital pin
#define XP 8   // can be a digital pin

#define BLACK   0x0000
#define GREY    0x632C
#define BLUE    0x001F
#define RED     0xF800
#define GREEN   0x07E0
#define DKGREEN 0x0400
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF

Adafruit_TFTLCD tft(LCD_CS, LCD_CD, LCD_WR, LCD_RD, LCD_RESET);
TouchScreen ts = TouchScreen(XP, YP, XM, YM, 300);

////////////////
//  MAX31856 setup code here
///////////////
// The power requirement for the board is less than 2mA.  Most microcontrollers can source or sink a lot more
// than that one each I/O pin.  For example, the ATmega328 supports up to 20mA.  So it is possible to power the
// board using I/O pins for power - so you can turn the board on and off (if you want to).
// FAULT and DRDY are not used by the library (see above)
#define SCK 31   // white   A    //3 old from original software template for MAX31856 code
#define CS0 33   // red     A    //4 old
#define CS1 35   // black   A    //5 old
#define CS2 37   // white   B    //6 old
#define CS3 39   // red     B    //7 old
#define SDI 41   // black   B    //8 old
#define SDO 43   // white   C    //9 old
// pin 45 wired up as well. C RED     GIVE 5v To this pin to power MAX31856 chip
// pin 47 wired up as well. C BLACK   GIVE GND to this pin for power
#define MAXV 45   // Positive 5 Volts pin on 31856 chip
#define MAXG 47   // Ground pin on 31856 chip
#define NUM_MAX31856   4

// MAX31856 Initial settings (see MAX31856.h and the MAX31856 datasheet)
// The default noise filter is 60Hz, suitable for the USA
#define CR0_INIT  (CR0_AUTOMATIC_CONVERSION + CR0_OPEN_CIRCUIT_FAULT_TYPE_K /* + CR0_NOISE_FILTER_50HZ */)
#define CR1_INIT  (CR1_AVERAGE_2_SAMPLES + CR1_THERMOCOUPLE_TYPE_K)
#define MASK_INIT (~(MASK_VOLTAGE_UNDER_OVER_FAULT + MASK_THERMOCOUPLE_OPEN_FAULT))

// Create the temperature object, defining the pins used for communication
MAX31856 *TemperatureSensor[NUM_MAX31856] = {
  new MAX31856(SDI, SDO, CS0, SCK),
  new MAX31856(SDI, SDO, CS1, SCK),
  new MAX31856(SDI, SDO, CS2, SCK),
  new MAX31856(SDI, SDO, CS3, SCK)
};
//////////
//  END MAX SETUP CODE
//////////


/////////////////////////////////
//    DEFINE VARIABLES HERE    //
/////////////////////////////////
int debugMode = 1;            // set to 1 for Serial Printouts at 9600
int debugCounter = 0;         // used to count through loops and display debug only every set number of cyles

int versionNumber = 3;        // no decimal places. use build number for that
int buildNumber = 1;          // build number

int eeprom1 = 0;       // where to store eeprom.  Accepts value of 0-255.  used to store current Setting
//int numberOfTempProbes = 1;
// Timing Variables

// SETPOINTS
int numberOfSetpoints = 17;
int setpoints[17] = {6500, 6000, 5500, 5000, 4500, 4000, 3500, 3000, 2500, 2000, 1600, 1200, 900, 700, 500, 300, 100};    // auger OFF times for each setting
// default {10300, 8500, 6700, 5000, 4000, 3000, 2100, 1600, 1100, 700};

int numberOfSmokeSettings = 7;                         // how many smoke settings are there?
int smokeSetting[7] = {60, 65, 70, 100, 100, 100, 100};   // fan duty cycle percentage.  60 = 60% on 40% off over set duty cylce period
int smokeDutyCylePeriod = 10000;                       // smoke duty cycle period 10 seconds = 10000

// note that calling setpoints start at zero.  for 10 setpoints, it's 0-9
int currentSetPoint = 8;
int buttonDelay = 500;        // prevent repeated button presses. debounce for this amount of time
int augerRunTime = 400;      // sets auger ON time in milliseconds
int ignitionTiming = 1200;    // sets auger OFF time for IGNITION SEQUENCE only
unsigned long ignitionSequenceLength = 200000;   // how long should the ignition sequence last? in milliseconds 240000
unsigned long shutdownTime = 240000;              // how long should the fan run to burn off remaining pellets 240000
unsigned long ignitionCountdown = ignitionSequenceLength / 1000;
unsigned long shutdownCountdown = shutdownTime / 1000;
unsigned long timeLastTempCheck = 0;
unsigned long tempCheckInterval = 1000;
int powerStatus = 0;  // for App   (0=OFF,1=Startup,2=Running,3=Shutdown)
double grillTemp = 0;
double meatTemp = 0;
//  FAN Smoke settings 

char delimiter = ",";

//  Variables for Display/icons/text placement
int powerButtonStartX = 45;   // X coordinate to start POWER button
int powerButtonStartY = 100;  // Y
int primeButtonStartX = 45;   // X coordinate to start PRIME button
int primeButtonStartY = 180;  // Y
int skipButtonStartX  = 280;  // X coordinate to show SKIP button
int skipButtonStartY  = 35;   // Y coordinate to show SKIP button

//  position of meat/grill temps
int grillTextStartX = 195;   // original top/bottom data should be 15, 5, 250, 3
int grillTextStartY = 10;   
int grillTempStartX = 200;  
int grillTempStartY = 35;    

int meatTextStartX = 10;    // original top/bottom data should be 15, 35, 200, 33
int meatTextStartY = 35;
int meatTempStartX = 200;
int meatTempStartY = 35;

// size and position of the Settings Up/Dn arrows
int settingsTriangleSize = 25;
int settingsTrianglesX = 270;
int settingsTrianglesY = 80;

// physical outputs, current status variables
int augerStatus = LOW;
int augerLastStatus = LOW;
int fanStatus = LOW;
int fanLastStatus = LOW;


// power, and timing status variables
bool powerState = false;      // store power state. true = on, false = off
unsigned long lastAugerOnTime = 0;        // when was the auger turned ON most recently
unsigned long lastAugerOffTime = 0;       // when was the auger turned OFF most recently
unsigned long lastFanOnTime = 0;          // when was the fan turned ON most recently
unsigned long lastFanOffTime = 0;         // when was the fan turned OFF most recently
unsigned long ignitionStartTime = 0;      // stores when the power was last turned on and ignition sequence started
unsigned long shutdownSequenceStartTime = 0;  // stores when the power was turned off, so we can run fan for awhile

// is the grill currently in the startup/ignition sequence?
bool ignitionSequenceInProgress = false;
bool ignitionSequencePreviousState = false;
bool shutdownSequenceInProgress = false;
bool shutdownSequencePreviousState = false;

bool grillPower = false;
bool lastGrillPowerState = false;

bool showTemps = false;

// BUTTON PRESS status variables
unsigned long lastButtonPress = 0;
unsigned long currentButtonPress = 0;
unsigned long lastPrimePress = 0;
unsigned long currentPrimePress = 0;
unsigned long lastSettingsUpPress = 0;
unsigned long lastSettingsDownPress = 0;

// actual button presses
bool powerButtonPressed = false;
bool primeButtonPressed = false;
bool settingsUpButtonPressed = false;
bool settingsDownButtonPressed = false;


/////////////////////////////////////////
//      ARDUINO SETUP CODE BELOW
/////////////////////////////////////////

// initializes serial, and display.  Verifies setpoint is valid.
void setup() {
  if (debugMode == 1) {
    Serial.begin(9600);
    debugln("Getting Started...");
  }
  Serial1.begin(9600);  // serial port for communicating with auxillary device

  
  // set output pins as outputs
  pinMode(igniter, OUTPUT);
  pinMode(fan, OUTPUT);
  pinMode(auger, OUTPUT);
  pinMode(MAXV, OUTPUT);
  pinMode(MAXG, OUTPUT);
/*
 * 
 * 
 * 
 
  digitalWrite(igniter, LOW);
  digitalWrite(fan, LOW);
  digitalWrite(auger, LOW);
//  digitalWrite(MAXV, HIGH);   // 5v power for max31856 chip // EDIT - this is now controlled during power on/off functions
  digitalWrite(MAXG, LOW);    // 5v GND for max31856
 */
digitalWrite(igniter, HIGH);
  digitalWrite(fan, HIGH);
  digitalWrite(auger, HIGH);
//  digitalWrite(MAXV, HIGH);   // 5v power for max31856 chip // EDIT - this is now controlled during power on/off functions
  digitalWrite(MAXG, LOW);    // 5v GND for max31856
  
////// MAX31856 CODE BELOW
  for (int i=0; i<NUM_MAX31856; i++) {
    TemperatureSensor[i]->writeRegister(REGISTER_CR0, CR0_INIT);
    TemperatureSensor[i]->writeRegister(REGISTER_CR1, CR1_INIT);
    TemperatureSensor[i]->writeRegister(REGISTER_MASK, MASK_INIT);
  }
  
  // Wait for the first samples to be taken
  delay(200);
/////// END MAX CODE

  
  currentSetPoint = EEPROM.read(eeprom1);   // reads EEPROM for saved setpoint setting
  // check to make sure setpoint value is a valid value
  if (currentSetPoint > (numberOfSetpoints - 1))
  {
    currentSetPoint = 4;    // if not a legit setpoint, make it 4 (setpoint 5, since we start at 0 as setpoint 1)
    EEPROM.update(eeprom1, currentSetPoint);
  }

  tft.reset();              // resets the display
  tft.begin(0x9341);        // initialize display with chipset 0x9341
  tft.setRotation(1);       // setting rotation.  1 is a landscape orientation
  tft.fillScreen(BLACK);    // screen starts out white.  This fills screen with black

  //  Draw White outer box
  // tft.drawRect(0,0,319,240,WHITE);   // uncomment if you want white border around screen
  sendAppUpdate();
  displayStartupScreen();               // prints the boot screen, showing name and build number etc
  delay(5000);                          // delays, to allow build screen to show for 5 seconds

  tft.fillScreen(BLACK);                // black out screen
  displayMainScreen();

  debugln("Setup Complete!");
  debugln("Running main routine");
  powerStatus = 0;  // status for App.  0=OFF
  sendAppUpdate();
}


//////////////////////////////////////////////////////////
//                                                      //
//            Fuction Declarations Go Here              //
//                                                      //
//////////////////////////////////////////////////////////



////////////////////////////////
//    LCD DISPLAY ROUTINES    //
////////////////////////////////

//  draws the boot screen that shows name and version number
void displayStartupScreen() {
  tft.setCursor(7, 20);
  tft.setTextColor(WHITE);
  tft.setTextSize(4);
  tft.print("Turnkit Grill");
  tft.setCursor(60, 60);
  tft.setTextSize(2);
  tft.print("Version ");
  tft.print(versionNumber);
  tft.print(" build ");
  tft.print(buildNumber);
  tft.setCursor(7,100);
  tft.setTextColor(WHITE);
  tft.setTextSize(1);
  tft.print("AUGER ON TIME:");
  tft.setCursor(7,130);
  tft.print("AUGER IGNTION OFF TIME:");
  tft.setCursor(7,160);
  tft.print("NUMBER OF SETPOINTS");
  tft.setCursor(7,190);
  tft.print("SETPOINTS:");
  tft.setTextColor(GREEN);
  tft.setCursor(150,100);
  tft.print(augerRunTime/100);
  tft.setCursor(150,130);
  tft.print(ignitionTiming/100);
  tft.setCursor(150,160);
  tft.print(numberOfSetpoints);
  for(int i=0; i<numberOfSetpoints; i++){
    tft.setCursor(75+(i*20),190);
    tft.print(setpoints[i]/100);
  }
}

//  draws the main display screen.  The 'home' look when the unit is powered OFF
void displayMainScreen() {
  drawPrimeButton(GREY);           // draws PRIME button in idle state with a GREY circle
  drawPowerButton("OFF");          // draws POWER button in OFF state
}

//  draws the Ignition Countdown information during power on ignition sequence
void displayIgnitionScreen() {
  clearDisplay();
  tft.setCursor(90, 10);
  tft.setTextColor(WHITE);
  tft.setTextSize(3);
  tft.print("IGNITION");
  tft.setCursor(80, 40);
  tft.print("COUNTDOWN");
  displayTimer(ignitionCountdown);
  displayRunningScreen();
  drawSkipButton(WHITE);
}

// draws the Shutdown Countdown information during power OFF shutdown sequence
void displayShutdownScreen() {
  clearDisplay();
  tft.fillRect(100,50,200,160,BLACK);
  tft.setCursor(50, 10);
  tft.setTextColor(WHITE);
  tft.setTextSize(3);
  tft.print("Shutting Down");
  tft.setCursor(50, 40);
  tft.print("Do Not Unplug");
  displayTimer(shutdownCountdown);
}

//  draws the status countdown of ignition sequence.   Milliseconds passed to this function is displayed as clock
void displayTimer(int timer) {
  int minutes = timer / 60;
  int seconds = (timer - (minutes * 60));
  tft.setCursor(106, 75);
  tft.setTextColor(WHITE, BLACK);
  tft.setTextSize(4);

  if (minutes < 10) {
    tft.setTextColor(BLACK, BLACK);
    tft.print("1");
    tft.setTextColor(WHITE, BLACK);
    tft.setCursor(130, 75);
  }

  tft.print(minutes);
  tft.setCursor(150, 75);
  tft.print(":");
  tft.setCursor(170, 75);

  if (seconds < 10) {
    tft.print("0");
    tft.setCursor(194, 75);
    tft.print(seconds);
  }
  else {
    tft.print(seconds);
  }
}

// clears display (black) and redraws power and prime buttons.  then triggers running screen to display
void clearDisplay() {
  tft.fillRect(0,2,350,60,BLACK);
  tft.fillRect(130,75,100,50,BLACK);
 // tft.fillScreen(BLACK);
 // drawPowerButton("ON");
 // drawPrimeButton(GREY);
 // displayRunningScreen();
}

//  draws the main display screen, the 'home' look when the unit is powered ON
void displayRunningScreen() {
  makeUpDownTriangles(settingsTriangleSize, settingsTrianglesX, settingsTrianglesY, RED, BLUE);   // (size, top point X, Y, up color, dn color)
  tft.setCursor(145, 165);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.print("Setpoint");
  displaySetpoint();
//  displayTemperatures();
}
//  draws the temperature area of the screen when running
void displayTemperatures() {
  //tft.fillRect(100,50,200,160,BLACK);
  tft.setCursor(grillTextStartX, grillTextStartY);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.print("Grill Temp");
//  tft.setCursor(meatTextStartX, meatTextStartY);
//  tft.print("Meat  Temp");
  displayTempValues();
}
void displayTempValues() {
  tft.setCursor(grillTempStartX, grillTempStartY);
  tft.setTextColor(YELLOW, BLACK);
  tft.setTextSize(4);  // was 3
  tft.print("0");
 // tft.setCursor(meatTempStartX, meatTempStartY);
 // tft.print("0");
}

void displayMeatTempValue(double temp){
  tft.setCursor(meatTempStartX, meatTempStartY);
  tft.setTextColor(YELLOW, BLACK);
  tft.setTextSize(3);
  tft.print(temp);
}

void displayGrillTempValue(double temp){
  tft.setCursor(grillTempStartX, grillTempStartY);
  tft.setTextColor(YELLOW, BLACK);
  tft.setTextSize(4);  //was 3
  tft.print(temp);
}

//  draws circle and text labeled POWER in selected position and color fill
void drawPowerButton(char powerState[])  {
  if (powerState == "ON") {
    tft.fillCircle(powerButtonStartX, powerButtonStartY, 32, DKGREEN);
  }
  else {
    tft.fillCircle(powerButtonStartX, powerButtonStartY, 32, RED);
  }
  tft.setCursor(powerButtonStartX - 28, powerButtonStartY - 8);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.print("POWER");
  if (powerState == "ON") {
    tft.setCursor(powerButtonStartX - 10, powerButtonStartY + 10);
  }
  else {
    tft.setCursor(powerButtonStartX - 15, powerButtonStartY + 10);
  }
  tft.print(powerState);
}

//  draws circle and text labled PRIME in selected position and color fill
void drawPrimeButton(char fillColor[])  {
  tft.fillCircle(primeButtonStartX, primeButtonStartY, 32, fillColor);
  tft.setCursor(primeButtonStartX - 28, primeButtonStartY - 8);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.print("PRIME");
}

//  draws SKIP button.  To skip the ignition sequence, when fire is already lit and user wants to skip ahead to program
void drawSkipButton(char fillColor[])  {
  tft.fillCircle(skipButtonStartX, skipButtonStartY, 25, fillColor);
  tft.setCursor(skipButtonStartX -22, skipButtonStartY -8);
  tft.setTextColor(BLACK);
  tft.setTextSize(2);
  tft.print("SKIP");
}

//  makeUpDownTriangles draws a matching pair of triangles of selected size, position, and color
void makeUpDownTriangles(int triangleSize, int startX, int startY, char fillUpColor[], char fillDownColor[]) {
  makeUpTriangle(triangleSize, startX, startY, fillUpColor);
  makeDownTriangle(triangleSize, startX, (startY + (triangleSize * 4) + (triangleSize * .75)), fillDownColor);
}

//  draws upward facing triangle of selected size, position, color
void makeUpTriangle(int triangleSize, int startX, int startY, char fillColor[]) {
  tft.fillTriangle(startX, startY, startX + triangleSize, startY + (triangleSize * 2), startX - triangleSize, startY + (triangleSize * 2), fillColor);
  tft.drawTriangle(startX, startY, startX + triangleSize, startY + (triangleSize * 2), startX - triangleSize, startY + (triangleSize * 2), WHITE);
}

//  draws downward facing triangle of selected size, position, color
void makeDownTriangle(int triangleSize, int startX, int startY, char fillColor[]) {
  tft.fillTriangle(startX, startY, startX + triangleSize, startY - (triangleSize * 2), startX - triangleSize, startY - (triangleSize * 2), fillColor);
  tft.drawTriangle(startX, startY, startX + triangleSize, startY - (triangleSize * 2), startX - triangleSize, startY - (triangleSize * 2), WHITE);
}

// if debug. displays text on bottom of LCD when output is HIGH/active
void indicatorON(char indicator[]) {
  if(debugMode){
    if(indicator == "FAN") {
      tft.setCursor(70,220);
    }
    if(indicator == "AUGER") {
      tft.setCursor(120,220);
    }
    if(indicator == "IGNITER") {
      tft.setCursor(200,220);
    }
    tft.setTextColor(WHITE,RED);
    tft.setTextSize(2);
    tft.print(indicator);
  }
}

void indicatorOFF(char indicator[]) {
  if(debugMode) {
    if(indicator == "FAN") {
      tft.fillRect(70,220,50,50,BLACK);
    }
    if(indicator == "AUGER") {
      tft.fillRect(120,220,70,50,BLACK);
    }
    if(indicator == "IGNITER") {
      tft.fillRect(200,220,100,50,BLACK);
    }
  }
}

////////////////////////////////////////////
//    PHYSICAL OUTPUT CONTROL FUNTIONS    //
////////////////////////////////////////////

////////////////////////////
/*
 * 
 * 
 * USE THIS FOR OLD ARDUINO MEGA BOARD
 * 
 * 
 */

// turns auger on.  sets last auger on time.
void augerON() {
  if (augerStatus != HIGH)
  {
    debugln("Auger ON");
    digitalWrite(auger, LOW);    // turns ON the auger
    augerStatus = HIGH;
    lastAugerOnTime = millis();
    indicatorON("AUGER");
  }
}

// turns auger off.  sets last auger off time.
void augerOFF() {
  if (augerStatus != LOW)
  {
    debugln("Auger OFF");
    digitalWrite(auger, HIGH); // turns OFF the auger
    augerStatus = LOW;
    lastAugerOffTime = millis();
    indicatorOFF("AUGER");
  }
}

// turns igniter on
void igniterON() {
  digitalWrite(igniter, LOW);  //  turns ON igniter
  debugln("Igniter ON");
  indicatorON("IGNITER");
}

// turns igniter off
void igniterOFF() {
  digitalWrite(igniter, HIGH); //  turns OFF igniter
  debugln("Igniter OFF");
  indicatorOFF("IGNITER");
}

// turns fan on
void fanON() {
  digitalWrite(fan, LOW);    //  you guessed it, turns fan ON
  fanStatus = HIGH;
  lastFanOnTime = millis();
  debugln("Fan ON");
  indicatorON("FAN");
}

// turns fan off
void fanOFF() {
  digitalWrite(fan, HIGH);   //  nothing gets by you.  turns fan OFF
  fanStatus = LOW;
  lastFanOffTime = millis();
  debugln("Fan OFF");
  indicatorOFF("FAN");
}



//////////////////////////////////////////////////////
/*
 * 
 * 
 *    USE THE BELOW VERSION FOR NEW BOARD
 * 
 * 
 * 
// turns auger on.  sets last auger on time.
void augerON() {
  if (augerStatus != HIGH)
  {
    debugln("Auger ON");
    digitalWrite(auger, HIGH);    // turns ON the auger
    augerStatus = HIGH;
    lastAugerOnTime = millis();
    indicatorON("AUGER");
  }
}

// turns auger off.  sets last auger off time.
void augerOFF() {
  if (augerStatus != LOW)
  {
    debugln("Auger OFF");
    digitalWrite(auger, LOW); // turns OFF the auger
    augerStatus = LOW;
    lastAugerOffTime = millis();
    indicatorOFF("AUGER");
  }
}

// turns igniter on
void igniterON() {
  digitalWrite(igniter, HIGH);  //  turns ON igniter
  debugln("Igniter ON");
  indicatorON("IGNITER");
}

// turns igniter off
void igniterOFF() {
  digitalWrite(igniter, LOW); //  turns OFF igniter
  debugln("Igniter OFF");
  indicatorOFF("IGNITER");
}

// turns fan on
void fanON() {
  digitalWrite(fan, HIGH);    //  you guessed it, turns fan ON
  fanStatus = HIGH;
  lastFanOnTime = millis();
  debugln("Fan ON");
  indicatorON("FAN");
}

// turns fan off
void fanOFF() {
  digitalWrite(fan, LOW);   //  nothing gets by you.  turns fan OFF
  fanStatus = LOW;
  lastFanOffTime = millis();
  debugln("Fan OFF");
  indicatorOFF("FAN");
}
*/
////////////////////////////////////////////


//////////////////////////////////
//    POWER CONTROL FUNCTIONS   //
//////////////////////////////////

// toggles power.  ON > OFF or OFF > ON.  runs corresponding power function
void togglePower() {

  if (powerState == false) {
    powerState = !powerState;
    debugln("Change Power State to ON");
    powerON();
  } else {
    powerState = !powerState;
    debugln("Change Power State to OFF");
    powerOFF();
  }
}

//  When the Power Button is pressed and the unit is currently OFF, turn it on.  trigger ignition
void powerON() {
  drawPowerButton("ON");
  debugln("POWER ON");
  grillPower = true;  // sets power on/off flag to true, which means power is now ON
  ignitionSequence(); // initiates startup/ignition sequence burn
  digitalWrite(MAXV, HIGH);  // turn on MAX31856 temp sensing board
  powerStatus = 1;  // status for App.  1=Ignition
  sendAppUpdate();
}

//  When the Power Button is pressed and the unit is currently ON, turn it OFF. trigger shutdown
void powerOFF() {
  drawPowerButton("OFF");
  debugln("POWER OFF");
  grillPower = false;
  shutdownSequence();
  showTemps = false;
  digitalWrite(MAXV, LOW);  // turn off MAX31856 temp sensing board
  powerStatus = 3;  // status for App.  3=Shutting Down
  sendAppUpdate();    // send update to app
}

// initiate ignition sequence.  start ignition timer
void ignitionSequence() {
  showTemps = false;
  debugln("IGNITION STARTED");
  displayIgnitionScreen();
  fanON();
  igniterON();
  ignitionSequenceInProgress = true;
  shutdownSequenceInProgress = false;
  ignitionStartTime = millis();     // sets the ignition sequence start time
}

// initiate shutdown sequence.  Run fan, start timer
void shutdownSequence() {
  ignitionSequenceInProgress = false;
  shutdownSequenceInProgress = true;
  EEPROM.update(eeprom1,currentSetPoint); // save setpoint to memory
  debugln("Setpoint saved to EEPROM");
  debugln("SHUTDOWN STARTED");
  displayShutdownScreen();
  augerOFF();
  igniterOFF();
  fanON();
  shutdownSequenceStartTime = millis();
  //  ALSO _ turn of screen, or display countdown timer? as fan runs to burn of remaining fuel
}



/////////////////////////////
//    DEBUG PRINTOUTS     //
////////////////////////////

// prints number, no newline
void debugNumber(unsigned long message) {
  if (debugMode == 1)
  {
    Serial.println(message);
  }
}

// prints message, no newline
void debug(char message[]) {
  if (debugMode == 1)
  {
    Serial.print(message);
  }
}

// prints a line using Serial.println
void debugln(char message[]) {
  if (debugMode == 1)
  {
    Serial.println(message);
  }
}



////////////////////////////////////
//      BUTTON PRESSES            //
////////////////////////////////////

// PRIME button pressed.  engage auger manually
void pressPrime() {
  primeButtonPressed = true;
  lastPrimePress = millis();
  drawPrimeButton(DKGREEN);
  debugln("engage PRIME AUGER");
  augerON();
}

// SKIP Button pressed
void pressSkip() {

  drawSkipButton(RED);
  debugln("pressSkip  function ran");
  ignitionSequenceInProgress = false;
        igniterOFF();
        augerOFF();
        debugln("STARTUP is over... moving on");
        debug("execute runProgram SETPOINT: ");
        debugNumber(currentSetPoint);
        runProgram(setpoints[currentSetPoint]);    // and start the progam using the selected setpoint
        clearDisplay();
        displayRunningScreen();
        displayTemperatures();
        debugCounter = 0;
  powerStatus = 2;  // status for App.  2 = Running mode
}

// setting UP button pressed. visual feedback on button. change current setting up one click. display new setting.
void pressSettingUp() {
  makeUpTriangle(settingsTriangleSize, settingsTrianglesX, settingsTrianglesY, WHITE);
  settingsUpButtonPressed = true;
  lastSettingsUpPress = millis();
  debugln("Settings UP button pressed");

  if (currentSetPoint < (numberOfSetpoints - 1))    // Example if current setpoint is 9 (starting from zero), and number of setpoint is 10
  { // we're already at the highest setting, since setting 9 is the tenth setting
    currentSetPoint++;                            // if we're less than 2 under the number of settings, then we can increase by one
  }
  displaySetpoint();
}

// setting DOWN button pressed.  visual feedback on button. change current setting down one click. display new setpoint
void pressSettingDown() {
  makeDownTriangle(settingsTriangleSize, settingsTrianglesX, (settingsTrianglesY + (settingsTriangleSize * 4) + (settingsTriangleSize * .75)), WHITE);
  settingsDownButtonPressed = true;
  lastSettingsDownPress = millis();
  debugln("Settings DOWN button pressed");

  if (currentSetPoint > 0)             // since lowest setpoint is zero, we need to be at least 1 in order to go down any lower.
  {
    currentSetPoint--;                // if we are above zero, then subtract one
  }
  displaySetpoint();
}


//////////////////////////
//    OTHER FUNCTIONS   //
//////////////////////////

//  Sends Serial Data out the second serial port (Serial1)
//  To the auxillary hardware port on grill
void sendAppUpdate() {
  int currentSetPointMath = currentSetPoint + 1;
  String sendMe = String('P' + powerStatus + ',' +ignitionCountdown + ',' + shutdownCountdown + ',' + currentSetPointMath + ',' + grillTemp + ',' + meatTemp);
  //Serial1.print('P' + powerStatus + ',' + ignitionCountdown + ',' + shutdownCountdown + ',' + currentSetPointMath + ",8,8");
 // Serial1.print("P2,0,0," + currentSetPointMath);
 // Serial1.print(',' + grillTemp);
 // Serial1.print(',' + meatTemp);
//  Serial1.print("P");
//  Serial1.print(powerStatus);
//  Serial1.print("I");
//  Serial1.print(ignitionCountdown);
//  Serial1.print("S");
//  Serial1.print(shutdownCountdown);
  Serial1.print("C");
  Serial1.print((currentSetPoint+1));
  Serial1.print("G");
  Serial1.print(grillTemp);
/*  Serial1.print("M");
  Serial1.println(meatTemp);
  */
}



// cleans up the touched buttons, returns visual state of touched buttons to default.
void cleanup() {
  if (primeButtonPressed == true) {
    unsigned long currentTime = millis();
    if ((currentTime - lastPrimePress) > 700) {
      augerOFF();
      drawPrimeButton(GREY);
      debugln("disengage PRIME AUGER");
      primeButtonPressed = false;
    }
  }

  if (settingsUpButtonPressed == true) {
    unsigned long currentTime = millis();
    if ((currentTime - lastSettingsUpPress) > 600) {
      makeUpTriangle(settingsTriangleSize, settingsTrianglesX, settingsTrianglesY, RED);
      debugln("reset UP arrow");
      settingsUpButtonPressed = false;
    }
  }

  if (settingsDownButtonPressed == true) {
    unsigned long currentTime = millis();
    if ((currentTime - lastSettingsDownPress) > 600) {
      makeDownTriangle(settingsTriangleSize, settingsTrianglesX, (settingsTrianglesY + (settingsTriangleSize * 4) + (settingsTriangleSize * .75)), BLUE);
      debugln("reset DOWN arrow");
      settingsDownButtonPressed = false;
    }
  }
}

// draws the current setpoint 1-10 on the screen
void displaySetpoint() {
  int displaySetPoint = (currentSetPoint + 1);
  tft.fillRect(180, 120, 60, 35, BLACK);
  tft.setCursor(180, 120);
  tft.setTextColor(WHITE, BLACK);
  tft.setTextSize(5);
  tft.print(displaySetPoint);
}

// runs program accordning to timing sent to function. typically this is the current setpoint time value, or ignition time value
void runProgram(int setpoint) {

  // get current time, save to currentTime variable
  unsigned long currentTime = millis();

  //  if the auger has been off for the set amount of time, turn it back on again
  if ((currentTime - lastAugerOffTime) > setpoint)
  {
    augerON();
  }

  // if the auger has been on for the set amount of time, turn it off.
  if ((currentTime - lastAugerOnTime) > augerRunTime)
  {
    if (primeButtonPressed == false)
    {
      augerOFF();
    }
  }

  if ((currentTime - timeLastTempCheck) > tempCheckInterval)
  {
    checkTemperatures();
    timeLastTempCheck = currentTime;
    sendAppUpdate();    // send update to App according to tempCheckInterval timing
  }

  if (ignitionSequenceInProgress == true)   // if grill is in ignition sequence, we want to keep fan 100%
  {
    if (fanStatus == LOW)                     // verify fan is on, if not, turn it on
    {
      fanON();
    }
  }
  else        //  ignition sequence is false, so lets have fun with fan timings
  {
 
    if (currentSetPoint < (numberOfSmokeSettings)-1)
    {
      // has the fan been on longer than the smoke setting duty cycle percentage that corresponds with the current setting?
      if ((currentTime - lastFanOnTime) > ((smokeSetting[currentSetPoint])*100))  // note the smokeSetting times are in percentage duty cycle.  Multiply by 100 to get milliseconds fan should be ON
      {
        if (fanStatus == HIGH)
        {
          fanOFF();
        }
      }
      
    }
    if ((currentTime - lastFanOffTime) > (100-(smokeSetting[currentSetPoint]))*100) // get the opposite percentage of the duty cycle.  Multiply by 100 to get milliseconds fan should be OFF
      {
        if (fanStatus == LOW)
        {
        fanON();
        }
      }
  }
  if ((currentTime - lastFanOffTime) > 10000)   // no matter what, if fan has been off for 10 seconds, turn it on
  {
    if(fanStatus == LOW)
    {
    fanON();
    }
  }
}



////////////
//   MAX31856 Temp Functions
////////////

// MAX31856 function to print temps to serial
void printTemperature(double temperature) {
  switch ((int) temperature) {
    case FAULT_OPEN:
      Serial.print("FAULT_OPEN");
      break;
    case FAULT_VOLTAGE:
      Serial.print("FAULT_VOLTAGE");
      break;
    case NO_MAX31856:
      Serial.print("NO_MAX31856");
      break;
    default:
      Serial.print(temperature);
      break;
  }
  Serial.print(" ");
}


//  function to poll the MAX31856 chip for new temp readings
void checkTemperatures(){
  for (int i=0; i<NUM_MAX31856; i++) {
    // Display the junction (IC) temperature first
    // Sometimes the junction temperature is not provided until a thermocouple is attached
    double temperature = TemperatureSensor[i]->readJunction(CELSIUS);
    if (temperature == NO_MAX31856)
      continue;
    Serial.print("J");
    Serial.print(i);
    Serial.print("=");
    printTemperature(temperature);

    // Display the thermocouple temperature
    temperature = TemperatureSensor[i]->readThermocouple(CELSIUS);
    if (temperature == NO_MAX31856)
      continue;
    Serial.print("T");
    Serial.print(i);
    Serial.print("=");
    printTemperature(temperature);

    Serial.print("\t");
  }
  
  Serial.println();
  if(showTemps == true)
  {
    grillTemp = TemperatureSensor[0]->readThermocouple(CELSIUS);
    meatTemp = TemperatureSensor[1]->readThermocouple(CELSIUS);
    if (grillTemp == FAULT_OPEN)
    {
      grillTemp = 0;
    }
    
    if (grillTemp == FAULT_VOLTAGE)
    {
      grillTemp = 0;
    }
    else
    {
      grillTemp = (grillTemp * 1.8)+32;
    }

    if (meatTemp == FAULT_OPEN)
    {
      meatTemp = 0;
    }
    
    if (meatTemp == FAULT_VOLTAGE)
    {
      meatTemp = 0;
    }
    else
    {
      meatTemp = (meatTemp * 1.8)+32;
    }
    displayGrillTempValue(grillTemp);
 //   displayMeatTempValue(meatTemp);
  }
}
////////////////////////////////////////////////////
//                                                //
//            ACTUAL PROGRAM BELOW                //
//                OH YEAH!!!!!!                   //
//                                                //
////////////////////////////////////////////////////

void loop() {

  // Read from Serial1 and write it to serial
  if (Serial1.available()) {
    char inByte = Serial1.read();
    Serial.print(inByte);
    if (inByte == '1'){
      pressSettingDown();
    }

    if (inByte == '2'){
      pressSettingUp();
    }
   
  }
  
  // if grill power is on, do this
  if (grillPower == true)
  {

    // if grill was just turned on last cycle change power state to on
    if (lastGrillPowerState != grillPower)
    {
      debugln("grillPower is seen as true");
      lastGrillPowerState = grillPower;
    }

    // is the ignition sequence still going?
    if (ignitionSequenceInProgress == true)
    {

      // if ignition sequence was just activated, log it and start timer
      if (ignitionSequencePreviousState != true)
      {
        debugln("ignition sequence in progress TRUE");
        debugln("execute runProgram STARTUP");
        ignitionSequencePreviousState = true;
      }
      unsigned long currentTime = millis();

      // has the ignition sequence gone long enough yet? Check to see
      if ((currentTime - ignitionStartTime) < ignitionSequenceLength) {
        int previousIgnitionCountdown = ignitionCountdown;
        ignitionCountdown = (ignitionSequenceLength - (currentTime - ignitionStartTime)) / 1000;

        if (ignitionCountdown != previousIgnitionCountdown) {
          displayTimer(ignitionCountdown);
        }

        // if not, keep it going
        runProgram(ignitionTiming);
      }
      else
      {
        // if it has, then stop it.

        ignitionSequenceInProgress = false;
        igniterOFF();
        augerOFF();
        debugln("STARTUP is over... moving on");
        debug("execute runProgram SETPOINT: ");
        debugNumber(currentSetPoint);
        runProgram(setpoints[currentSetPoint]);    // and start the progam using the selected setpoint
        clearDisplay();
        displayRunningScreen();
        displayTemperatures();
        debugCounter = 0;
      }
    }
    else          // if ignition sequence is FALSE run normal program setpoints
    {
      runProgram(setpoints[currentSetPoint]);
      showTemps = true;
      powerStatus = 2;  // status for App. 2 = running mode
      if (debugCounter == 1)
      {
        debug("running setpoint: ");
        debugNumber(currentSetPoint+1);
      }
      if (debugCounter > 6000)
      {
        debug("running setpoint: ");
        debugNumber(currentSetPoint+1);
        debugCounter = 0;
        
      }
      debugCounter++;
    }
  }
  else  // is grill power false?  Run fan for a time, then shut down
  {
    if (shutdownSequenceInProgress == true)
    {
      unsigned long currentTime = millis();
      if ((currentTime - shutdownSequenceStartTime) < shutdownTime) {
        int previousShutdownCountdown = shutdownCountdown;
        shutdownCountdown = (shutdownTime - (currentTime - shutdownSequenceStartTime)) / 1000;
        if (shutdownCountdown != previousShutdownCountdown) {
          displayTimer(shutdownCountdown);
          sendAppUpdate();
        }
      }
      else {
        
        fanOFF();
        shutdownSequenceInProgress = false;
        debugln("GRILL IS COMPLETELY OFF NOW");
        tft.fillScreen(BLACK);
        drawPowerButton("OFF");
        drawPrimeButton(GREY);
        powerStatus = 0;  //status for App.  0=OFF
        sendAppUpdate();
      }
    }
  }


  TSPoint p = ts.getPoint();    // Get's the touch point
  pinMode(YP, OUTPUT);        //.kbv these pins are shared with TFT
  pinMode(XM, OUTPUT);        //.kbv these pins are shared with TFT
  // if touch is greater than pressure threshhold then do something
  if (p.z > ts.pressureThreshhold) {
    p.x = map(p.x, TS_MINX, TS_MAXX, 0, 320);   // converts the touch point to pixel coordinates for X
    p.y = map(p.y, TS_MAXY, TS_MINY, 0, 240);   // for Y
    /*
          Serial.print("X = ");
          Serial.print(p.x);
          Serial.print("\tY = ");
          Serial.println(p.y);
    */
    currentButtonPress = millis();
    if ((currentButtonPress - lastButtonPress) > buttonDelay) {


      // Power Button Press
      if (p.x > 140 && p.x < 225 && p.y > 10 && p.y < 57) {
        debugln("POWER BUTTON PRESSED");
        togglePower();
      }

      // Prime Button Press
      if (p.x > 36 && p.x < 115 && p.y > 10 && p.y < 57) {
        debugln("PRIME BUTTON PRESSED");
        pressPrime();
      }


// if the grill is on, not currently running the shutdown sequence, then accept button presses
// for the settings up/down buttons.  Otherwise, ignore the presses where those buttons would be.
      if (grillPower == true && shutdownSequenceInProgress == false) {

        // Setting Up Button Press
        if (p.x > 140 && p.x < 210 && p.y > 185 && p.y < 216) {
          debugln("SETTING UP BUTTON PRESSED");
          pressSettingUp();
        }

        // Setting Down Button Press
        if (p.x > 50 && p.x < 118 && p.y > 185 && p.y < 220) {
          debugln("SETTING DOWN BUTTON PRESSED");
          pressSettingDown();
        }

      }

      if (ignitionSequenceInProgress == true)  {
        if (p.x > 230 && p.x < 310 && p.y > 185 && p.y < 230) {
          debugln("SKIP BUTTON PRESSED");
          pressSkip();
        }
      }
      
      lastButtonPress = millis();
    }
  }

  cleanup();    // clean up the touch buttons, restore the pressed buttons to normal state


}
