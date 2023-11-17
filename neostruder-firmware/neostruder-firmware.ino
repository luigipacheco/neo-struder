/// Import libraries///
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <SPI.h>
#include "Adafruit_MAX31856.h"
#include <ezOutput.h>
#include <PID_v1.h>
#include "WiFi.h"
#include <WiFiUdp.h>
#include <OSCMessage.h>
#include <OSCBundle.h>


// Display variables///
#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 64  // OLED display height, in pixels
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// Rotary encoder variables
//Motor encoder
#define SPIN_A 2
#define SPIN_B 4
#define SPIN_BUTTON 15

//Fan encoder
#define FPIN_A 17
#define FPIN_B 5
#define FPIN_BUTTON 16

//temp encoder
#define TPIN_A 19
#define TPIN_B 3
#define TPIN_BUTTON 18

// Fan pwm out
#define FPIN_OUT 14
#define FPWM_Ch 1
#define FPWM_Res 8
#define FPWM_Freq 250000  //1000 or 250 000
int FPWM_DutyCycle = 0;
//#define robotIn 12  // start/stop extruder


// Global Variables for Encoder State
static int svalue = 0;
static int fvalue = 0;
static int tvalue = 0;
int DEBONCE_TO = 150;
int DEBONCE_BTN = 200;
volatile bool sturnedCW = false;
volatile bool sturnedCCW = false;
volatile bool fturnedCW = false;
volatile bool fturnedCCW = false;
volatile bool tturnedCW = false;
volatile bool tturnedCCW = false;

unsigned long slastButtonPress = 0;
unsigned long sdebounceTime = 0;

unsigned long flastButtonPress = 0;
unsigned long fdebounceTime = 0;

unsigned long tlastButtonPress = 0;
unsigned long tdebounceTime = 0;

bool slastWasCW = false;
bool slastWasCCW = false;

bool flastWasCW = false;
bool flastWasCCW = false;

bool tlastWasCW = false;
bool tlastWasCCW = false;

//// Heater Variables ////
bool enable = false;
double Output;
const unsigned long WindowSize = 5000;
unsigned long windowStartTime;
int OutHeater = 12;
int OutFan = 6;
bool HeaterEnable = false;
bool FanEnable = false;
int FanSpeed = 0;
int FanStep = 10;
double curTemp = 0;
double tarTemp = 0;
unsigned long LastReadTime = 0;

#define MAXDRDY 34
#define MAXDO   35
#define MAXCS   33
#define MAXCLK  32
#define MAXDI   13
PID myPID(&curTemp, &Output, &tarTemp, 2, 5, 1, DIRECT);
// Use software SPI: CS, DI, DO, CLK
Adafruit_MAX31856 thermocouple = Adafruit_MAX31856(MAXCS, MAXDI, MAXDO, MAXCLK);


//motor variables
int OutStep = 25;
int OutDir = 26;
int OutEnable = 27;
bool MotorEnable = false;
float MotorRev = 800;
float MotorSpeed = 0;  // rev per minute
int MotorStep = 25;
unsigned long LastRunTime = 0;
unsigned long currentMotorTime = 0;
unsigned long previousMotorTime = 0;
long motorInterval = 0.1;

bool ignoreRobotMotorSpeed = false;

// network variables:
WiFiUDP udp;
int port = 55555;


const char* ssid = "RDF_Rapture";
const char* password = "WiFi4RDF*!";
// Set your Static IP address
IPAddress static_IP(192, 168, 1, 222);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);

void setup() {
  Serial.begin(115200);

  // Wifi Setup
  if (!WiFi.config(static_IP, gateway, subnet)) {
    Serial.println("STA Failed to configure");
  }

  // Connect to the wifi router's network
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi..");
  }

  // Confirm the Static Address
  Serial.print("ESP32_0 IP Address: ");
  Serial.println(WiFi.localIP());

  // Begin listening on the UDP port
  udp.begin(port);

  ///Oled setup
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {  // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
  }
  delay(2000);
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  // Display static text
  OLED();

  // Setup thermocouple
   while (!Serial) delay(1); // wait for Serial on Leonardo/Zero, etc

  Serial.println("MAX31856 test");
  // wait for MAX chip to stabilize
  delay(500);
  Serial.println("Initializing sensor...");
  if (!thermocouple.begin()) {
    Serial.println("ERROR.");
    while (1) delay(10);
  }
    Serial.println("SENSOR DONE.");
    pinMode(MAXDRDY, INPUT);


  //// MOTOR setup////
  pinMode(OutDir, OUTPUT);
  pinMode(OutStep, OUTPUT);
  //outStep.low();
  pinMode(OutEnable, OUTPUT);
  digitalWrite(OutEnable, true);  // activate extruder motor
  digitalWrite(OutDir, true);     //Anti-Clockwise
  //float curSpeed = 0;
  Serial.println("Motor setup OK");
  currentMotorTime = millis();
  previousMotorTime = millis();

  //Motor Encoder setup
  pinMode(SPIN_A, INPUT_PULLUP);
  pinMode(SPIN_B, INPUT_PULLUP);
  pinMode(SPIN_BUTTON, INPUT_PULLUP);
  attachInterrupt(SPIN_B, scheckEncoder, CHANGE);
  Serial.println("Reading from encoder: MOTOR ");

  //Fan Encoder setup
  pinMode(FPIN_A, INPUT_PULLUP);
  pinMode(FPIN_B, INPUT_PULLUP);
  pinMode(FPIN_BUTTON, INPUT_PULLUP);
  attachInterrupt(FPIN_B, fcheckEncoder, CHANGE);
  Serial.println("Reading from encoder: FAN ");

  //Temp  Encoder setup
  pinMode(TPIN_A, INPUT_PULLUP);
  pinMode(TPIN_B, INPUT_PULLUP);
  pinMode(TPIN_BUTTON, INPUT_PULLUP);
  attachInterrupt(TPIN_B, tcheckEncoder, CHANGE);
  Serial.println("Reading from encoder: TEMP ");

  // Fan PWM output
  ledcSetup(FPWM_Ch, FPWM_Freq, FPWM_Res);
  ledcAttachPin(FPIN_OUT, FPWM_Ch);
  ledcWrite(FPWM_Ch, FPWM_DutyCycle);

  // Robot input
  //pinMode(robotIn, INPUT);

  //PID control
  pinMode(OutHeater, OUTPUT);
  windowStartTime = millis();

  // Tell the PID to range between 0 and the full window size
  myPID.SetOutputLimits(0, WindowSize);

  // Turn the PID on
  myPID.SetMode(AUTOMATIC);

  if (!thermocouple.begin()) {
    Serial.println("Could not initialize thermocouple.");
    while (1) delay(10);
  }
  thermocouple.setThermocoupleType(MAX31856_TCTYPE_K);
  thermocouple.setConversionMode(MAX31856_CONTINUOUS);
  // thermocouple.triggerOneShot();
  // delay(500);
  if (thermocouple.conversionComplete()) {
    curTemp = thermocouple.readThermocoupleTemperature();
    Serial.println(curTemp);
  } 
  else {
    Serial.println("Conversion not complete!");
  }
}

void loop() {
  //Check for UDP changes
  check_for_OSC_message();
  // motor control
  MotorEnable = digitalRead(true);
  // if (!digitalRead(MAXDRDY)) {
  //   curTemp = thermocouple.readThermocoupleTemperature();
  // Serial.println(curTemp);
  // }
  // if (thermocouple.conversionComplete()) {
  //   curTemp = thermocouple.readThermocoupleTemperature();
  //   Serial.println(curTemp);
  // } 
  if (MotorEnable && MotorSpeed != 0) {
    currentMotorTime = millis();
    //Serial.print(currentMotorTime);
    float StepPerMillis = MotorRev * MotorSpeed / 30000000;
    float MillisPerStep = 1 / StepPerMillis;
    //Serial.println(MillisPerStep);
    digitalWrite(OutStep, true);
    delayMicroseconds(MillisPerStep);
    digitalWrite(OutStep, false);
    delayMicroseconds(MillisPerStep);
  }
    // thermocouple.triggerOneShot();
    // if (thermocouple.conversionComplete()) {
    //   curTemp = thermocouple.readThermocoupleTemperature();
    //   Serial.println(curTemp);
    // } 
    // else {
    //   Serial.println("Conversion not complete!");
    // }
    //OLED();  // I kept your OLED update here

    // HEATER Controller with PID control only if HeaterEnable is true
    if (HeaterEnable) {
        myPID.Compute();

        /************************************************
        turn the output pin on/off based on pid output
        ************************************************/
        unsigned long now = millis();
        if (now - windowStartTime > WindowSize) {
            // Time to shift the Relay Window
            windowStartTime += WindowSize;
        }
        if (Output > now - windowStartTime) {
            digitalWrite(OutHeater, HIGH);
            //Serial.println("heater on");
        } else {
            digitalWrite(OutHeater, LOW);
            //Serial.println("heater off");
        }
    } 
    else {
        digitalWrite(OutHeater, LOW);
        //Serial.println("heater off due to HeaterEnable being false");
    }

  // motor Rotary encoder

  if (sturnedCW) {
    svalue++;
    //MotorSpeed += MotorStep;
    MotorSpeed = svalue * MotorStep;
    //Serial.print("Turned CW: ");
    //Serial.println(svalue);
    sturnedCW = false;
    slastWasCW = true;
    sdebounceTime = millis();
    OLED();
  }

  if (sturnedCCW) {
    if (!MotorSpeed <= 0) {
      svalue--;
      //MotorSpeed -= MotorStep;
      MotorSpeed = svalue * MotorStep;
    }
    //Serial.print("Turned CCW: ");
    //Serial.println(svalue);
    sturnedCCW = false;
    slastWasCCW = true;
    sdebounceTime = millis();
    OLED();
  }

  if ((millis() - sdebounceTime) > DEBONCE_TO) {
    slastWasCW = false;
    slastWasCCW = false;
  }

  int sbtnState = (digitalRead(SPIN_BUTTON));
  // Set MotorSpeed to Zero
  if (sbtnState == LOW) {
    if (millis() - slastButtonPress > DEBONCE_BTN) {
      if (!MotorSpeed == 0) {
        // Turn off the motor speed
        MotorSpeed = 0;
        // Ignore Robot's Motor Speed

        //Serial.print('\n');
      } else {
        // Go back to previous motor speed
        MotorSpeed = svalue * MotorStep;

      }
    }
    slastButtonPress = millis();
    OLED();
  }

//Temperature rotary encoder

 if (tturnedCW) {
    tvalue++;
    tarTemp = tvalue * 5;  // Adjusting in 5-degree increments
    //Serial.print("Turned CW: ");
    //Serial.println(tvalue);
    tturnedCW = false;
    tlastWasCW = true;
    tdebounceTime = millis();
    OLED();
  }

  if (tturnedCCW) {
    if (tarTemp > 0) {   // Assuming temperature cannot go below 0.
      tvalue--;
      tarTemp = tvalue * 5;  // Adjusting in 5-degree increments
    }
    //Serial.print("Turned CCW: ");
    //Serial.println(tvalue);
    tturnedCCW = false;
    tlastWasCCW = true;
    tdebounceTime = millis();
    OLED();
  }

  if ((millis() - tdebounceTime) > DEBONCE_TO) {
    tlastWasCW = false;
    tlastWasCCW = false;
  }

  int tbtnState = (digitalRead(TPIN_BUTTON));  // Assuming you have TEMP_BUTTON for temperature control
  // Set tarTemp to Default/Initial Value
  if (tbtnState == LOW) {
    if (millis() - tlastButtonPress > DEBONCE_BTN) {
      if (tarTemp != 0) {  // Resetting the tarTemp
        tarTemp = 0;
        HeaterEnable = false;
      } else {
        // Go back to previous temperature
        tarTemp = tvalue * 5; 
        HeaterEnable = true;

      }
    }
    tlastButtonPress = millis();
    OLED();
  }

// Fan Rotary encoder

if (fturnedCW) {
  fvalue++;
  if (fvalue > 10) {
    fvalue = 10;
  }
  FanSpeed = fvalue * FanStep;
  // map(val, incoming_min, incoming_max, desired_min, desired_max);
  //Serial.println(FanSpeed);
  updateFan();
  //Serial.println(fvalue);
  fturnedCW = false;
  flastWasCW = true;
  flastWasCCW = false;
  fdebounceTime = millis();
  OLED();
}

if (fturnedCCW) {
  if (!FanSpeed == 0) {
    fvalue--;
    FanSpeed = fvalue * FanStep;
  }
  //Serial.println(FanSpeed);
  updateFan();
  //Serial.println(fvalue);
  fturnedCCW = false;
  flastWasCCW = true;
  flastWasCW = false;
  fdebounceTime = millis();
  OLED();
}

if ((millis() - fdebounceTime) > DEBONCE_TO) {
  flastWasCW = false;
  flastWasCCW = false;
}
int fbtnState = (digitalRead(FPIN_BUTTON));
if (fbtnState == LOW) {
  if (millis() - flastButtonPress > DEBONCE_BTN) {
    if (!FanSpeed == 0) {
      // Serial.println("Fan OFF");
      // Serial.println(String(FanEnable));
      FanSpeed = 0;
      FPWM_DutyCycle = map(FanSpeed, 0, 100, 0, 255);
      ledcWrite(FPWM_Ch, FPWM_DutyCycle);
      //digitalWrite(OutFan, FanEnable);
      //Serial.print('\n');
    } else {
      // Serial.println("Fan ON");
      // Serial.println(String(FanEnable));
      FanSpeed = fvalue * FanStep;
      FPWM_DutyCycle = map(FanSpeed, 0, 100, 0, 255);
      ledcWrite(FPWM_Ch, FPWM_DutyCycle);
      //digitalWrite(OutFan, FanEnable);
      //Serial.print('\n');
    }
  }
  flastButtonPress = millis();
  OLED();
}
}

void scheckEncoder() {
  if ((!sturnedCW) && (!sturnedCCW)) {
    int spinA = digitalRead(SPIN_A);
    delayMicroseconds(1500);
    int spinB = digitalRead(SPIN_B);
    if (spinA == spinB) {
      if (slastWasCW) {
        sturnedCW = true;
      } else {
        sturnedCCW = true;
      }
    } else {
      if (slastWasCCW) {
        sturnedCCW = true;
      } else {
        sturnedCW = true;
      }
    }
  }
}
void tcheckEncoder() {
  if ((!tturnedCW) && (!tturnedCCW)) {
    int tpinA = digitalRead(TPIN_A);
    delayMicroseconds(1500);
    int tpinB = digitalRead(TPIN_B);
    if (tpinA == tpinB) {
      if (tlastWasCW) {
        tturnedCW = true;
      } else {
        tturnedCCW = true;
      }
    } else {
      if (tlastWasCCW) {
        tturnedCCW = true;
      } else {
        tturnedCW = true;
      }
    }
  }
}
void fcheckEncoder() {
  if ((!fturnedCW) && (!fturnedCCW)) {
    int fpinA = digitalRead(FPIN_A);
    delayMicroseconds(1500);
    int fpinB = digitalRead(FPIN_B);
    if (fpinA == fpinB) {
      if (flastWasCW) {
        fturnedCW = true;
      } else {
        fturnedCCW = true;
      }
    } else {
      if (flastWasCCW) {
        fturnedCCW = true;
      } else {
        fturnedCW = true;
      }
    }
  }
}
void updateFan(void){
    FPWM_DutyCycle = map(FanSpeed, 0, 100, 0, 255);
    ledcWrite(FPWM_Ch, FPWM_DutyCycle);
}
void OLED(void) {
  //cTemp = module.readCelsius();
  display.clearDisplay();
  display.setCursor(0, 0);
  //display.println("Motor " + String(bool(digitalRead(robotIn))));
  display.println("Speed " + String(int(MotorSpeed)));
  display.println("Fan " + String(int(FanSpeed)) + " %");
  // display.println("H " + String(HeaterEnable));
  display.println(String(int(curTemp)) + "/" + String(int(tarTemp)) + " C");
  display.display();
}
void on_message_received(OSCMessage& msg) {
  // Get the LED index from the message address
  // Get the message address
  char msg_addr[255];
  msg.getAddress(msg_addr);
  Serial.println(msg_addr);
  String addr = "";
  addr += msg_addr;

  if (addr == "/MotorSpeed") {
    MotorSpeed = msg.getInt(0);
    Serial.println(MotorSpeed);
  } 

  else if (addr == "/FanSpeed") {
    FanSpeed = msg.getInt(0);
    updateFan();
    Serial.println(FanSpeed);
  }

  else if (addr == "/tarTemp") {
    tarTemp = msg.getInt(0);
    Serial.println(tarTemp);
  }

  // else if (addr == "/enable") {
  //   enable = msg.getBool(0);
  //   Serial.println(enable);
  // }
  OLED();
}

void check_for_OSC_message() {
  OSCMessage msg;
  int size = udp.parsePacket();
  if (size > 0) {
    while (size--) {
      msg.fill(udp.read());
    }
    if (!msg.hasError()) {
      // Get the message address
      char msg_addr[255];
      msg.getAddress(msg_addr);
      Serial.print("MSG_ADDR: ");
      Serial.println(msg_addr);

      // if (!msg.isBundle()) {
      msg.dispatch(msg_addr, on_message_received);

    }
  }
}
