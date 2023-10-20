/// Import libraries///
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <SPI.h>
#include "Adafruit_MAX31855.h"
#include <ezOutput.h>
#include <PID_v1.h>


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

#define robotIn 12  // start/stop extruder

int FPWM_DutyCycle = 0;

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
double Output;
const unsigned long WindowSize = 5000;
unsigned long windowStartTime;
int OutHeater = 13;
int OutFan = 6;
bool HeaterEnable = false;
bool FanEnable = false;
int FanSpeed = 0;
int FanStep = 10;
double curTemp = 0;
double tarTemp = 0;
unsigned long LastReadTime = 0;
int SO = 33;
int CS = 32;
int clk = 35;
PID myPID(&curTemp, &Output, &tarTemp, 2, 5, 1, DIRECT);
Adafruit_MAX31855 thermocouple(clk, CS, SO);


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


void setup() {
  Serial.begin(115200);
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
  Serial.print("Initializing sensor...");
  if (!thermocouple.begin()) {
    Serial.println("ERROR.");
    while (1) delay(10);
  }

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
  pinMode(robotIn, INPUT);

  //PID control
  pinMode(OutHeater, OUTPUT);
  windowStartTime = millis();

  // Tell the PID to range between 0 and the full window size
  myPID.SetOutputLimits(0, WindowSize);

  // Turn the PID on
  myPID.SetMode(AUTOMATIC);
  curTemp = thermocouple.readCelsius();
  if (isnan(curTemp)) {
     Serial.println("Thermocouple fault(s) detected!");
     uint8_t e = thermocouple.readError();
     if (e & MAX31855_FAULT_OPEN) Serial.println("FAULT: Thermocouple is open - no connections.");
     if (e & MAX31855_FAULT_SHORT_GND) Serial.println("FAULT: Thermocouple is short-circuited to GND.");
     if (e & MAX31855_FAULT_SHORT_VCC) Serial.println("FAULT: Thermocouple is short-circuited to VCC.");
   } else {
     Serial.print("C = ");
     Serial.println(curTemp);
   }

}

void loop() {
  // motor control
  MotorEnable = digitalRead(robotIn);
  
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
    curTemp = thermocouple.readCelsius();
    // HEATER Controller with PID control only if HeaterEnable is true
    if (HeaterEnable) {
        curTemp = thermocouple.readCelsius();  // Update the current temperature from the module
        OLED();  // I kept your OLED update here
        //Serial.println(curTemp);

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
  FPWM_DutyCycle = map(FanSpeed, 0, 100, 0, 255);
  ledcWrite(FPWM_Ch, FPWM_DutyCycle);
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
  FPWM_DutyCycle = map(FanSpeed, 0, 100, 0, 255);
  ledcWrite(FPWM_Ch, FPWM_DutyCycle);
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


void OLED(void) {
  //cTemp = module.readCelsius();
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Motor " + String(bool(digitalRead(robotIn))));
  display.println("Speed " + String(int(MotorSpeed)));
  display.println("Fan " + String(int(FanSpeed)) + " %");
  // display.println("H " + String(HeaterEnable));
  display.println(String(int(curTemp)) + "/" + String(int(tarTemp)) + " C");
  display.display();
}
