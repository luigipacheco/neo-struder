/// Import libraries///
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <max6675.h>
#include <ezOutput.h>

// Display variables///
#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 64  // OLED display height, in pixels
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// Rotary encoder variables
//Motor
#define SPIN_A 2
#define SPIN_B 4
#define SPIN_BUTTON 15

//Fan
#define FPIN_A 5
#define FPIN_B 18
#define FPIN_BUTTON 19

#define FPIN_OUT 14
#define FPWM_Ch 1
#define FPWM_Res   8
#define FPWM_Freq  250000  //1000 or 250 000
 
int FPWM_DutyCycle = 0;

//Temperature
//#define TPIN_A      2
//#define TPIN_B      4
//#define TPIN_BUTTON 15

static int svalue = 0;
static int fvalue = 0;
int DEBONCE_TO = 150;
int DEBONCE_BTN = 200;
volatile bool sturnedCW = false;
volatile bool sturnedCCW = false;
volatile bool fturnedCW = false;
volatile bool fturnedCCW = false;
// volatile bool turnedCW = false;
// volatile bool turnedCCW = false;

unsigned long slastButtonPress = 0;
unsigned long sdebounceTime = 0;

unsigned long flastButtonPress = 0;
unsigned long fdebounceTime = 0;

bool slastWasCW = false;
bool slastWasCCW = false;

bool flastWasCW = false;
bool flastWasCCW = false;

//// Heater Variables ////
int OutHeater = 3;
int OutFan = 6;
bool HeaterEnable = false;
bool FanEnable = false;
int FanSpeed = 0;
int FanStep = 10;
float cTemp = 0;
float sTemp = 200;
unsigned long LastReadTime = 0;
int SO = 12;
int CS = 10;
int sck = 13;
//MAX6675 module(sck, CS, SO);

//motor variables
int OutStep = 25;
int OutDir = 26;
int OutEnable = 27;
bool MotorEnable = true;
float MotorRev = 800;
float MotorSpeed = 0;  // rev per minute
int MotorStep = 25;
unsigned long LastRunTime = 0;
unsigned long currentMotorTime = 0;
unsigned long previousMotorTime = 0;
long motorInterval = 0.1;


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
  Serial.println("OLED setup OK");
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
  Serial.println("Reading from encoder: ");

  //Fan Encoder setup
  pinMode(FPIN_A, INPUT_PULLUP);
  pinMode(FPIN_B, INPUT_PULLUP);
  pinMode(FPIN_BUTTON, INPUT_PULLUP);
  attachInterrupt(FPIN_B, fcheckEncoder, CHANGE);
  Serial.println("Reading from encoder: ");

  // Fan PWM output
  ledcSetup(FPWM_Ch,FPWM_Freq,FPWM_Res);
  ledcAttachPin(FPIN_OUT, FPWM_Ch);
  ledcWrite(FPWM_Ch,FPWM_DutyCycle);
}

void loop() {
  // motor control
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

  // motor Rotary encoder

  if (sturnedCW) {
    svalue++;
    //MotorSpeed += MotorStep;
    MotorSpeed = svalue*MotorStep;
    //Serial.print("Turned CW: ");
    Serial.println(svalue);
    sturnedCW = false;
    slastWasCW = true;
    sdebounceTime = millis();
    OLED();
  }

  if (sturnedCCW) {
    if (!MotorSpeed <= 0) {
      svalue--;
      //MotorSpeed -= MotorStep;
      MotorSpeed = svalue*MotorStep;
    }
    //Serial.print("Turned CCW: ");
    Serial.println(svalue);
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
  if (sbtnState == LOW) {
    if (millis() - slastButtonPress > DEBONCE_BTN) {
      if (!MotorSpeed == 0) {
        MotorSpeed = 0;
        //digitalWrite(OutFan, FanEnable);
        //Serial.print('\n');
      } else {
         MotorSpeed =svalue*MotorStep ;
      }
    }
    slastButtonPress = millis();
    OLED();
  }

  // Fan Rotary encoder

  if (fturnedCW) {
    fvalue++;
    if(fvalue>10){
      fvalue =10;
    }
    FanSpeed = fvalue*FanStep;
    // map(val, incoming_min, incoming_max, desired_min, desired_max);
    Serial.println(FanSpeed);
    FPWM_DutyCycle = map(FanSpeed, 0, 100, 0, 255);
    ledcWrite(FPWM_Ch, FPWM_DutyCycle);
    Serial.println(fvalue);
    fturnedCW = false;
    flastWasCW = true;
    flastWasCCW = false;
    fdebounceTime = millis();
    OLED();
  }

  if (fturnedCCW) {
    if (!FanSpeed == 0) {
      fvalue--;
      //MotorSpeed -= MotorStep;
      FanSpeed = fvalue*FanStep;
    }
    Serial.println(FanSpeed);
    FPWM_DutyCycle = map(FanSpeed, 0, 100, 0, 255);
    ledcWrite(FPWM_Ch, FPWM_DutyCycle);
    Serial.println(fvalue);
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
        FanSpeed = fvalue *FanStep;
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
  display.println("Speed " + String(int(MotorSpeed)));
  display.println("Fan " + String(int(FanSpeed))+ " %");
  // display.println("H " + String(HeaterEnable));
  display.println(String(int(cTemp))+ "/" + String(int(cTemp))+" C");
  display.display();
}
