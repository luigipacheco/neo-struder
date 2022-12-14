/// Import libraries///
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "max6675.h"

// Display variables///
#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 64  // OLED display height, in pixels
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

//// MOTOR Variables ////

int OutStep = 7;
int OutDir = 8;
int OutEnable = 9;
bool MotorEnable = true;
float MotorRev = 800;
float MotorSpeed = 0;  // rev per minute
int MotorStep = 5;
unsigned long LastRunTime = 0;


//// Heater Variables ////
int OutHeater = 3;
int OutFan = 6;
bool HeaterEnable = false;
bool FanEnable = false;
float cTemp = 0;
float sTemp = 200;
unsigned long LastReadTime = 0;
int SO = 12;
int CS = 10;
int sck = 13;
MAX6675 module(sck, CS, SO);

//// Robot Variables ////
int inRobot = 11;

//// Encoder Variables ////

int counter = 0;
int clk = 2;
int dt = 4;
int btn = 5;
int aState;
int aLastState;
unsigned long lastButtonPress = 0;
const int DEBOUNCE_DELAY = 100;


void setup() {
    //// COM setup////
  Serial.begin(115200);
  Serial.println("Hello");
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
  pinMode(OutEnable, OUTPUT);
  digitalWrite(OutEnable, true); // activate extruder motor
  digitalWrite(OutDir, false);  //Anti-Clockwise
  Serial.println("Motor setup OK");

  //// Heater setup////
  pinMode(OutHeater, OUTPUT);
  pinMode(OutFan, OUTPUT);
  Serial.println("Heater setup OK");


  //// Encoder Setup ////
  pinMode(clk, INPUT);
  pinMode(dt, INPUT);
  pinMode(btn, INPUT_PULLUP);
  aLastState = digitalRead(clk);
  Serial.println("Encoder setup OK");
}

void loop() {

  digitalWrite(OutFan, FanEnable);

  // MOTOR Controller
  int timeElapsed = millis() - LastRunTime;
  LastRunTime = millis();
  if (MotorEnable && MotorSpeed != 0) {
    float StepPerMillis = MotorRev * MotorSpeed / 60000;  //400 step per rev * 10 rev per minute / 60 000 = 0.0666 per milisec = 66.6 per sec = 4000 per minute
    Serial.println(StepPerMillis);
    float MillisPerStep = 1 / StepPerMillis;              // 0.0666 step per milisec = 66.6 per sec = 4000 per minute
    Serial.println(MillisPerStep);

    for (int steps = 0; steps <= timeElapsed / (MillisPerStep + 1); steps++) {
      digitalWrite(OutStep, HIGH);
      delayMicroseconds(MillisPerStep * 1000 / 2);
      digitalWrite(OutStep, LOW);
      delayMicroseconds(MillisPerStep * 1000 / 2);
    }
  }

  // HEATER Controller TODO PID control
  // int timeElapsedRead = millis() - LastReadTime;
  // if (timeElapsedRead > 1000) {  // if 5 sec passed
  //   LastReadTime = millis();

  //   cTemp = module.readCelsius();
  //   OLED();
  //   Serial.println(cTemp);
  //   if (cTemp < sTemp && !HeaterEnable) {
  //     HeaterEnable = true;
  //     digitalWrite(OutHeater, HeaterEnable);
  //     Serial.println("heater on");
  //   }
  //   if (cTemp > sTemp && HeaterEnable) {
  //     HeaterEnable = false;
  //     digitalWrite(OutHeater, HeaterEnable);
  //     Serial.println("heater off");
  //   }
  // }

  // Encoder and button
  aState = digitalRead(clk);  // Reads the "current" state of the outputA
  // If the previous and the current state of the outputA are different, that means a Pulse has occured
  if (aState != aLastState) {
    // If the outputB state is different to the outputA state, that means the encoder is rotating clockwise
    if (digitalRead(dt) != aState) {
      counter--;
      MotorSpeed -= MotorStep;

    } else {
      counter++;
      MotorSpeed += MotorStep;
    }
    OLED();
    Serial.print("Position: ");
    Serial.println(counter);
    Serial.println(MotorSpeed);
  }
  aLastState = aState;  // Updates the previous state of the outputA with the current state
  int btnState = (digitalRead(btn));
  if (btnState == LOW) {
    if (millis() - lastButtonPress > DEBOUNCE_DELAY) {
      if (!FanEnable) {
        Serial.print("Fan OFF");
        Serial.print(String(FanEnable));
        FanEnable = true;
        //digitalWrite(OutFan, FanEnable);
        Serial.print('\n');
      } else {
        Serial.print("Fan ON");
        Serial.print(String(FanEnable));
        FanEnable = false;
        //digitalWrite(OutFan, FanEnable);
        Serial.print('\n');
      }
    }
    OLED();
    lastButtonPress = millis();
  }
}

void OLED(void) {
  cTemp = module.readCelsius();
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Speed " + String(int(MotorSpeed)));
  display.println("Fan " + String(FanEnable));
  // display.println("H " + String(HeaterEnable));
  // display.println("T " + String(cTemp));
  display.display();
  return true;
}