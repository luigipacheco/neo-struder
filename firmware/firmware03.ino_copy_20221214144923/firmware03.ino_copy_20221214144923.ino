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
int OutHeater = 5;
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
  ///Oled setup
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {  // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
  }
  delay(2000);
  display.clearDisplay();
  HeaterTemp = module.readCelsius();
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(0, 10);
  // Display static text
  delay(2000);
  OLED();
  display.display();
  //// MOTOR setup////
  pinMode(OutDir, OUTPUT);
  pinMode(OutStep, OUTPUT);
  pinMode(OutEnable, OUTPUT);
  digitalWrite(OutEnable, true);
  digitalWrite(OutDir, false);  //Anti-Clockwise

  //// Heater setup////
  pinMode(OutHeater, OUTPUT);
  pinMode(OutFan, OUTPUT);

  //analogReference(EXTERNAL);

  //// COM setup////
  inputString.reserve(200);
  Serial.begin(115200);
  Serial.println("Hello");

  //// Encoder Setup ////
  pinMode(clk, INPUT);
  pinMode(dt, INPUT);
  pinMode(btn, INPUT_PULLUP);
  aLastState = digitalRead(clk);

  /// Display

  //Init
  CMD("?");  //Display Default values
  //CMD("T 205");
  //CMD("F 1");
  //CMD("S 30"); //Set speed in Rev/min

  //CMD("M 1"); //Motor ON
}

void loop() {
  // Temp
  // Fan
  digitalWrite(OutFan, FanEnable);

  // MOTOR Controller
  int timeElapsed = millis() - LastRunTime;
  LastRunTime = millis();
  if (MotorEnable && MotorSpeed != 0 && FanEnable) {
    float StepPerMillis = MotorRev * MotorSpeed / 60000;  //400 step per rev * 10 rev per minute / 60 000 = 0.0666 per milisec = 66.6 per sec = 4000 per minute
    float MillisPerStep = 1 / StepPerMillis;              // 0.0666 step per milisec = 66.6 per sec = 4000 per minute

    //Serial.println((String)timeElapsed + " elapsed");


    //Serial.println((String)(timeElapsed/(MillisPerStep+1)) + " Steps");

    for (int steps = 0; steps <= timeElapsed / (MillisPerStep + 1); steps++) {
      digitalWrite(OutStep, HIGH);
      delayMicroseconds(MillisPerStep * 1000 / 2);
      digitalWrite(OutStep, LOW);
      delayMicroseconds(MillisPerStep * 1000 / 2);
    }
  }

  // HEATER Controller
  int timeElapsedRead = millis() - LastReadTime;
  if (timeElapsedRead > 5000) {  // if 5 sec passed
    float reading =0; //analogRead(InTemp); 
    LastReadTime = millis();

    // convert the reading value to resistance
    int SERIESRESISTOR = 4.7;     //KOhm
    int THERMISTORNOMINAL = 100;  //KOhm
    int TEMPERATURENOMINAL = 25;
    int BCOEFFICIENT = 3990;
    float RESISTANCECORRECTION = 2.5;

    reading = (1023 * 1.4) / reading - 1;  //M-Duino Analog is on 10V, signal is on 14V. max>1024)
    reading = SERIESRESISTOR / reading;
    reading = reading * RESISTANCECORRECTION;

    // convert the reading to degree
    float steinhart;
    steinhart = reading / THERMISTORNOMINAL;           // (R/Ro)
    steinhart = log(steinhart);                        // ln(R/Ro)
    steinhart /= BCOEFFICIENT;                         // 1/B * ln(R/Ro)
    steinhart += 1.0 / (TEMPERATURENOMINAL + 273.15);  // + (1/To)
    steinhart = 1.0 / steinhart;                       // Invert
    steinhart -= 273.15;                               // convert to C

    if (steinhart > 0)
      HeaterTempActual = steinhart;
    Serial.println("T " + (String)HeaterTempActual + " / " + (String)HeaterTemp + " °C");

    if (HeaterTempActual < HeaterTemp && !HeaterEnable) {
      HeaterEnable = true;
      digitalWrite(OutHeater, HeaterEnable);
      Serial.println("H 1 Heater ON");
    }
    if (HeaterTempActual > HeaterTemp && HeaterEnable) {
      HeaterEnable = false;
      digitalWrite(OutHeater, HeaterEnable);
      Serial.println("H 0 Heater OFF");
    }
  }


  // COM
  if (stringComplete) {
    CMD(inputString);
    // clear the string:
    inputString = "";
    stringComplete = false;
  }

  while (Serial.available() > 0) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
  // Encoder and button
  aState = digitalRead(clk);  // Reads the "current" state of the outputA
  // If the previous and the current state of the outputA are different, that means a Pulse has occured
  if (aState != aLastState) {
    // If the outputB state is different to the outputA state, that means the encoder is rotating clockwise
    if (digitalRead(dt) != aState) {
      counter++;
      MotorSpeed += MotorStep;

    } else {
      counter--;
      MotorSpeed -= MotorStep;
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
        digitalWrite(OutFan, FanEnable);
        Serial.print('\n');
      } else {
        Serial.print("Fan ON");
        Serial.print(String(FanEnable));
        FanEnable = false;
        digitalWrite(OutFan, FanEnable);
        Serial.print('\n');
      }
    }
    OLED();
    lastButtonPress = millis();
  }
}

boolean CMD(String inString) {
  String inChar = inString.substring(0, 1);
  inChar.toUpperCase();
  if (inChar == "S") {
    MotorSpeed = inString.substring(1).toFloat();
    Serial.println("S " + (String)MotorSpeed + " rev/min");
    return true;
  } else if (inChar == "M") {
    MotorEnable = inString.substring(1).toInt();
    digitalWrite(OutEnable, MotorEnable);
    Serial.print("M " + (String)MotorEnable);
    if (MotorEnable) Serial.println(" Motor ON");
    else Serial.println(" Motor OFF");
    return true;
  } else if (inChar == "H") {
    HeaterEnable = inString.substring(1).toInt();
    digitalWrite(OutHeater, HeaterEnable);
    Serial.print("H " + (String)HeaterEnable);
    if (HeaterEnable) Serial.println(" Heater ON");
    else Serial.println(" Heater OFF");
    return true;
  } else if (inChar == "F") {
    FanEnable = inString.substring(1).toInt();
    digitalWrite(OutFan, FanEnable);
    Serial.print("F " + (String)FanEnable);
    if (FanEnable) Serial.println(" Fan ON");
    else Serial.println(" Fan OFF");
    return true;
  } else if (inChar == "T") {
    HeaterTemp = inString.substring(1).toFloat();
    Serial.println("T " + (String)HeaterTemp + " °C");
    return true;
  } else if (inChar == "?") {
    Serial.println("------------");
    Serial.print("M " + (String)MotorEnable);
    if (MotorEnable) Serial.println(" Motor ON");
    else Serial.println(" Motor OFF");
    Serial.println("S " + (String)MotorSpeed + " rev/min");
    Serial.print("H " + (String)HeaterEnable);
    if (HeaterEnable) Serial.println(" Heater ON");
    else Serial.println(" Heater OFF");
    Serial.print("F " + (String)FanEnable);
    if (FanEnable) Serial.println(" Fan ON");
    else Serial.println(" Fan OFF");
    Serial.println("T " + (String)HeaterTemp + " °C");
    Serial.println("------------");
    return true;
  } else {
    Serial.print("Error processing :");
    Serial.print(inString);
    return false;
  }
}

boolean OLED() {
  HeaterTemp = module.readCelsius();
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Speed " + String(int(MotorSpeed)));
  display.println("Fan " + String(FanEnable));
  display.println("Temp " + String(HeaterTemp));
  display.display();
  return true;
}