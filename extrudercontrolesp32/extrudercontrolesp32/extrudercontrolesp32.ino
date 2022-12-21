/// Test for esp32  , 3mm noozle on massive dimension, range goes from 40 - 600 RPM///

int OutStep = 19;
int OutDir = 18;
int OutEnable = 5;
int pulselength = 7200;
int  inRPM = 15;
int potValue = 0;

int desiredRPM = 0;

void setup() {
  Serial.begin(9600);
  pinMode(OutStep, OUTPUT);
  pinMode(OutEnable, OUTPUT);
  digitalWrite(OutEnable, true);  // activate extruder motor
  digitalWrite(OutDir, false);    //Anti-Clockwise
  delay(1000);

}

void loop() {
  int val = analogRead(inRPM);
  //Serial.println(val);
  if (val != 0){
    pulselength = map(val, 1, 4095, 1024, 32);
    digitalWrite(OutStep, true);  //Anti-Clockwise
    delayMicroseconds(pulselength);
    digitalWrite(OutStep, false);  //Anti-Clockwise
    delayMicroseconds(pulselength);
    //Serial.println(pulselength);
  }
}
