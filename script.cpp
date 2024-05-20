#include <Arduino.h>
#include <PID_v1_bc.h>

#define FLOW_PIN 13
#define IN1 27
#define IN2 26
#define EN 14

volatile int flowCount = 0;
unsigned long oldTime = 0;
float calibration = 4.5;
float flowRate = 0.0;
double setpoint = 3.0;
double input, output;

double aggKp = 4, aggKi = 0.2, aggKd = 1;
double consKp = 1, consKi = 0.05, consKd = 0.25;

PID myPID(&input, &output, &setpoint, consKp, consKi, consKd, DIRECT);

void IRAM_ATTR flowISR() {
  flowCount++;
}

void setup() {
  Serial.begin(115200);
  
  pinMode(FLOW_PIN, INPUT_PULLUP);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  
  ledcSetup(0, 5000, 8);
  ledcAttachPin(EN, 0);
  
  attachInterrupt(digitalPinToInterrupt(FLOW_PIN), flowISR, FALLING);
  
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(1000);
  
  ledcWrite(0, 255);
}

void loop() {
  unsigned long currTime = millis();
  
  if (currTime - oldTime >= 1000) {
    flowRate = (flowCount / calibration) * 60.0;
    flowCount = 0;
    oldTime = currTime;
  }
  
  input = flowRate;
  
  double gap = abs(setpoint - input);
  if (gap < 0.5) {
    myPID.SetTunings(consKp, consKi, consKd);
  } else {
    myPID.SetTunings(aggKp, aggKi, aggKd);
  }
  
  myPID.Compute();
  
  int pwmValue = constrain(output, 0, 255);
  
  ledcWrite(0, pwmValue);
  
  Serial.print("Flow Rate: ");
  Serial.print(flowRate);
  Serial.print(" L/min\t");
  Serial.print("PWM: ");
  Serial.println(pwmValue);
}