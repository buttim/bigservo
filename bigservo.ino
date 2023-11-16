#include <PID_v1.h>  //PID loop from http://playground.arduino.cc/Code/PIDLibrary
#include <FastPwmPin.h>

#define RPWM 9
#define LPWM 10
#define PWMIN 2
#define POTENTIOMETER A0
double Pk1 = 1;  //speed it gets there
double Ik1 = 0;
double Dk1 = 0;

double setpoint1, input1, output1, Output1a;    // PID variables

PID PID1(&input1, &output1, &setpoint1, Pk1, Ik1 , Dk1, DIRECT);    // PID Setup

volatile unsigned long pwm;
unsigned long start;

int pot;

unsigned long currentMillis, previousMillis = 0;
const long interval = 4;

void setup() {
  pinMode(PWMIN, INPUT);
  pinMode(POTENTIOMETER, INPUT);
  pinMode(LPWM,OUTPUT);
  pinMode(RPWM,OUTPUT);
  FastPwmPin::enablePwmPin(LPWM, 22000L, 0);
  FastPwmPin::enablePwmPin(RPWM, 22000L, 0);
  attachInterrupt(digitalPinToInterrupt(PWMIN), timeit, CHANGE);
  
  //Serial.begin(115200);
  //Serial.println("VIA");

  PID1.SetMode(AUTOMATIC);
  PID1.SetOutputLimits(-255, 255);
  PID1.SetSampleTime(20);
}


void timeit() {
    if (digitalRead(PWMIN) == HIGH)
      start = micros();
    else
      pwm = micros() - start;
  }


void loop() {
  currentMillis = millis();
  if (currentMillis - previousMillis < interval) return;
  previousMillis = currentMillis;

  pot = analogRead(POTENTIOMETER);

  /*Serial.println(pwm);
  delay(200);*/

  setpoint1 = map(constrain(pwm,1000,2000),1000,2000,-255,255);
  input1 = map(pot,0,1023,-255,255);
  PID1.Compute();

  //Serial.println(Output1);    

  if (abs(output1)<20) {
      analogWrite(LPWM, 0);
      analogWrite(RPWM, 0);
  }
  else
    if (output1 > 0) {
      analogWrite(LPWM, output1);
      analogWrite(RPWM, 0);
    }
    else if (output1 < 0) {
      analogWrite(LPWM, 0);
      analogWrite(RPWM, -output1);
    }
}



