#include <PID_v1.h>  //PID loop from http://playground.arduino.cc/Code/PIDLibrary

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
  pinMode(2, INPUT);
  pinMode(A0, INPUT);
  pinMode(5,OUTPUT);
  pinMode(6,OUTPUT);
  attachInterrupt(0, timeit, CHANGE);
  
  Serial.begin(115200);
  Serial.println("VIA");

  /*while (1)  {
    analogWrite(5,128);
    analogWrite(6,0);
    delay(1000);
    analogWrite(5,0);
    analogWrite(6,128);
    delay(1000);
  }*/

  PID1.SetMode(AUTOMATIC);
  PID1.SetOutputLimits(-255, 255);
  PID1.SetSampleTime(20);
}


void timeit() {
    if (digitalRead(2) == HIGH)
      start = micros();
    else
      pwm = micros() - start;
  }


void loop() {
  currentMillis = millis();
  if (currentMillis - previousMillis < interval) return;
  previousMillis = currentMillis;

  pot = analogRead(A0);

  /*Serial.println(pwm);
  delay(200);*/

  setpoint1 = map(constrain(pwm,1000,2000),1000,2000,-255,255);
  input1 = map(pot,0,1023,-255,255);
  PID1.Compute();

  //Serial.println(Output1);    

  if (abs(output1)<20) {
      analogWrite(6, 0);
      analogWrite(5, 0);
  }
  else
    if (output1 > 0) {
      analogWrite(5, output1);
      analogWrite(6, 0);
    }
    else if (output1 < 0) {
      analogWrite(5, 0);
      analogWrite(6, -output1);
    }
}



