#include <FastPID.h>

#include <NewPing.h>

#include <IRremote.h>
#include <ir_Lego_PF_BitStreamEncoder.h>
#include <IRremoteInt.h>
#include <boarddefs.h>

#define PWM_PINA 5
#define PWM_PINB 6
#define DIR_A 4
#define DIR_B 7
#define TRIG_PIN 11
#define MAX_DISTANCE 200
#define ECHO_PIN0 12

enum {MOTORB, MOTORA};

int speedA, speedB = 0;
int directionA, directionB = HIGH;

float Kp=0.1, Ki=0.5, Kd=0;
int output_bits = sizeof ((unsigned long) 0);
bool output_signed = false;

FastPID forward_pid(Kp, Ki, Kd, output_bits, output_signed);

NewPing forward_sonar(TRIG_PIN, ECHO_PIN0, MAX_DISTANCE);

void setup() {
  // put your setup code here, to run once
  pinMode(PWM_PINA, OUTPUT);
  pinMode(PWM_PINB, OUTPUT);
  pinMode(DIR_A, OUTPUT);
  pinMode(DIR_B, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN0, INPUT);

  Serial.begin(9600);

  pinMode(13, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  //setmotor(0,0,MOTORA);
  unsigned long dist = forward_sonar.ping_cm();
  Serial.println(dist);
  if (dist < 20 && dist!=0){/*
      unsigned long setpoint = 20; 
      unsigned long feedback = dist;
      forward(forward_pid.step(setpoint, feedback));*/
      forward(0);
    }
  else forward(100);
  delay(50);
}

void setmotor(int speed, int direction, int motor) {
  speed = direction?255-speed:speed;
  int pwm_pin = motor?PWM_PINA:PWM_PINB;
  int dir_pin = motor?DIR_A:DIR_B;
  motor?speedA=speed:speedB=speed; 
  motor?directionA=direction:directionB=direction; 
  analogWrite(pwm_pin, speed);
  digitalWrite(dir_pin, direction);
}

void turnRight(int speed) {
  setmotor(speed, 1, MOTORB);
  setmotor(speed, 0, MOTORA);
}

void turnLeft(int speed) {
  setmotor(speed, 0, MOTORB);
  setmotor(speed, 1, MOTORA);
}

void forward(int speed) {
  setmotor(speed, 1, MOTORA);
  setmotor(speed, 1, MOTORB);
}

void reverse(int speed) {
  setmotor(speed, 0, MOTORA);
  setmotor(speed, 0, MOTORB);
}

