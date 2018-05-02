#include <newPing.h>
#include <IRremote.h>

#define PWM_PINA 5
#define PWM_PINB 6
#define DIR_A 4
#define DIR_B 7
#define TRIG_PIN 11
#define ECHO_PIN0 12

enum {MOTORB, MOTORA};

int speedA, speedB = 0;
int directionA, directionB = HIGH;

void setup() {
  // put your setup code here, to run once
  pinMode(PWM_PINA, OUTPUT);
  pinMode(PWM_PINB, OUTPUT);
  pinMode(DIR_A, OUTPUT);
  pinMode(DIR_B, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN0, INPUT);

  pinMode(13, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  //setmotor(0,0,MOTORA);
  forward(200);
  delay(1000);
  //digitalWrite(13, LOW);
  //turnRight();
  delay(1000);
  //digitalWrite(13, HIGH);
  //turnRight();
  //delay(1000);
  //digitalWrite(13, LOW);
  //reverse(200);
  //delay(1000);
  //digitalWrite(13, HIGH);
  forward(0);
  while (1);
}

void setmotor(int speed, int direction, int motor) {
  speed = direction?255-speed:speed;
  int pwm_pin = motor?PWM_PINA:PWM_PINB;
  int dir_pin = motor?DIR_B:DIR_A;
  motor?speedA=speed:speedB=speed; 
  motor?directionA=direction:directionB=direction; 
  analogWrite(pwm_pin, speed);
  digitalWrite(dir_pin, direction);
}

void turnRight() {
  setmotor(200, 1, MOTORB);
  setmotor(200, 0, MOTORA);
}

void turnLeft() {
  setmotor(200, 0, MOTORB);
  setmotor(200, 1, MOTORA);
}

void forward(int speed) {
  setmotor(speed, 1, MOTORA);
  setmotor(speed, 1, MOTORB);
}

void reverse(int speed) {
  setmotor(speed, 0, MOTORA);
  setmotor(speed, 0, MOTORB);
}

