#include <NewPing.h>

#include <IRremote.h>
#include <ir_Lego_PF_BitStreamEncoder.h>
#include <IRremoteInt.h>
#include <boarddefs.h>

#define PWM_PINA 6  // Motor control pins
#define PWM_PINB 5
#define DIR_A 7
#define DIR_B 4

#define FRONT_HIGH_TRIG_PIN 30  // Ultrasonic pins
#define FRONT_HIGH_ECHO_PIN 31
#define FRONT_LOW_TRIG_PIN 34
#define FRONT_LOW_ECHO_PIN 35
#define FRONT_SIDE_TRIG_PIN 38
#define FRONT_SIDE_ECHO_PIN 39
#define BACK_SIDE_TRIG_PIN 42
#define BACK_SIDE_ECHO_PIN 43

// Ultrasonic constants
#define MAX_DISTANCE 200
#define DIST_TOLERANCE 5
#define INCREMENT 20
#define BASE_DIST_SIDE  8
#define BASE_DIST_FRONT_L 15
#define BASE_DIST_FRONT_H 18


#define MINE_PIN 2  // Papilio control signals
#define PAP_RST 12
#define PAP_RESUME 9

#define IR_PIN 11           // IR PIN
#define IR_RESUME 0xFF38C7  // decoded IR values
#define IR_RESET 0xFFB04F

enum {MOTORB, MOTORA};

uint8_t speedA, speedB = 0;
uint8_t directionA, directionB = HIGH;

boolean wait_for_resume = true; // waiting status bit

uint8_t front_high_dist;  // ultrasonic distances
uint8_t front_low_dist;
uint8_t front_side_dist;
uint8_t back_side_dist;

// ultrasonic sensors
NewPing front_high_sonar(FRONT_HIGH_TRIG_PIN, FRONT_HIGH_ECHO_PIN, MAX_DISTANCE);
NewPing front_low_sonar(FRONT_LOW_TRIG_PIN, FRONT_LOW_ECHO_PIN, MAX_DISTANCE);
NewPing front_side_sonar(FRONT_SIDE_TRIG_PIN, FRONT_SIDE_ECHO_PIN, MAX_DISTANCE);
NewPing back_side_sonar(BACK_SIDE_TRIG_PIN, BACK_SIDE_ECHO_PIN, MAX_DISTANCE);

// IR receiver
IRrecv IR(IR_PIN);
decode_results IR_results;


int i = 0;

void setup() {
  // Motor pin setup
  pinMode(PWM_PINA, OUTPUT);
  pinMode(PWM_PINB, OUTPUT);
  pinMode(DIR_A, OUTPUT);
  pinMode(DIR_B, OUTPUT);

  // Ultrasonic pin setup
  pinMode(FRONT_HIGH_TRIG_PIN, OUTPUT);
  pinMode(FRONT_HIGH_ECHO_PIN, INPUT);
  pinMode(FRONT_LOW_TRIG_PIN, OUTPUT);
  pinMode(FRONT_LOW_ECHO_PIN, INPUT);
  pinMode(FRONT_SIDE_TRIG_PIN, OUTPUT);
  pinMode(FRONT_SIDE_ECHO_PIN, INPUT);
  pinMode(BACK_SIDE_TRIG_PIN, OUTPUT);
  pinMode(BACK_SIDE_ECHO_PIN, INPUT);

  // Papilio control pin setup
  pinMode(MINE_PIN, INPUT);
  pinMode(PAP_RST, OUTPUT);
  pinMode(PAP_RESUME, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(MINE_PIN), mineDet, RISING); // mine detection interrupt

  // IR setup
  pinMode(IR_PIN, INPUT);
  IR.enableIRIn();

  Serial.begin(9600);
  i=0;
}

void(*resetArduino)(void)=0; // function to reset arduino to addr 0

void loop() {
  Serial.print(wait_for_resume);
  Serial.print("\t");
   if (IR.decode(&IR_results)) {    // Check for IR data
    if(IR_results.value == IR_RESET) {
      reset();                // Reset system from remote
    }
    IR.resume(); // Receive the next value
  }
  if(wait_for_resume) {   // Wait for mine to be removed
    forward(0);           // Stop rover
    if(IR_results.value == IR_RESUME) {
      wait_for_resume = false;
      digitalWrite(PAP_RESUME, HIGH); // Send resume signal to papilio
      delay(20);
      digitalWrite(PAP_RESUME, LOW);
    }
  } else {
//    pingAll();  // Ping all ultrasonic sensors

    Serial.println(i);
    i++;
    forward(100);
    delay(1000);
    turnLeft(100);
    delay(1000);
    turnRight(1000);
    delay(1000);
  }
}

void setmotor(int speed, int direction, int motor) {  // Set a motor's speed and direction
  speed = direction?255-speed:speed;
  int pwm_pin = motor?PWM_PINA:PWM_PINB;
  int dir_pin = motor?DIR_A:DIR_B;
  motor?speedA=speed:speedB=speed; 
  motor?directionA=direction:directionB=direction; 
  analogWrite(pwm_pin, speed);
  digitalWrite(dir_pin, direction);
}

void turnRight(int speed) {   // Start right turn at given speed
  setmotor(speed, 1, MOTORB);
  setmotor(speed, 0, MOTORA);
}

void turnLeft(int speed) {  // Start left turn at given speed
  setmotor(speed, 0, MOTORB);
  setmotor(speed, 1, MOTORA);
}

void forward(int speed) {   // Move forward at given speed
  setmotor(speed, 1, MOTORA);
  setmotor(speed, 1, MOTORB);
}

void reverse(int speed) {   // Move backward at given speed
  setmotor(speed, 0, MOTORA);
  setmotor(speed, 0, MOTORB);
}

void mineDet() {    // Mine detection interrupt
  //forward(0);       // Stop rover
 // wait_for_resume = true; // Set waiting status bit
}

void reset() {  // Reset entire system
  digitalWrite(PAP_RST, HIGH);  // Send reset signal to papilio
  delay(20);
  digitalWrite(PAP_RST, LOW);
  delay(20);
  resetArduino();               // Reset Arduino to address 0
}

void pingAll() {  // Ping each ultrasonic for distance in cm
   front_high_dist = front_high_sonar.ping_cm();
   front_low_dist = front_low_sonar.ping_cm();
   front_side_dist = front_side_sonar.ping_cm();
   back_side_dist = back_side_sonar.ping_cm();
}

