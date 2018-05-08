#include <NewPing.h>

#include <IRremote.h>
#include <ir_Lego_PF_BitStreamEncoder.h>
#include <IRremoteInt.h>
#include <boarddefs.h>

#define PWM_PINA 6  // Motor control pins
#define PWM_PINB 5
#define DIR_A 7
#define DIR_B 4

//pathing constants
#define ORTHOG_TIME 900      // SET TO 1000 for full battery
#define BACK_STEP_TIME 600
#define OBSTACLE_LENGTH 15
#define OVERSHOOT_TIME 70    // CHANGE FOR BATTERY
#define BRAKE_TIME 50
#define CLEARANCE_TIME 100
#define DEBUG 0

#define FRONT_HIGH_TRIG_PIN 30  // Ultrasonic pins
#define FRONT_HIGH_ECHO_PIN 31
#define FRONT_LOW_TRIG_PIN 34
#define FRONT_LOW_ECHO_PIN 35
#define FRONT_SIDE_TRIG_PIN 38
#define FRONT_SIDE_ECHO_PIN 39
#define BACK_SIDE_TRIG_PIN 42
#define BACK_SIDE_ECHO_PIN 43
#define FRONT_LEFT_TRIG_PIN 46
#define FRONT_LEFT_ECHO_PIN 47

// Ultrasonic constants
#define MAX_DISTANCE 200
#define DIST_TOLERANCE 1
#define BASE_DIST_SIDE  3
#define BASE_DIST_FRONT_L 17
#define BASE_DIST_FRONT_H 18
#define BASE_DIST_LEFT 15
#define ALIGN_KC 75
#define ALIGN_KP ((255 - ALIGN_KC) / OBSTACLE_LENGTH)


#define MINE_PIN 2  // Papilio control signals
#define PAP_RST 12
#define PAP_RESUME 9
#define debouncing_time 1

#define IR_PIN 11           // IR PIN
#define IR_RESUME 0xFF38C7  // decoded IR values
#define IR_RESET 0xFFB04F
#define IR_DIR_UP 0xFF18E7
#define IR_DIR_DOWN 0xFF4AB5
#define IR_DIR_RIGHT 0xFF5AA5
#define IR_DIR_LEFT 0xFF10EF
#define IR_STOP 0xFF6897
enum {MOTORA, MOTORB};

enum {RIGHT, UP, LEFT, DOWN};

uint8_t speedA, speedB = 0;
uint8_t directionA, directionB = HIGH;

boolean wait_for_resume = true; // waiting status bit
boolean first_turn;             // First right status bit
boolean preObstacle;

#define SAMPS 4
#define LOGSAMPS 2

int16_t front_high_dist;  // ultrasonic distances
int16_t front_low_dist;
int16_t front_side_dist;
int16_t back_side_dist;
int16_t front_left_dist;

/* rolling averages */
uint8_t front_high_avg[SAMPS];
uint8_t front_low_avg[SAMPS];
uint8_t front_side_avg[SAMPS];
uint8_t back_side_avg[SAMPS];
uint8_t front_left_avg[SAMPS];

uint8_t qi;

/* pathing state */
uint8_t dir = 1;
boolean obstacle = true;
uint8_t min_dist_front_l = BASE_DIST_FRONT_L;
uint8_t min_dist_front_h = BASE_DIST_FRONT_H;
uint8_t target_dist_side = BASE_DIST_SIDE;
volatile unsigned long interruptTime;

// ultrasonic sensors
NewPing front_high_sonar(FRONT_HIGH_TRIG_PIN, FRONT_HIGH_ECHO_PIN, MAX_DISTANCE);
NewPing front_low_sonar(FRONT_LOW_TRIG_PIN, FRONT_LOW_ECHO_PIN, MAX_DISTANCE);
NewPing front_side_sonar(FRONT_SIDE_TRIG_PIN, FRONT_SIDE_ECHO_PIN, MAX_DISTANCE);
NewPing back_side_sonar(BACK_SIDE_TRIG_PIN, BACK_SIDE_ECHO_PIN, MAX_DISTANCE);
NewPing front_left_sonar(FRONT_LEFT_TRIG_PIN, FRONT_LEFT_ECHO_PIN, MAX_DISTANCE);
NewPing sonar = back_side_sonar;

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
  pinMode(FRONT_LEFT_TRIG_PIN, OUTPUT);
  pinMode(FRONT_LEFT_ECHO_PIN, INPUT);

  // Papilio control pin setup
  pinMode(MINE_PIN, INPUT);
  pinMode(PAP_RST, OUTPUT);
  pinMode(PAP_RESUME, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(MINE_PIN), debounceInterrupt, RISING); // mine detection interrupt

  // IR setup
  pinMode(IR_PIN, INPUT);
  IR.enableIRIn();

  Serial.begin(9600);

  //i=0;
  first_turn = true;
  obstacle = false;

  digitalWrite(13, HIGH);
  delay(200);
  digitalWrite(13, LOW);
}

void(*resetArduino)(void) = 0; // function to reset arduino to addr 0

void loop() {
  if (IR.decode(&IR_results)) {    // Check for IR data
    delay(100);
    Serial.println(IR_results.value, HEX);
    if (IR_results.value == IR_RESET) {
      Serial.println("Reseting");
      reset();                // Reset system from remote
    }
    IR.resume(); // Receive the next value
  } else IR_results.value = 0;
  //Serial.println(wait_for_resume);

  if (interruptTime != 0 && (long)micros() - interruptTime > 1000 * debouncing_time) {
    if (digitalRead(MINE_PIN) == HIGH) mineDet();
  }
  if (wait_for_resume) {  // Wait for mine to be removed
    forward(0);           // Stop rover
    //Serial.println("Waiting");
    if (IR_results.value == IR_RESUME) {
      wait_for_resume = false;
      digitalWrite(PAP_RESUME, HIGH); // Send resume signal to papilio
      delay(20);
      digitalWrite(PAP_RESUME, LOW);
    }
  } else {
    if (first_turn) {
      orthogRight();
      alignSensors();
      first_turn = false;
    } else {
      if (abs(front_side_dist - back_side_dist) > 2) alignSensors();
      forward(200);
      pingAll();  // Ping all ultrasonic sensors
      //      Serial.print(front_left_dist);
      //      Serial.print(" ");
      //      Serial.println(front_low_dist);

      //Serial.println(obstacle);

      //      if (DEBUG) { // if in debug mode, just do what the remote tells you.
      //        switch (IR_results.value) {
      //          case IR_DIR_UP:
      //            forward(150);
      //            break;
      //          case IR_DIR_DOWN:
      //            reverse(150);
      //            break;
      //          case IR_DIR_LEFT:
      //            turnLeft(100);
      //            break;
      //          case IR_DIR_RIGHT:
      //            turnRight(100);
      //            break;
      //          case IR_STOP:
      //            forward(0);
      //            break;
      //          default:
      //            break;
      //        }
      //      }

      //      else {
      pingAll();
      //forward(200);
      //wait_for_resume = 1;

      if (obstacle) {
        if (preObstacle) {
          if (back_side_dist < OBSTACLE_LENGTH) preObstacle = false;
        } else {
          if (back_side_dist >= OBSTACLE_LENGTH) {
            delay(OVERSHOOT_TIME);
            orthogRight();
            preObstacle = true;
          }
        }
        if ((front_high_dist < min_dist_front_h || front_low_dist < min_dist_front_l)) {
          if (front_high_dist - front_low_dist > OBSTACLE_LENGTH) {
            orthogLeft();
          } else  {
            obstacle = false;
            if (dir == 3) {
              reverse(200);
              delay(BACK_STEP_TIME);
              forward(0);
              delay(BRAKE_TIME);
              min_dist_front_l = front_low_sonar.ping_cm();
              min_dist_front_h = front_low_sonar.ping_cm();
              orthogLeft(); dir = 0;
              alignSensors();
              target_dist_side = (front_side_sonar.ping_median(SAMPS) + back_side_sonar.ping_median(SAMPS)) >> 1;
            }
            else {
              orthogLeft();
              alignSensors();
            }
          }
        }
      } else {
        if ((front_high_dist < min_dist_front_h || front_low_dist < min_dist_front_l)) {
          Serial.println("pathfinding");
          forward(0);
          delay(BRAKE_TIME);
          if (front_high_dist - front_low_dist > OBSTACLE_LENGTH) {
            orthogLeft();
            alignSensors();
            handleObstacle();
          }
          else {
            if (dir == 3) {
              reverse(200);
              delay(BACK_STEP_TIME);
              forward(0);
              delay(BRAKE_TIME);
              min_dist_front_l = front_low_sonar.ping_cm();
              min_dist_front_h = front_high_sonar.ping_cm();
              Serial.println(min_dist_front_l);
              Serial.print(" ");
              Serial.println(min_dist_front_h);
              Serial.print(" ");
              orthogLeft(); dir = 0;
              alignSensors();
              target_dist_side = (front_side_sonar.ping_median(SAMPS) + back_side_sonar.ping_median(SAMPS)) >> 1;
            }
            else {
              orthogLeft();
              alignSensors();
            }
          }
        } //else if(front_left_dist < BASE_DIST_LEFT) orthogLeft();
      }

      //  reverse(200);
      //  delay(BACK_STEP_TIME);
      //  orthogLeft();
      //  alignSensors();
      //  wait_for_resume = true;

      //      if ((front_high_dist <= BASE_DIST_FRONT_H || front_low_dist <= BASE_DIST_FRONT_L)) {
      //
      //        orthogLeft();
      //        alignSensors();
      //        wait_for_resume = true;
      //      }
      //        Serial.println(front_high_dist);
      //        Serial.print(" ");
      //        Serial.println(front_low_dist);
    }

    //Serial.println(i);
    //i++;
    //}
  }
}

void setmotor(uint8_t speed, int direction, uint8_t motor) {  // Set a motor's speed and direction
  speed = direction ? 255 - speed : speed;
  int pwm_pin, dir_pin;

  if (motor == MOTORA) {
    pwm_pin = PWM_PINA;
    dir_pin = DIR_A;
    speedA = speed;
    directionA = direction;
  }
  else {
    pwm_pin = PWM_PINB;
    dir_pin = DIR_B;
    speedB = speed;
    directionB = direction;
  }

  analogWrite(pwm_pin, speed);
  digitalWrite(dir_pin, direction);
}

void turnRight(uint8_t speed) {   // Start right turn at given speed
  setmotor(speed, 0, MOTORB);
  setmotor(speed, 1, MOTORA);
}

void turnLeft(uint8_t speed) {  // Start left turn at given speed
  setmotor(speed, 1, MOTORB);
  setmotor(speed, 0, MOTORA);
}

void forward(uint8_t speed) {   // Move forward at given speed
  setmotor(speed, 1, MOTORA);
  setmotor(speed, 1, MOTORB);
}

void reverse(uint8_t speed) {   // Move backward at given speed
  setmotor(speed, 0, MOTORA);
  setmotor(speed, 0, MOTORB);
}

void debounceInterrupt() {
  interruptTime = micros();
}

void mineDet() {    // Mine detection interrupt
  Serial.println("Wait state");
  forward(0);       // Stop rover
  wait_for_resume = true; // Set waiting status bit
}

void reset() {  // Reset entire system
  digitalWrite(PAP_RST, HIGH);  // Send reset signal to papilio
  delay(20);
  digitalWrite(PAP_RST, LOW);
  delay(20);
  resetArduino();               // Reset Arduino to address 0
}

void orthogLeft() {
  reverse(200);
  delay(CLEARANCE_TIME);
  turnLeft(255);
  delay(ORTHOG_TIME);
  forward(0);
  delay(BRAKE_TIME);
  dir++;
}

void orthogRight() {
  turnRight(255);
  delay(ORTHOG_TIME);
  forward(0);
  delay(BRAKE_TIME);
  dir--;
}

void handleObstacle() { // get around obstacle
  obstacle = true;
  preObstacle = false;
  //  orthogLeft();
  //  alignSensors();
  //  pingAll();
  //  forward(200);
  //  while (back_side_dist < OBSTACLE_LENGTH) {
  //    pingAll();
  //    alignSensors();
  //    forward(200);
  //  }
  //  delay(OVERSHOOT_TIME);
  //  orthogRight();
  //  alignSensors();
  //  forward(200);
  //  while (front_low_dist > min_dist_front_l && front_high_dist > min_dist_front_h && (back_side_dist >= OBSTACLE_LENGTH)) pingAll();
  //  while (front_low_dist > min_dist_front_l && front_high_dist > min_dist_front_h && (back_side_dist < OBSTACLE_LENGTH)) pingAll();
  //  if(back_side_dist >= OBSTACLE_LENGTH) {
  //    delay(OVERSHOOT_TIME);
  //    orthogRight();
  //  }
  //  else{
  //    forward(0);
  //    delay(BRAKE_TIME);
  //    orthogLeft();
  //  }
}

void alignSensors() {
  digitalWrite(13, HIGH);
//  if (wait_for_resume) {
//    forward(0);
//  } else {
    if (abs(front_side_dist - back_side_dist) < OBSTACLE_LENGTH && !obstacle) {
      while (abs(front_side_dist - back_side_dist)) {
        unsigned long speed = ALIGN_KC + abs(front_side_dist - back_side_dist) * ALIGN_KP;
        if (speed > 255) speed = 255;
        if (front_side_dist - back_side_dist > 2) turnRight(speed);
        else if (front_side_dist - back_side_dist < -2) turnLeft(speed);
        pingAll();
      }
      forward(0);
      delay(BRAKE_TIME);
    }
  //k]}
  digitalWrite(13, LOW);
}

void pingAll() {  // Ping each ultrasonic for distance in cm
  front_high_avg[qi] = front_high_sonar.ping_cm();
  front_low_avg[qi] = front_low_sonar.ping_cm();
  front_side_avg[qi] = front_side_sonar.ping_cm();
  back_side_avg[qi] = back_side_sonar.ping_cm();
  //front_left_avg[qi] = front_left_sonar.ping_cm();

  front_high_dist = 0;
  front_low_dist = 0;
  front_side_dist = 0;
  back_side_dist = 0;
  //front_left_dist = 0;
  for (int i = 0; i < SAMPS; i++) {
    front_high_dist += front_high_avg[i];
    front_low_dist += front_low_avg[i];
    front_side_dist += front_side_avg[i];
    back_side_dist += back_side_avg[i];
    //front_left_dist += front_left_avg[i];
  }
  front_high_dist = front_high_dist >> LOGSAMPS;
  front_low_dist = front_low_dist >> LOGSAMPS;
  front_side_dist = front_side_dist >> LOGSAMPS;
  back_side_dist = back_side_dist >> LOGSAMPS;
  //front_left_dist = front_left_dist >> LOGSAMPS;

  if (front_high_dist == 0) front_high_dist = MAX_DISTANCE;
  if (front_low_dist == 0) front_low_dist = MAX_DISTANCE;
  if (front_side_dist == 0) front_side_dist = MAX_DISTANCE;
  if (back_side_dist == 0) back_side_dist = MAX_DISTANCE;
  //if (front_left_dist == 0) front_left_dist = MAX_DISTANCE;

  qi++;
  if (qi == SAMPS) qi = 0;
  delay(15);

  //  Serial.print(front_high_dist);
  //  Serial.print(", ");
  //  Serial.print(front_low_dist);
  //  Serial.print(", ");
  //  Serial.println(abs(front_high_dist - front_low_dist));
  Serial.print(front_side_dist);
  Serial.print(", ");
  Serial.print(back_side_dist);
  Serial.print(", ");
  Serial.println(abs(front_side_dist - back_side_dist));
}

