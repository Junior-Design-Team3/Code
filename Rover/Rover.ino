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
#define ORTHOG_TIME 2500
#define BACK_STEP_TIME 1500
#define OBSTACLE_LENGTH 10
#define OVERSHOOT_TIME 100
#define BRAKE_TIME 50
#define DEBUG 0

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
#define BASE_DIST_SIDE  3
#define BASE_DIST_FRONT_L 12
#define BASE_DIST_FRONT_H 14
#define ALIGN_KP 1


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

#define SAMPS 1
#define LOGSAMPS 0

int16_t front_high_dist;  // ultrasonic distances
int16_t front_low_dist;
int16_t front_side_dist;
int16_t back_side_dist;

/* rolling averages */
uint8_t front_high_avg[SAMPS];
uint8_t front_low_avg[SAMPS];
uint8_t front_side_avg[SAMPS];
uint8_t back_side_avg[SAMPS];

uint8_t qi;

/* pathing state */
uint8_t dir = 1;
uint8_t obstacle = 0;
uint16_t min_dist_front_l = BASE_DIST_FRONT_L;
uint16_t min_dist_front_h = BASE_DIST_FRONT_H;
uint16_t target_dist_side = BASE_DIST_SIDE;
volatile unsigned long interruptTime;

// ultrasonic sensors
NewPing front_high_sonar(FRONT_HIGH_TRIG_PIN, FRONT_HIGH_ECHO_PIN, MAX_DISTANCE);
NewPing front_low_sonar(FRONT_LOW_TRIG_PIN, FRONT_LOW_ECHO_PIN, MAX_DISTANCE);
NewPing front_side_sonar(FRONT_SIDE_TRIG_PIN, FRONT_SIDE_ECHO_PIN, MAX_DISTANCE);
NewPing back_side_sonar(BACK_SIDE_TRIG_PIN, BACK_SIDE_ECHO_PIN, MAX_DISTANCE);
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

  // Papilio control pin setup
  pinMode(MINE_PIN, INPUT);
  pinMode(PAP_RST, OUTPUT);
  pinMode(PAP_RESUME, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(MINE_PIN), debounceInterrupt, RISING); // mine detection interrupt

  // IR setup
  pinMode(IR_PIN, INPUT);
  IR.enableIRIn();

  Serial.begin(9600);

  // Pathing state setup
  orthogRight(); // We're facing center of field, turn to face right.
  
  //i=0;

  digitalWrite(13, HIGH);
  delay(200);
  digitalWrite(13, LOW);
}

void(*resetArduino)(void)=0; // function to reset arduino to addr 0

void loop() {
   Serial.print(front_low_dist);
   Serial.print(", ");
   Serial.println(front_high_dist);
   if (IR.decode(&IR_results)) {    // Check for IR data
    delay(100);
    Serial.println(IR_results.value, HEX);
    if(IR_results.value == IR_RESET) {
      Serial.println("Reseting");
      reset();                // Reset system from remote
    }
    IR.resume(); // Receive the next value
   } else IR_results.value = 0;
  //Serial.println(wait_for_resume);

  if(interruptTime != 0 && (long)micros()-interruptTime > 1000 * debouncing_time) {
    if(digitalRead(MINE_PIN) == HIGH) mineDet();
  }
  if(wait_for_resume) {   // Wait for mine to be removed
    forward(0);           // Stop rover
    //Serial.println("Waiting");
    if(IR_results.value == IR_RESUME) {
      wait_for_resume = false;
      digitalWrite(PAP_RESUME, HIGH); // Send resume signal to papilio
      delay(20);
      digitalWrite(PAP_RESUME, LOW);
    }
  } else {
    forward(255);
    pingAll();  // Ping all ultrasonic sensors
//    Serial.println(front_low_dist);

    if (DEBUG){ // if in debug mode, just do what the remote tells you.
      switch(IR_results.value){
        case IR_DIR_UP:
          forward(150);
          break;
        case IR_DIR_DOWN:
          reverse(150);
          break;
        case IR_DIR_LEFT:
          turnLeft(100);
          break;
        case IR_DIR_RIGHT:
          turnRight(100);
          break;
        case IR_STOP:
          forward(0);
          break;
        default:
          break;
        }
    }

    else{
      pingAll();
      //forward(200);
      //wait_for_resume = 1;
      if (1 && (front_high_dist < BASE_DIST_FRONT_H || front_low_dist < BASE_DIST_FRONT_L)){
        Serial.println("pathfinding");
        forward(0);
        delay(BRAKE_TIME);
        if (front_high_dist - front_low_dist > OBSTACLE_LENGTH) handleObstacle(); // TODO if the high distance is >1ft more than the low...
        else if (dir == 3){
          reverse(150);
          delay(BACK_STEP_TIME);
          min_dist_front_l = front_low_sonar.ping_median(SAMPS);
          min_dist_front_h = front_low_sonar.ping_median(SAMPS);
          orthogLeft(); dir = 0;
          alignSensors(); // TODO
          target_dist_side = (front_side_sonar.ping_median(SAMPS) + back_side_sonar.ping_median(SAMPS)) >> 1;
        }
        else orthogLeft();
        }

      }

    //Serial.println(i);
    //i++;
  }
}

void setmotor(uint8_t speed, int direction, uint8_t motor) {  // Set a motor's speed and direction
  speed = direction?255-speed:speed;
  int pwm_pin, dir_pin;

  if(motor==MOTORA){
      pwm_pin = PWM_PINA;
      dir_pin = DIR_A;
      speedA = speed;
      directionA=direction;
  }
  else{
    pwm_pin = PWM_PINB;
      dir_pin = DIR_B;
      speedB = speed;
      directionB=direction;
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

void orthogLeft(){
  turnLeft(255);
  delay(ORTHOG_TIME);
  forward(0);
  delay(BRAKE_TIME);
  alignSensors();
  dir++;
  }
  
void orthogRight(){
  turnRight(255);
  delay(ORTHOG_TIME);
  forward(0);
  delay(BRAKE_TIME);
  alignSensors();
  dir--;
}

void handleObstacle(){ // get around obstacle
    orthogLeft();
    while(back_side_dist < OBSTACLE_LENGTH) pingAll();
    delay(OVERSHOOT_TIME);
    orthogRight();
    forward(150);
    while(front_low_dist > min_dist_front_l && front_high_dist > min_dist_front_h) pingAll();
    forward(0);
    delay(BRAKE_TIME);
    if (front_high_dist - front_low_dist > OBSTACLE_LENGTH) handleObstacle();
    else orthogLeft();
}

void alignSensors(){
  
    while(abs(front_side_dist - back_side_dist)){
        if (front_side_dist - back_side_dist > 2) turnRight(255);
        else if (front_side_dist - back_side_dist < -2) turnLeft(255);
        pingAll();
    }
    forward(0);
    delay(BRAKE_TIME);
}
  
void pingAll() {  // Ping each ultrasonic for distance in cm
   front_high_avg[qi] = front_high_sonar.ping_cm();
   front_low_avg[qi] = front_low_sonar.ping_cm();
   front_side_avg[qi] = front_side_sonar.ping_cm();
   back_side_avg[qi] = back_side_sonar.ping_cm();

   front_high_dist = 0;
   front_low_dist = 0;
   front_side_dist = 0;
   back_side_dist = 0;
   for (int i = 0; i < SAMPS; i++){
    front_high_dist += front_high_avg[i];
    front_low_dist += front_low_avg[i];
    front_side_dist += front_side_avg[i];
    back_side_dist += back_side_avg[i];
    }
   front_high_dist = front_high_dist >> LOGSAMPS;
   front_low_dist = front_low_dist >> LOGSAMPS;
   front_side_dist = front_side_dist >> LOGSAMPS;
   back_side_dist = back_side_dist >> LOGSAMPS;

   
   qi++;
   if (qi==SAMPS) qi=0;
}

