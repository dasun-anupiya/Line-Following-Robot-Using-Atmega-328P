#include <Arduino.h>

// Pin definitions
#define BUTTON_LINE_FOLLOW   7
#define BUTTON_OBSTACLE      4

#define IR_LEFT              A2
#define IR_MIDDLE            A1
#define IR_RIGHT             A0

#define ULTRA_TRIG           3
#define ULTRA_ECHO           2 

#define MOTOR_ENA            9  
#define MOTOR_ENB            6 
        
#define MOTOR_IN1            8
#define MOTOR_IN2            10
#define MOTOR_IN3            5
#define MOTOR_IN4            11

#define HEAD_LIGHT           12
#define BRAKE_LIGHT          13
#define R_IND                A3
#define G_IND                A4 
#define B_IND                A5 

// Constants
#define IR_THRESHOLD         500  // Adjust based on sensor calibration (higher value for black line)
#define NORMAL_SPEED         75  // Reduced from 200
#define SLOW_SPEED           50   // Reduced from 100
#define TURN_SPEED           30  // Reduced from 150
#define SHARP_TURN_SPEED     75  // Reduced from 200
#define TURN_90_DELAY        500  // ms for 90 degree turn, adjust as needed
#define REVERSE_DELAY        500  // ms for reverse
#define OBSTACLE_CLOSE       10   // cm
#define OBSTACLE_WARN        20   // cm
#define BUTTON_PRESSED       LOW  // Assuming buttons are active low (pulled high, ground on press)
#define DEBOUNCE_DELAY       50   // ms for debounce
#define BLINK_DELAY          200  // ms for blink

// Global variables
int robot_mode = 0;  // 0: off, 1: line follow, 2: obstacle avoid
int current_speed = 0;

// Function prototypes
void setMotorLeft(int speed, bool forward);
void setMotorRight(int speed, bool forward);
void forward(int speed);
void reverse(int speed);
void stopMotors();
void turnLeftSmooth();
void turnRightSmooth();
void turnLeft90();
void turnRight90();
void randomTurn();
void flickerBrake(int times);
void blinkIndicator(int pin, int times);
long getDistance();
bool isBlack(int pin);

void setup() {
  // Buttons
  pinMode(BUTTON_LINE_FOLLOW, INPUT_PULLUP);
  pinMode(BUTTON_OBSTACLE, INPUT_PULLUP);

  // IR sensors (analog, but used with threshold)
  pinMode(IR_LEFT, INPUT);
  pinMode(IR_MIDDLE, INPUT);
  pinMode(IR_RIGHT, INPUT);

  // Ultrasonic
  pinMode(ULTRA_TRIG, OUTPUT);
  pinMode(ULTRA_ECHO, INPUT);

  // Motors
  pinMode(MOTOR_ENA, OUTPUT);
  pinMode(MOTOR_ENB, OUTPUT);
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  pinMode(MOTOR_IN3, OUTPUT);
  pinMode(MOTOR_IN4, OUTPUT);

  // LEDs
  pinMode(HEAD_LIGHT, OUTPUT);
  pinMode(BRAKE_LIGHT, OUTPUT);
  pinMode(R_IND, OUTPUT);
  pinMode(G_IND, OUTPUT);
  pinMode(B_IND, OUTPUT);

  // Initial states
  digitalWrite(HEAD_LIGHT, LOW);
  digitalWrite(BRAKE_LIGHT, LOW);
  digitalWrite(R_IND, LOW);
  digitalWrite(G_IND, LOW);
  digitalWrite(B_IND, LOW);

  stopMotors();

  randomSeed(analogRead(A0));  // Seed random with noise
}

void loop() {
  // Button debouncing and mode switching
  static unsigned long lastDebounceTime1 = 0;
  static unsigned long lastDebounceTime2 = 0;
  static int lastButtonState1 = HIGH;
  static int lastButtonState2 = HIGH;

  int button1 = digitalRead(BUTTON_LINE_FOLLOW);
  int button2 = digitalRead(BUTTON_OBSTACLE);

  // Button 1 (Line Follow)
  if (button1 != lastButtonState1) {
    lastDebounceTime1 = millis();
  }
  if ((millis() - lastDebounceTime1) > DEBOUNCE_DELAY) {
    if (button1 == BUTTON_PRESSED) {
      robot_mode = 1;
      digitalWrite(G_IND, HIGH);
      digitalWrite(B_IND, LOW);
    }
  }
  lastButtonState1 = button1;

  // Button 2 (Obstacle)
  if (button2 != lastButtonState2) {
    lastDebounceTime2 = millis();
  }
  if ((millis() - lastDebounceTime2) > DEBOUNCE_DELAY) {
    if (button2 == BUTTON_PRESSED) {
      robot_mode = 2;
      digitalWrite(B_IND, HIGH);
      digitalWrite(G_IND, LOW);
    }
  }
  lastButtonState2 = button2;

  // If both buttons pressed simultaneously, turn off (optional safety)
  if (button1 == BUTTON_PRESSED && button2 == BUTTON_PRESSED) {
    robot_mode = 0;
    digitalWrite(G_IND, LOW);
    digitalWrite(B_IND, LOW);
    stopMotors();
  }

  // Mode execution
  if (robot_mode == 1) {
    // Line following mode
    bool left_black = isBlack(IR_LEFT);
    bool middle_black = isBlack(IR_MIDDLE);
    bool right_black = isBlack(IR_RIGHT);

    long distance = getDistance();

    int ir_state = 0;
    if (left_black) ir_state = 4;
    if (middle_black) ir_state = 2;
    if (right_black) ir_state = 1;
    if (left_black && middle_black) ir_state = 6;
    if (middle_black && right_black) ir_state = 3;
    if (left_black && middle_black && right_black) ir_state = 7;
    if (left_black && right_black) ir_state = 5;

    bool going_forward = false;

    switch (ir_state) {
      case 2:  // State 1: forward
        forward(NORMAL_SPEED);
        going_forward = true;
        break;
      case 4:  // State 2: left turn
        turnLeftSmooth();
        break;
      case 1:  // State 3: right turn
        turnRightSmooth();
        break;
      case 6:  // State 4: 90 left
        turnLeft90();
        break;
      case 3:  // State 5: 90 right
        turnRight90();
        break;
      case 7:  // State 6: T junction, random 90
        randomTurn();
        break;
      case 5:  // State 7: Y junction, random turn
        randomTurn();
        break;
      default:  // No line or other, stop
        stopMotors();
        break;
    }

    // Obstacle check only when going forward
    if (going_forward && distance < OBSTACLE_WARN) {
      flickerBrake(3);
      digitalWrite(BRAKE_LIGHT, HIGH);
      // Slow down smoothly to stop, approximating keeping 10cm
      for (int s = NORMAL_SPEED; s >= 0; s -= 10) {
        forward(s);
        delay(50);
      }
      stopMotors();
    }
  } else if (robot_mode == 2) {
    // Obstacle avoidance mode
    long distance = getDistance();

    if (distance < OBSTACLE_CLOSE) {
      // <10cm: blink R_IND 3 times, reverse, random turn
      blinkIndicator(R_IND, 3);
      flickerBrake(3);
      digitalWrite(BRAKE_LIGHT, HIGH);
      reverse(SLOW_SPEED);
      delay(REVERSE_DELAY);
      stopMotors();
      randomTurn();
    } else if (distance < OBSTACLE_WARN) {
      // <20cm: blink R_IND 3 times, slow down (simulate keeping 10cm by slowing to stop), random turn
      blinkIndicator(R_IND, 3);
      flickerBrake(3);
      digitalWrite(BRAKE_LIGHT, HIGH);
      // Slow down smoothly
      for (int s = current_speed; s >= 0; s -= 10) {
        forward(s);
        delay(50);
      }
      stopMotors();
      randomTurn();
    } else {
      // >20cm: forward normal
      forward(NORMAL_SPEED);
      digitalWrite(BRAKE_LIGHT, LOW);
    }
  } else {
    // Mode 0: off
    stopMotors();
  }

  delay(10);  // Small delay for stability
}

// Helper functions

bool isBlack(int pin) {
  int IR_VALUE = digitalRead(pin);
  return IR_VALUE;
}

long getDistance() {
  digitalWrite(ULTRA_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRA_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRA_TRIG, LOW);
  long duration = pulseIn(ULTRA_ECHO, HIGH);
  return duration * 0.034 / 2;  // cm
}

void setMotorLeft(int speed, bool forward) {
  digitalWrite(MOTOR_IN1, forward ? HIGH : LOW);
  digitalWrite(MOTOR_IN2, forward ? LOW : HIGH);
  analogWrite(MOTOR_ENA, speed);
}

void setMotorRight(int speed, bool forward) {
  digitalWrite(MOTOR_IN3, forward ? HIGH : LOW);
  digitalWrite(MOTOR_IN4, forward ? LOW : HIGH);
  analogWrite(MOTOR_ENB, speed);
}

void forward(int speed) {
  setMotorLeft(speed, true);
  setMotorRight(speed, true);
  digitalWrite(HEAD_LIGHT, HIGH);
  current_speed = speed;
}

void reverse(int speed) {
  setMotorLeft(speed, false);
  setMotorRight(speed, false);
  digitalWrite(HEAD_LIGHT, LOW);  // Assuming no headlight in reverse
  current_speed = speed;
}

void stopMotors() {
  analogWrite(MOTOR_ENA, 0);
  analogWrite(MOTOR_ENB, 0);
  digitalWrite(HEAD_LIGHT, LOW);
  digitalWrite(BRAKE_LIGHT, HIGH);  // Keep brake on when stopped
  current_speed = 0;
}

void turnLeftSmooth() {
  setMotorLeft(TURN_SPEED / 2, true);  // Left slower
  setMotorRight(TURN_SPEED, true);
  digitalWrite(HEAD_LIGHT, HIGH);
}

void turnRightSmooth() {
  setMotorLeft(TURN_SPEED, true);
  setMotorRight(TURN_SPEED / 2, true);  // Right slower
  digitalWrite(HEAD_LIGHT, HIGH);
}

void turnLeft90() {
  setMotorLeft(SHARP_TURN_SPEED, false);  // Left reverse
  setMotorRight(SHARP_TURN_SPEED, true);  // Right forward
  delay(TURN_90_DELAY);
  stopMotors();
}

void turnRight90() {
  setMotorLeft(SHARP_TURN_SPEED, true);   // Left forward
  setMotorRight(SHARP_TURN_SPEED, false); // Right reverse
  delay(TURN_90_DELAY);
  stopMotors();
}

void randomTurn() {
  if (random(0, 2) == 0) {
    turnLeft90();
  } else {
    turnRight90();
  }
}

void flickerBrake(int times) {
  for (int i = 0; i < times; i++) {
    digitalWrite(BRAKE_LIGHT, HIGH);
    delay(BLINK_DELAY);
    digitalWrite(BRAKE_LIGHT, LOW);
    delay(BLINK_DELAY);
  }
}

void blinkIndicator(int pin, int times) {
  for (int i = 0; i < times; i++) {
    digitalWrite(pin, HIGH);
    delay(BLINK_DELAY);
    digitalWrite(pin, LOW);
    delay(BLINK_DELAY);
  }
}