/**
 * @file sketch_pi_motor_bridge.ino
 * @brief Phase 1 — Raspberry Pi → Arduino Motor Command Bridge
 *
 * Purpose:
 *   Receives ASCII motor commands from the Raspberry Pi over USB serial
 *   and drives the AlphaBot2-Ar motors (TB6612FNG via proper dual-direction pins).
 *
 * Command Protocol (Pi → Arduino, 115200 baud, newline terminated):
 *   M,<leftSpeed>,<leftDir>,<rightSpeed>,<rightDir>
 *
 *   leftSpeed / rightSpeed : 0–255
 *   leftDir  / rightDir   : F (forward) | B (backward)
 *
 * Hardware (Verified via AlphaBot2-Ar jumper matrix):
 *   Motor A (Left)  — PWM: D6, AIN1: A1, AIN2: A0
 *   Motor B (Right) — PWM: D5, BIN1: A2, BIN2: A3
 */

#include <Arduino.h>

// ============================================================================
// Pin Definitions (Verified from hardware photos)
// ============================================================================
#define PWMA 6
#define AIN1 A1
#define AIN2 A0

#define PWMB 5
#define BIN1 A2
#define BIN2 A3

// ============================================================================
// Safety Config
// ============================================================================
#define MAX_SPEED       200   // Hard cap — never go above this (0–255)
#define WATCHDOG_MS    2000   // Stop motors if no command received in 2 seconds
#define SERIAL_BAUD   115200

// ============================================================================
// State
// ============================================================================
unsigned long lastCmdTime = 0;
bool motorsRunning = false;
String inputBuffer = "";

// ============================================================================
// Motor Helpers
// ============================================================================

void setMotor(int pwmPin, int in1Pin, int in2Pin, int speed, char dir) {
  speed = constrain(abs(speed), 0, MAX_SPEED);
  
  if (dir == 'F') {
    digitalWrite(in1Pin, HIGH);
    digitalWrite(in2Pin, LOW);
  } else {
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, HIGH);
  }
  
  analogWrite(pwmPin, speed);
}

void stopMotors() {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  analogWrite(PWMA, 0);

  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
  analogWrite(PWMB, 0);
  
  motorsRunning = false;
}

// ============================================================================
// Command Parser
// ============================================================================

void parseCommand(const String& cmd) {
  // Expected: M,<leftSpd>,<leftDir>,<rightSpd>,<rightDir>
  if (cmd.length() < 5 || cmd.charAt(0) != 'M') {
    Serial.println("ERR:unknown_command");
    return;
  }

  // Split by comma
  int idx[5];
  idx[0] = cmd.indexOf(',', 0);
  idx[1] = cmd.indexOf(',', idx[0] + 1);
  idx[2] = cmd.indexOf(',', idx[1] + 1);
  idx[3] = cmd.indexOf(',', idx[2] + 1);

  if (idx[0] < 0 || idx[1] < 0 || idx[2] < 0 || idx[3] < 0) {
    Serial.println("ERR:malformed_missing_fields");
    return;
  }

  int leftSpeed   = cmd.substring(idx[0] + 1, idx[1]).toInt();
  char leftDir    = cmd.charAt(idx[1] + 1);
  int rightSpeed  = cmd.substring(idx[2] + 1, idx[3]).toInt();
  char rightDir   = cmd.charAt(idx[3] + 1);

  // Validate direction chars
  if ((leftDir != 'F' && leftDir != 'B') || (rightDir != 'F' && rightDir != 'B')) {
    Serial.println("ERR:invalid_direction_char");
    return;
  }

  // Apply (Assuming A is left and B is right)
  setMotor(PWMA, AIN1, AIN2, leftSpeed,  leftDir);
  setMotor(PWMB, BIN1, BIN2, rightSpeed, rightDir);

  motorsRunning = (leftSpeed > 0 || rightSpeed > 0);
  lastCmdTime   = millis();

  Serial.println("OK");
}

// ============================================================================
// Setup
// ============================================================================

void setup() {
  Serial.begin(SERIAL_BAUD);

  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  stopMotors();
  lastCmdTime = millis();

  Serial.println("READY:swarm_motor_bridge_v2");
}

// ============================================================================
// Loop
// ============================================================================

void loop() {
  // -- Read serial input (non-blocking, newline terminated) --
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      inputBuffer.trim();
      if (inputBuffer.length() > 0) {
        parseCommand(inputBuffer);
      }
      inputBuffer = "";
    } else if (c != '\r') {
      inputBuffer += c;
    }
  }

  // -- Watchdog: stop if no command received within WATCHDOG_MS --
  if (motorsRunning && (millis() - lastCmdTime > WATCHDOG_MS)) {
    stopMotors();
    Serial.println("WATCHDOG:motors_stopped");
  }
}
