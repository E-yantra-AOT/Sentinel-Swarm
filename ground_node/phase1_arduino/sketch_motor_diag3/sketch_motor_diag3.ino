#include <Arduino.h>

#define PWMA 6
#define AIN1 A1
#define AIN2 A0

#define PWMB 5
#define BIN1 A2
#define BIN2 A3

void setup() {
  Serial.begin(115200);
  
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  // Initial state: stopped
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  analogWrite(PWMA, 0);
  
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
  analogWrite(PWMB, 0);
}

void loop() {
  Serial.println("MOTOR TEST: FORWARD (2 seconds)");
  
  // Motor A Forward
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  analogWrite(PWMA, 150);

  // Motor B Forward
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  analogWrite(PWMB, 150);

  delay(2000);

  Serial.println("MOTOR TEST: STOP (2 seconds)");
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  analogWrite(PWMA, 0);
  
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
  analogWrite(PWMB, 0);
  
  delay(2000);
}
