/**
 * motor_diag2.ino — Dual-direction-pin TB6612FNG test
 *
 * TB6612FNG truth table:
 *   AIN1=H AIN2=L PWMA>0 → FORWARD
 *   AIN1=L AIN2=H PWMA>0 → BACKWARD
 *   AIN1=H AIN2=H         → BRAKE (no movement!)  ← was our bug
 *   AIN1=L AIN2=L         → COAST
 *
 * Our previous sketch only set one direction pin — if the other floated
 * HIGH, both pins were HIGH = BRAKE. This test drives BOTH pins explicitly.
 *
 * Candidate second-direction pins tried: D2, D3, D8, D9, D11, D12
 */

#define SPEED 200
#define RUN_MS 1000

// Always drive STBY HIGH on all candidates
void enableSTBY() {
  const int pins[] = {2, 3, 8, 9, 11, 12, 13};
  for (int i = 0; i < 7; i++) {
    pinMode(pins[i], OUTPUT);
    digitalWrite(pins[i], HIGH);
  }
}

void allStop() {
  for (int p = 2; p <= 13; p++) {
    analogWrite(p, 0);
    digitalWrite(p, LOW);
  }
}

void tryMotorA(int ain1, int ain2, int pwm, const char* label) {
  allStop();
  enableSTBY();
  Serial.print("  A-FWD ");
  Serial.print(label);
  // Forward: AIN1=H, AIN2=L
  pinMode(ain1, OUTPUT); digitalWrite(ain1, HIGH);
  pinMode(ain2, OUTPUT); digitalWrite(ain2, LOW);
  pinMode(pwm,  OUTPUT); analogWrite(pwm, SPEED);
  delay(RUN_MS);
  allStop();
  delay(300);
}

void tryMotorB(int bin1, int bin2, int pwm, const char* label) {
  allStop();
  enableSTBY();
  Serial.print("  B-FWD ");
  Serial.print(label);
  // Forward: BIN1=H, BIN2=L
  pinMode(bin1, OUTPUT); digitalWrite(bin1, HIGH);
  pinMode(bin2, OUTPUT); digitalWrite(bin2, LOW);
  pinMode(pwm,  OUTPUT); analogWrite(pwm, SPEED);
  delay(RUN_MS);
  allStop();
  delay(300);
}

void setup() {
  Serial.begin(115200);
  allStop();
  delay(500);

  Serial.println("=== Dual-Pin Motor Diagnostic ===");
  Serial.println("Each step drives AIN1=H, AIN2=L (proper TB6612FNG FORWARD)");
  Serial.println("Watch for ANY wheel movement and report step number.\n");

  // ── Motor A candidates (PWM always D5, try different AIN1/AIN2 pairs) ──
  Serial.println("[MOTOR A — PWM=D5]");
  tryMotorA(4, 3, 5, "AIN1=D4 AIN2=D3");   // most likely on AlphaBot2-Ar
  tryMotorA(3, 4, 5, "AIN1=D3 AIN2=D4");   // reversed
  tryMotorA(4, 2, 5, "AIN1=D4 AIN2=D2");
  tryMotorA(4, 8, 5, "AIN1=D4 AIN2=D8");
  tryMotorA(4, 9, 5, "AIN1=D4 AIN2=D9");
  tryMotorA(4, 11,5, "AIN1=D4 AIN2=D11");

  // ── Motor A with PWM on D6 instead ──
  Serial.println("[MOTOR A — PWM=D6]");
  tryMotorA(7, 8, 6, "AIN1=D7 AIN2=D8");   // swapped motor assignment
  tryMotorA(8, 7, 6, "AIN1=D8 AIN2=D7");
  tryMotorA(7, 9, 6, "AIN1=D7 AIN2=D9");

  // ── Motor B candidates (PWM always D6, try different BIN pairs) ──
  Serial.println("[MOTOR B — PWM=D6]");
  tryMotorB(7, 8, 6, "BIN1=D7 BIN2=D8");   // most likely
  tryMotorB(8, 7, 6, "BIN1=D8 BIN2=D7");   // reversed
  tryMotorB(7, 2, 6, "BIN1=D7 BIN2=D2");
  tryMotorB(7, 9, 6, "BIN1=D7 BIN2=D9");
  tryMotorB(7, 11,6, "BIN1=D7 BIN2=D11");

  // ── Motor B with PWM on D5 ──
  Serial.println("[MOTOR B — PWM=D5]");
  tryMotorB(4, 3, 5, "BIN1=D4 BIN2=D3");
  tryMotorB(3, 4, 5, "BIN1=D3 BIN2=D4");

  allStop();
  Serial.println("\n=== Done. Report which label caused movement. ===");
}

void loop() {}
