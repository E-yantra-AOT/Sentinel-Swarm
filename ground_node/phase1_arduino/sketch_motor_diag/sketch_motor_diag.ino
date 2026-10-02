/**
 * motor_diag.ino — Raw Motor Pin Diagnostic
 *
 * No serial protocol, no parsing. Just directly drives every
 * possible motor pin combination to find what works.
 * Motors will try to spin in sequence — watch which one responds.
 *
 * Also sets STBY=HIGH on all candidate pins (3, 8, 12, 13) in case
 * your board revision needs it explicitly driven.
 */

// ── Candidate STBY pins (try all to be safe) ──────────────────────
// On most AlphaBot2-Ar boards STBY is hardwired HIGH,
// but some revisions connect it to an Arduino pin.
#define STBY_CANDIDATES_COUNT 4
const int STBY_CANDIDATES[] = {3, 8, 12, 13};

// ── Motor pin sets to try (PWM_A, DIR_A, PWM_B, DIR_B) ───────────
// Set 0: from your pins.h (D5/D4/D6/D7) — most likely
// Set 1: swapped A/B   
// Set 2: L9110s style (2 dir pins per motor, no PWM) — D4/D5/D6/D7 as IA/IB
#define TEST_SPEED 180  // 0–255, about 70% — clear and safe
#define RUN_MS    1200  // ms each direction runs

void allStop() {
  for (int p = 2; p <= 13; p++) {
    analogWrite(p, 0);
    digitalWrite(p, LOW);
  }
}

void setup() {
  Serial.begin(115200);

  // Set ALL digital pins 2-13 as outputs
  for (int p = 2; p <= 13; p++) {
    pinMode(p, OUTPUT);
    digitalWrite(p, LOW);
  }

  // Drive all STBY candidates HIGH (belt-and-suspenders)
  for (int i = 0; i < STBY_CANDIDATES_COUNT; i++) {
    digitalWrite(STBY_CANDIDATES[i], HIGH);
  }
  delay(200);

  Serial.println("=== Motor Pin Diagnostic ===");
  Serial.println("Watch/listen for motor movement on each step.");

  // ── TEST SET 0: pins.h mapping (D5 PWM_A, D4 DIR_A, D6 PWM_B, D7 DIR_B) ──
  Serial.println("\n[SET 0] Standard pins.h: PWM_A=D5 DIR_A=D4 PWM_B=D6 DIR_B=D7");

  Serial.println("  Step 1: Motor A FORWARD (D4=HIGH, D5=PWM)");
  digitalWrite(4, HIGH);  analogWrite(5, TEST_SPEED);
  digitalWrite(7, LOW);   analogWrite(6, 0);
  delay(RUN_MS);

  Serial.println("  Step 2: Motor A BACKWARD (D4=LOW, D5=PWM)");
  digitalWrite(4, LOW);   analogWrite(5, TEST_SPEED);
  delay(RUN_MS);

  allStop(); delay(400);

  Serial.println("  Step 3: Motor B FORWARD (D7=HIGH, D6=PWM)");
  digitalWrite(4, LOW);   analogWrite(5, 0);
  digitalWrite(7, HIGH);  analogWrite(6, TEST_SPEED);
  delay(RUN_MS);

  Serial.println("  Step 4: Motor B BACKWARD (D7=LOW, D6=PWM)");
  digitalWrite(7, LOW);   analogWrite(6, TEST_SPEED);
  delay(RUN_MS);

  allStop(); delay(600);

  // ── TEST SET 1: L9110S style — both dir pins toggled, no PWM ─────
  // L9110S: IA=HIGH IB=LOW → forward | IA=LOW IB=HIGH → backward
  Serial.println("\n[SET 1] L9110S style: D4/D5 as IA/IB for A, D6/D7 as IA/IB for B");

  Serial.println("  Step 5: Motor A FWD (D4=HIGH, D5=LOW)");
  digitalWrite(4, HIGH); digitalWrite(5, LOW);
  digitalWrite(6, LOW);  digitalWrite(7, LOW);
  delay(RUN_MS);

  Serial.println("  Step 6: Motor B FWD (D6=HIGH, D7=LOW)");
  digitalWrite(4, LOW);  digitalWrite(5, LOW);
  digitalWrite(6, HIGH); digitalWrite(7, LOW);
  delay(RUN_MS);

  allStop(); delay(600);

  // ── TEST SET 2: Shifted — D6/D7 as A, D4/D5 as B ────────────────
  Serial.println("\n[SET 2] Swapped: PWM_A=D6 DIR_A=D7 PWM_B=D4 DIR_B=D5");

  Serial.println("  Step 7: Motor A FWD (D7=HIGH, D6=PWM)");
  digitalWrite(7, HIGH); analogWrite(6, TEST_SPEED);
  delay(RUN_MS);

  Serial.println("  Step 8: Motor B FWD (D5=HIGH, D4=PWM)");
  digitalWrite(7, LOW);  analogWrite(6, 0);
  digitalWrite(5, HIGH); analogWrite(4, TEST_SPEED);
  delay(RUN_MS);

  allStop();
  Serial.println("\n=== Done. Reply which steps had movement ===");
}

void loop() {
  // Nothing — all tests ran in setup()
}
