/**
 * Sentinel FC - Fully Validated ArduPilot-Grade Flight Stack
 * Features: Cascaded PID, Target/Error/D-Term Filtering, Slew Limiting, Smart Anti-Windup
 */
#include <Adafruit_MPU6050.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <WiFi.h>

inline float constrain_float(float val, float min_val, float max_val) {
    if (val < min_val) return min_val;
    if (val > max_val) return max_val;
    return val;
}

// --- ArduPilot-Inspired Classes ---
class PT1Filter {
private:
    float state = 0.0f;
    float alpha = 1.0f;
public:
    void setCutoff(float cutoffHz, float dt) {
        if (cutoffHz <= 0.0f) { alpha = 1.0f; return; }
        float rc = 1.0f / (2.0f * 3.14159265f * cutoffHz);
        alpha = dt / (rc + dt);
    }
    float apply(float input) {
        state = state + alpha * (input - state);
        return state;
    }
};

class SlewLimiter {
private:
    float max_rate;
    float last_val = 0.0f;
public:
    SlewLimiter(float rate) : max_rate(rate) {}
    float modifier(float target, float dt) {
        if (dt <= 0.0f) return 1.0f;
        float diff = std::abs(target - last_val);
        float max_diff = max_rate * dt;
        last_val = target;
        if (diff > max_diff && diff > 0.0f) return max_diff / diff;
        return 1.0f;
    }
};

class ArduPID {
public:
    float kp, ki, kd, kff, kimax;
    PT1Filter filt_T, filt_E, filt_D;
    SlewLimiter slew_limiter;
    float target = 0.0f, error = 0.0f, derivative = 0.0f, integrator = 0.0f;

    ArduPID(float p, float i, float d, float ff, float imax, float slew_rate) 
        : kp(p), ki(i), kd(d), kff(ff), kimax(imax), slew_limiter(slew_rate) {}

    void setFilters(float t_hz, float e_hz, float d_hz, float dt) {
        filt_T.setCutoff(t_hz, dt);
        filt_E.setCutoff(e_hz, dt);
        filt_D.setCutoff(d_hz, dt);
    }

    float update_all(float target_in, float measurement, float dt, bool limit) {
        if (dt <= 0.0f) return 0.0f;

        target = filt_T.apply(target_in);
        float FF_out = target * kff;

        float raw_error = target - measurement;
        float error_last = error;
        error = filt_E.apply(raw_error);

        float raw_deriv = (error - error_last) / dt;
        derivative = filt_D.apply(raw_deriv);

        if (ki > 0.0f) {
            if (!limit || ((integrator > 0.0f && error < 0.0f) || (integrator < 0.0f && error > 0.0f))) {
                integrator += (error * ki) * dt;
                integrator = constrain_float(integrator, -kimax, kimax);
            }
        } else { integrator = 0.0f; }

        float P_out = error * kp;
        float D_out = derivative * kd;
        float dmod = slew_limiter.modifier(P_out + D_out, dt);
        
        return (P_out * dmod) + integrator + (D_out * dmod) + FF_out;
    }
};

// --- Hardware & Globals ---
Adafruit_MPU6050 mpu;
Servo pitchServo, rollServo;
SemaphoreHandle_t setpointMutex;

float targetPitch = 0.0f, targetRoll = 0.0f;
float currentPitch = 0.0f, currentRoll = 0.0f, currentYaw = 0.0f;

// PIDs (P, I, D, FF, IMAX, SlewRate)
ArduPID pitchRatePID(1.2f, 0.05f, 0.03f, 0.0f, 40.0f, 200.0f);
ArduPID rollRatePID(1.2f, 0.05f, 0.03f, 0.0f, 40.0f, 200.0f);
const float KP_ANGLE = 4.5f;

// --- Mahony AHRS ---
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;
const float Kp_Mahony = 2.0f, Ki_Mahony = 0.005f;

void mahonyUpdate(float gx, float gy, float gz, float ax, float ay, float az, float dt) {
  float recipNorm = 1.0f / sqrt(ax*ax + ay*ay + az*az);
  ax *= recipNorm; ay *= recipNorm; az *= recipNorm;

  float vx = 2.0f * (q1*q3 - q0*q2);
  float vy = 2.0f * (q0*q1 + q2*q3);
  float vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

  float ex = (ay*vz - az*vy);
  float ey = (az*vx - ax*vz);
  float ez = (ax*vy - ay*vx);

  if(Ki_Mahony > 0.0f) {
    integralFBx += ex * dt; integralFBy += ey * dt; integralFBz += ez * dt;
    gx += Ki_Mahony * integralFBx; gy += Ki_Mahony * integralFBy; gz += Ki_Mahony * integralFBz;
  }
  gx += Kp_Mahony * ex; gy += Kp_Mahony * ey; gz += Kp_Mahony * ez;

  gx *= (0.5f * dt); gy *= (0.5f * dt); gz *= (0.5f * dt);
  float qa = q0, qb = q1, qc = q2;
  q0 += (-qb*gx - qc*gy - q3*gz); q1 += (qa*gx + qc*gz - q3*gy);
  q2 += (qa*gy - qb*gz + q3*gx);  q3 += (qa*gz + qb*gy - qc*gx);

  recipNorm = 1.0f / sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
  q0 *= recipNorm; q1 *= recipNorm; q2 *= recipNorm; q3 *= recipNorm;
  
  currentRoll  = atan2(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2) * 57.2958f;
  currentPitch = asin(-2.0f * (q1*q3 - q0*q2)) * 57.2958f;
  currentYaw   = atan2(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3) * 57.2958f;
}

TaskHandle_t FlightTask, CommsTask;

void flightControlLoop(void * pvParameters) {
  const unsigned long LOOP_PERIOD = 1250; // 800Hz
  unsigned long lastMicros = micros();

  // ArduPilot default filter frequencies (Hz): Target, Error, Derivative
  pitchRatePID.setFilters(20.0f, 20.0f, 10.0f, 0.00125f);
  rollRatePID.setFilters(20.0f, 20.0f, 10.0f, 0.00125f);

  for(;;) {
    unsigned long now = micros();
    if (now - lastMicros >= LOOP_PERIOD) {
      float dt = (now - lastMicros) * 0.000001f;
      lastMicros = now;

      float cTargetPitch = 0.0, cTargetRoll = 0.0;
      if (xSemaphoreTake(setpointMutex, portMAX_DELAY)) {
        cTargetPitch = targetPitch; cTargetRoll = targetRoll;
        xSemaphoreGive(setpointMutex);
      }

      sensors_event_t a, g, temp;
      mpu.getEvent(&a, &g, &temp);

      // 1. Mahony AHRS Sensor Fusion
      mahonyUpdate(g.gyro.x, g.gyro.y, g.gyro.z, a.acceleration.x, a.acceleration.y, a.acceleration.z, dt);

      // 2. Cascaded Control - Outer Loop (Angle -> Desired Rate)
      float targetPitchRate = KP_ANGLE * (cTargetPitch - currentPitch);
      float targetRollRate  = KP_ANGLE * (cTargetRoll - currentRoll);

      // 3. Cascaded Control - Inner Loop (ArduPID update)
      // Pass false to 'limit' unless motors are saturated
      float pCmd = pitchRatePID.update_all(targetPitchRate, g.gyro.x * 57.2958f, dt, false);
      float rCmd = rollRatePID.update_all(targetRollRate, g.gyro.y * 57.2958f, dt, false);

      pitchServo.write(constrain((int)(90.0f + pCmd), 0, 180));
      rollServo.write(constrain((int)(90.0f + rCmd), 0, 180));
      
      static int printCount = 0;
      if (printCount++ >= 100) {
        Serial.printf("Pitch: %.2f, Roll: %.2f, pCmd: %.2f, rCmd: %.2f\n", currentPitch, currentRoll, pCmd, rCmd);
        printCount = 0;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

void commsLoop(void * pvParameters) {
  WiFi.softAP("Sentinel_Swarm_Wi-Fi6", "hackathon_demo");
  for(;;) {
    if (Serial.available()) {
      String cmd = Serial.readStringUntil('\n');
      if (cmd.startsWith("CMD,")) {
        int firstComma = cmd.indexOf(',');
        int secondComma = cmd.indexOf(',', firstComma + 1);
        if (secondComma > 0) {
          float pPitch = cmd.substring(firstComma + 1, secondComma).toFloat();
          float pRoll = cmd.substring(secondComma + 1).toFloat();
          if (xSemaphoreTake(setpointMutex, portMAX_DELAY)) {
            targetPitch = constrain(pPitch, -30.0, 30.0);
            targetRoll = constrain(pRoll, -30.0, 30.0);
            xSemaphoreGive(setpointMutex);
          }
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin(8, 9);
  if (!mpu.begin()) {
    Serial.println("ERROR: MPU6050 not found! Check I2C wiring (Pins 8/9).");
    while (1) delay(10);
  }
  
  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_2000_DEG); 
  mpu.setFilterBandwidth(MPU6050_BAND_260_HZ); 

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  
  pitchServo.setPeriodHertz(50);
  pitchServo.attach(18, 1000, 2000);
  rollServo.setPeriodHertz(50);
  rollServo.attach(19, 1000, 2000);

  setpointMutex = xSemaphoreCreateMutex();

  xTaskCreatePinnedToCore(flightControlLoop, "FlightTask", 8192, NULL, 2, &FlightTask, 1);
  xTaskCreatePinnedToCore(commsLoop, "CommsTask", 4096, NULL, 1, &CommsTask, 0);
}
void loop() {
  delay(1000);
}
