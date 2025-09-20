#include <Arduino.h>
#include <Wire.h>
#include "MPU9250.h"

MPU9250 mpu;

// ---------------- Motor Pins ----------------
#define M1_POS 26
#define M1_NEG 25
#define M2_POS 14
#define M2_NEG 27
#define M3_POS 17
#define M3_NEG 16

// ---------------- I2C Pins ----------------
#define SDA_PIN 21
#define SCL_PIN 22

// ---------------- Structs ----------------
typedef struct {
  float phi;
  float theta;
} Psi;

typedef struct {
  float phi_desired;
  float theta_desired;
} Psi_desired;

// ---------------- Globals ----------------
volatile Psi psi = {0.0001, 0.0001};                 // IMU-derived bend
volatile Psi_desired psi_desired = {0.0, 0.0}; // Desired psi

volatile float l1 = 0.0, l2 = 0.0, l3 = 0.0;            // Desired tendon lengths
volatile float l1_actual = 0.0, l2_actual = 0.0, l3_actual = 0.0; // Actual tendon lengths

// ---------------- Hardware constants ----------------
const float TENDON_R = 0.02f;              // geometry radius
const float DEAD_BAND = 0.01f; // 1 mm deadband
const float PWM_SCALE = 100.0f; // scale PID output to 0-255
const float L0[3] = {0.10f, 0.10f, 0.10f}; // rest/pre-tension lengths (meters)

// ---------------- PID gains ----------------
float Kp = 1000.0;
float Ki = 0.0;
float Kd = 0.0;

// PID state
float e1_prev = 0.0, e1_integral = 0.0;
float e2_prev = 0.0, e2_integral = 0.0;
float e3_prev = 0.0, e3_integral = 0.0;

// ---------------- Functions ----------------

// Convert desired psi -> desired tendon lengths
void psi2length(float phi, float theta) {
  l1 = -TENDON_R * cos(theta) * phi;
  l2 = -TENDON_R * ((-cos(theta)/2.0f) + (sqrt(3.0f)*sin(theta)/2.0f)) * phi;
  l3 = -TENDON_R * ((-cos(theta)/2.0f) - (sqrt(3.0f)*sin(theta)/2.0f)) * phi;

  float max_l = max(l1, max(l2, l3));

  // Shift so that the largest is zero (others become ≤ 0)
  if (max_l > 0) {
    l1 = l1 - max_l;
    l2 = l2 - max_l;
    l3 = l3 - max_l;
    
  }

  Serial.print("l1: ");
  Serial.print(l1, 4);  // 4 decimal places
  Serial.print("   l2: ");
  Serial.print(l2, 4);
  Serial.print("   l3: ");
  Serial.println(l3, 4);
}

// Convert measured psi -> actual tendon lengths
void psi2lengthactual(float phi, float theta) {
  l1_actual = -TENDON_R * cos(theta) * phi;
  l2_actual = -TENDON_R * ((-cos(theta)/2.0f) + (sqrt(3.0f)*sin(theta)/2.0f)) * phi;
  l3_actual = -TENDON_R * ((-cos(theta)/2.0f) - (sqrt(3.0f)*sin(theta)/2.0f)) * phi;

  float max_l = max(l1_actual, max(l2_actual, l3_actual));
  if (max_l > 0) {
    l1_actual = l1_actual - max_l;
    l2_actual = l2_actual - max_l;
    l3_actual = l3_actual - max_l;
  }
}

// Convert IMU readings to psi (radians)
void imu2bend(float roll_deg, float pitch_deg) {
  const float pi = 3.14159265f;
  float roll  = roll_deg * pi/180.0f;
  float pitch = pitch_deg * pi/180.0f;

  float sr = sin(roll), cr = cos(roll);
  float sp = sin(pitch), cp = cos(pitch);

  float cosphi = cp * cr;
  float sinphi = sqrt((sp*cr)*(sp*cr) + sr*sr);

  psi.phi   = atan2(sinphi, cosphi);
  psi.theta = atan2(-sr, sp*cr);
}

// Map PID output to motor driver
void setMotorPWM(int posPin, int negPin, float pwm) {
  int pwmVal = min(255, max(0, (int)(fabs(pwm) * PWM_SCALE)));

  // Errors
  float phi_error   = psi_desired.phi_desired - psi.phi;
  float theta_error = psi_desired.theta_desired - psi.theta;

  // Tolerances
  float phi_tol   = 0.05f * fabs(psi_desired.phi_desired);   // 5% buffer
  float theta_tol = 0.05f * fabs(psi_desired.theta_desired);

  // --- Nested buffer zone ---
  if (fabs(theta_error) < theta_tol) {
    if (fabs(phi_error) < phi_tol) {
      // Close enough in both φ and θ → stop motor
      analogWrite(posPin, 0);
      analogWrite(negPin, 0);
      return;
    }
  }

  // Normal motor drive logic
  if (pwm < DEAD_BAND) {
    analogWrite(posPin, pwmVal);
    analogWrite(negPin, 0);
  } else if (pwm > -DEAD_BAND) {
    analogWrite(posPin, 0);
    analogWrite(negPin, pwmVal);
  } else {
    analogWrite(posPin, 0);
    analogWrite(negPin, 0);
  }
}





// ---------------- FreeRTOS Tasks ----------------

// Read IMU and update psi
void TaskReadIMU(void *pvParameters) {
  (void) pvParameters;
  static uint32_t prev_ms = 0;

  for (;;) {
    if (mpu.update()) {
      uint32_t now = millis();
      if (now - prev_ms >= 25) {  // 40 Hz
        float roll  = mpu.getRoll();
        float pitch = mpu.getPitch() * -1;

        // Compute bending angles
        imu2bend(roll, pitch);

        // Update actual tendon lengths
        psi2lengthactual(psi.phi, psi.theta);

        // Debug: print phi/theta
        //Serial.print("phi (rad): "); Serial.print(psi.phi, 4);
        //Serial.print("\t theta (rad): "); Serial.println(psi.theta, 4);

        prev_ms = now;
      }
    }
    vTaskDelay(1 / portTICK_PERIOD_MS); // Yield to other tasks
  }
}

// PID motor control task
void TaskMotor(void *pvParameters) {
  int motor = *((int*)pvParameters);
  const float dt = 0.05f; // 50 ms

  for (;;) {
    psi2length(psi_desired.phi_desired, psi_desired.theta_desired);

    float error, derivative, pwm_output;
    switch(motor) {
      case 1:
        error = l1 - l1_actual;
        e1_integral += error * dt;
        derivative = (error - e1_prev) / dt;
        pwm_output = Kp*error + Ki*e1_integral + Kd*derivative;
        e1_prev = error;
        setMotorPWM(M1_POS, M1_NEG, pwm_output);
        break;
      case 2:
        error = l2 - l2_actual;
        e2_integral += error * dt;
        derivative = (error - e2_prev) / dt;
        pwm_output = Kp*error + Ki*e2_integral + Kd*derivative;
        e2_prev = error;
        setMotorPWM(M2_POS, M2_NEG, pwm_output);
        break;
      case 3:
        error = l3 - l3_actual;
        e3_integral += error * dt;
        derivative = (error - e3_prev) / dt;
        pwm_output = Kp*error + Ki*e3_integral + Kd*derivative;
        e3_prev = error;
        setMotorPWM(M3_POS, M3_NEG, pwm_output);
        break;
    }

    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}

// Serial input task: update psi_desired from Serial Monitor
void SerialTask(void *pvParameters) {
  (void) pvParameters;
  String input;

  for (;;) {
    if (Serial.available()) {
      input = Serial.readStringUntil('\n');  // read until newline

      // Expect input format: "phi theta"
      float phiIn, thetaIn;
      if (sscanf(input.c_str(), "%f %f", &phiIn, &thetaIn) == 2) {
        psi_desired.phi_desired = phiIn;
        psi_desired.theta_desired = thetaIn;
        Serial.print("Updated psi_desired → phi: ");
        Serial.print(psi_desired.phi_desired);
        Serial.print(" | theta: ");
        Serial.println(psi_desired.theta_desired);
      } else {
        Serial.println("Invalid input. Format: <phi> <theta>");
      }
    }
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}

// ---------------- Setup / Loop ----------------
void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);
  delay(2000);

  // Motor pins
  pinMode(M1_POS, OUTPUT); pinMode(M1_NEG, OUTPUT);
  pinMode(M2_POS, OUTPUT); pinMode(M2_NEG, OUTPUT);
  pinMode(M3_POS, OUTPUT); pinMode(M3_NEG, OUTPUT);

  // IMU
  if (!mpu.setup(0x68)) {
    Serial.println("MPU9250 init failed!");
    while (1);
  }
  Serial.println("Calibrating IMU...");
  mpu.calibrateAccelGyro();
  mpu.calibrateMag();
  Serial.println("Calibration complete.");

  static int motor1_id = 1;
  static int motor2_id = 2;
  static int motor3_id = 3;

  // Create FreeRTOS tasks with safe stack sizes
  xTaskCreate(TaskReadIMU, "ReadIMU", 2048, NULL, 2, NULL);
  xTaskCreate(TaskMotor, "Motor1", 2048, &motor1_id, 1, NULL);
  xTaskCreate(TaskMotor, "Motor2", 2048, &motor2_id, 1, NULL);
  xTaskCreate(TaskMotor, "Motor3", 2048, &motor3_id, 1, NULL);
  xTaskCreate(SerialTask, "SerialInput", 4096, NULL, 1, NULL);
}

void loop() {
  // empty: all handled in FreeRTOS tasks
}
