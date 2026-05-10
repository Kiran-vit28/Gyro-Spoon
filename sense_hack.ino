/* Tremor-stabilizer for spoon — ESP32 (uses driver LEDC API)
   Requirements:
   - Board: ESP32 Dev Module
   - MPU6050 library: MPU6050 (Electronic Cats / Jeff Rowberg)
   - MPU6050 wired to 3.3V, SDA=21, SCL=22
   - Servos powered from external 5V supply (common GND)
*/

#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>
#include <driver/ledc.h>

MPU6050 mpu;

// servo pins
const int SERVO1_PIN = 18;
const int SERVO2_PIN = 19;

// LEDC / PWM settings (driver API)
const int FREQ = 50; // 50 Hz for hobby servo
const ledc_timer_t TIMER = LEDC_TIMER_0;
const ledc_mode_t MODE = LEDC_LOW_SPEED_MODE;
const ledc_timer_bit_t RES = LEDC_TIMER_16_BIT;
const ledc_channel_t CH1 = LEDC_CHANNEL_0;
const ledc_channel_t CH2 = LEDC_CHANNEL_1;

// servo pulse limits (microseconds)
const int PULSE_MIN_US = 1000;
const int PULSE_MAX_US = 2000;

// PID params (start values — tune on hardware)
float kp = 3.5f;
float ki = 0.02f;
float kd = 1.5f;

// PID state
struct PID { float Kp, Ki, Kd, I, prevErr, outMin, outMax; };
PID pidRoll  = {kp, ki, kd, 0.0f, 0.0f, -45.0f, 45.0f};
PID pidPitch = {kp, ki, kd, 0.0f, 0.0f, -45.0f, 45.0f};

// complementary & LP filter
float alpha = 0.98f;
float roll_f = 0.0f, pitch_f = 0.0f;
const float ctrlFreq = 100.0f;
const float ctrlTs = 1.0f / ctrlFreq;
const float lpCutoff = 2.0f; // Hz
float lpAlpha;
float lpRoll = 0.0f, lpPitch = 0.0f;

// gyro bias
float gyroBiasX = 0.0f, gyroBiasY = 0.0f;

// timing
unsigned long lastMicros = 0;

// Utility: pulse_us -> LEDC duty for given RES, FREQ
uint32_t pulseToDuty(int pulse_us) {
  float period_us = 1000000.0f / (float)FREQ; // e.g. 20000us
  uint32_t maxDuty = (1UL << RES) - 1;
  float duty = (float)pulse_us / period_us * (float)maxDuty;
  if (duty < 0.0f) duty = 0.0f;
  if (duty > (float)maxDuty) duty = (float)maxDuty;
  return (uint32_t)(duty + 0.5f);
}

// write servo by channel using driver API
void writeServoDriver(ledc_channel_t ch, int angle) {
  angle = constrain(angle, 0, 180);
  int pulse = map(angle, 0, 180, PULSE_MIN_US, PULSE_MAX_US);
  uint32_t duty = pulseToDuty(pulse);
  ledc_set_duty(MODE, ch, duty);
  ledc_update_duty(MODE, ch);
}

// calibrate gyro biases (keep unit still while running)
void calibrateGyro(int samples = 500) {
  long sx = 0, sy = 0;
  for (int i=0;i<samples;i++){
    int16_t ax,ay,az,gx,gy,gz;
    mpu.getMotion6(&ax,&ay,&az,&gx,&gy,&gz); // returns void; fills values
    sx += gx;
    sy += gy;
    delay(2);
  }
  gyroBiasX = (float)sx / samples / 131.0f;
  gyroBiasY = (float)sy / samples / 131.0f;
  Serial.print("Gyro bias X Y: "); Serial.print(gyroBiasX); Serial.print(" "); Serial.println(gyroBiasY);
}

// PID step with anti-windup
float pidStep(PID &p, float err, float dt) {
  p.I += err * dt;
  const float I_MAX = 100.0f;
  if (p.I > I_MAX) p.I = I_MAX;
  if (p.I < -I_MAX) p.I = -I_MAX;
  float deriv = (dt > 1e-6f) ? (err - p.prevErr) / dt : 0.0f;
  float out = p.Kp * err + p.Ki * p.I + p.Kd * deriv;
  p.prevErr = err;
  if (out > p.outMax) out = p.outMax;
  if (out < p.outMin) out = p.outMin;
  return out;
}

void setupLEDCDriver() {
  // timer config (driver)
  ledc_timer_config_t tcfg = {
    .speed_mode = MODE,
    .duty_resolution = RES,
    .timer_num = TIMER,
    .freq_hz = FREQ,
    .clk_cfg = LEDC_AUTO_CLK
  };
  ledc_timer_config(&tcfg);

  // channel 1 config
  ledc_channel_config_t chcfg = {
    .gpio_num = SERVO1_PIN,
    .speed_mode = MODE,
    .channel = CH1,
    .intr_type = LEDC_INTR_DISABLE,
    .timer_sel = TIMER,
    .duty = 0,
    .hpoint = 0
  };
  ledc_channel_config(&chcfg);

  // channel 2 config (reuse struct)
  chcfg.gpio_num = SERVO2_PIN;
  chcfg.channel = CH2;
  ledc_channel_config(&chcfg);

  // set neutral pulse (1500us) initially
  uint32_t midDuty = pulseToDuty(1500);
  ledc_set_duty(MODE, CH1, midDuty);
  ledc_update_duty(MODE, CH1);
  ledc_set_duty(MODE, CH2, midDuty);
  ledc_update_duty(MODE, CH2);
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Wire.begin(); // default SDA=21, SCL=22 on most ESP32 dev boards

  mpu.initialize();
  Serial.println(mpu.testConnection() ? "MPU OK" : "MPU FAIL");

  // compute lpAlpha for discrete IIR
  float rc = 1.0f / (2.0f * PI * lpCutoff);
  lpAlpha = expf(-ctrlTs / rc);

  // setup LEDC using driver API
  setupLEDCDriver();

  // center servos (90 deg)
  writeServoDriver(CH1, 90);
  writeServoDriver(CH2, 90);

  // calibrate gyro (keep unit still)
  Serial.println("Calibrating gyro - keep still");
  calibrateGyro(500);

  lastMicros = micros();
}

void loop() {
  unsigned long now = micros();
  float dt = (now - lastMicros) * 1e-6f;
  if (dt <= 0.0f) { lastMicros = now; return; }
  if (dt > 0.05f) dt = 0.05f; // guard against long pauses
  lastMicros = now;

  // read IMU (getMotion6 is void)
  int16_t axr, ayr, azr, gxr, gyr, gzr;
  mpu.getMotion6(&axr, &ayr, &azr, &gxr, &gyr, &gzr);

  // scales
  float ax = (float)axr / 16384.0f;
  float ay = (float)ayr / 16384.0f;
  float az = (float)azr / 16384.0f;
  float gx = (float)gxr / 131.0f - gyroBiasX; // deg/s
  float gy = (float)gyr / 131.0f - gyroBiasY; // deg/s

  // accel-derived angles (deg)
  float rollAcc  = atan2f(ay, az) * 180.0f / PI;
  float pitchAcc = atan2f(-ax, sqrtf(ay*ay + az*az)) * 180.0f / PI;

  // complementary filter -> orientation
  roll_f  = alpha * (roll_f + gx * dt) + (1.0f - alpha) * rollAcc;
  pitch_f = alpha * (pitch_f + gy * dt) + (1.0f - alpha) * pitchAcc;

  // low-pass intended motion (IIR)
  lpRoll  = lpAlpha * lpRoll  + (1.0f - lpAlpha) * roll_f;
  lpPitch = lpAlpha * lpPitch + (1.0f - lpAlpha) * pitch_f;

  // tremor (high frequency) = measured - intended
  float tremorR = roll_f - lpRoll;
  float tremorP = pitch_f - lpPitch;

  // control to cancel tremor (error = -tremor)
  float errR = -tremorR;
  float errP = -tremorP;

  float outR = pidStep(pidRoll, errR, dt);
  float outP = pidStep(pidPitch, errP, dt);

  // map to servo angles (center 90)
  int servoAngle1 = (int)constrain(90.0f + outR, 0.0f, 180.0f);
  int servoAngle2 = (int)constrain(90.0f + outP, 0.0f, 180.0f);

  // write with driver
  writeServoDriver(CH1, servoAngle1);
  writeServoDriver(CH2, servoAngle2);

  // debug (print every ~50ms)
  static float dbgAcc = 0;
  dbgAcc += dt;
  if (dbgAcc >= 0.05f) {
    dbgAcc = 0;
    Serial.printf("r:%.2f lpR:%.2f tR:%.3f outR:%.2f | p:%.2f lpP:%.2f tP:%.3f outP:%.2f\n",
                  roll_f, lpRoll, tremorR, outR, pitch_f, lpPitch, tremorP, outP);
  }

  // tiny yield
  delay(2);
}