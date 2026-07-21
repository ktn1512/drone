#include <esp_now.h>
#include <WiFi.h>
#include <SPI.h>
#include <ESP32Servo.h>
#include <math.h>
#include "ICM42607_SPI.h"

// ==========================================================
// ==========================================================

// ================= ESC PINS =================
#define ESC_M1_PIN  13
#define ESC_M2_PIN 12
#define ESC_M3_PIN  14
#define ESC_M4_PIN  27

#define PULSE_MIN      1000
#define PULSE_MAX      2000
#define PULSE_OFF      1000
#define THROTTLE_IDLE  1100
#define THROTTLE_LIMIT 1800

Servo esc1, esc2, esc3, esc4;

// ================= ICM42607 SPI =================
#define ICM_CS 5
#define SPI_SCK  18
#define SPI_MISO 19
#define SPI_MOSI 23
#define ICM_SPI_HZ 10000000

ICM42607_SPI imu(ICM_CS, SPI);

// ==========================================================
// ATTITUDE ESTIMATOR CONFIG - ĐÃ SỬA (KHÔNG HOÁN ĐỔI)
// ==========================================================
const float COMPLEMENTARY_ALPHA = 0.99f;

// Hệ số dấu (điều chỉnh nếu cảm biến gắn ngược chiều)
#define DRONE_ROLL_SIGN    -1.0f
#define DRONE_PITCH_SIGN   -1.0f
#define DRONE_YAW_SIGN     -1.0f
#define DRONE_ROLL_RATE_SIGN  -1.0f
#define DRONE_PITCH_RATE_SIGN -1.0f
#define DRONE_YAW_RATE_SIGN   -1.0f

float gyroBiasX = 0.0f;
float gyroBiasY = 0.0f;
float gyroBiasZ = 0.0f;

// Biến lọc gyro Z (bổ sung để giảm nhiễu)
float gzFilt = 0.0f;

float icmRollX = 0.0f;    // góc roll từ accel (quanh trục X của IMU)
float icmPitchY = 0.0f;   // góc pitch từ accel (quanh trục Y của IMU)

float axFilt = 0.0f;
float ayFilt = 0.0f;
float azFilt = 1.0f;

float droneRoll = 0.0f;
float dronePitch = 0.0f;
float droneYaw = 0.0f;

uint32_t lastAttitudeUs = 0;

// ===== Định nghĩa struct AttitudeData =====
struct AttitudeData {
  float roll;
  float pitch;
  float yaw;
  float gx;
  float gy;
  float gz;
};

AttitudeData lastAtt;

// ================= ESP-NOW DATA =================
typedef struct __attribute__((packed)) {
  uint8_t magic;
  uint8_t cmd;
  uint16_t pulse_us;
  uint16_t adc1;
  uint16_t adc2;
  uint8_t button;
  uint8_t state;
  uint32_t seq;
} ControlData;

ControlData rxData;
volatile uint16_t receiverThrottle = PULSE_OFF;
volatile bool receiverCutoff = true;

// ================= TELEMETRY =================
typedef struct __attribute__((packed))
{
  float roll;
  float pitch;
  float yaw;
  float gyroRoll;
  float gyroPitch;
  float gyroYaw;
  float PAngleRoll;
  float IAngleRoll;
  float DAngleRoll;
  float PAnglePitch;
  float IAnglePitch;
  float DAnglePitch;
  float PRateRoll;
  float IRateRoll;
  float DRateRoll;
  float PRatePitch;
  float IRatePitch;
  float DRatePitch;
  float PRateYaw;
  float IRateYaw;
  float DRateYaw;
  uint16_t motor1;
  uint16_t motor2;
  uint16_t motor3;
  uint16_t motor4;
  uint32_t counter;
} TelemetryData;

TelemetryData txTelemetry;

// ================= PID DATA (nhận từ TX) =================
typedef struct __attribute__((packed))
{
  uint8_t magic;          // 0xA6
  float PAngleRoll;
  float IAngleRoll;
  float DAngleRoll;
  float PAnglePitch;
  float IAnglePitch;
  float DAnglePitch;
  float PRateRoll;
  float IRateRoll;
  float DRateRoll;
  float PRatePitch;
  float IRatePitch;
  float DRatePitch;
  float PRateYaw;
  float IRateYaw;
  float DRateYaw;
} PIDData;

uint32_t lastPacketTime = 0;
uint8_t telemetryPeer[] = { 0x00, 0x70, 0x07, 0x17, 0x9F, 0xBC };

// ================= PID BIẾN =================
float DesiredAngleRoll = 0.0f;
float DesiredAnglePitch = 0.0f;
float DesiredRateYaw = 0.0f;
float DesiredRateRoll = 0.0f;
float DesiredRatePitch = 0.0f;

float ErrorAngleRoll = 0.0f;
float ErrorAnglePitch = 0.0f;
float ErrorRateRoll = 0.0f;
float ErrorRatePitch = 0.0f;
float ErrorRateYaw = 0.0f;

float InputRoll = 0.0f;
float InputPitch = 0.0f;
float InputYaw = 0.0f;

float PrevErrorAngleRoll = 0.0f;
float PrevErrorAnglePitch = 0.0f;
float PrevItermAngleRoll = 0.0f;
float PrevItermAnglePitch = 0.0f;

float PrevErrorRateRoll = 0.0f;
float PrevErrorRatePitch = 0.0f;
float PrevErrorRateYaw = 0.0f;
float PrevItermRateRoll = 0.0f;
float PrevItermRatePitch = 0.0f;
float PrevItermRateYaw = 0.0f;

float PIDReturn[3];

const float PID_OUTPUT_LIMIT = 400.0f;
const float PID_ITERM_LIMIT = 80.0f;

// PID Angle
float PAngleRoll  = 4.0f;
float IAngleRoll  = 1.55f;
float DAngleRoll  = 0.0f;

float PAnglePitch = 4.0f;
float IAnglePitch = 1.55f;
float DAnglePitch = 0.0f;

// PID Rate
float PRateRoll  = 5.0f;
float IRateRoll  = 2.5f;
float DRateRoll  = 0.0f;

float PRatePitch = 4.0f;
float IRatePitch = 2.5f;
float DRatePitch = 0.0f;

float PRateYaw   = 0.0f;
float IRateYaw   = 0.0f;
float DRateYaw   = 0.0f;

// ================= MOTOR =================
float MotorInput1 = PULSE_OFF;
float MotorInput2 = PULSE_OFF;
float MotorInput3 = PULSE_OFF;
float MotorInput4 = PULSE_OFF;

uint32_t LoopTimer = 0;
uint32_t lastPrint = 0;

#define LOOP_PERIOD_US 2500     // 400Hz
#define PRINT_PERIOD_MS 500

// ==========================================================
// UTILS
// ==========================================================
float wrap180(float angle) {
  while (angle > 180.0f) angle -= 360.0f;
  while (angle < -180.0f) angle += 360.0f;
  return angle;
}

// ================= PID FUNCTION =================
void pid_equation(float Error, float P, float I, float D,
                  float PrevError, float PrevIterm)
{
  const float dt = LOOP_PERIOD_US / 1000000.0f; 
  float Pterm = P * Error;

  float Iterm = PrevIterm + I * (Error + PrevError) * dt / 2.0f;
  if (Iterm > PID_ITERM_LIMIT) Iterm = PID_ITERM_LIMIT;
  else if (Iterm < -PID_ITERM_LIMIT) Iterm = -PID_ITERM_LIMIT;

  float Dterm = D * (Error - PrevError) / dt;

  float PIDOutput = Pterm + Iterm + Dterm;

  if (PIDOutput > PID_OUTPUT_LIMIT) PIDOutput = PID_OUTPUT_LIMIT;
  else if (PIDOutput < -PID_OUTPUT_LIMIT) PIDOutput = -PID_OUTPUT_LIMIT;

  PIDReturn[0] = PIDOutput;
  PIDReturn[1] = Error;
  PIDReturn[2] = Iterm;
}

void reset_pid()
{
  PrevErrorRateRoll = 0.0f;  PrevErrorRatePitch = 0.0f;  PrevErrorRateYaw = 0.0f;
  PrevItermRateRoll = 0.0f;  PrevItermRatePitch = 0.0f;  PrevItermRateYaw = 0.0f;
  PrevErrorAngleRoll = 0.0f; PrevErrorAnglePitch = 0.0f;
  PrevItermAngleRoll = 0.0f; PrevItermAnglePitch = 0.0f;
}

// ==========================================================
// IMU FUNCTIONS - ĐÃ SỬA KHÔNG HOÁN ĐỔI + CẢI TIẾN YAW
// ==========================================================
void calibrateGyroLocal(uint16_t samples, uint16_t delayMs)
{
  float sumGx = 0.0f, sumGy = 0.0f, sumGz = 0.0f;
  float sumRollAcc = 0.0f, sumPitchAcc = 0.0f;

  for (uint16_t i = 0; i < samples; i++) {
    ICM42607_SPI::Data data;
    imu.read(data);

    sumGx += data.gx;
    sumGy += data.gy;
    sumGz += data.gz;

    float rollAcc = atan2f(data.ay, data.az) * 180.0f / PI;
    float pitchAcc = atan2f(-data.ax, sqrtf(data.ay * data.ay + data.az * data.az)) * 180.0f / PI;

    sumRollAcc += rollAcc;
    sumPitchAcc += pitchAcc;

    delay(delayMs);
  }

  gyroBiasX = sumGx / samples;
  gyroBiasY = sumGy / samples;
  gyroBiasZ = sumGz / samples;

  // In giá trị bias để kiểm tra
  Serial.println("=== Gyro Bias ===");
  Serial.print("Bias X (deg/s): "); Serial.println(gyroBiasX, 4);
  Serial.print("Bias Y (deg/s): "); Serial.println(gyroBiasY, 4);
  Serial.print("Bias Z (deg/s): "); Serial.println(gyroBiasZ, 4);
  Serial.println("=================");

  icmRollX = sumRollAcc / samples;
  icmPitchY = sumPitchAcc / samples;

  droneRoll = wrap180(DRONE_ROLL_SIGN * icmRollX);
  dronePitch = wrap180(DRONE_PITCH_SIGN * icmPitchY);
  droneYaw = 0.0f;
  gzFilt = 0.0f;  // reset bộ lọc

  lastAttitudeUs = micros();
  Serial.println("Calibrate Gyro DONE!");
}

void updateAttitudeLocal(AttitudeData &att)
{
  ICM42607_SPI::Data data;
  imu.read(data);

  uint32_t nowUs = micros();
  float dt = (nowUs - lastAttitudeUs) / 1000000.0f;
  lastAttitudeUs = nowUs;

  if (dt <= 0.0f || dt > 0.05f) dt = LOOP_PERIOD_US / 1000000.0f;

  float gx = data.gx - gyroBiasX;
  float gy = data.gy - gyroBiasY;
  float gz = data.gz - gyroBiasZ;

  // ---- Lọc gyro Z để giảm nhiễu trước khi tích phân ----
  const float GYRO_LPF = 0.15f;   // có thể điều chỉnh 0.05 → 0.30
  gzFilt += GYRO_LPF * (gz - gzFilt);

  // Lọc gia tốc (đã có sẵn)
  const float ACC_LPF = 0.12f;
  axFilt += ACC_LPF * (data.ax - axFilt);
  ayFilt += ACC_LPF * (data.ay - ayFilt);
  azFilt += ACC_LPF * (data.az - azFilt);

  float accRollX = atan2f(ayFilt, azFilt) * 180.0f / PI;
  float accPitchY = atan2f(-axFilt, sqrtf(ayFilt * ayFilt + azFilt * azFilt)) * 180.0f / PI;

  icmRollX = COMPLEMENTARY_ALPHA * (icmRollX + gx * dt) +
             (1.0f - COMPLEMENTARY_ALPHA) * accRollX;

  icmPitchY = COMPLEMENTARY_ALPHA * (icmPitchY + gy * dt) +
              (1.0f - COMPLEMENTARY_ALPHA) * accPitchY;

  icmRollX = wrap180(icmRollX);
  icmPitchY = wrap180(icmPitchY);

  // Gán góc (không hoán đổi)
  droneRoll = wrap180(DRONE_ROLL_SIGN * icmRollX);
  dronePitch = wrap180(DRONE_PITCH_SIGN * icmPitchY);

  // Tích phân yaw sử dụng gz đã lọc
  droneYaw = wrap180(droneYaw + DRONE_YAW_RATE_SIGN * gzFilt * dt);

  // Gán giá trị rate (không hoán đổi)
  att.roll = droneRoll;
  att.pitch = dronePitch;
  att.yaw = droneYaw;
  att.gx = DRONE_ROLL_RATE_SIGN * gx;   // roll rate = gx
  att.gy = DRONE_PITCH_RATE_SIGN * gy;  // pitch rate = gy
  att.gz = DRONE_YAW_RATE_SIGN * gz;
}

// ================= MOTOR WRITE =================
void writeMotors(int m1, int m2, int m3, int m4)
{
  esc1.writeMicroseconds(m1);
  esc2.writeMicroseconds(m2);
  esc3.writeMicroseconds(m3);
  esc4.writeMicroseconds(m4);
}

// ================= ESP-NOW RECEIVE =================
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len)
{
  uint8_t magic = incomingData[0];

  if (magic == 0xA5 && len == sizeof(ControlData))
  {
    memcpy(&rxData, incomingData, sizeof(rxData));
    if (rxData.magic != 0xA5) return;

    lastPacketTime = millis();

    if (rxData.cmd == 0 || rxData.button == 1) {
      receiverThrottle = PULSE_OFF;
      receiverCutoff = true;
    } else {
      receiverThrottle = constrain(rxData.pulse_us, PULSE_MIN, THROTTLE_LIMIT);
      receiverCutoff = false;
    }
  }
  else if (magic == 0xA6 && len == sizeof(PIDData))
  {
    PIDData pid;
    memcpy(&pid, incomingData, sizeof(pid));

    PAngleRoll  = pid.PAngleRoll;
    IAngleRoll  = pid.IAngleRoll;
    DAngleRoll  = pid.DAngleRoll;
    PAnglePitch = pid.PAnglePitch;
    IAnglePitch = pid.IAnglePitch;
    DAnglePitch = pid.DAnglePitch;
    PRateRoll   = pid.PRateRoll;
    IRateRoll   = pid.IRateRoll;
    DRateRoll   = pid.DRateRoll;
    PRatePitch  = pid.PRatePitch;
    IRatePitch  = pid.IRatePitch;
    DRatePitch  = pid.DRatePitch;
    PRateYaw    = pid.PRateYaw;
    IRateYaw    = pid.IRateYaw;
    DRateYaw    = pid.DRateYaw;

    Serial.println("PID updated via ESP-NOW");
  }
}

// ================= SEND TELEMETRY =================
void sendTelemetry()
{
  static uint32_t packetCounter = 0;

  txTelemetry.roll = droneRoll;
  txTelemetry.pitch = dronePitch;
  txTelemetry.yaw = droneYaw;

  txTelemetry.gyroRoll  = lastAtt.gx;
  txTelemetry.gyroPitch = lastAtt.gy;
  txTelemetry.gyroYaw   = lastAtt.gz;

  txTelemetry.PAngleRoll  = PAngleRoll;
  txTelemetry.IAngleRoll  = IAngleRoll;
  txTelemetry.DAngleRoll  = DAngleRoll;
  txTelemetry.PAnglePitch = PAnglePitch;
  txTelemetry.IAnglePitch = IAnglePitch;
  txTelemetry.DAnglePitch = DAnglePitch;

  txTelemetry.PRateRoll  = PRateRoll;
  txTelemetry.IRateRoll  = IRateRoll;
  txTelemetry.DRateRoll  = DRateRoll;
  txTelemetry.PRatePitch = PRatePitch;
  txTelemetry.IRatePitch = IRatePitch;
  txTelemetry.DRatePitch = DRatePitch;
  txTelemetry.PRateYaw   = PRateYaw;
  txTelemetry.IRateYaw   = IRateYaw;
  txTelemetry.DRateYaw   = DRateYaw;

  txTelemetry.motor1 = (uint16_t)MotorInput1;
  txTelemetry.motor2 = (uint16_t)MotorInput2;
  txTelemetry.motor3 = (uint16_t)MotorInput3;
  txTelemetry.motor4 = (uint16_t)MotorInput4;

  txTelemetry.counter = packetCounter++;

  esp_now_send(telemetryPeer, (uint8_t*)&txTelemetry, sizeof(txTelemetry));
}

// ================= SETUP =================
void setup()
{
  Serial.begin(115200);
  delay(1000);

  esc1.setPeriodHertz(400);
  esc2.setPeriodHertz(400);
  esc3.setPeriodHertz(400);
  esc4.setPeriodHertz(400);

  esc1.attach(ESC_M1_PIN, PULSE_MIN, PULSE_MAX);
  esc2.attach(ESC_M2_PIN, PULSE_MIN, PULSE_MAX);
  esc3.attach(ESC_M3_PIN, PULSE_MIN, PULSE_MAX);
  esc4.attach(ESC_M4_PIN, PULSE_MIN, PULSE_MAX);

  writeMotors(PULSE_OFF, PULSE_OFF, PULSE_OFF, PULSE_OFF);
  Serial.println("ESC ready");

  pinMode(ICM_CS, OUTPUT);
  digitalWrite(ICM_CS, HIGH);
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, ICM_CS);

  if (!imu.begin(ICM_SPI_HZ)) {
    Serial.println("ICM42607 loi ket noi");
    while (1) { writeMotors(PULSE_OFF, PULSE_OFF, PULSE_OFF, PULSE_OFF); delay(500); }
  }

  imu.setGyroFS(ICM42607_SPI::GYRO_FS_500DPS);
  imu.setAccelFS(ICM42607_SPI::ACCEL_FS_4G);
  imu.setODR(ICM42607_SPI::ODR_800HZ, ICM42607_SPI::ODR_800HZ);
  imu.setFilter(ICM42607_SPI::BW_180HZ, ICM42607_SPI::BW_180HZ);

  Serial.println("Calibrating gyro... (2000 samples)");
  calibrateGyroLocal(2000, 2);   // tăng số mẫu lên 2000 để bias ổn định hơn

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  if (esp_now_init() != ESP_OK) {
    while (1) { writeMotors(PULSE_OFF, PULSE_OFF, PULSE_OFF, PULSE_OFF); delay(500); }
  }
  esp_now_register_recv_cb(OnDataRecv);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, telemetryPeer, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Add Telemetry Peer Failed");
  }

  Serial.print("sizeof(TelemetryData) = ");
  Serial.println(sizeof(TelemetryData));
  Serial.println("Receiver Ready");
  lastPacketTime = millis();
  LoopTimer = micros();
}

// ================= LOOP =================
void loop()
{
  AttitudeData att;
  updateAttitudeLocal(att);
  lastAtt = att;

  if (millis() - lastPacketTime > 200) {
    receiverCutoff = true;
    receiverThrottle = PULSE_OFF;
  }

  float Roll = att.roll;
  float Pitch = att.pitch;
  float RateRoll = att.gx;
  float RatePitch = att.gy;
  float RateYaw = att.gz;

  uint16_t InputThrottle = receiverThrottle;

  // ================= ANGLE PID =================
  DesiredAngleRoll = 0.0f;
  DesiredAnglePitch = 0.0f;
  DesiredRateYaw = 0.0f;

  ErrorAngleRoll = DesiredAngleRoll - Roll;
  if (fabs(ErrorAngleRoll) < 1.0f)
    ErrorAngleRoll = 0.0f;
  ErrorAnglePitch = DesiredAnglePitch - Pitch;
if (fabs(ErrorAnglePitch) < 1.0f)
    ErrorAnglePitch = 0.0f;
  pid_equation(ErrorAngleRoll, PAngleRoll, IAngleRoll, DAngleRoll, PrevErrorAngleRoll, PrevItermAngleRoll);
  DesiredRateRoll = PIDReturn[0];
  PrevErrorAngleRoll = PIDReturn[1];
  PrevItermAngleRoll = PIDReturn[2];

  pid_equation(ErrorAnglePitch, PAnglePitch, IAnglePitch, DAnglePitch, PrevErrorAnglePitch, PrevItermAnglePitch);
  DesiredRatePitch = PIDReturn[0];
  PrevErrorAnglePitch = PIDReturn[1];
  PrevItermAnglePitch = PIDReturn[2];

  // ================= RATE PID =================
  ErrorRateRoll = DesiredRateRoll - RateRoll;
  ErrorRatePitch = DesiredRatePitch - RatePitch;
  ErrorRateYaw = DesiredRateYaw - RateYaw;

  pid_equation(ErrorRateRoll, PRateRoll, IRateRoll, DRateRoll, PrevErrorRateRoll, PrevItermRateRoll);
  InputRoll = PIDReturn[0];
  PrevErrorRateRoll = PIDReturn[1];
  PrevItermRateRoll = PIDReturn[2];

  pid_equation(ErrorRatePitch, PRatePitch, IRatePitch, DRatePitch, PrevErrorRatePitch, PrevItermRatePitch);
  InputPitch = PIDReturn[0];
  PrevErrorRatePitch = PIDReturn[1];
  PrevItermRatePitch = PIDReturn[2];

  pid_equation(ErrorRateYaw, PRateYaw, IRateYaw, DRateYaw, PrevErrorRateYaw, PrevItermRateYaw);
  InputYaw = PIDReturn[0];
  PrevErrorRateYaw = PIDReturn[1];
  PrevItermRateYaw = PIDReturn[2];

  // ================= MIXER (FRAME X) =================
  if (InputThrottle > THROTTLE_LIMIT) InputThrottle = THROTTLE_LIMIT;

  MotorInput1 = InputThrottle + InputPitch + InputRoll - InputYaw;
  MotorInput2 = InputThrottle + InputPitch - InputRoll + InputYaw;
  MotorInput3 = InputThrottle - InputPitch - InputRoll - InputYaw;
  MotorInput4 = InputThrottle - InputPitch + InputRoll + InputYaw;

  MotorInput1 = constrain(MotorInput1, PULSE_MIN, PULSE_MAX);
  MotorInput2 = constrain(MotorInput2, PULSE_MIN, PULSE_MAX);
  MotorInput3 = constrain(MotorInput3, PULSE_MIN, PULSE_MAX);
  MotorInput4 = constrain(MotorInput4, PULSE_MIN, PULSE_MAX);

  if (MotorInput1 < THROTTLE_IDLE) MotorInput1 = THROTTLE_IDLE;
  if (MotorInput2 < THROTTLE_IDLE) MotorInput2 = THROTTLE_IDLE;
  if (MotorInput3 < THROTTLE_IDLE) MotorInput3 = THROTTLE_IDLE;
  if (MotorInput4 < THROTTLE_IDLE) MotorInput4 = THROTTLE_IDLE;

  // ================= CUTOFF =================
  if (receiverCutoff || InputThrottle < 1050) {
    MotorInput1 = PULSE_OFF;
    MotorInput2 = PULSE_OFF;
    MotorInput3 = PULSE_OFF;
    MotorInput4 = PULSE_OFF;
    reset_pid();
  }

  writeMotors((int)MotorInput1, (int)MotorInput2, (int)MotorInput3, (int)MotorInput4);
  
  static uint32_t lastTelemetry = 0;
  if (millis() - lastTelemetry >= 50) {
    lastTelemetry = millis();
    sendTelemetry();
  }

  // ================= DEBUG =================
  uint32_t now = millis();
  if (now - lastPrint >= PRINT_PERIOD_MS) {
    lastPrint = now;
    Serial.print("THR:"); Serial.print(receiverThrottle);
    Serial.print(" | ROLL:"); Serial.print(Roll, 1);
    Serial.print(" | PITCH:"); Serial.print(Pitch, 1);
    Serial.print(" | YAW:"); Serial.print(droneYaw, 1);

    Serial.print(" | PID: ");
    Serial.print("PAngleR="); Serial.print(PAngleRoll, 2); Serial.print(" ");
    Serial.print("IAngleR="); Serial.print(IAngleRoll, 2); Serial.print(" ");
    Serial.print("DAngleR="); Serial.print(DAngleRoll, 2); Serial.print(" ");
    Serial.print("PAngleP="); Serial.print(PAnglePitch, 2); Serial.print(" ");
    Serial.print("IAngleP="); Serial.print(IAnglePitch, 2); Serial.print(" ");
    Serial.print("DAngleP="); Serial.print(DAnglePitch, 2); Serial.print(" ");
    Serial.print("PRateR=");  Serial.print(PRateRoll, 2);   Serial.print(" ");
    Serial.print("IRateR=");  Serial.print(IRateRoll, 2);   Serial.print(" ");
    Serial.print("DRateR=");  Serial.print(DRateRoll, 2);   Serial.print(" ");
    Serial.print("PRateP=");  Serial.print(PRatePitch, 2);  Serial.print(" ");
    Serial.print("IRateP=");  Serial.print(IRatePitch, 2);  Serial.print(" ");
    Serial.print("DRateP=");  Serial.print(DRatePitch, 2);  Serial.print(" ");
    Serial.print("PRateY=");  Serial.print(PRateYaw, 2);    Serial.print(" ");
    Serial.print("IRateY=");  Serial.print(IRateYaw, 2);    Serial.print(" ");
    Serial.print("DRateY=");  Serial.print(DRateYaw, 2);

    Serial.print(" || M1:"); Serial.print(MotorInput1, 0);
    Serial.print(" | M2:"); Serial.print(MotorInput2, 0);
    Serial.print(" | M3:"); Serial.print(MotorInput3, 0);
    Serial.print(" | M4:"); Serial.println(MotorInput4, 0);
  }

  while ((uint32_t)(micros() - LoopTimer) < LOOP_PERIOD_US) {}
  LoopTimer = micros();
}
