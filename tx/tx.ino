#include <WiFi.h>
#include <esp_now.h>

// ================== RX MAC ==================
uint8_t receiverMac[] = { 0x44, 0x1D, 0x64, 0xFA, 0xC0, 0xE4 };

// ================== PIN ESP32 DEVKIT V1 ==================
#define JOY_ADC1 33
#define JOY_ADC2 32
#define JOY_ADC3 35

#define JOY_BTN 4

// ================== UART2 (chân 18, 19) ==================
#define UART2_TX 19
#define UART2_RX 18

// ================== XUNG ESC ==================
#define PULSE_OFF 1000
#define PULSE_IDLE 1000
#define PULSE_START 1100
#define PULSE_MAX 1800

// ================== JOYSTICK ==================
#define ADC_CENTER_LOW 500
#define ADC_CENTER_HIGH 2500
#define REVERSE_ADC1 false
#define THROTTLE_UP_STEP_US 5
#define THROTTLE_DOWN_STEP_US 8
#define THROTTLE_UPDATE_MS 20
#define SEND_PERIOD_MS 20
#define PRINT_PERIOD_MS 100
#define NEUTRAL_CONFIRM_MS 100

enum ArmState : uint8_t {
  WAIT_FIRST_PUSH = 0,
  FIRST_PUSH_HOLD = 1,
  NORMAL_CONTROL = 2
};

ArmState armState = WAIT_FIRST_PUSH;

// ================== DATA ==================
typedef struct __attribute__((packed)) {
  uint8_t magic;
  uint8_t cmd;
  uint16_t pulse_us;  // giá trị ga đã xử lý
  uint16_t adc1;      // raw throttle
  uint16_t adc2;      // raw roll
  uint16_t adc3;      // raw pitch     ← thêm
  uint8_t button;
  uint8_t state;
  uint32_t seq;
} ControlData;

ControlData data;

// ================= TELEMETRY RX =================
typedef struct __attribute__((packed)) {
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

TelemetryData telemetry;

// ================== PID DATA ==================
typedef struct __attribute__((packed)) {
  uint8_t magic;
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

uint16_t targetPulse = PULSE_OFF;
uint16_t sendPulse = PULSE_OFF;
uint16_t holdPulse = PULSE_START;

uint32_t seqCount = 0;
uint32_t lastSend = 0;
uint32_t lastPrint = 0;
uint32_t lastThrottleUpdate = 0;
uint32_t neutralStart = 0;

String serialBuffer = "";

// ================== CALLBACK ==================
void onDataSent(const wifi_tx_info_t *tx_info,
                esp_now_send_status_t status)
{
    if (status == ESP_NOW_SEND_SUCCESS) {
        // gửi thành công
    } else {
        // gửi thất bại
    }
}

// ================== HÀM PHỤ ==================
void printMac(const uint8_t *mac) {
  for (int i = 0; i < 6; i++) {
    if (mac[i] < 16) Serial.print("0");
    Serial.print(mac[i], HEX);
    if (i < 5) Serial.print(":");
  }
}

bool isExpectedSender(const uint8_t *mac) {
  for (int i = 0; i < 6; i++) {
    if (mac[i] != receiverMac[i]) return false;
  }
  return true;
}

void onTelemetryRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
  if (info == nullptr || incomingData == nullptr || len <= 0) {
    return;
  }

  // Chỉ nhận gói từ đúng drone/RX
  if (!isExpectedSender(info->src_addr)) {
    Serial.print("IGNORE PACKET FROM ");
    printMac(info->src_addr);
    Serial.println();
    return;
  }

  if (len != sizeof(TelemetryData)) {
    Serial.print("SIZE MISMATCH FROM ");
    printMac(info->src_addr);
    Serial.print(" | RX len = ");
    Serial.print(len);
    Serial.print(" | expected = ");
    Serial.println(sizeof(TelemetryData));
    return;
  }

  memcpy(&telemetry, incomingData, sizeof(TelemetryData));

  // Chuỗi telemetry đầy đủ
  char buffer[512];
  sprintf(buffer,
          "ROLL=%.2f PITCH=%.2f YAW=%.2f "
          "GYRO_R=%.2f GYRO_P=%.2f GYRO_Y=%.2f "
          "PAngleR=%.2f IAngleR=%.2f DAngleR=%.2f "
          "PAngleP=%.2f IAngleP=%.2f DAngleP=%.2f "
          "PRateR=%.2f IRateR=%.2f DRateR=%.2f "
          "PRateP=%.2f IRateP=%.2f DRateP=%.2f "
          "PRateY=%.2f IRateY=%.2f DRateY=%.2f "
          "M1=%u M2=%u M3=%u M4=%u CNT=%u\n",
          telemetry.roll, telemetry.pitch, telemetry.yaw,
          telemetry.gyroRoll, telemetry.gyroPitch, telemetry.gyroYaw,
          telemetry.PAngleRoll, telemetry.IAngleRoll, telemetry.DAngleRoll,
          telemetry.PAnglePitch, telemetry.IAnglePitch, telemetry.DAnglePitch,
          telemetry.PRateRoll, telemetry.IRateRoll, telemetry.DRateRoll,
          telemetry.PRatePitch, telemetry.IRatePitch, telemetry.DRatePitch,
          telemetry.PRateYaw, telemetry.IRateYaw, telemetry.DRateYaw,
          telemetry.motor1, telemetry.motor2, telemetry.motor3, telemetry.motor4,
          telemetry.counter);

  // Xuất ra Serial (USB)
  Serial.print(buffer);

  // Xuất ra UART2 (chân 18, 19)
  Serial2.print(buffer);
}

// ================== ADC ==================
uint16_t readAdcAvg(uint8_t pin, int samples = 8) {
  uint32_t sum = 0;
  for (int i = 0; i < samples; i++) {
    sum += analogRead(pin);
    delayMicroseconds(300);
  }
  return sum / samples;
}

bool isThrottleCenter(uint16_t adc) {
  return adc >= ADC_CENTER_LOW && adc <= ADC_CENTER_HIGH;
}

uint16_t updateThrottleByHold(uint16_t adc) {
  uint32_t now = millis();
  if (now - lastThrottleUpdate < THROTTLE_UPDATE_MS) return holdPulse;
  lastThrottleUpdate = now;

  if (isThrottleCenter(adc)) return holdPulse;

  bool stickUp = REVERSE_ADC1 ? (adc < ADC_CENTER_LOW) : (adc > ADC_CENTER_HIGH);
  if (stickUp) {
    holdPulse += THROTTLE_UP_STEP_US;
  } else {
    if (holdPulse > PULSE_IDLE) holdPulse -= THROTTLE_DOWN_STEP_US;
  }
  holdPulse = constrain(holdPulse, PULSE_IDLE, PULSE_MAX);
  return holdPulse;
}

// ================== GỬI LỆNH ==================
void sendControl(uint8_t cmd, uint16_t pulse, uint16_t adc1, uint16_t adc2, uint16_t adc3, bool buttonPressed) {
  data.magic = 0xA5;
  data.cmd = cmd;
  data.pulse_us = pulse;
  data.adc1 = adc1;
  data.adc2 = adc2;
  data.adc3 = adc3;  // thêm
  data.button = buttonPressed;
  data.state = armState;
  data.seq = seqCount++;
  esp_now_send(receiverMac, (uint8_t *)&data, sizeof(data));
}

void sendPID(const PIDData &pid) {
  esp_now_send(receiverMac, (uint8_t *)&pid, sizeof(pid));
  Serial.println("PID sent to drone");
  Serial2.println("PID sent to drone");
}

void parseAndSendPID(String line) {
  if (!line.startsWith("PID:")) return;
  line.remove(0, 4);

  float values[15];
  int idx = 0;

  char buf[256];
  line.toCharArray(buf, sizeof(buf));

  char *token = strtok(buf, ",");
  while (token != NULL && idx < 15) {
    values[idx++] = atof(token);
    token = strtok(NULL, ",");
  }

  if (idx != 15) {
    Serial.println("PID: Invalid number of values (need 15)");
    Serial2.println("PID: Invalid number of values (need 15)");
    return;
  }

  PIDData pid;
  pid.magic = 0xA6;
  pid.PAngleRoll = values[0];
  pid.IAngleRoll = values[1];
  pid.DAngleRoll = values[2];
  pid.PAnglePitch = values[3];
  pid.IAnglePitch = values[4];
  pid.DAnglePitch = values[5];
  pid.PRateRoll = values[6];
  pid.IRateRoll = values[7];
  pid.DRateRoll = values[8];
  pid.PRatePitch = values[9];
  pid.IRatePitch = values[10];
  pid.DRatePitch = values[11];
  pid.PRateYaw = values[12];
  pid.IRateYaw = values[13];
  pid.DRateYaw = values[14];

  sendPID(pid);
}

// ================== SETUP ==================
void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, UART2_RX, UART2_TX);  // RX=19, TX=18

  pinMode(JOY_BTN, INPUT_PULLUP);
  analogReadResolution(12);
  analogSetPinAttenuation(JOY_ADC1, ADC_11db);
  analogSetPinAttenuation(JOY_ADC2, ADC_11db);
  analogSetPinAttenuation(JOY_ADC3, ADC_11db);

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);

  Serial.print("TX MAC: ");
  Serial.println(WiFi.macAddress());
  Serial2.print("TX MAC: ");
  Serial2.println(WiFi.macAddress());

  Serial.print("Expected RX MAC: ");
  printMac(receiverMac);
  Serial.println();
  Serial2.print("Expected RX MAC: ");
  // In MAC ra UART2 (có thể dùng printMac nhưng cần hàm riêng cho Serial2 hoặc làm tương tự)
  for (int i = 0; i < 6; i++) {
    if (receiverMac[i] < 16) Serial2.print("0");
    Serial2.print(receiverMac[i], HEX);
    if (i < 5) Serial2.print(":");
  }
  Serial2.println();

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW FAIL");
    Serial2.println("ESP-NOW FAIL");
    while (1)
      ;
  }

  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onTelemetryRecv);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, receiverMac, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("ADD PEER FAIL");
    Serial2.println("ADD PEER FAIL");
    while (1)
      ;
  }

  Serial.println("TX READY");
  Serial2.println("TX READY");
  Serial.print("sizeof(ControlData) = ");
  Serial.println(sizeof(ControlData));
  Serial.print("sizeof(PIDData) = ");
  Serial.println(sizeof(PIDData));
  Serial.print("sizeof(TelemetryData) = ");
  Serial.println(sizeof(TelemetryData));
  Serial2.print("sizeof(ControlData) = ");
  Serial2.println(sizeof(ControlData));
  Serial2.print("sizeof(PIDData) = ");
  Serial2.println(sizeof(PIDData));
  Serial2.print("sizeof(TelemetryData) = ");
  Serial2.println(sizeof(TelemetryData));
}

// ================== LOOP ==================
void loop() {
  uint32_t now = millis();

  // Đọc lệnh từ Serial (từ Python)
  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\n') {
      serialBuffer.trim();
      if (serialBuffer.startsWith("PID:")) {
        parseAndSendPID(serialBuffer);
      }
      serialBuffer = "";
    } else if (c != '\r') {
      serialBuffer += c;
    }
  }

  // Xử lý joystick
  uint16_t adc1 = readAdcAvg(JOY_ADC1, 4);
  uint16_t adc2 = readAdcAvg(JOY_ADC2, 4);
  uint16_t adc3 = readAdcAvg(JOY_ADC3, 4);
  bool buttonPressed = digitalRead(JOY_BTN) == LOW;
  bool center = isThrottleCenter(adc1);
  bool deflected = !center;

  if (buttonPressed) {
    armState = WAIT_FIRST_PUSH;
    targetPulse = PULSE_OFF;
    sendPulse = PULSE_OFF;
    holdPulse = PULSE_START;
    neutralStart = 0;
  } else {
    switch (armState) {
      case WAIT_FIRST_PUSH:
        targetPulse = PULSE_OFF;
        sendPulse = PULSE_OFF;
        if (deflected) {
          armState = FIRST_PUSH_HOLD;
          targetPulse = PULSE_START;
          sendPulse = PULSE_START;
          holdPulse = PULSE_START;
          neutralStart = 0;
        }
        break;

      case FIRST_PUSH_HOLD:
        targetPulse = PULSE_START;
        sendPulse = PULSE_START;
        holdPulse = PULSE_START;
        if (center) {
          if (neutralStart == 0) neutralStart = now;
          if (now - neutralStart >= NEUTRAL_CONFIRM_MS) {
            armState = NORMAL_CONTROL;
            targetPulse = holdPulse;
            sendPulse = holdPulse;
          }
        } else {
          neutralStart = 0;
        }
        break;

      case NORMAL_CONTROL:
        targetPulse = updateThrottleByHold(adc1);
        sendPulse = targetPulse;
        break;
    }
  }

  if (now - lastSend >= SEND_PERIOD_MS) {
    lastSend = now;
    uint8_t cmd = buttonPressed ? 0 : 1;
    sendControl(cmd, sendPulse, adc1, adc2, adc3, buttonPressed);
  }

  if (now - lastPrint >= PRINT_PERIOD_MS) {
    lastPrint = now;
    String line = "ADC1=" + String(adc1) + " ADC2=" + String(adc2) + " SEND_US=" + String(sendPulse) + " STATE=" + String((uint8_t)armState) + "\n";
    Serial.print(line);
    Serial2.print(line);
  }
}