#include <ArduinoJson.h>
#include <WebSocketsClient.h>
#include <WiFi.h>
#include <Wire.h>

// Network settings
constexpr char WIFI_SSID[] = "your-ssid";
constexpr char WIFI_PASSWORD[] = "your-password";
constexpr char SERVER_IP[] = "192.168.x.x";
constexpr uint16_t SERVER_PORT = 8080;
constexpr char SERVER_PATH[] = "/";

// Hardware settings
constexpr uint8_t MPU6050_ADDRESS = 0x68;
constexpr uint8_t I2C_SDA_PIN = 21;
constexpr uint8_t I2C_SCL_PIN = 22;
constexpr uint8_t JOYSTICK_X_PIN = 34;
constexpr uint8_t JOYSTICK_Y_PIN = 35;
constexpr uint8_t JOYSTICK_BUTTON_PIN = 32;

// Timing
constexpr unsigned long TELEMETRY_INTERVAL_MS = 125;  // 8 Hz
constexpr unsigned long WIFI_RECONNECT_INTERVAL_MS = 5000;

// Movement tuning
constexpr float MAX_TILT_DEGREES = 45.0f;
constexpr float RAD_TO_DEGREES = 57.2957795131f;

// Joystick tuning for a 12-bit ESP32 ADC
constexpr int ADC_CENTER = 2048;
constexpr long DEADZONE_RADIUS_SQ = 1200L * 1200L; // Larger deadzone, squared (800 out of 2048)

WebSocketsClient webSocket;

bool webSocketConnected = false;
unsigned long lastTelemetryMs = 0;
unsigned long lastWifiReconnectMs = 0;
float lastAimAngle = 0.0f;

struct Movement {
  float vx;
  float vy;
};

float clampFloat(float value, float minValue, float maxValue) {
  if (value < minValue) {
    return minValue;
  }
  if (value > maxValue) {
    return maxValue;
  }
  return value;
}

void writeMpuRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission(true);
}

bool readMpuAccelRaw(int16_t &ax, int16_t &ay, int16_t &az) {
  constexpr uint8_t ACCEL_XOUT_H = 0x3B;
  constexpr uint8_t ACCEL_BYTES = 6;

  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(ACCEL_XOUT_H);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  if (Wire.requestFrom(MPU6050_ADDRESS, ACCEL_BYTES, true) != ACCEL_BYTES) {
    return false;
  }

  ax = static_cast<int16_t>((Wire.read() << 8) | Wire.read());
  ay = static_cast<int16_t>((Wire.read() << 8) | Wire.read());
  az = static_cast<int16_t>((Wire.read() << 8) | Wire.read());
  return true;
}

bool setupMpu6050() {
  constexpr uint8_t PWR_MGMT_1 = 0x6B;
  constexpr uint8_t ACCEL_CONFIG = 0x1C;

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(400000);

  writeMpuRegister(PWR_MGMT_1, 0x00);  // Wake the MPU-6050.
  writeMpuRegister(ACCEL_CONFIG, 0x00); // +/- 2g full-scale range.

  int16_t ax = 0;
  int16_t ay = 0;
  int16_t az = 0;
  return readMpuAccelRaw(ax, ay, az);
}

Movement readMovement() {
  int16_t rawAx = 0;
  int16_t rawAy = 0;
  int16_t rawAz = 0;

  if (!readMpuAccelRaw(rawAx, rawAy, rawAz)) {
    return {0.0f, 0.0f};
  }

  const float ax = static_cast<float>(rawAx);
  const float ay = static_cast<float>(rawAy);
  const float az = static_cast<float>(rawAz);

  const float pitch = atan2f(-ax, sqrtf((ay * ay) + (az * az))) * RAD_TO_DEGREES;
  const float roll = atan2f(ay, az) * RAD_TO_DEGREES;

  // Normalize tilt into velocity vectors, then clamp so extreme tilt never exceeds [-1, 1].
  // Mirror X and Y movement independently
  const float vx = clampFloat(-roll / MAX_TILT_DEGREES, -1.0f, 1.0f);
  const float vy = clampFloat(-pitch / MAX_TILT_DEGREES, -1.0f, 1.0f);

  Serial.printf("vx=%f, vy=%f\n", vx, vy);
  return {vx, vy};
}

float readAimAngle() {
  // Oversample to reduce ESP32 ADC noise
  long sumX = 0;
  long sumY = 0;
  const int numSamples = 8;
  for (int i = 0; i < numSamples; i++) {
    sumX += analogRead(JOYSTICK_X_PIN);
    sumY += analogRead(JOYSTICK_Y_PIN);
  }
  const int rawX = sumX / numSamples;
  const int rawY = sumY / numSamples;

  // Invert Y so up is positive.
  // Mirror X so left/right match the hardware orientation.
  long dx = ADC_CENTER - rawX;
  long dy = ADC_CENTER - rawY;

  // Axial deadzone (snap to perfect horizontal/vertical if close to axis)
  constexpr long AXIS_SNAP_THRESHOLD = 300;
  if (dx > -AXIS_SNAP_THRESHOLD && dx < AXIS_SNAP_THRESHOLD) dx = 0;
  if (dy > -AXIS_SNAP_THRESHOLD && dy < AXIS_SNAP_THRESHOLD) dy = 0;

  // Use a proper radial deadzone to avoid spring-back wobble and noise
  if ((dx * dx + dy * dy) > DEADZONE_RADIUS_SQ) {
    lastAimAngle = atan2f(static_cast<float>(-dy), static_cast<float>(-dx));
  }

  // Inside the deadzone, keep the previous aim angle so aim persists after release.
  return lastAimAngle;
}

bool readShooting() {
  return digitalRead(JOYSTICK_BUTTON_PIN) == LOW;
}

void webSocketEvent(WStype_t type, uint8_t *payload, size_t length) {
  (void)payload;
  (void)length;

  switch (type) {
    case WStype_CONNECTED:
      webSocketConnected = true;
      break;
    case WStype_DISCONNECTED:
      webSocketConnected = false;
      break;
    default:
      break;
  }
}

void connectWifiIfNeeded() {
  if (WiFi.status() == WL_CONNECTED) {
    return;
  }

  const unsigned long now = millis();
  if (now - lastWifiReconnectMs >= WIFI_RECONNECT_INTERVAL_MS) {
    lastWifiReconnectMs = now;
    WiFi.disconnect();
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  }
}

void sendTelemetry() {
  const Movement movement = readMovement();
  const float aimAngle = readAimAngle();
  const bool isShooting = readShooting();

  StaticJsonDocument<128> doc;
  doc["vx"] = movement.vx;
  doc["vy"] = movement.vy;
  doc["aim_angle"] = aimAngle;
  doc["is_shooting"] = isShooting;

  char payload[128];
  const size_t payloadLength = serializeJson(doc, payload, sizeof(payload));

  if (webSocketConnected && payloadLength > 0) {
    webSocket.sendTXT(payload, payloadLength);
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(JOYSTICK_BUTTON_PIN, INPUT_PULLUP);
  analogReadResolution(12);

  const bool mpuReady = setupMpu6050();
  if (!mpuReady) {
    Serial.println("MPU-6050 not detected. Check wiring and address.");
  }

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  webSocket.begin(SERVER_IP, SERVER_PORT, SERVER_PATH);
  webSocket.onEvent(webSocketEvent);
  webSocket.setReconnectInterval(5000);
  webSocket.enableHeartbeat(15000, 3000, 2);
}

void loop() {
  connectWifiIfNeeded();
  webSocket.loop();

  const unsigned long now = millis();
  if (now - lastTelemetryMs >= TELEMETRY_INTERVAL_MS) {
    lastTelemetryMs = now;
    sendTelemetry();
  }
}
