#include <WiFi.h>
#include <WiFiUdp.h>

// ---------- WiFi credentials ----------
const char* ssid     = "MANGO";
const char* password = "remotamente";

// ---------- OSC / UDP settings ----------
const unsigned int localPort = 8000;   // UDP port to listen on
WiFiUDP udp;

// ---------- Output settings ----------
const int ledPin = 2;     // onboard LED, used for debug/visual feedback
const int dimmerPin = 15; // D15, goes to the DEWIN PWM dimmer module

const int ledChannel = 0;
const int dimmerChannel = 1;
const int ledcFreq = 5000;
const int ledcResolution = 8; // 0-255

// buffer for incoming packets
char packetBuffer[512];

void setup() {
  Serial.begin(115200);
  delay(500);

  // Setup PWM on both output pins (ESP32 core >= 3.x uses ledcAttach)
  #if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcAttach(ledPin, ledcFreq, ledcResolution);
    ledcAttach(dimmerPin, ledcFreq, ledcResolution);
  #else
    ledcSetup(ledChannel, ledcFreq, ledcResolution);
    ledcAttachPin(ledPin, ledChannel);
    ledcSetup(dimmerChannel, ledcFreq, ledcResolution);
    ledcAttachPin(dimmerPin, dimmerChannel);
  #endif

  connectToWiFi();

  udp.begin(localPort);
  Serial.printf("Listening for OSC on UDP port %u\n", localPort);
}

void loop() {
  int packetSize = udp.parsePacket();
  if (packetSize > 0 && packetSize < (int)sizeof(packetBuffer)) {
    int len = udp.read(packetBuffer, sizeof(packetBuffer) - 1);
    if (len > 0) {
      packetBuffer[len] = 0;
      handleOSCPacket(packetBuffer, len);
    }
  }

  // basic reconnect handling
  if (WiFi.status() != WL_CONNECTED) {
    connectToWiFi();
  }
}

void connectToWiFi() {
  Serial.printf("Connecting to WiFi '%s'...\n", ssid);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 20000) {
    delay(250);
    Serial.print(".");
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("Connected. IP address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("WiFi connection failed, will retry in loop().");
  }
}

// ---------- Minimal OSC parsing ----------
// Reads an OSC address pattern + type tag string + one numeric argument
// (int32 or float32), which covers most controllers (TouchOSC, Max, etc.)

void handleOSCPacket(const char* data, int len) {
  int pos = 0;

  // 1. Read address pattern (null-terminated, padded to 4 bytes)
  String address = readOSCString(data, len, pos);
  if (address.length() == 0) return;

  // 2. Read type tag string, e.g. ",f" or ",i"
  String typeTags = readOSCString(data, len, pos);
  if (typeTags.length() < 2 || typeTags[0] != ',') return;

  char typeChar = typeTags[1];

  if (address == "/dimmer") {
    float value = 0.0f;

    if (typeChar == 'f' && pos + 4 <= len) {
      value = readOSCFloat(data, pos);
    } else if (typeChar == 'i' && pos + 4 <= len) {
      value = (float)readOSCInt(data, pos);
    } else {
      Serial.println("Unsupported /dimmer argument type");
      return;
    }

    setDimmer(value);
  }
}

// Reads a null-terminated OSC string, advances pos past its 4-byte padding
String readOSCString(const char* data, int len, int &pos) {
  int start = pos;
  while (pos < len && data[pos] != '\0') pos++;
  if (pos >= len) return String();

  String result(data + start, pos - start);

  // consume the null terminator plus padding to next multiple of 4
  pos++; // skip null
  while (pos % 4 != 0) pos++;

  return result;
}

float readOSCFloat(const char* data, int &pos) {
  uint32_t raw =
    ((uint32_t)(uint8_t)data[pos]     << 24) |
    ((uint32_t)(uint8_t)data[pos + 1] << 16) |
    ((uint32_t)(uint8_t)data[pos + 2] << 8)  |
    ((uint32_t)(uint8_t)data[pos + 3]);
  pos += 4;

  float value;
  memcpy(&value, &raw, sizeof(value));
  return value;
}

int32_t readOSCInt(const char* data, int &pos) {
  int32_t value =
    ((int32_t)(uint8_t)data[pos]     << 24) |
    ((int32_t)(uint8_t)data[pos + 1] << 16) |
    ((int32_t)(uint8_t)data[pos + 2] << 8)  |
    ((int32_t)(uint8_t)data[pos + 3]);
  pos += 4;
  return value;
}

// ---------- LED control ----------
// Accepts either 0.0-1.0 (typical OSC float range) or 0-255 (int) and
// scales accordingly.

void setDimmer(float value) {
  int duty;

  if (value <= 1.0f && value >= 0.0f) {
    duty = (int)(value * 255.0f);
  } else {
    duty = (int)value;
  }

  duty = constrain(duty, 0, 255);

  #if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcWrite(ledPin, duty);
    ledcWrite(dimmerPin, duty);
  #else
    ledcWrite(ledChannel, duty);
    ledcWrite(dimmerChannel, duty);
  #endif

  //Serial.printf("/dimmer -> %.3f (duty %d)\n", value, duty);
}
