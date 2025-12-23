#include <Arduino.h>
#include <FastLED.h>
#include <WiFi.h>
#include <WiFiUdp.h>

// ---------------------------------------------------------------------------
// WiFi + OSC configuration
// ---------------------------------------------------------------------------
#define WIFI_SSID "n2048"
#define WIFI_PASS "16377240"
#define OSC_PORT 9000
#define OSC_ADDRESS "/leds"
#define ACK_PORT 9001
#define ACK_INTERVAL_MS 500
#define DEBUG_OSC 1

// ---------------------------------------------------------------------------
// LED configuration
// ---------------------------------------------------------------------------
#define NUM_COLUMNS 12
#define NUM_ROWS 19
#define NUM_LEDS (NUM_COLUMNS * NUM_ROWS)

#define DATA_PIN_2 2
#define DATA_PIN_3 3
#define DATA_PIN_4 4
#define DATA_PIN_5 5
#define DATA_PIN_6 6
#define DATA_PIN_7 7
#define LED_TYPE WS2811
#define COLOR_ORDER GRB
#define DEFAULT_BRIGHTNESS 64  // 0-255

#define PAYLOAD_SIZE (NUM_LEDS * 3)
#define TEST_TIMEOUT_MS 2000
#define POWER_LIMIT_MA 3000

CRGB leds[NUM_LEDS];
uint8_t payload[PAYLOAD_SIZE];
CRGBPalette16 currentPalette = RainbowColors_p;
TBlendType currentBlending = LINEARBLEND;
uint8_t paletteIndex = 0;

WiFiUDP Udp;
uint32_t lastFrameMs = 0;
uint32_t lastPatternMs = 0;
#if DEBUG_OSC
uint32_t lastAckMs = 0;
#else
uint32_t lastAckMs = 0;
#endif
#if DEBUG_OSC
uint32_t lastDebugMs = 0;
uint32_t oscPackets = 0;
uint32_t oscErrors = 0;
uint32_t lastPacketSize = 0;
uint32_t lastBlobSize = 0;
uint32_t lastOscMs = 0;
#endif

static void changePalettePeriodically();
static void fillLedsFromPalette(uint8_t colorIndex);

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
static inline uint16_t mapLedIndex(uint16_t logicalIndex) {
  uint16_t col = logicalIndex / NUM_ROWS;
  uint16_t row = logicalIndex % NUM_ROWS;  // row 0 = bottom
  uint16_t physicalRow = row;
  if (col % 2 == 1) {
    physicalRow = (NUM_ROWS - 1) - row;
  }
  return col * NUM_ROWS + physicalRow;
}

static void applyFrameToLeds(uint16_t ledCount) {
  uint16_t idx = 0;
  for (uint16_t i = 0; i < NUM_LEDS; ++i) {
    leds[i] = CRGB::Black;
  }
  for (uint16_t i = 0; i < ledCount; ++i) {
    uint8_t r = payload[idx++];
    uint8_t g = payload[idx++];
    uint8_t b = payload[idx++];
    uint16_t physical = mapLedIndex(i);
    if (physical < NUM_LEDS) {
      leds[physical].setRGB(r, g, b);
    }
  }
  FastLED.show();
}

static void runTestPattern() {
  const uint32_t now = millis();
  if (now - lastPatternMs < 20) {
    return;
  }
  lastPatternMs = now;
  changePalettePeriodically();
  paletteIndex++;
  fillLedsFromPalette(paletteIndex);
  FastLED.show();
}

static size_t oscAlign4(size_t v) {
  return (v + 3) & ~static_cast<size_t>(3);
}

static size_t writeOscString(uint8_t *buffer, size_t offset, size_t maxLen, const char *text) {
  const size_t len = strlen(text);
  if (offset + len + 1 > maxLen) {
    return 0;
  }
  memcpy(buffer + offset, text, len);
  buffer[offset + len] = '\0';
  size_t padded = oscAlign4(offset + len + 1);
  if (padded > maxLen) {
    return 0;
  }
  size_t i = offset + len + 1;
  while (i < padded && i < maxLen) {
    buffer[i++] = '\0';
  }
  return padded;
}

static void sendOscAck(const IPAddress &remoteIp) {
  const uint32_t now = millis();
  if (now - lastAckMs < ACK_INTERVAL_MS) {
    return;
  }
  lastAckMs = now;
  uint8_t buffer[32];
  size_t offset = 0;
  offset = writeOscString(buffer, offset, sizeof(buffer), "/leds_ack");
  if (offset == 0) {
    return;
  }
  offset = writeOscString(buffer, offset, sizeof(buffer), ",");
  if (offset == 0) {
    return;
  }
  Udp.beginPacket(remoteIp, ACK_PORT);
  Udp.write(buffer, offset);
  Udp.endPacket();
}

static bool readOscString(const uint8_t *data, size_t len, size_t *offset, const char *expected) {
  if (*offset >= len) {
    return false;
  }
  size_t start = *offset;
  size_t i = start;
  while (i < len && data[i] != '\0') {
    i++;
  }
  if (i >= len) {
    return false;
  }
  size_t strLen = i - start;
  if (expected != nullptr) {
    size_t expectedLen = strlen(expected);
    if (expectedLen != strLen || memcmp(data + start, expected, strLen) != 0) {
      return false;
    }
  }
  i = oscAlign4(i + 1);
  if (i > len) {
    return false;
  }
  *offset = i;
  return true;
}

static bool handleOscPacket(const uint8_t *data, size_t len, const IPAddress &remoteIp) {
#if DEBUG_OSC
  lastPacketSize = static_cast<uint32_t>(len);
#endif
  size_t offset = 0;
  if (!readOscString(data, len, &offset, OSC_ADDRESS)) {
#if DEBUG_OSC
    oscErrors++;
#endif
    return false;
  }

  // Type tags string starts with ',' and must include a blob ('b') as first arg.
  if (offset >= len || data[offset] != ',') {
#if DEBUG_OSC
    oscErrors++;
#endif
    return false;
  }
  size_t tagStart = offset;
  size_t tagEnd = tagStart;
  while (tagEnd < len && data[tagEnd] != '\0') {
    tagEnd++;
  }
  if (tagEnd >= len) {
#if DEBUG_OSC
    oscErrors++;
#endif
    return false;
  }
  if ((tagEnd - tagStart) < 2 || data[tagStart + 1] != 'b') {
#if DEBUG_OSC
    oscErrors++;
#endif
    return false;
  }
  offset = oscAlign4(tagEnd + 1);
  if (offset + 4 > len) {
#if DEBUG_OSC
    oscErrors++;
#endif
    return false;
  }

  uint32_t blobLen = (static_cast<uint32_t>(data[offset]) << 24) |
                     (static_cast<uint32_t>(data[offset + 1]) << 16) |
                     (static_cast<uint32_t>(data[offset + 2]) << 8) |
                     static_cast<uint32_t>(data[offset + 3]);
  offset += 4;
  if (blobLen > PAYLOAD_SIZE || offset + blobLen > len) {
#if DEBUG_OSC
    oscErrors++;
#endif
    return false;
  }

  memcpy(payload, data + offset, blobLen);
  applyFrameToLeds(static_cast<uint16_t>(blobLen / 3));
  lastFrameMs = millis();
#if DEBUG_OSC
  oscPackets++;
  lastBlobSize = blobLen;
  lastOscMs = lastFrameMs;
  if (oscPackets == 1 && blobLen >= 6) {
    Serial.print("First payload bytes: ");
    for (int i = 0; i < 6; ++i) {
      if (i > 0) {
        Serial.print(' ');
      }
      Serial.print(payload[i]);
    }
    Serial.println();
  }
#endif
  sendOscAck(remoteIp);
  return true;
}

static void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(200);
    if (millis() - start > 15000) {
      break;
    }
  }
}

// ---------------------------------------------------------------------------
// Setup / loop
// ---------------------------------------------------------------------------
void setup() {
  delay(1000);
  Serial.begin(115200);
  Serial.setTimeout(0);

  connectWiFi();
  if (WiFi.status() == WL_CONNECTED) {
    Udp.begin(OSC_PORT);
  }
#if DEBUG_OSC
  Serial.print("WiFi status: ");
  Serial.println(WiFi.status() == WL_CONNECTED ? "connected" : "not connected");
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
    Serial.print("OSC port: ");
    Serial.println(OSC_PORT);
    Serial.print("OSC address: ");
    Serial.println(OSC_ADDRESS);
  }
#endif

  FastLED.addLeds<LED_TYPE, DATA_PIN_2, COLOR_ORDER>(leds, NUM_LEDS)
      .setCorrection(TypicalLEDStrip);
  FastLED.addLeds<LED_TYPE, DATA_PIN_3, COLOR_ORDER>(leds, NUM_LEDS);
  FastLED.addLeds<LED_TYPE, DATA_PIN_4, COLOR_ORDER>(leds, NUM_LEDS);
  FastLED.addLeds<LED_TYPE, DATA_PIN_5, COLOR_ORDER>(leds, NUM_LEDS);
  FastLED.addLeds<LED_TYPE, DATA_PIN_6, COLOR_ORDER>(leds, NUM_LEDS);
  FastLED.addLeds<LED_TYPE, DATA_PIN_7, COLOR_ORDER>(leds, NUM_LEDS);
  FastLED.setBrightness(DEFAULT_BRIGHTNESS);
  FastLED.setMaxPowerInVoltsAndMilliamps(5, POWER_LIMIT_MA);
  FastLED.clear(true);
  leds[0] = CRGB(255, 255, 255);
  FastLED.show();
  delay(200);
  FastLED.clear(true);
}

void loop() {
  int packetSize = Udp.parsePacket();
  if (packetSize > 0) {
#if DEBUG_OSC
    if (oscPackets == 0 && oscErrors == 0) {
      Serial.print("Remote IP: ");
      Serial.print(Udp.remoteIP());
      Serial.print(" port: ");
      Serial.println(Udp.remotePort());
    }
#endif
    IPAddress remoteIp = Udp.remoteIP();
    static uint8_t oscBuffer[PAYLOAD_SIZE + 64];
    if (packetSize > static_cast<int>(sizeof(oscBuffer))) {
      while (packetSize-- > 0) {
        Udp.read();
      }
#if DEBUG_OSC
      oscErrors++;
#endif
    } else {
      int len = Udp.read(oscBuffer, sizeof(oscBuffer));
      if (len > 0) {
        handleOscPacket(oscBuffer, static_cast<size_t>(len), remoteIp);
      }
    }
  }

  const uint32_t now = millis();
  if (now - lastFrameMs > TEST_TIMEOUT_MS) {
    runTestPattern();
  }

#if DEBUG_OSC
  if (now - lastDebugMs > 1000) {
    lastDebugMs = now;
    Serial.print("OSC packets=");
    Serial.print(oscPackets);
    Serial.print(" errors=");
    Serial.print(oscErrors);
    Serial.print(" last_packet=");
    Serial.print(lastPacketSize);
    Serial.print(" last_blob=");
    Serial.print(lastBlobSize);
    Serial.print(" last_ms=");
    Serial.println(lastOscMs);
  }
#endif
}

// ---------------------------------------------------------------------------
// Palette helpers (based on FastLED ColorPalette example)
// ---------------------------------------------------------------------------
static void fillLedsFromPalette(uint8_t colorIndex) {
  for (int i = 0; i < NUM_LEDS; ++i) {
    leds[i] = ColorFromPalette(currentPalette, colorIndex, 255, currentBlending);
    colorIndex += 3;
  }
}

static void changePalettePeriodically() {
  uint8_t secondHand = (millis() / 1000) % 60;
  static uint8_t lastSecond = 99;
  if (lastSecond == secondHand) {
    return;
  }
  lastSecond = secondHand;

  secondHand = 0;
  if (secondHand == 0)  { currentPalette = OceanColors_p;       currentBlending = LINEARBLEND; }
  if (secondHand == 10) { currentPalette = RainbowStripeColors_p; currentBlending = NOBLEND; }
  if (secondHand == 20) { currentPalette = CloudColors_p;         currentBlending = LINEARBLEND; }
  if (secondHand == 30) { currentPalette = PartyColors_p;         currentBlending = LINEARBLEND; }
  if (secondHand == 40) { currentPalette = LavaColors_p;          currentBlending = LINEARBLEND; }
  if (secondHand == 50) { currentPalette = ForestColors_p;        currentBlending = LINEARBLEND; }
}

// ---------------------------------------------------------------------------
// OSC payload notes:
// - Send a single OSC blob at address /leds containing RGB bytes for each LED.
// - Blob length should be NUM_LEDS * 3. Extra data is ignored.
// - Adjust DATA_PIN_x to match available Nano ESP32 pins (3.3V logic level).
// ---------------------------------------------------------------------------
