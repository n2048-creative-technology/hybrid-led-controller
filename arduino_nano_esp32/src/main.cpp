#include <Arduino.h>
#include <FastLED.h>
#include <WiFi.h>
#include <esp_now.h>

// ---------------------------------------------------------------------------
// ESP-NOW satellite (receives LED frames)
// ---------------------------------------------------------------------------
static constexpr uint16_t kNumColumns = 12;
static constexpr uint16_t kNumRows = 19;
static constexpr uint16_t kNumLeds = kNumColumns * kNumRows;
static constexpr uint16_t kPayloadSize = kNumLeds * 3;

static constexpr uint16_t kChunkSize = 200;
static constexpr uint8_t kMaxChunks = (kPayloadSize + kChunkSize - 1) / kChunkSize;

static constexpr uint8_t kDiscoveryMsg[] = {'D','I','S','C','O','V','E','R'};
static constexpr uint8_t kHereMsg[] = {'H','E','R','E'};

#define LED_TYPE WS2811
#define COLOR_ORDER GRB
static constexpr uint8_t kDefaultBrightness = 64;
static constexpr uint16_t kPowerLimitMa = 3000;

CRGB leds[kNumLeds];

static uint8_t frameBuffer[kPayloadSize];
static bool chunkReceived[kMaxChunks];
static uint8_t currentFrameId = 0;
static uint8_t expectedChunks = 0;
static uint16_t chunksReceived = 0;
static uint32_t framesApplied = 0;
static uint32_t packetsReceived = 0;
static uint32_t lastDebugMs = 0;
static uint32_t lastFrameMs = 0;

static uint8_t broadcastMac[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

static inline uint16_t mapLedIndex(uint16_t logicalIndex) {
  uint16_t col = logicalIndex / kNumRows;
  uint16_t row = logicalIndex % kNumRows;
  uint16_t physicalRow = row;
  if (col % 2 == 1) {
    physicalRow = (kNumRows - 1) - row;
  }
  return col * kNumRows + physicalRow;
}

static void applyFrame(uint16_t ledCount) {
  for (uint16_t i = 0; i < kNumLeds; ++i) {
    leds[i] = CRGB::Black;
  }
  uint16_t idx = 0;
  for (uint16_t i = 0; i < ledCount; ++i) {
    uint8_t r = frameBuffer[idx++];
    uint8_t g = frameBuffer[idx++];
    uint8_t b = frameBuffer[idx++];
    uint16_t physical = mapLedIndex(i);
    if (physical < kNumLeds) {
      leds[physical].setRGB(r, g, b);
    }
  }
  FastLED.show();
  lastFrameMs = millis();
}

static void resetFrameState(uint8_t frameId, uint8_t chunkCount) {
  currentFrameId = frameId;
  expectedChunks = chunkCount;
  chunksReceived = 0;
  for (uint8_t i = 0; i < kMaxChunks; ++i) {
    chunkReceived[i] = false;
  }
}

static void handleDataPacket(const uint8_t *data, int len) {
  if (len < 5) {
    return;
  }
  packetsReceived++;
  const uint8_t frameId = data[0];
  const uint8_t chunkIndex = data[1];
  const uint8_t chunkCount = data[2];
  const uint16_t chunkLen = static_cast<uint16_t>(data[3]) |
                            (static_cast<uint16_t>(data[4]) << 8);
  if (chunkCount == 0 || chunkCount > kMaxChunks) {
    return;
  }
  if (chunkIndex >= chunkCount) {
    return;
  }
  if (len < 5 + chunkLen) {
    return;
  }

  if (frameId != currentFrameId || expectedChunks != chunkCount) {
    resetFrameState(frameId, chunkCount);
  }

  if (!chunkReceived[chunkIndex]) {
    chunkReceived[chunkIndex] = true;
    chunksReceived++;
  }

  const uint16_t offset = static_cast<uint16_t>(chunkIndex) * kChunkSize;
  if (offset + chunkLen > kPayloadSize) {
    return;
  }
  memcpy(frameBuffer + offset, data + 5, chunkLen);

  if (chunksReceived >= expectedChunks) {
    applyFrame(kNumLeds);
    framesApplied++;
  }
}

static void onEspNowRecv(const uint8_t *mac, const uint8_t *data, int len) {
  if (len == static_cast<int>(sizeof(kDiscoveryMsg)) &&
      memcmp(data, kDiscoveryMsg, sizeof(kDiscoveryMsg)) == 0) {
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, mac, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    esp_now_add_peer(&peerInfo);
    esp_now_send(mac, kHereMsg, sizeof(kHereMsg));
    return;
  }

  handleDataPacket(data, len);
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }
  esp_now_register_recv_cb(onEspNowRecv);

  esp_now_peer_info_t broadcastPeer = {};
  memcpy(broadcastPeer.peer_addr, broadcastMac, 6);
  broadcastPeer.channel = 0;
  broadcastPeer.encrypt = false;
  esp_now_add_peer(&broadcastPeer);

  FastLED.addLeds<LED_TYPE, D2, COLOR_ORDER>(leds, kNumLeds).setCorrection(TypicalLEDStrip);
  FastLED.addLeds<LED_TYPE, D3, COLOR_ORDER>(leds, kNumLeds);
  FastLED.addLeds<LED_TYPE, D4, COLOR_ORDER>(leds, kNumLeds);
  FastLED.addLeds<LED_TYPE, D5, COLOR_ORDER>(leds, kNumLeds);
  FastLED.addLeds<LED_TYPE, D6, COLOR_ORDER>(leds, kNumLeds);
  FastLED.addLeds<LED_TYPE, D7, COLOR_ORDER>(leds, kNumLeds);
  FastLED.addLeds<LED_TYPE, D8, COLOR_ORDER>(leds, kNumLeds);
  FastLED.addLeds<LED_TYPE, D9, COLOR_ORDER>(leds, kNumLeds);
  FastLED.addLeds<LED_TYPE, D10, COLOR_ORDER>(leds, kNumLeds);
  FastLED.addLeds<LED_TYPE, D11, COLOR_ORDER>(leds, kNumLeds);
  FastLED.addLeds<LED_TYPE, D12, COLOR_ORDER>(leds, kNumLeds);
  FastLED.addLeds<LED_TYPE, D13, COLOR_ORDER>(leds, kNumLeds);
  FastLED.setBrightness(kDefaultBrightness);
  FastLED.setMaxPowerInVoltsAndMilliamps(5, kPowerLimitMa);
  FastLED.clear(true);
  leds[0] = CRGB(255, 255, 255);
  FastLED.show();
  delay(500);
  FastLED.clear(true);
  FastLED.show();
}

void loop() {
  delay(2);
  const uint32_t now = millis();
  if (now - lastDebugMs > 1000) {
    lastDebugMs = now;
    Serial.print("Debug: packets=");
    Serial.print(packetsReceived);
    Serial.print(" frames=");
    Serial.println(framesApplied);
  }

  if (framesApplied == 0 && now - lastFrameMs > 1000) {
    leds[0] = CRGB(50, 0, 0);
    FastLED.show();
  }
}
