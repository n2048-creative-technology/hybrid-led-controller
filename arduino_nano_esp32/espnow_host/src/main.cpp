#include <Arduino.h>
#include <algorithm>
#include <WiFi.h>
#include <esp_now.h>

// ---------------------------------------------------------------------------
// ESP-NOW host bridge (serial -> ESP-NOW) with autodiscovery
// ---------------------------------------------------------------------------
static constexpr uint32_t kSerialBaud = 115200;
static constexpr uint8_t kSerialHeaderA = 0xAA;
static constexpr uint8_t kSerialHeaderB = 0x55;

static constexpr uint16_t kPayloadSize = 12 * 19 * 3;
static constexpr uint16_t kChunkSize = 200;
static constexpr uint8_t kMaxChunks = (kPayloadSize + kChunkSize - 1) / kChunkSize;

static constexpr uint32_t kDiscoveryIntervalMs = 3000;
static constexpr uint8_t kDiscoveryMsg[] = {'D','I','S','C','O','V','E','R'};
static constexpr uint8_t kHereMsg[] = {'H','E','R','E'};

static uint8_t frameId = 0;
static uint8_t payload[kPayloadSize];
static uint8_t rawPayload[kPayloadSize];

static uint8_t broadcastMac[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

struct Peer {
  uint8_t mac[6];
};

static constexpr size_t kMaxPeers = 20;
static Peer peers[kMaxPeers];
static size_t peerCount = 0;

enum class ParseState {
  WaitA,
  WaitB,
  Target,
  Flags,
  Len1,
  Len2,
  Payload,
  Ck1,
  Ck2
};

static ParseState parseState = ParseState::WaitA;
static uint8_t targetId = 0;
static uint8_t payloadFlags = 0;
static uint16_t payloadLen = 0;
static uint16_t payloadPos = 0;
static uint16_t checksum = 0;
static uint16_t checksumRx = 0;

static uint32_t serialFramesOk = 0;
static uint32_t serialFramesBad = 0;
static uint32_t espNowSends = 0;
static uint32_t lastDebugMs = 0;

static bool decodeRleToRaw(const uint8_t *src, uint16_t srcLen, uint8_t *dst, uint16_t dstLen) {
  uint16_t srcPos = 0;
  uint16_t dstPos = 0;
  while (srcPos + 3 < srcLen) {
    uint8_t run = src[srcPos++];
    uint8_t r = src[srcPos++];
    uint8_t g = src[srcPos++];
    uint8_t b = src[srcPos++];
    for (uint8_t i = 0; i < run; ++i) {
      if (dstPos + 3 > dstLen) {
        return false;
      }
      dst[dstPos++] = r;
      dst[dstPos++] = g;
      dst[dstPos++] = b;
    }
  }
  return dstPos == dstLen;
}

static bool macLess(const uint8_t *a, const uint8_t *b) {
  return memcmp(a, b, 6) < 0;
}

static void printMac(const uint8_t *mac) {
  for (int i = 0; i < 6; ++i) {
    if (i > 0) {
      Serial.print(':');
    }
    if (mac[i] < 16) {
      Serial.print('0');
    }
    Serial.print(mac[i], HEX);
  }
}

static int findPeerIndex(const uint8_t *mac) {
  for (size_t i = 0; i < peerCount; ++i) {
    if (memcmp(peers[i].mac, mac, 6) == 0) {
      return static_cast<int>(i);
    }
  }
  return -1;
}

static bool addPeer(const uint8_t *mac) {
  if (peerCount >= kMaxPeers) {
    return false;
  }
  if (findPeerIndex(mac) >= 0) {
    return true;
  }
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, mac, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    return false;
  }
  memcpy(peers[peerCount].mac, mac, 6);
  peerCount++;
  return true;
}

static void sortPeers() {
  if (peerCount < 2) {
    return;
  }
  for (size_t i = 0; i + 1 < peerCount; ++i) {
    for (size_t j = i + 1; j < peerCount; ++j) {
      if (macLess(peers[j].mac, peers[i].mac)) {
        Peer tmp = peers[i];
        peers[i] = peers[j];
        peers[j] = tmp;
      }
    }
  }
}

static void printPeerMap() {
  Serial.println("Discovered satellites:");
  for (size_t i = 0; i < peerCount; ++i) {
    Serial.print("ID ");
    Serial.print(i + 1);
    Serial.print(" -> ");
    printMac(peers[i].mac);
    Serial.println();
  }
}

static void sendChunkToPeer(const uint8_t *mac, uint8_t chunkIndex, uint8_t chunkCount,
                            const uint8_t *data, uint16_t len) {
  uint8_t buffer[5 + kChunkSize];
  buffer[0] = frameId;
  buffer[1] = chunkIndex;
  buffer[2] = chunkCount;
  buffer[3] = static_cast<uint8_t>(len & 0xFF);
  buffer[4] = static_cast<uint8_t>((len >> 8) & 0xFF);
  memcpy(buffer + 5, data, len);
  esp_now_send(mac, buffer, 5 + len);
  espNowSends++;
}

static void sendFrameToPeers(uint8_t target, const uint8_t *data, uint16_t len) {
  if (peerCount == 0) {
    return;
  }
  const uint8_t chunkCount = static_cast<uint8_t>((len + kChunkSize - 1) / kChunkSize);
  if (chunkCount == 0 || chunkCount > kMaxChunks) {
    return;
  }

  if (target == 0) {
    for (size_t idx = 0; idx < peerCount; ++idx) {
      for (uint8_t chunk = 0; chunk < chunkCount; ++chunk) {
        const uint16_t offset = chunk * kChunkSize;
        const uint16_t chunkLen = static_cast<uint16_t>(
            std::min<uint16_t>(kChunkSize, len - offset));
        sendChunkToPeer(peers[idx].mac, chunk, chunkCount, data + offset, chunkLen);
        delayMicroseconds(200);
      }
    }
  } else if (target >= 1 && target <= static_cast<uint8_t>(peerCount)) {
    const uint8_t *mac = peers[target - 1].mac;
    for (uint8_t chunk = 0; chunk < chunkCount; ++chunk) {
      const uint16_t offset = chunk * kChunkSize;
      const uint16_t chunkLen = static_cast<uint16_t>(
          std::min<uint16_t>(kChunkSize, len - offset));
      sendChunkToPeer(mac, chunk, chunkCount, data + offset, chunkLen);
      delayMicroseconds(200);
    }
  }
}

static void onEspNowRecv(const uint8_t *mac, const uint8_t *data, int len) {
  if (len == static_cast<int>(sizeof(kHereMsg)) &&
      memcmp(data, kHereMsg, sizeof(kHereMsg)) == 0) {
    bool added = addPeer(mac);
    if (added) {
      sortPeers();
      printPeerMap();
    }
  }
}

static void sendDiscovery() {
  esp_now_send(broadcastMac, kDiscoveryMsg, sizeof(kDiscoveryMsg));
}

static void handleByte(uint8_t b) {
  switch (parseState) {
    case ParseState::WaitA:
      if (b == kSerialHeaderA) {
        parseState = ParseState::WaitB;
      }
      break;
    case ParseState::WaitB:
      parseState = (b == kSerialHeaderB) ? ParseState::Target : ParseState::WaitA;
      break;
    case ParseState::Target:
      targetId = b;
      checksum = targetId;
      parseState = ParseState::Flags;
      break;
    case ParseState::Flags:
      payloadFlags = b;
      checksum = static_cast<uint16_t>(checksum + b);
      parseState = ParseState::Len1;
      break;
    case ParseState::Len1:
      payloadLen = b;
      checksum = static_cast<uint16_t>(checksum + b);
      parseState = ParseState::Len2;
      break;
    case ParseState::Len2:
      payloadLen |= static_cast<uint16_t>(b) << 8;
      checksum = static_cast<uint16_t>(checksum + b);
      if (payloadLen == 0 || payloadLen > kPayloadSize) {
        parseState = ParseState::WaitA;
      } else {
        payloadPos = 0;
        parseState = ParseState::Payload;
      }
      break;
    case ParseState::Payload:
      payload[payloadPos++] = b;
      checksum = static_cast<uint16_t>(checksum + b);
      if (payloadPos >= payloadLen) {
        parseState = ParseState::Ck1;
      }
      break;
    case ParseState::Ck1:
      checksumRx = b;
      parseState = ParseState::Ck2;
      break;
    case ParseState::Ck2:
      checksumRx |= static_cast<uint16_t>(b) << 8;
      if (checksumRx == checksum) {
        bool ok = true;
        const bool isRle = (payloadFlags & 0x01) != 0;
        const uint8_t *sendBuf = payload;
        uint16_t sendLen = payloadLen;
        if (isRle) {
          ok = decodeRleToRaw(payload, payloadLen, rawPayload, kPayloadSize);
          sendBuf = rawPayload;
          sendLen = kPayloadSize;
        }
        if (ok) {
          sendFrameToPeers(targetId, sendBuf, sendLen);
          frameId++;
          serialFramesOk++;
          if (serialFramesOk <= 5) {
            Serial.print("Serial frame ok: target=");
            Serial.print(targetId);
            Serial.print(" len=");
            Serial.print(payloadLen);
            Serial.print(" flags=");
            Serial.println(payloadFlags);
          }
        } else {
          serialFramesBad++;
          if (serialFramesBad <= 5) {
            Serial.println("Serial decode failed");
          }
        }
      } else {
        serialFramesBad++;
        if (serialFramesBad <= 5) {
          Serial.print("Serial checksum mismatch: got=");
          Serial.print(checksumRx);
          Serial.print(" expected=");
          Serial.println(checksum);
        }
      }
      parseState = ParseState::WaitA;
      break;
  }
}

void setup() {
  Serial.begin(kSerialBaud);
  Serial.setRxBufferSize(8192);

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

  Serial.println("ESP-NOW host ready");
}

void loop() {
  while (Serial.available() > 0) {
    uint8_t b = static_cast<uint8_t>(Serial.read());
    handleByte(b);
  }

  static uint32_t lastDiscovery = 0;
  const uint32_t now = millis();
  if (now - lastDiscovery > kDiscoveryIntervalMs) {
    lastDiscovery = now;
    sendDiscovery();
  }

  if (now - lastDebugMs > 1000) {
    lastDebugMs = now;
    Serial.print("Debug: peers=");
    Serial.print(peerCount);
    Serial.print(" serial_ok=");
    Serial.print(serialFramesOk);
    Serial.print(" serial_bad=");
    Serial.print(serialFramesBad);
    Serial.print(" espnow_sends=");
    Serial.println(espNowSends);
  }
}
