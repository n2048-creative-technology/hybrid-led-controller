#include "ofApp.h"
#include <algorithm>
#include <filesystem>

namespace {
glm::vec2 estimateBitmapStringSize(const std::string& text, float charW, float charH, float lineH) {
  size_t maxLen = 0;
  size_t lines = 1;
  size_t lineLen = 0;
  for (char ch : text) {
    if (ch == '\n') {
      maxLen = std::max(maxLen, lineLen);
      lineLen = 0;
      lines++;
      continue;
    }
    lineLen++;
  }
  maxLen = std::max(maxLen, lineLen);
  return {static_cast<float>(maxLen) * charW, static_cast<float>(lines) * lineH};
}

constexpr const char* kPresetFilename = "midi_presets.json";
constexpr uint8_t kSerialHeaderA = 0xAA;
constexpr uint8_t kSerialHeaderB = 0x55;
constexpr uint8_t kSerialFlagKeepalive = 0x02;

uint16_t computeSerialChecksum(uint8_t targetId, uint8_t flags, uint16_t payloadLen,
                               const std::vector<uint8_t>& payload) {
  uint32_t sum = targetId + flags + (payloadLen & 0xFF) + (payloadLen >> 8);
  for (size_t i = 0; i < payload.size(); ++i) {
    sum += payload[i];
  }
  return static_cast<uint16_t>(sum & 0xFFFF);
}

}

//--------------------------------------------------------------
void ofApp::setup() {
  ofSetWindowTitle("Hybrid LED Controller");
  ofSetFrameRate(60);
  ofBackground(10);

  downsampleFbo.allocate(kCols, kRows, GL_RGB);
  downsampleFbo.getTexture().setTextureMinMagFilter(GL_NEAREST, GL_NEAREST);
  downsamplePixels.allocate(kCols, kRows, OF_PIXELS_RGB);

  payload.assign(kPayloadSize, 0);
  setupSerial();

  // Optional default video (put in bin/data as video.mp4)
  loadVideo("video.mp4");

  // MIDI setup
  setupMidi();
  satellitesPath = ofToDataPath("satellites.json", true);
  loadSatelliteConfig();
  presetsPath = ofToDataPath(kPresetFilename, true);
  loadPresetsFromFile();
  initSatellitePresets();
}

//--------------------------------------------------------------
void ofApp::update() {
  float dt = ofGetLastFrameTime();
  if (!pauseAutomation) globalTime += dt;

  if (source == Source::Video && videoLoaded && !paused) {
    video.update();
    if (!video.isPlaying()) {
      video.setLoopState(OF_LOOP_NORMAL);
      video.play();
    }
    int currentFrame = video.getCurrentFrame();
    if (currentFrame != lastVideoFrame) {
      lastVideoFrame = currentFrame;
      lastVideoFrameMillis = ofGetElapsedTimeMillis();
    } else if (currentFrame >= 0 && ofGetElapsedTimeMillis() - lastVideoFrameMillis > 2000) {
      video.setLoopState(OF_LOOP_NORMAL);
      video.play();
      lastVideoFrameMillis = ofGetElapsedTimeMillis();
    }
  }

  updateDownsample();
  if (serialReady && !serialDevice.empty()) {
    if (!std::filesystem::exists(serialDevice)) {
      serialReady = false;
      serialStatus = "Serial: Disconnected";
      if (serial.isInitialized()) {
        serial.close();
      }
      lastSerialAttemptMs = ofGetElapsedTimeMillis();
    }
  }
  if (!serialReady) {
    uint32_t now = ofGetElapsedTimeMillis();
    if (now - lastSerialAttemptMs >= serialReconnectDelayMs) {
      lastSerialAttemptMs = now;
      const bool ok = setupSerial();
      if (ok) {
        serialReconnectDelayMs = 1000;
      } else {
        serialReconnectDelayMs = std::min<uint32_t>(serialReconnectDelayMs * 2, 10000);
      }
    }
  }
  sendFrameIfDue();
  sendSerialKeepalive();
}

//--------------------------------------------------------------
void ofApp::draw() {
  drawPreviews();
  drawUiText(20, ofGetHeight() - 220);
}

//--------------------------------------------------------------
void ofApp::exit() {
  if (videoLoaded) {
    video.stop();
  }
}

//--------------------------------------------------------------
void ofApp::loadVideo(const std::string& path) {
  if (path.empty()) return;
  ofFile file(path);
  if (!file.exists()) return;
  video.stop();
  video.close();
  videoLoaded = video.load(path);
  if (videoLoaded) {
    video.setLoopState(OF_LOOP_NORMAL);
    video.setPixelFormat(OF_PIXELS_RGB);
    video.play();
    paused = false;
    lastVideoFrame = -1;
    lastVideoFrameMillis = ofGetElapsedTimeMillis();
  }
}

//--------------------------------------------------------------
void ofApp::loadVideoDialog() {
  ofFileDialogResult res = ofSystemLoadDialog("Select video file");
  if (res.bSuccess) loadVideo(res.getPath());
}

//--------------------------------------------------------------
void ofApp::updateDownsample() {
  if (source == Source::MidiPattern) {
    drawLinePatternPixels();
    return;
  }
  if (source == Source::Video) {
    if (!videoLoaded || !video.isFrameNew()) return;
    const ofPixels& srcPixels = video.getPixels();
    if (!srcPixels.isAllocated()) return;
    downsamplePixels = srcPixels;
    downsamplePixels.resize(kCols, kRows);
  }
}

//--------------------------------------------------------------
void ofApp::drawLinePatternPixels() {
  float a = ofDegToRad(ofClamp(angleDeg, 0.0f, 90.0f));
  ofVec2f n(cosf(a), sinf(a));
  float rotCyclesPerSec = rotationSpeedDegPerSec / 360.0f;
  float sOffset = fmodf(automationPhase + rotCyclesPerSec * globalTime, 1.0f);
  float yOffset = fmodf(automationPhase + (verticalSpeedPxPerSec / (float)kRows) * globalTime, 1.0f);
  float phase = fmodf(n.x * sOffset + n.y * yOffset, 1.0f);
  if (phase < 0) phase += 1.0f;
  auto periodicDelta = [](float x, float y, float p){ float d = fmodf(x - y + p * 0.5f, p) - p * 0.5f; return fabsf(d); };
  float widthNorm = lineWidth / (float)std::max(kCols, kRows);
  const bool forceFullOn = (lineWidth >= 11.9f) && (lineFalloff >= 0.99f);

  ofColor base(lineColor);
  for (int s = 0; s < kCols; ++s) {
    for (int y = 0; y < kRows; ++y) {
      float ss = (s + 0.5f) / (float)kCols;
      float yy = (y + 0.5f) / (float)kRows;
      float v = ss * n.x + yy * n.y;
      float d = periodicDelta(v, phase, 1.0f);
      float intensity = 1.0f;
      if (!forceFullOn) {
        float baseIntensity = ofClamp(1.0f - d / std::max(0.0001f, widthNorm), 0.0f, 1.0f);
        float hardMask = (d <= widthNorm) ? 1.0f : 0.0f;
        intensity = ofLerp(baseIntensity, hardMask, ofClamp(lineFalloff, 0.0f, 1.0f));
      }
      ofColor c = base * intensity;
      int yTop = (kRows - 1) - y;  // top-origin storage
      downsamplePixels.setColor(s, yTop, c);
    }
  }
}

//--------------------------------------------------------------
void ofApp::buildPayload() {
  if (blackout) {
    std::fill(payload.begin(), payload.end(), 0);
    return;
  }
  // Map downsamplePixels (top-origin) to payload according to mapping controls
  // Use mapping approach compatible with video-to-LED-matrix
  for (int x = 0; x < kCols; ++x) {
    int xRot = x + columnOffset;
    while (xRot < 0) xRot += kCols;
    xRot %= kCols;
    for (int y = 0; y < kRows; ++y) {  // y=0 bottom
      int sampleY = (kRows - 1) - y;  // convert to pixel space (top origin)
      if (verticalFlip) {
        sampleY = (kRows - 1) - sampleY;
      }
      ofColor c = downsamplePixels.getColor(x, sampleY);
      const float ledBrightness = powf(ofClamp(brightnessScalar, 0.0f, 1.0f), brightnessGamma);
      uint8_t r = static_cast<uint8_t>(ofClamp(c.r * ledBrightness, 0.0f, 255.0f));
      uint8_t g = static_cast<uint8_t>(ofClamp(c.g * ledBrightness, 0.0f, 255.0f));
      uint8_t b = static_cast<uint8_t>(ofClamp(c.b * ledBrightness, 0.0f, 255.0f));

      int yMapped = y;
      if (serpentine && (xRot % 2 == 1)) {
        yMapped = (kRows - 1) - y;
      }
      int ledIndex = xRot * kRows + yMapped;
      const int base = ledIndex * 3;
      payload[base + 0] = r;
      payload[base + 1] = g;
      payload[base + 2] = b;
    }
  }
}

static ofFloatColor hsbToFloatColor(float h, float s, float b) {
  return ofFloatColor::fromHsb(h / 255.0f, s / 255.0f, b / 255.0f);
}

void ofApp::applyCurrentToPreset(Preset& preset) const {
  preset.hasData = true;
  preset.source = source;
  preset.lineWidth = lineWidth;
  preset.angleDeg = angleDeg;
  preset.rotationSpeedDegPerSec = rotationSpeedDegPerSec;
  preset.verticalSpeedPxPerSec = verticalSpeedPxPerSec;
  preset.pauseAutomation = pauseAutomation;
  preset.automationPhase = automationPhase;
  preset.lineFalloff = lineFalloff;
  preset.lineColor = lineColor;
  preset.lineHue = lineHue;
  preset.lineSat = lineSat;
  preset.lineBri = lineBri;
  preset.serpentine = serpentine;
  preset.verticalFlip = verticalFlip;
  preset.columnOffset = columnOffset;
  preset.targetSendFps = targetSendFps;
  preset.brightnessScalar = brightnessScalar;
  preset.sendOsc = sendOsc;
  preset.blackout = blackout;
}

void ofApp::applyPresetToCurrent(const Preset& preset) {
  source = preset.source;
  lineWidth = preset.lineWidth;
  angleDeg = preset.angleDeg;
  rotationSpeedDegPerSec = preset.rotationSpeedDegPerSec;
  verticalSpeedPxPerSec = preset.verticalSpeedPxPerSec;
  pauseAutomation = preset.pauseAutomation;
  automationPhase = preset.automationPhase;
  lineFalloff = preset.lineFalloff;
  lineColor = preset.lineColor;
  lineHue = preset.lineHue;
  lineSat = preset.lineSat;
  lineBri = preset.lineBri;
  lineColor = hsbToFloatColor(lineHue, lineSat, lineBri);
  serpentine = preset.serpentine;
  verticalFlip = preset.verticalFlip;
  columnOffset = preset.columnOffset;
  targetSendFps = ofClamp(preset.targetSendFps, 1.0f, 120.0f);
  brightnessScalar = ofClamp(preset.brightnessScalar, 0.0f, 1.0f);
  sendOsc = preset.sendOsc;
  blackout = preset.blackout;
}

int ofApp::firstActiveSatellite() const {
  for (int i = 0; i < kSatelliteCount; ++i) {
    if (satelliteActive[i]) {
      return i;
    }
  }
  return -1;
}

void ofApp::applyCurrentToActiveSatellites() {
  for (int i = 0; i < kSatelliteCount; ++i) {
    if (satelliteActive[i]) {
      applyCurrentToPreset(satellitePresets[i]);
    }
  }
}

//--------------------------------------------------------------
void ofApp::sendFrameIfDue() {
  if (!sendOsc) {
    return;
  }
  const float bytesPerFrame = static_cast<float>(payload.size() + 7);
  const float bytesPerSec = static_cast<float>(serialBaud) / 10.0f;
  maxSerialFps = std::max(1.0f, bytesPerSec / bytesPerFrame);
  uint32_t now = ofGetElapsedTimeMillis();
  if (now < serialBackoffUntilMs) {
    return;
  }

  const bool forceAll = forceSendAllOnce;
  if (!forceAll && firstActiveSatellite() < 0) {
    return;
  }

  Preset saved;
  applyCurrentToPreset(saved);

  for (int i = 0; i < kSatelliteCount; ++i) {
    if (!forceAll && !satelliteActive[i]) {
      continue;
    }
    const Preset& preset = satellitePresets[i];
    const float effectiveFps = std::max(1.0f, std::min(preset.targetSendFps, maxSerialFps));
    const uint32_t intervalMs = static_cast<uint32_t>(1000.0f / effectiveFps);
    if (!forceAll && now - lastSendMillisBySlot[i] < intervalMs) {
      continue;
    }
    lastSendMillisBySlot[i] = now;
    applyPresetToCurrent(preset);
    if (source == Source::MidiPattern) {
      drawLinePatternPixels();
    }
    buildPayload();
    const int target = satelliteIds[i];
    if (target >= 0 && target <= 9) {
      sendSerialFrame(static_cast<uint8_t>(target));
    }
  }

  applyPresetToCurrent(saved);
  if (forceAll) {
    forceSendAllOnce = false;
  }
}

//--------------------------------------------------------------
bool ofApp::setupSerial() {
  serialReady = false;
  serialStatus = "Serial: Not connected";
  serialDevice.clear();
  if (serial.isInitialized()) {
    serial.close();
  }

  std::string desired;
  const std::string configPath = ofToDataPath("serial_device.txt", true);
  if (ofFile::doesFileExist(configPath)) {
    ofBuffer buffer = ofBufferFromFile(configPath);
    for (const auto& line : buffer.getLines()) {
      std::string entry = ofTrim(line);
      if (!entry.empty() && entry[0] != '#') {
        size_t comma = entry.find(',');
        if (comma != std::string::npos) {
          desired = ofTrim(entry.substr(0, comma));
          std::string baudStr = ofTrim(entry.substr(comma + 1));
          if (!baudStr.empty()) {
            serialBaud = std::max(9600, ofToInt(baudStr));
          }
        } else {
          desired = entry;
        }
        break;
      }
    }
  }

  const bool requireExact = !desired.empty() && desired != "auto";
  std::string autoDevice;
  if (!requireExact) {
    const std::string byIdPath = "/dev/serial/by-id";
    try {
      if (std::filesystem::exists(byIdPath)) {
        for (const auto& entry : std::filesystem::directory_iterator(byIdPath)) {
          if (!entry.is_symlink()) {
            continue;
          }
          const std::string name = entry.path().filename().string();
          if (name.find("Arduino") == std::string::npos &&
              name.find("Nano") == std::string::npos &&
              name.find("ESP32") == std::string::npos) {
            continue;
          }
          autoDevice = entry.path().string();
          break;
        }
        if (autoDevice.empty()) {
          for (const auto& entry : std::filesystem::directory_iterator(byIdPath)) {
            if (entry.is_symlink()) {
              autoDevice = entry.path().string();
              break;
            }
          }
        }
      }
    } catch (const std::exception& e) {
      ofLogWarning() << "Serial auto-detect failed: " << e.what();
    }
  }

  if (requireExact) {
    if (serial.setup(desired, serialBaud)) {
      serialReady = true;
      serialDevice = desired;
      serialStatus = std::string("Serial: ") + serialDevice;
      serialConsecutiveErrors = 0;
      serialLastOkMs = ofGetElapsedTimeMillis();
      return true;
    } else {
      serialReady = false;
      serialStatus = std::string("Serial: Failed to open ") + desired;
    }
    return false;
  }

  int selected = -1;
  if (!autoDevice.empty()) {
    desired = autoDevice;
  }

  auto devices = serial.getDeviceList();
  if (devices.empty()) {
    serialStatus = "Serial: No devices";
    return false;
  }

  if (!desired.empty() && desired != "auto") {
    for (size_t i = 0; i < devices.size(); ++i) {
      auto dev = devices[i];
      if (dev.getDevicePath() == desired || dev.getDeviceName() == desired) {
        selected = static_cast<int>(i);
        break;
      }
    }
  }
  if (selected < 0) {
    int preferred = -1;
    int fallback = -1;
    for (size_t i = 0; i < devices.size(); ++i) {
      auto dev = devices[i];
      const std::string path = dev.getDevicePath();
      if (ofIsStringInString(path, "/dev/ttyS")) {
        continue;
      }
      if (fallback < 0) {
        fallback = static_cast<int>(i);
      }
      if (ofIsStringInString(path, "/dev/ttyACM") || ofIsStringInString(path, "/dev/ttyUSB")) {
        preferred = static_cast<int>(i);
        break;
      }
    }
    selected = (preferred >= 0) ? preferred : std::max(0, fallback);
  }

  const std::string devicePath = devices[selected].getDevicePath();
  if (serial.setup(devicePath, serialBaud)) {
    serialReady = true;
    serialDevice = devicePath;
    if (!autoDevice.empty()) {
      serialStatus = std::string("Serial: auto -> ") + serialDevice;
    } else {
      serialStatus = std::string("Serial: ") + serialDevice;
    }
    serialConsecutiveErrors = 0;
    serialLastOkMs = ofGetElapsedTimeMillis();
    return true;
  } else {
    serialReady = false;
    serialStatus = std::string("Serial: Failed to open ") + devicePath;
  }
  return false;
}

//--------------------------------------------------------------
void ofApp::sendSerialFrame(uint8_t target) {
  if (!serialReady) {
    return;
  }
  encodeRlePayload();
  const bool useRle = !rlePayload.empty() && rlePayload.size() < payload.size();
  const std::vector<uint8_t>& framePayload = useRle ? rlePayload : payload;
  const uint8_t flags = useRle ? 0x01 : 0x00;
  const uint16_t payloadLen = static_cast<uint16_t>(framePayload.size());
  const uint16_t checksum = computeSerialChecksum(target, flags, payloadLen, framePayload);
  const size_t frameSize = 2 + 1 + 1 + 2 + payloadLen + 2;
  if (serialFrame.size() != frameSize) {
    serialFrame.resize(frameSize);
  }
  size_t offset = 0;
  serialFrame[offset++] = kSerialHeaderA;
  serialFrame[offset++] = kSerialHeaderB;
  serialFrame[offset++] = target;
  serialFrame[offset++] = flags;
  serialFrame[offset++] = static_cast<uint8_t>(payloadLen & 0xFF);
  serialFrame[offset++] = static_cast<uint8_t>((payloadLen >> 8) & 0xFF);
  if (!framePayload.empty()) {
    memcpy(serialFrame.data() + offset, framePayload.data(), payloadLen);
    offset += payloadLen;
  }
  serialFrame[offset++] = static_cast<uint8_t>(checksum & 0xFF);
  serialFrame[offset++] = static_cast<uint8_t>((checksum >> 8) & 0xFF);

  const int written = serial.writeBytes(serialFrame.data(), static_cast<int>(serialFrame.size()));
  if (written == static_cast<int>(serialFrame.size())) {
    framesSent.fetch_add(1);
    serialConsecutiveErrors = 0;
    serialLastOkMs = ofGetElapsedTimeMillis();
    serialBackoffUntilMs = 0;
  } else {
    framesDropped.fetch_add(1);
    if (written <= 0) {
      serialConsecutiveErrors++;
      const uint32_t now = ofGetElapsedTimeMillis();
      serialBackoffUntilMs = now + 100;
      if (serialConsecutiveErrors >= 20 || (now - serialLastOkMs) > 10000) {
        if (!serialDevice.empty() && !std::filesystem::exists(serialDevice)) {
          serialReady = false;
          serialStatus = "Serial: Disconnected";
          if (serial.isInitialized()) {
            serial.close();
          }
          lastSerialAttemptMs = now;
        } else {
          serialStatus = "Serial: Write stalled";
        }
      }
    }
  }
}

//--------------------------------------------------------------
void ofApp::sendSerialKeepalive() {
  if (!serialReady) {
    return;
  }
  const uint32_t now = ofGetElapsedTimeMillis();
  if (now < serialBackoffUntilMs) {
    return;
  }
  if ((now - serialLastOkMs) < serialKeepaliveIntervalMs) {
    return;
  }
  if ((now - lastKeepaliveMs) < serialKeepaliveIntervalMs) {
    return;
  }
  lastKeepaliveMs = now;

  const uint8_t payloadByte = 0x00;
  const uint16_t payloadLen = 1;
  const uint8_t flags = kSerialFlagKeepalive;
  const uint16_t checksum = computeSerialChecksum(0, flags, payloadLen,
                                                   std::vector<uint8_t>{payloadByte});
  const size_t frameSize = 2 + 1 + 1 + 2 + payloadLen + 2;
  if (serialFrame.size() != frameSize) {
    serialFrame.resize(frameSize);
  }
  size_t offset = 0;
  serialFrame[offset++] = kSerialHeaderA;
  serialFrame[offset++] = kSerialHeaderB;
  serialFrame[offset++] = 0;
  serialFrame[offset++] = flags;
  serialFrame[offset++] = static_cast<uint8_t>(payloadLen & 0xFF);
  serialFrame[offset++] = static_cast<uint8_t>((payloadLen >> 8) & 0xFF);
  serialFrame[offset++] = payloadByte;
  serialFrame[offset++] = static_cast<uint8_t>(checksum & 0xFF);
  serialFrame[offset++] = static_cast<uint8_t>((checksum >> 8) & 0xFF);

  const int written = serial.writeBytes(serialFrame.data(), static_cast<int>(serialFrame.size()));
  if (written == static_cast<int>(serialFrame.size())) {
    serialConsecutiveErrors = 0;
    serialLastOkMs = now;
    serialBackoffUntilMs = 0;
  } else if (written <= 0) {
    serialConsecutiveErrors++;
    serialBackoffUntilMs = now + 100;
  }
}

//--------------------------------------------------------------
void ofApp::encodeRlePayload() {
  rlePayload.clear();
  if (payload.empty()) {
    return;
  }
  const size_t pixelCount = payload.size() / 3;
  rlePayload.reserve(pixelCount * 4);
  size_t i = 0;
  while (i < pixelCount) {
    const size_t base = i * 3;
    uint8_t r = payload[base];
    uint8_t g = payload[base + 1];
    uint8_t b = payload[base + 2];
    uint8_t run = 1;
    while (i + run < pixelCount && run < 255) {
      const size_t nextBase = (i + run) * 3;
      if (payload[nextBase] != r || payload[nextBase + 1] != g || payload[nextBase + 2] != b) {
        break;
      }
      run++;
    }
    rlePayload.push_back(run);
    rlePayload.push_back(r);
    rlePayload.push_back(g);
    rlePayload.push_back(b);
    i += run;
  }
}

//--------------------------------------------------------------
void ofApp::drawPreviews() {
  float margin = 20.0f;
  float halfW = ofGetWidth() * 0.5f - margin * 1.5f;
  float videoH = halfW * 9.0f / 16.0f;
  if (videoH > ofGetHeight() - 2 * margin) {
    videoH = ofGetHeight() - 2 * margin;
    halfW = videoH * 16.0f / 9.0f;
  }

  ofPushStyle();
  ofSetColor(255);
  if (source == Source::Video && videoLoaded) {
    video.draw(margin, margin, halfW, videoH);
  } else {
  ofSetColor(60);
    ofDrawRectangle(margin, margin, halfW, videoH);
    ofSetColor(200);
    std::string lbl = (source == Source::MidiPattern) ? "MIDI pattern" : "";
    ofDrawBitmapString("Preview: " + lbl, margin + 10, margin + 20);
  }

  // Grid preview of what we will send
  float gridScale = std::min(halfW / kCols, videoH / kRows);
  float gx = margin + halfW + margin;
  float gy = margin;
  drawGridPreview(gx, gy, gridScale);

  ofPopStyle();
}

//--------------------------------------------------------------
void ofApp::drawGridPreview(float x, float y, float scale) {
  ofPushMatrix();
  ofTranslate(x, y);
  ofScale(scale, scale);
  ofFill();
  for (int cx = 0; cx < kCols; ++cx) {
    for (int cy = 0; cy < kRows; ++cy) {
      int sampleY = (kRows - 1) - cy;
      ofColor c = downsamplePixels.getColor(cx, sampleY);
      ofSetColor(c);
      ofDrawRectangle(cx, (kRows - 1) - cy, 1.0f, 1.0f);
    }
  }
  ofNoFill();
  ofSetColor(40, 200);
  for (int gx = 0; gx <= kCols; ++gx) ofDrawLine(gx, 0, gx, kRows);
  for (int gy = 0; gy <= kRows; ++gy) ofDrawLine(0, gy, kCols, gy);
  ofPopMatrix();
}

//--------------------------------------------------------------
void ofApp::drawUiText(float x, float y) {
  const float charW = 8.0f;
  const float charH = 12.0f;
  const float lineH = 14.0f;
  const float sliderH = 10.0f;
  const float sliderGap = 8.0f;
  const float satellitesGap = 6.0f;
  const float satellitesH = charH * 2.4f;
  const bool connected = serialReady;
  const std::string statusText = connected ? "SERIAL READY" : "SERIAL NOT READY";
  const ofColor statusColor = connected ? ofColor(0, 200, 90) : ofColor(220, 60, 60);

  std::stringstream ss;
  ss << "Mode: ";
  ss << (source == Source::Video ? "Video" : "MIDI Pattern") << "\n";
  ss << serialStatus << "\n";
  ss << "Send FPS: " << targetSendFps << " (max " << maxSerialFps << ")  Frames sent: " << framesSent.load()
     << " dropped: " << framesDropped.load() << "\n";
  ss << "Brightness: " << brightnessScalar << "  Mapping: " << (serpentine ? "serpentine" : "linear") << "\n";
  ss << "Vertical flip: " << (verticalFlip ? "on" : "off") << "  Column offset: " << columnOffset << "\n";
  ss << "Send: " << (sendOsc ? "on" : "off") << "  Blackout: " << (blackout ? "on" : "off") << "\n";
  ss << midiStatus << "\n";
  ss << "Controls:\n";
  ss << "  Space play/pause (video)  |  L load file  |  F fullscreen\n";
  ss << "  M switch mode (video/midi)  |  ,/. mapping linear/serpentine  |  V vertical flip  |  [ ] rotate  |  R reset\n";
  ss << "  Up/Down send fps  |  +/- brightness\n";
  ss << "  1-8 toggle satellite active  |  0 toggle all\n";
  ss << "  Line: A/Z angle  |  W/S width  |  O/P rot speed  |  K/I vertical speed  |  Q pause anim\n";
  ss << "  Edit bin/data/serial_device.txt to set serial port (or auto)\n";
  ss << "  Edit bin/data/satellites.json for names/IDs\n";
  ss << "  Drag-and-drop a video file onto window\n";
  const glm::vec2 textSize = estimateBitmapStringSize(ss.str(), charW, charH, lineH);

  float pad = 4.0f;
  float statusH = charH + pad * 2.0f;
  float statusW = (statusText.size() * charW) + pad * 3.0f + statusH;

  float gap = 10.0f;
  float blockW = std::max(statusW, textSize.x);
  float blockH = statusH + gap + textSize.y + sliderGap + sliderH + satellitesGap + satellitesH;

  float availW = std::max(40.0f, ofGetWidth() - x - 20.0f);
  float availH = std::max(40.0f, ofGetHeight() - 20.0f);

  float desiredScale = 6.0f;
  float scaleW = availW / blockW;
  float scaleH = availH / blockH;
  float uiScale = std::min(desiredScale, std::min(scaleW, scaleH));
  uiScale = std::max(1.0f, uiScale);

  float scaledW = blockW * uiScale;
  float scaledH = blockH * uiScale;
  float drawX = x;
  float drawY = y;
  if (drawX + scaledW > ofGetWidth() - 10.0f) drawX = std::max(10.0f, ofGetWidth() - 10.0f - scaledW);
  if (drawY + scaledH > ofGetHeight() - 10.0f) drawY = std::max(10.0f, ofGetHeight() - 10.0f - scaledH);
  if (drawY < 10.0f) drawY = 10.0f;

  ofPushMatrix();
  ofTranslate(drawX, drawY);
  ofScale(uiScale, uiScale);
  ofPushStyle();
  ofSetColor(20, 20, 20, 220);
  ofDrawRectangle(0, 0, statusW, statusH);
  ofSetColor(statusColor);
  ofDrawCircle(pad + statusH * 0.5f, statusH * 0.5f, statusH * 0.35f);
  ofSetColor(255);
  ofDrawBitmapString(statusText, pad * 2.0f + statusH, statusH - pad);
  ofPopStyle();
  ofDrawBitmapStringHighlight(ss.str(), 0, statusH + 10.0f);
  float sliderY = statusH + 10.0f + textSize.y + sliderGap;
  float sliderW = blockW;
  ofSetColor(40, 40, 40, 200);
  ofDrawRectangle(0, sliderY, sliderW, sliderH);
  ofSetColor(60, 160, 255, 220);
  ofDrawRectangle(0, sliderY, sliderW * brightnessScalar, sliderH);
  ofNoFill();
  ofSetColor(255);
  ofDrawRectangle(0, sliderY, sliderW, sliderH);
  ofFill();

  drawSatelliteUi(0, sliderY + sliderH + satellitesGap);
  ofPopMatrix();

  brightnessSliderRect.set(drawX, drawY + sliderY * uiScale, blockW * uiScale, sliderH * uiScale);
}

//--------------------------------------------------------------
void ofApp::drawSatelliteUi(float x, float y) {
  const float radius = 6.0f;
  const float gap = 10.0f;
  const float labelGap = 6.0f;
  const float labelY = y + radius * 2.2f + labelGap;
  float cursorX = x;

  for (int i = 0; i < kSatelliteCount; ++i) {
    const bool active = satelliteActive[i];
    const ofColor fill = active ? ofColor(60, 200, 90) : ofColor(80, 80, 80);
    ofSetColor(fill);
    ofDrawCircle(cursorX + radius, y + radius, radius);
    ofSetColor(20);
    ofDrawBitmapString(std::to_string(i + 1), cursorX + radius - 3.0f, y + radius + 4.0f);
    ofSetColor(230);
    const std::string base = satelliteNames[i].empty() ? ("Light " + std::to_string(i + 1)) : satelliteNames[i];
    const std::string label = base + " (" + std::to_string(satelliteIds[i]) + ")";
    ofDrawBitmapString(label, cursorX, labelY + 10.0f);
    cursorX += radius * 2.0f + gap + (label.size() * 6.0f);
  }
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key) {
  bool settingsChanged = false;
  const int activeBefore = firstActiveSatellite();
  switch (key) {
    case ' ':
      paused = !paused;
      if (videoLoaded) video.setPaused(paused);
      break;
    case 'l': case 'L':
      loadVideoDialog();
      break;
    case 'f': case 'F':
      ofToggleFullscreen();
      break;
    case 'm': case 'M':
      source = (source == Source::Video) ? Source::MidiPattern : Source::Video;
      settingsChanged = true;
      break;
    case ',':
    case '<':
      serpentine = false;
      settingsChanged = true;
      break;
    case '.':
    case '>':
      serpentine = true;
      settingsChanged = true;
      break;
    case 'v': case 'V': verticalFlip = !verticalFlip; settingsChanged = true; break;
    case '[': columnOffset--; settingsChanged = true; break;
    case ']': columnOffset++; settingsChanged = true; break;
    case 'r': case 'R': columnOffset = 0; settingsChanged = true; break;
    case '+': case '=': brightnessScalar = ofClamp(brightnessScalar + 0.05f, 0.0f, 1.0f); settingsChanged = true; break;
    case '-': case '_': brightnessScalar = ofClamp(brightnessScalar - 0.05f, 0.0f, 1.0f); settingsChanged = true; break;
    case OF_KEY_UP:   targetSendFps = std::min(120.0f, targetSendFps + 1.0f); settingsChanged = true; break;
    case OF_KEY_DOWN: targetSendFps = std::max(1.0f, targetSendFps - 1.0f); settingsChanged = true; break;

    // Line pattern controls
    case 'a': case 'A': angleDeg = ofClamp(angleDeg + 2.0f, 0.0f, 90.0f); settingsChanged = true; break;
    case 'z': case 'Z': angleDeg = ofClamp(angleDeg - 2.0f, 0.0f, 90.0f); settingsChanged = true; break;
    case 'w': case 'W': lineWidth = ofClamp(lineWidth + 0.5f, 0.5f, 12.0f); settingsChanged = true; break;
    case 's': case 'S': lineWidth = ofClamp(lineWidth - 0.5f, 0.5f, 12.0f); settingsChanged = true; break;
    case 'o': case 'O': rotationSpeedDegPerSec = ofClamp(rotationSpeedDegPerSec + 5.0f, -360.0f, 360.0f); settingsChanged = true; break;
    case 'p': case 'P': rotationSpeedDegPerSec = ofClamp(rotationSpeedDegPerSec - 5.0f, -360.0f, 360.0f); settingsChanged = true; break;
    case 'k': case 'K': verticalSpeedPxPerSec = ofClamp(verticalSpeedPxPerSec + 0.5f, -20.0f, 20.0f); settingsChanged = true; break;
    case 'i': case 'I': verticalSpeedPxPerSec = ofClamp(verticalSpeedPxPerSec - 0.5f, -20.0f, 20.0f); settingsChanged = true; break;
    case 'q': case 'Q': pauseAutomation = !pauseAutomation; settingsChanged = true; break;
    case '0': {
      bool anyOff = false;
      for (bool active : satelliteActive) {
        if (!active) {
          anyOff = true;
          break;
        }
      }
      for (int i = 0; i < kSatelliteCount; ++i) {
        satelliteActive[i] = anyOff;
      }
      syncMidiLedState();
    } break;
    case '1': case '2': case '3': case '4': case '5': case '6': case '7': case '8': {
      const int idx = key - '1';
      toggleSatelliteActive(idx);
    } break;
    default:
      break;
  }
  if (settingsChanged) {
    applyCurrentToActiveSatellites();
  } else if (activeBefore < 0) {
    const int activeNow = firstActiveSatellite();
    if (activeNow >= 0) {
      applyPresetToCurrent(satellitePresets[activeNow]);
    }
  }
}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button) {
  if (brightnessSliderRect.inside(static_cast<float>(x), static_cast<float>(y))) {
    brightnessSliderActive = true;
    float t = (x - brightnessSliderRect.x) / brightnessSliderRect.width;
    brightnessScalar = ofClamp(t, 0.0f, 1.0f);
    applyCurrentToActiveSatellites();
  }
}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button) {
  if (brightnessSliderActive) {
    float t = (x - brightnessSliderRect.x) / brightnessSliderRect.width;
    brightnessScalar = ofClamp(t, 0.0f, 1.0f);
    applyCurrentToActiveSatellites();
  }
}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button) { brightnessSliderActive = false; }

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo) { if (!dragInfo.files.empty()) loadVideo(dragInfo.files[0]); }

//--------------------------------------------------------------
void ofApp::setupMidi() {
#if HAS_OFXMIDI
  midiPorts.clear();
  midiStatus = "MIDI: Scanning";
  auto ports = midiIn.getInPortList();
  for (auto &p : ports) midiPorts.push_back(p);
  // Try to find nanoKONTROL2 / KORG
  int found = -1;
  for (int i = 0; i < (int)midiPorts.size(); ++i) {
    std::string name = midiPorts[i];
    std::string low = name; std::transform(low.begin(), low.end(), low.begin(), ::tolower);
    if (low.find("nanokontrol2") != std::string::npos || low.find("korg") != std::string::npos) {
      found = i; break;
    }
  }
  if (found >= 0) {
    if (midiIn.isOpen()) { midiIn.closePort(); midiIn.removeListener(this); }
    midiIn.openPort(found);
    midiIn.addListener(this);
    midiIn.ignoreTypes(false, false, false);
    midiDeviceName = midiIn.getName();
    midiEnabled = true;
    midiPortName = midiPorts[found];
    midiStatus = std::string("MIDI: Connected ") + midiDeviceName;
    setupMidiOut();
  } else if (!midiPorts.empty()) {
    // Open first available
    int idx = 0;
    if (midiIn.isOpen()) { midiIn.closePort(); midiIn.removeListener(this); }
    midiIn.openPort(idx);
    midiIn.addListener(this);
    midiIn.ignoreTypes(false, false, false);
    midiDeviceName = midiIn.getName();
    midiEnabled = true;
    midiPortName = midiPorts[idx];
    midiStatus = std::string("MIDI: Connected ") + midiDeviceName;
    setupMidiOut();
  } else {
    midiEnabled = false;
    midiStatus = "MIDI: Not detected";
    midiOutEnabled = false;
  }
#else
  midiEnabled = false;
  midiStatus = "MIDI: ofxMidi not installed";
#endif
}

//--------------------------------------------------------------
void ofApp::setupMidiOut() {
#if HAS_OFXMIDI
  midiOutEnabled = false;
  midiOutPorts.clear();
  auto ports = midiOut.getOutPortList();
  for (auto &p : ports) midiOutPorts.push_back(p);
  if (midiOutPorts.empty()) {
    return;
  }

  int selected = -1;
  if (!midiPortName.empty()) {
    const std::string target = ofToLower(midiPortName);
    for (size_t i = 0; i < midiOutPorts.size(); ++i) {
      if (ofToLower(midiOutPorts[i]) == target) {
        selected = static_cast<int>(i);
        break;
      }
    }
    if (selected < 0) {
      for (size_t i = 0; i < midiOutPorts.size(); ++i) {
        const std::string low = ofToLower(midiOutPorts[i]);
        if (low.find("nanokontrol2") != std::string::npos || low.find("korg") != std::string::npos) {
          selected = static_cast<int>(i);
          break;
        }
      }
    }
  }
  if (selected < 0) {
    selected = 0;
  }
  midiOut.openPort(selected);
  midiOutEnabled = true;
  syncMidiLedState();
#endif
}

//--------------------------------------------------------------
void ofApp::setSatelliteActive(int idx, bool active) {
  if (idx < 0 || idx >= kSatelliteCount) {
    return;
  }
  satelliteActive[idx] = active;
  syncMidiLedState();
}

//--------------------------------------------------------------
void ofApp::toggleSatelliteActive(int idx) {
  if (idx < 0 || idx >= kSatelliteCount) {
    return;
  }
  satelliteActive[idx] = !satelliteActive[idx];
  syncMidiLedState();
}

//--------------------------------------------------------------
void ofApp::syncMidiLedState() {
#if HAS_OFXMIDI
  if (!midiOutEnabled) {
    return;
  }
  for (int i = 0; i < kSatelliteCount; ++i) {
    const int cc = 32 + i; // S buttons
    const int val = satelliteActive[i] ? 127 : 0;
    midiOut.sendControlChange(1, cc, val);
  }
#endif
}

//--------------------------------------------------------------
void ofApp::loadSatelliteConfig() {
  satelliteIds.fill(0);
  satelliteNames.fill(std::string());
  satelliteActive.fill(true);

  for (int i = 0; i < kSatelliteCount; ++i) {
    satelliteIds[i] = i + 1;
    satelliteNames[i] = "Light " + std::to_string(i + 1);
  }

  if (satellitesPath.empty() || !ofFile::doesFileExist(satellitesPath)) {
    ofJson root;
    root["satellites"] = ofJson::array();
    for (int i = 0; i < kSatelliteCount; ++i) {
      ofJson entry;
      entry["id"] = satelliteIds[i];
      entry["name"] = satelliteNames[i];
      entry["active"] = true;
      root["satellites"].push_back(entry);
    }
    ofSavePrettyJson(satellitesPath, root);
    return;
  }

  ofJson root;
  try {
    root = ofLoadJson(satellitesPath);
  } catch (const std::exception& e) {
    ofLogWarning() << "Failed to read satellites config: " << e.what();
    return;
  }
  if (!root.contains("satellites") || !root["satellites"].is_array()) {
    return;
  }
  const auto& arr = root["satellites"];
  const size_t count = std::min(arr.size(), static_cast<size_t>(kSatelliteCount));
  for (size_t i = 0; i < count; ++i) {
    const auto& entry = arr[i];
    if (!entry.is_object()) {
      continue;
    }
    satelliteIds[i] = entry.value("id", satelliteIds[i]);
    satelliteNames[i] = entry.value("name", satelliteNames[i]);
    satelliteActive[i] = entry.value("active", true);
  }
  syncMidiLedState();
}

//--------------------------------------------------------------
void ofApp::initSatellitePresets() {
  for (int i = 0; i < kSatelliteCount; ++i) {
    applyCurrentToPreset(satellitePresets[i]);
    lastSendMillisBySlot[i] = 0;
  }
  if (firstActiveSatellite() < 0) {
    satelliteActive[0] = true;
  }
}

//--------------------------------------------------------------
void ofApp::loadPresetsFromFile() {
  if (presetsPath.empty() || !ofFile::doesFileExist(presetsPath)) {
    return;
  }
  ofJson root;
  try {
    root = ofLoadJson(presetsPath);
  } catch (const std::exception& e) {
    ofLogWarning() << "Failed to read presets: " << e.what();
    return;
  }
  auto parsePreset = [&](const ofJson& entry, Preset& preset) {
    if (!entry.is_object()) {
      return;
    }
    preset.hasData = entry.value("hasData", false);
    std::string sourceStr = entry.value("source", "video");
    if (sourceStr == "midi") preset.source = Source::MidiPattern;
    preset.lineWidth = entry.value("lineWidth", preset.lineWidth);
    preset.angleDeg = entry.value("angleDeg", preset.angleDeg);
    preset.rotationSpeedDegPerSec = entry.value("rotationSpeedDegPerSec", preset.rotationSpeedDegPerSec);
    preset.verticalSpeedPxPerSec = entry.value("verticalSpeedPxPerSec", preset.verticalSpeedPxPerSec);
    preset.pauseAutomation = entry.value("pauseAutomation", preset.pauseAutomation);
    preset.automationPhase = entry.value("automationPhase", preset.automationPhase);
    preset.lineFalloff = entry.value("lineFalloff", preset.lineFalloff);
    if (entry.contains("lineColor") && entry["lineColor"].is_array()) {
      const auto& col = entry["lineColor"];
      if (col.size() >= 3) {
        preset.lineColor = ofFloatColor(
          col[0].get<float>(),
          col[1].get<float>(),
          col[2].get<float>(),
          col.size() > 3 ? col[3].get<float>() : 1.0f
        );
      }
    }
    preset.lineHue = entry.value("lineHue", preset.lineHue);
    preset.lineSat = entry.value("lineSat", preset.lineSat);
    preset.lineBri = entry.value("lineBri", preset.lineBri);
    if (!entry.contains("lineHue") || !entry.contains("lineSat") || !entry.contains("lineBri")) {
      ofColor c(preset.lineColor);
      float h = 0.0f, s = 0.0f, b = 0.0f;
      c.getHsb(h, s, b);
      preset.lineHue = h;
      preset.lineSat = s;
      preset.lineBri = b;
    }
    preset.serpentine = entry.value("serpentine", preset.serpentine);
    preset.verticalFlip = entry.value("verticalFlip", preset.verticalFlip);
    preset.columnOffset = entry.value("columnOffset", preset.columnOffset);
    preset.targetSendFps = entry.value("targetSendFps", preset.targetSendFps);
    preset.brightnessScalar = entry.value("brightnessScalar", preset.brightnessScalar);
    preset.sendOsc = entry.value("sendOsc", preset.sendOsc);
    preset.blackout = entry.value("blackout", preset.blackout);
  };

  auto parseSlot = [&](const ofJson& slot, PresetSlot& out) {
    if (!slot.is_object()) {
      return;
    }
    out.hasData = slot.value("hasData", false);
    if (!slot.contains("satellites") || !slot["satellites"].is_array()) {
      return;
    }
    const auto& sats = slot["satellites"];
    const size_t count = std::min(sats.size(), static_cast<size_t>(kSatelliteCount));
    for (size_t i = 0; i < count; ++i) {
      parsePreset(sats[i], out.satellites[i]);
    }
  };

  if (root.contains("presets") && root["presets"].is_array()) {
    const auto& arr = root["presets"];
    const size_t count = std::min(arr.size(), static_cast<size_t>(kPresetCount));
    bool isNewFormat = false;
    for (size_t i = 0; i < count; ++i) {
      if (arr[i].is_object() && arr[i].contains("satellites")) {
        isNewFormat = true;
        break;
      }
    }
    if (isNewFormat) {
      for (size_t i = 0; i < count; ++i) {
        parseSlot(arr[i], presets[i]);
      }
      return;
    }
  }

  if (root.contains("presetsByTarget") && root["presetsByTarget"].is_array()) {
    const auto& targets = root["presetsByTarget"];
    const size_t targetCount = std::min(targets.size(), static_cast<size_t>(kSatelliteCount + 1));
    for (int slot = 0; slot < kPresetCount; ++slot) {
      presets[slot].hasData = true;
      for (int sat = 0; sat < kSatelliteCount; ++sat) {
        const size_t targetIdx = static_cast<size_t>(sat + 1);
        if (targetIdx >= targetCount || !targets[targetIdx].is_array()) {
          continue;
        }
        const auto& arr = targets[targetIdx];
        if (slot < static_cast<int>(arr.size())) {
          parsePreset(arr[slot], presets[slot].satellites[sat]);
        }
      }
    }
    return;
  }

  if (root.contains("presets") && root["presets"].is_array()) {
    const auto& arr = root["presets"];
    const size_t count = std::min(arr.size(), static_cast<size_t>(kPresetCount));
    for (size_t i = 0; i < count; ++i) {
      Preset preset;
      parsePreset(arr[i], preset);
      presets[i].hasData = preset.hasData;
      for (int sat = 0; sat < kSatelliteCount; ++sat) {
        presets[i].satellites[sat] = preset;
      }
    }
  }
}

//--------------------------------------------------------------
void ofApp::savePresetsToFile() const {
  if (presetsPath.empty()) {
    return;
  }
  ofJson root;
  root["presets"] = ofJson::array();
  for (int i = 0; i < kPresetCount; ++i) {
    const PresetSlot& slot = presets[i];
    ofJson slotEntry;
    slotEntry["hasData"] = slot.hasData;
    slotEntry["satellites"] = ofJson::array();
    for (int sat = 0; sat < kSatelliteCount; ++sat) {
      const Preset& preset = slot.satellites[sat];
      ofJson entry;
      entry["hasData"] = preset.hasData;
      entry["source"] = (preset.source == Source::Video) ? "video" : "midi";
      entry["lineWidth"] = preset.lineWidth;
      entry["angleDeg"] = preset.angleDeg;
      entry["rotationSpeedDegPerSec"] = preset.rotationSpeedDegPerSec;
      entry["verticalSpeedPxPerSec"] = preset.verticalSpeedPxPerSec;
      entry["pauseAutomation"] = preset.pauseAutomation;
      entry["automationPhase"] = preset.automationPhase;
      entry["lineFalloff"] = preset.lineFalloff;
      entry["lineColor"] = {preset.lineColor.r, preset.lineColor.g, preset.lineColor.b, preset.lineColor.a};
      entry["lineHue"] = preset.lineHue;
      entry["lineSat"] = preset.lineSat;
      entry["lineBri"] = preset.lineBri;
      entry["serpentine"] = preset.serpentine;
      entry["verticalFlip"] = preset.verticalFlip;
      entry["columnOffset"] = preset.columnOffset;
      entry["targetSendFps"] = preset.targetSendFps;
      entry["brightnessScalar"] = preset.brightnessScalar;
      entry["sendOsc"] = preset.sendOsc;
      entry["blackout"] = preset.blackout;
      slotEntry["satellites"].push_back(entry);
    }
    root["presets"].push_back(slotEntry);
  }
  ofSavePrettyJson(presetsPath, root);
}

//--------------------------------------------------------------
void ofApp::storePreset(int idx) {
  if (idx < 0 || idx >= kPresetCount) {
    return;
  }
  PresetSlot& slot = presets[idx];
  slot.hasData = true;
  for (int sat = 0; sat < kSatelliteCount; ++sat) {
    slot.satellites[sat] = satellitePresets[sat];
    slot.satellites[sat].hasData = true;
  }
  savePresetsToFile();
}

//--------------------------------------------------------------
void ofApp::recallPreset(int idx) {
  if (idx < 0 || idx >= kPresetCount) {
    return;
  }
  const PresetSlot& slot = presets[idx];
  if (!slot.hasData) {
    return;
  }
  for (int sat = 0; sat < kSatelliteCount; ++sat) {
    satellitePresets[sat] = slot.satellites[sat];
  }
  const int activeNow = firstActiveSatellite();
  if (activeNow >= 0) {
    applyPresetToCurrent(satellitePresets[activeNow]);
  }
  forceSendAllOnce = true;
}

//--------------------------------------------------------------
void ofApp::openMidiPortByIndex(int idx) {
#if HAS_OFXMIDI
  if (midiPorts.empty()) { midiStatus = "MIDI: No ports"; return; }
  int maxIdx = std::max(0, (int)midiPorts.size() - 1);
  int sel = ofClamp(idx, 0, maxIdx);
  if (midiIn.isOpen()) { midiIn.closePort(); midiIn.removeListener(this); }
  midiIn.openPort(sel);
  midiIn.addListener(this);
  midiIn.ignoreTypes(false, false, false);
  midiDeviceName = midiIn.getName();
  midiEnabled = true;
  midiPortName = midiPorts[sel];
  midiStatus = std::string("MIDI: Connected ") + midiDeviceName;
#endif
}

#if HAS_OFXMIDI
//--------------------------------------------------------------
void ofApp::newMidiMessage(ofxMidiMessage &msg) {
  // Map CCs similarly to cylinder-led-controller
  if (msg.status == MIDI_CONTROL_CHANGE) {
    bool settingsChanged = false;
    int cc = msg.control;
    int v = msg.value; // 0..127
    auto toRange = [&](float lo, float hi){ return ofMap(v, 0, 127, lo, hi, true); };
    auto toBipolar = [&](){ return ofMap(v, 0, 127, -1.0f, 1.0f, true); };
    switch (cc) {
      // Sliders 0..7
      case 0:  lineWidth = toRange(0.5f, 12.0f); settingsChanged = true; break;
      case 1:  angleDeg = toRange(0.0f, 90.0f); settingsChanged = true; break;
      case 2:  rotationSpeedDegPerSec = toBipolar() * 360.0f; settingsChanged = true; break;
      case 3:  verticalSpeedPxPerSec = toBipolar() * 20.0f; settingsChanged = true; break;
      case 4:  automationPhase = toRange(0.0f, 1.0f); settingsChanged = true; break;
      case 5:  lineHue = toRange(0.0f, 255.0f); lineColor = hsbToFloatColor(lineHue, lineSat, lineBri); settingsChanged = true; break;
      case 6:  lineSat = toRange(0.0f, 255.0f); lineColor = hsbToFloatColor(lineHue, lineSat, lineBri); settingsChanged = true; break;
      case 7:  lineBri = toRange(0.0f, 255.0f); lineColor = hsbToFloatColor(lineHue, lineSat, lineBri); settingsChanged = true; break;
      // Knobs 16..23 mirror
      case 16: lineFalloff = toRange(0.0f, 1.0f); settingsChanged = true; break;
      // case 17: angleDeg = toRange(0.0f, 90.0f); break;
      // case 18: rotationSpeedDegPerSec = toBipolar() * 360.0f; break;
      // case 19: verticalSpeedPxPerSec = toBipolar() * 20.0f; break;
      // case 20: automationPhase = toRange(0.0f, 1.0f); break;
      // case 21: { ofColor c(lineColor); float h,s,b; c.getHsb(h,s,b); h = ofMap(v,0,127,0,255,true); c.setHsb((unsigned char)h,(unsigned char)s,(unsigned char)b); lineColor = ofFloatColor(c); } break;
      // case 22: { ofColor c(lineColor); float h,s,b; c.getHsb(h,s,b); s = ofMap(v,0,127,0,255,true); c.setHsb((unsigned char)h,(unsigned char)s,(unsigned char)b); lineColor = ofFloatColor(c); } break;
      // case 23: { ofColor c(lineColor); float h,s,b; c.getHsb(h,s,b); b = ofMap(v,0,127,0,255,true); c.setHsb((unsigned char)h,(unsigned char)s,(unsigned char)b); lineColor = ofFloatColor(c); } break;
      // Transport controls
      case 41: if (v > 0) { pauseAutomation = true; settingsChanged = true; } break;  // PLAY
      case 42: if (v > 0) { pauseAutomation = false; settingsChanged = true; } break; // STOP
      case 45: if (v > 0) { blackout = !blackout; settingsChanged = true; } break;    // REC
      case 46: if (v > 0) { // CYCLE toggles source
        source = (source == Source::Video) ? Source::MidiPattern : Source::Video;
        settingsChanged = true;
      } break;
      // S buttons 1-8 toggle satellites
      case 32: case 33: case 34: case 35: case 36: case 37: case 38: case 39:
        if (v > 0) {
          const int activeBefore = firstActiveSatellite();
          toggleSatelliteActive(cc - 32);
          if (activeBefore < 0) {
            const int activeNow = firstActiveSatellite();
            if (activeNow >= 0) {
              applyPresetToCurrent(satellitePresets[activeNow]);
            }
          }
        }
        break;
      // M buttons 1-8 store presets
      case 48: case 49: case 50: case 51: case 52: case 53: case 54: case 55:
        if (v > 0) storePreset(cc - 48);
        break;
      // R buttons 1-8 recall presets
      case 64: case 65: case 66: case 67: case 68: case 69: case 70: case 71:
        if (v > 0) recallPreset(cc - 64);
        break;
      default: break;
    }
    if (settingsChanged) {
      applyCurrentToActiveSatellites();
    }
  }
}
#endif
