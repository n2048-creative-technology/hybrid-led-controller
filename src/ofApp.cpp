#include "ofApp.h"
#include <algorithm>

namespace {
uint16_t crc16Update(uint16_t c, uint8_t data) {
  c ^= (uint16_t)data << 8;
  for (uint8_t i = 0; i < 8; i++) {
    if (c & 0x8000) {
      c = (c << 1) ^ 0x1021;
    } else {
      c <<= 1;
    }
  }
  return c;
}

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
  packet.reserve(2 + 1 + 2 + 2 + kPayloadSize + 2);

  startSenderThread();
  refreshPorts();
  autoConnectPort();

  // Optional default video (put in bin/data as video.mp4)
  loadVideo("video.mp4");

  // MIDI setup
  setupMidi();
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

  // Serial auto connect/reconnect logic (same as video-to-LED-matrix)
  const float now = ofGetElapsedTimef();
  if (!serialConnected.load() && autoConnect) {
    autoConnectPort();
  }
  if (serialConnected.load() && !connectedPort.empty()) {
    if (!ofFile::doesFileExist(connectedPort)) {
      closeSerial();
    }
  }
  if (serialConnected.load()) {
    uint32_t sinceWrite = now - lastWriteMillis.load();
    if (sinceWrite > 3000 && now - serialConnectMillis > 2000) {
      closeSerial();
    }
  } else if (autoConnect) {
    uint32_t nowMs = ofGetElapsedTimeMillis();
    if (nowMs - lastReconnectAttemptMillis > reconnectIntervalMs) {
      lastReconnectAttemptMillis = nowMs;
      if (!lastKnownPort.empty() && ofFile::doesFileExist(lastKnownPort)) {
        {
          std::lock_guard<std::mutex> lock(connectMutex);
          pendingConnectPort = lastKnownPort;
        }
        requestConnect.store(true);
        senderCv.notify_all();
      }
    }
  }

  updateDownsample();
  sendFrameIfDue();
}

//--------------------------------------------------------------
void ofApp::draw() {
  drawPreviews();
  drawUiText(20, ofGetHeight() - 220);
}

//--------------------------------------------------------------
void ofApp::exit() {
  stopSenderThread();
  if (videoLoaded) {
    video.stop();
  }
}

//--------------------------------------------------------------
void ofApp::refreshPorts() {
  devices = serial.getDeviceList();
}

//--------------------------------------------------------------
bool ofApp::autoConnectPort() {
  if (serialConnected.load()) return true;
  if (!lastKnownPort.empty() && ofFile::doesFileExist(lastKnownPort)) {
    {
      std::lock_guard<std::mutex> lock(connectMutex);
      pendingConnectPort = lastKnownPort;
    }
    requestConnect.store(true);
    senderCv.notify_all();
    return true;
  }
  if (devices.empty()) refreshPorts();
  for (size_t i = 0; i < devices.size(); ++i) {
    const std::string name = devices[i].getDeviceName();
    const std::string path = devices[i].getDevicePath();
    const bool looksLikeSerial =
        name.find("ttyACM") != std::string::npos || name.find("ttyUSB") != std::string::npos ||
        name.find("usbmodem") != std::string::npos || name.find("COM") != std::string::npos ||
        path.find("ttyACM") != std::string::npos || path.find("ttyUSB") != std::string::npos ||
        path.find("usbmodem") != std::string::npos || path.find("COM") != std::string::npos;
    if (looksLikeSerial) {
      if (connectToPortIndex(static_cast<int>(i))) return true;
    }
  }
  if (!devices.empty()) return connectToPortIndex(0);
  return false;
}

//--------------------------------------------------------------
bool ofApp::connectToPortIndex(int idx) {
  if (idx < 0 || idx >= static_cast<int>(devices.size())) return false;
  serialConnected.store(false);
  selectedPortIndex = idx;
  connectedPort = devices[idx].getDevicePath();
  lastKnownPort = connectedPort;
  {
    std::lock_guard<std::mutex> lock(connectMutex);
    pendingConnectPort = connectedPort;
  }
  requestConnect.store(true);
  senderCv.notify_all();
  return true;
}

//--------------------------------------------------------------
void ofApp::closeSerial() {
  serialConnected.store(false);
  if (!connectedPort.empty()) lastKnownPort = connectedPort;
  connectedPort.clear();
  selectedPortIndex = -1;
  requestClose.store(true);
  senderCv.notify_all();
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

  ofColor base(lineColor);
  for (int s = 0; s < kCols; ++s) {
    for (int y = 0; y < kRows; ++y) {
      float ss = (s + 0.5f) / (float)kCols;
      float yy = (y + 0.5f) / (float)kRows;
      float v = ss * n.x + yy * n.y;
      float d = periodicDelta(v, phase, 1.0f);
      float intensity = ofClamp(1.0f - d / std::max(0.0001f, widthNorm), 0.0f, 1.0f);
      ofColor c = base * intensity;
      int yTop = (kRows - 1) - y;  // top-origin storage
      downsamplePixels.setColor(s, yTop, c);
    }
  }
}

//--------------------------------------------------------------
void ofApp::buildPayload() {
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
      uint8_t r = static_cast<uint8_t>(ofClamp(c.r * brightnessScalar, 0.0f, 255.0f));
      uint8_t g = static_cast<uint8_t>(ofClamp(c.g * brightnessScalar, 0.0f, 255.0f));
      uint8_t b = static_cast<uint8_t>(ofClamp(c.b * brightnessScalar, 0.0f, 255.0f));

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

//--------------------------------------------------------------
void ofApp::sendFrameIfDue() {
  if (!sendSerial) return;
  float bytesPerSecond = static_cast<float>(baud) / 10.0f;  // 8N1 framing
  float maxFps = bytesPerSecond / static_cast<float>(kPacketBytes);
  maxSendFps = maxFps;
  float effectiveFps = targetSendFps;
  if (maxFps > 0.1f && effectiveFps > maxFps) {
    effectiveFps = maxFps;
    sendFpsClamped = true;
  } else {
    sendFpsClamped = false;
  }
  uint32_t intervalMs = effectiveFps <= 0.1f ? 1000 : static_cast<uint32_t>(1000.0f / effectiveFps);
  uint32_t now = ofGetElapsedTimeMillis();
  if (now - lastSendMillis < intervalMs) return;
  lastSendMillis = now;
  if (!serialConnected.load()) {
    if (autoConnect) autoConnectPort();
    return;
  }
  if (now - serialConnectMillis < 1500) return;  // allow Arduino to reboot

  buildPayload();

  packet.clear();
  packet.push_back(kMagic0);
  packet.push_back(kMagic1);
  packet.push_back(kProtocolVersion);
  uint16_t len = kPayloadSize;
  packet.push_back(len & 0xFF);
  packet.push_back((len >> 8) & 0xFF);
  packet.push_back(frameId & 0xFF);
  packet.push_back((frameId >> 8) & 0xFF);
  packet.insert(packet.end(), payload.begin(), payload.end());

  uint16_t crc = 0xFFFF;
  for (size_t i = 2; i < packet.size(); ++i) {
    crc = crc16Update(crc, packet[i]);
  }
  packet.push_back(crc & 0xFF);
  packet.push_back((crc >> 8) & 0xFF);

  {
    std::lock_guard<std::mutex> lock(senderMutex);
    if (hasPendingPacket) {
      framesDropped.fetch_add(1);
    }
    pendingPacket = packet;
    hasPendingPacket = true;
    lastPacket = packet;
    hasLastPacket = true;
  }
  senderCv.notify_one();
  frameId++;
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
  const bool connected = serialConnected.load();
  const std::string statusText = connected ? "CONNECTED" : "DISCONNECTED";
  const ofColor statusColor = connected ? ofColor(0, 200, 90) : ofColor(220, 60, 60);

  std::stringstream ss;
  ss << "Mode: ";
  ss << (source == Source::Video ? "Video" : "MIDI Pattern") << "\n";
  ss << "Serial: " << (connected ? connectedPort : "[disconnected]") << " @ " << baud;
  ss << (autoConnect ? " (auto)" : " (manual)") << "\n";
  ss << "Send FPS: " << targetSendFps << "  Frames sent: " << framesSent.load()
     << " dropped: " << framesDropped.load() << "\n";
  ss << "Max FPS @ baud: " << maxSendFps << (sendFpsClamped ? " (clamped)" : "") << "\n";
  ss << "Brightness: " << brightnessScalar << "  Mapping: " << (serpentine ? "serpentine" : "linear") << "\n";
  ss << "Vertical flip: " << (verticalFlip ? "on" : "off") << "  Column offset: " << columnOffset << "\n";
  ss << "Send serial: " << (sendSerial ? "on" : "off") << "\n";
  ss << midiStatus << "\n";
  ss << "Controls:\n";
  ss << "  Space play/pause (video)  |  L load file  |  F fullscreen\n";
  ss << "  M switch mode (video/midi)  |  1/2 mapping linear/serpentine  |  V vertical flip  |  [ ] rotate  |  R reset\n";
  ss << "  Up/Down send fps  |  +/- brightness\n";
  ss << "  Line: A/Z angle  |  W/S width  |  O/P rot speed  |  K/I vertical speed  |  Q pause anim\n";
  ss << "  Drag-and-drop a video file onto window\n";
  const glm::vec2 textSize = estimateBitmapStringSize(ss.str(), charW, charH, lineH);

  float pad = 4.0f;
  float statusH = charH + pad * 2.0f;
  float statusW = (statusText.size() * charW) + pad * 3.0f + statusH;

  float gap = 10.0f;
  float blockW = std::max(statusW, textSize.x);
  float blockH = statusH + gap + textSize.y + sliderGap + sliderH;

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
  ofPopMatrix();

  brightnessSliderRect.set(drawX, drawY + sliderY * uiScale, blockW * uiScale, sliderH * uiScale);
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key) {
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
      if (source == Source::Video) source = Source::MidiPattern; else source = Source::Video;
      break;
    case '1': serpentine = false; break;
    case '2': serpentine = true; break;
    case 'v': case 'V': verticalFlip = !verticalFlip; break;
    case '[': columnOffset--; break;
    case ']': columnOffset++; break;
    case 'r': case 'R': columnOffset = 0; break;
    case '+': case '=': brightnessScalar = ofClamp(brightnessScalar + 0.05f, 0.0f, 1.0f); break;
    case '-': case '_': brightnessScalar = ofClamp(brightnessScalar - 0.05f, 0.0f, 1.0f); break;
    case OF_KEY_UP:   targetSendFps = std::min(120.0f, targetSendFps + 1.0f); break;
    case OF_KEY_DOWN: targetSendFps = std::max(1.0f, targetSendFps - 1.0f); break;

    // Line pattern controls
    case 'a': case 'A': angleDeg = ofClamp(angleDeg + 2.0f, 0.0f, 90.0f); break;
    case 'z': case 'Z': angleDeg = ofClamp(angleDeg - 2.0f, 0.0f, 90.0f); break;
    case 'w': case 'W': lineWidth = ofClamp(lineWidth + 0.5f, 0.5f, 12.0f); break;
    case 's': case 'S': lineWidth = ofClamp(lineWidth - 0.5f, 0.5f, 12.0f); break;
    case 'o': case 'O': rotationSpeedDegPerSec = ofClamp(rotationSpeedDegPerSec + 5.0f, -360.0f, 360.0f); break;
    case 'p': case 'P': rotationSpeedDegPerSec = ofClamp(rotationSpeedDegPerSec - 5.0f, -360.0f, 360.0f); break;
    case 'k': case 'K': verticalSpeedPxPerSec = ofClamp(verticalSpeedPxPerSec + 0.5f, -20.0f, 20.0f); break;
    case 'i': case 'I': verticalSpeedPxPerSec = ofClamp(verticalSpeedPxPerSec - 0.5f, -20.0f, 20.0f); break;
    case 'q': case 'Q': pauseAutomation = !pauseAutomation; break;
    default:
      if (key >= '0' && key <= '9') {
        int idx = key - '0';
        connectToPortIndex(idx);
      }
      break;
  }
}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button) {
  if (brightnessSliderRect.inside(static_cast<float>(x), static_cast<float>(y))) {
    brightnessSliderActive = true;
    float t = (x - brightnessSliderRect.x) / brightnessSliderRect.width;
    brightnessScalar = ofClamp(t, 0.0f, 1.0f);
  }
}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button) {
  if (brightnessSliderActive) {
    float t = (x - brightnessSliderRect.x) / brightnessSliderRect.width;
    brightnessScalar = ofClamp(t, 0.0f, 1.0f);
  }
}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button) { brightnessSliderActive = false; }

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo) { if (!dragInfo.files.empty()) loadVideo(dragInfo.files[0]); }

//--------------------------------------------------------------
void ofApp::startSenderThread() {
  senderStop.store(false);
  senderThread = std::thread(&ofApp::senderThreadLoop, this);
}

//--------------------------------------------------------------
void ofApp::stopSenderThread() {
  senderStop.store(true);
  senderCv.notify_all();
  if (senderThread.joinable()) senderThread.join();
  {
    std::lock_guard<std::mutex> serialLock(serialMutex);
    if (serial.isInitialized()) serial.close();
  }
  requestClose.store(false);
}

//--------------------------------------------------------------
void ofApp::senderThreadLoop() {
  while (!senderStop.load()) {
    std::vector<uint8_t> toSend;
    {
      std::unique_lock<std::mutex> lock(senderMutex);
      senderCv.wait_for(lock, std::chrono::milliseconds(100), [&] {
        return senderStop.load() || hasPendingPacket || requestClose.load() || requestConnect.load();
      });
      if (senderStop.load()) break;
      if (requestClose.load()) {
        requestClose.store(false);
        hasPendingPacket = false;
        lock.unlock();
        std::lock_guard<std::mutex> serialLock(serialMutex);
        if (serial.isInitialized()) serial.close();
        continue;
      }
      if (requestConnect.load()) {
        requestConnect.store(false);
        std::string port;
        {
          std::lock_guard<std::mutex> portLock(connectMutex);
          port = pendingConnectPort;
        }
        lock.unlock();
        if (!port.empty()) {
          std::lock_guard<std::mutex> serialLock(serialMutex);
          if (serial.isInitialized()) serial.close();
          bool ok = serial.setup(port, baud);
          if (ok && serial.isInitialized()) {
            serial.flush(true, true);
            connectedPort = port;
            serialConnected.store(true);
            serialConnectMillis = ofGetElapsedTimeMillis();
            ofLogNotice() << "Connected to " << connectedPort << " @ " << baud;
          } else {
            serial.close();
            serialConnected.store(false);
          }
        }
        continue;
      }
      if (hasPendingPacket) {
        toSend = pendingPacket;
        hasPendingPacket = false;
      } else {
        uint32_t now = ofGetElapsedTimeMillis();
        uint32_t lastWrite = lastWriteMillis.load();
        if (hasLastPacket && serialConnected.load() && now - lastWrite > 400) {
          toSend = lastPacket;
        }
      }
    }
    if (toSend.empty()) continue;
    if (!serialConnected.load()) { framesDropped.fetch_add(1); continue; }
    long written = OF_SERIAL_ERROR;
    {
      std::lock_guard<std::mutex> serialLock(serialMutex);
      if (serial.isInitialized()) {
        written = serial.writeBytes(toSend.data(), toSend.size());
      }
    }
    if (written < 0 || static_cast<size_t>(written) < toSend.size()) {
      framesDropped.fetch_add(1);
      serialConnected.store(false);
      requestClose.store(true);
      senderCv.notify_all();
      continue;
    }
    framesSent.fetch_add(1);
    lastWriteMillis.store(ofGetElapsedTimeMillis());
  }
}

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
  } else {
    midiEnabled = false;
    midiStatus = "MIDI: Not detected";
  }
#else
  midiEnabled = false;
  midiStatus = "MIDI: ofxMidi not installed";
#endif
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
    int cc = msg.control;
    int v = msg.value; // 0..127
    auto toRange = [&](float lo, float hi){ return ofMap(v, 0, 127, lo, hi, true); };
    auto toBipolar = [&](){ return ofMap(v, 0, 127, -1.0f, 1.0f, true); };
    switch (cc) {
      // Sliders 0..7
      case 0:  lineWidth = toRange(0.5f, 12.0f); break;
      case 1:  angleDeg = toRange(0.0f, 90.0f); break;
      case 2:  rotationSpeedDegPerSec = toBipolar() * 360.0f; break;
      case 3:  verticalSpeedPxPerSec = toBipolar() * 20.0f; break;
      case 4:  automationPhase = toRange(0.0f, 1.0f); break;
      case 5:  { ofColor c(lineColor); float h,s,b; c.getHsb(h,s,b); h = ofMap(v,0,127,0,255,true); c.setHsb((unsigned char)h,(unsigned char)s,(unsigned char)b); lineColor = ofFloatColor(c); } break;
      case 6:  { ofColor c(lineColor); float h,s,b; c.getHsb(h,s,b); s = ofMap(v,0,127,0,255,true); c.setHsb((unsigned char)h,(unsigned char)s,(unsigned char)b); lineColor = ofFloatColor(c); } break;
      case 7:  { ofColor c(lineColor); float h,s,b; c.getHsb(h,s,b); b = ofMap(v,0,127,0,255,true); c.setHsb((unsigned char)h,(unsigned char)s,(unsigned char)b); lineColor = ofFloatColor(c); } break;
      // Knobs 16..23 mirror
      case 16: lineWidth = toRange(0.5f, 12.0f); break;
      case 17: angleDeg = toRange(0.0f, 90.0f); break;
      case 18: rotationSpeedDegPerSec = toBipolar() * 360.0f; break;
      case 19: verticalSpeedPxPerSec = toBipolar() * 20.0f; break;
      case 20: automationPhase = toRange(0.0f, 1.0f); break;
      case 21: { ofColor c(lineColor); float h,s,b; c.getHsb(h,s,b); h = ofMap(v,0,127,0,255,true); c.setHsb((unsigned char)h,(unsigned char)s,(unsigned char)b); lineColor = ofFloatColor(c); } break;
      case 22: { ofColor c(lineColor); float h,s,b; c.getHsb(h,s,b); s = ofMap(v,0,127,0,255,true); c.setHsb((unsigned char)h,(unsigned char)s,(unsigned char)b); lineColor = ofFloatColor(c); } break;
      case 23: { ofColor c(lineColor); float h,s,b; c.getHsb(h,s,b); b = ofMap(v,0,127,0,255,true); c.setHsb((unsigned char)h,(unsigned char)s,(unsigned char)b); lineColor = ofFloatColor(c); } break;
      // Solo/Mute/Rec buttons examples
      case 32: if (v > 0) pauseAutomation = !pauseAutomation; break; // Solo 1
      case 48: if (v > 0) sendSerial = !sendSerial; break; // Mute 1 toggles serial send
      case 49: if (v > 0) { // Mute 2 cycles mode video/midi
        source = (source == Source::Video) ? Source::MidiPattern : Source::Video;
      } break;
      case 64: if (v > 0) autoConnect = true; break;   // Rec 1 enables auto-connect
      case 65: if (v > 0) { autoConnect = false; closeSerial(); } break; // Rec 2 disconnect
      default: break;
    }
  }
}
#endif
