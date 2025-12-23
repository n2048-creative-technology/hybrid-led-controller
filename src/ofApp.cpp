#include "ofApp.h"
#include <algorithm>

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
  setupOscSenders();
  ackReceiver.setup(ackPort);

  // Optional default video (put in bin/data as video.mp4)
  loadVideo("video.mp4");

  // MIDI setup
  setupMidi();
  presetsPath = ofToDataPath(kPresetFilename, true);
  loadPresetsFromFile();
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
  handleOscAcks();
  sendFrameIfDue();
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
void ofApp::setupOscSenders() {
  oscHosts.clear();
  oscSenders.clear();

  const std::string configPath = ofToDataPath("osc_devices.txt", true);
  if (ofFile::doesFileExist(configPath)) {
    ofBuffer buffer = ofBufferFromFile(configPath);
    for (const auto& line : buffer.getLines()) {
      std::string host = ofTrim(line);
      if (!host.empty() && host[0] != '#') {
        oscHosts.push_back(host);
      }
    }
  }

  if (oscHosts.empty()) {
    oscHosts.push_back("broadcast");
  }

  for (const auto& host : oscHosts) {
    ofxOscSenderSettings settings;
    const bool looksLikeBroadcast = host == "broadcast" || host == "255.255.255.255" ||
                                    ofIsStringInString(host, ".255");
    settings.host = (host == "broadcast") ? "255.255.255.255" : host;
    settings.broadcast = looksLikeBroadcast;
    settings.port = oscPort;
    ofxOscSender sender;
    if (sender.setup(settings)) {
      oscSenders.push_back(std::move(sender));
    } else {
      ofLogWarning() << "OSC sender setup failed for host: " << settings.host
                     << " port: " << settings.port;
    }
  }

  oscReady = !oscSenders.empty();
}

//--------------------------------------------------------------
void ofApp::handleOscAcks() {
  const uint32_t now = ofGetElapsedTimeMillis();
  while (ackReceiver.hasWaitingMessages()) {
    ofxOscMessage msg;
    ackReceiver.getNextMessage(msg);
    if (msg.getAddress() != "/leds_ack") {
      continue;
    }
    std::string host = msg.getRemoteHost();
    if (host.empty()) {
      host = "unknown";
    }
    lastSeenByHost[host] = now;
  }

  for (auto it = lastSeenByHost.begin(); it != lastSeenByHost.end();) {
    if (now - it->second > activeDeviceTtlMs) {
      it = lastSeenByHost.erase(it);
    } else {
      ++it;
    }
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
  if (!sendOsc) {
    return;
  }
  const float effectiveFps = std::max(1.0f, targetSendFps);
  uint32_t intervalMs = static_cast<uint32_t>(1000.0f / effectiveFps);
  uint32_t now = ofGetElapsedTimeMillis();
  if (now - lastSendMillis < intervalMs) {
    return;
  }
  lastSendMillis = now;

  if (!oscReady) {
    framesDropped.fetch_add(1);
    return;
  }
  buildPayload();
  sendOscFrame();
}

//--------------------------------------------------------------
void ofApp::sendOscFrame() {
  ofxOscMessage message;
  message.setAddress(oscAddress);
  ofBuffer blob(reinterpret_cast<const char*>(payload.data()), payload.size());
  message.addBlobArg(blob);

  std::lock_guard<std::mutex> lock(oscMutex);
  for (auto& sender : oscSenders) {
    if (sender.isReady()) {
      sender.sendMessage(message, false);
    }
  }
  framesSent.fetch_add(1);
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
  const bool connected = oscReady;
  const std::string statusText = connected ? "OSC READY" : "NO OSC HOSTS";
  const ofColor statusColor = connected ? ofColor(0, 200, 90) : ofColor(220, 60, 60);

  std::stringstream ss;
  ss << "Mode: ";
  ss << (source == Source::Video ? "Video" : "MIDI Pattern") << "\n";
  ss << "OSC: " << oscAddress << " @ " << oscPort << "  Ack: " << ackPort << "\n";
  ss << "Configured: " << oscHosts.size() << "  Ready: " << oscSenders.size()
     << "  Active: " << lastSeenByHost.size() << "\n";
  if (!oscHosts.empty()) {
    ss << "Hosts: ";
    for (size_t i = 0; i < oscHosts.size(); ++i) {
      if (i > 0) ss << ", ";
      ss << oscHosts[i];
    }
    ss << "\n";
  }
  ss << "Send FPS: " << targetSendFps << "  Frames sent: " << framesSent.load()
     << " dropped: " << framesDropped.load() << "\n";
  ss << "Brightness: " << brightnessScalar << "  Mapping: " << (serpentine ? "serpentine" : "linear") << "\n";
  ss << "Vertical flip: " << (verticalFlip ? "on" : "off") << "  Column offset: " << columnOffset << "\n";
  ss << "Send OSC: " << (sendOsc ? "on" : "off") << "  Blackout: " << (blackout ? "on" : "off") << "\n";
  ss << midiStatus << "\n";
  ss << "Controls:\n";
  ss << "  Space play/pause (video)  |  L load file  |  F fullscreen\n";
  ss << "  M switch mode (video/midi)  |  1/2 mapping linear/serpentine  |  V vertical flip  |  [ ] rotate  |  R reset\n";
  ss << "  Up/Down send fps  |  +/- brightness\n";
  ss << "  Line: A/Z angle  |  W/S width  |  O/P rot speed  |  K/I vertical speed  |  Q pause anim\n";
  ss << "  Edit bin/data/osc_devices.txt to set ESP32 IPs\n";
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
  if (!root.contains("presets") || !root["presets"].is_array()) {
    return;
  }
  const auto& arr = root["presets"];
  const size_t count = std::min(arr.size(), static_cast<size_t>(kPresetCount));
  for (size_t i = 0; i < count; ++i) {
    const auto& entry = arr[i];
    if (!entry.is_object()) {
      continue;
    }
    Preset preset;
    preset.hasData = entry.value("hasData", false);
    std::string sourceStr = entry.value("source", "video");
    if (sourceStr == "midi") preset.source = Source::MidiPattern;
    preset.lineWidth = entry.value("lineWidth", preset.lineWidth);
    preset.angleDeg = entry.value("angleDeg", preset.angleDeg);
    preset.rotationSpeedDegPerSec = entry.value("rotationSpeedDegPerSec", preset.rotationSpeedDegPerSec);
    preset.verticalSpeedPxPerSec = entry.value("verticalSpeedPxPerSec", preset.verticalSpeedPxPerSec);
    preset.pauseAutomation = entry.value("pauseAutomation", preset.pauseAutomation);
    preset.automationPhase = entry.value("automationPhase", preset.automationPhase);
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
    preset.serpentine = entry.value("serpentine", preset.serpentine);
    preset.verticalFlip = entry.value("verticalFlip", preset.verticalFlip);
    preset.columnOffset = entry.value("columnOffset", preset.columnOffset);
    preset.targetSendFps = entry.value("targetSendFps", preset.targetSendFps);
    preset.brightnessScalar = entry.value("brightnessScalar", preset.brightnessScalar);
    preset.sendOsc = entry.value("sendOsc", preset.sendOsc);
    preset.blackout = entry.value("blackout", preset.blackout);
    presets[i] = preset;
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
    const Preset& preset = presets[i];
    ofJson entry;
    entry["hasData"] = preset.hasData;
    entry["source"] = (preset.source == Source::Video) ? "video" : "midi";
    entry["lineWidth"] = preset.lineWidth;
    entry["angleDeg"] = preset.angleDeg;
    entry["rotationSpeedDegPerSec"] = preset.rotationSpeedDegPerSec;
    entry["verticalSpeedPxPerSec"] = preset.verticalSpeedPxPerSec;
    entry["pauseAutomation"] = preset.pauseAutomation;
    entry["automationPhase"] = preset.automationPhase;
    entry["lineColor"] = {preset.lineColor.r, preset.lineColor.g, preset.lineColor.b, preset.lineColor.a};
    entry["serpentine"] = preset.serpentine;
    entry["verticalFlip"] = preset.verticalFlip;
    entry["columnOffset"] = preset.columnOffset;
    entry["targetSendFps"] = preset.targetSendFps;
    entry["brightnessScalar"] = preset.brightnessScalar;
    entry["sendOsc"] = preset.sendOsc;
    entry["blackout"] = preset.blackout;
    root["presets"].push_back(entry);
  }
  ofSavePrettyJson(presetsPath, root);
}

//--------------------------------------------------------------
void ofApp::storePreset(int idx) {
  if (idx < 0 || idx >= kPresetCount) {
    return;
  }
  Preset preset;
  preset.hasData = true;
  preset.source = source;
  preset.lineWidth = lineWidth;
  preset.angleDeg = angleDeg;
  preset.rotationSpeedDegPerSec = rotationSpeedDegPerSec;
  preset.verticalSpeedPxPerSec = verticalSpeedPxPerSec;
  preset.pauseAutomation = pauseAutomation;
  preset.automationPhase = automationPhase;
  preset.lineColor = lineColor;
  preset.serpentine = serpentine;
  preset.verticalFlip = verticalFlip;
  preset.columnOffset = columnOffset;
  preset.targetSendFps = targetSendFps;
  preset.brightnessScalar = brightnessScalar;
  preset.sendOsc = sendOsc;
  preset.blackout = blackout;
  presets[idx] = preset;
  savePresetsToFile();
}

//--------------------------------------------------------------
void ofApp::recallPreset(int idx) {
  if (idx < 0 || idx >= kPresetCount) {
    return;
  }
  const Preset& preset = presets[idx];
  if (!preset.hasData) {
    return;
  }
  source = preset.source;
  lineWidth = preset.lineWidth;
  angleDeg = preset.angleDeg;
  rotationSpeedDegPerSec = preset.rotationSpeedDegPerSec;
  verticalSpeedPxPerSec = preset.verticalSpeedPxPerSec;
  pauseAutomation = preset.pauseAutomation;
  automationPhase = preset.automationPhase;
  lineColor = preset.lineColor;
  serpentine = preset.serpentine;
  verticalFlip = preset.verticalFlip;
  columnOffset = preset.columnOffset;
  targetSendFps = ofClamp(preset.targetSendFps, 1.0f, 120.0f);
  brightnessScalar = ofClamp(preset.brightnessScalar, 0.0f, 1.0f);
  sendOsc = preset.sendOsc;
  blackout = preset.blackout;
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
      // case 16: lineWidth = toRange(0.5f, 12.0f); break;
      // case 17: angleDeg = toRange(0.0f, 90.0f); break;
      // case 18: rotationSpeedDegPerSec = toBipolar() * 360.0f; break;
      // case 19: verticalSpeedPxPerSec = toBipolar() * 20.0f; break;
      // case 20: automationPhase = toRange(0.0f, 1.0f); break;
      // case 21: { ofColor c(lineColor); float h,s,b; c.getHsb(h,s,b); h = ofMap(v,0,127,0,255,true); c.setHsb((unsigned char)h,(unsigned char)s,(unsigned char)b); lineColor = ofFloatColor(c); } break;
      // case 22: { ofColor c(lineColor); float h,s,b; c.getHsb(h,s,b); s = ofMap(v,0,127,0,255,true); c.setHsb((unsigned char)h,(unsigned char)s,(unsigned char)b); lineColor = ofFloatColor(c); } break;
      // case 23: { ofColor c(lineColor); float h,s,b; c.getHsb(h,s,b); b = ofMap(v,0,127,0,255,true); c.setHsb((unsigned char)h,(unsigned char)s,(unsigned char)b); lineColor = ofFloatColor(c); } break;
      // Transport controls
      case 41: if (v > 0) pauseAutomation = true; break;  // PLAY
      case 42: if (v > 0) pauseAutomation = false; break; // STOP
      case 45: if (v > 0) blackout = !blackout; break;    // REC
      case 46: if (v > 0) { // CYCLE toggles source
        source = (source == Source::Video) ? Source::MidiPattern : Source::Video;
      } break;
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
  }
}
#endif
