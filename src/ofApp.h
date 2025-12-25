#pragma once

#include "ofMain.h"
#include <atomic>
#include <array>
#include "LedMapper.h"
//#if __has_include("ofxMidi.h")
#include "ofxMidi.h"
#define HAS_OFXMIDI 1
//#else
//#define HAS_OFXMIDI 0
//#endif

class ofApp : public ofBaseApp
#if HAS_OFXMIDI
            , public ofxMidiListener
#endif
{
public:
  void setup() override;
  void update() override;
  void draw() override;
  void exit() override;

  void keyPressed(int key) override;
  void mousePressed(int x, int y, int button) override;
  void mouseDragged(int x, int y, int button) override;
  void mouseReleased(int x, int y, int button) override;
  void dragEvent(ofDragInfo dragInfo) override;
#if HAS_OFXMIDI
  void newMidiMessage(ofxMidiMessage &msg);
#endif

private:
  // Source selection
  enum class Source { VideoImage, MidiPattern, Webcam };
  Source source = Source::VideoImage;

  // Video/image pipeline (from video-to-LED-matrix)
  void updateDownsample();
  // Pattern generator (from cylinder-led-controller)
  void drawLinePatternPixels();
  void buildPayload();
  void drawPreviews();
  void drawGridPreview(float x, float y, float scale);
  void drawUiText(float x, float y);
  void drawSatelliteUi(float x, float y);
  void loadVideo(const std::string& path);
  void loadVideoDialog();
  
  // MIDI
  void setupMidi();
  void openMidiPortByIndex(int idx);
  void toggleSatelliteActive(int idx);
  void setSatelliteActive(int idx, bool active);
  void syncMidiLedState();
  void setupMidiOut();
  static constexpr int kCols = 12;
  static constexpr int kRows = 19;
  static constexpr int kNumLeds = kCols * kRows;
  static constexpr int kPayloadSize = kNumLeds * 3;
  // Video/image
  ofVideoPlayer video;
  bool videoLoaded = false;
  ofImage image;
  bool imageLoaded = false;
  bool paused = false; // video pause
  int lastVideoFrame = -1;
  uint32_t lastVideoFrameMillis = 0;
  // Webcam
  ofVideoGrabber webcam;
  bool webcamReady = false;
  int webcamWidth = 640;
  int webcamHeight = 480;

  // Downsampled grid
  ofFbo downsampleFbo;
  ofPixels downsamplePixels;

  // Line/pattern params (from cylinder-led-controller)
  ofFloatColor lineColor = ofFloatColor::fromHsb(160.0f/255.0f, 1.0f, 1.0f);
  float lineHue = 160.0f;                // 0..255
  float lineSat = 255.0f;                // 0..255
  float lineBri = 255.0f;                // 0..255
  float lineWidth = 2.0f;                 // px in grid-space
  float angleDeg = 0.0f;                  // 0..90
  float rotationSpeedDegPerSec = 30.0f;   // -360..360
  float verticalSpeedPxPerSec = 2.0f;     // -20..20 in grid px/s
  bool pauseAutomation = false;
  float automationPhase = 0.0f;           // 0..1
  float lineFalloff = 0.0f;               // 0..1 (soft -> hard edge)
  float globalTime = 0.0f;

  // Mapping
  LedMapper mapper;
  bool serpentine = false;
  bool verticalFlip = false;
  int columnOffset = 0;

  // Sender timing
  float targetSendFps = 30.0f;
  float brightnessScalar = 0.8f;
  float brightnessGamma = 8.0f;
  bool sendOsc = true;                 // allow pausing sender via MIDI
  bool blackout = false;

  std::atomic<uint64_t> framesSent{0};
  std::atomic<uint64_t> framesDropped{0};

  std::vector<uint8_t> payload;
  std::vector<uint8_t> serialFrame;
  std::vector<uint8_t> rlePayload;

  // UI interactions
  ofRectangle brightnessSliderRect;
  bool brightnessSliderActive = false;

  // MIDI state
  bool midiEnabled = false;
  std::string midiDeviceName;
#if HAS_OFXMIDI
  ofxMidiIn midiIn;
#endif
  std::vector<std::string> midiPorts;
  std::vector<std::string> midiOutPorts;
  std::string midiStatus = "MIDI: Not detected";
  std::string midiPortName;
#if HAS_OFXMIDI
  ofxMidiOut midiOut;
#endif
  bool midiOutEnabled = false;

  // Serial (ESP32 host bridge)
  ofSerial serial;
  bool serialReady = false;
  std::string serialStatus = "Serial: Not connected";
  std::string serialDevice;
  int serialBaud = 115200;
  int targetId = 0; // 0 = broadcast, 1-9 = satellites
  uint32_t lastSerialReconnectMs = 0;
  uint32_t serialReconnectDelayMs = 1000;
  uint32_t lastSerialAttemptMs = 0;
  float maxSerialFps = 0.0f;
  uint32_t serialConsecutiveErrors = 0;
  uint32_t serialLastOkMs = 0;
  uint32_t serialBackoffUntilMs = 0;
  uint32_t serialKeepaliveIntervalMs = 500;
  uint32_t lastKeepaliveMs = 0;

  struct Preset {
    bool hasData = false;
    Source source = Source::VideoImage;
    float lineWidth = 2.0f;
    float angleDeg = 0.0f;
    float rotationSpeedDegPerSec = 30.0f;
    float verticalSpeedPxPerSec = 2.0f;
    bool pauseAutomation = false;
    float automationPhase = 0.0f;
    float lineFalloff = 0.0f;
    ofFloatColor lineColor = ofFloatColor::fromHsb(160.0f/255.0f, 1.0f, 1.0f);
    float lineHue = 160.0f;
    float lineSat = 255.0f;
    float lineBri = 255.0f;
    bool serpentine = false;
    bool verticalFlip = false;
    int columnOffset = 0;
    float targetSendFps = 30.0f;
    float brightnessScalar = 0.8f;
    bool sendOsc = true;
    bool blackout = false;
  };

  static constexpr int kPresetCount = 8;
  static constexpr int kSatelliteCount = 8;
  struct PresetSlot {
    bool hasData = false;
    std::array<Preset, kSatelliteCount> satellites;
  };
  std::array<PresetSlot, kPresetCount> presets;
  std::array<Preset, kSatelliteCount> satellitePresets;
  std::array<bool, kSatelliteCount> satelliteActive;
  std::array<uint32_t, kSatelliteCount> lastSendMillisBySlot;
  bool forceSendAllOnce = false;
  std::array<int, kSatelliteCount> satelliteIds;
  std::array<std::string, kSatelliteCount> satelliteNames;
  std::string satellitesPath;

  void loadPresetsFromFile();
  void savePresetsToFile() const;
  void storePreset(int idx);
  void recallPreset(int idx);
  void loadSatelliteConfig();
  void initSatellitePresets();
  void applyCurrentToPreset(Preset& preset) const;
  void applyPresetToCurrent(const Preset& preset);
  void applyCurrentToActiveSatellites();
  int firstActiveSatellite() const;

  void sendFrameIfDue();
  bool setupSerial();
  void sendSerialFrame(uint8_t target);
  void sendSerialKeepalive();
  void encodeRlePayload();
  std::string presetsPath;
};
