#pragma once

#include "ofMain.h"
#include "ofxOsc.h"
#include <atomic>
#include <array>
#include <mutex>
#include <unordered_map>
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
  enum class Source { Video, MidiPattern };
  Source source = Source::Video;

  // Video pipeline (from video-to-LED-matrix)
  void updateDownsample();
  // Pattern generator (from cylinder-led-controller)
  void drawLinePatternPixels();
  void buildPayload();
  void drawPreviews();
  void drawGridPreview(float x, float y, float scale);
  void drawUiText(float x, float y);
  void loadVideo(const std::string& path);
  void loadVideoDialog();
  
  // MIDI
  void setupMidi();
  void openMidiPortByIndex(int idx);
  void loadPresetsFromFile();
  void savePresetsToFile() const;
  void storePreset(int idx);
  void recallPreset(int idx);

  // OSC sender (from wifiLedController)
  void setupOscSenders();
  void sendOscFrame();
  void handleOscAcks();
  void sendFrameIfDue();

  static constexpr int kCols = 12;
  static constexpr int kRows = 19;
  static constexpr int kNumLeds = kCols * kRows;
  static constexpr int kPayloadSize = kNumLeds * 3;
  // Video
  ofVideoPlayer video;
  bool videoLoaded = false;
  bool paused = false; // video pause
  int lastVideoFrame = -1;
  uint32_t lastVideoFrameMillis = 0;

  // Downsampled grid
  ofFbo downsampleFbo;
  ofPixels downsamplePixels;

  // Line/pattern params (from cylinder-led-controller)
  ofFloatColor lineColor = ofFloatColor::fromHsb(160.0f/255.0f, 1.0f, 1.0f);
  float lineWidth = 2.0f;                 // px in grid-space
  float angleDeg = 0.0f;                  // 0..90
  float rotationSpeedDegPerSec = 30.0f;   // -360..360
  float verticalSpeedPxPerSec = 2.0f;     // -20..20 in grid px/s
  bool pauseAutomation = false;
  float automationPhase = 0.0f;           // 0..1
  float globalTime = 0.0f;

  // Mapping
  LedMapper mapper;
  bool serpentine = false;
  bool verticalFlip = false;
  int columnOffset = 0;

  // OSC
  std::vector<std::string> oscHosts;
  std::vector<ofxOscSender> oscSenders;
  ofxOscReceiver ackReceiver;
  int oscPort = 9000;
  int ackPort = 9001;
  std::string oscAddress = "/leds";
  bool oscReady = false;
  std::unordered_map<std::string, uint32_t> lastSeenByHost;
  uint32_t activeDeviceTtlMs = 3000;

  // Sender timing
  float targetSendFps = 30.0f;
  float brightnessScalar = 0.8f;
  bool sendOsc = true;                 // allow pausing sender via MIDI
  bool blackout = false;

  std::atomic<uint64_t> framesSent{0};
  std::atomic<uint64_t> framesDropped{0};
  uint32_t lastSendMillis = 0;

  std::vector<uint8_t> payload;
  std::mutex oscMutex;

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
  std::string midiStatus = "MIDI: Not detected";
  std::string midiPortName;

  struct Preset {
    bool hasData = false;
    Source source = Source::Video;
    float lineWidth = 2.0f;
    float angleDeg = 0.0f;
    float rotationSpeedDegPerSec = 30.0f;
    float verticalSpeedPxPerSec = 2.0f;
    bool pauseAutomation = false;
    float automationPhase = 0.0f;
    ofFloatColor lineColor = ofFloatColor::fromHsb(160.0f/255.0f, 1.0f, 1.0f);
    bool serpentine = false;
    bool verticalFlip = false;
    int columnOffset = 0;
    float targetSendFps = 30.0f;
    float brightnessScalar = 0.8f;
    bool sendOsc = true;
    bool blackout = false;
  };

  static constexpr int kPresetCount = 8;
  std::array<Preset, kPresetCount> presets;
  std::string presetsPath;
};
