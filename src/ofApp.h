#pragma once

#include "ofMain.h"
#include <atomic>
#include <condition_variable>
#include <mutex>
#include <thread>
#include "LedMapper.h"
#if __has_include("ofxMidi.h")
#include "ofxMidi.h"
#define HAS_OFXMIDI 1
#else
#define HAS_OFXMIDI 0
#endif

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

  // Serial sender thread (from video-to-LED-matrix)
  void startSenderThread();
  void stopSenderThread();
  void senderThreadLoop();
  void refreshPorts();
  bool autoConnectPort();
  bool connectToPortIndex(int idx);
  void closeSerial();
  void sendFrameIfDue();

  static constexpr int kCols = 12;
  static constexpr int kRows = 19;
  static constexpr int kNumLeds = kCols * kRows;
  static constexpr int kPayloadSize = kNumLeds * 3;
  static constexpr int kPacketBytes = 2 + 1 + 2 + 2 + kPayloadSize + 2;
  static constexpr uint8_t kProtocolVersion = 0x01;
  static constexpr uint8_t kMagic0 = 0xAA;
  static constexpr uint8_t kMagic1 = 0x55;

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

  // Serial
  ofSerial serial;
  std::vector<ofSerialDeviceInfo> devices;
  int selectedPortIndex = -1;
  std::string connectedPort;
  std::string lastKnownPort;
  int baud = 115200;
  bool autoConnect = true;
  std::atomic<bool> serialConnected{false};
  uint32_t serialConnectMillis = 0;

  // Sender timing
  float targetSendFps = 30.0f;
  float maxSendFps = 0.0f;
  bool sendFpsClamped = false;
  float brightnessScalar = 0.8f;
  bool sendSerial = true;                 // allow pausing sender via MIDI

  uint16_t frameId = 0;
  std::atomic<uint64_t> framesSent{0};
  std::atomic<uint64_t> framesDropped{0};
  uint32_t lastSendMillis = 0;
  std::atomic<uint32_t> lastWriteMillis{0};

  std::vector<uint8_t> payload;
  std::vector<uint8_t> packet;

  float lastPortRefresh = 0.0f;
  uint32_t lastReconnectAttemptMillis = 0;
  uint32_t reconnectIntervalMs = 500;

  std::thread senderThread;
  std::mutex senderMutex;
  std::condition_variable senderCv;
  std::vector<uint8_t> pendingPacket;
  bool hasPendingPacket = false;
  std::vector<uint8_t> lastPacket;
  bool hasLastPacket = false;
  std::atomic<bool> senderStop{false};
  std::atomic<bool> requestClose{false};
  std::atomic<bool> requestConnect{false};
  std::mutex connectMutex;
  std::string pendingConnectPort;
  std::mutex serialMutex;

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
};
