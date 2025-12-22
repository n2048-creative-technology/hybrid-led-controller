# Hybrid LED Controller

A combined openFrameworks application that blends the strengths of two existing apps without modifying them:

- Video ingest + OSC streaming (payload compatible with `wifiLedController`).
- MIDI‑driven LED pattern control (KORG nanoKONTROL2) inspired by `cylinder-led-controller`.

It lets you switch live between a MIDI‑controlled expression and a video‑driven expression, while streaming LED frames over WiFi/OSC to ESP32 receivers.


## Features

- Two sources:
  - Video: play a video and downsample to a 12×19 grid.
  - MIDI Pattern: animated line pattern with full controls via KORG nanoKONTROL2.
- OSC sender:
  - Multi‑host OSC streaming with broadcast support and ACK tracking.
- Mapping options: column rotation, serpentine wiring, vertical flip, brightness control.
- Simple, dependency‑light UI overlays with keyboard shortcuts.


## Requirements

- openFrameworks (0.11.x or later recommended).
- Addons: `ofxMidi`, `ofxOsc` installed in `OF_ROOT/addons/`.
- ESP32 receivers listening for OSC frames (as in `wifiLedController`).


## Build

From the project folder:

```
make -j4
```

The executable is created at `bin/hybrid-led-controller`.


## Run

- Optional: place a `video.mp4` in `bin/data/` – the app will try to load it on startup.
- Start the app. If a KORG nanoKONTROL2 (or other MIDI device) is connected, the app auto‑opens the first matching MIDI input.
- Edit `bin/data/osc_devices.txt` to set ESP32 IPs or leave it as `broadcast`.


## Modes (sources)

- Video mode:
  - Plays the loaded video and downsamples to 12×19 grid.
  - UI shows the video preview (left) and the LED grid preview (right).
- MIDI Pattern mode:
  - Generates an animated line pattern.
  - All parameters mapped to nanoKONTROL2 for live performance.

Switching sources
- Keyboard: `M` toggles Video ↔ MIDI Pattern
- MIDI: Mute 2 (CC 49) toggles Video ↔ MIDI Pattern


## MIDI Controller (KORG nanoKONTROL2)

Typical CC layout in CC mode is assumed. The app scans all input ports and prefers names containing “nanokontrol2” or “korg”; otherwise it opens the first available MIDI input.

- Sliders 0..7
  - CC 0: Line Width (0.5–12 px)
  - CC 1: Angle (0–90 deg)
  - CC 2: Rotation Speed (±360 deg/s)
  - CC 3: Vertical Speed (±20 px/s)
  - CC 4: Timeline Phase (0–1)
  - CC 5: Color Hue (0–255)
  - CC 6: Color Saturation (0–255)
  - CC 7: Color Brightness (0–255)
- Knobs 16..23: Mirror the sliders above (same parameters).
- Buttons
  - Solo 1 (CC 32): Toggle Pause Automation
  - Mute 1 (CC 48): Toggle OSC Send on/off (quick safety mute)
  - Mute 2 (CC 49): Toggle Mode Video ↔ MIDI Pattern

Notes
- The device should be in CC mode (not DAW mode). If values don’t change, check the device mode or use KORG’s configuration utility.


## Keyboard Shortcuts

- Global
  - `M`: switch mode (Video/MIDI)
  - `F`: fullscreen
  - `+/-`: global brightness
  - Up/Down: target send FPS (1–120)
  - `1/2`: mapping mode linear/serpentine
  - `V`: vertical flip
  - `[` `]`: rotate columns
  - `R`: reset column offset
- Video
  - `L`: load video file (dialog)
  - Space: play/pause
  - Drag‑and‑drop: drop a video file to load
- MIDI Pattern (line)
  - `A/Z`: angle ±
  - `W/S`: line width ±
  - `O/P`: rotation speed ±
  - `K/I`: vertical speed ±
  - `Q`: pause automation


## OSC Payload

- Address: `/leds`
- Payload: one blob containing RGB triplets for the 12×19 LED grid (684 bytes).
- Host list: `bin/data/osc_devices.txt` (one IP/hostname per line, or `broadcast`).


## LED Mapping Options

- Column Rotation: rotate the cylinder horizontally (column wiring offset).
- Serpentine: reverse direction of every odd column.
- Vertical Flip: mirror top/bottom.
- Brightness Scalar: global software dimming (applies to both sources).


## Troubleshooting

- MIDI not detected
  - Ensure `ofxMidi` is installed and built.
  - Confirm the device is in CC mode (not DAW).
  - Use a MIDI monitor to verify CC messages are being sent.
- No OSC connection
  - Check `bin/data/osc_devices.txt` points at reachable ESP32 IPs (or use `broadcast`).
  - Verify the ESP32 receiver is listening on port 9000 and sending ACKs to port 9001.
- Low FPS / dropped frames
  - Lower the target send FPS or check WiFi stability and receiver throughput.


## License

No license changes. This app is a new project that reuses ideas and runtime integration approaches; it does not modify the original `cylinder-led-controller` or `video-to-LED-matrix` projects.
