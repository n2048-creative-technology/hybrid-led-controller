# Hybrid LED Controller

A combined openFrameworks application that blends the strengths of two existing apps without modifying them:

- Video ingest + robust threaded serial streaming (protocol, reconnects, CRC) from `video-to-LED-matrix`.
- MIDI‑driven LED pattern control (KORG nanoKONTROL2) inspired by `cylinder-led-controller`.

It lets you switch live between a MIDI‑controlled expression and a video‑driven expression, while always using the streaming style and protocol expected by the Arduino receiver used by `video-to-LED-matrix`.


## Features

- Two sources:
  - Video: play a video and downsample to a 12×19 grid.
  - MIDI Pattern: animated line pattern with full controls via KORG nanoKONTROL2.
- Robust sender:
  - Threaded serial writer with back‑pressure, keep‑alive resend, auto‑connect/reconnect.
  - CRC16 framing and protocol compatible with `video-to-LED-matrix` Arduino sketch.
- Mapping options: column rotation, serpentine wiring, vertical flip, brightness control.
- Simple, dependency‑light UI overlays with keyboard shortcuts.


## Requirements

- openFrameworks (0.11.x or later recommended).
- Addon: `ofxMidi` installed in `OF_ROOT/addons/ofxMidi`.
- Arduino receiver compatible with the `video-to-LED-matrix` packet format.


## Build

From the project folder:

```
make -j4
```

The executable is created at `bin/hybrid-led-controller`.


## Run

- Optional: place a `video.mp4` in `bin/data/` – the app will try to load it on startup.
- Start the app. If a KORG nanoKONTROL2 (or other MIDI device) is connected, the app auto‑opens the first matching MIDI input.
- Serial auto‑connect is enabled by default and will connect to the last known port if available, or the first matching `ttyACM/ttyUSB/usbmodem/COM` port.


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
  - Mute 1 (CC 48): Toggle Serial Send on/off (quick safety mute)
  - Mute 2 (CC 49): Toggle Mode Video ↔ MIDI Pattern
  - Rec 1 (CC 64): Enable Serial Auto‑Connect
  - Rec 2 (CC 65): Disconnect + Disable Auto‑Connect

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
  - `0..9`: connect to listed serial port index
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


## Serial Protocol

- Geometry: 12 columns × 19 rows = 228 LEDs, RGB payload = 684 bytes.
- Framing (byte order, all values little‑endian):
  - 0: `0xAA` (magic0)
  - 1: `0x55` (magic1)
  - 2: `0x01` (protocol version)
  - 3–4: `uint16 length` = 684
  - 5–6: `uint16 frameId` (increments each frame)
  - 7..(7+length-1): payload RGB triplets (R,G, B) for each LED
  - end: `uint16 crc16` over bytes starting at index 2 (version) through end of payload
- CRC: CRC16‑CCITT (poly 0x1021), initial value 0xFFFF.
- Baud: default 115200; max effective FPS is auto‑computed to stay within serial bandwidth.
- Keep‑alive: the sender thread resends the last packet if no data was written for ~400ms to help receivers detect link health.

This matches the Arduino receiver shipped with `video-to-LED-matrix`.


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
- No serial connection
  - Check Arduino is on a `ttyACM`/`ttyUSB`/`usbmodem` port.
  - Press a digit `0..9` to force connect to a listed port index.
  - Use Rec 1 (CC 64) to re‑enable auto‑connect; Rec 2 (CC 65) disconnects.
- Low FPS / dropped frames
  - The app auto‑clamps send FPS to serial bandwidth. Increase baud on both ends if necessary and rebuild.


## License

No license changes. This app is a new project that reuses ideas and runtime integration approaches; it does not modify the original `cylinder-led-controller` or `video-to-LED-matrix` projects.

