# Biomechanical Throw Tracker

A wearable, dual-microcontroller glove that captures real-time kinematic and
biomechanical data from an Ultimate Frisbee throw — and turns it into live
sports-science metrics on a browser-based 3D dashboard.

The system streams hand/forearm motion, muscle activation, and grip-release
timing at up to ~100 Hz, then computes release speed, spin rate, snap force,
release latency, and throw angle (Hyzer/Anhyzer) in real time, all in-browser.

---

## How it works

```
 ┌────────────────────┐   ESP-NOW (2.4 GHz, ~100 Hz)   ┌──────────────────────┐   USB Serial (115200)   ┌────────────────────┐
 │   Glove Node        │ ───────────────────────────▶  │   Base Station       │ ──────────────────────▶ │   Browser           │
 │ XIAO ESP32-C3       │                                │ ESP32 WROOM          │   CSV telemetry          │ visualizer.html     │
 │ + BNO085 IMU        │                                │ (receiver.ino)       │   over Web Serial API     │ Three.js 3D hand +  │
 │ + AD8232 (EMG)      │                                └──────────────────────┘                          │ physics engine +    │
 │ + FSR via ADS1115   │                                                                                   │ live metrics        │
 └────────────────────┘                                                                                   └────────────────────┘
```

1. **Glove Node** (`Main_glove.ino`, worn on the wrist) reads the IMU, EMG, and
   grip-sensor data and packages it into a compact C++ struct, sent over
   **ESP-NOW** at ~100 Hz.
2. **Base Station** (`receiver.ino`) receives that struct via ESP-NOW and
   prints it as a single CSV-style line over **USB serial** at 115200 baud.
3. **`visualizer.html`** connects to that serial port via the **Web Serial
   API**, parses each line, and runs a JavaScript physics/state-machine engine
   that detects the throw, computes biomechanical metrics, and renders a live
   3D hand.

---

## Hardware

| Component | Role |
|---|---|
| **Seeed Studio XIAO ESP32-C3** | Glove-mounted MCU. Reads sensors over I2C/analog, streams via ESP-NOW. |
| **ESP32 WROOM** | Base station. Receives ESP-NOW packets and forwards them to a PC over USB serial. |
| **BNO085 (9-axis IMU, I2C addr `0x4B`)** | Linear acceleration, calibrated gyroscope (rad/s), and rotation-vector quaternions. |
| **AD8232 (ECG → repurposed for EMG)** | Forearm muscle activation, placed over the *flexor carpi ulnaris*; read on ADS1115 channel A0. |
| **Force-Sensitive Resistor (FSR)** | Mounted on the thumb; grip pressure read on ADS1115 channel A1 and used to trigger the throw state machine. |
| **ADS1115 (16-bit ADC, I2C addr `0x48`)** | Samples EMG (A0) and FSR (A1) at 860 SPS, gain ×1, and passes clean digital voltages to the ESP32 over I2C. |

### Glove Node wiring (XIAO ESP32-C3)

| Signal | Pin |
|---|---|
| I2C SDA | `D2` |
| I2C SCL | `D3` |
| Sensor power enable | `D0` (drives a MOSFET/load switch powering the IMU + ADC) |

I2C runs at 100 kHz during sensor initialization, then switches to 400 kHz for
normal operation.

---

## Software pipeline

### 1. Glove firmware — `Main_glove.ino`

- Initializes the ADS1115 (`0x48`, gain ×1, 860 SPS) and BNO085 (`0x4B`)
- Powers the sensors via the `SENSOR_PWR` pin
- Registers an ESP-NOW peer using a **hardcoded MAC address** for the base
  station (`broadcastAddress[]` near the top of the file — update this to your
  base station's MAC)
- Enables three BNO085 reports at 100 Hz (10,000 µs interval): rotation
  vector (quaternion), linear acceleration, and calibrated gyroscope
- On each rotation-vector event, also reads the ADS1115 (EMG on A0, FSR on
  A1) and transmits the full telemetry struct over ESP-NOW
- Self-restarts (`ESP.restart()`) if sensor init fails or no sensor data is
  received for 1 second — a simple watchdog for a wearable device

**Telemetry struct** (must match `receiver.ino` exactly):

```cpp
typedef struct struct_message {
  float cardiac;                 // EMG voltage (ADS1115 A0)
  float fsr;                     // FSR voltage (ADS1115 A1)
  float q_r, q_i, q_j, q_k;       // BNO085 rotation-vector quaternion
  float a_x, a_y, a_z;            // BNO085 linear acceleration (m/s²)
  float g_x, g_y, g_z;            // BNO085 calibrated gyroscope (rad/s)
} struct_message;
```

### 2. Base station firmware — `receiver.ino`

- Registers an ESP-NOW receive callback and prints each incoming struct as a
  single comma-separated line over USB serial (115200 baud), e.g.:

  ```
  Cardiac_V:1.482, FSR_V:0.012, Quat_R:0.998, Quat_I:0.012, Quat_J:-0.034, Quat_K:0.004, Accel_X:0.21, Accel_Y:-0.05, Accel_Z:9.78, Gyro_X:0.01, Gyro_Y:0.00, Gyro_Z:-0.02
  ```

- If no packet is received for 2 seconds, it prints a zeroed line so the
  dashboard doesn't display stale data after a glove disconnect

### 3. Dashboard & physics engine — `visualizer.html`

A self-contained page (Three.js r128 + Chart.js, loaded from CDN) that:

- Connects to the base station via the **Web Serial API** (`navigator.serial`)
- Renders a low-poly **Three.js** 3D hand (palm, fingers, thumb as separate
  meshes) that mirrors the glove's calibrated orientation in real time
- Plots live FSR voltage on a rolling Chart.js line chart
- Implements the throw **state machine** based on FSR voltage:
  - **Arm** — when FSR voltage rises above `1.0 V` (disc gripped), reset
    velocity integrators and peak trackers, badge → "TRACKING MOTION..."
  - **Track** — while gripped, integrate linear acceleration (`v += a·dt`,
    `dt = 0.01 s`) for live velocity and track the peak acceleration magnitude
    and peak smoothed EMG value
  - **Snap** — when FSR voltage drops below `0.5 V` (disc released), freeze
    the math, badge → "RELEASE CAPTURED", and compute the final throw metrics
- Applies an EMG signal-processing chain: rectify against a `1.5 V` baseline,
  then exponentially smooth (`smoothed = 0.9·smoothed + 0.1·rectified·100`) to
  produce a "Forearm Tension" value used for the latency metric
- Provides a **Tare Geometry** button that stores the inverse of the current
  rotation quaternion as an offset, so all subsequent orientations are
  reported relative to that "zero" pose — useful since the IMU is taped to the
  glove at an arbitrary angle

---

## Biomechanical metrics

| Metric | How it's computed |
|---|---|
| **Release Speed (m/s)** | `v = Σ(a · dt)` — linear acceleration integrated over the grip duration (`dt = 0.01 s` per sample), magnitude of the resulting velocity vector |
| **Peak Arm Acceleration (m/s²)** | Maximum `‖(aₓ, a_y, a_z)‖` observed during the grip |
| **Release Spin (RPM)** | `‖(gₓ, g_y, g_z)‖ × 9.549` — gyroscope magnitude (rad/s) at release converted to RPM |
| **Snap Force (N)** | `F = m·a` — peak arm acceleration × mass of a regulation Discraft UltraStar (`0.175 kg`) |
| **Release Latency (ms)** | Time between the peak smoothed-EMG sample and the FSR-detected release — the "whip effect"; lower = more efficient kinetic chain |
| **Pitch & Roll, "Hyzer Angle" (°)** | The calibrated release-time quaternion converted to pitch (asin) and roll (atan2) Euler angles |

---

## Getting started

### Requirements

- Arduino IDE (or PlatformIO) with the **ESP32 board package** installed
- Libraries (Glove Node): `Adafruit_ADS1X15`, `Adafruit_BNO08x` (and their
  dependency `Adafruit_BusIO`), plus the built-in `Wire`, `esp_now`, `WiFi`
- Libraries (Base Station): built-in `esp_now`, `WiFi`
- A Chromium-based browser (Chrome/Edge) for **Web Serial API** support —
  `visualizer.html` can be opened directly as a local file

### Setup

1. **Find your base station's MAC address**
   - Flash a minimal sketch (or use `WiFi.macAddress()`) on the ESP32 WROOM to
     print its MAC address over serial.

2. **Flash the glove node**
   - Open `Main_glove.ino` in the Arduino IDE
   - Select the **Seeed Studio XIAO ESP32-C3** board
   - Update `broadcastAddress[]` to the base station's MAC address from step 1
   - Install the required libraries, then upload to the glove-mounted board

3. **Flash the base station**
   - Open `receiver.ino`
   - Select your **ESP32 WROOM** board
   - Upload, then connect the board to your PC via USB

4. **Run the dashboard**
   - Open `visualizer.html` in Chrome/Edge
   - Click **Initialize Serial Link** and select the base station's serial port
   - With the glove resting flat/still, click **Tare Geometry** to zero the
     3D orientation
   - Grip the disc (FSR > 1.0 V) and throw — metrics populate automatically
     when the FSR drops below 0.5 V on release

---

## Repository structure

| File | Description |
|---|---|
| `Main_glove.ino` | Firmware for the glove-mounted XIAO ESP32-C3 (sensor reading + ESP-NOW transmit) |
| `receiver.ino` | Firmware for the ESP32 WROOM base station (ESP-NOW receive + USB serial forward) |
| `visualizer.html` | Browser dashboard: Web Serial input, physics engine, Three.js 3D hand, live metrics |

---

## Future work

- On-glove SD logging for sessions without a connected laptop
- Per-throw session history and export (CSV/JSON) for offline analysis
- Additional throw types (forehand/flick vs. backhand) with automatic
  classification
- Comparison view for tracking technique changes over time
- Auto-detect the base station's MAC address instead of hardcoding it

---

## License

MIT — see [LICENSE](LICENSE).
