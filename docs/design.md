# ShipGuard Design Document

Prototype design for a damage-risk monitoring node that rides inside a fragile package. This document describes a design that can actually be built on the board. Values marked "example" are meant to be calibrated with box experiments. They are not certified values.

## 0. Scope: MVP and extensions

The whole design is kept on paper, but this table fixes what is implemented now (**MVP**) and what comes next (**Extension 1**, **Extension 2**). Sections below tag items with `(MVP)` or `(Ext 1)`.

| Item | Decision | Reason |
|---|---|---|
| 4-state machine (`STORAGE/TRANSIT/HANDLING/ALERT`, per-state duty cycle) | **Ext 1.** MVP has only `MONITORING` and `ALERT` | Per-state sampling and sensor scheduling add the most debugging work |
| Cumulative damage `D` (Miner-style approximation) | **Ext 1** | Fixed-point overflow risk. Shock and tilt are enough to show L0 to L3 |
| Vibration RMS | **Ext 1** (first to add) | Shock and tilt already show profile differences in drop tests |
| Automatic ACK escalation timer | **Ext 1.** MVP has the button ACK only | Needs another timer, and the MVP has no "shipper" to notify |
| Smartphone push alerts | **Ext 1 (opt-in)** | Adds external-service cost and security concerns. The PC dashboard shows the idea |
| PIR, 3 profiles on a potentiometer | **MVP** | Already verified by the smoke test, low code cost |
| CRC-8 serial frame | **MVP** | Small code, high value |
| Power model (datasheet-based mAh estimate) | **MVP** | A calculation, not code |
| Host unit tests, CI | **Ext 2** | Quality items that do not change behavior |
| Direct accelerometer register read | **After Ext 1 (optional)** | Pure optimization |
| Communication plan, security notes | **No change** | Documents, not code |

**MVP logic summary**
```
loop() at 400 Hz:
  read accelerometer -> detect shock event, compute tilt (static gate)
  -> risk score = max(shock score, tilt score)
  -> level L0..L3 (with hysteresis)
  -> update LED / buzzer / OLED, send serial frame
  -> button press = ACK
  -> potentiometer picks the profile, PIR marks "person present"
```

**Changes made during on-hardware integration** (reflected in each section):
- ACK rule: valid only during an alert. It silences the buzzer only (acknowledged, still active). The buzzer sounds again if the risk rises (5.3, 6.1).
- The alarm sounds until ACK. An alert caused by tilt alone clears by itself if the level returns to L0 before ACK (5.3).
- PIR emulates "the driver came to the cargo". It does not change the risk score. It is used for response time, the ACK prompt, and separating shock causes (7).
- PC commands `SET_INTERVAL` (report period) and `ARM` (new shipment) are implemented. `ACK_LOG` and `PRESENCE` frames were added (8).
- PC receiver `pc/gateway.py` and dashboard `pc/dashboard.py` were written (8.4).

## 1. Constraints verified by the smoke test

| Item | Measured | Meaning for the design |
|---|---|---|
| SRAM | 2,048 B total, about 1,200 B free after `setup()` | Keep application data under about 400 B, the rest is stack and margin |
| Flash | 32 KB, sensor-kit library baseline about 6.4 KB | Application target about 10 KB |
| Accelerometer | 393 samples/s at 400 Hz ODR, no sample loss. `getAcceleration()` takes 672 µs, about 0.9 ms per sample | 36% of the 2.5 ms sample period is reading, about 1.6 ms left for compute |
| Range | Can be set to ±16 g. A bare board tap still saturates | **Saturation-aware score** is required |
| Rest reading | (x, y, z) = (0.012, 0.038, 1.025) g | Magnitude 1.026 g (about 2.6% error). **Reference-pose calibration** needed |
| I2C | 0x19 accelerometer, 0x38 DHT20, 0x3C OLED, 0x77 BMP280, 400 kHz, no conflicts | |
| DHT20 | `read()` needs 1 s or more between reads and blocks. Split API exists: `requestData()`, `readData()`, `convert()` | **Split environment reads** so they never block |
| BMP280 | The Seeed driver can wait forever (no timeout) | Call only after an ACK check, include a bus-recovery routine |
| Temperature | BMP280 24.50 °C, DHT20 23.08 °C (1.4 °C apart) | Use DHT20 for ambient. Cause of the gap is unknown |
| PIR | D2 (INT0), output stays HIGH about 2.2 s after motion | Use an interrupt for the "handler nearby" event |
| OLED | U8x8 text mode, "SMOKE TEST / OLED OK" shown | No graphics mode |
| Pins | Button D4, buzzer D5, LED D6, potentiometer A0, sound A2, light A3 (all checked with the `l` and `o` smoke-test commands) | |
| Tilt (`t`) | At rest roll/pitch 0.7 to 3.5°. Tilting fast, roll followed from 53.6° to 85.7° while `|a|` swung between 0.54 and 1.14 g | Need a static gate and a dead band. The angle math itself is fine |
| Sample values | Pressure 1013.6 hPa, humidity 51.4%, light 428, sound 126, potentiometer 969, button 0 | All sensors read sensible values |

## 2. Software structure

**Important (verified):** the Arduino IDE and arduino-cli compile only the `.cpp` and `.h` files directly inside the sketch folder. They ignore `.cpp` files in subfolders such as `src/core/` (confirmed by an `undefined reference` link error). So all files sit flat in the sketch folder, and file names keep the core/hal split.

```
firmware/
  shipguard/                  # this whole folder is the Arduino sketch, no subfolders
    shipguard.ino             # setup()/loop(): scheduler, hardware access, calls the core modules
    crc8.h / crc8.cpp         # [core] CRC-8
    frame.h / frame.cpp       # [core] frame encode/decode
    features.h / features.cpp # [core] shock, delta-V, tilt (fixed-point, tilt in float)
    risk.h / risk.cpp         # [core] profiles, risk score, level with hysteresis
    fsm.h / fsm.cpp           # [core] MONITORING/ALERT state machine
  tests/                      # host unit tests: only the core files, built with g++
```

**MVP simplification:** hardware access (accelerometer, OLED, button, buzzer, LED, PIR) stays inside `shipguard.ino`. Only the five core files avoid including `Arduino.h`, so they compile on a PC. Splitting out a separate hardware layer is part of Ext 2.

**Rule: the core knows nothing about hardware.** It does not include `Arduino.h`. Time comes in as an argument (`now_ms`), input comes in as a struct, and output goes back as a struct. This lets plain `g++` build the core for unit tests.

## 3. Execution model: cooperative scheduler

No `delay()`. A task table runs on `micros()`.

| Task | Period | Work | Time budget |
|---|---|---|---|
| `sample` | 2.5 ms (400 Hz) | Read accelerometer, update features, check shock trigger | 0.9 ms read + at most 0.6 ms compute |
| `second` | 1 s | Update tilt and risk score, run state transitions. Vibration RMS window closes here in **Ext 1** | at most 1.5 ms |
| `env` | 10 s (`MONITORING` only) | Non-blocking DHT20/BMP280 read | Split into steps of about 1 ms or less |
| `ui` | 20 Hz | LED, buzzer pattern, partial OLED update | Only a few characters, at most 1.5 ms |
| `comm` | Event or periodic | Send frames, receive commands | Sending only fills the TX buffer |

- **Rule:** no task may run past the next `sample` deadline (2.5 ms). A missed deadline raises the `deadline_miss` counter, which is reported in the status frame.
- DHT20: call `requestData()`, then call `readData()` at least 80 ms later. This removes blocking.
- `env` stops during `ALERT` (the Ext 1 `HANDLING` state stops it too).
- **Planned optimization (to be measured):** the library reads X, Y and Z separately and converts to float (672 µs). Reading 6 bytes from register `0x28|0x80` in one go and converting to integer mg should take about 0.3 ms. Measure before and after.
- PIR (D2) uses `attachInterrupt` only to set a flag. The main loop does the work. The buzzer uses `tone()` (Timer2).

## 4. Feature extraction (fixed-point, units of mg)

- Sample: `int16 ax, ay, az` in mg. The magnitude `|a|` uses an integer square root.
- **Saturation:** if any axis has `|axis| >= 15,000 mg`, set `sat = 1`.
- **Shock event:** `dyn = | |a| - 1000 |` (gravity removed). Start when `dyn > 2,000 mg`. End when `dyn < 500 mg` for 8 samples (20 ms). Maximum length 200 ms.
  - `peak_mg` = largest `dyn` in the event
  - `delta-V (mm/s) ~ sum(dyn) x 0.0245` (2.5 ms sample period, 1 mg for 2.5 ms = 0.0245 mm/s). In integers: `sum(dyn) / 41`. This ignores direction, and **for a saturated waveform it is a lower bound**.
  - A 16-sample ring buffer (96 B) keeps the 20 ms before the event.
- **Vibration RMS (1 s window, Ext 1):** not in the MVP. Clip `dyn` to ±2,000 mg (shocks use their own path) and scale in 2 mg units. The maximum is 1,000² × 400 = 4×10⁸, below 2³², so a `uint32` sum does not overflow.
- **Tilt:** once per second, average the vector over 1 s and compute the angle to the reference pose (the gravity vector stored at calibration) in float. The rest offset (y = 0.038 g, about 2°) is removed by calibration at start. `dwell` counts the seconds above the limit angle. **Static gate:** update the angle only when `| |a| - 1000 | < 150 mg`, because `|a|` swung from 0.54 to 1.14 g during fast tilting. Rest roll/pitch noise was 0.7 to 3.5° (about ±1.5°), so a 5° dead band is used.
- **Open detection:** if the light value (A3) jumps by 200 or more from its moving average, set the `open` flag.

## 5. Risk score

### 5.1 Sensitivity profiles (constants in flash, chosen by the potentiometer)

| Item | GLASS (glass, ceramic) | ELECTRONICS | ROBUST |
|---|---|---|---|
| `G_crit` peak (mg) | 4,000 | 8,000 | 14,000 |
| `dV_crit` (mm/s) | 1,500 | 2,500 | 4,000 |
| `vib_limit` RMS (mg) | 150 | 300 | 600 |
| `tilt_limit` (°) / `tilt_dwell` (s) | 30 / 30 | 45 / 60 | 60 / 120 |

`dV_crit` was picked by converting to a drop height with `h = dV²/2g`: about 11 cm for GLASS, 32 cm for ELECTRONICS, 82 cm for ROBUST. So in the planned 5/10/20 cm drop tests, GLASS is meant to show L1, L2, L3 in turn. **All values are examples to be calibrated by experiment.**

### 5.2 Score formula (internal `uint16`, displayed as 0 to 100)

**MVP**
- `r_shock = 100 x max(peak/G_crit, dV/dV_crit)`, computed per event. When saturated, use `peak = 16,000` and set the `sat` flag. Hold for 30 s after the event, then decay 2 points per second.
- `r_tilt = 100 x dwell / tilt_dwell`. If the angle exceeds `1.5 x tilt_limit`, the minimum is 70.
- **Score `S = max(r_shock, r_tilt)`**

**Added in Ext 1**
- `r_vib = 100 x RMS_1s(EMA) / vib_limit`
- `D` (cumulative damage): per shock event add `20 x (peak/G_crit)²` percent when `peak >= 0.3 G_crit`. For vibration, add `0.005 x (RMS/vib_limit)²` percent per second when RMS is at least 50% of the limit. `D` never decreases. (A simplified Miner-style example.)
- The score then becomes **`S = max(r_shock, r_vib, r_tilt, D)`** (weakest-link style).

### 5.3 Levels and hysteresis

| Level | Condition | Local action |
|---|---|---|
| L0 | `S < 40` | Normal (LED off or slow blink) |
| L1 | `S >= 40` | LED blinks, OLED "WATCH" |
| L2 | `S >= 70` | Enter alert (6.1), intermittent buzzer, OLED "L2 CHECK" |
| L3 | `S >= 100` or a `sat` event | Enter alert, continuous buzzer, OLED "L3 INSPECT" |

- To go down, the score must stay below the threshold minus 10 for at least 5 s (hysteresis).
- **The alarm sounds until ACK** (button D4 or PC command `0x81`, same function). It does not stop when the level drops. The pattern follows the highest level reached during that alert (L3 reached = continuous, otherwise intermittent).
- **Clear rule by alert cause:**
  - **Shock alert** (shock score reached 70 or more and entered L2+): an event type. The item may already be damaged even if the score falls, so it **stays until ACK**.
  - **Tilt alert** (shock score below 70): a state type. If the level returns to L0 **before ACK, it clears by itself**, and the log records an alert that ended without ACK (this limits alert fatigue). If a shock arrives during a tilt alert, it becomes a shock alert.
- **Meaning of ACK:** "I saw the alert and will handle it", not "the problem is fixed". ACK silences only the buzzer. The alert state (LED, OLED `ALERT ACKED`) stays until the level drops below L2 (the "acknowledged, still active" state from industrial alarm management). If **risk rises after ACK** (level goes up, or a new shock scores 70 or more), the buzzer sounds again and a new ACK is needed. An ACK pressed when no alert is active is ignored, so it cannot pre-silence the next alert.
- In Ext 1, `D` is not cleared by ACK. Only a new shipment (`ARM`) resets it.

## 6. State machine

### 6.1 MVP: `MONITORING` and `ALERT`

| Current state | Condition | Next state |
|---|---|---|
| MONITORING | Level >= L2 | ALERT (waiting for ACK). If shock score >= 70, it is locked as a shock alert |
| ALERT (waiting) | Button or PC ACK | ALERT (acknowledged), send `ACK_LOG` |
| ALERT (waiting) | Tilt alert and level = L0 | MONITORING (cleared without ACK) |
| ALERT (acknowledged) | Level rises, or a new shock scores 70 or more | ALERT (waiting) (sounds again) |
| ALERT (acknowledged) | Level < L2 | MONITORING |
| Any | PC `ARM` | Reset everything, go to MONITORING |

| State | Buzzer | LED | OLED row 6 | OLED row 7 |
|---|---|---|---|---|
| MONITORING | Off | L0 off, L1 blinks | `MONITOR` | `PIR` / `OPEN` |
| ALERT (waiting) | On | On | `ALERT PRESS` | `PRESS BTN TO ACK` when a person is detected |
| ALERT (acknowledged) | Off | On if L2 or higher | `ALERT ACKED` | `PIR` / `OPEN` |

- `MONITORING`: the accelerometer always runs at 400 Hz. The risk score is computed and L0/L1 are shown only on the OLED and LED, so the driver is not disturbed. Environment sensors are read every 10 s (the first about 1.5 s after boot). The serial summary is sent every 60 s by default. For testing, the PC requests 1 s with `SET_INTERVAL`.
- `ALERT`: environment reads stop and serial reports speed up to at most every 5 s. An `ALERT` frame is sent immediately on every level or state change.
- **ODR stays at 400 Hz in both states.** The accelerometer interrupt pin is not available, and a lower ODR would miss short shocks. Only report period, environment schedule and UI change by state.
- In the MVP, PIR and open detection (light jump) do not change the state or the risk score. The state change belongs to the Ext 1 `HANDLING` state. See section 7 for the MVP use of PIR.

### 6.2 Ext 1: context states (`STORAGE/TRANSIT/HANDLING`)

`MONITORING` is split as below to complete the battery-saving story.

| Transition | Condition (test mode compresses time by 30x) |
|---|---|
| `STORAGE -> TRANSIT` | RMS_1s >= 25 mg for 5 s in a row (needs vibration RMS) |
| `TRANSIT -> STORAGE` | RMS_1s < 15 mg for 60 s in a row |
| `* -> HANDLING` | Any of: shock event, tilt change over 10° within 2 s, PIR active, box open. Stay at least 10 s, extend while events continue |
| `HANDLING -> TRANSIT/STORAGE` | Return to the previous context after 10 quiet seconds |

| State | Periodic report | Environment read | UI |
|---|---|---|---|
| STORAGE | 900 s | 10 s | OLED low rate |
| TRANSIT | 60 s | 10 s | OLED 1 Hz |
| HANDLING | 5 s | Stopped | Only needed characters |
| ALERT | 2 s | Stopped | Buzzer and LED first |

~~Air-segment (`AIR`) detection~~ was removed when the deployment scope narrowed to a metropolitan "warehouse to store" route, which has no air segment. The `SG_FLAG_AIR` bit in `frame.h` and the `SIM_AIR` command number stay reserved. In dead zones (tunnels, underground docks), events are stored and forwarded later.

## 7. Alerts, ACK and escalation

**MVP:** only the local actuators (buzzer, LED, OLED) and the button ACK work. Role-based push and automatic escalation are not demonstrated in code.

**PIR = driver arrived (MVP).** A PIR sensor inside a closed box would detect nothing, so the PIR emulates "a person (the driver) came to the cargo". In a real product, the BLE signal strength (RSSI) of the driver's phone would give the same information. It does not change the risk score.
- Detected state = PIR output is HIGH, or less than 5 s since the last HIGH. A PIR stays HIGH about 2 s and LOW for 1 to 2 s while a person is present, so a shorter hold time makes it flicker and log a new "arrival" each time (measured).
- A `PRESENCE` frame is sent once when detection starts. The dashboard computes **alert -> driver arrival -> ACK time**.
- If a person is detected while an alert waits for ACK, OLED row 7 shows `PRESS BTN TO ACK`.
- Every frame carries a PIR bit, so the dashboard splits shocks into "during handling (person present)" and "unattended (vehicle moving)".
- The PIR range is several meters, so even sitting at a desk triggers it. In tests, the PIR is hidden under the Sensor Kit to represent "driver absent", then taken out at the moment the driver checks, to represent "driver arrived".

**Ext 1:** the design below would run in the PC Python gateway, which parses frames. **External notification services are enabled only after the user confirms.** Until then, local dashboard alerts stand in.

| Level | Driver | Customer | Shipper |
|---|---|---|---|
| L1 | No alert | No alert | Shown on dashboard |
| L2 | **Push:** "Check at the next safe stop" | No alert | Push |
| L3 | **Push:** "Stop somewhere safe, then check the item" | Push (summary) | Push (detail) |
| Open (TAMPER) | Push | Push | Push |

- Re-alert cooldown: L2 120 s, L3 60 s (until ACK).
- **Escalation:** if there is no ACK within 300 s at L3 or 600 s at L2, send an escalation alert to the shipper.
- Alert wording never asks the driver to do anything while driving.

## 8. Serial protocol (115200 8N1)

### 8.1 Frame

```
[SOF][LEN][TYPE][SEQ][PAYLOAD (LEN bytes)][CRC8]
```

- Node to PC: `SOF = 0xA5`. PC to node: `SOF = 0x5A`.
- `CRC8`: polynomial `0x07`, initial value `0x00`, computed from `LEN` to the end of `PAYLOAD`.
- `SEQ`: one byte, incremented per direction. The PC detects gaps and duplicates.
- Maximum `LEN = 24`, so a frame is at most 29 B. At 115200 baud that is about 2.5 ms. Sending returns at once after the data goes into the 64 B TX buffer.
- Receiving is hardened with SOF sync, length check, CRC check and a timeout. Bad frames are dropped and counted.

### 8.2 Node to PC messages

| TYPE | Name | Content |
|---|---|---|
| `0x01` | TELEMETRY | Periodic summary (payload below) |
| `0x02` | EVENT | Shock event (same layout as TELEMETRY, values at event time) |
| `0x03` | ALERT | Level transition (same layout) |
| `0x04` | ACK_LOG | 8 B: `t_s` u32, source u8 (0 = button, 1 = PC), level at ACK u8, risk at ACK u16. Sent once per accepted ACK |
| `0x05` | STATUS | 8 B: uptime u32, free SRAM u16, `deadline_miss` u8 (low byte), sensor error u8. Reply to `PING` |
| `0x06` | PRESENCE | Once when person detection starts (same layout as TELEMETRY) |

TELEMETRY payload (24 B, little-endian):

| Offset | Field | Type | Description |
|---|---|---|---|
| 0 | `t_s` | u32 | Seconds since boot |
| 4 | `state` | u8 | 0 = MONITORING, 1 = ALERT (MVP). Values 2 to 4 are reserved for STORAGE/TRANSIT/HANDLING in Ext 1 |
| 5 | `level` | u8 | 0 to 3 |
| 6 | `profile` | u8 | 0 to 2 |
| 7 | `flags` | u8 | bit0 sat, bit1 open, bit2 pir (person detected), bit3 air (reserved, unused), bit4 ack_pending, bit5 sensor_fault |
| 8 | `risk` | u16 | 0 to 999 |
| 10 | `peak_mg` | u16 | Last shock peak |
| 12 | `dv_mms` | u16 | Last delta-V |
| 14 | `vib_rms_mg` | u16 | Always 0 in the MVP. Filled in Ext 1 |
| 16 | `tilt_x10` | i16 | Units of 0.1° |
| 18 | `temp_x100` | i16 | DHT20, 0.01 °C |
| 20 | `hum_x100` | u16 | |
| 22 | `press_x10` | u16 | 0.1 hPa |

Even the MVP sends many distinct data points: risk score, peak, tilt, temperature, state and alert level. The unused fields are placeholders, so Ext 1 fills in values without changing the frame format.

### 8.3 PC to node commands

| TYPE | Name | Arguments |
|---|---|---|
| `0x81` | ACK | None |
| `0x82` | SET_PROFILE | u8 profile number |
| `0x83` | SET_THRESH | u8 item, u16 value (**not implemented in the MVP**, ignored) |
| `0x84` | PING | None (answered with STATUS) |
| `0x85` | SET_INTERVAL | u8 MONITORING report period in seconds (1 to 60, default 60). The receiver requests 1 s at start and when it detects a board reboot |
| `0x86` | SIM_AIR | Reserved (air segment removed, unused) |
| `0x87` | ARM | None. Starts a new shipment (emulates scanning the dispatch QR code): treat the box as at rest, retake the reference pose, and reset risk score, level and state machine |

- The node also accepts the text command `h` (prints one status line, for debugging). Bytes that arrive in the middle of a frame are never read as text.

### 8.4 PC side: receiver and dashboard

| File | Role in the architecture | What it does |
|---|---|---|
| `pc/gateway.py` | Driver phone gateway (fog) | Parses frames and checks CRC, prints to screen, saves `pc/data/run_*.csv` (with a `latest.csv` link). Keyboard commands: `a` ACK, `p0/p1/p2` profile, `i N` report period, `r` ARM, `s` status, `q` quit. On exit it prints the CRC error count and the sequence gap count. Works without pyserial by using POSIX termios |
| `pc/dashboard.py` | Cloud and application layer | Redraws `latest.csv` every second: status line (level, ACK state, environment, driver response time, shock cause counts), risk score with L1 to L3 bands, shock peaks (handled / unattended), tilt with profile limit, temperature, humidity. `--static out.png` makes a static figure for reports |

Run: terminal 1 `python3 pc/gateway.py`, terminal 2 `python3 pc/dashboard.py`. Opening the serial port reboots the UNO, so restarting the receiver restarts the board.

## 9. Memory budget (SRAM)

| Item | Bytes |
|---|---|
| Pre-trigger ring buffer (16 x 6 B) | 96 |
| Event log (8 x 12 B) | 96 |
| Feature/accumulator variables, state machine | about 80 |
| TX frame buffer | 32 |
| RX frame buffer | 32 |
| Total (application) | about 340 |
| Free (including stack) | about 850 |

- **Rule:** at least 400 B of free SRAM even on the worst path. The status frame carries free SRAM so it is measured at runtime.
- The MVP has no vibration accumulator and no `D`, so real usage of "feature/accumulator variables" is below the table value. The difference is kept as safety margin.
- **Measured (early, `arduino-cli compile`, UNO):** whole `firmware/shipguard/shipguard.ino` uses **Flash 20,736 B (64%), SRAM 1,013 B (49%), free SRAM 1,035 B**. This is well above the target (400 B free or more). With `--warnings all`, there are 0 warnings in own code.
- **Measured (after on-hardware integration):** compile result **Flash 21,986 B (68%), static SRAM 1,092 B (53%), 956 B left**. Free SRAM in the runtime `STATUS` frame is 887 to 914 B (including stack use), above the 400 B target. 0 warnings in own code.
- All strings use `F()` (flash). The profile table is in `PROGMEM`.
- Flash target: application about 10 KB (about 17 KB with libraries).

## 10. Verification plan and metrics

| Metric | Target | Method |
|---|---|---|
| Sample deadline | `deadline_miss` = 0 (also during `HANDLING`) | Counter, repeated scenarios |
| Shock detection delay | Event start to L2 display under 100 ms (local) | Serial timestamps and LED |
| Drop test | GLASS shows L1/L2/L3 at 5/10/20 cm | Drops onto padding, 5 times each |
| False alarms | At most 1 L2-or-higher alert in 20 normal handling trials (pick up, carry) | Experiment |
| Frame robustness | 100% CRC detection under injected noise, keeps running after recovery | Host tests and loss injection |
| Memory | Free SRAM >= 400 B, Flash <= 20 KB | Compile result and runtime report |
| Host unit tests | CRC, frame, features, score, state machine, hysteresis, ACK rules, tilt alert auto-clear | `firmware/tests/test_core.cpp`, **10 tests pass**. CI is Ext 2 |
| Frame link (on hardware) | 0 CRC errors and 0 sequence gaps at the receiver | Measured: 0 in several 1 to 3 minute runs |

## 11. Open issues and risks

1. **Profile values are examples.** The real shock size inside a padded box is unknown until the drop experiments. A bare board saturated even at ±16 g.
2. The delta-V of a saturated shock is a lower bound, so the score may be too low. That is why a saturated event is treated as L3 automatically.
3. The accelerometer interrupt pin is not available, so low-power wake-up cannot be built. The power model is a datasheet-based calculation, and its assumptions are stated.
4. The cause of the 1.4 °C gap between BMP280 and DHT20 is not known.
5. Smartphone alerts leave through an external service, so they need user approval.
6. **Scope was reduced to the MVP.** Vibration RMS, cumulative damage, the 4-state context, automatic escalation and smartphone alerts stay in the design but are not implemented in the current prototype (deferred to Ext 1).
7. **OLED row 7 was not visible while running.** A diagnostic screen showed that rows 0 to 7 are all visible. The problem went away after the profile-name line was fitted to 16 columns (`ELECTRONICS` plus padding exceeded 16 columns and U8x8 writes past the end instead of cutting) and the last line length was matched. Which of the two was the cause was not isolated. Rule: always write at most 16 characters per OLED line.
8. **The PIR range is several meters,** so a "nobody present" state is hard to create. Tests hide the PIR under the kit and take it out when the driver checks (section 7). Flicker was reduced by increasing the hold time.
