// Smoke test for the ShipGuard prototype: Arduino UNO R3 + Arduino Sensor Kit Base + PIR.
// Purpose: verify pins, I2C devices, accelerometer range and sample rate, and actuators
// before the design is frozen (plan step 1, see CLAUDE.md section 7).
//
// Serial Monitor: 115200 baud. Send one letter (line ending does not matter):
//   h help          i I2C scan          s sensor snapshot     l live inputs (5 s)
//   r rate test     g range test (tap the board)              t tilt angles (5 s)
//   o output test (LED, buzzer, OLED)   p PIR pin finder (10 s)

#include "Arduino_SensorKit.h"

// ---------- PINS ----------
// Button/buzzer/LED/pot/mic come from the Arduino_Sensorkit Combined_Demo example.
// Light on A3 was confirmed by the analog pin diagnostic in HW2_Q10.
const uint8_t PIN_BUTTON = 4;
const uint8_t PIN_BUZZER = 5;
const uint8_t PIN_LED    = 6;
const uint8_t PIN_POT    = A0;
const uint8_t PIN_MIC    = A2;
const uint8_t PIN_LIGHT  = A3;
const uint8_t ACCEL_ADDR = 0x19;
#define USE_WIRE_TIMEOUT 0   // set to 1 to enable Wire.setWireTimeout (off while debugging the accelerometer)

// Pins the PIR output could be on. D4/D5/D6 are taken by the kit, D0/D1 by USB serial.
const uint8_t PIR_CANDIDATES[] = {2, 3, 7, 8, 9, 10, 11, 12, 13, 15};  // 15 = A1

// The kit wrapper hides the LIS3DHTR range setter, so this test drives the Grove driver directly.
LIS3DHTR<TwoWire> lis;

struct RateCase { odr_type_t odr; uint16_t hz; };
const RateCase RATES[] = {
  {LIS3DHTR_DATARATE_50HZ, 50},   {LIS3DHTR_DATARATE_100HZ, 100},
  {LIS3DHTR_DATARATE_200HZ, 200}, {LIS3DHTR_DATARATE_400HZ, 400},
  {LIS3DHTR_DATARATE_1_6KH, 1600}, {LIS3DHTR_DATARATE_5KHZ, 5000},
};

struct RangeCase { scale_type_t range; uint8_t g; };
const RangeCase RANGES[] = {
  {LIS3DHTR_RANGE_2G, 2}, {LIS3DHTR_RANGE_4G, 4},
  {LIS3DHTR_RANGE_8G, 8}, {LIS3DHTR_RANGE_16G, 16},
};

extern int __heap_start, *__brkval;
int freeRam() {
  int v;
  return (int)&v - (__brkval == 0 ? (int)&__heap_start : (int)__brkval);
}

void printI2cClock() {
  uint8_t prescaler = 1 << (2 * (TWSR & 0x03));
  Serial.print(F("  I2C clock ~ "));
  Serial.print(F_CPU / (16UL + 2UL * TWBR * prescaler));
  Serial.println(F(" Hz"));
}

void printHelp() {
  Serial.println(F("\nShipGuard smoke test. Commands:"));
  Serial.println(F("  h help | i I2C scan | s snapshot | l live inputs | r rate test"));
  Serial.println(F("  g range test | t tilt | o outputs | p PIR pin finder"));
}

// Set by setup(). The Seeed BMP280 driver spins forever if the sensor does not answer,
// so it is only used when a device really acks on the bus.
bool envOk = false;
bool bmpOk = false;

// If the MCU was reset in the middle of an I2C transfer, a slave can keep SDA held low and the bus
// stays dead until power is cycled. Clocking SCL up to 9 times releases it, then a STOP is sent.
// Must run before Wire.begin().
bool i2cBusRecover() {
  pinMode(SDA, INPUT_PULLUP);
  pinMode(SCL, OUTPUT);
  digitalWrite(SCL, HIGH);
  bool wasStuck = (digitalRead(SDA) == LOW);
  for (uint8_t i = 0; i < 9 && digitalRead(SDA) == LOW; i++) {
    digitalWrite(SCL, LOW);  delayMicroseconds(5);
    digitalWrite(SCL, HIGH); delayMicroseconds(5);
  }
  pinMode(SDA, OUTPUT);                       // STOP condition: SDA low -> high while SCL is high
  digitalWrite(SDA, LOW);  delayMicroseconds(5);
  digitalWrite(SCL, HIGH); delayMicroseconds(5);
  digitalWrite(SDA, HIGH); delayMicroseconds(5);
  pinMode(SDA, INPUT_PULLUP);
  pinMode(SCL, INPUT_PULLUP);
  return wasStuck;
}

bool i2cAcks(uint8_t addr, uint8_t tries = 5) {
  for (uint8_t i = 0; i < tries; i++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) return true;
    delay(50);
  }
  return false;
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_BUTTON, INPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_LED, OUTPUT);
  // LED steady ON means setup() is still running (or stuck). It turns OFF when setup() finishes,
  // so this works as a progress signal even if the Serial Monitor shows nothing.
  digitalWrite(PIN_LED, HIGH);

  Serial.println(F("\nBOOT smoke_test (Serial Monitor must be 115200 baud)"));

  bool wasStuck = i2cBusRecover();
  Serial.println(wasStuck ? F("I2C bus was STUCK (SDA held low), recovered. Unplug/replug USB if devices still fail.")
                          : F("I2C bus idle, ok"));
  Wire.begin();
#if defined(WIRE_HAS_TIMEOUT) && USE_WIRE_TIMEOUT
  Wire.setWireTimeout(3000, true);   // never block forever inside Wire itself
#endif
  delay(150);                        // DHT20 needs about 100 ms after power-up

  // Environment and pressure are initialized before the OLED on purpose (learned in HW2_Q10).
  // Each step prints first, so the last line on screen shows where a hang happened.
  Serial.println(F("init DHT20 (0x38)..."));
  envOk = i2cAcks(0x38) && Environment_I2C.begin();
  Serial.println(envOk ? F("  DHT20 ok") : F("  DHT20 FAILED (no ack at 0x38)"));

  Serial.println(F("init BMP280 (0x76 or 0x77)..."));
  if (i2cAcks(0x77) || i2cAcks(0x76)) {
    bmpOk = Pressure.begin();        // safe now: the sensor answers, so the driver will not spin forever
    Serial.println(bmpOk ? F("  BMP280 ok") : F("  BMP280 init FAILED"));
  } else {
    Serial.println(F("  BMP280 not found, skipped (the Seeed driver would hang forever)"));
  }

  Serial.println(F("init OLED..."));
  Oled.begin();
  Oled.setFlipMode(true);
  Oled.setFont(u8x8_font_amstrad_cpc_extended_r);
  Serial.println(F("  OLED done"));

  Serial.println(F("init LIS3DHTR..."));
  lis.begin(Wire, ACCEL_ADDR);
  delay(100);
  lis.setOutputDataRate(LIS3DHTR_DATARATE_400HZ);
  lis.setFullScaleRange(LIS3DHTR_RANGE_16G);
  lis.setHighSolution(true);
  Serial.print(F("  LIS3DHTR present: ")); Serial.println(lis.isConnection() ? F("ok") : F("FAILED"));

  Serial.print(F("free SRAM: ")); Serial.print(freeRam()); Serial.println(F(" bytes"));
  digitalWrite(PIN_LED, LOW);
  Serial.println(F("=== setup done ==="));
  printHelp();
}

void loop() {
  if (!Serial.available()) return;
  switch (Serial.read()) {
    case 'h': printHelp(); break;
    case 'i': i2cScan(); break;
    case 's': sensorSnapshot(); break;
    case 'l': liveInputs(); break;
    case 'r': rateTest(); break;
    case 'g': rangeTest(); break;
    case 't': tiltTest(); break;
    case 'o': outputTest(); break;
    case 'p': pirFinder(); break;
    default: break;
  }
}

// ---------- I2C scan ----------
void i2cScan() {
  Serial.println(F("[i] I2C scan. Expect 0x19 accel, 0x38 DHT20, 0x3C OLED, 0x76 or 0x77 BMP280."));
  uint8_t found = 0;
  for (uint8_t a = 0x08; a < 0x78; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Serial.print(F("  found 0x"));
      Serial.println(a, HEX);
      found++;
    }
  }
  Serial.print(F("  devices: "));
  Serial.println(found);
  printI2cClock();
}

// ---------- One-shot snapshot of every sensor ----------
void sensorSnapshot() {
  Serial.println(F("[s] sensor snapshot"));
  float x, y, z;
  // Accelerometer first: the kit example warns that reading it right after the
  // barometer, without a gap, can hang the bus.
  lis.getAcceleration(&x, &y, &z);
  Serial.print(F("  accel g  x=")); Serial.print(x, 3);
  Serial.print(F(" y="));          Serial.print(y, 3);
  Serial.print(F(" z="));          Serial.println(z, 3);
  delay(50);

  if (bmpOk) {
    Serial.print(F("  pressure hPa=")); Serial.println(Pressure.readPressure() / 100.0, 2);
    Serial.print(F("  altitude m="));   Serial.println(Pressure.readAltitude(), 1);
    Serial.print(F("  BMP280 temp C=")); Serial.println(Pressure.readTemperature(), 2);
  } else {
    Serial.println(F("  BMP280 unavailable (skipped)"));
  }

  if (envOk) {
    Serial.print(F("  DHT20 temp C="));  Serial.println(Environment_I2C.readTemperature(), 2);
    Serial.print(F("  DHT20 hum %="));   Serial.println(Environment_I2C.getHumidity(), 2);
  } else {
    Serial.println(F("  DHT20 unavailable (skipped)"));
  }

  Serial.print(F("  light A3="));  Serial.println(analogRead(PIN_LIGHT));
  Serial.print(F("  mic A2="));    Serial.println(analogRead(PIN_MIC));
  Serial.print(F("  pot A0="));    Serial.println(analogRead(PIN_POT));
  Serial.print(F("  button D4=")); Serial.println(digitalRead(PIN_BUTTON));
  Serial.print(F("  free SRAM=")); Serial.println(freeRam());
}

// ---------- Live analog and button values for 5 s ----------
void liveInputs() {
  Serial.println(F("[l] live inputs for 5 s: press the button, turn the pot, clap, cover the light sensor"));
  Serial.println(F("  btn,pot,mic,light"));
  for (uint8_t i = 0; i < 25; i++) {
    Serial.print(digitalRead(PIN_BUTTON)); Serial.print(',');
    Serial.print(analogRead(PIN_POT));     Serial.print(',');
    Serial.print(analogRead(PIN_MIC));     Serial.print(',');
    Serial.println(analogRead(PIN_LIGHT));
    delay(200);
  }
}

// ---------- How many new accelerometer samples can we actually read per second ----------
void rateTest() {
  Serial.println(F("[r] accelerometer read rate. Board must be still."));
  Wire.setClock(400000L);
  printI2cClock();
  lis.setFullScaleRange(LIS3DHTR_RANGE_16G);
  float x, y, z;

  uint32_t t0 = micros();
  lis.getAcceleration(&x, &y, &z);
  Serial.print(F("  one getAcceleration() call: "));
  Serial.print(micros() - t0);
  Serial.println(F(" us"));

  Serial.println(F("  ODR_Hz,new_samples_per_s"));
  for (uint8_t i = 0; i < sizeof(RATES) / sizeof(RATES[0]); i++) {
    lis.setOutputDataRate(RATES[i].odr);
    delay(50);
    uint16_t n = 0;
    uint32_t start = millis();
    while (millis() - start < 1000) {
      if (lis.available()) {
        lis.getAcceleration(&x, &y, &z);
        n++;
      }
    }
    Serial.print(F("  ")); Serial.print(RATES[i].hz);
    Serial.print(',');     Serial.println(n);
  }
  lis.setOutputDataRate(LIS3DHTR_DATARATE_400HZ);
}

// ---------- Which range saturates when the board is tapped ----------
void rangeTest() {
  Serial.println(F("[g] range test. For each range, tap the board firmly once inside the 3 s window."));
  lis.setOutputDataRate(LIS3DHTR_DATARATE_400HZ);
  float x, y, z;
  for (uint8_t i = 0; i < sizeof(RANGES) / sizeof(RANGES[0]); i++) {
    lis.setFullScaleRange(RANGES[i].range);
    Serial.print(F("  range +-")); Serial.print(RANGES[i].g);
    Serial.println(F("g: get ready..."));
    delay(2000);
    Serial.println(F("  TAP NOW"));
    float peakMag = 0, peakAxis = 0;
    uint16_t samples = 0;
    uint32_t start = millis();
    while (millis() - start < 3000) {
      if (!lis.available()) continue;
      lis.getAcceleration(&x, &y, &z);
      samples++;
      float mag = sqrt(x * x + y * y + z * z);
      float axis = max(fabs(x), max(fabs(y), fabs(z)));
      if (mag > peakMag) peakMag = mag;
      if (axis > peakAxis) peakAxis = axis;
    }
    if (samples == 0) {
      Serial.println(F("  NO SAMPLES read: the accelerometer is not responding. Unplug/replug USB, then run i and s."));
      continue;
    }
    Serial.print(F("  samples=")); Serial.print(samples);
    Serial.print(F(", peak |a|=")); Serial.print(peakMag, 2);
    Serial.print(F(" g, peak axis=")); Serial.print(peakAxis, 2);
    Serial.println(peakAxis >= 0.95f * RANGES[i].g ? F(" g  -> SATURATED") : F(" g  -> ok"));
  }
  lis.setFullScaleRange(LIS3DHTR_RANGE_16G);
}

// ---------- Roll and pitch from the gravity vector ----------
void tiltTest() {
  Serial.println(F("[t] tilt for 5 s: slowly tilt the board. roll,pitch in degrees, |a| should stay near 1 g"));
  Serial.println(F("  roll_deg,pitch_deg,mag_g"));
  float x, y, z;
  for (uint8_t i = 0; i < 10; i++) {
    lis.getAcceleration(&x, &y, &z);
    float roll = atan2(y, z) * 180.0f / PI;
    float pitch = atan2(-x, sqrt(y * y + z * z)) * 180.0f / PI;
    Serial.print(F("  ")); Serial.print(roll, 1);
    Serial.print(',');     Serial.print(pitch, 1);
    Serial.print(',');     Serial.println(sqrt(x * x + y * y + z * z), 2);
    delay(500);
  }
}

// ---------- LED, buzzer, OLED ----------
void outputTest() {
  Serial.println(F("[o] output test: LED, buzzer 1k/2k/3k Hz, OLED text"));
  Oled.clear();
  Oled.setCursor(0, 0); Oled.print(F("SMOKE TEST"));
  Oled.setCursor(0, 2); Oled.print(F("OLED OK"));
  digitalWrite(PIN_LED, HIGH);
  const uint16_t tones[] = {1000, 2000, 3000};
  for (uint8_t i = 0; i < 3; i++) {
    tone(PIN_BUZZER, tones[i]);
    delay(250);
    noTone(PIN_BUZZER);
    delay(100);
  }
  digitalWrite(PIN_LED, LOW);
  Serial.println(F("  done. Did the LED blink, the buzzer sound, and the OLED show text?"));
}

// ---------- Find which digital pin the PIR is wired to ----------
void pirFinder() {
  Serial.println(F("[p] PIR finder for 20 s. Connect the PIR first; it needs ~60 s warm-up after power-on."));
  Serial.println(F("  Wave a hand 5-10 cm in front of it repeatedly. Pins that change state are printed."));
  const uint8_t n = sizeof(PIR_CANDIDATES);
  bool last[sizeof(PIR_CANDIDATES)];
  for (uint8_t i = 0; i < n; i++) pinMode(PIR_CANDIDATES[i], INPUT_PULLUP);  // unconnected pins stay HIGH
  delay(20);
  Serial.print(F("  initial levels:"));
  for (uint8_t i = 0; i < n; i++) {
    last[i] = digitalRead(PIR_CANDIDATES[i]);
    Serial.print(' '); Serial.print(PIR_CANDIDATES[i]); Serial.print('='); Serial.print(last[i]);
  }
  Serial.println(F("   (a pin idling at 0 is probably driven by the PIR)"));
  bool any = false;
  uint32_t start = millis();
  while (millis() - start < 20000) {
    for (uint8_t i = 0; i < n; i++) {
      bool now = digitalRead(PIR_CANDIDATES[i]);
      if (now != last[i]) {
        last[i] = now;
        any = true;
        Serial.print(F("  pin ")); Serial.print(PIR_CANDIDATES[i]);
        Serial.print(F(" -> "));   Serial.println(now ? F("HIGH") : F("LOW"));
      }
    }
    delay(10);
  }
  if (!any) Serial.println(F("  no pin changed. Check the port, warm-up time, and the PIR sensitivity/delay dials."));
  for (uint8_t i = 0; i < n; i++) pinMode(PIR_CANDIDATES[i], INPUT);
}
