#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <math.h>

// ── Hardware config ───────────────────────────────────────
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
#define OLED_ADDRESS  0x3c
#define SDA_PIN       21
#define SCL_PIN       22
#define BUTTON_PIN    0

// ── MPU Registers ─────────────────────────────────────────
#define MPU_PWR_MGMT_1   0x6B
#define MPU_SMPLRT_DIV   0x19
#define MPU_DLPF_CONFIG  0x1A
#define MPU_GYRO_CONFIG  0x1B
#define MPU_ACCEL_CONFIG 0x1C
#define MPU_ACCEL_XOUT   0x3B
#define MPU_TEMP_OUT     0x41
#define MPU_GYRO_XOUT    0x43
#define MPU_WHO_AM_I     0x75
#define MPU_MOT_THR      0x1F
#define MPU_MOT_DUR      0x20
#define MPU_FF_THR       0x1D
#define MPU_FF_DUR       0x1E
#define MPU_INT_ENABLE   0x38
#define MPU_INT_STATUS   0x3A

// ── Pages ─────────────────────────────────────────────────
#define TOTAL_PAGES      7
#define PAGE_ORIENTATION 0
#define PAGE_ACCEL       1
#define PAGE_GYRO        2
#define PAGE_ANGLES      3
#define PAGE_TEMP        4
#define PAGE_GFORCE      5
#define PAGE_EVENTS      6

#define FLAT_THRESHOLD   15.0f
#define FREEFALL_GFORCE  0.3f
#define SHOCK_GFORCE     2.5f
#define ALPHA            0.96f

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

uint8_t MPU_ADDR = 0x68;

// ── Sensor values ─────────────────────────────────────────
float ax, ay, az;
float gx, gy, gz;
float tempC, tempF;
float pitch = 0, roll = 0, yaw = 0;
float accelPitch, accelRoll;
float gForce;

// ── Events ────────────────────────────────────────────────
bool  freeFallDetected = false, shockDetected = false, motionDetected = false;
unsigned long freeFallTime = 0, shockTime = 0, motionTime = 0;
int   freeFallCount = 0, shockCount = 0;

// ── Timing & button ───────────────────────────────────────
unsigned long lastTime = 0;
int  currentPage = 0;
bool lastBtnState = HIGH;
unsigned long lastDebounce = 0;

// ─────────────────────────────────────────────────────────
// I2C helpers
// ─────────────────────────────────────────────────────────
void mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission(true);
}

uint8_t mpuReadByte(uint8_t reg) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, (uint8_t)1, (uint8_t)true);
  return Wire.available() ? Wire.read() : 0;
}

void mpuBurstRead(int16_t &rax, int16_t &ray, int16_t &raz,
                  int16_t &rtp,
                  int16_t &rgx, int16_t &rgy, int16_t &rgz) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_ACCEL_XOUT);
  Wire.endTransmission(false);
  uint8_t n = Wire.requestFrom(MPU_ADDR, (uint8_t)14, (uint8_t)true);

  if (n < 14) {
    Serial.printf("Burst read fail: got %d bytes\n", n);
    rax = ray = raz = rtp = rgx = rgy = rgz = 0;
    return;
  }

  rax = (Wire.read() << 8) | Wire.read();
  ray = (Wire.read() << 8) | Wire.read();
  raz = (Wire.read() << 8) | Wire.read();
  rtp = (Wire.read() << 8) | Wire.read();
  rgx = (Wire.read() << 8) | Wire.read();
  rgy = (Wire.read() << 8) | Wire.read();
  rgz = (Wire.read() << 8) | Wire.read();
}

// ─────────────────────────────────────────────────────────
// Find MPU address
// ─────────────────────────────────────────────────────────
bool findMPU() {
  uint8_t addrs[] = {0x68, 0x69};
  for (int i = 0; i < 2; i++) {
    Wire.beginTransmission(addrs[i]);
    if (Wire.endTransmission() == 0) {
      MPU_ADDR = addrs[i];
      Serial.printf("MPU found at 0x%02X\n", MPU_ADDR);
      return true;
    }
    delay(10);
  }
  return false;
}

// ─────────────────────────────────────────────────────────
// MPU Init
// ─────────────────────────────────────────────────────────
void mpuInit() {
  mpuWrite(MPU_PWR_MGMT_1,   0x80);  // Full reset
  delay(200);
  mpuWrite(MPU_PWR_MGMT_1,   0x00);  // Wake up
  delay(200);
  mpuWrite(MPU_SMPLRT_DIV,   0x07);  // 125Hz
  mpuWrite(MPU_DLPF_CONFIG,  0x03);  // 44Hz DLPF
  mpuWrite(MPU_GYRO_CONFIG,  0x00);  // ±250°/s
  mpuWrite(MPU_ACCEL_CONFIG, 0x00);  // ±2g
  mpuWrite(MPU_FF_THR,       0x0A);
  mpuWrite(MPU_FF_DUR,       0x05);
  mpuWrite(MPU_MOT_THR,      0x14);
  mpuWrite(MPU_MOT_DUR,      0x28);
  mpuWrite(MPU_INT_ENABLE,   0x07);
  delay(100);

  // Verify woke up
  uint8_t pwr = mpuReadByte(MPU_PWR_MGMT_1);
  Serial.printf("PWR_MGMT_1 after init = 0x%02X (want 0x00)\n", pwr);
}

// ─────────────────────────────────────────────────────────
// Read sensors — raw, no filtering
// ─────────────────────────────────────────────────────────
void readSensors() {
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0f;
  if (dt <= 0 || dt > 0.5f) dt = 0.008f;
  lastTime = now;

  int16_t rax, ray, raz, rtp, rgx, rgy, rgz;
  mpuBurstRead(rax, ray, raz, rtp, rgx, rgy, rgz);

  // Convert to physical units — no offsets, no filters
  ax = rax / 16384.0f;
  ay = ray / 16384.0f;
  az = raz / 16384.0f;
  gx = rgx / 131.0f;
  gy = rgy / 131.0f;
  gz = rgz / 131.0f;
  tempC = rtp / 340.0f + 36.53f;
  tempF = tempC * 9.0f / 5.0f + 32.0f;

  // Angles
  accelPitch = atan2f(-ax, sqrtf(ay*ay + az*az)) * 180.0f / PI;
  accelRoll  = atan2f(ay, az) * 180.0f / PI;

  // Complementary filter
  pitch = ALPHA * (pitch + gy * dt) + (1.0f - ALPHA) * accelPitch;
  roll  = ALPHA * (roll  + gx * dt) + (1.0f - ALPHA) * accelRoll;
  yaw  += gz * dt;
  if (yaw >  180.0f) yaw -= 360.0f;
  if (yaw < -180.0f) yaw += 360.0f;

  gForce = sqrtf(ax*ax + ay*ay + az*az);

  // Events
  uint8_t intStatus = mpuReadByte(MPU_INT_STATUS);

  if ((intStatus & 0x80) || gForce < FREEFALL_GFORCE) {
    if (!freeFallDetected) { freeFallDetected = true; freeFallTime = millis(); freeFallCount++; }
  } else if (millis() - freeFallTime > 1500) freeFallDetected = false;

  if (intStatus & 0x40) { motionDetected = true; motionTime = millis(); }
  else if (millis() - motionTime > 2000) motionDetected = false;

  if (gForce > SHOCK_GFORCE) {
    if (!shockDetected) { shockDetected = true; shockTime = millis(); shockCount++; }
  } else if (millis() - shockTime > 1500) shockDetected = false;
}

// ─────────────────────────────────────────────────────────
// Button
// ─────────────────────────────────────────────────────────
void handleButton() {
  bool state = digitalRead(BUTTON_PIN);
  if (state == LOW && lastBtnState == HIGH &&
      millis() - lastDebounce > 200) {
    currentPage = (currentPage + 1) % TOTAL_PAGES;
    lastDebounce = millis();
    Serial.printf("Page -> %d\n", currentPage);
  }
  lastBtnState = state;
}

// ─────────────────────────────────────────────────────────
// Draw helpers
// ─────────────────────────────────────────────────────────
void drawTitleBar(const char* title) {
  display.fillRect(0, 0, 128, 11, SSD1306_WHITE);
  display.setTextColor(SSD1306_BLACK);
  display.setTextSize(1);
  display.setCursor(2, 2);
  display.print(title);
  display.setCursor(104, 2);
  display.print(currentPage + 1);
  display.print("/");
  display.print(TOTAL_PAGES);
  display.setTextColor(SSD1306_WHITE);
}

void drawSpiritLevel(float p, float r) {
  int cx = 96, cy = 37;
  int dx = constrain((int)(r / 90.0f * 18), -18, 18);
  int dy = constrain((int)(p / 90.0f * 18), -18, 18);
  display.drawCircle(cx, cy, 20, SSD1306_WHITE);
  display.drawFastHLine(cx-20, cy, 40, SSD1306_WHITE);
  display.drawFastVLine(cx, cy-20, 40, SSD1306_WHITE);
  display.fillCircle(cx+dx, cy+dy, 4, SSD1306_WHITE);
}

void drawBar(int x, int y, int w, int h, float val, float maxVal) {
  display.drawRect(x, y, w, h, SSD1306_WHITE);
  int filled = constrain((int)(fabsf(val) / maxVal * (w-2)), 0, w-2);
  display.fillRect(x+1, y+1, filled, h-2, SSD1306_WHITE);
}

String getOrientation() {
  float ap = fabsf(pitch), ar = fabsf(roll);
  if (ar > 155)                           return "UPSIDE DOWN";
  if (ap < FLAT_THRESHOLD && ar < FLAT_THRESHOLD)
    return (az > 0) ? "FLAT (UP)" : "FLAT (DOWN)";
  if (ap > ar) return (pitch > 0) ? "TILT FORWARD" : "TILT BACK";
  else         return (roll  > 0) ? "TILT RIGHT"   : "TILT LEFT";
}

// ─────────────────────────────────────────────────────────
// Pages
// ─────────────────────────────────────────────────────────
void drawPageOrientation() {
  drawTitleBar(" ORIENTATION");
  display.setTextSize(1);
  display.setCursor(0, 14); display.print("Mode:");
  display.setCursor(0, 24); display.print("> "); display.print(getOrientation());
  display.setCursor(0, 39); display.print("P:"); display.print(pitch, 1); display.print((char)247);
  display.setCursor(0, 49); display.print("R:"); display.print(roll,  1); display.print((char)247);
  drawSpiritLevel(pitch, roll);
}

void drawPageAccel() {
  drawTitleBar(" ACCELEROMETER");
  display.setTextSize(1);
  display.setCursor(0, 14); display.print("X:"); display.print(ax, 3); display.print("g");
  drawBar(68, 14, 58, 7, ax, 2.0f);
  display.setCursor(0, 26); display.print("Y:"); display.print(ay, 3); display.print("g");
  drawBar(68, 26, 58, 7, ay, 2.0f);
  display.setCursor(0, 38); display.print("Z:"); display.print(az, 3); display.print("g");
  drawBar(68, 38, 58, 7, az, 2.0f);
  display.setCursor(0, 53); display.print("Range: +-2g  125Hz");
}

void drawPageGyro() {
  drawTitleBar("   GYROSCOPE");
  display.setTextSize(1);
  display.setCursor(0, 14); display.print("X:"); display.print(gx, 1); display.print((char)247); display.print("/s");
  drawBar(78, 14, 48, 7, gx, 250.0f);
  display.setCursor(0, 26); display.print("Y:"); display.print(gy, 1); display.print((char)247); display.print("/s");
  drawBar(78, 26, 48, 7, gy, 250.0f);
  display.setCursor(0, 38); display.print("Z:"); display.print(gz, 1); display.print((char)247); display.print("/s");
  drawBar(78, 38, 48, 7, gz, 250.0f);
  display.setCursor(0, 53); display.print("Range: +-250 deg/s");
}

void drawPageAngles() {
  drawTitleBar("    ANGLES");
  display.setTextSize(1);
  display.setCursor(0, 14); display.print("Pitch: "); display.print(pitch, 2); display.print((char)247);
  display.setCursor(0, 24); display.print("Roll:  "); display.print(roll,  2); display.print((char)247);
  display.setCursor(0, 34); display.print("Yaw:   "); display.print(yaw,   2); display.print((char)247);
  display.drawFastHLine(0, 45, 128, SSD1306_WHITE);
  display.setCursor(0, 49); display.print("AP:"); display.print(accelPitch, 1); display.print((char)247);
  display.print(" AR:"); display.print(accelRoll, 1); display.print((char)247);
}

void drawPageTemp() {
  drawTitleBar("  TEMPERATURE");
  display.setTextSize(3);
  display.setCursor(5, 17); display.print(tempC, 1);
  display.setTextSize(2);   display.print((char)247); display.print("C");
  display.setTextSize(1);
  display.setCursor(5, 52); display.print(tempF, 1); display.print((char)247); display.print("F (internal)");
}

void drawPageGForce() {
  drawTitleBar("   G-FORCE");
  display.setTextSize(2);
  display.setCursor(5, 14); display.print(gForce, 2); display.print("g");
  display.setTextSize(1);
  drawBar(0, 35, 128, 10, gForce, 4.0f);
  display.setCursor(0, 50);
  if      (gForce < FREEFALL_GFORCE)          display.print("!! FREE FALL !!");
  else if (gForce > SHOCK_GFORCE)             display.print("!! SHOCK/IMPACT !!");
  else if (gForce > 0.9f && gForce < 1.1f)   display.print("Normal (1g)");
  else                                        display.print("Movement detected");
}

void drawPageEvents() {
  drawTitleBar(" EVENTS & ALERTS");
  display.setTextSize(1);
  display.setCursor(0, 14); display.print("FreeFall: ");
  if (freeFallDetected) display.print("!! YES !!");
  else { display.print("No ("); display.print(freeFallCount); display.print("x)"); }
  display.setCursor(0, 25); display.print("Shock:    ");
  if (shockDetected) display.print("!! YES !!");
  else { display.print("No ("); display.print(shockCount); display.print("x)"); }
  display.setCursor(0, 36); display.print("Motion:   ");
  display.print(motionDetected ? "ACTIVE" : "Still");
  display.drawFastHLine(0, 47, 128, SSD1306_WHITE);
  display.setCursor(0, 51); display.print("G:"); display.print(gForce, 2);
  display.print("g  T:"); display.print(tempC, 1); display.print((char)247); display.print("C");
}

// ─────────────────────────────────────────────────────────
// Setup
// ─────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);
  delay(500);

  // ── OLED ──────────────────────────────────────────────
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
    Serial.println("OLED failed!");
    while (true);
  }
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(15, 20); display.print("Initializing...");
  display.display();
  delay(500);

  // ── Find MPU ──────────────────────────────────────────
  display.clearDisplay();
  display.setCursor(10, 20); display.print("Finding MPU6050...");
  display.display();

  if (!findMPU()) {
    display.clearDisplay();
    display.setCursor(0, 20); display.print("MPU6050 NOT FOUND!");
    display.setCursor(0, 35); display.print("Check wiring.");
    display.display();
    while (true);
  }

  display.clearDisplay();
  display.setCursor(5, 20); display.printf("MPU found: 0x%02X", MPU_ADDR);
  display.display();
  delay(800);

  mpuInit();

  // ── Verify real data is coming ────────────────────────
  int16_t rax, ray, raz, rtp, rgx, rgy, rgz;
  mpuBurstRead(rax, ray, raz, rtp, rgx, rgy, rgz);
  Serial.printf("First read → Ax:%d Ay:%d Az:%d Gx:%d Gy:%d Gz:%d Tp:%d\n",
                rax, ray, raz, rgx, rgy, rgz, rtp);

  if (rax == 0 && ray == 0 && raz == 0) {
    display.clearDisplay();
    display.setCursor(0, 15); display.print("MPU reading zeros!");
    display.setCursor(0, 28); display.print("Try: AD0 -> GND");
    display.setCursor(0, 41); display.print("or press EN/Reset");
    display.display();
    Serial.println("WARNING: All zeros — MPU not responding to reads!");
    delay(3000);   // show warning then continue anyway
  }

  lastTime = millis();

  display.clearDisplay();
  display.setCursor(10, 20); display.print("Ready!");
  display.setCursor(0,  35); display.print("Press BOOT to");
  display.setCursor(0,  45); display.print("change page");
  display.display();
  delay(1200);
}

// ─────────────────────────────────────────────────────────
// Loop
// ─────────────────────────────────────────────────────────
void loop() {
  handleButton();
  readSensors();

  display.clearDisplay();
  switch (currentPage) {
    case PAGE_ORIENTATION: drawPageOrientation(); break;
    case PAGE_ACCEL:       drawPageAccel();       break;
    case PAGE_GYRO:        drawPageGyro();        break;
    case PAGE_ANGLES:      drawPageAngles();      break;
    case PAGE_TEMP:        drawPageTemp();        break;
    case PAGE_GFORCE:      drawPageGForce();      break;
    case PAGE_EVENTS:      drawPageEvents();      break;
  }
  display.display();

  Serial.printf("P:%.1f R:%.1f Y:%.1f | Ax:%.3f Ay:%.3f Az:%.3f | Gx:%.1f Gy:%.1f Gz:%.1f | G:%.2fg | T:%.1fC\n",
    pitch, roll, yaw, ax, ay, az, gx, gy, gz, gForce, tempC);

  delay(50);
}