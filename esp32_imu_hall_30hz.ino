#include <Arduino.h>
#include <Wire.h>
#include <vl53l4cd_class.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Preferences.h>

#define DEV_I2C    Wire
#define SerialPort Serial
#define HALL_PIN   4
#define BTN_PIN    27

VL53L4CD tof(&DEV_I2C, -1);
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &DEV_I2C);
Preferences prefs;
bool haveCalib = false;

volatile uint32_t cutCount  = 0;
volatile uint32_t lastCutMs = 0;
const uint32_t CUT_DEBOUNCE_MS = 150;
uint32_t lastPrintedCount = 0;

// --- Marker button ---
const uint16_t BTN_DEBOUNCE_MS = 50;
bool     btnLast       = HIGH;    // INPUT_PULLUP idles HIGH
uint32_t btnLastChange = 0;
bool     markPending   = false;   // set on press, cleared once logged
uint32_t markCount     = 0;       // running total of presses
// basically the button will read 0 when not pressed and 1 when you press it, then return back to 0
// i have also a aprt that counts how many times that button has been pressed 


const uint32_t SAMPLE_PERIOD_MS = 33;   // ~30 Hz (1000/33 = 30.3) basically about 30
uint32_t lastSample = 0;

void IRAM_ATTR onCut() {
  uint32_t now = millis();
  if (now - lastCutMs > CUT_DEBOUNCE_MS) {
    cutCount++;
    lastCutMs = now;
  }
}

// Runs every pass through loop(), not on the 33 ms schedule,
// so a quick press can't fall between samples.
void pollButton() {
  uint32_t now = millis();
  bool level = digitalRead(BTN_PIN);

  if (level != btnLast && (now - btnLastChange) > BTN_DEBOUNCE_MS) {
    btnLastChange = now;
    btnLast = level;
    if (level == LOW) {           // pressed: pin pulled down to GND
      markPending = true;
      markCount++;
    }
  }
}

void setup() {
  SerialPort.begin(115200);
  delay(1000);

  DEV_I2C.begin();
  DEV_I2C.setClock(100000);

  // --- ToF  ---
  tof.begin();
  tof.VL53L4CD_Off();
  tof.InitSensor();
  tof.VL53L4CD_SetRangeTiming(20, 0);   // 20 ms budget to keep up with 30 Hz
  tof.VL53L4CD_StartRanging();

  // --- IMU  ---
  if (!bno.begin()) {
    SerialPort.println("BNO055 not found — check wiring/address");
    while (1) delay(10);
  }
  delay(1000);
  bno.setMode(OPERATION_MODE_IMUPLUS);
  delay(25);
  bno.setExtCrystalUse(true);

  // Load saved gyro+accel calibration
  prefs.begin("bno055", false);
  if (prefs.isKey("calib")) {
    adafruit_bno055_offsets_t calib;
    prefs.getBytes("calib", &calib, sizeof(calib));
    bno.setSensorOffsets(calib);
    haveCalib = true;
    SerialPort.println("Loaded saved calibration. Ready.");
  } else {
    SerialPort.println("No saved calibration found.");
  }

  // --- Hall sensor (active-low module) ---
  pinMode(HALL_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(HALL_PIN), onCut, FALLING);

  // --- Marker button: one leg to GPIO 27, diagonal leg to GND ---
  pinMode(BTN_PIN, INPUT_PULLUP);
  btnLast = digitalRead(BTN_PIN);

  SerialPort.println("All sensors ready.");
}

void loop() {
  // Button is checked on EVERY pass, before the sample-rate gate
  pollButton();

  // Run the body on a fixed ~33 ms schedule
  if (millis() - lastSample < SAMPLE_PERIOD_MS) return;
  lastSample = millis();

  // --- ToF distance ---
  uint16_t distance = 0;
  bool haveDist = false;
  uint8_t ready = 0;
  tof.VL53L4CD_CheckForDataReady(&ready);
  if (ready) {
    VL53L4CD_Result_t r;
    tof.VL53L4CD_ClearInterrupt();
    tof.VL53L4CD_GetResult(&r);
    distance = r.distance_mm;
    haveDist = true;
  }

  // --- IMU linear acceleration ---
  sensors_event_t linAcc;
  bno.getEvent(&linAcc, Adafruit_BNO055::VECTOR_LINEARACCEL);

  // --- Hall: live magnet presence + running cut count ---
  // note to myself make sure you are using the correct magnet polarity. one side will work better than the other 
  int hallState = !digitalRead(HALL_PIN);   // active-low: invert so present = 1
  uint32_t cuts = cutCount;

  // --- Put the mark on exactly one row ---
  int mark = markPending ? 1 : 0;
  markPending = false;

  // --- Output ---
  SerialPort.print(millis());
  SerialPort.print(" ms | Dist:");
  if (haveDist) SerialPort.print(distance); else SerialPort.print("--");
  SerialPort.print(" mm | LinAcc X:");
  SerialPort.print(linAcc.acceleration.x, 2);
  SerialPort.print(" Y:");
  SerialPort.print(linAcc.acceleration.y, 2);
  SerialPort.print(" Z:");
  SerialPort.print(linAcc.acceleration.z, 2);
  SerialPort.print(" m/s^2 | Magnet:");
  SerialPort.print(hallState);
  SerialPort.print(" | Cuts:");
  SerialPort.print(cuts);
  SerialPort.print(" | Mark:");
  SerialPort.print(mark);
  SerialPort.print(" | Marks:");
  SerialPort.println(markCount);

  if (cuts != lastPrintedCount) {
    SerialPort.print(">>> CUT #");
    SerialPort.println(cuts);
    lastPrintedCount = cuts;
  }

  if (mark) {
    SerialPort.print(">>> MARK #");
    SerialPort.println(markCount);
  }
}

// hall sensor check uncomment this section to check it 
// uncomment this section to test the magnet polarity. I had this issue before where it wasn't reading bc of the way it was facing. 
// also note we are using the 26N magnets that was in the agricultural space GRaf

// #define HALL_PIN 4

// void setup() {
//   Serial.begin(115200);
//   pinMode(HALL_PIN, INPUT);
// }

// void loop() {
//   Serial.print("magnet: ");
//   Serial.println(!digitalRead(HALL_PIN));   // inverted: 1 = magnet present, 0 = absent ; this may change based on your magnet 
//   delay(100);
// }
