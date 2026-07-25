/*
 * SparkFun Thing Plus ESP32 - GNSS-to-SD Logger
 * ------------------------------------------------
 * Behavior:
 *   1. On power-up, the board does nothing (idle).
 *   2. When the switch on GPIO25 reads LOW, power up the Qwiic bus and
 *      the GNSS module, and blink the onboard status LED while waiting
 *      for a valid satellite fix. No data is recorded during this time.
 *   3. Once a valid 3D fix is obtained, open a CSV file on the SD card,
 *      switch the status LED to solid ON, and begin logging GNSS data
 *      (from a SparkFun GPS-RTK-SMA / u-blox module, connected via
 *      Qwiic) at 1 Hz.
 *   4. When the switch reads HIGH again, close the file (if open), turn
 *      off the status LED, power down the Qwiic bus, and return to idle.
 *
 * Libraries required (install via Library Manager):
 *   - "SparkFun u-blox GNSS v3" by SparkFun Electronics
 *   - "SD" (bundled with the ESP32 core)
 *
 * Wiring notes:
 *   - GNSS module: connected via Qwiic (I2C, default SDA=21, SCL=22 on
 *     ESP32 Thing Plus). GPIO0 is toggled HIGH to enable the XC6222 LDO
 *     regulator's power output to the Qwiic connector (up to 700mA at
 *     3.3V). Note: GPIO0 is a boot-strapping pin, but it's only touched
 *     here well after boot has completed, inside setup()/onSwitchLow(),
 *     so this is safe.
 *   - SD card: uses the onboard microSD slot on the Thing Plus C
 *     (SPI, CS = GPIO5 by default on that board -- verify against your
 *     specific revision's schematic and adjust SD_CS_PIN if needed)
 *   - Switch (SPDT): wired to GPIO25, which has an internal pull-up
 *     enabled in software (INPUT_PULLUP) -- no external resistor needed.
 *     Switch reads HIGH at power-on (idle); moving the switch pulls it
 *     LOW to begin acquiring/recording.
 *   - Status LED: onboard LED (GPIO13 on the SparkFun Thing Plus ESP32).
 *     Blinks while waiting for a GNSS fix, solid ON while recording,
 *     off while idle.
 *
 * State machine overview:
 *
 *      IDLE  --switch LOW-->  WAITING_FOR_FIX  --valid fix-->  RECORDING
 *       ^                            |                              |
 *       |                            |                              |
 *       +-------------- switch HIGH -+------------------------------+
 *
 *   IDLE:             Everything powered down. Nothing happens.
 *   WAITING_FOR_FIX:   Qwiic bus + GNSS module powered on, LED blinking,
 *                      polling for a fix good enough to start recording.
 *   RECORDING:         Log file open, LED solid, writing one row per
 *                      second as new GNSS data arrives.
 */

#include <Wire.h>                       // I2C bus, used to talk to the GNSS module
#include <SPI.h>                        // SPI bus, used by the SD card
#include <SD.h>                         // SD card read/write (bundled with ESP32 core)
#include <SparkFun_u-blox_GNSS_v3.h>    // High-level u-blox GNSS driver

// =======================================================================
// PIN DEFINITIONS
// =======================================================================
#define SWITCH_PIN     25   // SPDT switch input. Internal pull-up enabled in
                             // setup(), so idle/open = HIGH, closed = LOW.
#define SD_CS_PIN      5    // SPI chip-select for the onboard microSD slot.
                             // Confirm against your board's schematic.
#define STATUS_LED_PIN 13   // Onboard status LED on the Thing Plus ESP32.
#define QWIIC_PWR_PIN  0    // Drives the XC6222 LDO enable line. HIGH turns
                             // on 3.3V power (up to 700mA) to the Qwiic
                             // connector; LOW cuts it off to save power
                             // while idle.

// =======================================================================
// TIMING / THRESHOLD CONSTANTS
// =======================================================================
const unsigned long DEBOUNCE_MS = 50;   // How long the switch reading must
                                         // stay stable before we trust it.
                                         // Filters out mechanical "bounce"
                                         // so one physical flip can't be
                                         // misread as multiple transitions.

const unsigned long BLINK_MS    = 250;  // Half-period of the status LED
                                         // blink while waiting for a fix
                                         // (LED toggles every BLINK_MS).

const byte MIN_FIX_TYPE         = 3;    // Minimum GNSS fixType required
                                         // before we start recording.
                                         // 3 = "3D Fix" (see writeGnssRow()
                                         // for the full fixType table).
                                         // Raise this if you'd rather wait
                                         // for a specific fix quality.

// =======================================================================
// GLOBAL OBJECTS
// =======================================================================
SFE_UBLOX_GNSS myGNSS;   // Driver object for the u-blox GNSS module.
File logFile;            // Currently open log file. Only valid/open while
                          // state == RECORDING; the SD library's File
                          // class evaluates as "false" when not open, which
                          // several functions below rely on.

// =======================================================================
// STATE MACHINE
// =======================================================================
// The entire program's behavior is driven by which of these three states
// we're in. See the state diagram in the header comment above.
enum LoggerState { IDLE, WAITING_FOR_FIX, RECORDING };
LoggerState state = IDLE;

// --- Switch debouncing state ---
int lastRawReading  = HIGH;     // Raw (unfiltered) pin reading from the
                                 // previous loop() iteration.
int stableReading    = HIGH;    // The debounced switch reading we actually
                                 // trust and act on.
unsigned long lastDebounceTime = 0; // millis() timestamp of the last time
                                     // the raw reading changed.

// --- LED blink state (used only while WAITING_FOR_FIX) ---
unsigned long lastBlinkTime = 0; // millis() timestamp of the last LED toggle.
bool ledOn = false;               // Current commanded LED state while blinking.

// --- One-time GNSS setup guard ---
bool gnssInitialized = false;    // True once myGNSS.begin() + configuration
                                  // has succeeded at least once, so we don't
                                  // redo that setup on every switch cycle.

// =======================================================================
// setup() -- runs once at power-on / reset
// =======================================================================
// Deliberately does as little as possible: it only arms the I/O pins.
// No I2C, no SD card mount, no GNSS communication happens here, which is
// what makes "power on -> do nothing" literally true. Everything else is
// initialized lazily, the first time the switch actually triggers it.
void setup() {
  Serial.begin(115200);
  delay(500); // brief pause so the Serial Monitor has time to connect

  pinMode(SWITCH_PIN, INPUT_PULLUP); // GPIO25 supports an internal pull-up,
                                      // so no external resistor is needed.

  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LOW); // LED off at boot

  pinMode(QWIIC_PWR_PIN, OUTPUT);
  digitalWrite(QWIIC_PWR_PIN, LOW);  // Qwiic bus unpowered until needed

  Serial.println("System idle. Waiting for switch on GPIO25 to go LOW...");
}

// =======================================================================
// loop() -- runs continuously
// =======================================================================
// Two jobs, every pass:
//   1. Read and debounce the switch, and fire the appropriate transition
//      function (onSwitchLow / onSwitchHigh) when it changes state.
//   2. Do whatever ongoing work the current state requires (blinking the
//      LED and polling for a fix, or logging new GNSS fixes).
void loop() {
  int rawReading = digitalRead(SWITCH_PIN);

  // --- Debounce logic ---
  // Every time the raw reading changes, reset the debounce timer. Only
  // once the reading has held steady for DEBOUNCE_MS do we treat it as a
  // real, intentional switch transition and act on it.
  if (rawReading != lastRawReading) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > DEBOUNCE_MS) {
    if (rawReading != stableReading) {
      stableReading = rawReading;

      // Only start if we're fully idle, and only stop if we're doing
      // something (waiting or recording) -- this makes each transition
      // function idempotent with respect to repeated/spurious triggers.
      if (stableReading == LOW && state == IDLE) {
        onSwitchLow();
      } else if (stableReading == HIGH && state != IDLE) {
        onSwitchHigh();
      }
    }
  }
  lastRawReading = rawReading;

  // --- Per-state background work ---
  if (state == WAITING_FOR_FIX) {
    updateBlinkingLed();                  // keep the LED blinking
    checkForFixAndMaybeStartLogging();    // watch for a good enough fix
  } else if (state == RECORDING) {
    logIfNewFixAvailable();               // write a row whenever new data arrives
  }
  // Nothing to do here when state == IDLE.
}

// =======================================================================
// onSwitchLow() -- called once, exactly when the switch transitions to LOW
// =======================================================================
// Powers up the Qwiic bus and GNSS module, mounts the SD card, and moves
// into WAITING_FOR_FIX. Deliberately does NOT open a log file yet -- that
// only happens once checkForFixAndMaybeStartLogging() sees a valid fix,
// so no empty or garbage-data files are created if the switch is flipped
// back off before a fix is ever acquired.
void onSwitchLow() {
  Serial.println("Switch LOW detected -- powering up and waiting for GNSS fix.");

  // Power up the Qwiic bus before talking to the GNSS module.
  digitalWrite(QWIIC_PWR_PIN, HIGH);
  delay(1000); // u-blox modules can take up to ~1s to boot and respond on I2C

  // --- SD card ---
  // SD.begin() mounts the filesystem on the microSD card via the SPI bus.
  // If this fails (no card inserted, bad card, wrong CS pin, etc.) there's
  // nowhere to log to, so bail back to idle immediately.
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("ERROR: SD card initialization failed. Staying idle.");
    digitalWrite(QWIIC_PWR_PIN, LOW);
    state = IDLE;
    return;
  }

  // --- GNSS module ---
  // Only configure the module the first time this ever succeeds; on later
  // switch cycles the module is already configured and we skip straight
  // to waiting for a fix.
  if (!gnssInitialized) {
    Wire.begin(); // start the I2C bus (default pins: SDA=21, SCL=22)

    // Try a few times, spaced out, in case the module is still finishing
    // its own boot sequence when we first ask for it.
    bool gnssFound = false;
    for (int attempt = 1; attempt <= 5; attempt++) {
      Serial.print("Looking for GNSS module (attempt ");
      Serial.print(attempt);
      Serial.println(" of 5)...");
      if (myGNSS.begin(Wire)) {
        gnssFound = true;
        break;
      }
      delay(500);
    }

    if (!gnssFound) {
      // Couldn't find the module at all -- scan the I2C bus and print
      // whatever IS responding, to help diagnose wiring vs. power issues.
      Serial.println("ERROR: u-blox GNSS module not detected. Staying idle.");
      scanI2CBus();
      digitalWrite(QWIIC_PWR_PIN, LOW);
      state = IDLE;
      return;
    }

    // Module found -- configure it once.
    myGNSS.setI2COutput(COM_TYPE_UBX);   // use UBX protocol on I2C
    myGNSS.setNavigationFrequency(1);    // 1 fix per second
    myGNSS.setAutoPVT(true);             // module pushes a fresh PVT
                                          // (Position/Velocity/Time) packet
                                          // every cycle; getPVT() then just
                                          // checks whether one has arrived,
                                          // rather than requesting one fresh
                                          // each call.
    gnssInitialized = true;
  }

  // Everything is powered and ready -- start waiting for a fix.
  Serial.println("Waiting for a valid GNSS fix before recording begins...");
  lastBlinkTime = millis();
  ledOn = false;
  digitalWrite(STATUS_LED_PIN, LOW);
  state = WAITING_FOR_FIX;
}

// =======================================================================
// onSwitchHigh() -- called once, exactly when the switch transitions to HIGH
// =======================================================================
// Safe to call whether we were WAITING_FOR_FIX (no file was ever opened)
// or RECORDING (a file is open and needs to be flushed/closed). Always
// leaves the system fully powered down and back in IDLE.
void onSwitchHigh() {
  Serial.println("Switch HIGH detected -- stopping.");

  // logFile evaluates "true" only if a file is currently open (i.e. we
  // had reached RECORDING). If we were only WAITING_FOR_FIX, this is
  // skipped since there's nothing to close.
  if (logFile) {
    logFile.flush();
    logFile.close();
    Serial.println("File closed.");
  }

  digitalWrite(STATUS_LED_PIN, LOW);
  digitalWrite(QWIIC_PWR_PIN, LOW); // power down the Qwiic bus while idle

  state = IDLE;
  Serial.println("System idle.");
}

// =======================================================================
// updateBlinkingLed() -- non-blocking LED blink while WAITING_FOR_FIX
// =======================================================================
// Uses millis() instead of delay() so it never blocks loop() -- the
// switch and GNSS polling both stay responsive while the LED blinks.
void updateBlinkingLed() {
  if (millis() - lastBlinkTime >= BLINK_MS) {
    lastBlinkTime = millis();
    ledOn = !ledOn;
    digitalWrite(STATUS_LED_PIN, ledOn ? HIGH : LOW);
  }
}

// =======================================================================
// checkForFixAndMaybeStartLogging() -- called every loop() while WAITING_FOR_FIX
// =======================================================================
// Polls the GNSS module for new data. As soon as a fix meeting
// MIN_FIX_TYPE arrives, this opens a new log file, writes the CSV header,
// switches the LED to solid ON, transitions to RECORDING, and logs that
// first valid fix immediately (rather than waiting a further second for
// the next one).
void checkForFixAndMaybeStartLogging() {
  // getPVT() returns true only when a new Position/Velocity/Time packet
  // has arrived since the last call. If nothing new has arrived yet,
  // there's nothing to check this pass.
  if (!myGNSS.getPVT()) {
    return;
  }

  byte fixType = myGNSS.getFixType();
  if (fixType < MIN_FIX_TYPE) {
    return; // fix isn't good enough yet -- keep waiting (and blinking)
  }

  // --- Valid fix acquired: open the log file ---
  // Find the next unused filename (gpslog_000.csv, gpslog_001.csv, ...)
  // so each recording session gets its own file and nothing is overwritten.
  char filename[24];
  int fileIndex = 0;
  do {
    snprintf(filename, sizeof(filename), "/gpslog_%03d.csv", fileIndex);
    fileIndex++;
  } while (SD.exists(filename) && fileIndex < 1000);

  logFile = SD.open(filename, FILE_WRITE);
  if (!logFile) {
    Serial.println("ERROR: Could not open log file on SD card. Staying idle.");
    digitalWrite(STATUS_LED_PIN, LOW);
    digitalWrite(QWIIC_PWR_PIN, LOW);
    state = IDLE;
    return;
  }

  // Write the CSV header row once, at the top of the file.
  logFile.println("date,time,lat_deg,lon_deg,fix_type,rtk_status");

  Serial.print("Valid fix acquired. Logging to: ");
  Serial.println(filename);

  digitalWrite(STATUS_LED_PIN, HIGH); // switch from blinking to solid ON
  state = RECORDING;

  writeGnssRow(); // log this first valid fix right away, don't wait ~1s more
}

// =======================================================================
// logIfNewFixAvailable() -- called every loop() while RECORDING
// =======================================================================
// Mirrors the polling pattern used in checkForFixAndMaybeStartLogging():
// getPVT() only returns true once per second (matching the 1 Hz
// setNavigationFrequency() configured in onSwitchLow()), which naturally
// throttles logging to 1 Hz without needing a separate timer here.
void logIfNewFixAvailable() {
  if (myGNSS.getPVT()) {
    writeGnssRow();
  }
}

// =======================================================================
// writeGnssRow() -- formats and writes one CSV row to the open log file
// =======================================================================
// Reads the most recently retrieved PVT data. myGNSS caches the last
// packet internally after getPVT() returns true, so these getter calls
// just return cached values -- they don't trigger fresh I2C reads.
void writeGnssRow() {
  // --- Raw values from the GNSS module ---
  int32_t latRaw = myGNSS.getLatitude();   // degrees, scaled by 1e-7
  int32_t lonRaw = myGNSS.getLongitude();  // degrees, scaled by 1e-7
  byte    fixType = myGNSS.getFixType();               // 0-5, see table below
  byte    carrierSoln = myGNSS.getCarrierSolutionType(); // 0=None, 1=Float, 2=Fixed

  // Convert lat/lon from scaled integers to plain decimal degrees.
  double latDeg = latRaw / 10000000.0;
  double lonDeg = lonRaw / 10000000.0;

  // --- fixType: overall type of position solution ---
  //   0 = No Fix            no position available
  //   1 = Dead Reckoning     estimated from motion sensors only (n/a on
  //                          this module -- no IMU on the GPS-RTK-SMA)
  //   2 = 2D Fix             lat/lon only, not enough sats for altitude
  //   3 = 3D Fix             full lat/lon/altitude solution
  //   4 = GNSS + Dead Reck.  combined fix (n/a here, same reason as above)
  //   5 = Time Only          enough sats for precise time, not position
  const char* fixStr;
  switch (fixType) {
    case 0:  fixStr = "No Fix";         break;
    case 1:  fixStr = "Dead Reckoning"; break;
    case 2:  fixStr = "2D Fix";         break;
    case 3:  fixStr = "3D Fix";         break;
    case 4:  fixStr = "GNSS+DR";        break;
    case 5:  fixStr = "Time Only";      break;
    default: fixStr = "Unknown";        break;
  }

  // --- carrierSoln: RTK correction status, independent of fixType ---
  //   0 = No RTK      standard GNSS accuracy (~1-3m)
  //   1 = RTK Float    correction in use, ambiguities not yet resolved
  //                    (~10cm-1m, still converging)
  //   2 = RTK Fixed    full RTK precision (~1-2cm)
  const char* rtkStr;
  switch (carrierSoln) {
    case 1:  rtkStr = "RTK Float"; break;
    case 2:  rtkStr = "RTK Fixed"; break;
    default: rtkStr = "No RTK";    break;
  }

  // --- Timestamp, formatted as separate date/time strings ---
  char dateStr[11]; // "YYYY-MM-DD" + null terminator
  char timeStr[9];  // "HH:MM:SS"   + null terminator
  snprintf(dateStr, sizeof(dateStr), "%04u-%02u-%02u",
           myGNSS.getYear(), myGNSS.getMonth(), myGNSS.getDay());
  snprintf(timeStr, sizeof(timeStr), "%02u:%02u:%02u",
           myGNSS.getHour(), myGNSS.getMinute(), myGNSS.getSecond());

  // --- Write the CSV row: date,time,lat_deg,lon_deg,fix_type,rtk_status ---
  logFile.print(dateStr);
  logFile.print(",");
  logFile.print(timeStr);
  logFile.print(",");
  logFile.print(latDeg, 7);   // 7 decimal places matches the sensor's
  logFile.print(",");         // native 1e-7 degree resolution
  logFile.print(lonDeg, 7);
  logFile.print(",");
  logFile.print(fixStr);
  logFile.print(",");
  logFile.println(rtkStr);

  // Flush immediately after every row so data already survives an
  // unexpected power loss, rather than sitting in a buffer that could be
  // lost. This trades a little write speed/SD card longevity for safety.
  logFile.flush();

  // Mirror the same row to Serial for live debugging/monitoring.
  Serial.print(dateStr); Serial.print(" ");
  Serial.print(timeStr); Serial.print(" | Lat: ");
  Serial.print(latDeg, 7); Serial.print(" Lon: ");
  Serial.print(lonDeg, 7); Serial.print(" | ");
  Serial.print(fixStr); Serial.print(" | ");
  Serial.println(rtkStr);
}

// =======================================================================
// scanI2CBus() -- diagnostic helper, called only when GNSS detection fails
// =======================================================================
// Manually probes every possible 7-bit I2C address (1-126) and reports
// which ones respond. Used to distinguish "nothing at all is on the bus"
// (points to a wiring/power problem) from "something responded, but not
// at the expected u-blox address" (points to a configuration issue).
void scanI2CBus() {
  Serial.println("Scanning I2C bus for devices...");
  int devicesFound = 0;
  for (byte address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    byte error = Wire.endTransmission();
    if (error == 0) { // 0 means the device ACK'd -- something is there
      Serial.print("  Device found at address 0x");
      if (address < 16) Serial.print("0"); // zero-pad single-digit hex
      Serial.println(address, HEX);
      devicesFound++;
    }
  }
  if (devicesFound == 0) {
    Serial.println("  No I2C devices found at all -- check Qwiic cable, "
                    "connector seating, and that GPIO0 is actually enabling "
                    "the LDO output.");
  } else {
    Serial.println("  (u-blox GNSS modules normally respond at 0x42)");
  }
}
