/*
 * ============================================================================
 * GNSS + TOF/IMU RECEIVER & LOGGER (state machine)
 * ============================================================================
 * This board:
 *   1. Sits idle until a switch on GPIO25 goes LOW.
 *   2. Powers the Qwiic bus, starts a u-blox GNSS module, and blinks the
 *      status LED while waiting for a valid 3D fix.
 *   3. Once a fix is obtained, opens a CSV file and starts logging. TOF/IMU
 *      samples arrive continuously over ESP-NOW (~50 Hz) and are buffered;
 *      once each new 1 Hz GNSS fix arrives, the buffered samples are matched
 *      to whichever GNSS fix (the previous one or the new one) is closer in
 *      time, and written out as merged rows.
 *   4. When the switch goes HIGH again, flushes any remaining buffered
 *      samples, closes the file, and returns to idle.
 *
 * Design notes:
 *   - All rows use ONE shared clock: this board's own millis(), stamped at
 *     the moment each TOF/IMU packet arrives. This avoids any mismatch
 *     between the sender's clock and this board's clock.
 *   - Because GNSS only updates once a second but TOF/IMU arrives ~50x a
 *     second, choosing a TRUE nearest-neighbor GNSS match requires waiting
 *     for the *next* GNSS fix before we know whether a buffered sample was
 *     closer to the fix before it or after it. This adds ~0.5-1s of
 *     latency between a sample arriving and it being written to disk -
 *     acceptable here in exchange for better time alignment in the log.
 * ============================================================================
 */

#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <esp_now.h>
#include <WiFi.h>
#include <SparkFun_u-blox_GNSS_v3.h>
#include "DataTypes.h"  // GnssSample, SensorPacket, BufferedSample - see that
                          // file for why these live in a separate header

// ----------------------------------------------------------------------------
// Pin definitions (confirmed against SparkFun Thing Plus ESP32 WROOM docs)
// ----------------------------------------------------------------------------
#define SWITCH_PIN     25  // user switch, wired with internal pullup: LOW = active
#define STAT_LED_PIN   13  // onboard status LED
#define QWIIC_PWR_PIN  0   // controls power to the Qwiic bus (and thus the GNSS module)
#define SD_CS_PIN      5   // SD card chip-select
// SD card SPI bus uses the ESP32's default VSPI pins: SCK=18, MISO=19, MOSI=23

// ----------------------------------------------------------------------------
// State machine
// ----------------------------------------------------------------------------
enum State { IDLE, ACQUIRING_GPS, RECORDING };
volatile State currentState = IDLE;
// "volatile" because this is read/written from both loop() and (indirectly,
// via the check in onDataRecv) the ESP-NOW receive callback, which runs in
// a separate task context.

SFE_UBLOX_GNSS myGNSS;
File logFile;
char currentLogFileName[32] = "";  // set fresh each time recording starts

// Finds the next unused "/log_NNNN.csv" filename on the SD card, so each
// recording session gets its own file instead of appending to the same one.
// Checking the card itself (rather than counting in RAM) means this keeps
// working correctly across power cycles/resets too.
bool getNextLogFileName(char *outName, size_t outSize) {
  for (int i = 1; i <= 9999; i++) {
    snprintf(outName, outSize, "/log_%04d.csv", i);
    if (!SD.exists(outName)) return true;
  }
  return false;  // all 9999 slots taken - extremely unlikely in practice
}

// ----------------------------------------------------------------------------
// Concurrency
// ----------------------------------------------------------------------------
// The ESP-NOW receive callback (onDataRecv) runs in the WiFi driver's own
// FreeRTOS task - NOT the same task as loop(). Both loop() and the callback
// touch the shared sample buffer and the SD file, so a mutex protects both
// to prevent corruption from simultaneous access.
SemaphoreHandle_t dataMutex;

// ----------------------------------------------------------------------------
// GNSS sample tracking
// ----------------------------------------------------------------------------
// We keep two GNSS "anchor" samples at a time: the previous one and the
// current one. Every buffered TOF/IMU sample gets compared against both to
// find whichever is closer in time. (GnssSample type is defined in DataTypes.h)

GnssSample prevGnss;  // the GNSS fix from ~1 second ago
GnssSample currGnss;  // the most recently read GNSS fix

// ----------------------------------------------------------------------------
// ESP-NOW packet format
// ----------------------------------------------------------------------------
// (SensorPacket type - must exactly match the sender's struct - is defined
// in DataTypes.h)

// ----------------------------------------------------------------------------
// Sample buffer
// ----------------------------------------------------------------------------
// Holds TOF/IMU samples that have arrived but not yet been matched to a
// GNSS fix and written to disk. At ~50 Hz, a 1-second GNSS interval means
// up to ~50 samples could accumulate; 100 gives comfortable headroom.
// (BufferedSample type is defined in DataTypes.h)

#define BUFFER_SIZE 100
BufferedSample sampleBuffer[BUFFER_SIZE];
int bufferCount = 0;
unsigned long droppedSamples = 0;  // counts samples lost if the buffer ever fills up

unsigned long lastGnssPollMs = 0;
unsigned long lastBlinkMs = 0;
bool ledState = false;

// ----------------------------------------------------------------------------
// ESP-NOW receive callback
// ----------------------------------------------------------------------------
// Fired automatically whenever a packet arrives from the sender, regardless
// of what loop() is doing. Runs in a different FreeRTOS task than loop().
void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
  // Sanity check: ignore anything that isn't exactly one SensorPacket -
  // guards against stray/corrupted packets.
  if (len != sizeof(SensorPacket)) return;

  // Only buffer data while actively recording - if we're idle or still
  // waiting for a GPS fix, the sender may still be streaming (it doesn't
  // know or care about our state), but we simply discard those packets.
  if (currentState != RECORDING) return;

  SensorPacket packet;
  memcpy(&packet, incomingData, sizeof(packet));

  // Timestamp on arrival using THIS board's clock - this is what makes
  // every row in the final CSV share one common time base.
  unsigned long arrivalMs = millis();

  if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
    if (bufferCount < BUFFER_SIZE) {
      BufferedSample &s = sampleBuffer[bufferCount++];
      s.arrivalMs    = arrivalMs;
      s.distance_mm  = packet.distance_mm;
      s.range_status = packet.range_status;
      s.accel_x      = packet.accel_x;
      s.accel_y      = packet.accel_y;
      s.accel_z      = packet.accel_z;
    } else {
      // Buffer overrun - this shouldn't normally happen at ~50 Hz into a
      // 100-slot buffer with 1-second flushes, but if it does, we drop the
      // sample rather than block the callback or overwrite existing data.
      droppedSamples++;
    }
    xSemaphoreGive(dataMutex);
  }
}

// ----------------------------------------------------------------------------
// Writes one merged row (TOF/IMU sample + matched GNSS fix) to the log file
// ----------------------------------------------------------------------------
void writeRow(const BufferedSample &s, const GnssSample &g) {
  if (logFile) {
    logFile.printf("%lu,%s,%s,%.7f,%.7f,%u,%u,%u,%u,%.4f,%.4f,%.4f\n",
                    s.arrivalMs,
                    g.date, g.time,
                    g.latDeg, g.lonDeg,
                    g.fixType, g.rtkStatus,
                    s.distance_mm, s.range_status,
                    s.accel_x, s.accel_y, s.accel_z);
  }
}

// ----------------------------------------------------------------------------
// Called right after a NEW GNSS fix has been captured into currGnss.
// At this point every buffered sample has two anchors to compare against:
// prevGnss (the fix from ~1s ago) and currGnss (the one that just arrived).
// Whichever is closer in time to a given sample "wins" and is used for
// that sample's logged position/fix data.
// ----------------------------------------------------------------------------
void flushBufferWithNearestMatch() {
  if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
    for (int i = 0; i < bufferCount; i++) {
      BufferedSample &s = sampleBuffer[i];
      long distToPrev = labs((long)(s.arrivalMs - prevGnss.arrivalMs));
      long distToCurr = labs((long)(s.arrivalMs - currGnss.arrivalMs));
      writeRow(s, (distToCurr < distToPrev) ? currGnss : prevGnss);
    }
    bufferCount = 0;  // buffer is now fully drained
    xSemaphoreGive(dataMutex);
  }
}

// ----------------------------------------------------------------------------
// Used only when recording STOPS (switch goes HIGH). There's no "next" GNSS
// fix coming to bracket the remaining buffered samples against, so they're
// all matched to the last known fix instead - slightly less precise for
// just this final ~0-1s tail, but ensures no buffered data is lost.
// ----------------------------------------------------------------------------
void flushBufferWithLastKnown() {
  if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
    for (int i = 0; i < bufferCount; i++) {
      writeRow(sampleBuffer[i], currGnss);
    }
    bufferCount = 0;
    xSemaphoreGive(dataMutex);
  }
}

// ----------------------------------------------------------------------------
// Reads the GNSS module's current fix and stores it into the given sample
// ----------------------------------------------------------------------------
void captureGnssSample(GnssSample &dest) {
  dest.arrivalMs  = millis();
  dest.fixType    = myGNSS.getFixType();               // 3 = valid 3D fix
  dest.rtkStatus  = myGNSS.getCarrierSolutionType();    // 0=none, 1=float, 2=fixed
  dest.latDeg     = myGNSS.getLatitude() / 1e7;         // u-blox reports 1e-7 degree units
  dest.lonDeg     = myGNSS.getLongitude() / 1e7;

  snprintf(dest.date, sizeof(dest.date), "%04u-%02u-%02u",
            myGNSS.getYear(), myGNSS.getMonth(), myGNSS.getDay());
  snprintf(dest.time, sizeof(dest.time), "%02u:%02u:%02u",
            myGNSS.getHour(), myGNSS.getMinute(), myGNSS.getSecond());
}

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  // Switch uses the internal pullup, so idle (open) reads HIGH and pressed
  // (grounded) reads LOW - matches the "switch LOW = active" requirement.
  pinMode(SWITCH_PIN, INPUT_PULLUP);
  pinMode(STAT_LED_PIN, OUTPUT);
  pinMode(QWIIC_PWR_PIN, OUTPUT);
  digitalWrite(STAT_LED_PIN, LOW);
  digitalWrite(QWIIC_PWR_PIN, LOW);  // Qwiic bus (and GNSS module) off at boot

  dataMutex = xSemaphoreCreateMutex();

  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("SD card init failed!");
    while (1) delay(10);
  }

  // WiFi station mode is required for ESP-NOW even though we're not
  // joining a network.
  WiFi.mode(WIFI_STA);
  Serial.print("Receiver MAC: ");
  Serial.println(WiFi.macAddress());  // should read d4:e9:f4:fa:c4:2c

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed!");
    while (1) delay(10);
  }
  esp_now_register_recv_cb(onDataRecv);

  Serial.println("Setup complete. Idle.");
}

void loop() {
  bool switchLow = (digitalRead(SWITCH_PIN) == LOW);

  switch (currentState) {

    // ------------------------------------------------------------------
    case IDLE:
      if (switchLow) {
        Serial.println("Switch LOW -> powering up Qwiic bus, acquiring GPS fix...");
        digitalWrite(QWIIC_PWR_PIN, HIGH);
        delay(500);  // give the GNSS module time to boot before talking to it

        Wire.begin();  // I2C: SDA=21, SCL=22 (ESP32 defaults)
        if (!myGNSS.begin(Wire)) {
          Serial.println("u-blox GNSS not detected!");
          // Intentionally don't halt here - stay in ACQUIRING_GPS and let
          // the next loop iterations keep trying, in case it's a transient
          // power-up timing issue.
        }
        myGNSS.setI2COutput(COM_TYPE_UBX);   // use UBX binary protocol only
        myGNSS.setNavigationFrequency(1);    // 1 Hz fix updates

        currentState = ACQUIRING_GPS;
        lastBlinkMs = millis();
      }
      break;

    // ------------------------------------------------------------------
    case ACQUIRING_GPS: {
      if (!switchLow) {
        // User flipped the switch back before a fix was ever acquired.
        digitalWrite(QWIIC_PWR_PIN, LOW);
        digitalWrite(STAT_LED_PIN, LOW);
        currentState = IDLE;
        Serial.println("Switch HIGH -> returning to idle.");
        break;
      }

      // Blink status LED at 1 Hz (toggle every 500ms = full cycle every 1s)
      // while waiting for a fix.
      if (millis() - lastBlinkMs >= 500) {
        lastBlinkMs = millis();
        ledState = !ledState;
        digitalWrite(STAT_LED_PIN, ledState ? HIGH : LOW);
      }

      if (myGNSS.getFixType() == 3) {  // 3 = valid 3D fix
        if (!getNextLogFileName(currentLogFileName, sizeof(currentLogFileName))) {
          Serial.println("No available log filename slots on SD card!");
          break;  // stays in ACQUIRING_GPS; won't attempt to open a file
        }

        logFile = SD.open(currentLogFileName, FILE_WRITE);
        if (logFile) {
          logFile.println("timestamp_ms,date,time,lat_deg,lon_deg,fix_type,rtk_status,distance_mm,range_status,accel_x,accel_y,accel_z");

          // Seed both GNSS anchors with this first fix, and start with an
          // empty sample buffer.
          captureGnssSample(currGnss);
          prevGnss = currGnss;
          bufferCount = 0;

          digitalWrite(STAT_LED_PIN, HIGH);  // solid on = recording
          currentState = RECORDING;
          lastGnssPollMs = millis();
          Serial.print("3D fix acquired -> recording started: ");
          Serial.println(currentLogFileName);
        } else {
          Serial.println("Failed to open log file!");
          // Stays in ACQUIRING_GPS and will retry opening the file next
          // time a fix check passes, since currentState wasn't advanced.
        }
      }
      break;
    }

    // ------------------------------------------------------------------
    case RECORDING:
      if (!switchLow) {
        // Recording is ending - write out whatever's left in the buffer
        // (matched to the last known GNSS fix, since there's no future
        // fix left to bracket against), then close up.
        flushBufferWithLastKnown();

        if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
          if (logFile) logFile.close();
          xSemaphoreGive(dataMutex);
        }
        digitalWrite(STAT_LED_PIN, LOW);
        digitalWrite(QWIIC_PWR_PIN, LOW);
        currentState = IDLE;
        Serial.println("Switch HIGH -> recording stopped, returning to idle.");
        break;
      }

      // Poll GNSS once per second. Each new fix becomes the new "current"
      // anchor (the old "current" slides down to become "previous"), and
      // triggers matching/writing everything currently buffered.
      if (millis() - lastGnssPollMs >= 1000) {
        lastGnssPollMs = millis();
        prevGnss = currGnss;
        captureGnssSample(currGnss);
        flushBufferWithNearestMatch();

        // Flush the SD file to disk periodically (not on every row) so
        // data survives a power loss without the overhead of flushing
        // ~50 times a second.
        if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
          if (logFile) logFile.flush();
          xSemaphoreGive(dataMutex);
        }
      }
      break;
  }
}
