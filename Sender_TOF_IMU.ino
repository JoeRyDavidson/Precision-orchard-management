/*
 * ============================================================================
 * TOF + IMU SENDER
 * ============================================================================
 * This board reads:
 *   - Distance from an Adafruit VL53L4CD time-of-flight sensor
 *   - Linear acceleration (gravity removed) from an Adafruit BNO055 IMU
 * both over the same I2C bus, and streams each combined reading wirelessly
 * to a second ESP32 (the SparkFun Thing Plus) using ESP-NOW.
 *
 * Sample rate: ~50 Hz, set by the ToF sensor's timing budget (20 ms).
 * This board runs independently of the receiver's state - it always streams,
 * regardless of whether the receiver is currently logging. The receiver
 * decides what to do with each packet.
 * ============================================================================
 */

#include <Wire.h>
#include <vl53l4cd_class.h>       // STM32duino VL53L4CD library
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <esp_now.h>
#include <WiFi.h>

// ----------------------------------------------------------------------------
// Time-of-flight sensor setup
// ----------------------------------------------------------------------------
// XSHUT is the sensor's hardware shutdown/reset pin. The library needs it
// even though we're not using multiple ToF sensors, because it uses XSHUT
// internally as part of the init sequence.
#define XSHUT_PIN 4
VL53L4CD sensor_vl53l4cd_sat(&Wire, XSHUT_PIN);

// ----------------------------------------------------------------------------
// IMU setup
// ----------------------------------------------------------------------------
// Default I2C address 0x28 (BNO055's ADR pin left low/unconnected).
// Sensor ID 55 is arbitrary - only matters if you have multiple Adafruit
// unified-sensor devices and need to tell them apart in code.
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

// ----------------------------------------------------------------------------
// ESP-NOW setup
// ----------------------------------------------------------------------------
// MAC address of the receiving ESP32 (SparkFun Thing Plus).
uint8_t receiverMac[] = {0x28, 0x05, 0xA5, 0x59, 0xB9, 0x78};


// This struct defines the wire format sent over ESP-NOW. It must be
// byte-for-byte IDENTICAL (same field order, same types) in both the
// sender and receiver sketches, since ESP-NOW just copies raw bytes -
// there's no schema negotiation.
typedef struct {
  uint32_t timestamp_ms;   // this board's own millis() - NOT used by the
                            // receiver for logging (see receiver comments),
                            // but kept here for optional debugging/latency
                            // measurement between sender and receiver.
  uint16_t distance_mm;    // ToF distance reading, millimeters
  uint8_t  range_status;   // 0 = valid reading; nonzero = degraded/invalid
  float    accel_x;        // linear acceleration (gravity-compensated), m/s^2
  float    accel_y;
  float    accel_z;
} SensorPacket;

SensorPacket packet;

// Optional callback fired after each ESP-NOW send attempt completes.
// Left mostly empty - uncomment the print if you want to debug dropped sends.
//
// NOTE: newer versions of the ESP32 Arduino core (3.x) changed this
// callback's signature - it now receives a wifi_tx_info_t* (which contains
// the destination MAC among other transmit info) instead of a raw
// const uint8_t* MAC address. Using the old signature causes a compile
// error like: "invalid conversion ... to esp_now_send_cb_t".
void onDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {
  // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Sent OK" : "Send FAIL");
}

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);  // wait for USB serial to be ready (some boards need this)

  Wire.begin();  // default ESP32 I2C pins: SDA=21, SCL=22
                 // if your wiring differs, use Wire.begin(SDA_PIN, SCL_PIN)

  // --- Initialize the ToF sensor ---
  sensor_vl53l4cd_sat.begin();          // low-level driver init
  sensor_vl53l4cd_sat.VL53L4CD_Off();   // ensure a clean state before configuring
  sensor_vl53l4cd_sat.InitSensor();     // full sensor boot/init routine

  // Timing budget = 20 ms, InterMeasurement = 0.
  // With InterMeasurement set to 0, the sensor ranges continuously back-to-back,
  // so the timing budget alone sets the sample period: 20 ms budget -> ~50 Hz.
  // (Valid timing budget range is 10-200 ms; lower = faster but noisier reads,
  // shorter max range.)
  sensor_vl53l4cd_sat.VL53L4CD_SetRangeTiming(20, 0);
  sensor_vl53l4cd_sat.VL53L4CD_StartRanging();

  // --- Initialize the IMU ---
  if (!bno.begin()) {
    Serial.println("No BNO055 detected! Check wiring/address.");
    while (1) delay(10);  // halt - nothing useful to do without the IMU
  }
  delay(1000);  // give the BNO055 a moment to finish its internal boot/self-test
  bno.setExtCrystalUse(true);  // use the sensor's external crystal for more
                                // accurate timing, if your breakout has one
                                // (Adafruit's does) - improves fusion accuracy

  // --- Initialize WiFi in station mode (required by ESP-NOW, even though
  //     we're not connecting to a WiFi network) ---
  WiFi.mode(WIFI_STA);
  Serial.print("Sender MAC: ");
  Serial.println(WiFi.macAddress());  // print our own MAC for reference

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed!");
    while (1) delay(10);
  }
  esp_now_register_send_cb(onDataSent);

  // Register the receiver as an ESP-NOW "peer" - required before esp_now_send()
  // will work targeting that address.
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, receiverMac, 6);
  peerInfo.channel = 0;       // 0 = use the current WiFi channel
  peerInfo.encrypt = false;   // no encryption - simplest for a private link;
                               // add a pre-shared key here if security matters
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add ESP-NOW peer!");
    while (1) delay(10);
  }

  Serial.println("Setup complete. Streaming...");
}

void loop() {
  uint8_t NewDataReady = 0;
  VL53L4CD_Result_t results;
  uint8_t status;

  // Poll until the ToF sensor signals a new measurement is ready.
  // Because the sensor is configured for continuous ranging at ~50 Hz,
  // this loop naturally paces the whole sketch at ~50 Hz too - we don't
  // need a separate delay() or timer.
  do {
    status = sensor_vl53l4cd_sat.VL53L4CD_CheckForDataReady(&NewDataReady);
  } while (!NewDataReady);

  if ((!status) && (NewDataReady != 0)) {
    // Clearing the interrupt is REQUIRED after every read - the sensor
    // won't start its next measurement until this happens.
    sensor_vl53l4cd_sat.VL53L4CD_ClearInterrupt();
    sensor_vl53l4cd_sat.VL53L4CD_GetResult(&results);

    // Read linear acceleration (gravity subtracted out by the BNO055's
    // internal sensor fusion) as a 3-axis vector, in m/s^2.
    sensors_event_t linAccelData;
    bno.getEvent(&linAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);

    // Pack everything into the outgoing struct...
    packet.timestamp_ms = millis();
    packet.distance_mm  = results.distance_mm;
    packet.range_status = results.range_status;
    packet.accel_x       = linAccelData.acceleration.x;
    packet.accel_y       = linAccelData.acceleration.y;
    packet.accel_z       = linAccelData.acceleration.z;

    // ...and send it. This is fire-and-forget - ESP-NOW doesn't guarantee
    // delivery, so occasional dropped packets are possible and are treated
    // as acceptable for this streaming use case (a gap in the log, not a
    // corrupted one).
    esp_now_send(receiverMac, (uint8_t *)&packet, sizeof(packet));
  }
}
