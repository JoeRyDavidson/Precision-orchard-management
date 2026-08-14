#pragma once
#include <Arduino.h>

/*
 * These type definitions live in their own header (rather than directly in
 * the .ino) to work around a well-known Arduino IDE quirk: the IDE
 * auto-generates forward declarations ("prototypes") for every function in
 * the sketch and inserts them near the very top of the file - BEFORE any
 * struct definitions written later in the .ino. If a function takes a
 * custom struct as a parameter, that auto-inserted prototype ends up
 * referencing a type that hasn't been defined yet, causing errors like
 * "'BufferedSample' does not name a type".
 *
 * Putting the structs in a header and #include-ing it at the top of the
 * .ino avoids this: the header's contents are pulled in by the preprocessor
 * before the auto-generated prototypes are compiled, so the types are
 * already known by the time they're needed.
 */

// One GNSS fix "anchor" sample - the previous or most recent fix read from
// the u-blox module, used to bracket buffered TOF/IMU samples in time.
struct GnssSample {
  unsigned long arrivalMs = 0;  // receiver's millis() when this fix was read
  char    date[11] = "";
  char    time[9]  = "";
  double  latDeg    = 0.0;
  double  lonDeg    = 0.0;
  uint8_t fixType   = 0;   // 3 = valid 3D fix
  uint8_t rtkStatus = 0;   // 0 = none, 1 = float, 2 = fixed
};

// ESP-NOW packet format sent by the sender board. Must exactly match the
// struct defined in the sender sketch (same field order and types) - ESP-NOW
// just copies raw bytes, so there's no schema negotiation between boards.
typedef struct {
  uint32_t timestamp_ms;  // sender's own clock - not used for logging on the
                            // receiver side, since we use our own arrival
                            // time instead for a single common clock
  uint16_t distance_mm;
  uint8_t  range_status;
  float    accel_x;
  float    accel_y;
  float    accel_z;
} SensorPacket;

// One TOF/IMU sample sitting in the receiver's buffer, waiting to be matched
// against a GNSS fix and written to the log file.
struct BufferedSample {
  unsigned long arrivalMs;
  uint16_t distance_mm;
  uint8_t  range_status;
  float    accel_x, accel_y, accel_z;
};
