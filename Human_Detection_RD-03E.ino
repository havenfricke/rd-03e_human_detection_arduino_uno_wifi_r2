/*
  RD-03E Human Detection (Robust, compile-safe)
  Board: Arduino Uno WiFi Rev2

  IMPORTANT: THIS C++ FILE is in .ino format to allow Arduino IDE compatibility.
             It uses C++17 standard as set in the Arduino IDE settings.

  Sensor: Serial1 @ 250000 (pins 0/1)
  Debug : Serial  @ 115200 (USB)

  Frame (7 bytes):
    AA AA  STATUS  DIST_L  DIST_H  55 55

  STATUS:
    0x00 = no target
    0x01 = moving target
    0x02 = micro-motion target

  VCC -> 5V
  GND -> GND
  OT1 -> 0 (RX)
  RX  -> 1 (TX)
*/

#include <Arduino.h>

// -----------------------------
// Types (must be above functions)
// -----------------------------
enum PresenceState : uint8_t { ABSENT = 0, PRESENT = 1 };
enum ProxZone : uint8_t { ZONE_UNKNOWN = 0, ZONE_NEAR, ZONE_MID, ZONE_FAR };

// -----------------------------
// Serial settings
// -----------------------------
static const uint32_t SENSOR_BAUD = 250000;
static const uint32_t DEBUG_BAUD  = 115200;

// -----------------------------
// Presence / debounce settings
// -----------------------------
static const uint16_t PRESENT_DEBOUNCE_MS = 250;   // consistent present required
static const uint16_t ABSENT_HOLD_MS      = 1500;  // stay present briefly after last hit

// -----------------------------
// Proximity zones (cm) + hysteresis
// -----------------------------
static const uint16_t NEAR_ENTER_CM = 80;
static const uint16_t NEAR_EXIT_CM  = 95;

static const uint16_t MID_ENTER_CM  = 200;
static const uint16_t MID_EXIT_CM   = 230;

static const uint16_t MIN_VALID_CM  = 1;
static const uint16_t MAX_VALID_CM  = 600;

// -----------------------------
// Still-person logic (micro-motion + stable distance)
// -----------------------------
static const uint16_t STABLE_WINDOW_MS = 1200;
static const uint16_t STABLE_DELTA_CM  = 3;

// -----------------------------
// Optional smoothing (simple IIR)
// 0 = none, 100 = heavy
// -----------------------------
static const uint8_t SMOOTHING_PERCENT = 35;

// -----------------------------
// Frame parsing
// -----------------------------
static const uint8_t FRAME_LEN = 7;
static uint8_t frameBuf[FRAME_LEN];
static uint8_t frameIdx = 0;

// -----------------------------
// State
// -----------------------------
static PresenceState presence = ABSENT;
static ProxZone zone = ZONE_UNKNOWN;

static uint8_t lastStatus = 0xFF;

static uint32_t lastPresentMs = 0;
static uint32_t presentStartMs = 0;
static bool pendingPresent = false;

static bool haveDistance = false;
static uint16_t smoothDistanceCm = 0;

static uint32_t stableStartMs = 0;
static uint16_t stableAnchorCm = 0;
static bool stillPerson = false;

// -----------------------------
// Helpers
// -----------------------------
static inline bool isPresentStatus(uint8_t s) {
  return (s == 0x01 || s == 0x02);
}

static const char* zoneName(ProxZone z) {
  switch (z) {
    case ZONE_NEAR: return "NEAR";
    case ZONE_MID:  return "MID";
    case ZONE_FAR:  return "FAR";
    default:        return "UNKNOWN";
  }
}

static void printMotion(uint8_t st) {
  Serial.print('['); Serial.print(millis()); Serial.print("] MOTION: ");
  if (st == 0x01) Serial.println("MOVING");
  else if (st == 0x02) Serial.println("MICRO-MOTION");
  else Serial.println("NONE");
}

static void printPresence(PresenceState ps) {
  Serial.print('['); Serial.print(millis()); Serial.print("] PRESENCE: ");
  Serial.println(ps == PRESENT ? "PRESENT" : "ABSENT");
}

static void printZone(ProxZone z, uint16_t dcm) {
  Serial.print('['); Serial.print(millis()); Serial.print("] ZONE: ");
  Serial.print(zoneName(z));
  Serial.print(" | dist=");
  Serial.print(dcm);
  Serial.println("cm");
}

static void printStill(bool still, uint16_t dcm) {
  Serial.print('['); Serial.print(millis()); Serial.print("] STILL_PERSON: ");
  Serial.print(still ? "TRUE" : "FALSE");
  Serial.print(" | dist=");
  Serial.print(dcm);
  Serial.println("cm");
}

static void updateSmoothing(uint16_t raw) {
  if (!haveDistance) {
    haveDistance = true;
    smoothDistanceCm = raw;
    return;
  }

  if (SMOOTHING_PERCENT == 0) {
    smoothDistanceCm = raw;
    return;
  }

  uint32_t a = SMOOTHING_PERCENT;     // 0..100
  uint32_t inv = 100 - a;

  uint32_t sm = (uint32_t)smoothDistanceCm * inv + (uint32_t)raw * a;
  smoothDistanceCm = (uint16_t)(sm / 100);
}

static ProxZone computeZoneWithHysteresis(ProxZone current, uint16_t dcm) {
  if (dcm < MIN_VALID_CM || dcm > MAX_VALID_CM) return ZONE_UNKNOWN;

  switch (current) {
    case ZONE_NEAR:
      if (dcm > NEAR_EXIT_CM) return ZONE_MID;
      return ZONE_NEAR;

    case ZONE_MID:
      if (dcm <= NEAR_ENTER_CM) return ZONE_NEAR;
      if (dcm > MID_EXIT_CM)    return ZONE_FAR;
      return ZONE_MID;

    case ZONE_FAR:
      if (dcm <= MID_ENTER_CM) return ZONE_MID;
      return ZONE_FAR;

    case ZONE_UNKNOWN:
    default:
      if (dcm <= NEAR_ENTER_CM) return ZONE_NEAR;
      if (dcm <= MID_ENTER_CM)  return ZONE_MID;
      return ZONE_FAR;
  }
}

static void handlePresenceAndStill(uint8_t status, uint16_t dcm) {
  uint32_t now = millis();

  // motion status change
  if (status != lastStatus) {
    lastStatus = status;
    printMotion(status);
  }

  // presence debounce + hold
  if (isPresentStatus(status)) {
    lastPresentMs = now;

    if (presence == ABSENT) {
      if (!pendingPresent) {
        pendingPresent = true;
        presentStartMs = now;
      } else if (now - presentStartMs >= PRESENT_DEBOUNCE_MS) {
        presence = PRESENT;
        pendingPresent = false;
        printPresence(presence);
      }
    } else {
      pendingPresent = false;
    }
  } else {
    pendingPresent = false;

    if (presence == PRESENT && (now - lastPresentMs >= ABSENT_HOLD_MS)) {
      presence = ABSENT;
      printPresence(presence);

      // reset still state when going absent
      if (stillPerson) {
        stillPerson = false;
        printStill(false, dcm);
      }
      stableStartMs = 0;
    }
  }

  // still-person detection: only when present + micro-motion + valid distance
  bool canStill =
    (presence == PRESENT) &&
    (status == 0x02) &&
    (dcm >= MIN_VALID_CM && dcm <= MAX_VALID_CM);

  if (!canStill) {
    if (stillPerson) {
      stillPerson = false;
      printStill(false, dcm);
    }
    stableStartMs = 0;
    return;
  }

  if (stableStartMs == 0) {
    stableStartMs = now;
    stableAnchorCm = dcm;
    return;
  }

  uint16_t diff = (dcm > stableAnchorCm) ? (dcm - stableAnchorCm) : (stableAnchorCm - dcm);
  if (diff > STABLE_DELTA_CM) {
    stableStartMs = now;
    stableAnchorCm = dcm;
    if (stillPerson) {
      stillPerson = false;
      printStill(false, dcm);
    }
    return;
  }

  if (!stillPerson && (now - stableStartMs >= STABLE_WINDOW_MS)) {
    stillPerson = true;
    printStill(true, dcm);
  }
}

static void handleFrame(uint8_t status, uint16_t rawDistanceCm) {
  updateSmoothing(rawDistanceCm);
  uint16_t dcm = haveDistance ? smoothDistanceCm : rawDistanceCm;

  handlePresenceAndStill(status, dcm);

  ProxZone newZone = (presence == PRESENT)
    ? computeZoneWithHysteresis(zone, dcm)
    : ZONE_UNKNOWN;

  if (newZone != zone) {
    zone = newZone;
    printZone(zone, dcm);
  }
}

static void pollSensor() {
  while (Serial1.available() > 0) {
    uint8_t b = (uint8_t)Serial1.read();

    // Sync on AA AA
    if (frameIdx == 0) {
      if (b != 0xAA) continue;
      frameBuf[frameIdx++] = b;
      continue;
    }
    if (frameIdx == 1) {
      if (b != 0xAA) { frameIdx = 0; continue; }
      frameBuf[frameIdx++] = b;
      continue;
    }

    frameBuf[frameIdx++] = b;

    if (frameIdx >= FRAME_LEN) {
      frameIdx = 0;

      // Validate tail
      if (frameBuf[5] != 0x55 || frameBuf[6] != 0x55) return;

      uint8_t status = frameBuf[2];
      uint16_t dist  = (uint16_t)frameBuf[3] | ((uint16_t)frameBuf[4] << 8);

      handleFrame(status, dist);
    }
  }
}

void setup() {
  Serial.begin(DEBUG_BAUD);
  while (!Serial) { }
  delay(200);

  Serial.println("RD-03E Human Detection (Robust) - READY");
  Serial1.begin(SENSOR_BAUD);
  delay(200);
}

void loop() {
  pollSensor();
}
