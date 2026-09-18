#include "ECUSim.h"
#include "PIDUpdateSerialControl.h"
#include "CANMesasgeHandle.h"

byte PID_Value_Map[PIDMemSize];
MCP_CAN CAN(10); // CAN CS: pin 10


// Helper: write raw bytes into PID_Value_Map at the correct offset
void setPid(uint8_t pid, uint8_t a, uint8_t b = 0, uint8_t c = 0, uint8_t d = 0) {
  uint8_t  len    = pgm_read_byte(PIDByteLengthMap + pid);
  unsigned offset = pgm_read_word(PIDAddressMap    + pid);
  if (len >= 1) PID_Value_Map[offset]     = a;
  if (len >= 2) PID_Value_Map[offset + 1] = b;
  if (len >= 3) PID_Value_Map[offset + 2] = c;
  if (len >= 4) PID_Value_Map[offset + 3] = d;
}

// Simulated engine state — drifts gradually each call
static uint16_t simRpm     = 1500;
static uint8_t  simSpeed   = 60;
static int8_t   simCoolant = 90;   // °C
static uint8_t  simLoad    = 45;   // %
static uint8_t  simThrottle= 20;   // %
static int8_t   simTiming  = 15;   // °
static unsigned long lastDrift = 0;

void driftEcuState() {
  // Each value drifts by a small random amount each call
  simRpm      = constrain((int)simRpm      + random(-300, 300), 600,  6000);
  simSpeed    = constrain((int)simSpeed    + random(-15,   15),   0,    180);
  simCoolant  = constrain((int)simCoolant  + random(-12,   12),   60,   110);
  simLoad     = constrain((int)simLoad     + random(-13,   13),   5,    95);
  simThrottle = constrain((int)simThrottle + random(-13,   13),   0,    100);
  simTiming   = constrain((int)simTiming   + random(-12,   12),   -10,  40);

  // ISO 15031-5 encoding
  uint16_t rpmRaw = simRpm * 4;
  setPid(0x0C, rpmRaw >> 8, rpmRaw & 0xFF);          // RPM: (A*256+B)/4
  setPid(0x0D, simSpeed);                             // Speed: A km/h
  setPid(0x05, (uint8_t)(simCoolant + 40));           // Coolant: A-40 °C
  setPid(0x04, (uint8_t)(simLoad    * 2.55f));        // Load: A/2.55 %
  setPid(0x11, (uint8_t)(simThrottle* 2.55f));        // Throttle: A/2.55 %
  setPid(0x0E, (uint8_t)((simTiming + 64) * 2));       // Timing: A/2-64 °
  Serial.println("MUDOU -----------------------------------------------");
  Serial.println("  0x0C  Engine RPM        " + String(simRpm) + " RPM");
  Serial.println("  0x0D  Vehicle speed     " + String(simSpeed) + " km/h");
  Serial.println("  0x05  Coolant temp      " + String(simCoolant) + " C");
  Serial.println("  0x04  Engine load       " + String(simLoad) + " %");
  Serial.println("  0x11  Throttle pos      " + String(simThrottle) + " %");
  Serial.println("  0x0E  Timing advance    " + String(simTiming) + " deg");
}

void setup()
{
  Serial.begin(115200);
  Serial.println(F("------------------- Arduino setup start ---------------------------"));
  initializePIDValueMap();
  initializeCAN();
}

void loop()
{
  if (millis() - lastDrift > 2000) {  // only drift every 2 seconds
    driftEcuState();
    lastDrift = millis();
  }
  if (Serial.available() >= SERIAL_MSG_LENGTH)
    parsePIDUPdateMessage();
  if (CAN.checkReceive() == CAN_MSGAVAIL)
    handleCANMessage();
}

