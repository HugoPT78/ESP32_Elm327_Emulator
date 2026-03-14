/**
 * ESP32 ELM327 v2.1 Bluetooth Emulator
 * — Simulated OBD2 Data Mode —
 *
 * Emulates an ELM327 OBD2 adapter over Bluetooth Classic (SPP).
 * Returns realistic simulated sensor values that cycle through a
 * 30-second driving profile: idle → acceleration → cruise → decel.
 *
 * Hardware:
 *   - ESP32 (any variant with Bluetooth Classic)
 *   - No CAN transceiver needed in simulation mode
 *   - Optional: set USE_REAL_CAN true and wire SN65HVD230 on GPIO5/GPIO4
 *     to use live vehicle data instead of simulation.
 *
 * Compatible with: Torque, OBD Auto Doctor, Car Scanner, DashCommand
 */

#include <Arduino.h>
#include <BluetoothSerial.h>
#include "esp_mac.h"

// ─── Mode Switch ─────────────────────────────────────────────────────────────
// Set to true to use real CAN bus hardware instead of simulation
#define USE_REAL_CAN  false

#if USE_REAL_CAN
  #include "driver/twai.h"
  #define CAN_TX_PIN      GPIO_NUM_5
  #define CAN_RX_PIN      GPIO_NUM_4
  #define OBD_TIMEOUT_MS  200
#endif

// ─── ELM327 Settings ─────────────────────────────────────────────────────────
#define BT_DEVICE_NAME  "OBD2_ESP32"
#define ELM_VERSION     "ELM327 v2.1"

// ─── 10-Minute Driving Profile ───────────────────────────────────────────────
//
// Simulates a realistic urban+road trip cycle that loops every 10 minutes.
// Gear is calculated live from speed/RPM ratio (no profile field needed).
// Boost (MAP above atmospheric) rises with throttle/load on turbo assumption.
//
//  t(s) │ Phase                  │ RPM  │ Spd │ Thr │ Load │ Cool │ MAF  │ MAP  │ IAT │ Fuel% │ Volt
//  ─────┼────────────────────────┼──────┼─────┼─────┼──────┼──────┼──────┼──────┼─────┼───────┼──────
//    0  │ Cold idle              │  950 │   0 │  12 │  18  │  65  │  2.5 │  98  │  28 │  75   │ 12.4
//   30  │ Warm-up idle           │  850 │   0 │  12 │  20  │  75  │  3.0 │ 100  │  30 │  75   │ 13.8
//   60  │ Pull away (1st)        │ 2800 │  15 │  55 │  65  │  78  │ 18.0 │ 145  │  32 │  74   │ 14.2
//   90  │ 2nd gear accel         │ 3200 │  35 │  60 │  72  │  82  │ 24.0 │ 155  │  34 │  73   │ 14.3
//  120  │ 3rd gear cruise        │ 2400 │  55 │  30 │  42  │  87  │ 14.0 │ 115  │  35 │  72   │ 14.2
//  150  │ Traffic stop (idle)    │  820 │   0 │  10 │  18  │  89  │  2.8 │ 100  │  36 │  72   │ 13.9
//  180  │ Re-accelerate 1→2→3    │ 3500 │  50 │  70 │  80  │  90  │ 28.0 │ 165  │  37 │  71   │ 14.4
//  240  │ 4th gear road cruise   │ 2200 │  80 │  28 │  38  │  90  │ 13.0 │ 112  │  38 │  70   │ 14.2
//  300  │ 5th gear motorway      │ 2600 │ 110 │  32 │  44  │  91  │ 16.0 │ 118  │  39 │  69   │ 14.3
//  360  │ Hard overtake (boost)  │ 4200 │ 130 │  85 │  92  │  92  │ 38.0 │ 210  │  41 │  67   │ 14.5
//  390  │ Settle back to cruise  │ 2600 │ 110 │  30 │  40  │  91  │ 14.0 │ 115  │  40 │  67   │ 14.2
//  420  │ Exit ramp decel        │ 1400 │  60 │   5 │  14  │  90  │  4.5 │ 102  │  39 │  67   │ 13.8
//  450  │ Urban crawl            │ 1600 │  30 │  25 │  32  │  89  │  9.0 │ 108  │  38 │  66   │ 14.0
//  480  │ Stop (engine on)       │  820 │   0 │  10 │  18  │  89  │  2.8 │ 100  │  37 │  66   │ 13.8
//  510  │ Final run home         │ 2400 │  50 │  40 │  52  │  89  │ 16.0 │ 120  │  37 │  65   │ 14.1
//  560  │ Slow + park            │ 1000 │  10 │  15 │  22  │  88  │  4.0 │ 100  │  36 │  65   │ 13.6
//  600  │ Idle (loop back)       │  850 │   0 │  12 │  18  │  87  │  2.8 │ 100  │  35 │  65   │ 13.5

#define PROFILE_LEN    17
#define CYCLE_SECONDS  600.0f   // 10 minutes

struct ProfilePoint {
  float t;          // seconds within cycle
  float rpm;        // engine RPM
  float speed;      // vehicle speed km/h
  float throttle;   // throttle position 0–100 %
  float load;       // engine load 0–100 %
  float coolant;    // coolant temperature °C
  float maf;        // MAF air flow g/s
  float map_kpa;    // MAP absolute pressure kPa (101 = atmospheric, >101 = boost)
  float intake;     // intake air temperature °C
  float fuel_pct;   // fuel tank level %
  float voltage;    // battery/module voltage V
  float fuelTrim;   // short-term fuel trim %
};

const ProfilePoint profile[PROFILE_LEN] = {
  //   t      rpm   spd  thr  load  cool   maf   map   iat  fuel   volt  ftrim
  {   0.0f,   950,    0,  12,   18,   65,   2.5,   98,   28,  75.0, 12.4,  0.0 },
  {  30.0f,   850,    0,  12,   20,   75,   3.0,  100,   30,  75.0, 13.8,  0.0 },
  {  60.0f,  2800,   15,  55,   65,   78,  18.0,  145,   32,  74.8, 14.2,  1.6 },
  {  90.0f,  3200,   35,  60,   72,   82,  24.0,  155,   34,  74.5, 14.3,  0.8 },
  { 120.0f,  2400,   55,  30,   42,   87,  14.0,  115,   35,  74.2, 14.2,  0.4 },
  { 150.0f,   820,    0,  10,   18,   89,   2.8,  100,   36,  74.0, 13.9,  0.0 },
  { 180.0f,  3500,   50,  70,   80,   90,  28.0,  165,   37,  73.5, 14.4,  1.2 },
  { 240.0f,  2200,   80,  28,   38,   90,  13.0,  112,   38,  73.0, 14.2,  0.4 },
  { 300.0f,  2600,  110,  32,   44,   91,  16.0,  118,   39,  72.0, 14.3,  0.4 },
  { 360.0f,  4200,  130,  85,   92,   92,  38.0,  210,   41,  70.5, 14.5,  2.0 },
  { 390.0f,  2600,  110,  30,   40,   91,  14.0,  115,   40,  70.0, 14.2,  0.4 },
  { 420.0f,  1400,   60,   5,   14,   90,   4.5,  102,   39,  69.8, 13.8, -1.6 },
  { 450.0f,  1600,   30,  25,   32,   89,   9.0,  108,   38,  69.5, 14.0,  0.0 },
  { 480.0f,   820,    0,  10,   18,   89,   2.8,  100,   37,  69.3, 13.8,  0.0 },
  { 510.0f,  2400,   50,  40,   52,   89,  16.0,  120,   37,  68.8, 14.1,  0.8 },
  { 560.0f,  1000,   10,  15,   22,   88,   4.0,  100,   36,  68.5, 13.6,  0.0 },
  { 600.0f,   850,    0,  12,   18,   87,   2.8,  100,   35,  68.0, 13.5,  0.0 },
};

// ─── Gear Calculation ────────────────────────────────────────────────────────
// Calculates current gear from speed/RPM ratio (final drive ~3.9, tyre ~0.35m).
// Returns 0 if stationary, 1–6 for gears.
uint8_t calcGear(float rpm, float speed_kmh) {
  if (speed_kmh < 2.0f || rpm < 500.0f) return 0;
  // Approximate speed-per-1000rpm thresholds for each gear
  // Gear: 1=~8, 2=~16, 3=~26, 4=~36, 5=~48, 6=~60 km/h per 1000rpm
  float ratio = speed_kmh / (rpm / 1000.0f);
  if      (ratio <  12.0f) return 1;
  else if (ratio <  21.0f) return 2;
  else if (ratio <  31.0f) return 3;
  else if (ratio <  42.0f) return 4;
  else if (ratio <  54.0f) return 5;
  else                     return 6;
}

// ─── Globals ─────────────────────────────────────────────────────────────────
BluetoothSerial SerialBT;

String   inputBuffer = "";
bool     echoOn      = true;
bool     linefeedOn  = true;
bool     headersOn   = false;
bool     spacesOn    = true;
uint8_t  protocol    = 6;    // 6 = ISO 15765-4 CAN 11-bit 500kbps
uint32_t obdHeader   = 0x7DF;

// ─── Freeze Simulation ────────────────────────────────────────────────────────
// Controlled from Arduino Serial Monitor. Send command + Enter:
//
//   F10  → OFF              — adapter responds normally
//   F11  → NO DATA          — responds with "NO DATA", BT stays connected (default)
//   F12  → SILENT DROP      — no response at all, app times out + disconnects
//   F13  → UNABLE TO CONNECT — sends "UNABLE TO CONNECT", BT stays connected
//   F14  → RANDOM TIMEOUT   — randomly silently drops 3 consecutive requests
//                             for a randomly chosen PID, then resumes normally.
//                             Simulates intermittent signal loss per sensor.
//
// Default freeze mode on startup is F11 (NO DATA).

enum FreezeMode {
  FREEZE_OFF         = 0,   // F10 – normal operation
  FREEZE_NODATA      = 1,   // F11 – reply "NO DATA"            (BT stays alive)
  FREEZE_SILENT      = 2,   // F12 – silent drop                (app disconnects)
  FREEZE_UNABLE      = 3,   // F13 – reply "UNABLE TO CONNECT"  (BT stays alive)
  FREEZE_RANDTIMEOUT = 4    // F14 – random 3x silent drop on one PID
};

FreezeMode freezeMode  = FREEZE_NODATA;  // default: F11 on startup
String     serialBuffer = "";            // Serial Monitor input buffer

// F14 state — tracks which PID is being timed out and how many drops remain
uint8_t  f14TargetPid    = 0xFF;   // PID currently being dropped (0xFF = none chosen yet)
uint8_t  f14DropsLeft    = 0;      // remaining silent drops for this PID burst
uint32_t f14NextTriggerMs = 0;     // earliest time the next burst can start

// ─── Serial Monitor Command Handler ──────────────────────────────────────────
void handleSerialCommand(String cmd) {
  cmd.trim();
  cmd.toUpperCase();

  if (cmd == "F10") {
    freezeMode = FREEZE_OFF;
    Serial.println("[FREEZE] F10 — OFF: adapter responding normally");
  }
  else if (cmd == "F11") {
    freezeMode = FREEZE_NODATA;
    Serial.println("[FREEZE] F11 — NO DATA: BT alive, sending NO DATA");
  }
  else if (cmd == "F12") {
    freezeMode = FREEZE_SILENT;
    Serial.println("[FREEZE] F12 — SILENT: requests dropped, app will disconnect");
  }
  else if (cmd == "F13") {
    freezeMode = FREEZE_UNABLE;
    Serial.println("[FREEZE] F13 — UNABLE TO CONNECT: BT alive, sending UNABLE TO CONNECT");
  }
  else if (cmd == "F14") {
    freezeMode       = FREEZE_RANDTIMEOUT;
    f14TargetPid     = 0xFF;   // no burst active yet — will pick randomly on next PID hit
    f14DropsLeft     = 0;
    f14NextTriggerMs = 0;      // allow first burst immediately
    Serial.println("[FREEZE] F14 — RANDOM TIMEOUT: will silently drop 3x same PID randomly");
  }
  else {
    Serial.println("[CMD] Unknown: " + cmd);
    Serial.println("[CMD] F10=OFF  F11=NO DATA  F12=SILENT  F13=UNABLE  F14=RANDOM TIMEOUT");
  }
}

// ─── Freeze Check (no auto-expiry — stays until F10) ─────────────────────────
void checkFreeze() {
  // intentionally empty — mode only changes via Serial commands
}

// ─── Interpolation ───────────────────────────────────────────────────────────

float lerpf(float a, float b, float t) { return a + (b - a) * t; }

ProfilePoint getCurrentProfile() {
  float t = fmod((float)(millis() / 1000.0), CYCLE_SECONDS);
  int lo = 0, hi = 1;
  for (int i = 0; i < PROFILE_LEN - 1; i++) {
    if (t >= profile[i].t && t < profile[i + 1].t) { lo = i; hi = i + 1; break; }
  }
  float span  = profile[hi].t - profile[lo].t;
  float alpha = (span > 0) ? (t - profile[lo].t) / span : 0.0f;
  ProfilePoint p;
  p.t        = t;
  p.rpm      = lerpf(profile[lo].rpm,      profile[hi].rpm,      alpha);
  p.speed    = lerpf(profile[lo].speed,    profile[hi].speed,    alpha);
  p.throttle = lerpf(profile[lo].throttle, profile[hi].throttle, alpha);
  p.load     = lerpf(profile[lo].load,     profile[hi].load,     alpha);
  p.coolant  = lerpf(profile[lo].coolant,  profile[hi].coolant,  alpha);
  p.maf      = lerpf(profile[lo].maf,      profile[hi].maf,      alpha);
  p.map_kpa  = lerpf(profile[lo].map_kpa,  profile[hi].map_kpa,  alpha);
  p.intake   = lerpf(profile[lo].intake,   profile[hi].intake,   alpha);
  p.fuel_pct = lerpf(profile[lo].fuel_pct, profile[hi].fuel_pct, alpha);
  p.voltage  = lerpf(profile[lo].voltage,  profile[hi].voltage,  alpha);
  p.fuelTrim = lerpf(profile[lo].fuelTrim, profile[hi].fuelTrim, alpha);
  return p;
}

// ─── Utility ─────────────────────────────────────────────────────────────────

void btPrint(const String &s) {
  SerialBT.print(s);
  SerialBT.print(linefeedOn ? "\r\n" : "\r");
}

void btPrompt() { SerialBT.print("\r\n>"); }

String hexByte(uint8_t b) {
  char buf[3]; sprintf(buf, "%02X", b); return String(buf);
}

String sp() { return spacesOn ? " " : ""; }

String makeResponse(uint8_t svc, uint8_t pid, uint8_t *data, int len) {
  String r = "";
  if (headersOn) r += "7E8 " + hexByte((uint8_t)(2 + len)) + " ";
  r += hexByte(svc + 0x40) + sp() + hexByte(pid);
  for (int i = 0; i < len; i++) r += sp() + hexByte(data[i]);
  return r;
}

// ─── Simulated OBD2 PID Table ────────────────────────────────────────────────

int simulatePID(uint8_t service, uint8_t pid, uint8_t *buf) {
  ProfilePoint p = getCurrentProfile();
  uint8_t gear   = calcGear(p.rpm, p.speed);

  // Boost pressure in kPa above atmospheric (101 kPa).
  // Positive = turbo boost, negative = vacuum (throttled engine).
  float boost_kpa = p.map_kpa - 101.0f;

  // Timing advance: derived from load+rpm (higher load/rpm = more advance)
  float timing = 8.0f + (p.load / 100.0f) * 12.0f + (p.rpm / 6000.0f) * 8.0f;

  if (service == 0x01) {
    switch (pid) {

      // ── Supported PIDs bitmask 0x01–0x20 ─────────────────────────────────
      // Bits for: 04,05,06,07,0B,0C,0D,0E,0F,10,11,1C,1F
      case 0x00:
        buf[0]=0xBE; buf[1]=0x3F; buf[2]=0xB8; buf[3]=0x13;
        return 4;

      // ── Monitor status – no DTCs, all monitors complete ───────────────────
      case 0x01:
        buf[0]=0x00; buf[1]=0x07; buf[2]=0xFF; buf[3]=0x00;
        return 4;

      // ── Fuel system status – closed-loop ─────────────────────────────────
      case 0x03:
        buf[0]=0x02; buf[1]=0x00;
        return 2;

      // ── Engine load  A*100/255 ────────────────────────────────────────────
      case 0x04:
        buf[0] = (uint8_t)(p.load * 255.0f / 100.0f);
        return 1;

      // ── Coolant temperature  A-40 °C ──────────────────────────────────────
      case 0x05:
        buf[0] = (uint8_t)(p.coolant + 40.0f);
        return 1;

      // ── Short-term fuel trim  (A-128)*100/128 ─────────────────────────────
      case 0x06:
        buf[0] = (uint8_t)((p.fuelTrim * 128.0f / 100.0f) + 128.0f);
        return 1;

      // ── Long-term fuel trim  (fixed 0%) ───────────────────────────────────
      case 0x07:
        buf[0] = 128;
        return 1;

      // ── MAP absolute pressure  A kPa ──────────────────────────────────────
      // This is the primary boost/vacuum sensor.
      // 101 kPa = atmospheric, >101 = boost, <101 = vacuum
      case 0x0B:
        buf[0] = (uint8_t)constrain(p.map_kpa, 0, 255);
        return 1;

      // ── Engine RPM  (256A+B)/4 ────────────────────────────────────────────
      case 0x0C: {
        uint16_t raw = (uint16_t)(p.rpm * 4.0f);
        buf[0] = (raw >> 8) & 0xFF;
        buf[1] = raw & 0xFF;
        return 2;
      }

      // ── Vehicle speed  A km/h ─────────────────────────────────────────────
      case 0x0D:
        buf[0] = (uint8_t)constrain(p.speed, 0, 255);
        return 1;

      // ── Timing advance  A/2-64 degrees ───────────────────────────────────
      case 0x0E:
        buf[0] = (uint8_t)((timing + 64.0f) * 2.0f);
        return 1;

      // ── Intake air temperature  A-40 °C ───────────────────────────────────
      case 0x0F:
        buf[0] = (uint8_t)(p.intake + 40.0f);
        return 1;

      // ── MAF air flow  (256A+B)/100 g/s ───────────────────────────────────
      case 0x10: {
        uint16_t raw = (uint16_t)(p.maf * 100.0f);
        buf[0] = (raw >> 8) & 0xFF;
        buf[1] = raw & 0xFF;
        return 2;
      }

      // ── Throttle position  A*100/255 % ───────────────────────────────────
      case 0x11:
        buf[0] = (uint8_t)(p.throttle * 255.0f / 100.0f);
        return 1;

      // ── OBD standard  (EOBD) ─────────────────────────────────────────────
      case 0x1C:
        buf[0] = 0x06;
        return 1;

      // ── Runtime since engine start (seconds) ──────────────────────────────
      case 0x1F: {
        uint16_t s = (uint16_t)(millis() / 1000);
        buf[0] = (s >> 8) & 0xFF;
        buf[1] = s & 0xFF;
        return 2;
      }

      // ── Supported PIDs 0x21–0x40 ─────────────────────────────────────────
      case 0x20:
        buf[0]=0xA0; buf[1]=0x00; buf[2]=0x20; buf[3]=0x01;
        return 4;

      // ── Distance with MIL on (0 km) ───────────────────────────────────────
      case 0x21:
        buf[0]=0x00; buf[1]=0x00;
        return 2;

      // ── Fuel tank level  A*100/255 % ─────────────────────────────────────
      case 0x2F:
        buf[0] = (uint8_t)(p.fuel_pct * 255.0f / 100.0f);
        return 1;

      // ── Barometric pressure  A kPa ────────────────────────────────────────
      case 0x33:
        buf[0] = 101;
        return 1;

      // ── Supported PIDs 0x41–0x60 ─────────────────────────────────────────
      case 0x40:
        buf[0]=0x44; buf[1]=0x00; buf[2]=0x00; buf[3]=0x01;
        return 4;

      // ── Control module voltage  (256A+B)/1000 V ───────────────────────────
      case 0x42: {
        uint16_t raw = (uint16_t)(p.voltage * 1000.0f);
        buf[0] = (raw >> 8) & 0xFF;
        buf[1] = raw & 0xFF;
        return 2;
      }

      // ── Absolute throttle position B (redundant sensor) ───────────────────
      case 0x47:
        buf[0] = (uint8_t)(p.throttle * 255.0f / 100.0f);
        return 1;

      // ── Ambient air temperature  A-40 °C ──────────────────────────────────
      case 0x46:
        buf[0] = (uint8_t)(22.0f + 40.0f);   // fixed 22°C ambient
        return 1;

      // ── Fuel type – gasoline ──────────────────────────────────────────────
      case 0x51:
        buf[0] = 0x01;
        return 1;

      // ── Engine oil temperature  A-40 °C ───────────────────────────────────
      case 0x5C:
        buf[0] = (uint8_t)((p.coolant - 8.0f) + 40.0f);
        return 1;

      // ── Relative throttle position  A*100/255 % ───────────────────────────
      case 0x45:
        buf[0] = (uint8_t)(p.throttle * 255.0f / 100.0f);
        return 1;

      // ── Boost pressure (Turbo / Supercharger) ─────────────────────────────
      // PID 0x70 – Boost pressure control. A = boost in kPa above atmos * 2
      // Used by Car Scanner, Torque Pro for boost gauge.
      // Formula: (A * 3) - 101  kPa  → we encode boost_kpa as (boost_kpa+101)/3
      case 0x70: {
        // Encode as signed kPa relative: (A-128)*0.5 kPa per bit
        int16_t raw = (int16_t)((boost_kpa + 128.0f) * 2.0f);
        if (raw < 0)   raw = 0;
        if (raw > 255) raw = 255;
        buf[0] = (uint8_t)raw;
        return 1;
      }

      // ── Transmission gear (PID 0xA4) ──────────────────────────────────────
      // PID 0xA4 – Transmission actual gear.
      // Formula: A = gear number (0=neutral/park, 1–8 = gears)
      case 0xA4:
        buf[0] = gear;
        buf[1] = 0x00;
        return 2;

      default:
        return -1;
    }

  } else if (service == 0x09) {
    switch (pid) {

      // ── VIN – BMW E36 Compact 316i ────────────────────────────────────────
      // Full VIN: WBABG910XTJD12345  (17 chars)
      //
      // Breakdown:
      //   WBA        – BMW AG, Germany
      //   BG91       – E36/5 Compact, 316i (M43B16 engine)
      //   0          – Restraint: seat belts only
      //   X          – Check digit
      //   T          – Model year 1996
      //   J          – Munich plant
      //   D12345     – Sequential serial number
      //
      // OBD2 Service 09 PID 02 encodes VIN as:
      //   byte 0 = number of data items (0x01)
      //   bytes 1–5 = first 5 chars of VIN (remaining chars handled by ISO-TP
      //               multi-frame in a real ECU; single-frame gives prefix)
      case 0x02: {
        const char vin[] = "WBABG910XTJD12345";
        buf[0] = 0x01;                    // 1 data item
        for (int i = 0; i < 5; i++) buf[i + 1] = (uint8_t)vin[i];
        return 6;
      }

      // ── ECU / Calibration ID ──────────────────────────────────────────────
      case 0x04:
        buf[0]=0x01;
        buf[1]='M'; buf[2]='4'; buf[3]='3'; buf[4]='B'; buf[5]='1';
        return 6;

      default:
        return -1;
    }
  }

  return -1;
}

// ─── Real CAN (compiled only when USE_REAL_CAN = true) ───────────────────────

#if USE_REAL_CAN
bool canInit() {
  twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
  twai_timing_config_t  t = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t  f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  if (twai_driver_install(&g, &t, &f) != ESP_OK) return false;
  return twai_start() == ESP_OK;
}

int realObdRequest(uint8_t service, uint8_t pid, uint8_t *rd) {
  twai_message_t tx = {};
  tx.identifier = obdHeader;
  tx.data_length_code = 8;
  tx.data[0]=0x02; tx.data[1]=service; tx.data[2]=pid;
  for (int i=3;i<8;i++) tx.data[i]=0xCC;
  twai_message_t dummy;
  while (twai_receive(&dummy, 0) == ESP_OK) {}
  if (twai_transmit(&tx, pdMS_TO_TICKS(50)) != ESP_OK) return -1;
  uint32_t start = millis();
  while (millis()-start < OBD_TIMEOUT_MS) {
    twai_message_t rx;
    if (twai_receive(&rx, pdMS_TO_TICKS(10)) == ESP_OK) {
      if (rx.identifier>=0x7E8 && rx.identifier<=0x7EF) {
        uint8_t pci=rx.data[0];
        if ((pci&0xF0)==0x00) {
          uint8_t len=pci&0x0F;
          if (rx.data[1]==(service+0x40) && rx.data[2]==pid) {
            int db=len-2; if(db<0)db=0; if(db>5)db=5;
            for (int i=0;i<db;i++) rd[i]=rx.data[3+i];
            return db;
          }
        }
      }
    }
  }
  return -1;
}
#endif

// ─── AT Command Handler ───────────────────────────────────────────────────────

void handleAT(String cmd) {
  cmd.trim();
  String u = cmd; u.toUpperCase();

  // ── ATZ – Full reset: restore defaults + print version banner ────────────────
  if (u=="ATZ"||u=="AT Z") {
    echoOn=true; linefeedOn=true; headersOn=false; spacesOn=true; protocol=6;
    SerialBT.print("\r\n");
    delay(500);
    SerialBT.print("\r\n");
    SerialBT.print(ELM_VERSION);
    SerialBT.print("\r\n");
    return;
  }

  // ── ATI – Version string ──────────────────────────────────────────────────
  if (u=="ATI"||u=="AT I") { btPrint(ELM_VERSION); return; }

  // ── ATRV – Battery voltage (live from profile) ────────────────────────────
  if (u.startsWith("ATRV")||u.startsWith("AT RV")) {
    char vb[8]; dtostrf(getCurrentProfile().voltage, 4, 1, vb);
    btPrint(String(vb)+"V"); return;
  }

  // ── ATE0 / ATE1 – Echo off/on ─────────────────────────────────────────────
  // ATE0: the command itself is echoed one last time, THEN echo is disabled
  if (u=="ATE0"||u=="AT E0") {
    if (echoOn) { SerialBT.print(cmd); SerialBT.print("\r\n"); }
    echoOn = false;
    SerialBT.print("OK\r\n");
    return;
  }
  if (u=="ATE1"||u=="AT E1") { echoOn=true; btPrint("OK"); return; }

  if (u=="ATL0"||u=="AT L0")  { linefeedOn=false; btPrint("OK"); return; }
  if (u=="ATL1"||u=="AT L1")  { linefeedOn=true;  btPrint("OK"); return; }

  // ── ATH0 / ATH1 – Headers off/on ──────────────────────────────────────────
  // Real ELM327 sends "OK" + blank line when toggling headers
  if (u=="ATH0"||u=="AT H0") {
    headersOn = false;
    SerialBT.print("OK\r\n\r\n");
    return;
  }
  if (u=="ATH1"||u=="AT H1") {
    headersOn = true;
    SerialBT.print("OK\r\n\r\n");
    return;
  }

  if (u=="ATS0"||u=="AT S0")  { spacesOn=false; btPrint("OK"); return; }
  if (u=="ATS1"||u=="AT S1")  { spacesOn=true;  btPrint("OK"); return; }

  // ── ATSP – Set Protocol ───────────────────────────────────────────────────
  // Each protocol number gets the exact ELM327 2.1 acknowledgement string.
  // ATSP0 = auto-detect, ATSP6 = ISO 15765-4 CAN 11-bit 500kbps (most common),
  // ATSP8 = ISO 15765-4 CAN 11-bit 250kbps, ATSP9 = CAN 29-bit 250kbps, etc.
  if (u.startsWith("ATSP")||u.startsWith("AT SP")) {
    String pnum = u;
    pnum.replace("AT SP",""); pnum.replace("ATSP",""); pnum.trim();
    if (pnum.length() > 0) protocol = (uint8_t)strtol(pnum.c_str(), nullptr, 16);

    switch (protocol) {
      case 0x0:
        // Auto – ELM327 tries all protocols and reports which was found
        SerialBT.print("OK\r\n");
        delay(100);
        SerialBT.print("SEARCHING...\r\n");
        delay(600);
        SerialBT.print("ISO 15765-4 (CAN 11/500)\r\n");
        break;
      case 0x1:
        SerialBT.print("OK\r\n");
        delay(50);
        SerialBT.print("SAE J1850 PWM\r\n");
        break;
      case 0x2:
        SerialBT.print("OK\r\n");
        delay(50);
        SerialBT.print("SAE J1850 VPW\r\n");
        break;
      case 0x3:
        SerialBT.print("OK\r\n");
        delay(50);
        SerialBT.print("ISO 9141-2\r\n");
        break;
      case 0x4:
        SerialBT.print("OK\r\n");
        delay(50);
        SerialBT.print("ISO 14230-4 KWP (5 baud)\r\n");
        break;
      case 0x5:
        SerialBT.print("OK\r\n");
        delay(50);
        SerialBT.print("ISO 14230-4 KWP (fast)\r\n");
        break;
      case 0x6:
        // Most common – ISO 15765-4 CAN 11-bit 500kbps
        SerialBT.print("OK\r\n");
        delay(100);
        SerialBT.print("SEARCHING...\r\n");
        delay(500);
        SerialBT.print("ISO 15765-4 (CAN 11/500)\r\n");
        break;
      case 0x7:
        SerialBT.print("OK\r\n");
        delay(100);
        SerialBT.print("SEARCHING...\r\n");
        delay(500);
        SerialBT.print("ISO 15765-4 (CAN 29/500)\r\n");
        break;
      case 0x8:
        SerialBT.print("OK\r\n");
        delay(100);
        SerialBT.print("SEARCHING...\r\n");
        delay(500);
        SerialBT.print("ISO 15765-4 (CAN 11/250)\r\n");
        break;
      case 0x9:
        SerialBT.print("OK\r\n");
        delay(100);
        SerialBT.print("SEARCHING...\r\n");
        delay(500);
        SerialBT.print("ISO 15765-4 (CAN 29/250)\r\n");
        break;
      case 0xA:
        SerialBT.print("OK\r\n");
        delay(50);
        SerialBT.print("SAE J1939 (CAN 29/250)\r\n");
        break;
      default:
        SerialBT.print("OK\r\n");
        break;
    }
    return;
  }

  if (u.startsWith("ATTP")||u.startsWith("AT TP")) { btPrint("OK"); return; }
  if (u=="ATDP"||u=="AT DP")  { btPrint("ISO 15765-4 (CAN 11/500)"); return; }
  if (u=="ATDPN"||u=="AT DPN") {
    char pb[4]; sprintf(pb, "%X", protocol); btPrint(String(pb)); return;
  }
  if (u=="ATPC" ||u=="AT PC")  { btPrint("OK"); return; }
  if (u.startsWith("ATST")||u.startsWith("AT ST"))   { btPrint("OK"); return; }
  if (u.startsWith("ATAT")||u.startsWith("AT AT"))   { btPrint("OK"); return; }
  if (u.startsWith("ATCAF")||u.startsWith("AT CAF")) { btPrint("OK"); return; }
  if (u=="ATD"||u=="AT D") {
    echoOn=true; linefeedOn=true; headersOn=false; spacesOn=true;
    btPrint("OK"); return;
  }
  if (u.startsWith("ATMA")||u.startsWith("AT MA"))   { btPrint("STOPPED"); return; }
  if (u.startsWith("ATCM")||u.startsWith("AT CM")||
      u.startsWith("ATCF")||u.startsWith("AT CF"))   { btPrint("OK"); return; }
  if (u=="AT@1") { btPrint("OBDII to RS232 Interpreter"); return; }
  if (u=="AT@2") { btPrint("?"); return; }
  if (u=="ATIGN"||u=="AT IGN") { btPrint("ON"); return; }

  btPrint("OK");
}

// ─── OBD2 Command Handler ─────────────────────────────────────────────────────

void handleOBD(String cmd) {
  cmd.trim();
  if (cmd.length() < 4) { btPrint("NO DATA"); return; }

  // ── Freeze mode check ────────────────────────────────────────────────────
  if (freezeMode != FREEZE_OFF) {
    switch (freezeMode) {

      case FREEZE_NODATA:
        Serial.println("[FREEZE-F11] NO DATA → " + cmd);
        btPrint("NO DATA");
        return;

      case FREEZE_SILENT:
        Serial.println("[FREEZE-F12] SILENT DROP → " + cmd);
        return;

      case FREEZE_UNABLE:
        Serial.println("[FREEZE-F13] UNABLE TO CONNECT → " + cmd);
        btPrint("UNABLE TO CONNECT");
        return;

      case FREEZE_RANDTIMEOUT: {
        // Parse the PID from this request
        uint8_t reqPid = (uint8_t)strtol(cmd.substring(2, 4).c_str(), nullptr, 16);

        // ── Start a new burst if none is active and enough time has passed ──
        if (f14DropsLeft == 0 && millis() >= f14NextTriggerMs) {
          // Pick this PID as the target randomly (~30% chance per PID request)
          if (random(0, 100) < 30) {
            f14TargetPid = reqPid;
            f14DropsLeft = 3;
            Serial.println("[FREEZE-F14] New burst: PID 0x"
                           + String(f14TargetPid, HEX)
                           + " will be silently dropped 3x");
          }
        }

        // ── Drop silently if this request matches the active burst PID ──────
        if (f14DropsLeft > 0 && reqPid == f14TargetPid) {
          f14DropsLeft--;
          Serial.println("[FREEZE-F14] Silent drop PID 0x"
                         + String(reqPid, HEX)
                         + " — drops left: " + String(f14DropsLeft));
          if (f14DropsLeft == 0) {
            // Burst complete — wait 3–8 seconds before allowing a new one
            f14NextTriggerMs = millis() + (uint32_t)random(3000, 8000);
            f14TargetPid     = 0xFF;
            Serial.println("[FREEZE-F14] Burst complete. Next burst in ~"
                           + String((f14NextTriggerMs - millis()) / 1000) + "s");
          }
          return;  // silent drop — no response sent
        }

        // All other PIDs respond normally during F14 mode
        break;
      }

      default:
        break;
    }
  }

  uint8_t service = (uint8_t)strtol(cmd.substring(0, 2).c_str(), nullptr, 16);
  uint8_t pid     = (uint8_t)strtol(cmd.substring(2, 4).c_str(), nullptr, 16);

  uint8_t data[8] = {};
  int len;

#if USE_REAL_CAN
  len = realObdRequest(service, pid, data);
#else
  len = simulatePID(service, pid, data);
#endif

  if (len < 0) { btPrint("NO DATA"); return; }
  btPrint(makeResponse(service, pid, data, len));
}

// ─── Command Dispatcher ───────────────────────────────────────────────────────

void processCommand(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) { btPrompt(); return; }
  if (echoOn) { SerialBT.print(cmd); SerialBT.print("\r\n"); }
  String u = cmd; u.toUpperCase();
  if (u.startsWith("AT")) handleAT(cmd);
  else                    handleOBD(cmd);
  btPrompt();
}

// ─── Setup & Loop ────────────────────────────────────────────────────────────

void setup() {
  Serial.begin(115200);
  randomSeed(esp_random());   // seed RNG from hardware entropy (for F14)
  Serial.println("[ESP32 ELM327 Emulator] Starting...");
  Serial.println(USE_REAL_CAN ? "[MODE] Real CAN bus" : "[MODE] 10-min simulated driving profile");

  // Print Bluetooth MAC address (available before BT stack starts)
  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_BT);
  char macStr[18];
  sprintf(macStr, "%02X:%02X:%02X:%02X:%02X:%02X",
          mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  Serial.println("[BT] MAC Address: " + String(macStr));

  SerialBT.setPin("1234", 4);
  SerialBT.begin(BT_DEVICE_NAME);
  Serial.println("[BT] Device: " + String(BT_DEVICE_NAME) + "  PIN: 1234");
  Serial.println("[INFO] Serial: F10=OFF  F11=NO DATA(default)  F12=SILENT  F13=UNABLE  F14=RANDOM TIMEOUT");

#if USE_REAL_CAN
  if (canInit()) Serial.println("[CAN] TWAI started @ 500kbps");
  else           Serial.println("[CAN] WARNING: TWAI init failed.");
#endif
}

bool clientConnected = false;

void loop() {
  if (SerialBT.connected() && !clientConnected) {
    clientConnected = true;
    delay(200);
    SerialBT.print("\r\n" ELM_VERSION "\r\n\r\n>");
    Serial.println("[BT] Client connected");
  }
  if (!SerialBT.connected() && clientConnected) {
    clientConnected = false;
    inputBuffer = "";
    Serial.println("[BT] Client disconnected");
  }

  // Update freeze expiry
  checkFreeze();

  // ── Read Serial Monitor commands (F11 / F10) ─────────────────────────────
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r' || c == '\n') {
      if (serialBuffer.length() > 0) {
        handleSerialCommand(serialBuffer);
        serialBuffer = "";
      }
    } else {
      serialBuffer += c;
    }
  }

  // ── Read Bluetooth OBD commands ───────────────────────────────────────────
  while (SerialBT.available()) {
    char c = SerialBT.read();
    if (c == '\r' || c == '\n') {
      if (inputBuffer.length() > 0) {
        processCommand(inputBuffer);
        inputBuffer = "";
      }
    } else if (c == 0x08 || c == 0x7F) {
      if (inputBuffer.length() > 0) inputBuffer.remove(inputBuffer.length() - 1);
    } else {
      inputBuffer += c;
    }
  }
}
