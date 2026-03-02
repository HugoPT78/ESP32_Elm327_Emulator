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

// ─── 30-Second Driving Profile ───────────────────────────────────────────────
//
//  Time │ Phase           │ RPM  │ Speed │ Throttle │ Load │ Coolant │ MAF
//  ─────┼─────────────────┼──────┼───────┼──────────┼──────┼─────────┼──────
//   0s  │ Idle            │  800 │   0   │   15%    │ 20%  │  85°C   │  3.0
//   5s  │ Begin accel     │ 2200 │  30   │   45%    │ 55%  │  88°C   │ 14.0
//  12s  │ Mid accel       │ 3500 │  70   │   65%    │ 75%  │  90°C   │ 22.0
//  20s  │ Cruise          │ 2400 │  90   │   25%    │ 40%  │  90°C   │ 12.0
//  26s  │ Deceleration    │ 1200 │  40   │    5%    │ 15%  │  89°C   │  5.0
//  30s  │ Idle (loops)    │  800 │   0   │   15%    │ 20%  │  85°C   │  3.0

#define PROFILE_LEN    6
#define CYCLE_SECONDS  30.0f

struct ProfilePoint {
  float t;          // seconds within cycle
  float rpm;        // engine RPM
  float speed;      // vehicle speed km/h
  float throttle;   // throttle position 0–100 %
  float load;       // engine load 0–100 %
  float coolant;    // coolant temperature °C
  float maf;        // MAF rate g/s
  float map_kpa;    // MAP absolute pressure kPa
  float voltage;    // battery/control module voltage V
  float intake;     // intake air temperature °C
  float timing;     // timing advance degrees before TDC
  float fuelTrim;   // short-term fuel trim %
};

const ProfilePoint profile[PROFILE_LEN] = {
  //  t      rpm    spd  thr  load  cool   maf   map   volt   iat  timing  ftrim
  {  0.0f,   800,    0,  15,   20,   85,   3.0,   35,  12.6,   30,   8.0,   0.0 },
  {  5.0f,  2200,   30,  45,   55,   88,  14.0,   65,  14.2,   32,  12.0,   1.6 },
  { 12.0f,  3500,   70,  65,   75,   90,  22.0,   80,  14.4,   35,  15.0,  -0.8 },
  { 20.0f,  2400,   90,  25,   40,   90,  12.0,   55,  14.3,   36,  11.0,   0.4 },
  { 26.0f,  1200,   40,   5,   15,   89,   5.0,   40,  13.8,   34,   6.0,  -1.6 },
  { 30.0f,   800,    0,  15,   20,   85,   3.0,   35,  12.6,   30,   8.0,   0.0 },
};

// ─── Globals ─────────────────────────────────────────────────────────────────
BluetoothSerial SerialBT;

String   inputBuffer = "";
bool     echoOn      = true;
bool     linefeedOn  = true;
bool     headersOn   = false;
bool     spacesOn    = true;
uint8_t  protocol    = 6;    // 6 = ISO 15765-4 CAN 11-bit 500kbps
uint32_t obdHeader   = 0x7DF;

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
  p.voltage  = lerpf(profile[lo].voltage,  profile[hi].voltage,  alpha);
  p.intake   = lerpf(profile[lo].intake,   profile[hi].intake,   alpha);
  p.timing   = lerpf(profile[lo].timing,   profile[hi].timing,   alpha);
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

  if (service == 0x01) {
    switch (pid) {

      // Supported PIDs 0x01–0x20  (bitmask)
      case 0x00:
        buf[0]=0xBE; buf[1]=0x3F; buf[2]=0xA8; buf[3]=0x13;
        return 4;

      // Monitor status (no DTCs, all monitors ready)
      case 0x01:
        buf[0]=0x00; buf[1]=0x07; buf[2]=0xFF; buf[3]=0x00;
        return 4;

      // Fuel system status – closed-loop
      case 0x03:
        buf[0]=0x02; buf[1]=0x00;
        return 2;

      // Engine load  A*100/255
      case 0x04:
        buf[0] = (uint8_t)(p.load * 255.0f / 100.0f);
        return 1;

      // Coolant temp  A-40
      case 0x05:
        buf[0] = (uint8_t)(p.coolant + 40.0f);
        return 1;

      // Short-term fuel trim  (A-128)*100/128
      case 0x06:
        buf[0] = (uint8_t)((p.fuelTrim * 128.0f / 100.0f) + 128.0f);
        return 1;

      // Long-term fuel trim  0%
      case 0x07:
        buf[0] = 128;
        return 1;

      // MAP absolute pressure  A kPa
      case 0x0B:
        buf[0] = (uint8_t)(p.map_kpa);
        return 1;

      // Engine RPM  (256A+B)/4
      case 0x0C: {
        uint16_t raw = (uint16_t)(p.rpm * 4.0f);
        buf[0] = (raw >> 8) & 0xFF;
        buf[1] = raw & 0xFF;
        return 2;
      }

      // Vehicle speed  A km/h
      case 0x0D:
        buf[0] = (uint8_t)(p.speed);
        return 1;

      // Timing advance  A/2-64
      case 0x0E:
        buf[0] = (uint8_t)((p.timing + 64.0f) * 2.0f);
        return 1;

      // Intake air temp  A-40
      case 0x0F:
        buf[0] = (uint8_t)(p.intake + 40.0f);
        return 1;

      // MAF  (256A+B)/100
      case 0x10: {
        uint16_t raw = (uint16_t)(p.maf * 100.0f);
        buf[0] = (raw >> 8) & 0xFF;
        buf[1] = raw & 0xFF;
        return 2;
      }

      // Throttle position  A*100/255
      case 0x11:
        buf[0] = (uint8_t)(p.throttle * 255.0f / 100.0f);
        return 1;

      // OBD standard
      case 0x1C:
        buf[0] = 0x06;
        return 1;

      // Runtime since start (seconds)
      case 0x1F: {
        uint16_t s = (uint16_t)(millis() / 1000);
        buf[0] = (s >> 8) & 0xFF;
        buf[1] = s & 0xFF;
        return 2;
      }

      // Supported PIDs 0x21–0x40
      case 0x20:
        buf[0]=0x80; buf[1]=0x00; buf[2]=0x00; buf[3]=0x01;
        return 4;

      // Distance with MIL on
      case 0x21:
        buf[0]=0x00; buf[1]=0x00;
        return 2;

      // Fuel tank level  A*100/255 %
      case 0x2F:
        buf[0] = (uint8_t)(0.65f * 255.0f);   // 65%
        return 1;

      // Barometric pressure
      case 0x33:
        buf[0] = 101;
        return 1;

      // Control module voltage  (256A+B)/1000
      case 0x42: {
        uint16_t raw = (uint16_t)(p.voltage * 1000.0f);
        buf[0] = (raw >> 8) & 0xFF;
        buf[1] = raw & 0xFF;
        return 2;
      }

      // Ambient air temperature
      case 0x46:
        buf[0] = (uint8_t)(28.0f + 40.0f);
        return 1;

      // Fuel type – gasoline
      case 0x51:
        buf[0] = 0x01;
        return 1;

      // Oil temperature  A-40
      case 0x5C:
        buf[0] = (uint8_t)((p.coolant - 5.0f) + 40.0f);
        return 1;

      default:
        return -1;
    }

  } else if (service == 0x09) {
    switch (pid) {

      // VIN message count + first 5 chars
      case 0x02:
        buf[0]=0x01;
        buf[1]='1'; buf[2]='H'; buf[3]='G'; buf[4]='B'; buf[5]='H';
        return 6;

      // Calibration ID
      case 0x04:
        buf[0]=0x01;
        buf[1]='E'; buf[2]='S'; buf[3]='P'; buf[4]='3'; buf[5]='2';
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
  Serial.println("[ESP32 ELM327 Emulator] Starting...");
  Serial.println(USE_REAL_CAN ? "[MODE] Real CAN bus" : "[MODE] 30s simulated driving profile");

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
