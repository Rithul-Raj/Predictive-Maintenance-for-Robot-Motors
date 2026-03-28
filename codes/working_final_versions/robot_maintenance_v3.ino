/*
 * ================================================================
 *  PREDICTIVE MAINTENANCE — WEB CONTROLLED ROBOT
 *  Sensors : ACS712 (current), MPU6050 (vibration), DHT11 (temp)
 *  Control : Web dashboard via WiFi hotspot
 *  Faults  : Frozen Adaptive Baseline + Z-Score + State Machine
 *
 *  Baseline Logic:
 *   - Collected during warmup (60 samples)
 *   - FROZEN after warmup — does not drift with sensor values
 *   - User can manually re-warmup via dashboard button
 *   - Auto re-warmup every 5 min ONLY if state is NORMAL
 *   - Warmup blocked if fault/warning is active
 * ================================================================
 */

#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <math.h>

// ─────────────────────────────────────────────
//  WiFi
// ─────────────────────────────────────────────
const char* ssid     = "ESP32_ROBOT";
const char* password = "12345678";
WiFiServer server(80);

// ─────────────────────────────────────────────
//  Pins
// ─────────────────────────────────────────────
#define ACS712_PIN 34
#define DHT_PIN    4
#define DHT_TYPE   DHT11
#define IN1 26
#define IN2 27
#define ENA 25
#define IN3 14
#define IN4 12
#define ENB 33

// ─────────────────────────────────────────────
//  Objects
// ─────────────────────────────────────────────
Adafruit_MPU6050 mpu;
DHT dht(DHT_PIN, DHT_TYPE);

// ─────────────────────────────────────────────
//  Baseline Config
// ─────────────────────────────────────────────
#define WARMUP_SAMPLES        60       // samples to collect for baseline
#define AUTO_WARMUP_INTERVAL  300000UL // 5 minutes in ms

// ─────────────────────────────────────────────
//  State Machine Config
// ─────────────────────────────────────────────
#define WARNING_PERSIST  5
#define FAULT_PERSIST    3
#define RECOVERY_PERSIST 10

// ─────────────────────────────────────────────
//  Z-Score Thresholds
// ─────────────────────────────────────────────
#define Z_RAW_WARN    2.5f
#define Z_VIB_WARN    2.8f
#define Z_TEMP_WARN   2.0f

// Weighted score thresholds
#define SCORE_WARN     3.5f
#define SCORE_FAULT    6.0f
#define SCORE_CRITICAL 8.0f

// Hard absolute safety limits
#define HARD_VIB_LIMIT   18.0f
#define HARD_TEMP_LIMIT  70.0f
#define HARD_DELTA_LIMIT 600

// ─────────────────────────────────────────────
//  Fault Flags
// ─────────────────────────────────────────────
#define FAULT_NONE          0x00
#define FAULT_OVERCURRENT   0x01
#define FAULT_VIBRATION     0x02
#define FAULT_OVERTEMP      0x04
#define FAULT_SUDDEN_SPIKE  0x08
#define FAULT_MULTI_SENSOR  0x10

// ─────────────────────────────────────────────
//  FROZEN Baseline (set once during warmup)
// ─────────────────────────────────────────────
struct FrozenBaseline {
  float mean  = 0;
  float stdev = 1;
  bool  valid = false;   // false = not yet collected

  // Called during warmup to accumulate
  float  _sum   = 0;
  float  _sumSq = 0;
  int    _count = 0;

  void reset() {
    _sum = 0; _sumSq = 0; _count = 0;
    valid = false;
  }

  // Feed one sample during warmup phase
  bool feed(float val) {
    _sum   += val;
    _sumSq += val * val;
    _count++;
    if (_count >= WARMUP_SAMPLES) {
      mean        = _sum / _count;
      float var   = (_sumSq / _count) - (mean * mean);
      stdev       = (var > 0) ? sqrt(var) : 0.5f;
      valid       = true;
      return true;  // warmup complete
    }
    return false;
  }

  float zScore(float val) {
    if (!valid) return 0;
    return fabs(val - mean) / stdev;
  }

  int progress() { return min(100, (_count * 100) / WARMUP_SAMPLES); }
};

FrozenBaseline baseRaw, baseVib, baseTemp;

// ─────────────────────────────────────────────
//  State Machine
// ─────────────────────────────────────────────
enum MotorState {
  STATE_WARMUP,
  STATE_NORMAL,
  STATE_WARNING,
  STATE_FAULT,
  STATE_CRITICAL,
  STATE_STOPPED    // user turned off motors; waiting for restart
};
MotorState motorState = STATE_WARMUP;

int warnCount = 0, faultCount = 0, recoveryCount = 0;
int prevRaw   = 0;

// ─────────────────────────────────────────────
//  Warmup Tracking
// ─────────────────────────────────────────────
bool          inWarmup         = true;   // currently collecting baseline
bool          warmupBlocked    = false;  // true if fault present when user tries
unsigned long lastAutoWarmup   = 0;      // timestamp of last auto-warmup start
bool          autoWarmupPending = false; // auto-warmup queued

// ─────────────────────────────────────────────
//  Robot Control
// ─────────────────────────────────────────────
String moveCmd      = "stop";  // last direction from user
String lastMoveCmd  = "stop";  // remembered for restart after stop

// ─────────────────────────────────────────────
//  Live Sensor Cache (for dashboard)
// ─────────────────────────────────────────────
float   lastRaw = 0, lastVib = 0, lastTemp = 0;
float   lastScore = 0;
uint8_t lastFaultFlags = FAULT_NONE;

// ─────────────────────────────────────────────
//  Helpers
// ─────────────────────────────────────────────
String stateToString(MotorState s) {
  switch (s) {
    case STATE_WARMUP:   return "WARMUP";
    case STATE_NORMAL:   return "NORMAL";
    case STATE_WARNING:  return "WARNING";
    case STATE_FAULT:    return "FAULT";
    case STATE_CRITICAL: return "CRITICAL";
    case STATE_STOPPED:  return "STOPPED";
    default:             return "UNKNOWN";
  }
}

String faultToString(uint8_t f) {
  if (f == FAULT_NONE) return "NONE";
  String s = "";
  if (f & FAULT_OVERCURRENT)  s += "OVERCURRENT ";
  if (f & FAULT_VIBRATION)    s += "VIBRATION ";
  if (f & FAULT_OVERTEMP)     s += "OVERTEMP ";
  if (f & FAULT_SUDDEN_SPIKE) s += "SUDDEN SPIKE ";
  if (f & FAULT_MULTI_SENSOR) s += "MULTI-SENSOR ";
  s.trim();
  return s;
}

// ─────────────────────────────────────────────
//  Motor Control
// ─────────────────────────────────────────────
void motorForward(int spd = 180) {
  digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
  analogWrite(ENA, spd);   analogWrite(ENB, spd);
}
void motorBackward(int spd = 180) {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  analogWrite(ENA, spd);   analogWrite(ENB, spd);
}
void motorLeft(int spd = 150) {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
  analogWrite(ENA, spd);   analogWrite(ENB, spd);
}
void motorRight(int spd = 150) {
  digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  analogWrite(ENA, spd);   analogWrite(ENB, spd);
}
void motorStop() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}
void motorEmergencyStop() {
  analogWrite(ENA, 0);    analogWrite(ENB, 0);
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
}

void driveByCmd(String cmd, int spd = 180) {
  if      (cmd == "forward")  motorForward(spd);
  else if (cmd == "backward") motorBackward(spd);
  else if (cmd == "left")     motorLeft(spd);
  else if (cmd == "right")    motorRight(spd);
  else                        motorStop();
}

// ─────────────────────────────────────────────
//  Apply Movement based on state
// ─────────────────────────────────────────────
void applyMovement() {
  // Hard fault — always emergency stop, no override
  if (motorState == STATE_CRITICAL) {
    motorEmergencyStop();
    return;
  }

  // User stopped the robot — wait for restart command
  if (motorState == STATE_STOPPED) {
    motorStop();
    return;
  }

  // Still warming up or in warmup — hold stop
  if (motorState == STATE_WARMUP) {
    motorStop();
    return;
  }

  // Soft fault warning — slow to 50%
  if (motorState == STATE_WARNING || motorState == STATE_FAULT) {
    driveByCmd(moveCmd, 90);
    return;
  }

  // Normal — full speed
  driveByCmd(moveCmd, 180);
}

// ─────────────────────────────────────────────
//  Fault Analysis (uses FROZEN baseline)
// ─────────────────────────────────────────────
float analyzeFault(float raw, float vib, float temp, int delta, uint8_t &flags) {
  flags = FAULT_NONE;

  float zR = baseRaw.zScore(raw);
  float zV = baseVib.zScore(vib);
  float zT = baseTemp.zScore(temp);

  uint8_t hits = 0;
  if (zR >= Z_RAW_WARN)                     { flags |= FAULT_OVERCURRENT;  hits++; }
  if (zV >= Z_VIB_WARN  || vib  > HARD_VIB_LIMIT)  { flags |= FAULT_VIBRATION;    hits++; }
  if (zT >= Z_TEMP_WARN || temp > HARD_TEMP_LIMIT)  { flags |= FAULT_OVERTEMP;     hits++; }
  if (abs(delta) > HARD_DELTA_LIMIT)               { flags |= FAULT_SUDDEN_SPIKE; hits++; }
  if (hits >= 2)                                     flags |= FAULT_MULTI_SENSOR;

  float score = (zR * 0.40f) + (zV * 0.40f) + (zT * 0.20f);
  if (flags & FAULT_MULTI_SENSOR)  score *= 1.3f;
  if (flags & FAULT_SUDDEN_SPIKE)  score += 2.0f;
  return score;
}

// ─────────────────────────────────────────────
//  State Machine Update
// ─────────────────────────────────────────────
void updateState(float score, uint8_t flags) {
  // STOPPED and CRITICAL only exit via user/recovery — don't override here
  if (motorState == STATE_STOPPED)  return;

  if (score >= SCORE_CRITICAL || (flags & FAULT_MULTI_SENSOR && score >= SCORE_FAULT)) {
    faultCount++; warnCount = 0; recoveryCount = 0;
    if (faultCount >= FAULT_PERSIST) motorState = STATE_CRITICAL;

  } else if (score >= SCORE_FAULT) {
    faultCount++; warnCount = 0; recoveryCount = 0;
    if (faultCount >= FAULT_PERSIST) motorState = STATE_FAULT;

  } else if (score >= SCORE_WARN) {
    warnCount++; faultCount = 0; recoveryCount = 0;
    if (warnCount >= WARNING_PERSIST) motorState = STATE_WARNING;

  } else {
    recoveryCount++;
    faultCount = 0; warnCount = 0;
    if (recoveryCount >= RECOVERY_PERSIST) {
      recoveryCount = 0;
      motorState    = STATE_NORMAL;
      lastFaultFlags = FAULT_NONE;
      lastScore      = 0;
    }
  }
}

// ─────────────────────────────────────────────
//  Trigger Warmup (reset baseline collection)
// ─────────────────────────────────────────────
void startWarmup() {
  baseRaw.reset();
  baseVib.reset();
  baseTemp.reset();
  inWarmup   = true;
  motorState = STATE_WARMUP;
  warnCount = faultCount = recoveryCount = 0;
  lastFaultFlags = FAULT_NONE;
  lastScore      = 0;
  Serial.println("[WARMUP] Baseline collection started...");
}

// ─────────────────────────────────────────────
//  Web Dashboard
// ─────────────────────────────────────────────
String buildDashboard() {

  // State colour
  String stateColor = "#00ff88";
  if (motorState == STATE_WARNING)  stateColor = "#ffaa00";
  if (motorState == STATE_FAULT)    stateColor = "#ff6600";
  if (motorState == STATE_CRITICAL) stateColor = "#ff2222";
  if (motorState == STATE_WARMUP)   stateColor = "#4499ff";
  if (motorState == STATE_STOPPED)  stateColor = "#888888";

  bool isHardFault = (motorState == STATE_CRITICAL);
  bool isSoftFault = (motorState == STATE_WARNING || motorState == STATE_FAULT);
  bool isStopped   = (motorState == STATE_STOPPED);

  // ── Alert Box ──
  String alertBox = "";

  if (isHardFault) {
    // Hard fault: auto-stopped, no user choice
    alertBox = R"(
<div class='alert hard'>
  <div class='alert-icon'>&#9888;</div>
  <b>CRITICAL FAULT — Emergency Stop</b><br>
  <small>)" + faultToString(lastFaultFlags) + R"(</small><br>
  <small>Fix the motor issue then re-warmup to resume.</small><br>
  <a href='/warmup'><button class='btn-blue' style='margin-top:8px'>Re-Warmup After Fix</button></a>
</div>)";

  } else if (isStopped) {
    // User stopped — show start button
    alertBox = R"(
<div class='alert info'>
  <div class='alert-icon'>&#9632;</div>
  <b>Motors Stopped</b><br>
  <small>Robot is powered but motors are off.</small><br>
  <a href='/start'><button class='btn-green' style='margin-top:8px'>&#9654; Start Robot</button></a>
</div>)";

  } else if (isSoftFault) {
    // Soft fault: show persistent alert with actions
    // Motors already slowed to 50% automatically
    alertBox = R"(
<div class='alert soft'>
  <div class='alert-icon'>&#9888;</div>
  <b>FAULT DETECTED</b> — Motors slowed to 50%<br>
  <small style='display:block;margin:4px 0'>)" + faultToString(lastFaultFlags) + R"(</small>
  <small>Score: )" + String(lastScore, 2) + R"( &nbsp;|&nbsp; Choose action:</small><br>
  <div style='margin-top:10px'>
    <a href='/action/turnoff'><button class='btn-red'>&#9646;&#9646; Turn Off Motors</button></a>
    <a href='/action/slowdown'><button class='btn-yellow'>&#9660; Slow Down (50%)</button></a>
  </div>
  <small style='display:block;margin-top:6px;color:#aa8833'>This alert stays until fault clears.</small>
</div>)";
  }

  // ── Warmup blocked message ──
  String warmupMsg = "";
  if (warmupBlocked) {
    warmupMsg = "<div class='alert soft' style='font-size:0.75rem;padding:8px'>"
                "&#9888; Cannot warmup during fault. Clear fault first.</div>";
    warmupBlocked = false; // show once
  }

  // ── Auto warmup countdown ──
  unsigned long elapsed  = millis() - lastAutoWarmup;
  unsigned long remaining = (elapsed < AUTO_WARMUP_INTERVAL)
                            ? (AUTO_WARMUP_INTERVAL - elapsed) / 1000
                            : 0;
  String autoWarmupInfo = "";
  if (motorState == STATE_NORMAL && !inWarmup) {
    autoWarmupInfo = "<div style='font-size:0.65rem;color:#446688;text-align:center;margin-bottom:6px'>"
                     "Auto re-warmup in " + String(remaining) + "s (only in NORMAL state)</div>";
  }

  // ── Build page ──
  String page = R"(<!DOCTYPE html>
<html>
<head>
<meta charset='UTF-8'>
<meta name='viewport' content='width=device-width,initial-scale=1'>
<meta http-equiv='refresh' content='2'>
<title>Robot Maintenance</title>
<style>
  @import url('https://fonts.googleapis.com/css2?family=Share+Tech+Mono&family=Exo+2:wght@400;600;700&display=swap');
  *{box-sizing:border-box;margin:0;padding:0}
  body{background:#090d14;color:#b8ccd8;font-family:'Exo 2',sans-serif;min-height:100vh;padding:14px;max-width:480px;margin:auto}
  h1{font-family:'Share Tech Mono',monospace;color:#00ccff;font-size:1.2rem;letter-spacing:3px;text-align:center;margin-bottom:14px;padding-bottom:10px;border-bottom:1px solid #1a2a3a}
  .status-bar{display:flex;justify-content:space-between;align-items:center;background:#0f1820;border:1px solid #1e3a5f;border-radius:8px;padding:10px 14px;margin-bottom:10px}
  .state-badge{font-family:'Share Tech Mono',monospace;font-size:0.95rem;font-weight:bold;padding:5px 14px;border-radius:4px;border:2px solid )" + stateColor + R"(;color:)" + stateColor + R"(}
  .grid{display:grid;grid-template-columns:1fr 1fr 1fr;gap:8px;margin-bottom:10px}
  .card{background:#0f1820;border:1px solid #1e3a5f;border-radius:8px;padding:10px;text-align:center}
  .card .val{font-family:'Share Tech Mono',monospace;font-size:1.2rem;color:#00ccff;margin:4px 0}
  .card .lbl{font-size:0.65rem;color:#6688aa;text-transform:uppercase;letter-spacing:1px}
  .card .base{font-size:0.65rem;color:#446688}
  .alert{border-radius:8px;padding:12px 14px;margin-bottom:10px;text-align:center;line-height:1.8}
  .alert-icon{font-size:1.6rem;margin-bottom:4px}
  .alert.hard{background:#1a0505;border:2px solid #ff3333;color:#ff9999}
  .alert.soft{background:#1a1200;border:2px solid #ffaa00;color:#ffcc55}
  .alert.info{background:#0a1520;border:2px solid #3399ff;color:#88ccff}
  .controls{background:#0f1820;border:1px solid #1e3a5f;border-radius:8px;padding:12px;margin-bottom:10px}
  .ctrl-title{font-size:0.65rem;color:#446688;text-transform:uppercase;letter-spacing:2px;text-align:center;margin-bottom:10px}
  .dpad{display:grid;grid-template-columns:1fr 1fr 1fr;gap:6px;max-width:200px;margin:0 auto}
  .btn{background:#111e2e;border:1px solid #1e3a5f;color:#b8ccd8;padding:16px;border-radius:6px;font-size:1.3rem;text-decoration:none;display:block;text-align:center;transition:background 0.15s}
  .btn:hover{background:#1a2e44}
  .btn.stop-btn{background:#1a0f0f;border-color:#5a2020;color:#ff8888}
  .btn-red{background:#4a1010;border:none;color:#ffaaaa;padding:9px 18px;border-radius:6px;font-size:0.82rem;cursor:pointer;margin:3px;font-family:'Exo 2',sans-serif}
  .btn-yellow{background:#2e1e00;border:none;color:#ffcc44;padding:9px 18px;border-radius:6px;font-size:0.82rem;cursor:pointer;margin:3px;font-family:'Exo 2',sans-serif}
  .btn-green{background:#0a2a14;border:2px solid #00aa44;color:#44ff88;padding:12px 28px;border-radius:6px;font-size:1rem;cursor:pointer;font-family:'Exo 2',sans-serif;font-weight:600}
  .btn-blue{background:#0a1a2e;border:2px solid #0066cc;color:#66aaff;padding:9px 18px;border-radius:6px;font-size:0.82rem;cursor:pointer;font-family:'Exo 2',sans-serif}
  .warmup-section{background:#0f1820;border:1px solid #1e3a5f;border-radius:8px;padding:12px;margin-bottom:10px;text-align:center}
  .warmup-bar{background:#0a1520;border-radius:4px;height:8px;margin:8px 0}
  .warmup-fill{background:linear-gradient(90deg,#0066cc,#00ccff);height:8px;border-radius:4px;transition:width 0.3s}
  .fault-tags{text-align:center;margin-bottom:10px}
  .tag{display:inline-block;background:#1a0a2e;border:1px solid #5533aa;color:#aa88ff;font-size:0.65rem;padding:2px 8px;border-radius:12px;margin:2px;font-family:'Share Tech Mono',monospace}
  .footer{text-align:center;font-size:0.6rem;color:#2a3a4a;margin-top:8px}
  .empty{display:block}
</style>
</head>
<body>
<h1>&#9881; ROBOT MAINTENANCE</h1>

<div class='status-bar'>
  <div>
    <div style='font-size:0.6rem;color:#446688;margin-bottom:3px'>MOTOR STATE</div>
    <div class='state-badge'>)" + stateToString(motorState) + R"(</div>
  </div>
  <div style='text-align:right'>
    <div style='font-size:0.6rem;color:#446688'>FAULT SCORE</div>
    <div style='font-family:Share Tech Mono,monospace;font-size:1.3rem;color:#ffcc44'>)" + String(lastScore, 2) + R"(</div>
  </div>
</div>

<div class='grid'>
  <div class='card'>
    <div class='lbl'>Current</div>
    <div class='val'>)" + String((int)lastRaw) + R"(</div>
    <div class='base'>base )" + String(baseRaw.mean, 0) + R"(</div>
  </div>
  <div class='card'>
    <div class='lbl'>Vibration</div>
    <div class='val'>)" + String(lastVib, 2) + R"(</div>
    <div class='base'>base )" + String(baseVib.mean, 2) + R"(</div>
  </div>
  <div class='card'>
    <div class='lbl'>Temp &deg;C</div>
    <div class='val'>)" + String(lastTemp, 1) + R"(</div>
    <div class='base'>base )" + String(baseTemp.mean, 1) + R"(</div>
  </div>
</div>

)" + alertBox + warmupMsg + R"(

<div class='controls'>
  <div class='ctrl-title'>Direction Control</div>
  <div class='dpad'>
    <span class='empty'></span>
    <a href='/cmd/forward'  class='btn'>&#8679;</a>
    <span class='empty'></span>
    <a href='/cmd/left'     class='btn'>&#8678;</a>
    <a href='/cmd/stop'     class='btn stop-btn'>&#9632;</a>
    <a href='/cmd/right'    class='btn'>&#8680;</a>
    <span class='empty'></span>
    <a href='/cmd/backward' class='btn'>&#8681;</a>
    <span class='empty'></span>
  </div>
</div>

)";

  // Fault tags row
  if (lastFaultFlags != FAULT_NONE) {
    page += "<div class='fault-tags'>";
    if (lastFaultFlags & FAULT_OVERCURRENT)  page += "<span class='tag'>OVERCURRENT</span>";
    if (lastFaultFlags & FAULT_VIBRATION)    page += "<span class='tag'>VIBRATION</span>";
    if (lastFaultFlags & FAULT_OVERTEMP)     page += "<span class='tag'>OVERTEMP</span>";
    if (lastFaultFlags & FAULT_SUDDEN_SPIKE) page += "<span class='tag'>SUDDEN SPIKE</span>";
    if (lastFaultFlags & FAULT_MULTI_SENSOR) page += "<span class='tag'>MULTI-SENSOR</span>";
    page += "</div>";
  }

  // ── Warmup Section ──
  page += "<div class='warmup-section'>";
  if (inWarmup) {
    int pct = baseRaw.progress();
    page += "<div style='font-size:0.75rem;color:#4499ff;margin-bottom:6px'>"
            "&#9654; Collecting baseline... " + String(pct) + "%</div>"
            "<div class='warmup-bar'><div class='warmup-fill' style='width:" + String(pct) + "%'></div></div>"
            "<div style='font-size:0.65rem;color:#446688;margin-top:6px'>"
            "Keep robot in NORMAL running condition during warmup.</div>";
  } else {
    page += autoWarmupInfo;
    page += "<div style='font-size:0.7rem;color:#446688;margin-bottom:8px'>"
            "&#9432; Warmup in NORMAL state only — not during faults.</div>"
            "<a href='/warmup'><button class='btn-blue'>&#9664; Manual Re-Warmup</button></a>";
  }
  page += "</div>";

  page += R"(
<div class='footer'>Auto-refresh 2s &nbsp;|&nbsp; )" + WiFi.softAPIP().toString() + R"(</div>
</body></html>)";

  return page;
}

// ─────────────────────────────────────────────
//  Setup
// ─────────────────────────────────────────────
void setup() {
  Serial.begin(115200);

  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT); pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT); pinMode(ENB, OUTPUT);

  dht.begin();

  if (!mpu.begin()) {
    Serial.println("MPU6050 not found!");
    while (1) delay(500);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  WiFi.softAP(ssid, password);
  server.begin();

  lastAutoWarmup = millis();  // start auto-warmup timer
  startWarmup();              // begin baseline collection on boot

  Serial.println("===== ROBOT MAINTENANCE STARTED =====");
  Serial.println("Dashboard: http://" + WiFi.softAPIP().toString());
}

// ─────────────────────────────────────────────
//  Loop
// ─────────────────────────────────────────────
void loop() {

  // ── Web Request Handling ──
  WiFiClient client = server.available();
  if (client) {
    String req = client.readStringUntil('\r');
    client.flush();

    // ── Direction commands ──
    if (req.indexOf("/cmd/forward")  != -1) { moveCmd = "forward";  lastMoveCmd = moveCmd; }
    if (req.indexOf("/cmd/backward") != -1) { moveCmd = "backward"; lastMoveCmd = moveCmd; }
    if (req.indexOf("/cmd/left")     != -1) { moveCmd = "left";     lastMoveCmd = moveCmd; }
    if (req.indexOf("/cmd/right")    != -1) { moveCmd = "right";    lastMoveCmd = moveCmd; }
    if (req.indexOf("/cmd/stop")     != -1) { moveCmd = "stop"; }

    // ── Fault actions ──
    if (req.indexOf("/action/turnoff") != -1) {
      motorState  = STATE_STOPPED;
      motorEmergencyStop();
    }
    if (req.indexOf("/action/slowdown") != -1) {
      // Already slowed in applyMovement — just acknowledge
    }

    // ── Start robot after stop ──
    if (req.indexOf("/start") != -1) {
      if (motorState == STATE_STOPPED) {
        // Only restart if no active hard fault
        if (lastScore < SCORE_CRITICAL) {
          motorState = STATE_NORMAL;
          moveCmd    = lastMoveCmd;  // resume last direction
          warnCount = faultCount = recoveryCount = 0;
        }
      }
    }

    // ── Manual warmup ──
    if (req.indexOf("/warmup") != -1) {
      // Block if fault or warning is active
      if (motorState == STATE_WARNING || motorState == STATE_FAULT || motorState == STATE_CRITICAL) {
        warmupBlocked = true;
        Serial.println("[WARMUP] Blocked — fault active");
      } else {
        startWarmup();
        lastAutoWarmup = millis();  // reset auto timer too
      }
    }

    String html = buildDashboard();
    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: text/html");
    client.println("Connection: close");
    client.println("Content-Length: " + String(html.length()));
    client.println();
    client.print(html);
    client.stop();
  }

  // ─────────────────────────────────────────
  //  Sensor Reading
  // ─────────────────────────────────────────
  int raw = analogRead(ACS712_PIN);

  sensors_event_t a, g, tm;
  mpu.getEvent(&a, &g, &tm);
  float accel = sqrt(a.acceleration.x * a.acceleration.x +
                     a.acceleration.y * a.acceleration.y +
                     a.acceleration.z * a.acceleration.z);
  float vibration = fabs(accel - 9.8f);

  float temperature = dht.readTemperature();
  if (isnan(temperature) || temperature < 10.0f) temperature = 30.0f;
  temperature += 2.0f;

  int delta = raw - prevRaw;
  prevRaw   = raw;

  lastRaw  = raw;
  lastVib  = vibration;
  lastTemp = temperature;

  // ─────────────────────────────────────────
  //  Warmup Phase — Feed baseline
  // ─────────────────────────────────────────
  if (inWarmup) {
    bool rawDone  = baseRaw.feed((float)raw);
    baseVib.feed(vibration);
    baseTemp.feed(temperature);

    if (rawDone) {
      inWarmup   = false;
      motorState = STATE_NORMAL;
      lastAutoWarmup = millis();  // reset auto-warmup timer after successful warmup
      Serial.println("[WARMUP] Done. raw=" + String(baseRaw.mean, 1)
                   + " vib=" + String(baseVib.mean, 3)
                   + " temp=" + String(baseTemp.mean, 1));
    }
    motorStop();  // keep motors off during warmup
    delay(200);
    return;
  }

  // ─────────────────────────────────────────
  //  Auto Re-Warmup (every 5 min, NORMAL only)
  // ─────────────────────────────────────────
  if (motorState == STATE_NORMAL &&
      (millis() - lastAutoWarmup) >= AUTO_WARMUP_INTERVAL) {
    Serial.println("[AUTO-WARMUP] Triggering scheduled re-warmup...");
    startWarmup();
    // lastAutoWarmup will be reset when warmup completes
    delay(200);
    return;
  }

  // ─────────────────────────────────────────
  //  Fault Analysis
  // ─────────────────────────────────────────
  if (motorState != STATE_STOPPED && motorState != STATE_WARMUP) {
    lastScore = analyzeFault((float)raw, vibration, temperature, delta, lastFaultFlags);
    updateState(lastScore, lastFaultFlags);

    if (motorState != STATE_NORMAL) {
      Serial.println("[" + stateToString(motorState) + "] score="
                   + String(lastScore, 2) + " | " + faultToString(lastFaultFlags));
    }
  }

  // ─────────────────────────────────────────
  //  Apply Motors
  // ─────────────────────────────────────────
  applyMovement();

  delay(200);
}
