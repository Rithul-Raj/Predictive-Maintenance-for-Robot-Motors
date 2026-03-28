/*
 * ================================================================
 *  PREDICTIVE MAINTENANCE — WEB CONTROLLED ROBOT
 *  Sensors : ACS712 (current), MPU6050 (vibration), DHT11 (temp)
 *  Control : Web dashboard via WiFi hotspot
 *  Faults  : Adaptive Z-Score + Weighted Score + State Machine
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
//  Adaptive Baseline Config
// ─────────────────────────────────────────────
#define WARMUP_SAMPLES   60
#define ROLLING_WINDOW   80

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
#define Z_RAW_FAULT   3.5f
#define Z_VIB_WARN    2.8f
#define Z_VIB_FAULT   4.0f
#define Z_TEMP_WARN   2.0f
#define Z_TEMP_FAULT  3.0f

// Weighted score thresholds
#define SCORE_WARN     3.5f
#define SCORE_FAULT    6.0f
#define SCORE_CRITICAL 8.0f

// Hard absolute limits
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
//  Rolling Stats
// ─────────────────────────────────────────────
struct RollingStats {
  float buf[ROLLING_WINDOW];
  int   idx   = 0;
  int   count = 0;
  float sum   = 0;
  float sumSq = 0;
  float mean  = 0;
  float stdev = 1;

  void push(float val) {
    if (count == ROLLING_WINDOW) {
      float old = buf[idx];
      sum   -= old;
      sumSq -= old * old;
    } else {
      count++;
    }
    buf[idx] = val;
    idx = (idx + 1) % ROLLING_WINDOW;
    sum   += val;
    sumSq += val * val;
    mean  = sum / count;
    float v = (sumSq / count) - (mean * mean);
    stdev = (v > 0) ? sqrt(v) : 0.5f;
  }

  float zScore(float val) { return fabs(val - mean) / stdev; }
  bool  ready()           { return count >= WARMUP_SAMPLES; }
};

RollingStats statsRaw, statsVib, statsTemp;

// ─────────────────────────────────────────────
//  State
// ─────────────────────────────────────────────
enum MotorState { STATE_WARMUP, STATE_NORMAL, STATE_WARNING, STATE_FAULT, STATE_CRITICAL };
MotorState motorState = STATE_WARMUP;

int warnCount = 0, faultCount = 0, recoveryCount = 0;
int prevRaw   = 0;

// Robot movement command from web
// "stop","forward","backward","left","right"
String moveCmd = "stop";

// Latest sensor values (for web dashboard)
float  lastRaw  = 0, lastVib = 0, lastTemp = 0;
float  lastScore = 0;
uint8_t lastFaultFlags = 0;

// User action from web on soft fault
// "none","turnoff","precaution"
String userAction = "none";

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
  analogWrite(ENA, 0);   analogWrite(ENB, 0);
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
}

// ─────────────────────────────────────────────
//  Apply movement based on command + state
// ─────────────────────────────────────────────
void applyMovement() {
  // Hard faults always override web command
  if (motorState == STATE_CRITICAL) {
    motorEmergencyStop();
    return;
  }

  // If user chose turnoff on soft fault
  if (userAction == "turnoff") {
    motorStop();
    return;
  }

  // Precaution mode — slow down
  if (userAction == "precaution" || motorState == STATE_WARNING) {
    int slowSpd = 90;
    if      (moveCmd == "forward")  motorForward(slowSpd);
    else if (moveCmd == "backward") motorBackward(slowSpd);
    else if (moveCmd == "left")     motorLeft(slowSpd);
    else if (moveCmd == "right")    motorRight(slowSpd);
    else                            motorStop();
    return;
  }

  // FAULT state without user decision — stop and wait
  if (motorState == STATE_FAULT) {
    motorStop();
    return;
  }

  // Normal operation — full speed
  if      (moveCmd == "forward")  motorForward();
  else if (moveCmd == "backward") motorBackward();
  else if (moveCmd == "left")     motorLeft();
  else if (moveCmd == "right")    motorRight();
  else                            motorStop();
}

// ─────────────────────────────────────────────
//  Fault Analysis
// ─────────────────────────────────────────────
float analyzeFault(float raw, float vib, float temp, int delta, uint8_t &flags) {
  flags = FAULT_NONE;
  float zR = statsRaw.zScore(raw);
  float zV = statsVib.zScore(vib);
  float zT = statsTemp.zScore(temp);
  uint8_t hits = 0;

  if (zR >= Z_RAW_WARN)          { flags |= FAULT_OVERCURRENT;  hits++; }
  if (zV >= Z_VIB_WARN || vib > HARD_VIB_LIMIT) { flags |= FAULT_VIBRATION; hits++; }
  if (zT >= Z_TEMP_WARN || temp > HARD_TEMP_LIMIT) { flags |= FAULT_OVERTEMP; hits++; }
  if (abs(delta) > HARD_DELTA_LIMIT) { flags |= FAULT_SUDDEN_SPIKE; hits++; }
  if (hits >= 2)                     flags |= FAULT_MULTI_SENSOR;

  float score = (zR * 0.40f) + (zV * 0.40f) + (zT * 0.20f);
  if (flags & FAULT_MULTI_SENSOR)  score *= 1.3f;
  if (flags & FAULT_SUDDEN_SPIKE)  score += 2.0f;
  return score;
}

// ─────────────────────────────────────────────
//  State Machine
// ─────────────────────────────────────────────
void updateState(float score, uint8_t flags) {
  if (score >= SCORE_CRITICAL || (flags & FAULT_MULTI_SENSOR && score >= SCORE_FAULT)) {
    faultCount++; warnCount = 0; recoveryCount = 0;
    if (faultCount >= FAULT_PERSIST) { motorState = STATE_CRITICAL; userAction = "none"; }
  } else if (score >= SCORE_FAULT) {
    faultCount++; warnCount = 0; recoveryCount = 0;
    if (faultCount >= FAULT_PERSIST) {
      if (motorState != STATE_FAULT) userAction = "none"; // reset user choice on new fault
      motorState = STATE_FAULT;
    }
  } else if (score >= SCORE_WARN) {
    warnCount++; faultCount = 0; recoveryCount = 0;
    if (warnCount >= WARNING_PERSIST) motorState = STATE_WARNING;
  } else {
    recoveryCount++;
    if (recoveryCount >= RECOVERY_PERSIST) {
      motorState = STATE_NORMAL;
      warnCount = faultCount = recoveryCount = 0;
      userAction = "none";
    }
  }
}

// ─────────────────────────────────────────────
//  Web Dashboard HTML
// ─────────────────────────────────────────────
String buildDashboard() {
  String stateColor = "#00ff88";
  if (motorState == STATE_WARNING)  stateColor = "#ffaa00";
  if (motorState == STATE_FAULT)    stateColor = "#ff4444";
  if (motorState == STATE_CRITICAL) stateColor = "#ff0000";
  if (motorState == STATE_WARMUP)   stateColor = "#888888";

  bool isSoftFault = (motorState == STATE_WARNING || motorState == STATE_FAULT);
  bool isHardFault = (motorState == STATE_CRITICAL);

  String alertBox = "";
  if (isHardFault) {
    alertBox = R"(
      <div class='alert hard'>
        &#9888; CRITICAL FAULT — Motors Emergency Stopped<br>
        <small>)" + faultToString(lastFaultFlags) + R"(</small>
      </div>)";
  } else if (isSoftFault && userAction == "none") {
    alertBox = R"(
      <div class='alert soft'>
        &#9888; FAULT DETECTED: )" + faultToString(lastFaultFlags) + R"(<br>
        <small>Choose an action:</small><br>
        <a href='/action/turnoff'><button class='btn-red'>Turn Off Motors</button></a>
        <a href='/action/precaution'><button class='btn-yellow'>Precaution Mode (slow)</button></a>
        <a href='/action/ignore'><button class='btn-gray'>Ignore &amp; Continue</button></a>
      </div>)";
  } else if (isSoftFault && userAction != "none") {
    alertBox = "<div class='alert soft'>&#9888; " + faultToString(lastFaultFlags)
             + " | Mode: <b>" + userAction + "</b>"
             + " <a href='/action/none'><button class='btn-gray' style='padding:4px 10px;font-size:12px'>Reset</button></a></div>";
  }

  String page = R"(<!DOCTYPE html>
<html>
<head>
<meta charset='UTF-8'>
<meta name='viewport' content='width=device-width,initial-scale=1'>
<meta http-equiv='refresh' content='2'>
<title>Robot Control</title>
<style>
  @import url('https://fonts.googleapis.com/css2?family=Share+Tech+Mono&family=Exo+2:wght@400;700&display=swap');
  *{box-sizing:border-box;margin:0;padding:0}
  body{background:#0a0e17;color:#c8d6e5;font-family:'Exo 2',sans-serif;min-height:100vh;padding:16px}
  h1{font-family:'Share Tech Mono',monospace;color:#00ccff;font-size:1.3rem;letter-spacing:3px;text-align:center;margin-bottom:16px}
  .status-bar{display:flex;justify-content:space-between;align-items:center;background:#111827;border:1px solid #1e3a5f;border-radius:8px;padding:10px 14px;margin-bottom:12px}
  .state-badge{font-family:'Share Tech Mono',monospace;font-size:1rem;font-weight:bold;padding:4px 12px;border-radius:4px;background:#111;border:2px solid )" + stateColor + R"(;color:)" + stateColor + R"(}
  .score{font-size:0.8rem;color:#8899aa}
  .grid{display:grid;grid-template-columns:1fr 1fr 1fr;gap:8px;margin-bottom:12px}
  .card{background:#111827;border:1px solid #1e3a5f;border-radius:8px;padding:10px;text-align:center}
  .card .val{font-family:'Share Tech Mono',monospace;font-size:1.3rem;color:#00ccff;margin:4px 0}
  .card .lbl{font-size:0.7rem;color:#8899aa;text-transform:uppercase;letter-spacing:1px}
  .alert{border-radius:8px;padding:12px;margin-bottom:12px;font-size:0.85rem;line-height:1.7;text-align:center}
  .alert.hard{background:#2a0a0a;border:2px solid #ff4444;color:#ff8888}
  .alert.soft{background:#1a1500;border:2px solid #ffaa00;color:#ffcc44}
  .controls{background:#111827;border:1px solid #1e3a5f;border-radius:8px;padding:12px;margin-bottom:12px}
  .ctrl-grid{display:grid;grid-template-columns:1fr 1fr 1fr;gap:8px;max-width:240px;margin:0 auto}
  .btn{background:#1a2a3a;border:1px solid #2a4a6a;color:#c8d6e5;padding:14px;border-radius:6px;font-size:1.2rem;text-decoration:none;display:block;text-align:center;cursor:pointer;transition:background 0.15s}
  .btn:active,.btn:hover{background:#1e3a5f}
  .btn.stop{background:#2a1a1a;border-color:#6a2a2a;color:#ff8888}
  .btn-red{background:#5a1a1a;border:none;color:#ffaaaa;padding:8px 16px;border-radius:6px;font-size:0.85rem;cursor:pointer;margin:4px}
  .btn-yellow{background:#3a2a00;border:none;color:#ffcc44;padding:8px 16px;border-radius:6px;font-size:0.85rem;cursor:pointer;margin:4px}
  .btn-gray{background:#1e2a3a;border:none;color:#8899aa;padding:8px 16px;border-radius:6px;font-size:0.85rem;cursor:pointer;margin:4px}
  .fault-tag{display:inline-block;background:#1a0a2a;border:1px solid #5533aa;color:#aa88ff;font-size:0.7rem;padding:2px 8px;border-radius:12px;margin:2px;font-family:'Share Tech Mono',monospace}
  .warmup-bar{background:#1a2a3a;border-radius:4px;height:6px;margin-top:6px}
  .warmup-fill{background:#00ccff;height:6px;border-radius:4px}
  .section-title{font-size:0.7rem;color:#556677;text-transform:uppercase;letter-spacing:2px;margin-bottom:8px}
</style>
</head>
<body>
<h1>&#9881; ROBOT MAINTENANCE</h1>

<div class='status-bar'>
  <div>
    <div class='section-title'>Motor State</div>
    <div class='state-badge'>)" + stateToString(motorState) + R"(</div>
  </div>
  <div style='text-align:right'>
    <div class='score'>Fault Score</div>
    <div style='font-family:Share Tech Mono,monospace;font-size:1.2rem;color:#ffcc44'>)" + String(lastScore, 2) + R"(</div>
  </div>
</div>

<div class='grid'>
  <div class='card'>
    <div class='lbl'>Current (raw)</div>
    <div class='val'>)" + String((int)lastRaw) + R"(</div>
    <div class='lbl'>&#x7e; )" + String(statsRaw.mean, 0) + R"( base</div>
  </div>
  <div class='card'>
    <div class='lbl'>Vibration</div>
    <div class='val'>)" + String(lastVib, 2) + R"(</div>
    <div class='lbl'>&#x7e; )" + String(statsVib.mean, 2) + R"( base</div>
  </div>
  <div class='card'>
    <div class='lbl'>Temp &deg;C</div>
    <div class='val'>)" + String(lastTemp, 1) + R"(</div>
    <div class='lbl'>&#x7e; )" + String(statsTemp.mean, 1) + R"( base</div>
  </div>
</div>

)" + alertBox + R"(

<div class='controls'>
  <div class='section-title' style='text-align:center'>Direction Control)</div>
  <div class='ctrl-grid'>
    <div></div>
    <a href='/cmd/forward' class='btn'>&#8679;</a>
    <div></div>
    <a href='/cmd/left'    class='btn'>&#8678;</a>
    <a href='/cmd/stop'    class='btn stop'>&#9632;</a>
    <a href='/cmd/right'   class='btn'>&#8680;</a>
    <div></div>
    <a href='/cmd/backward' class='btn'>&#8681;</a>
    <div></div>
  </div>
</div>

)";

  // Fault tags
  if (lastFaultFlags != FAULT_NONE) {
    page += "<div style='text-align:center;margin-bottom:12px'>";
    if (lastFaultFlags & FAULT_OVERCURRENT)  page += "<span class='fault-tag'>OVERCURRENT</span>";
    if (lastFaultFlags & FAULT_VIBRATION)    page += "<span class='fault-tag'>VIBRATION</span>";
    if (lastFaultFlags & FAULT_OVERTEMP)     page += "<span class='fault-tag'>OVERTEMP</span>";
    if (lastFaultFlags & FAULT_SUDDEN_SPIKE) page += "<span class='fault-tag'>SUDDEN SPIKE</span>";
    if (lastFaultFlags & FAULT_MULTI_SENSOR) page += "<span class='fault-tag'>MULTI-SENSOR</span>";
    page += "</div>";
  }

  // Warmup progress
  if (motorState == STATE_WARMUP) {
    int pct = min(100, (statsRaw.count * 100) / WARMUP_SAMPLES);
    page += "<div class='card' style='margin-bottom:12px'>"
            "<div class='lbl'>Calibrating baseline — " + String(pct) + "%</div>"
            "<div class='warmup-bar'><div class='warmup-fill' style='width:" + String(pct) + "%'></div></div>"
            "</div>";
  }

  page += R"(
<div style='text-align:center;font-size:0.65rem;color:#334455;margin-top:8px'>
  Auto-refresh 2s &nbsp;|&nbsp; IP: )" + WiFi.softAPIP().toString() + R"(
</div>
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

  Serial.println("===== ROBOT STARTED =====");
  Serial.println("Dashboard: http://" + WiFi.softAPIP().toString());
}

// ─────────────────────────────────────────────
//  Loop
// ─────────────────────────────────────────────
void loop() {

  // ── Web Requests ──
  WiFiClient client = server.available();
  if (client) {
    String req = client.readStringUntil('\r');
    client.flush();

    // Movement commands
    if (req.indexOf("/cmd/forward")  != -1) moveCmd = "forward";
    if (req.indexOf("/cmd/backward") != -1) moveCmd = "backward";
    if (req.indexOf("/cmd/left")     != -1) moveCmd = "left";
    if (req.indexOf("/cmd/right")    != -1) moveCmd = "right";
    if (req.indexOf("/cmd/stop")     != -1) moveCmd = "stop";

    // User fault actions
    if (req.indexOf("/action/turnoff")    != -1) userAction = "turnoff";
    if (req.indexOf("/action/precaution") != -1) userAction = "precaution";
    if (req.indexOf("/action/ignore")     != -1) userAction = "ignore";
    if (req.indexOf("/action/none")       != -1) userAction = "none";

    String html = buildDashboard();
    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: text/html");
    client.println("Connection: close");
    client.println("Content-Length: " + String(html.length()));
    client.println();
    client.print(html);
    client.stop();
  }

  // ── Sensor Read ──
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

  // ── Push to Rolling Stats ──
  statsRaw.push((float)raw);
  statsVib.push(vibration);
  statsTemp.push(temperature);

  lastRaw  = raw;
  lastVib  = vibration;
  lastTemp = temperature;

  // ── Fault Analysis (after warmup) ──
  if (statsRaw.ready()) {
    if (motorState == STATE_WARMUP) {
      motorState = STATE_NORMAL;
      Serial.println("[INFO] Baseline ready. raw_mean=" + String(statsRaw.mean, 1)
                   + " vib_mean=" + String(statsVib.mean, 3));
    }

    lastScore = analyzeFault((float)raw, vibration, temperature, delta, lastFaultFlags);
    updateState(lastScore, lastFaultFlags);

    // Serial log on fault only
    if (motorState != STATE_NORMAL) {
      Serial.println("[" + stateToString(motorState) + "] score=" + String(lastScore, 2)
                   + " flags=" + faultToString(lastFaultFlags));
    }
  }

  // ── Apply Motor Movement ──
  applyMovement();

  delay(200);
}
