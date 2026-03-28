/*
 * ============================================================
 *  PREDICTIVE MAINTENANCE — ROBOT MOTOR
 *  Sensors : ACS712 (current), MPU6050 (vibration), DHT11 (temp)
 *  Logic   : Adaptive Baseline + Rolling Z-Score + Weighted Score
 *            + State Machine + Fault Combination Detection
 * ============================================================
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
WiFiServer dataServer(12345);
WiFiClient dataClient;

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
#define WARMUP_SAMPLES     60     // readings before fault detection starts
#define ROLLING_WINDOW     80     // samples for rolling mean/std update
#define ROLLING_UPDATE_INT 50     // update rolling stats every N samples

// ─────────────────────────────────────────────
//  State Machine Config
// ─────────────────────────────────────────────
#define WARNING_PERSIST    5      // consecutive warnings before escalating
#define FAULT_PERSIST      3      // consecutive fault scores before alerting
#define RECOVERY_PERSIST   10     // consecutive normals before clearing fault

// ─────────────────────────────────────────────
//  Z-Score Thresholds (sensor-specific)
// ─────────────────────────────────────────────
// Based on dataset analysis:
//   raw current drifts heavily between sessions → z=2.5 is safe
//   vibration spikes more sharply              → z=2.8
//   temp changes slowly                        → z=2.0
#define Z_RAW_WARN    2.5f
#define Z_RAW_FAULT   3.5f
#define Z_VIB_WARN    2.8f
#define Z_VIB_FAULT   4.0f
#define Z_TEMP_WARN   2.0f
#define Z_TEMP_FAULT  3.0f

// Weighted fault score thresholds
#define SCORE_WARN    3.5f
#define SCORE_FAULT   6.0f
#define SCORE_CRITICAL 8.0f

// Hard absolute limits (safety net — catches sensor runaway)
// From dataset: raw never exceeded 4095 even in fault; vibration >15 is extreme
#define HARD_VIB_LIMIT   18.0f
#define HARD_TEMP_LIMIT  70.0f    // DHT11 motor overtemp limit
#define HARD_DELTA_LIMIT 600      // sudden current spike limit (raw units)

// ─────────────────────────────────────────────
//  Circular Buffer for Rolling Stats
// ─────────────────────────────────────────────
struct RollingStats {
  float buf[ROLLING_WINDOW];
  int   idx    = 0;
  int   count  = 0;
  float sum    = 0;
  float sumSq  = 0;
  float mean   = 0;
  float stdev  = 1;   // never zero

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
    float variance = (sumSq / count) - (mean * mean);
    stdev = (variance > 0) ? sqrt(variance) : 0.5f;
  }

  float zScore(float val) {
    return fabs(val - mean) / stdev;
  }

  bool ready() { return count >= WARMUP_SAMPLES; }
};

// ─────────────────────────────────────────────
//  Per-Sensor Rolling Stats
// ─────────────────────────────────────────────
RollingStats statsRaw;
RollingStats statsVib;
RollingStats statsTemp;

// ─────────────────────────────────────────────
//  State Machine
// ─────────────────────────────────────────────
enum MotorState { STATE_WARMUP, STATE_NORMAL, STATE_WARNING, STATE_FAULT, STATE_CRITICAL };
MotorState motorState = STATE_WARMUP;

int  warnCount     = 0;
int  faultCount    = 0;
int  recoveryCount = 0;
int  sampleCount   = 0;

// ─────────────────────────────────────────────
//  Fault Flag Bits  (combinable)
// ─────────────────────────────────────────────
#define FAULT_NONE          0x00
#define FAULT_OVERCURRENT   0x01   // raw current spike
#define FAULT_VIBRATION     0x02   // bearing / imbalance
#define FAULT_OVERTEMP      0x04   // thermal
#define FAULT_SUDDEN_SPIKE  0x08   // delta spike (sudden load)
#define FAULT_MULTI_SENSOR  0x10   // 2+ sensors triggered simultaneously

// ─────────────────────────────────────────────
//  Robot command
// ─────────────────────────────────────────────
String command = "normal";

// ─────────────────────────────────────────────
//  Previous raw for delta spike detection
// ─────────────────────────────────────────────
int prevRaw = 0;

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

String faultCodeToString(uint8_t code) {
  if (code == FAULT_NONE) return "NONE";
  String s = "";
  if (code & FAULT_OVERCURRENT)  s += "OVERCURRENT|";
  if (code & FAULT_VIBRATION)    s += "VIBRATION|";
  if (code & FAULT_OVERTEMP)     s += "OVERTEMP|";
  if (code & FAULT_SUDDEN_SPIKE) s += "SUDDEN_SPIKE|";
  if (code & FAULT_MULTI_SENSOR) s += "MULTI_SENSOR|";
  if (s.endsWith("|")) s = s.substring(0, s.length() - 1);
  return s;
}

// ─────────────────────────────────────────────
//  Core Fault Analysis
//  Returns: weighted fault score + sets faultFlags
// ─────────────────────────────────────────────
float analyzeFault(float raw, float vibration, float temperature,
                   int delta, uint8_t &faultFlags) {
  faultFlags = FAULT_NONE;

  float zRaw  = statsRaw.zScore(raw);
  float zVib  = statsVib.zScore(vibration);
  float zTemp = statsTemp.zScore(temperature);

  // ── Per-sensor fault flags ──
  uint8_t sensorHits = 0;

  if (zRaw >= Z_RAW_WARN) {
    faultFlags |= FAULT_OVERCURRENT;
    sensorHits++;
  }
  if (zVib >= Z_VIB_WARN) {
    faultFlags |= FAULT_VIBRATION;
    sensorHits++;
  }
  if (zTemp >= Z_TEMP_WARN) {
    faultFlags |= FAULT_OVERTEMP;
    sensorHits++;
  }

  // ── Sudden delta spike (hard absolute) ──
  if (abs(delta) > HARD_DELTA_LIMIT) {
    faultFlags |= FAULT_SUDDEN_SPIKE;
    sensorHits++;
  }

  // ── Hard absolute safety nets ──
  if (vibration > HARD_VIB_LIMIT) faultFlags |= FAULT_VIBRATION;
  if (temperature > HARD_TEMP_LIMIT) faultFlags |= FAULT_OVERTEMP;

  // ── Multi-sensor flag ──
  if (sensorHits >= 2) faultFlags |= FAULT_MULTI_SENSOR;

  // ── Weighted Score ──
  //  Current & vibration are primary indicators from dataset analysis
  //  Temp is secondary (DHT11 is slow, temp was flat at 32 in all sessions)
  float score = (zRaw  * 0.40f)
              + (zVib  * 0.40f)
              + (zTemp * 0.20f);

  // Boost score if multi-sensor
  if (faultFlags & FAULT_MULTI_SENSOR) score *= 1.3f;

  // Boost score if sudden spike
  if (faultFlags & FAULT_SUDDEN_SPIKE) score += 2.0f;

  return score;
}

// ─────────────────────────────────────────────
//  State Machine Transition
// ─────────────────────────────────────────────
MotorState updateStateMachine(float score, uint8_t faultFlags) {
  if (score >= SCORE_CRITICAL || (faultFlags & FAULT_MULTI_SENSOR && score >= SCORE_FAULT)) {
    faultCount++;
    warnCount     = 0;
    recoveryCount = 0;
    if (faultCount >= FAULT_PERSIST) return STATE_CRITICAL;

  } else if (score >= SCORE_FAULT) {
    faultCount++;
    warnCount     = 0;
    recoveryCount = 0;
    if (faultCount >= FAULT_PERSIST) return STATE_FAULT;

  } else if (score >= SCORE_WARN) {
    warnCount++;
    faultCount    = 0;
    recoveryCount = 0;
    if (warnCount >= WARNING_PERSIST) return STATE_WARNING;

  } else {
    recoveryCount++;
    if (recoveryCount >= RECOVERY_PERSIST) {
      warnCount  = 0;
      faultCount = 0;
      return STATE_NORMAL;
    }
    // Hold current state during recovery period
    return motorState;
  }

  return motorState; // hold current if not yet persisted
}

// ─────────────────────────────────────────────
//  Fault Action — what to do per state
// ─────────────────────────────────────────────
void handleFaultAction(MotorState state, uint8_t faultFlags, float score) {
  switch (state) {

    case STATE_WARNING:
      // Slow down motors as precaution
      analogWrite(ENA, 120);
      analogWrite(ENB, 120);
      Serial.println("[WARNING] Motor slowed — monitoring");
      break;

    case STATE_FAULT:
      // Stop if multi-sensor or vibration fault (bearing risk)
      if ((faultFlags & FAULT_MULTI_SENSOR) || (faultFlags & FAULT_VIBRATION)) {
        analogWrite(ENA, 0);
        analogWrite(ENB, 0);
        Serial.println("[FAULT] Motor STOPPED — " + faultCodeToString(faultFlags));
      } else {
        // Single sensor fault — reduce speed, continue
        analogWrite(ENA, 80);
        analogWrite(ENB, 80);
        Serial.println("[FAULT] Motor reduced — " + faultCodeToString(faultFlags));
      }
      break;

    case STATE_CRITICAL:
      // Full stop immediately
      analogWrite(ENA, 0);
      analogWrite(ENB, 0);
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, LOW);
      Serial.println("[CRITICAL] Emergency stop — " + faultCodeToString(faultFlags));
      break;

    default:
      break;
  }
}

// ─────────────────────────────────────────────
//  Motor Control
// ─────────────────────────────────────────────
void moveForward() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, 180);
  analogWrite(ENB, 180);
}

void stopMotor() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
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
    while (1);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  WiFi.softAP(ssid, password);
  server.begin();
  dataServer.begin();

  Serial.println("\n===== ESP32 PREDICTIVE MAINTENANCE STARTED =====");
  Serial.println("IP: " + WiFi.softAPIP().toString());
  Serial.println("Warming up baseline — " + String(WARMUP_SAMPLES) + " samples needed...");
  Serial.println("time,raw,delta,vibration,temp,state,motor_state,fault_score,fault_flags");
}

// ─────────────────────────────────────────────
//  Loop
// ─────────────────────────────────────────────
void loop() {

  // ── Data client ──
  if (!dataClient || !dataClient.connected()) {
    dataClient = dataServer.available();
  }

  // ── Web control ──
  WiFiClient client = server.available();
  if (client) {
    String request = client.readStringUntil('\r');
    if (request.indexOf("/m") != -1) command = "moving";
    if (request.indexOf("/b") != -1) command = "blocked";
    if (request.indexOf("/n") != -1) command = "normal";

    client.println("HTTP/1.1 200 OK");
    client.println("Content-type:text/html\n");
    client.println("<html><body>");
    client.println("<h2>Robot Control</h2>");
    client.println("<p>State: <b>" + stateToString(motorState) + "</b></p>");
    client.println("<a href='/m'><button>Moving</button></a> ");
    client.println("<a href='/b'><button>Blocked</button></a> ");
    client.println("<a href='/n'><button>Normal</button></a>");
    client.println("</body></html>");
    client.stop();
  }

  // ─────────────────────────────────────────
  //  Sensor Readings
  // ─────────────────────────────────────────
  int raw = analogRead(ACS712_PIN);

  sensors_event_t a, g, temp_mpu;
  mpu.getEvent(&a, &g, &temp_mpu);

  float accel_mag = sqrt(
    a.acceleration.x * a.acceleration.x +
    a.acceleration.y * a.acceleration.y +
    a.acceleration.z * a.acceleration.z
  );
  float vibration = fabs(accel_mag - 9.8f);

  float temperature = dht.readTemperature();
  if (isnan(temperature) || temperature < 10.0f) temperature = 30.0f;
  temperature += 2.0f;  // calibration offset

  int delta = raw - prevRaw;
  prevRaw   = raw;

  sampleCount++;

  // ─────────────────────────────────────────
  //  Feed Rolling Stats (always, every sample)
  // ─────────────────────────────────────────
  statsRaw.push((float)raw);
  statsVib.push(vibration);
  statsTemp.push(temperature);

  // ─────────────────────────────────────────
  //  Fault Analysis (only after warmup)
  // ─────────────────────────────────────────
  uint8_t faultFlags = FAULT_NONE;
  float   faultScore = 0.0f;
  String  faultStr   = "NONE";

  if (statsRaw.ready()) {
    if (motorState == STATE_WARMUP) {
      motorState = STATE_NORMAL;
      Serial.println("[INFO] Warmup complete. Adaptive baseline established.");
      Serial.println("[INFO] raw_mean=" + String(statsRaw.mean, 1)
                   + " vib_mean=" + String(statsVib.mean, 3));
    }

    faultScore = analyzeFault((float)raw, vibration, temperature, delta, faultFlags);
    faultStr   = faultCodeToString(faultFlags);

    // State machine transition
    MotorState newState = updateStateMachine(faultScore, faultFlags);
    motorState = newState;

    // Take action based on state
    if (motorState == STATE_WARNING || motorState == STATE_FAULT || motorState == STATE_CRITICAL) {
      handleFaultAction(motorState, faultFlags, faultScore);
    } else if (command == "moving" || command == "blocked") {
      moveForward();  // back to normal operation
    }

  } else {
    // Still in warmup — just run motors normally
    if (command == "moving" || command == "blocked") moveForward();
    else stopMotor();
  }

  // ─────────────────────────────────────────
  //  Output CSV Line
  //  Format: time,raw,delta,vibration,temp,command,motor_state,score,faults
  // ─────────────────────────────────────────
  String data = String(millis())      + ","
              + String(raw)           + ","
              + String(delta)         + ","
              + String(vibration, 3)  + ","
              + String(temperature, 1)+ ","
              + command               + ","
              + stateToString(motorState) + ","
              + String(faultScore, 2) + ","
              + faultStr;

  Serial.println(data);
  if (dataClient && dataClient.connected()) {
    dataClient.println(data);
  }

  delay(200);
}
