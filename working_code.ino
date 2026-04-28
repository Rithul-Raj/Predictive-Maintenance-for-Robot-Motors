/*
 * ================================================================
 *  PREDICTIVE MAINTENANCE — WEB CONTROLLED ROBOT  v4
 *  Fixes:
 *   1. Vibration smoothed over 5-sample average — no bump false alerts
 *   2. Warmup progresses without needing motor movement
 *   3. Motors respond to commands during warmup
 *   4. Persist counts raised — single spike can't trigger fault
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
//  Config
// ─────────────────────────────────────────────
#define WARMUP_SAMPLES        60
#define AUTO_WARMUP_INTERVAL  300000UL

#define WARNING_PERSIST       6
#define FAULT_PERSIST         4
#define RECOVERY_PERSIST      12

#define VIB_SMOOTH_N          5    // average over 5 samples before scoring

// Z-score thresholds
#define Z_RAW_WARN    2.5f
#define Z_VIB_WARN    3.2f         // raised — bumps give single spike, not sustained
#define Z_TEMP_WARN   2.0f

// Score thresholds
#define SCORE_WARN     3.5f
#define SCORE_FAULT    6.0f
#define SCORE_CRITICAL 8.5f

// Hard absolute limits
#define HARD_VIB_LIMIT    18.0f
#define HARD_TEMP_LIMIT   70.0f
#define HARD_DELTA_LIMIT  600

// ─────────────────────────────────────────────
//  Fault Flags
// ─────────────────────────────────────────────
#define FAULT_NONE          0x00
#define FAULT_OVERCURRENT   0x01
#define FAULT_LOWCURRENT    0x02   // current below baseline
#define FAULT_VIBRATION     0x04
#define FAULT_OVERTEMP      0x08   // temp alert only — not in multi-sensor unless >50C
#define FAULT_SUDDEN_SPIKE  0x10
#define FAULT_MULTI_SENSOR  0x20

// Temp threshold to include in multi-sensor fault (genuine danger)
#define HARD_TEMP_CRITICAL  50.0f

// ─────────────────────────────────────────────
//  Frozen Baseline
// ─────────────────────────────────────────────
struct FrozenBaseline {
  float mean=0, stdev=1;
  bool  valid=false;
  float _sum=0, _sumSq=0;
  int   _count=0;

  void reset() { _sum=0; _sumSq=0; _count=0; valid=false; mean=0; stdev=1; }

  bool feed(float v) {
    _sum+=v; _sumSq+=v*v; _count++;
    if (_count >= WARMUP_SAMPLES) {
      mean  = _sum/_count;
      float var = (_sumSq/_count)-(mean*mean);
      stdev = (var>0) ? sqrtf(var) : 0.5f;
      valid = true; return true;
    }
    return false;
  }

  float zScore(float v)       { return valid ? fabsf(v-mean)/stdev : 0; }
  float zScoreSigned(float v) { return valid ? (v-mean)/stdev : 0; }   // + = above base, - = below base
  int   progress()            { return min(100,(_count*100)/WARMUP_SAMPLES); }
};

FrozenBaseline baseRaw, baseVib, baseTemp;

// ─────────────────────────────────────────────
//  Vibration Smoother — 5-sample moving average
//  Single bumps become small blips, sustained
//  vibration stays elevated → reliable detection
// ─────────────────────────────────────────────
struct VibSmoother {
  float buf[VIB_SMOOTH_N]={0};
  int idx=0, count=0;
  float sum=0;
  float push(float v) {
    if (count==VIB_SMOOTH_N) sum-=buf[idx]; else count++;
    buf[idx]=v; idx=(idx+1)%VIB_SMOOTH_N; sum+=v;
    return sum/count;
  }
} vibSmoother;

// ─────────────────────────────────────────────
//  State Machine
// ─────────────────────────────────────────────
enum MotorState { STATE_WARMUP, STATE_NORMAL, STATE_WARNING, STATE_FAULT, STATE_CRITICAL, STATE_STOPPED };
MotorState motorState = STATE_WARMUP;
int warnCount=0, faultCount=0, recoveryCount=0, prevRaw=0;

// ─────────────────────────────────────────────
//  Warmup
// ─────────────────────────────────────────────
bool          inWarmup      = true;
bool          warmupBlocked = false;
unsigned long lastAutoWarmup= 0;

// ─────────────────────────────────────────────
//  Robot control
// ─────────────────────────────────────────────
String moveCmd="stop", lastMoveCmd="stop";

// Per-wheel speed (0-255). Adjust spdA/spdB to fix straight-line drift.
// ENA = left wheel (IN1/IN2), ENB = right wheel (IN3/IN4)
int spdA = 180;   // left wheel  — increase if robot drifts left
int spdB = 180;   // right wheel — increase if robot drifts right

// ─────────────────────────────────────────────
//  Dashboard cache
// ─────────────────────────────────────────────
float   lastRaw=0, lastVib=0, lastVibSmooth=0, lastTemp=0, lastHumidity=0, lastScore=0;
uint8_t lastFaultFlags=FAULT_NONE;

// DHT11 must not be read faster than every 2 seconds
unsigned long lastDHTread = 0;

// ─────────────────────────────────────────────
//  Helpers
// ─────────────────────────────────────────────
String stateToStr(MotorState s) {
  switch(s){case STATE_WARMUP:return "WARMUP";case STATE_NORMAL:return "NORMAL";
  case STATE_WARNING:return "WARNING";case STATE_FAULT:return "FAULT";
  case STATE_CRITICAL:return "CRITICAL";case STATE_STOPPED:return "STOPPED";default:return "UNKNOWN";}
}
String faultToStr(uint8_t f) {
  if(!f) return "NONE"; String s="";
  if(f&FAULT_OVERCURRENT)  s+="OVERCURRENT ";
  if(f&FAULT_LOWCURRENT)   s+="LOW CURRENT ";
  if(f&FAULT_VIBRATION)    s+="VIBRATION ";
  if(f&FAULT_SUDDEN_SPIKE) s+="SUDDEN SPIKE ";
  if(f&FAULT_MULTI_SENSOR) s+="MULTI-SENSOR ";
  if(s.length()==0) return "NONE";
  s.trim(); return s;
}

// ─────────────────────────────────────────────
//  Motor Control
// ─────────────────────────────────────────────
// scale: 1.0 = full user speed, 0.5 = fault slow mode
void motorForward(float scale=1.0)  {digitalWrite(IN1,LOW);digitalWrite(IN2,HIGH);digitalWrite(IN3,LOW);digitalWrite(IN4,HIGH);analogWrite(ENA,spdA*scale);analogWrite(ENB,spdB*scale);}
void motorBackward(float scale=1.0) {digitalWrite(IN1,HIGH);digitalWrite(IN2,LOW);digitalWrite(IN3,HIGH);digitalWrite(IN4,LOW);analogWrite(ENA,spdA*scale);analogWrite(ENB,spdB*scale);}
void motorLeft(float scale=1.0)     {digitalWrite(IN1,HIGH);digitalWrite(IN2,LOW);digitalWrite(IN3,LOW);digitalWrite(IN4,HIGH);analogWrite(ENA,spdA*scale);analogWrite(ENB,spdB*scale);}
void motorRight(float scale=1.0)    {digitalWrite(IN1,LOW);digitalWrite(IN2,HIGH);digitalWrite(IN3,HIGH);digitalWrite(IN4,LOW);analogWrite(ENA,spdA*scale);analogWrite(ENB,spdB*scale);}
void motorStop()                    {analogWrite(ENA,0);analogWrite(ENB,0);}
void motorEmergencyStop()           {analogWrite(ENA,0);analogWrite(ENB,0);digitalWrite(IN1,LOW);digitalWrite(IN2,LOW);digitalWrite(IN3,LOW);digitalWrite(IN4,LOW);}

void driveByCmd(String cmd, float scale=1.0) {
  if(cmd=="forward")       motorForward(scale);
  else if(cmd=="backward") motorBackward(scale);
  else if(cmd=="left")     motorLeft(scale);
  else if(cmd=="right")    motorRight(scale);
  else                     motorStop();
}

void applyMovement() {
  if(motorState==STATE_CRITICAL) {motorEmergencyStop();return;}
  if(motorState==STATE_STOPPED)  {motorStop();return;}
  // Warmup: allow full movement so baseline reflects real operating condition
  if(motorState==STATE_WARMUP)   {driveByCmd(moveCmd,1.0);return;}
  // Soft fault: auto-slow to 50%
  if(motorState==STATE_WARNING||motorState==STATE_FAULT) {driveByCmd(moveCmd,0.5);return;}
  driveByCmd(moveCmd,1.0);
}

// ─────────────────────────────────────────────
//  Fault Analysis
//  - Current: directional — high = OVERCURRENT, low = LOWCURRENT
//  - Vibration: only high matters
//  - Temp/Humidity: alert only — NOT counted in multi-sensor hits or score
//                   EXCEPT temp > 50C is included in multi-sensor (real danger)
//  - Delta: sudden spike regardless of direction
// ─────────────────────────────────────────────
float analyzeFault(float raw, float vibS, float temp, int delta, uint8_t &flags) {
  flags = FAULT_NONE;

  float zRsigned = baseRaw.zScoreSigned(raw);   // + = high current, - = low current
  float zRabs    = fabsf(zRsigned);
  float zV       = baseVib.zScore(vibS);         // vibration — only high matters

  uint8_t hits = 0;  // only motor-related sensors count toward multi-sensor

  // ── Current — directional ──
  if (zRsigned >= Z_RAW_WARN)        { flags |= FAULT_OVERCURRENT; hits++; }
  else if (zRsigned <= -Z_RAW_WARN)  { flags |= FAULT_LOWCURRENT;  hits++; }

  // ── Vibration — only high ──
  if (zV >= Z_VIB_WARN || vibS > HARD_VIB_LIMIT) { flags |= FAULT_VIBRATION; hits++; }

  // ── Sudden delta spike ──
  if (abs(delta) > HARD_DELTA_LIMIT) { flags |= FAULT_SUDDEN_SPIKE; hits++; }

  // ── Temperature — info alert only, NOT in motor fault score ──
  // Exception: temp > 50C is real motor danger → included in multi-sensor
  #define TEMP_ALERT_WARN 38.0f   // show info alert above this (early warning)
  if (temp > HARD_TEMP_CRITICAL) {
    flags |= FAULT_OVERTEMP;
    hits++;   // only this case counts toward multi-sensor
  } else if (temp > TEMP_ALERT_WARN) {
    flags |= FAULT_OVERTEMP;  // display only — no score, no hits
  }

  // ── Multi-sensor: 2+ motor sensors triggered ──
  if (hits >= 2) flags |= FAULT_MULTI_SENSOR;

  // ── Weighted score — temp NOT included ──
  float score = (zRabs * 0.50f) + (zV * 0.50f);
  if (flags & FAULT_MULTI_SENSOR) score *= 1.3f;
  if (flags & FAULT_SUDDEN_SPIKE) score += 2.0f;
  return score;
}

// ─────────────────────────────────────────────
//  State Machine
// ─────────────────────────────────────────────
void updateState(float score, uint8_t flags) {
  if(motorState==STATE_STOPPED) return;
  if(score>=SCORE_CRITICAL||(flags&FAULT_MULTI_SENSOR&&score>=SCORE_FAULT)){
    faultCount++;warnCount=0;recoveryCount=0;
    if(faultCount>=FAULT_PERSIST) motorState=STATE_CRITICAL;
  } else if(score>=SCORE_FAULT){
    faultCount++;warnCount=0;recoveryCount=0;
    if(faultCount>=FAULT_PERSIST) motorState=STATE_FAULT;
  } else if(score>=SCORE_WARN){
    warnCount++;faultCount=0;recoveryCount=0;
    if(warnCount>=WARNING_PERSIST) motorState=STATE_WARNING;
  } else {
    recoveryCount++;faultCount=0;warnCount=0;
    if(recoveryCount>=RECOVERY_PERSIST){
      recoveryCount=0;motorState=STATE_NORMAL;
      lastFaultFlags=FAULT_NONE;lastScore=0;
    }
  }
}

void startWarmup() {
  baseRaw.reset();baseVib.reset();baseTemp.reset();
  inWarmup=true; motorState=STATE_WARMUP;
  warnCount=faultCount=recoveryCount=0;
  lastFaultFlags=FAULT_NONE; lastScore=0;
  Serial.println("[WARMUP] Started...");
}

// ─────────────────────────────────────────────
//  Dashboard
// ─────────────────────────────────────────────
String buildDashboard() {
  String sc="#00ff88";
  if(motorState==STATE_WARNING)  sc="#ffaa00";
  if(motorState==STATE_FAULT)    sc="#ff6600";
  if(motorState==STATE_CRITICAL) sc="#ff2222";
  if(motorState==STATE_WARMUP)   sc="#4499ff";
  if(motorState==STATE_STOPPED)  sc="#888888";

  bool isHard=(motorState==STATE_CRITICAL);
  bool isSoft=(motorState==STATE_WARNING||motorState==STATE_FAULT);
  bool isStopped=(motorState==STATE_STOPPED);

  String alertBox="";
  if(isHard) {
    alertBox="<div class='alert hard'><div class='ai'>&#9888;</div>"
             "<b>CRITICAL FAULT — Emergency Stopped</b><br>"
             "<small>"+faultToStr(lastFaultFlags)+"</small><br>"
             "<small>Fix the issue, then re-warmup to resume.</small><br>"
             "<a href='/warmup'><button class='btn-blue' style='margin-top:8px'>Re-Warmup After Fix</button></a></div>";
  } else if(isStopped) {
    alertBox="<div class='alert info'><div class='ai'>&#9632;</div>"
             "<b>Motors Stopped by User</b><br>"
             "<small>Robot is powered. Press Start to resume.</small><br>"
             "<a href='/start'><button class='btn-green' style='margin-top:8px'>&#9654; Start Robot</button></a></div>";
  } else if(isSoft) {
    alertBox="<div class='alert soft'><div class='ai'>&#9888;</div>"
             "<b>FAULT DETECTED</b> — Motors auto-slowed to 50%<br>"
             "<small style='display:block;margin:3px 0'>"+faultToStr(lastFaultFlags)+"</small>"
             "<small>Score: <b>"+String(lastScore,2)+"</b> &nbsp;|&nbsp; Alert stays until fault clears.</small><br>"
             "<div style='margin-top:10px'>"
             "<a href='/action/turnoff'><button class='btn-red'>&#9646;&#9646; Turn Off Motors</button></a>"
             "<span style='display:inline-block;width:6px'></span>"
             "<a href='/action/slowdown'><button class='btn-yellow'>&#9660; Keep Slow (50%)</button></a>"
             "</div></div>";
  }

  String warnMsg="";
  if(warmupBlocked) {
    warnMsg="<div class='alert soft' style='font-size:0.72rem;padding:8px;margin-bottom:8px'>"
            "&#9888; Cannot warmup during fault. Clear fault first.</div>";
    warmupBlocked=false;
  }

  unsigned long rem=0;
  if(!inWarmup&&motorState==STATE_NORMAL){
    unsigned long el=millis()-lastAutoWarmup;
    rem=(el<AUTO_WARMUP_INTERVAL)?(AUTO_WARMUP_INTERVAL-el)/1000:0;
  }

  String page=R"(<!DOCTYPE html>
<html><head>
<meta charset='UTF-8'>
<meta name='viewport' content='width=device-width,initial-scale=1'>
<meta http-equiv='refresh' content='2'>
<title>Robot Maintenance</title>
<style>
@import url('https://fonts.googleapis.com/css2?family=Share+Tech+Mono&family=Exo+2:wght@400;600;700&display=swap');
*{box-sizing:border-box;margin:0;padding:0}
body{background:#090d14;color:#b8ccd8;font-family:'Exo 2',sans-serif;height:100vh;overflow:hidden;padding:14px;max-width:460px;margin:auto}
h1{font-family:'Share Tech Mono',monospace;color:#00ccff;font-size:1.1rem;letter-spacing:3px;text-align:center;margin-bottom:12px;padding-bottom:10px;border-bottom:1px solid #1a2a3a}
.sb{display:flex;justify-content:space-between;align-items:center;background:#0f1820;border:1px solid #1e3a5f;border-radius:8px;padding:10px 14px;margin-bottom:10px}
.badge{font-family:'Share Tech Mono',monospace;font-size:0.88rem;font-weight:bold;padding:5px 12px;border-radius:4px;border:2px solid )"+sc+R"(;color:)"+sc+R"(}
.grid{display:grid;grid-template-columns:1fr 1fr 1fr 1fr;gap:6px;margin-bottom:10px}
.card{background:#0f1820;border:1px solid #1e3a5f;border-radius:8px;padding:10px;text-align:center}
.card .val{font-family:'Share Tech Mono',monospace;font-size:1.1rem;color:#00ccff;margin:3px 0}
.card .lbl{font-size:0.6rem;color:#6688aa;text-transform:uppercase;letter-spacing:1px}
.card .base{font-size:0.58rem;color:#3a5a7a}
.alert{border-radius:8px;padding:12px 14px;margin-bottom:10px;text-align:center;line-height:1.9}
.ai{font-size:1.4rem;margin-bottom:3px}
.alert.hard{background:#1a0505;border:2px solid #ff3333;color:#ff9999}
.alert.soft{background:#1a1200;border:2px solid #ffaa00;color:#ffcc55}
.alert.info{background:#0a1520;border:2px solid #3399ff;color:#88ccff}
.ctrl{background:#0f1820;border:1px solid #1e3a5f;border-radius:8px;padding:12px;margin-bottom:10px}
.ct{font-size:0.6rem;color:#446688;text-transform:uppercase;letter-spacing:2px;text-align:center;margin-bottom:10px}
.dpad{display:grid;grid-template-columns:1fr 1fr 1fr;gap:6px;max-width:186px;margin:0 auto}
.btn{background:#111e2e;border:1px solid #1e3a5f;color:#b8ccd8;padding:15px;border-radius:6px;font-size:1.2rem;text-decoration:none;display:block;text-align:center}
.btn:hover{background:#1a2e44}
.btn.sb2{background:#1a0f0f;border-color:#5a2020;color:#ff8888}
.btn-red{background:#4a1010;border:none;color:#ffaaaa;padding:9px 15px;border-radius:6px;font-size:0.78rem;cursor:pointer;margin:2px;font-family:'Exo 2',sans-serif}
.btn-yellow{background:#2e1e00;border:none;color:#ffcc44;padding:9px 15px;border-radius:6px;font-size:0.78rem;cursor:pointer;margin:2px;font-family:'Exo 2',sans-serif}
.btn-green{background:#0a2a14;border:2px solid #00aa44;color:#44ff88;padding:11px 24px;border-radius:6px;font-size:0.92rem;cursor:pointer;font-family:'Exo 2',sans-serif;font-weight:600}
.btn-blue{background:#0a1a2e;border:2px solid #0066cc;color:#66aaff;padding:8px 15px;border-radius:6px;font-size:0.78rem;cursor:pointer;font-family:'Exo 2',sans-serif}
.btn-spd{background:#111e2e;border:1px solid #1e3a5f;color:#b8ccd8;padding:6px 18px;border-radius:5px;font-size:0.85rem;cursor:pointer;margin:2px}
.ws{background:#0f1820;border:1px solid #1e3a5f;border-radius:8px;padding:12px;margin-bottom:10px;text-align:center}
.wb{background:#0a1520;border-radius:4px;height:8px;margin:8px 0 4px}
.wf{background:linear-gradient(90deg,#0066cc,#00ccff);height:8px;border-radius:4px}
.ft{text-align:center;margin-bottom:8px}
.tag{display:inline-block;background:#1a0a2e;border:1px solid #5533aa;color:#aa88ff;font-size:0.6rem;padding:2px 8px;border-radius:12px;margin:2px;font-family:'Share Tech Mono',monospace}
.footer{text-align:center;font-size:0.56rem;color:#2a3a4a;margin-top:8px}
</style></head><body>
<h1>&#9881; ROBOT MAINTENANCE</h1>
<div class='sb'>
  <div><div style='font-size:0.56rem;color:#446688;margin-bottom:3px'>MOTOR STATE</div>
  <div class='badge'>)"+stateToStr(motorState)+R"(</div></div>
  <div style='text-align:right'><div style='font-size:0.56rem;color:#446688'>FAULT SCORE</div>
  <div style='font-family:Share Tech Mono,monospace;font-size:1.2rem;color:#ffcc44'>)"+String(lastScore,2)+R"(</div></div>
</div>
)";
  page+="<div class='ws'>";
  if(inWarmup){
    int pct=baseRaw.progress();
    page+="<div style='font-size:0.75rem;color:#4499ff;margin-bottom:6px'>&#9654; Collecting baseline... <b>"+String(pct)+"%</b></div>"
          "<div class='wb'><div class='wf' style='width:"+String(pct)+"%'></div></div>"
          "<div style='font-size:0.6rem;color:#446688;margin-top:6px'>Run robot normally during warmup. Motors are active.</div>";
  } else {
    if(motorState==STATE_NORMAL)
      page+="<div style='font-size:0.6rem;color:#446688;margin-bottom:6px'>Auto re-warmup in <b>"+String(rem)+"s</b> (NORMAL state only)</div>";
    page+="<div style='font-size:0.66rem;color:#446688;margin-bottom:8px'>&#9432; Warmup only in normal condition — not during faults.</div>"
          "<a href='/warmup'><button class='btn-blue'>&#9664; Manual Re-Warmup</button></a>";
  }
  page+="</div>";
  page+=R"(<div class='grid'>
  <div class='card'><div class='lbl'>Current</div><div class='val'>)"+String((int)lastRaw)+R"(</div><div class='base'>base )"+String(baseRaw.mean,0)+R"(</div></div>
  <div class='card'><div class='lbl'>Vibration</div><div class='val'>)"+String(lastVib,2)+R"(</div><div class='base'>avg )"+String(lastVibSmooth,2)+R"( | base )"+String(baseVib.mean,2)+R"(</div></div>
  <div class='card'><div class='lbl'>Temp &deg;C</div><div class='val'>)"+String(lastTemp,1)+R"(</div><div class='base'>base )"+String(baseTemp.mean,1)+R"(</div></div>
  <div class='card'><div class='lbl'>Humidity %</div><div class='val'>)"+String(lastHumidity,1)+R"(</div><div class='base'>DHT11</div></div>
</div>
)"+alertBox+warnMsg+R"(
<div class='ctrl'><div class='ct'>Direction Control</div>
  <div class='dpad'>
    <span></span><a href='/cmd/forward' class='btn'>&#8679;</a><span></span>
    <a href='/cmd/left' class='btn'>&#8678;</a>
    <a href='/cmd/stop' class='btn sb2'>&#9632;</a>
    <a href='/cmd/right' class='btn'>&#8680;</a>
    <span></span><a href='/cmd/backward' class='btn'>&#8681;</a><span></span>
  </div>
</div>
<div class='ctrl'>
  <div class='ct'>Wheel Speed (0-255)</div>
  <div style='display:grid;grid-template-columns:1fr 1fr;gap:10px;margin-top:4px'>
    <div style='text-align:center'>
      <div style='font-size:0.62rem;color:#6688aa;margin-bottom:4px'>LEFT WHEEL (ENA)</div>
      <div style='font-family:Share Tech Mono,monospace;font-size:1.1rem;color:#00ccff;margin-bottom:6px'>)"+String(spdA)+R"(</div>
      <a href='/spd/a/up'><button class='btn-spd'>&#9650;</button></a>
      <a href='/spd/a/dn'><button class='btn-spd'>&#9660;</button></a>
    </div>
    <div style='text-align:center'>
      <div style='font-size:0.62rem;color:#6688aa;margin-bottom:4px'>RIGHT WHEEL (ENB)</div>
      <div style='font-family:Share Tech Mono,monospace;font-size:1.1rem;color:#00ccff;margin-bottom:6px'>)"+String(spdB)+R"(</div>
      <a href='/spd/b/up'><button class='btn-spd'>&#9650;</button></a>
      <a href='/spd/b/dn'><button class='btn-spd'>&#9660;</button></a>
    </div>
  </div>
  <div style='font-size:0.58rem;color:#335566;text-align:center;margin-top:8px'>
    Each press &#177;10. If robot drifts left, increase LEFT or decrease RIGHT.
  </div>
</div>
)";

  if(lastFaultFlags!=FAULT_NONE){
    page+="<div class='ft'>";
    if(lastFaultFlags&FAULT_OVERCURRENT)  page+="<span class='tag'>OVERCURRENT</span>";
    if(lastFaultFlags&FAULT_LOWCURRENT)   page+="<span class='tag' style='border-color:#3377ff;color:#88aaff'>LOW CURRENT</span>";
    if(lastFaultFlags&FAULT_VIBRATION)    page+="<span class='tag'>VIBRATION</span>";
    if(lastFaultFlags&FAULT_SUDDEN_SPIKE) page+="<span class='tag'>SUDDEN SPIKE</span>";
    if(lastFaultFlags&FAULT_MULTI_SENSOR) page+="<span class='tag' style='border-color:#ff3333;color:#ff8888'>MULTI-SENSOR</span>";
    // FAULT_OVERTEMP intentionally excluded — shown in separate temp info alert below
    page+="</div>";
  }

  // Temp/Humidity — separate info alert, independent of motor fault state
  if(lastFaultFlags&FAULT_OVERTEMP){
    String tc = (lastTemp>HARD_TEMP_CRITICAL) ? "#ff4444" : "#ffaa00";
    page+="<div style='background:#180e00;border:1px solid "+tc+";border-radius:8px;"
          "padding:8px 12px;margin-bottom:10px;font-size:0.75rem;color:"+tc+";text-align:center'>"
          "&#9651; TEMP ALERT: "+String(lastTemp,1)+"&deg;C";
    if(lastTemp>HARD_TEMP_CRITICAL) page+=" &nbsp;|&nbsp; <b>Above 50C — included in fault</b>";
    page+="</div>";
  }
  if(lastHumidity>90.0f){
    page+="<div style='background:#000f1a;border:1px solid #0099cc;border-radius:8px;"
          "padding:8px 12px;margin-bottom:10px;font-size:0.75rem;color:#44bbdd;text-align:center'>"
          "&#9650; HUMIDITY: "+String(lastHumidity,1)+"% — Very High</div>";
  } else if(lastHumidity>0.0f){
    page+="<div style='background:#000a10;border:1px solid #1a4a5a;border-radius:8px;"
          "padding:6px 12px;margin-bottom:10px;font-size:0.72rem;color:#336677;text-align:center'>"
          "&#9673; Humidity: "+String(lastHumidity,1)+"%</div>";
  }


  page+="<div class='footer'>Auto-refresh 2s &nbsp;|&nbsp; "+WiFi.softAPIP().toString()+"</div>";
  page+="</body></html>";
  return page;
}

// ─────────────────────────────────────────────
//  Setup
// ─────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  pinMode(IN1,OUTPUT);pinMode(IN2,OUTPUT);pinMode(ENA,OUTPUT);
  pinMode(IN3,OUTPUT);pinMode(IN4,OUTPUT);pinMode(ENB,OUTPUT);
  dht.begin();
  if(!mpu.begin()){Serial.println("MPU6050 not found!");while(1)delay(500);}
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  WiFi.softAP(ssid,password);
  server.begin();
  lastAutoWarmup=millis();
  startWarmup();
  Serial.println("===== ROBOT MAINTENANCE v4 =====");
  Serial.println("Dashboard: http://"+WiFi.softAPIP().toString());
}

// ─────────────────────────────────────────────
//  Loop
// ─────────────────────────────────────────────
void loop() {

  // ── Web — always handled first, never skipped ──
  WiFiClient client=server.available();
  if(client){
    String req=client.readStringUntil('\r');
    client.flush();

    if(req.indexOf("/cmd/forward") !=-1){moveCmd="forward"; lastMoveCmd=moveCmd;}
    if(req.indexOf("/cmd/backward")!=-1){moveCmd="backward";lastMoveCmd=moveCmd;}
    if(req.indexOf("/cmd/left")    !=-1){moveCmd="left";    lastMoveCmd=moveCmd;}
    if(req.indexOf("/cmd/right")   !=-1){moveCmd="right";   lastMoveCmd=moveCmd;}
    if(req.indexOf("/cmd/stop")    !=-1){moveCmd="stop";}

    if(req.indexOf("/action/turnoff")!=-1){
      motorState=STATE_STOPPED; motorEmergencyStop();
    }

    if(req.indexOf("/start")!=-1&&motorState==STATE_STOPPED){
      if(lastScore<SCORE_CRITICAL){
        motorState=STATE_NORMAL; moveCmd=lastMoveCmd;
        warnCount=faultCount=recoveryCount=0;
      }
    }

    if(req.indexOf("/spd/a/up")!=-1){ spdA=min(255,spdA+10); }
    if(req.indexOf("/spd/a/dn")!=-1){ spdA=max(0,  spdA-10); }
    if(req.indexOf("/spd/b/up")!=-1){ spdB=min(255,spdB+10); }
    if(req.indexOf("/spd/b/dn")!=-1){ spdB=max(0,  spdB-10); }

    if(req.indexOf("/warmup")!=-1){
      if(motorState==STATE_WARNING||motorState==STATE_FAULT||motorState==STATE_CRITICAL){
        warmupBlocked=true;
        Serial.println("[WARMUP] Blocked — fault active");
      } else {
        startWarmup(); lastAutoWarmup=millis();
      }
    }

    String html=buildDashboard();
    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: text/html");
    client.println("Connection: close");
    client.println("Content-Length: "+String(html.length()));
    client.println();
    client.print(html);
    client.stop();
  }

  // ── Sensor Read ──
  int raw=analogRead(ACS712_PIN);

  sensors_event_t a,g,tm;
  mpu.getEvent(&a,&g,&tm);
  float accel=sqrtf(a.acceleration.x*a.acceleration.x+
                    a.acceleration.y*a.acceleration.y+
                    a.acceleration.z*a.acceleration.z);
  float vibRaw   =fabsf(accel-9.8f);
  float vibSmooth=vibSmoother.push(vibRaw);  // 5-sample average

  // DHT11 max poll rate is 1 read per 2 seconds — reading faster returns stale values
  // Calibration offsets — sensor reads low, add offset to get actual value:
  //   Temp:     sensor ~2-3°C raw  → actual ~30°C  → TEMP_OFFSET = +28
  //   Humidity: sensor ~8-9% raw   → actual ~65%   → HUMIDITY_OFFSET = +60
  const float TEMP_OFFSET     = 28.0f;  // tweak: actual_temp - sensor_raw
  const float HUMIDITY_OFFSET = 60.0f;  // tweak: actual_humidity - sensor_raw

  if (millis() - lastDHTread >= 2000) {
    float t = dht.readTemperature();
    float h = dht.readHumidity();
    if (!isnan(t) && t > 0.0f && t < 80.0f) lastTemp     = t + TEMP_OFFSET;
    if (!isnan(h) && h > 0.0f && h < 100.0f) lastHumidity = constrain(h + HUMIDITY_OFFSET, 0.0f, 100.0f);
    lastDHTread = millis();
  }
  // Note: lastTemp and lastHumidity hold previous valid reading if DHT returns NaN

  int delta=raw-prevRaw;
  prevRaw=raw;

  lastRaw=raw; lastVib=vibRaw; lastVibSmooth=vibSmooth;

  // ── Warmup — NO return, motors still active below ──
  if(inWarmup){
    bool done=baseRaw.feed((float)raw);
    baseVib.feed(vibSmooth);   // baseline uses smoothed vibration
    baseTemp.feed(lastTemp);
    if(done){
      inWarmup=false; motorState=STATE_NORMAL; lastAutoWarmup=millis();
      Serial.println("[WARMUP] Complete. raw="+String(baseRaw.mean,1)
                    +" vib="+String(baseVib.mean,3)+" temp="+String(baseTemp.mean,1));
    }
    // Fall through — motors get applied below
  }

  // ── Auto Re-Warmup every 5 min (NORMAL only) ──
  if(!inWarmup&&motorState==STATE_NORMAL&&(millis()-lastAutoWarmup)>=AUTO_WARMUP_INTERVAL){
    Serial.println("[AUTO-WARMUP] 5-min re-warmup");
    startWarmup();
  }

  // ── Fault Analysis ──
  if(!inWarmup&&motorState!=STATE_STOPPED&&motorState!=STATE_WARMUP){
    lastScore=analyzeFault((float)raw,vibSmooth,lastTemp,delta,lastFaultFlags);
    updateState(lastScore,lastFaultFlags);
    if(motorState!=STATE_NORMAL)
      Serial.println("["+stateToStr(motorState)+"] score="+String(lastScore,2)+" | "+faultToStr(lastFaultFlags));
  }

  // ── Motors ──
  applyMovement();

  delay(200);
}
