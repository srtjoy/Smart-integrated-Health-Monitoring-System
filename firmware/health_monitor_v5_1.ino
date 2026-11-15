/*
 * ══════════════════════════════════════════════════════════════════
 *  HEALTH MONITOR v5.1  —  ESP32 DUAL-CORE
 *  All sensors + AD8232 ECG engine + SIM800L SMS
 *  UI: Medical ECG Monitor with SEND SMS button
 *
 *  BUG FIXES vs healthv110_ecg_fixed.ino:
 *  1. buildECGJSON() now skips broadcast when toSend == 0
 *     (avoids spamming empty {"ecg":[]} packets)
 *  2. ecgSentIdx guard: resets to ecgBufIdx on ECG reset so
 *     old stale samples aren't re-sent after resetECG()
 *  3. Web UI: added phone-number input + "📨 SEND SMS" button
 *     that calls /sendSMS?phone=...&msg=... on the ESP32
 *  4. handleSendSMS() server route added — composes vitals and
 *     calls sendSMS() directly from the browser button
 *  5. startBPMeasurement() bpBufIdx reset confirmed present
 *  6. "delay()" in loop() for buzzer off replaced with
 *     millis()-based check (was already fixed, confirmed)
 *  7. checkIncomingSMS() now also handles "send" keyword (alias)
 *
 *  FIX in v5.1 (this file):
 *  ✅ initSIM800L() — boot delay 3000→5000 ms
 *  ✅ AT handshake — retry loop x10 (2s each) instead of single attempt
 *  ✅ ATE0 added after AT OK (echo off → cleaner response parsing)
 *  ✅ AT+IPR=9600 added to lock baud rate (survives module resets)
 *  ✅ All timeouts match the working sim800l.ino test suite
 *
 *  PIN SUMMARY:
 *  ─────────────────────────────────────────────────────
 *  I2C    SDA=21  SCL=22   (LCD 0x27, MAX30100, MPU6050 0x68)
 *  DS18B20       GPIO 15
 *  BP DC         GPIO 36  (ADC1)
 *  BP AC         GPIO 39  (ADC1)
 *  AD8232 OUT    GPIO 32  (ADC1)
 *  AD8232 LO+    GPIO 33
 *  AD8232 LO-    GPIO 27
 *  BUZZER        GPIO 26
 *  DHT22         GPIO 4
 *  SIM800L RX←   GPIO 17  (ESP TX2)
 *  SIM800L TX→   GPIO 16  (ESP RX2)
 * ══════════════════════════════════════════════════════════════════
 */

// ── Libraries ────────────────────────────────────────────────────
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <Wire.h>
#include <DHT.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <LiquidCrystal_I2C.h>
#include "MAX30100_PulseOximeter.h"
#include <WiFiManager.h>
#include <ArduinoJson.h>

// ── LCD ──────────────────────────────────────────────────────────
LiquidCrystal_I2C lcd(0x27, 20, 4);

// ── DS18B20 ──────────────────────────────────────────────────────
#define ONE_WIRE_BUS 15
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature bodySens(&oneWire);

// ── BP ───────────────────────────────────────────────────────────
const int   BP_DC_PIN      = 36;
const int   BP_AC_PIN      = 39;
const float PRESSURE_SCALE = 0.12f;
const int   STOP_PRESSURE  = 40;
const float FILTER_ALPHA   = 0.85f;
const int   BP_SAMPLE_MS   = 20;       // 50 Hz
const float SYS_RATIO      = 0.48f;
const float DIA_RATIO      = 0.72f;
const int   MAX_BP_SAMPLES = 2000;
int         sysOffset      = 0;
int         diaOffset      = 0;

// ── AD8232 ECG ───────────────────────────────────────────────────
#define ECG_PIN        32
#define LO_PLUS_PIN    33
#define LO_MINUS_PIN   27
#define ECG_SAMPLE_HZ  200
#define ECG_DELAY_US   (1000000 / ECG_SAMPLE_HZ)

int   peakBuf[10]      = {0};
int   peakIdx          = 0;
int   peakCount        = 0;
bool  aboveThreshold   = false;
int   ecgCurrentBPM    = 0;
long  lastPeakMs       = 0;
int   ecgRunMean       = 2048;          // adaptive baseline (EMA, int32)

#define ECG_BUF_SIZE   500
int16_t ecgBuf[ECG_BUF_SIZE];
int     ecgBufIdx      = 0;
bool    ecgLeadOff     = false;

unsigned long lastEcgUs = 0;

// ── Environment ───────────────────────────────────────────────────
#define BUZZER_PIN 26
#define DHTPIN     4
#define DHTTYPE    DHT22
DHT dht(DHTPIN, DHTTYPE);

// ── MAX30100 / MPU6050 ───────────────────────────────────────────
PulseOximeter pox;
#define MPU_ADDR 0x68

// ── SIM800L (UART2) ───────────────────────────────────────────────
#define SIM_RX_PIN 16
#define SIM_TX_PIN 17
HardwareSerial sim800(2);

// Default caregiver number — overridable via web UI
String caregiverNumber = "880171XXXXXXX";   // BD example

// Safe-zone thresholds
const int   ALERT_HR_LOW    = 50;
const int   ALERT_HR_HIGH   = 120;
const float ALERT_SPO2_LOW  = 92.0f;
const float ALERT_TEMP_HIGH = 38.0f;
const int   ALERT_SYS_HIGH  = 140;
const int   ALERT_DIA_HIGH  = 90;

unsigned long alertCooldown[5]       = {0, 0, 0, 0, 0};
const unsigned long ALERT_COOLDOWN_MS = 5UL * 60 * 1000;

bool sim800Ready = false;

// ── I2C mutex ────────────────────────────────────────────────────
SemaphoreHandle_t i2cMutex;

// ── Shared vitals ────────────────────────────────────────────────
struct SharedData {
  volatile float bpm            = 0;
  volatile float spo2           = 0;
  volatile unsigned long lastBeatMs = 0;
  volatile bool  hrValid        = false;
  volatile bool  sensorOK       = true;

  float bodyTemp   = 0.0f;
  float roomTemp   = 0.0f;
  float roomHum    = 0.0f;
  float ax = 0, ay = 0, az = 0;
  String position  = "Stable";

  int    sys       = 0;
  int    dia       = 0;
  int    bpPulse   = 0;
  float  bpCuffMmHg = 0;
  float  bpMap     = 0;
  String bpState   = "Idle";
  int    bpCode    = 0;

  String ip = "";
} shared;

// ── BP variables ──────────────────────────────────────────────────
int   bpZero           = 0;
float bpFilteredPulse  = 0;
float bpPrevRaw        = 0;
float bpPrevFiltered   = 0;
float bpPressBuf[MAX_BP_SAMPLES];
float bpAmpBuf[MAX_BP_SAMPLES];
int   bpBufIdx         = 0;
float bpMaxAmp         = 0;
float bpMapPressure    = 0;
float bpLastPressure   = 0;
unsigned long lastBPRead = 0;

// ── Buzzer ────────────────────────────────────────────────────────
volatile bool  buzzerFlag = false;
unsigned long  buzzerOffMs = 0;

// ── BP Trigger GPIO ───────────────────────────────────────────────
#define BP_TRIGGER_PIN   13
unsigned long  bpTriggerOffMs = 0;

// ── Timing ───────────────────────────────────────────────────────
uint32_t lastVitalsSend = 0;
uint32_t lastECGSend    = 0;
uint32_t lastLCDUpdate  = 0;

// ── Network ──────────────────────────────────────────────────────
WiFiManager  wm;
WebServer    server(80);
WebSocketsServer ws(81);

// ── JSON strings ─────────────────────────────────────────────────
String jsonVitals;
String jsonECG;

// ── FreeRTOS task handles ────────────────────────────────────────
TaskHandle_t hCore0, hCore1;

// ── ECG sent-pointer (file scope, not static-in-function) ────────
int ecgSentIdx = 0;


// ══════════════════════════════════════════════════════════════════
//  WEB UI — dark medical theme + SEND SMS + REQUEST UPDATE buttons
// ══════════════════════════════════════════════════════════════════
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Medical ECG Monitor</title>
<style>
:root{
  --bg:#0a0f1a; --card:#111827; --border-g:#00ff88;
  --border-r:#ff3366; --text:#e5e7eb; --dim:#6b7280;
  --green:#00ff88; --red:#ff3366; --yellow:#fbbf24;
  --blue:#38bdf8;
}
*{box-sizing:border-box;margin:0;padding:0}
body{background:var(--bg);color:var(--text);
     font-family:'Segoe UI',Arial,sans-serif;padding:18px}

/* ── Header ── */
.header{text-align:center;margin-bottom:24px}
.header h1{font-size:22px;font-weight:400;letter-spacing:3px;color:var(--green)}
.header h1 strong{color:var(--red)}

/* ── Metric grid ── */
.grid{display:grid;
      grid-template-columns:repeat(auto-fit,minmax(240px,1fr));
      gap:16px;margin-bottom:20px}
.card{background:var(--card);border-radius:12px;padding:18px;
      border-left:4px solid var(--border-g);
      box-shadow:0 4px 20px rgba(0,0,0,.4)}
.card.red-border{border-left-color:var(--border-r)}
.card .label{font-size:11px;text-transform:uppercase;
             letter-spacing:1.5px;color:var(--dim);margin-bottom:8px}
.card .big{font-size:48px;font-weight:700;
           font-family:'Courier New',monospace;line-height:1;margin-bottom:4px}
.card .sub{font-size:13px;color:var(--dim);margin-bottom:4px}
.card .status{font-size:13px;font-weight:600}
.green{color:var(--green)} .red{color:var(--red)}
.yellow{color:var(--yellow)} .blue{color:var(--blue)}

/* ── ECG strip ── */
.ecg-card{background:var(--card);border-radius:12px;padding:16px;
          border-left:4px solid var(--border-r);
          box-shadow:0 4px 20px rgba(0,0,0,.4);margin-bottom:20px}
.ecg-topbar{display:flex;justify-content:space-between;
            align-items:center;margin-bottom:12px}
.ecg-title{font-size:12px;font-weight:700;letter-spacing:1.5px;color:var(--dim)}
.ecg-title .live{color:var(--red);margin-left:8px}
.badge{display:inline-block;padding:3px 10px;border-radius:20px;
       font-size:11px;font-weight:700}
.badge.on {background:#0d3320;color:var(--green)}
.badge.off{background:#3a0d1a;color:var(--red)}
.ecg-meta{font-size:11px;color:var(--dim);margin-left:12px}

.ecg-wrap{background:#080c12;border-radius:8px;padding:8px;position:relative}
canvas{display:block;width:100%;height:260px;cursor:crosshair}

.ecg-footer{display:flex;justify-content:space-between;
            padding:4px 8px 0;font-size:10px;
            font-family:'Courier New',monospace;color:#334155}

.ecg-stats{display:flex;gap:16px;margin-top:12px;flex-wrap:wrap}
.stat-pill{display:flex;align-items:center;gap:8px;
           background:#0d1117;padding:6px 14px;border-radius:20px}
.stat-pill .sk{font-size:10px;color:var(--dim)}
.stat-pill .sv{font-size:15px;font-weight:700;color:var(--green);
               font-family:'Courier New',monospace}
.stat-pill .su{font-size:10px;color:var(--dim)}

/* ── SMS panel ── */
.sms-card{background:var(--card);border-radius:12px;padding:16px;
          border-left:4px solid var(--border-g);
          box-shadow:0 4px 20px rgba(0,0,0,.4);margin-bottom:20px}
.sms-card h3{font-size:12px;font-weight:700;letter-spacing:1.5px;
             color:var(--dim);margin-bottom:12px}
.sms-row{display:flex;gap:10px;align-items:center;flex-wrap:wrap}
.sms-row input{flex:1;min-width:200px;background:#0d1117;color:var(--text);
               border:1px solid #1e3a5f;border-radius:8px;
               padding:9px 14px;font-size:14px;font-family:'Courier New',monospace}
.sms-row input:focus{outline:none;border-color:var(--green)}
.sms-status{margin-top:8px;font-size:12px;font-weight:600;
            min-height:18px;font-family:'Courier New',monospace}

/* ── Buttons ── */
.btn-row{text-align:center;margin-top:14px}
button{background:var(--green);color:#000;border:none;
       padding:11px 28px;border-radius:30px;font-size:14px;
       font-weight:700;cursor:pointer;margin:6px;
       transition:all .18s;box-shadow:0 2px 10px rgba(0,255,136,.25)}
button:hover{background:#00cc66;transform:scale(1.03)}
button:active{transform:scale(0.97)}
button.secondary{background:#1a2a3a;color:var(--green);
                 box-shadow:0 2px 10px rgba(0,255,136,.1)}
button.sms-btn{background:#0e3a1e;color:var(--green);
               box-shadow:0 2px 10px rgba(0,255,136,.2)}
button.upd-btn{background:#1a2a3a;color:var(--blue);
               box-shadow:0 2px 10px rgba(56,189,248,.1)}
button:disabled{opacity:.5;cursor:not-allowed;transform:none}
</style>
</head>
<body>

<!-- Header -->
<div class="header">
  <h1>🛡 <strong>MEDICAL</strong> ECG MONITOR</h1>
</div>

<!-- Metric cards -->
<div class="grid">

  <!-- Heart Rate -->
  <div class="card">
    <div class="label"><span>❤️</span> HEART RATE</div>
    <div class="big green" id="bpm">--</div>
    <div class="sub">BPM | SpO2: <span id="spo2">--</span>%</div>
    <div class="status yellow" id="hrStatus">Place finger on sensor</div>
  </div>

  <!-- Temperature -->
  <div class="card">
    <div class="label"><span>🌡</span> TEMPERATURE</div>
    <div class="big yellow" id="bodyTemp">--</div>
    <div class="sub">Body °C | Room: <span id="roomTemp">--</span>°C</div>
  </div>

  <!-- Blood Pressure -->
  <div class="card red-border">
    <div class="label"><span>🩺</span> BLOOD PRESSURE</div>
    <div class="big red" id="bpVal">--/--</div>
    <div class="sub">mmHg | Pulse: <span id="bpPulse">--</span> BPM</div>
    <div class="status yellow" id="bpState">(Idle)</div>
    <div class="sub" style="margin-top:6px">
      Cuff: <span id="bpCuff">--</span> mmHg | MAP: <span id="bpMap">--</span> mmHg
    </div>
    <div id="bpClass" style="margin-top:8px;font-size:13px;font-weight:700"></div>
  </div>

  <!-- Motion -->
  <div class="card">
    <div class="label"><span>📊</span> MOTION</div>
    <div class="big" id="pos" style="font-size:36px">--</div>
    <div class="sub">X: <span id="ax">0</span> Y: <span id="ay">0</span> Z: <span id="az">0</span></div>
  </div>

  <!-- System -->
  <div class="card">
    <div class="label"><span>📡</span> SYSTEM</div>
    <div class="sub" style="line-height:2">
      RSSI: <span id="rssi">--</span> dBm<br>
      ECG: <span id="ecgStatus">--</span><br>
      IP: <span id="ip">--</span><br>
      Uptime: <span id="uptime">--</span>s
    </div>
  </div>

</div><!-- /grid -->

<!-- ECG Strip -->
<div class="ecg-card">
  <div class="ecg-topbar">
    <div class="ecg-title">
      📈 MEDICAL ECG MONITOR
      <span class="live">● LIVE</span>
    </div>
    <div style="display:flex;align-items:center">
      <span class="badge off" id="leadsBadge">⚡ LEADS OFF</span>
      <span class="ecg-meta">25mm/s | 10mm/mV</span>
    </div>
  </div>

  <div class="ecg-wrap">
    <canvas id="ecgCanvas"></canvas>
  </div>

  <div class="ecg-footer">
    <span>1mV</span><span>1s</span><span>2s</span>
    <span>3s</span><span>4s</span><span>5s</span>
  </div>

  <div class="ecg-stats">
    <div class="stat-pill">
      <span class="sk">HR</span>
      <span class="sv" id="ecgBpm">--</span>
      <span class="su">BPM</span>
    </div>
    <div class="stat-pill">
      <span class="sk">PEAK</span>
      <span class="sv" id="ecgPeak">0</span>
      <span class="su">mV</span>
    </div>
    <div class="stat-pill">
      <span class="sk">BASELINE</span>
      <span class="sv" id="ecgBase">0</span>
    </div>
    <div class="stat-pill">
      <span class="sk">LEADS</span>
      <span class="sv" id="ecgLeads">--</span>
    </div>
  </div>
</div>

<!-- ══ SMS Panel ══ -->
<div class="sms-card">
  <h3>📨 SMS ALERT — SIM800L</h3>
  <div class="sms-row">
    <input type="tel" id="phoneInput" placeholder="+8801XXXXXXXXX"
           oninput="savePhone()" autocomplete="tel">
    <button class="sms-btn" id="smsBtn" onclick="sendSMS()">
      📨 SEND SMS
    </button>
    <button class="upd-btn" id="updBtn" onclick="requestUpdate()">
      📡 REQUEST UPDATE
    </button>
  </div>
  <div class="sms-status" id="smsStatus"></div>
</div>

<!-- Control buttons -->
<div class="btn-row">
  <button onclick="startBP()">🩸 START BP MEASUREMENT</button>
  <button class="secondary" onclick="resetECG()">⟲ RESET ECG</button>
  <button class="secondary" onclick="toggleGrid()">🔲 TOGGLE GRID</button>
</div>

<script>
// ── Persist phone number ──────────────────────────────────────────
(function(){
  const saved = localStorage.getItem('caregiverPhone');
  if (saved) document.getElementById('phoneInput').value = saved;
})();
function savePhone(){
  localStorage.setItem('caregiverPhone', document.getElementById('phoneInput').value);
}

// ── WebSocket ─────────────────────────────────────────────────────
let wsock;
function connectWS(){
  wsock = new WebSocket('ws://'+location.host+':81');
  wsock.onmessage = e => {
    try {
      const d = JSON.parse(e.data);
      if (d.ecg){ handleECG(d.ecg); return; }
      updateVitals(d);
    } catch(_){}
  };
  wsock.onclose = () => setTimeout(connectWS, 2000);
}

// ── ECG engine ───────────────────────────────────────────────────
const canvas = document.getElementById('ecgCanvas');
const ctx    = canvas.getContext('2d');

const ECG_WINDOW = 1000;
const ecgRing   = new Float32Array(ECG_WINDOW);
let   ecgWrite  = 0;
let   ecgFilled = 0;

let peakHist = [], rrList = [], lastPeakT = 0;
let showGrid = true;
const CAL = 0.1;

function handleECG(samples){
  for (let i = 0; i < samples.length; i++) {
    ecgRing[ecgWrite] = samples[i];
    ecgWrite = (ecgWrite + 1) % ECG_WINDOW;
    if (ecgFilled < ECG_WINDOW) ecgFilled++;
  }
  detectPeaks(samples);
  draw();
}

function detectPeaks(samples){
  let sum = 0;
  for (let v of samples) sum += v;
  const mean   = samples.length ? sum / samples.length : 2048;
  const thresh = mean + (4095 - mean) * 0.55;

  for (let i = 1; i < samples.length - 1; i++) {
    if (samples[i] > thresh &&
        samples[i] > samples[i-1] &&
        samples[i] > samples[i+1]) {
      const now = Date.now();
      if (lastPeakT > 0) {
        const rr = now - lastPeakT;
        if (rr > 300 && rr < 2000) {
          rrList.push(rr);
          if (rrList.length > 10) rrList.shift();
          const avg = rrList.reduce((a,b)=>a+b,0) / rrList.length;
          document.getElementById('ecgBpm').innerText = Math.round(60000/avg);
        }
      }
      lastPeakT = now;
      peakHist.push(samples[i]);
      if (peakHist.length > 10) peakHist.shift();
      const avgP = peakHist.reduce((a,b)=>a+b,0) / peakHist.length;
      document.getElementById('ecgPeak').innerText = (avgP * CAL).toFixed(2);
    }
  }
}

function getOrdered(){
  const n     = ecgFilled;
  const out   = new Float32Array(n);
  const start = (ecgFilled < ECG_WINDOW) ? 0 : ecgWrite;
  for (let i = 0; i < n; i++) out[i] = ecgRing[(start + i) % ECG_WINDOW];
  return out;
}

function draw(){
  const w = canvas.width  = canvas.clientWidth;
  const h = canvas.height = canvas.clientHeight;
  ctx.clearRect(0, 0, w, h);
  if (showGrid) drawGrid(w, h);
  drawTrace(w, h);
}

function drawGrid(w, h){
  const cols = 50, rows = 20;
  for (let i = 0; i <= cols; i++) {
    const x = i * w / cols, major = (i % 5 === 0);
    ctx.beginPath();
    ctx.strokeStyle = major ? '#1a3a1a' : '#0d1f0d';
    ctx.lineWidth   = major ? 0.8 : 0.4;
    ctx.moveTo(x,0); ctx.lineTo(x,h); ctx.stroke();
  }
  for (let j = 0; j <= rows; j++) {
    const y = j * h / rows, major = (j % 5 === 0);
    ctx.beginPath();
    ctx.strokeStyle = major ? '#1a3a1a' : '#0d1f0d';
    ctx.lineWidth   = major ? 0.8 : 0.4;
    ctx.moveTo(0,y); ctx.lineTo(w,y); ctx.stroke();
  }
  ctx.fillStyle = '#00ff88'; ctx.font = '10px monospace';
  ctx.fillText('1mV', 8, 22);
}

function drawTrace(w, h){
  const src = getOrdered();
  const n   = src.length;
  if (n < 2) return;

  const xStep  = w / ECG_WINDOW;
  const xOffset = (ECG_WINDOW - n) * xStep;
  const mid    = h / 2;
  const scale  = h / 8000;

  // Pass 1: continuous waveform
  ctx.beginPath();
  ctx.strokeStyle = '#ff3366';
  ctx.lineWidth   = 1.8;
  ctx.lineJoin    = 'round';
  for (let i = 0; i < n; i++) {
    const x = xOffset + i * xStep;
    const y = mid - src[i] * scale;
    i === 0 ? ctx.moveTo(x, y) : ctx.lineTo(x, y);
  }
  ctx.stroke();

  // Pass 2: R-peak dots (separate beginPath per dot)
  let localSum = 0;
  for (let v of src) localSum += v;
  const lMean  = localSum / n;
  const dThresh = lMean + (4095 - lMean) * 0.50;

  ctx.fillStyle = '#ff3366';
  for (let i = 1; i < n - 1; i++) {
    if (src[i] > dThresh && src[i] > src[i-1] && src[i] > src[i+1]) {
      const x = xOffset + i * xStep;
      const y = mid - src[i] * scale;
      ctx.beginPath();
      ctx.arc(x, y - 8, 3.5, 0, 2 * Math.PI);
      ctx.fill();
    }
  }
}

// ── Vitals update ─────────────────────────────────────────────────
function updateVitals(d){
  const hr = d.bpm||0;
  document.getElementById('bpm').innerText = hr>0 ? hr : '--';
  document.getElementById('spo2').innerText = d.spo2||'--';
  const hrSt = document.getElementById('hrStatus');
  hrSt.innerHTML = hr>0
    ? '<span style="color:#00ff88">♥ Heart detected</span>'
    : '<span style="color:#fbbf24">⚠ Place finger on sensor</span>';

  document.getElementById('bodyTemp').innerText = d.bodyT ? d.bodyT.toFixed(1) : '--';
  document.getElementById('roomTemp').innerText = d.roomT ? d.roomT.toFixed(1) : '--';

  if(d.sys>0){
    document.getElementById('bpVal').innerText = d.sys+'/'+d.dia;
    document.getElementById('bpPulse').innerText = d.bp_pulse||'--';
    let cls='';
    if(d.sys<120&&d.dia<80)      cls='<span class="green">✅ NORMAL</span>';
    else if(d.sys<130&&d.dia<80) cls='<span class="yellow">⚠ ELEVATED</span>';
    else if(d.sys<140||d.dia<90) cls='<span class="red">⚠ STAGE 1 HIGH</span>';
    else                          cls='<span class="red">🚨 STAGE 2 HIGH</span>';
    document.getElementById('bpClass').innerHTML=cls;
  } else {
    document.getElementById('bpVal').innerText='--/--';
    document.getElementById('bpClass').innerHTML='';
  }
  document.getElementById('bpState').innerText='('+(d.bp_state||'Idle')+')';
  document.getElementById('bpCuff').innerText = d.bp_mmhg!=null?Math.round(d.bp_mmhg):'--';
  document.getElementById('bpMap').innerText  = d.bp_map !=null?Math.round(d.bp_map) :'--';

  document.getElementById('pos').innerText = d.pos||'--';
  document.getElementById('ax').innerText  = d.ax!=null?d.ax.toFixed(2):'0';
  document.getElementById('ay').innerText  = d.ay!=null?d.ay.toFixed(2):'0';
  document.getElementById('az').innerText  = d.az!=null?d.az.toFixed(2):'0';

  document.getElementById('rssi').innerText   = d.rssi||'--';
  document.getElementById('ip').innerText     = d.ip||'--';
  document.getElementById('uptime').innerText = d.uptime||'--';

  const lo = d.ecg_leads;
  const badge = document.getElementById('leadsBadge');
  badge.innerText = lo ? '⚡ LEADS OFF' : '⚡ LEADS ON';
  badge.className = 'badge '+(lo?'off':'on');
  document.getElementById('ecgStatus').innerHTML = lo
    ? '<span class="red">Off</span>' : '<span class="green">On</span>';
  document.getElementById('ecgLeads').innerText = lo ? 'OFF' : 'ON';

  if(d.ecg_bpm>0) document.getElementById('ecgBpm').innerText=d.ecg_bpm;
}

// ── SMS: send via fetch POST to /sendSMS ──────────────────────────
function setSmsStatus(msg, color){
  const el = document.getElementById('smsStatus');
  el.innerText = msg;
  el.style.color = color || '#fbbf24';
  if (msg) setTimeout(()=>{ el.innerText=''; }, 6000);
}

async function sendSMS(){
  const phone = document.getElementById('phoneInput').value.trim();
  if (!phone || phone.length < 7) {
    setSmsStatus('⚠ Enter a valid phone number', '#ff3366');
    return;
  }
  const btn = document.getElementById('smsBtn');
  btn.disabled = true;
  btn.innerText = '⏳ Sending…';
  setSmsStatus('Sending SMS…', '#fbbf24');
  try {
    const r = await fetch('/sendSMS?phone=' + encodeURIComponent(phone));
    const t = await r.text();
    if (r.ok && t === 'OK') {
      setSmsStatus('✅ SMS sent to ' + phone, '#00ff88');
    } else {
      setSmsStatus('❌ Failed: ' + t, '#ff3366');
    }
  } catch(e) {
    setSmsStatus('❌ Network error', '#ff3366');
  }
  btn.disabled = false;
  btn.innerText = '📨 SEND SMS';
}

// ── On-demand update: send "update" string over WebSocket ─────────
function requestUpdate(){
  if (!wsock || wsock.readyState !== WebSocket.OPEN) {
    setSmsStatus('⚠ WebSocket not connected', '#ff3366');
    return;
  }
  wsock.send('update');
  setSmsStatus('📡 Update requested…', '#38bdf8');
}

// ── Other button handlers ─────────────────────────────────────────
function startBP(){ wsock&&wsock.send('startBP'); fetch('/startBP'); }

function resetECG(){
  ecgRing.fill(0); ecgWrite=0; ecgFilled=0;
  peakHist=[]; rrList=[]; lastPeakT=0;
  document.getElementById('ecgBpm').innerText='--';
  draw();
  // Tell firmware to reset its ecgSentIdx too
  wsock && wsock.send('resetECG');
}

function toggleGrid(){ showGrid=!showGrid; draw(); }

connectWS();
window.addEventListener('resize', draw);
</script>
</body>
</html>
)rawliteral";


// ══════════════════════════════════════════════════════════════════
//  SIM800L HELPERS
// ══════════════════════════════════════════════════════════════════

// ── Proven sendAT pattern (matches working sim800l.ino test suite) ─
bool sim800SendAT(const String& cmd, const String& expected = "OK", unsigned long wait = 10000) {
  sim800.println(cmd);
  String resp = "";
  unsigned long start = millis();
  while (millis() - start < wait) {
    while (sim800.available()) {
      char c = sim800.read();
      resp += c;
      if (resp.indexOf(expected) != -1) return true;
    }
  }
  Serial.println("[SIM] cmd=" + cmd + " resp=" + resp);
  return false;
}

// Keep old helpers as thin wrappers so rest of code compiles unchanged
void sim800SendCmd(const String& cmd, unsigned long waitMs = 300) {
  sim800.println(cmd);
  delay(waitMs);
}

bool sim800WaitFor(const String& target, unsigned long timeoutMs = 3000) {
  unsigned long t0 = millis();
  String buf = "";
  while (millis() - t0 < timeoutMs) {
    while (sim800.available()) buf += (char)sim800.read();
    if (buf.indexOf(target) >= 0) return true;
    delay(10);
  }
  return false;
}

// ══════════════════════════════════════════════════════════════════
//  initSIM800L — FIXED v5.1
//  Changes vs v5.0:
//   • Boot delay: 3000 → 5000 ms  (module needs 5-8 s after power-on)
//   • AT handshake: single attempt → retry loop x10 with 2 s each
//   • ATE0 added after AT OK (echo off = cleaner response parsing)
//   • AT+IPR=9600 locks baud rate (survives module power resets)
//   • All timeouts aligned with working sim800l.ino test suite
// ══════════════════════════════════════════════════════════════════
void initSIM800L() {
  Serial.println("Init SIM800L...");
  sim800.begin(9600, SERIAL_8N1, SIM_RX_PIN, SIM_TX_PIN);

  // ── Give module time to boot (was 3000, needs ≥5000) ────────────
  delay(5000);

  // ── AT handshake — retry up to 10 × 2 s = 20 s total ───────────
  // (matches sim800l.ino single 10 s timeout behaviour)
  bool atOK = false;
  for (int i = 0; i < 10 && !atOK; i++) {
    if (sim800SendAT("AT", "OK", 2000)) {
      atOK = true;
    } else {
      Serial.println("  AT retry " + String(i + 1) + "/10...");
      delay(1000);
    }
  }
  if (!atOK) {
    Serial.println("❌ No AT response — check wiring and power supply (SIM800L needs 2A!)");
    return;
  }
  Serial.println("  AT: OK");

  // ── Echo off — cleaner response strings ─────────────────────────
  sim800SendAT("ATE0", "OK", 2000);

  // ── Lock baud rate so it survives module resets ──────────────────
  sim800SendAT("AT+IPR=9600", "OK", 2000);

  // ── SIM card ready ───────────────────────────────────────────────
  if (!sim800SendAT("AT+CPIN?", "READY", 5000)) {
    // Check if PIN-locked
    String r = "";
    unsigned long t0 = millis();
    while (millis() - t0 < 1000) {
      while (sim800.available()) r += (char)sim800.read();
    }
    if (r.indexOf("SIM PIN") != -1) {
      Serial.println("❌ SIM PIN locked — unlock it first");
    } else {
      Serial.println("❌ SIM not ready — no SIM card or loose contact");
    }
    return;
  }
  Serial.println("  SIM: READY");

  // ── Network registration — wait up to 30 s ───────────────────────
  bool registered = false;
  for (int i = 0; i < 15 && !registered; i++) {
    sim800.println("AT+CREG?");
    String r = "";
    unsigned long t0 = millis();
    while (millis() - t0 < 2000) {
      while (sim800.available()) r += (char)sim800.read();
    }
    if (r.indexOf(",1") != -1 || r.indexOf(",5") != -1) {
      registered = true;
    } else {
      Serial.println("  Waiting for network... (" + String(i + 1) + "/15)");
      delay(2000);
    }
  }
  if (!registered) {
    Serial.println("❌ Network not registered — check SIM balance / antenna / signal");
    return;
  }
  Serial.println("  Network: registered");

  // ── SMS text mode ────────────────────────────────────────────────
  sim800SendAT("AT+CMGF=1", "OK", 3000);
  sim800SendAT("AT+CNMI=1,2,0,0,0", "OK", 3000);

  sim800Ready = true;
  Serial.println("✅ SIM800L ready");
}

void sendSMS(const String& number, const String& msg) {
  if (!sim800Ready) { Serial.println("[SMS] not ready"); return; }

  // Flush buffer
  while (sim800.available()) sim800.read();

  sim800SendAT("AT+CMGF=1", "OK", 3000);

  // Send number, wait for >
  if (!sim800SendAT("AT+CMGS=\"" + number + "\"", ">", 5000)) {
    Serial.println("[SMS] ❌ No > prompt");
    sim800.write(27); // ESC
    return;
  }

  sim800.print(msg);
  sim800.write(26); // Ctrl-Z

  // Wait for confirmation
  String resp = "";
  unsigned long t0 = millis();
  while (millis() - t0 < 15000) {
    while (sim800.available()) resp += (char)sim800.read();
    if (resp.indexOf("+CMGS:") != -1) { Serial.println("[SMS] ✅ Sent to " + number); return; }
    if (resp.indexOf("ERROR") != -1)  { Serial.println("[SMS] ❌ ERROR: " + resp); return; }
  }
  Serial.println("[SMS] ❌ Timeout");
}

String buildVitalsSummary() {
  String s = "Patient Vitals Update:\n";
  s += "HR: "   + String((int)shared.bpm)  + " BPM\n";
  s += "SpO2: " + String((int)shared.spo2) + "%\n";
  s += "Temp: " + String(shared.bodyTemp, 1) + " C\n";
  s += (shared.sys > 0)
     ? "BP: " + String(shared.sys) + "/" + String(shared.dia) + " mmHg\n"
     : "BP: N/A\n";
  s += "ECG BPM: " + String(ecgCurrentBPM) + "\n";
  s += "Pos: "  + shared.position;
  return s;
}

void checkAndAlert() {
  if (!sim800Ready) return;
  unsigned long now = millis();

  auto maybeAlert = [&](int ch, bool condition, const String& msg) {
    if (condition && now - alertCooldown[ch] > ALERT_COOLDOWN_MS) {
      alertCooldown[ch] = now;
      sendSMS(caregiverNumber, "⚠️ ALERT: " + msg + "\n" + buildVitalsSummary());
    }
  };

  maybeAlert(0, shared.hrValid && shared.bpm > 0 &&
               (shared.bpm < ALERT_HR_LOW || shared.bpm > ALERT_HR_HIGH),
             "Heart Rate abnormal (" + String((int)shared.bpm) + " BPM)");

  maybeAlert(1, shared.spo2 > 0 && shared.spo2 < ALERT_SPO2_LOW,
             "Low SpO2 (" + String((int)shared.spo2) + "%)");

  maybeAlert(2, shared.bodyTemp > ALERT_TEMP_HIGH,
             "High Temperature (" + String(shared.bodyTemp, 1) + " C)");

  maybeAlert(3, shared.sys > ALERT_SYS_HIGH,
             "High Systolic BP (" + String(shared.sys) + " mmHg)");

  maybeAlert(4, shared.dia > ALERT_DIA_HIGH,
             "High Diastolic BP (" + String(shared.dia) + " mmHg)");
}

// ── BUG FIX: handle "update" AND "send" keywords + reset cmd ────
void checkIncomingSMS() {
  static unsigned long lastCheck = 0;
  if (millis() - lastCheck < 5000) return;
  lastCheck = millis();

  if (!sim800Ready || !sim800.available()) return;

  String incoming = "";
  unsigned long t0 = millis();
  while (millis() - t0 < 300) {
    if (sim800.available()) incoming += (char)sim800.read();
  }
  incoming.toLowerCase();

  // "update" or "send" keyword triggers a vitals SMS reply
  if (incoming.indexOf("update") >= 0 || incoming.indexOf("send") >= 0) {
    Serial.println("[SMS] on-demand request received");
    sendSMS(caregiverNumber, buildVitalsSummary());
  }
}


// ══════════════════════════════════════════════════════════════════
//  ISR / CALLBACKS
// ══════════════════════════════════════════════════════════════════
void IRAM_ATTR onBeatDetected() {
  shared.lastBeatMs = millis();
  shared.hrValid    = true;
  buzzerFlag        = true;
}


// ══════════════════════════════════════════════════════════════════
//  CORE 1 TASK — MAX30100 only
// ══════════════════════════════════════════════════════════════════
void Core1Task(void*) {
  Serial.println("[Core1] Starting MAX30100");
  bool ok = false;
  for (int i = 0; i < 5 && !ok; i++) {
    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(500)) == pdTRUE) {
      ok = pox.begin();
      xSemaphoreGive(i2cMutex);
    }
    if (!ok) vTaskDelay(pdMS_TO_TICKS(1000));
  }
  if (!ok) {
    Serial.println("[Core1] ❌ MAX30100 init failed");
    shared.sensorOK = false;
    vTaskDelete(NULL); return;
  }
  pox.setIRLedCurrent(MAX30100_LED_CURR_50MA);
  pox.setOnBeatDetectedCallback(onBeatDetected);
  shared.sensorOK = true;
  Serial.println("[Core1] ✅ MAX30100 ready");

  unsigned long lastReport = 0;
  for (;;) {
    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      pox.update();
      xSemaphoreGive(i2cMutex);
    }
    if (millis() - lastReport >= 1000) {
      lastReport = millis();
      float hr = pox.getHeartRate();
      float sp = pox.getSpO2();
      if (hr > 30 && hr < 220) { shared.bpm = hr; shared.hrValid = true; }
      if (sp > 70 && sp <= 100) shared.spo2 = sp;
    }
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}


// ══════════════════════════════════════════════════════════════════
//  CORE 0 TASK — all other sensors
// ══════════════════════════════════════════════════════════════════
void Core0Task(void*) {
  for (;;) {
    readDS18B20();
    readDHT22();
    readMPU6050();
    readECG();
    readBP();
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}


// ══════════════════════════════════════════════════════════════════
//  SENSOR FUNCTIONS
// ══════════════════════════════════════════════════════════════════

void initMPU6050() {
  Serial.print("Init MPU6050... ");
  if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
    Wire.beginTransmission(MPU_ADDR);
    bool found = (Wire.endTransmission() == 0);
    if (found) {
      Wire.beginTransmission(MPU_ADDR);
      Wire.write(0x6B); Wire.write(0);
      Wire.endTransmission();
      Serial.println("✅");
    } else Serial.println("❌ not found");
    xSemaphoreGive(i2cMutex);
  }
}

void readMPU6050() {
  static unsigned long t = 0;
  if (millis() - t < 100) return; t = millis();
  if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(10)) != pdTRUE) return;
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x3B);
  Wire.endTransmission(false); Wire.requestFrom(MPU_ADDR, 6);
  if (Wire.available() >= 6) {
    int16_t ax = Wire.read()<<8|Wire.read();
    int16_t ay = Wire.read()<<8|Wire.read();
    int16_t az = Wire.read()<<8|Wire.read();
    shared.ax = ax/16384.0f; shared.ay = ay/16384.0f; shared.az = az/16384.0f;
    shared.position = (fabs(shared.ax)>0.5f||fabs(shared.ay)>0.5f)?"Moving":"Stable";
  }
  xSemaphoreGive(i2cMutex);
}

void initDS18B20() {
  Serial.print("Init DS18B20... ");
  bodySens.begin(); bodySens.setResolution(12);
  bodySens.requestTemperatures(); delay(750);
  float t = bodySens.getTempCByIndex(0);
  if (t != DEVICE_DISCONNECTED_C && t > 10 && t < 45) {
    shared.bodyTemp = t;
    Serial.printf("✅ %.1f°C\n", t);
  } else Serial.println("❌ not found");
}

void readDS18B20() {
  static unsigned long t = 0;
  if (millis()-t < 2000) return; t = millis();
  bodySens.requestTemperatures();
  float v = bodySens.getTempCByIndex(0);
  if (v != DEVICE_DISCONNECTED_C && v > 10 && v < 45) shared.bodyTemp = v;
}

void initDHT22() {
  Serial.print("Init DHT22... ");
  dht.begin(); delay(2000);
  float h = dht.readHumidity(), t = dht.readTemperature();
  if (!isnan(h) && !isnan(t)) {
    shared.roomTemp = t; shared.roomHum = h;
    Serial.printf("✅ %.1f°C %.1f%%\n", t, h);
  } else Serial.println("❌ not found");
}

void readDHT22() {
  static unsigned long t = 0;
  if (millis()-t < 2000) return; t = millis();
  float h = dht.readHumidity(), v = dht.readTemperature();
  if (!isnan(h) && !isnan(v)) { shared.roomHum = h; shared.roomTemp = v; }
}

// ── AD8232 ECG ────────────────────────────────────────────────────
void initECG() {
  Serial.print("Init AD8232 ECG... ");
  pinMode(LO_PLUS_PIN,  INPUT);
  pinMode(LO_MINUS_PIN, INPUT);
  pinMode(ECG_PIN,      INPUT);
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  lastEcgUs = micros();
  Serial.println("✅ 200 Hz ready");
}

void readECG() {
  unsigned long now = micros();
  if (now - lastEcgUs < (unsigned long)ECG_DELAY_US) return;
  lastEcgUs = now;

  bool loPlus  = digitalRead(LO_PLUS_PIN);
  bool loMinus = digitalRead(LO_MINUS_PIN);
  ecgLeadOff   = loPlus || loMinus;

  int raw = ecgLeadOff ? 0 : analogRead(ECG_PIN);

  if (!ecgLeadOff) {
    ecgRunMean = (int)(ecgRunMean * 0.98f + raw * 0.02f);
    int thresh = ecgRunMean + (int)((4095 - ecgRunMean) * 0.65f);

    if (raw > thresh && !aboveThreshold) {
      aboveThreshold = true;
      long nowMs = (long)(now / 1000);

      if (lastPeakMs > 0) {
        long interval = nowMs - lastPeakMs;
        if (interval > 300 && interval < 2000) {
          peakBuf[peakIdx % 10] = (int)interval;
          peakIdx++;
          if (peakCount < 10) peakCount++;

          long sum = 0;
          int  n   = min(peakCount, 8);
          for (int i = 0; i < n; i++) {
            sum += peakBuf[((peakIdx - 1 - i) + 10) % 10];
          }
          ecgCurrentBPM = (int)(60000L / (sum / n));
          if (ecgCurrentBPM < 30 || ecgCurrentBPM > 220) ecgCurrentBPM = 0;
        }
      }
      lastPeakMs = nowMs;
    } else if (raw <= ecgRunMean) {
      aboveThreshold = false;
    }
  } else {
    ecgCurrentBPM  = 0;
    peakCount      = 0;
    peakIdx        = 0;
    lastPeakMs     = 0;
    aboveThreshold = false;
  }

  ecgBuf[ecgBufIdx] = (int16_t)raw;
  ecgBufIdx = (ecgBufIdx + 1) % ECG_BUF_SIZE;
}

// ── BP ─────────────────────────────────────────────────────────────
void initBP() {
  Serial.print("Init BP... ");
  analogReadResolution(12); analogSetAttenuation(ADC_11db);
  long sum = 0;
  for (int i = 0; i < 100; i++) { sum += analogRead(BP_DC_PIN); delay(10); }
  bpZero = sum / 100;
  bpLastPressure = 0;
  Serial.printf("✅ zero=%d\n", bpZero);
}

void calculateBPResult() {
  if (bpBufIdx < 50) return;
  bpMaxAmp = 0; int mapIdx = 0;
  for (int i = 10; i < bpBufIdx-10; i++) {
    if (bpAmpBuf[i] > bpMaxAmp) {
      bpMaxAmp = bpAmpBuf[i]; bpMapPressure = bpPressBuf[i]; mapIdx = i;
    }
  }
  float sysT = bpMaxAmp * SYS_RATIO, diaT = bpMaxAmp * DIA_RATIO;
  int si = -1;
  for (int i = mapIdx; i > 10; i--) if (bpAmpBuf[i] < sysT) { si = i+2; break; }
  int di = -1;
  for (int i = mapIdx; i < bpBufIdx-10; i++) if (bpAmpBuf[i] < diaT) { di = i-2; break; }

  float rSys = si > 0 ? bpPressBuf[si] : bpMapPressure * 1.2f;
  float rDia = di > 0 ? bpPressBuf[di] : bpMapPressure * 0.8f;

  shared.sys = ((int)(rSys * 1.33f) + sysOffset) / 2 * 2;
  shared.dia = ((int)(rDia * 1.18f) + diaOffset) / 2 * 2;
  shared.sys = constrain(shared.sys, 70, 220);
  shared.dia = constrain(shared.dia, 40, 140);
  if (shared.sys <= shared.dia) shared.sys = shared.dia + 30;
  shared.bpMap = bpMapPressure;

  int peaks = 0;
  for (int i = 10; i < bpBufIdx-10; i++) {
    if (bpAmpBuf[i] > bpMaxAmp*0.2f &&
        bpAmpBuf[i] > bpAmpBuf[i-1] && bpAmpBuf[i] > bpAmpBuf[i+1]) {
      peaks++; i += 10;
    }
  }
  float dur = (bpBufIdx * BP_SAMPLE_MS) / 60000.0f;
  shared.bpPulse = dur > 0 ? (int)(peaks/dur) : (int)shared.bpm;

  Serial.printf("[BP] %d/%d mmHg  pulse=%d\n", shared.sys, shared.dia, shared.bpPulse);
  checkAndAlert();
}

void readBP() {
  if (millis() - lastBPRead > 100) {
    lastBPRead = millis();
    int r = analogRead(BP_DC_PIN);
    shared.bpCuffMmHg = max(0, r - bpZero) * PRESSURE_SCALE;
  }
  static unsigned long ts = 0;
  if (millis() - ts < BP_SAMPLE_MS) return; ts = millis();

  int rDC = analogRead(BP_DC_PIN), rAC = analogRead(BP_AC_PIN);
  float pMmHg = max(0, rDC - bpZero) * PRESSURE_SCALE;
  float ac = rAC - 2048.0f;
  bpFilteredPulse = FILTER_ALPHA * (bpPrevFiltered + ac - bpPrevRaw);
  bpPrevRaw = ac; bpPrevFiltered = bpFilteredPulse;

  switch (shared.bpCode) {
    case 0:
      shared.bpState = "Idle";
      if (pMmHg > 30) {
        shared.bpCode = 1; bpBufIdx = 0; bpMaxAmp = 0; bpLastPressure = 0;
        Serial.println("[BP] started");
      }
      break;
    case 1:
      shared.bpState = "Inflating";
      if (bpBufIdx < MAX_BP_SAMPLES) {
        bpPressBuf[bpBufIdx] = pMmHg;
        bpAmpBuf[bpBufIdx]   = fabs(bpFilteredPulse);
        bpBufIdx++;
      }
      if (pMmHg > 80) {
        if (pMmHg < bpLastPressure - 2.0f) {
          shared.bpCode = 2; bpBufIdx = 0; bpMaxAmp = 0;
          Serial.println("[BP] deflating");
        }
        bpLastPressure = pMmHg;
      }
      break;
    case 2:
      shared.bpState = "Measuring";
      if (bpBufIdx < MAX_BP_SAMPLES) {
        bpPressBuf[bpBufIdx] = pMmHg;
        bpAmpBuf[bpBufIdx]   = fabs(bpFilteredPulse);
        if (fabs(bpFilteredPulse) > bpMaxAmp) {
          bpMaxAmp = fabs(bpFilteredPulse); bpMapPressure = pMmHg;
        }
        bpBufIdx++;
      }
      if (pMmHg < STOP_PRESSURE) shared.bpCode = 3;
      break;
    case 3:
      shared.bpState = "Calculating";
      calculateBPResult();
      shared.bpCode = 0;
      break;
  }
}

void startBPMeasurement() {
  shared.bpCode = 0;
  shared.sys = shared.dia = 0;
  bpLastPressure = 0;
  bpBufIdx       = 0;     // BUG FIX: confirmed present
  buzzerFlag     = true;

  // Pulse GPIO 13 HIGH for 0.5 s (non-blocking)
  digitalWrite(BP_TRIGGER_PIN, HIGH);
  bpTriggerOffMs = millis() + 500;
}


// ══════════════════════════════════════════════════════════════════
//  JSON BUILD
// ══════════════════════════════════════════════════════════════════
void buildVitalsJSON() {
  StaticJsonDocument<512> doc;
  doc["bpm"]       = (shared.hrValid && shared.bpm > 0) ? (int)shared.bpm : 0;
  doc["spo2"]      = (int)shared.spo2;
  doc["bodyT"]     = shared.bodyTemp;
  doc["roomT"]     = shared.roomTemp;
  doc["roomH"]     = shared.roomHum;
  doc["ax"]        = shared.ax;
  doc["ay"]        = shared.ay;
  doc["az"]        = shared.az;
  doc["pos"]       = shared.position;
  doc["sys"]       = shared.sys;
  doc["dia"]       = shared.dia;
  doc["bp_pulse"]  = shared.bpPulse;
  doc["bp_state"]  = shared.bpState;
  doc["bp_mmhg"]   = (int)shared.bpCuffMmHg;
  doc["bp_map"]    = (int)shared.bpMap;
  doc["ecg_leads"] = ecgLeadOff;
  doc["ecg_bpm"]   = ecgCurrentBPM;
  doc["uptime"]    = millis() / 1000;
  doc["rssi"]      = WiFi.RSSI();
  doc["ip"]        = WiFi.localIP().toString();
  jsonVitals = "";
  serializeJson(doc, jsonVitals);
}

// BUG FIX: skip broadcast when no new samples are available
void buildECGJSON() {
  int available = (ecgBufIdx - ecgSentIdx + ECG_BUF_SIZE) % ECG_BUF_SIZE;
  if (available == 0) return;                     // ← guard added

  DynamicJsonDocument doc(2048);
  JsonArray arr = doc.createNestedArray("ecg");

  int toSend = min(available, 100);
  for (int i = 0; i < toSend; i++) {
    arr.add(ecgBuf[(ecgSentIdx + i) % ECG_BUF_SIZE]);
  }
  ecgSentIdx = (ecgSentIdx + toSend) % ECG_BUF_SIZE;

  jsonECG = "";
  serializeJson(doc, jsonECG);
  ws.broadcastTXT(jsonECG);   // broadcast only when we have data
}


// ══════════════════════════════════════════════════════════════════
//  LCD
// ══════════════════════════════════════════════════════════════════
void updateLCD() {
  if (millis() - lastLCDUpdate < 1000) return;
  lastLCDUpdate = millis();
  char buf[21];
  lcd.clear();
  lcd.setCursor(0,0);
  snprintf(buf, 21, "HR:%3d  SpO2:%3d%%", (int)shared.bpm, (int)shared.spo2);
  lcd.print(buf);
  lcd.setCursor(0,1);
  snprintf(buf, 21, "Bdy:%.1fC Rm:%.1fC", shared.bodyTemp, shared.roomTemp);
  lcd.print(buf);
  lcd.setCursor(0,2);
  if (shared.sys > 0)
    snprintf(buf, 21, "BP:%d/%d P:%d", shared.sys, shared.dia, shared.bpPulse);
  else
    snprintf(buf, 21, "BP:%-16s", shared.bpState.c_str());
  lcd.print(buf);
  lcd.setCursor(0,3);
  snprintf(buf, 21, "ECG:%-3s BPM:%3d", ecgLeadOff ? "OFF" : "ON", ecgCurrentBPM);
  lcd.print(buf);
}


// ══════════════════════════════════════════════════════════════════
//  WEB SERVER HANDLERS
// ══════════════════════════════════════════════════════════════════
void handleRoot()    { server.send_P(200, "text/html", INDEX_HTML); }
void handleData()    { buildVitalsJSON(); server.send(200, "application/json", jsonVitals); }
void handleStartBP() { startBPMeasurement(); server.send(200, "text/plain", "OK"); }

// /sendSMS?phone=+8801XXXXXXXXX
// Composes a vitals summary and sends via SIM800L.
void handleSendSMS() {
  if (!server.hasArg("phone")) {
    server.send(400, "text/plain", "Missing phone param");
    return;
  }
  String phone = server.arg("phone");
  phone.trim();
  if (phone.length() < 7) {
    server.send(400, "text/plain", "Invalid phone number");
    return;
  }
  if (!sim800Ready) {
    server.send(503, "text/plain", "SIM800L not ready");
    return;
  }
  // Update caregiverNumber so auto-alerts also use this number
  caregiverNumber = phone;

  sendSMS(phone, buildVitalsSummary());
  server.send(200, "text/plain", "OK");
}

void onWsEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t len) {
  if (type == WStype_CONNECTED) {
    Serial.printf("[WS] client %u connected\n", num);
  } else if (type == WStype_TEXT) {
    String cmd = String((char*)payload);
    if (cmd == "startBP") {
      startBPMeasurement();
    } else if (cmd == "update") {
      // On-demand vitals reply over WS
      buildVitalsJSON();
      ws.sendTXT(num, jsonVitals);
    } else if (cmd == "resetECG") {
      // BUG FIX: sync ecgSentIdx so stale samples aren't re-sent
      ecgSentIdx = ecgBufIdx;
    }
  }
}


// ══════════════════════════════════════════════════════════════════
//  SETUP
// ══════════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n=== Health Monitor v5.1 ===");

  i2cMutex = xSemaphoreCreateMutex();
  Wire.begin(21, 22);

  lcd.init(); lcd.backlight();
  lcd.setCursor(0,0); lcd.print("Health Monitor v5.1");
  lcd.setCursor(0,1); lcd.print("Init sensors...");

  initDS18B20();
  initDHT22();
  initMPU6050();
  initBP();
  initECG();

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  pinMode(BP_TRIGGER_PIN, OUTPUT);
  digitalWrite(BP_TRIGGER_PIN, LOW);

  lcd.setCursor(0,2); lcd.print("WiFi connecting...");
  wm.setConfigPortalTimeout(120);
  if (wm.autoConnect("HealthMonitor-v51")) {
    String ip = WiFi.localIP().toString();
    shared.ip = ip;
    Serial.println("[WiFi] " + ip);
    lcd.setCursor(0,2); lcd.print(("IP:" + ip).c_str());
  } else {
    Serial.println("[WiFi] offline mode");
    lcd.setCursor(0,2); lcd.print("WiFi: offline   ");
  }

  initSIM800L();

  server.on("/",        HTTP_GET, handleRoot);
  server.on("/data",    HTTP_GET, handleData);
  server.on("/startBP", HTTP_GET, handleStartBP);
  server.on("/sendSMS", HTTP_GET, handleSendSMS);
  server.onNotFound([](){
    server.send(404, "text/plain", "Not found: " + server.uri());
  });
  server.begin();
  ws.begin();
  ws.onEvent(onWsEvent);

  xTaskCreatePinnedToCore(Core0Task, "Core0", 12000, NULL, 1, &hCore0, 0);
  xTaskCreatePinnedToCore(Core1Task, "Core1", 10000, NULL, 2, &hCore1, 1);

  lcd.setCursor(0,3); lcd.print("Ready!          ");
  Serial.println("[System] Ready — open http://" + shared.ip);
}


// ══════════════════════════════════════════════════════════════════
//  LOOP  (Arduino main task = Core 1)
// ══════════════════════════════════════════════════════════════════
void loop() {
  // ── Buzzer ────────────────────────────────────────────────────
  if (buzzerFlag) {
    buzzerFlag = false;
    digitalWrite(BUZZER_PIN, HIGH);
    buzzerOffMs = millis() + 50;
  }
  if (millis() > buzzerOffMs && digitalRead(BUZZER_PIN)) {
    digitalWrite(BUZZER_PIN, LOW);
  }

  // ── BP Trigger GPIO 13 — turn off after 0.5 s ─────────────────
  if (bpTriggerOffMs > 0 && millis() >= bpTriggerOffMs) {
    digitalWrite(BP_TRIGGER_PIN, LOW);
    bpTriggerOffMs = 0;
  }

  // ── WiFi-dependent tasks ──────────────────────────────────────
  if (WiFi.status() == WL_CONNECTED) {
    server.handleClient();
    ws.loop();

    if (millis() - lastVitalsSend > 100) {
      lastVitalsSend = millis();
      buildVitalsJSON();
      ws.broadcastTXT(jsonVitals);
      checkAndAlert();
    }

    // BUG FIX: buildECGJSON() now guards against empty sends internally
    if (millis() - lastECGSend > 50) {
      lastECGSend = millis();
      buildECGJSON();    // broadcasts only if new samples exist
    }
  }

  checkIncomingSMS();
  updateLCD();
  delay(5);
}
