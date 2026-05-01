// ── Pins ─────────────────────────────────────────────────────────
#define AIN1 20
#define AIN2 19
#define BIN1 22
#define BIN2 23
#define STBY 21
#define PWMA 18
#define PWMB 24

#define EMITTER_ODD 17

#define MOTOR_TRIM 0

#define PWM_FREQ 20000
#define PWM_RES  8
#define CHANNEL_A 0
#define CHANNEL_B 1

#define EMA_ALPHA 0.12f

float smoothPosition = 0;
bool  smoothInitialized = false;

#include <QTRSensors.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoOTA.h>

const char* ssid     = "SopercoolCar";
const char* password = "superduper";
WebServer server(80);

// ── QTR Sensors ───────────────────────────────────────────────────
QTRSensors qtr;
const uint8_t SENSOR_COUNT = 11;
uint8_t qtrPins[SENSOR_COUNT] = {2,3,4,5,6,7,8,9,10,11,12};

uint16_t sensorValues[SENSOR_COUNT];
uint16_t calibValues[SENSOR_COUNT];
int  linePosition     = 0;
int  lastKnownPosition = 0;        
const int LINE_CENTER = ((SENSOR_COUNT - 1) * 1000) / 2;
bool lineDetected     = false;

bool isFollowing = false;

// ── PID params ────────────────────────────────────────────────────

struct PIDParams {
  float kp        = 0.08f;
  float ki        = 0.0f;
  float kd        = 0.40f;
  int   baseSpeed = 120;
  int   maxSpeed  = 220;
};
PIDParams pid;

float pidIntegral  = 0;
int   pidLastError = 0;

void resetPID() { pidIntegral = 0; pidLastError = 0; }

// ── Run metrics ───────────────────────────────────────────────────
struct RunMetrics {
  unsigned long startMs    = 0;
  unsigned long durationMs = 0;
  long          errorSum   = 0;
  long          errorSqSum = 0;
  int           maxError   = 0;
  int           lostCount  = 0;
  int           samples    = 0;
};
RunMetrics run;

// ── Sensor reading ────────────────────────────────────────────────
void readSensors() {
  qtr.read(sensorValues);
  qtr.readCalibrated(calibValues);
  linePosition = qtr.readLineBlack(calibValues);

  int activeCount = 0;
  for (int i = 0; i < SENSOR_COUNT; i++) {
    if (calibValues[i] > 75) activeCount++;
  }
  lineDetected = activeCount >= 1;

  if (lineDetected) {
    if (!smoothInitialized) {
      smoothPosition    = linePosition;
      smoothInitialized = true;
    } else {
      smoothPosition = EMA_ALPHA * linePosition
                     + (1.0f - EMA_ALPHA) * smoothPosition;
    }
    lastKnownPosition = (int)smoothPosition;
  }
}

int lineError() {
  return (int)smoothPosition - LINE_CENTER;
}

int lineDirection() {
  if (!lineDetected) return 0;
  int e = lineError();
  if      (e < -500) return -1;
  else if (e >  500) return  1;
  else               return  0;
}

// ── Motors ────────────────────────────────────────────────────────
void setupMotors() {
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);
  ledcSetup(CHANNEL_A, PWM_FREQ, PWM_RES);
  ledcAttachPin(PWMA, CHANNEL_A);
  ledcSetup(CHANNEL_B, PWM_FREQ, PWM_RES);
  ledcAttachPin(PWMB, CHANNEL_B);
}


// ── PID ───────────────────────────────────────────────────────────
void runPID() {
  int error = lineError();

  run.samples++;
  int absErr = abs(error);
  run.errorSum   += absErr;
  run.errorSqSum += (long)error * error;
  if (absErr > run.maxError) run.maxError = absErr;
  if (!lineDetected) run.lostCount++;

  pidIntegral += error;
  pidIntegral  = constrain(pidIntegral, -10000, 10000);

  float correction = pid.kp * error
                   + pid.ki * pidIntegral
                   + pid.kd * (error - pidLastError);
  pidLastError = error;

  float turnRatio = abs(correction) / (float)pid.maxSpeed;
  int   speed     = pid.baseSpeed * (1.0f - 0.45f * constrain(turnRatio, 0.0f, 1.0f));

  int leftSpeed  = speed - (int)correction;
  int rightSpeed = speed + (int)correction;
  leftSpeed  = constrain(leftSpeed,  -pid.maxSpeed, pid.maxSpeed);
  rightSpeed = constrain(rightSpeed, -pid.maxSpeed, pid.maxSpeed);

  setMotors(leftSpeed, rightSpeed);
}

// ── Web – HTML ────────────────────────────────────────────────────
void handleRoot() {
  String html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Line Follower</title>
  <style>
    * { box-sizing: border-box; margin: 0; padding: 0; }
    body { font-family: monospace; background: #111; color: #eee;
           max-width: 640px; margin: 0 auto; padding: 16px; }
    h2 { color: #0f0; margin-bottom: 12px; }
    h3 { color: #aaa; margin: 16px 0 8px; }

    .card { background: #1e1e1e; border-radius: 8px;
            padding: 14px; margin-bottom: 12px; }

    .stat-row { display: flex; gap: 12px; flex-wrap: wrap; margin-bottom: 6px; }
    .stat { background: #2a2a2a; border-radius: 5px; padding: 6px 12px;
            flex: 1; text-align: center; }
    .stat label { display: block; font-size: 0.7em; color: #888; }
    .stat span  { font-size: 1.2em; color: #0f0; }

    #barsRaw   { display: flex; align-items: flex-end; height: 80px; gap: 3px; margin-top: 8px; }
    #barsCalib { display: flex; align-items: flex-end; height: 80px; gap: 3px; margin-top: 8px; }
    #barsMin { display: flex; align-items: flex-end; height: 80px; gap: 3px; margin-top: 8px; }
    #barsMax { display: flex; align-items: flex-end; height: 80px; gap: 3px; margin-top: 8px; }

    .bar-wrap { flex: 1; display: flex; flex-direction: column; align-items: center; gap: 2px; }
    .bar { width: 100%; background: #0f0; border-radius: 2px 2px 0 0;
           transition: height 0.1s; min-height: 2px; }
    .bar-wrap span { font-size: 0.6em; color: #555; }

    #pos-track { height: 12px; background: #2a2a2a; border-radius: 6px;
                 margin-top: 10px; position: relative; }
    #pos-dot { width: 14px; height: 14px; background: #0f0; border-radius: 50%;
               position: absolute; top: -1px; margin-left: -7px; transition: left 0.1s; }
    #center-mark { width: 2px; height: 100%; background: #555;
                   position: absolute; left: 50%; }

    .btn-grid { display: grid; grid-template-columns: repeat(3, 1fr);
                gap: 8px; max-width: 240px; margin: 0 auto; }
    .btn { background: #2a2a2a; color: #eee; border: 1px solid #444;
           border-radius: 8px; padding: 18px; font-size: 1.4em;
           cursor: pointer; user-select: none; text-align: center;
           transition: background 0.1s; }
    .btn:active, .btn.active { background: #0a5; }
    .btn.stop { background: #500; }
    .btn.stop:active { background: #a00; }

    .speed-row { display: flex; align-items: center; gap: 10px; margin-top: 10px; }
    .speed-row input { flex: 1; accent-color: #0f0; }

    .btn-red { background: #500; margin-top: 4px; width: 100%;
               border: 1px solid #a00; border-radius: 8px; padding: 12px;
               color: #eee; font-family: monospace; font-size: 1em; cursor: pointer; }
    .btn-red:active { background: #a00; }

    .pid-row { display: flex; align-items: center; gap: 8px; margin-bottom: 7px; }
    .pid-row label { width: 65px; font-size: 0.8em; color: #aaa; flex-shrink: 0; }
    .pid-row input[type=range]  { flex: 1; accent-color: #0f0; }
    .pid-row input[type=number] {
      width: 72px; background: #2a2a2a; border: 1px solid #444;
      border-radius: 5px; padding: 4px 6px; color: #0f0;
      font-family: monospace; font-size: 0.9em;
    }
    .action-btn {
      flex: 1; background: #1e3a1e; border: 1px solid #0a5;
      border-radius: 8px; padding: 10px; color: #eee;
      font-family: monospace; font-size: 0.95em; cursor: pointer;
    }
    .action-btn:active { background: #0a5; }
    .go-btn { background: #1a1a3a; border-color: #44f; }
    .go-btn.running { background: #500; border-color: #a00; }

    #runTable { width: 100%; border-collapse: collapse; font-size: 0.75em; font-family: monospace; }
    #runTable th { color: #888; border-bottom: 1px solid #333; padding: 4px 8px; text-align: right; }
    #runTable th:first-child { text-align: left; }
    #runTable td { padding: 5px 8px; text-align: right; border-bottom: 1px solid #1e1e1e; cursor: pointer; }
    #runTable td:first-child { text-align: left; }
    #runTable tr:hover td { background: #2a2a2a; }
    #runTable tr.best  td { color: #0f0; }
    #runTable tr.worst td { color: #a00; }
  </style>
</head>
<body>

<h2>🚗 Line Follower</h2>

<!-- Status -->
<div class="card">
  <div class="stat-row">
    <div class="stat"><label>Linje funnet</label><span id="lineOk">-</span></div>
    <div class="stat"><label>Posisjon</label><span id="pos">-</span></div>
    <div class="stat"><label>Smooth pos</label><span id="smoothPos">-</span></div>
    <div class="stat"><label>Feil</label><span id="err">-</span></div>
    <div class="stat"><label>Retning</label><span id="dir">-</span></div>
  </div>
  <!-- FIX: removed onpointerdown from status card — bypassed toggleFollow() and desynced UI -->
  <div class="stat"><label>Status</label><span id="followStatus">-</span></div>
  <div id="pos-track">
    <div id="center-mark"></div>
    <div id="pos-dot"></div>
  </div>
  <div id="barsRaw"></div>
  <div id="barsCalib"></div>
  <p style="font-size:0.7em;color:#555;margin-top:8px;">Cal Min</p>
  <div id="barsMin"></div>
  <p style="font-size:0.7em;color:#555;margin-top:8px;">Cal Max</p>
  <div id="barsMax"></div>
</div>

<!-- Manual drive -->
<div class="card">
  <h3>Manuell kjøring</h3>
  <div class="speed-row">
    <label>Hastighet venstre</label>
    <input type="range" id="spdL" min="50" max="255" value="150">
    <span id="spdValL">150</span>
  </div>
  <div class="speed-row">
    <label>Hastighet høyre</label>
    <input type="range" id="spdR" min="50" max="255" value="150">
    <span id="spdValR">150</span>
  </div>
  <br>
  <div class="btn-grid">
    <div></div>
    <div class="btn" id="btn-fwd"   onpointerdown="drive('fwd')"   onpointerup="drive('stop')" onpointerleave="drive('stop')">▲</div>
    <div></div>
    <div class="btn" id="btn-left"  onpointerdown="drive('left')"  onpointerup="drive('stop')" onpointerleave="drive('stop')">◀</div>
    <div class="btn stop"           onpointerdown="drive('stop')">■</div>
    <div class="btn" id="btn-right" onpointerdown="drive('right')" onpointerup="drive('stop')" onpointerleave="drive('stop')">▶</div>
    <div></div>
    <div class="btn" id="btn-rev"   onpointerdown="drive('rev')"   onpointerup="drive('stop')" onpointerleave="drive('stop')">▼</div>
    <div></div>
  </div>
</div>

<!-- PID Tuning -->
<div class="card">
  <h3>⚙️ PID Tuning</h3>
  <div class="pid-row">
    <label>Kp</label>
    <input type="range"  id="kp"      min="0" max="0.5"   step="0.001"  value="0.08">
    <input type="number" id="kpN"     min="0" max="0.5"   step="0.001"  value="0.08">
  </div>
  <div class="pid-row">
    <label>Ki</label>
    <input type="range"  id="ki"      min="0" max="0.05"  step="0.0001" value="0">
    <input type="number" id="kiN"     min="0" max="0.05"  step="0.0001" value="0">
  </div>
  <div class="pid-row">
    <label>Kd</label>
    <input type="range"  id="kd"      min="0" max="3"     step="0.01"   value="0.4">
    <input type="number" id="kdN"     min="0" max="3"     step="0.01"   value="0.4">
  </div>
  <div class="pid-row">
    <label>Base spd</label>
    <input type="range"  id="baseSpd" min="40" max="220"  step="5"      value="120">
    <input type="number" id="baseSpdN" min="40" max="220" step="5"      value="120">
  </div>
  <div class="pid-row">
    <label>Max spd</label>
    <input type="range"  id="maxSpd"  min="80" max="255"  step="5"      value="220">
    <input type="number" id="maxSpdN" min="80" max="255"  step="5"      value="220">
  </div>
  <div style="display:flex;gap:8px;margin-top:12px;">
    <button class="action-btn" onclick="applyPID()">📡 Apply to robot</button>
    <button class="action-btn go-btn" id="goBtn" onclick="toggleFollow()">▶ Start run</button>
  </div>
  <!-- Live run stats -->
  <div id="runStats" style="display:none;margin-top:12px;">
    <div class="stat-row">
      <div class="stat"><label>Duration</label><span id="mDuration">-</span></div>
      <div class="stat"><label>Avg error</label><span id="mAvgErr">-</span></div>
      <div class="stat"><label>Max error</label><span id="mMaxErr">-</span></div>
      <div class="stat"><label>Line lost</label><span id="mLost">-</span></div>
    </div>
    <div class="stat-row" style="margin-top:6px">
      <div class="stat" style="flex:3">
        <label>Score (lower = better)</label>
        <span id="mScore" style="font-size:1.6em">-</span>
      </div>
    </div>
  </div>
</div>

<!-- Run history -->
<div class="card">
  <h3>📊 Run History</h3>
  <p style="font-size:0.78em;color:#888;margin-bottom:8px;">
    Click any row to reload those params into the sliders.
  </p>
  <div style="overflow-x:auto;">
    <table id="runTable">
      <thead>
        <tr>
          <th>#</th><th>Score</th><th>Kp</th><th>Ki</th><th>Kd</th>
          <th>Base</th><th>Max</th><th>AvgErr</th><th>Lost</th><th>Dur</th>
        </tr>
      </thead>
      <tbody id="runBody"></tbody>
    </table>
  </div>
  <button class="action-btn" style="margin-top:8px;background:#333;border-color:#555"
          onclick="clearHistory()">🗑 Clear history</button>
</div>

<!-- System -->
<div class="card">
  <h3>System</h3>
  <p style="font-size:0.85em;color:#888;margin-bottom:8px;">
    Rekalibrerer sensorene (kjør over linjen under kalibrering)
  </p>
  <button class="btn-red" onclick="doRecalibrate()">🔄 Rekalibrering</button>
  <br><br>
  <button class="btn-red" style="background:#333;border-color:#555;"
          onclick="doReset()">⚡ Reset ESP32</button>
</div>

<script>
  // ── Manual drive sliders ──────────────────────────────────────────
  const spdL    = document.getElementById('spdL');
  const spdR    = document.getElementById('spdR');
  const spdValL = document.getElementById('spdValL');
  const spdValR = document.getElementById('spdValR');
  spdL.oninput = () => spdValL.textContent = spdL.value;
  spdR.oninput = () => spdValR.textContent = spdR.value;

  

  // ── Build sensor bars (done once) ────────────────────────────────
  // FIX: single declaration block — was declared twice causing JS crash
  const N            = 11;
  const barsDivRaw   = document.getElementById('barsRaw');
  const barsDivCalib = document.getElementById('barsCalib');
  let   barElsRaw    = [];
  let   barElsCalib  = [];




  const barsDivMin = document.getElementById('barsMin');
  const barsDivMax = document.getElementById('barsMax');
  let barElsMin = [], barElsMax = [];

  for (let i = 0; i < N; i++) {
    [
      { div: barsDivMin, arr: barElsMin, color: '#f80' },
      { div: barsDivMax, arr: barElsMax, color: '#f0f' }
    ].forEach(({ div, arr, color }) => {
      const w = document.createElement('div'); w.className = 'bar-wrap';
      const b = document.createElement('div'); b.className = 'bar';
      b.style.height = '4px'; b.style.background = color;
      const s = document.createElement('span'); s.textContent = i;
      w.appendChild(b); w.appendChild(s); div.appendChild(w);
      arr.push(b);
    });
  }

  for (let i = 0; i < N; i++) {
    [
      { div: barsDivRaw,   arr: barElsRaw,   color: '#0f0' },
      { div: barsDivCalib, arr: barElsCalib, color: '#0ff' }
    ].forEach(({ div, arr, color }) => {
      const w = document.createElement('div'); w.className = 'bar-wrap';
      const b = document.createElement('div'); b.className = 'bar';
      b.style.height = '4px'; b.style.background = color;
      const s = document.createElement('span'); s.textContent = i;
      w.appendChild(b); w.appendChild(s); div.appendChild(w);
      arr.push(b);
    });
  }

  // ── PID slider ↔ number sync ──────────────────────────────────────
  ['kp','ki','kd','baseSpd','maxSpd'].forEach(id => {
    const slider = document.getElementById(id);
    const number = document.getElementById(id + 'N');
    slider.oninput = () => { number.value = slider.value; };
    number.oninput = () => { slider.value = number.value; };
  });

  function getPIDValues() {
    return {
      kp:        parseFloat(document.getElementById('kp').value),
      ki:        parseFloat(document.getElementById('ki').value),
      kd:        parseFloat(document.getElementById('kd').value),
      baseSpeed: parseInt(document.getElementById('baseSpd').value),
      maxSpeed:  parseInt(document.getElementById('maxSpd').value),
    };
  }

  function setPIDValues(p) {
    const map = { kp:'kp', ki:'ki', kd:'kd', baseSpeed:'baseSpd', maxSpeed:'maxSpd' };
    for (const [key, id] of Object.entries(map)) {
      document.getElementById(id).value  = p[key];
      document.getElementById(id + 'N').value = p[key];
    }
  }

  async function applyPID() {
    const p = getPIDValues();
    await fetch(`/setpid?kp=${p.kp}&ki=${p.ki}&kd=${p.kd}&baseSpeed=${p.baseSpeed}&maxSpeed=${p.maxSpeed}`);
  }

  // ── Follow toggle ─────────────────────────────────────────────────
  let followRunning = false;
  let metricsTimer  = null;

  async function toggleFollow() {
    const btn = document.getElementById('goBtn');

    if (!followRunning) {
      await applyPID();          // push sliders to robot before starting
      await fetch('/following');
      followRunning = true;
      btn.textContent = '⏹ Stop run';
      btn.classList.add('running');
      document.getElementById('runStats').style.display = 'block';
      metricsTimer = setInterval(pollMetrics, 500);
    } else {
      await fetch('/following');
      followRunning = false;
      btn.textContent = '▶ Start run';
      btn.classList.remove('running');
      clearInterval(metricsTimer);
      await pollMetrics(true);   // final snapshot
    }
  }

  async function pollMetrics(isFinal = false) {
    const d = await fetch('/metrics').then(r => r.json());
    document.getElementById('mDuration').textContent = (d.durationMs / 1000).toFixed(1) + 's';
    document.getElementById('mAvgErr').textContent   = d.avgError.toFixed(0);
    document.getElementById('mMaxErr').textContent   = d.maxError;
    document.getElementById('mLost').textContent     = d.lostCount;
    document.getElementById('mScore').textContent    = d.score.toFixed(2);
    if (isFinal && d.samples > 10) saveRun(d);
  }

  // ── Run history ───────────────────────────────────────────────────
  let runHistory = [];

  function saveRun(d) {
    runHistory.unshift({
      id:        runHistory.length + 1,
      score:     d.score,
      kp:        d.kp,
      ki:        d.ki,
      kd:        d.kd,
      baseSpeed: d.baseSpeed,
      maxSpeed:  d.maxSpeed,
      avgError:  d.avgError,
      lostCount: d.lostCount,
      duration:  (d.durationMs / 1000).toFixed(1),
    });
    renderHistory();
  }

  function renderHistory() {
    const tbody  = document.getElementById('runBody');
    tbody.innerHTML = '';
    const scores = runHistory.map(r => r.score);
    const best   = Math.min(...scores);
    const worst  = Math.max(...scores);

    runHistory.forEach(r => {
      const tr = document.createElement('tr');
      if (r.score === best)                       tr.className = 'best';
      if (r.score === worst && runHistory.length > 1) tr.className = 'worst';
      tr.innerHTML = `
        <td>#${r.id}</td>
        <td>${r.score.toFixed(2)}</td>
        <td>${r.kp}</td><td>${r.ki}</td><td>${r.kd}</td>
        <td>${r.baseSpeed}</td><td>${r.maxSpeed}</td>
        <td>${r.avgError.toFixed(0)}</td>
        <td>${r.lostCount}</td>
        <td>${r.duration}s</td>`;
      tr.onclick = () => {
        setPIDValues(r);
        window.scrollTo({ top: 0, behavior: 'smooth' });
      };
      tbody.appendChild(tr);
    });
  }

  function clearHistory() {
    if (!confirm('Clear all run history?')) return;
    runHistory = [];
    renderHistory();
  }

  // ── Sensor poll ───────────────────────────────────────────────────
  setInterval(() => {
    fetch('/data').then(r => r.json()).then(d => {
      document.getElementById('lineOk').textContent       = d.lineDetected ? '✅' : '❌';
      document.getElementById('followStatus').textContent = d.following ? '🟢 Following' : '🔴 Stopped';
      document.getElementById('pos').textContent          = d.position;
      document.getElementById('err').textContent          = d.error;
      const dirMap = { '-1':'◀ Venstre', '0':'▲ Rett', '1':'▶ Høyre' };
      document.getElementById('dir').textContent          = dirMap[String(d.direction)] || '-';
      document.getElementById('smoothPos').textContent = d.smoothPos;

      d.sensorsRaw.forEach((v, i) => {
        if (!barElsRaw[i]) return;
        barElsRaw[i].style.height = Math.max(2, Math.round(v * 74 / 2500)) + 'px';
        barElsRaw[i].parentElement.querySelector('span').textContent = v;
      });
      d.sensorsCalib.forEach((v, i) => {
        if (!barElsCalib[i]) return;
        barElsCalib[i].style.height = Math.max(2, Math.round(v * 74 / 1000)) + 'px';
        barElsCalib[i].parentElement.querySelector('span').textContent = v;
      });

      if (d.calMin) d.calMin.forEach((v, i) => {
        if (!barElsMin[i]) return;
        barElsMin[i].style.height = Math.max(2, Math.round(v * 74 / 2500)) + 'px';
        barElsMin[i].parentElement.querySelector('span').textContent = v;
      });
      if (d.calMax) d.calMax.forEach((v, i) => {
        if (!barElsMax[i]) return;
        barElsMax[i].style.height = Math.max(2, Math.round(v * 74 / 2500)) + 'px';
        barElsMax[i].parentElement.querySelector('span').textContent = v;
      });

      const pct = d.position / ((N - 1) * 1000) * 100;
      document.getElementById('pos-dot').style.left = pct.toFixed(1) + '%';
    }).catch(() => {});
  }, 120);

  // ── Manual drive commands ─────────────────────────────────────────
  function drive(cmd) {
    fetch('/drive?cmd=' + cmd + '&spdL=' + spdL.value + '&spdR=' + spdR.value).catch(() => {});
  }

  function doRecalibrate() {
    if (!confirm('Start rekalibrering? Kjør bilen over linjen i 4 sekunder.')) return;
    fetch('/recalibrate');
  }

  function doReset() {
    if (!confirm('Reset ESP32?')) return;
    fetch('/reset');
  }
</script>
</body>
</html>
)rawliteral";
  server.send(200, "text/html", html);
}

// ── Web – JSON data ───────────────────────────────────────────────
void handleData() {
  String json = "{";
  json += "\"position\":"     + String(linePosition)              + ",";
  json += "\"smoothPos\":"    + String((int)smoothPosition)       + ",";
  json += "\"center\":"       + String(LINE_CENTER)               + ",";
  json += "\"error\":"        + String(lineError())               + ",";
  json += "\"direction\":"    + String(lineDirection())           + ",";
  json += "\"following\":"    + String(isFollowing ? "true" : "false") + ",";
  json += "\"lineDetected\":" + String(lineDetected ? "true" : "false") + ",";
  json += "\"sensorsCalib\":[";
  for (int i = 0; i < SENSOR_COUNT; i++) {
    json += String(calibValues[i]);
    if (i < SENSOR_COUNT - 1) json += ",";
  }
  json += "],";
  json += "\"sensorsRaw\":[";
  for (int i = 0; i < SENSOR_COUNT; i++) {
    json += String(sensorValues[i]);
    if (i < SENSOR_COUNT - 1) json += ",";
  }
  json += "]";
  json += ",\"calMin\":[";
  for (int i = 0; i < SENSOR_COUNT; i++) {
    json += String(qtr.calibrationOn.minimum[i]);
    if (i < SENSOR_COUNT - 1) json += ",";
  }
  json += "],\"calMax\":[";
  for (int i = 0; i < SENSOR_COUNT; i++) {
    json += String(qtr.calibrationOn.maximum[i]);
    if (i < SENSOR_COUNT - 1) json += ",";
  }
 
  json += "]}";
  server.send(200, "application/json", json);
}

// ── Web – Set PID ─────────────────────────────────────────────────
void handleSetPID() {
  if (server.hasArg("kp"))        pid.kp        = server.arg("kp").toFloat();
  if (server.hasArg("ki"))        pid.ki        = server.arg("ki").toFloat();
  if (server.hasArg("kd"))        pid.kd        = server.arg("kd").toFloat();
  if (server.hasArg("baseSpeed")) pid.baseSpeed = server.arg("baseSpeed").toInt();
  if (server.hasArg("maxSpeed"))  pid.maxSpeed  = server.arg("maxSpeed").toInt();
  resetPID();
  server.send(200, "text/plain", "OK");
}

// ── Web – Metrics ─────────────────────────────────────────────────
void handleMetrics() {
  float avgError = run.samples > 0 ? (float)run.errorSum   / run.samples : 0;
  float variance = run.samples > 0 ? (float)run.errorSqSum / run.samples : 0;
  float score    = avgError * 0.04f + sqrt(variance) * 0.02f + run.lostCount * 50.0f;

  String json = "{";
  json += "\"durationMs\":"  + String(run.durationMs)     + ",";
  json += "\"avgError\":"    + String(avgError,  1)        + ",";
  json += "\"maxError\":"    + String(run.maxError)        + ",";
  json += "\"variance\":"    + String(variance,  1)        + ",";
  json += "\"lostCount\":"   + String(run.lostCount)       + ",";
  json += "\"samples\":"     + String(run.samples)         + ",";
  json += "\"score\":"       + String(score,     2)        + ",";
  json += "\"kp\":"          + String(pid.kp,    4)        + ",";
  json += "\"ki\":"          + String(pid.ki,    4)        + ",";
  json += "\"kd\":"          + String(pid.kd,    4)        + ",";
  json += "\"baseSpeed\":"   + String(pid.baseSpeed)       + ",";
  json += "\"maxSpeed\":"    + String(pid.maxSpeed)        + "}";
  server.send(200, "application/json", json);
}

// ── Web – Manual drive ────────────────────────────────────────────
void handleDrive() {
  String cmd = server.arg("cmd");
  int spdL   = server.arg("spdL").toInt();
  int spdR   = server.arg("spdR").toInt();
  if (spdL == 0) spdL = 150;
  if (spdR == 0) spdR = 150;

  if      (cmd == "fwd")   setMotors( spdL,  spdR);
  else if (cmd == "rev")   setMotors(-spdL, -spdR);
  else if (cmd == "left")  setMotors(-spdL,  spdR);
  else if (cmd == "right") setMotors( spdL, -spdR);
  else                     stopMotors();

  server.send(200, "text/plain", "OK");
}

// ── Web – Following toggle ────────────────────────────────────────
void handleFollowing() {
  isFollowing = !isFollowing;
  if (isFollowing) {
    resetPID();
    run         = RunMetrics();
    run.startMs = millis();
  } else {
    run.durationMs = millis() - run.startMs;
    stopMotors();
  }
  server.send(200, "text/plain", "OK");
}

// ── Web – Recalibrate ─────────────────────────────────────────────
void handleRecalibrate() {
  server.send(200, "text/plain", "Kalibrerer...");
  digitalWrite(LED_BLUE, LOW);

  const int CAL_SPEED = 90;
  const int SAMPLES   = 400;
  const int PHASE_LEN = 30;

  for (uint16_t i = 0; i < SAMPLES; i++) {
    int phase = (i / PHASE_LEN) % 4;
    if      (phase == 0) setMotors(-CAL_SPEED,  CAL_SPEED);
    else if (phase == 1) setMotors( CAL_SPEED, -CAL_SPEED);
    else if (phase == 2) setMotors(-CAL_SPEED,  CAL_SPEED);
    else                 setMotors( CAL_SPEED, -CAL_SPEED);
    qtr.calibrate();
    delay(15);
  }
  stopMotors();
  digitalWrite(LED_BLUE, HIGH);
}

// ── Web – Reset ───────────────────────────────────────────────────
void handleReset() {
  server.send(200, "text/plain", "Restarting...");
  delay(300);
  ESP.restart();
}


// ── Setup ─────────────────────────────────────────────────────────
void setup() {
  setupMotors();

  pinMode(LED_RED,   OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE,  OUTPUT);
  digitalWrite(LED_RED,   HIGH);
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_BLUE,  HIGH);

  for (int i = 0; i < 10; i++) {
    digitalWrite(LED_RED, LOW);  delay(100);
    digitalWrite(LED_RED, HIGH); delay(100);
  }

  startWiFi();
  startOTA();
  digitalWrite(STBY, HIGH);

  pinMode(EMITTER_ODD, OUTPUT);
  digitalWrite(EMITTER_ODD, HIGH);

  qtr.setTypeRC();
  qtr.setSensorPins(qtrPins, SENSOR_COUNT);
  delay(500);

  // Initial calibration
  digitalWrite(LED_BLUE, LOW);
  for (uint16_t i = 0; i < 200; i++) qtr.calibrate();
  digitalWrite(LED_BLUE, HIGH);

  lastKnownPosition = LINE_CENTER; 

  digitalWrite(LED_GREEN, LOW);
}

// ── Loop ──────────────────────────────────────────────────────────
void loop() {
  server.handleClient();
  ArduinoOTA.handle();

  readSensors(); 

  if (isFollowing) {
    run.durationMs = millis() - run.startMs;
    if (lineDetected) {
      runPID();
    } else {
      //int dir = (lastKnownPosition < LINE_CENTER) ? -1 : 1;
      //setMotors(dir * -70, dir * 70);
    }
  }
}

