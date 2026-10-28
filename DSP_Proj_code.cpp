/**
 * ============================================================================
 * DUAL MPU-6050 SEISMOGRAPH — REAL-TIME DATA ACQUISITION, DSP & WEB DASHBOARD
 * Target Board : ESP32 (doit-esp32-devkit-v1 / esp32dev)
 * Framework    : Arduino on PlatformIO IDE (No OLED — Serial Monitor IP Output)
 *
 * Architecture:
 *   1. Dual MPU-6050 I2C Acquisition at 100 Hz (Ts = 10 ms):
 *      - MPU-6050 #1 (Surface Sensor)   : I2C Address 0x68 (AD0 pin -> GND)
 *      - MPU-6050 #2 (Borehole Sensor)  : I2C Address 0x69 (AD0 pin -> 3.3V)
 *   2. Sensor Fusion (Ambient Surface Noise Rejection & Complementary Tilt Compensation):
 *      - Computes vertical seismic acceleration from both sensors and applies
 *        differential subtraction (Borehole - Surface) to isolate subterranean
 *        P/S waves from surface cultural interference (footsteps, traffic).
 *   3. Cascaded Embedded DSP Pipeline:
 *      - Stage A: 1st-Order IIR High-Pass Filter (DC & 1g Gravity Drift Removal)
 *        y_hp[n] = alpha * (y_hp[n-1] + x[n] - x[n-1]),  alpha = 0.985
 *      - Stage B: 2nd-Order Butterworth Low-Pass Filter (fc = 10 Hz @ fs = 100 Hz)
 *        Zero-ripple passband filter to remove high-frequency electrical/structural
 *        noise while preserving true physical seismic wave amplitudes.
 *   4. PlatformIO Serial Monitor (115200 Baud):
 *      - Prints the Webpage URL (http://<IP_ADDRESS>) at startup AND repeats it
 *        every 3 seconds so you can open the Serial Monitor at any time to see
 *        the IP address and sensor status.
 *      - Send 'c' in the Serial Monitor to toggle 100 Hz CSV streaming for Simulink.
 *   5. Embedded Highcharts Web Server (Port 80):
 *      - Graph 1 (Combined) : MPU-6050 #1 (Surface) & MPU-6050 #2 (Borehole) vs. Time
 *      - Graph 2 (Separate) : Sensor Fusion Output vs. Time
 *      - Graph 3 (Separate) : IIR High-Pass + Butterworth Low-Pass Filtered vs. Time
 * ============================================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>

// ============================================================================
// 1. WI-FI CREDENTIALS (Update with your router / mobile hotspot details)
//    If connection fails after 12 seconds, ESP32 automatically starts an
//    Access Point ("ESP32-Seismograph") at IP 192.168.4.1 and prints it to Serial.
// ============================================================================
const char* WIFI_SSID     = "YOUR_WIFI_SSID";
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";

const char* AP_FALLBACK_SSID = "ESP32-Seismograph";
const char* AP_FALLBACK_PASS = "seismic123";

// ============================================================================
// 2. HARDWARE PIN & I2C ADDRESS CONFIGURATION (ESP32 Default I2C)
// ============================================================================
#define I2C_SDA_PIN         21
#define I2C_SCL_PIN         22

#define MPU1_I2C_ADDR       0x68    // Surface MPU-6050  (AD0 tied to GND)
#define MPU2_I2C_ADDR       0x69    // Borehole MPU-6050 (AD0 tied to 3.3V)

// ============================================================================
// 3. SAMPLING & DSP FILTER CONSTANTS (100 Hz Sampling Rate)
// ============================================================================
#define SAMPLE_RATE_HZ      100.0f
#define SAMPLE_INTERVAL_US  10000UL // 10,000 us = 10 ms (100 Hz)
#define SERIAL_IP_PRINT_MS  3000UL  // Re-print Webpage IP to Serial Monitor every 3s

// Stage 1: 1st-Order IIR High-Pass Filter (Slide 4: alpha between 0.98 and 0.99)
// Difference equation: y_hp[n] = ALPHA * (y_hp[n-1] + x[n] - x[n-1])
// At fs = 100 Hz and alpha = 0.985, cutoff fc = ~0.24 Hz (removes 9.81 m/s^2 gravity & thermal drift)
static const float IIR_HP_ALPHA = 0.985f;

// Stage 2: 2nd-Order Butterworth Low-Pass Filter (fs = 100 Hz, fc = 10 Hz)
// Bilinear transform exact coefficients for flat zero-ripple passband:
// H(z) = (b0 + b1*z^-1 + b2*z^-2) / (1 + a1*z^-1 + a2*z^-2)
static const float BW_B0 =  0.06745527f;
static const float BW_B1 =  0.13491055f;
static const float BW_B2 =  0.06745527f;
static const float BW_A1 = -1.14298050f;
static const float BW_A2 =  0.41280160f;

// ============================================================================
// 4. GLOBAL OBJECTS & DSP STATE VARIABLES
// ============================================================================
Adafruit_MPU6050 mpuSurface;   // Sensor 1 (0x68)
Adafruit_MPU6050 mpuBorehole;  // Sensor 2 (0x69)
WebServer server(80);

bool mpu1Connected     = false;
bool mpu2Connected     = false;
bool forceSimMode      = false; // Can be toggled via Web UI if testing without sensors
bool serialCsvMode     = false; // False = print Webpage IP every 3s; True = 100 Hz Simulink CSV
String activeIP        = "0.0.0.0";
String wifiModeStr     = "Connecting";

// Complementary / Differential Sensor Fusion state
float fusedRoll1 = 0.0f, fusedPitch1 = 0.0f;
float fusedRoll2 = 0.0f, fusedPitch2 = 0.0f;

// IIR High-Pass filter state variables
float hp_prev_x = 0.0f;
float hp_prev_y = 0.0f;

// 2nd-Order Butterworth Low-Pass filter state variables (Direct Form I)
float lp_x1 = 0.0f, lp_x2 = 0.0f;
float lp_y1 = 0.0f, lp_y2 = 0.0f;

// Circular Ring Buffer to stream 100 Hz samples cleanly to Highcharts without drops
#define RING_BUFFER_SIZE 64
struct SeismicSample {
  uint32_t seq;       // Monotonic sample sequence ID
  uint32_t time_ms;   // Timestamp in ms
  float mpu1_raw;     // Input 1: Surface MPU-6050 acceleration (m/s^2)
  float mpu2_raw;     // Input 2: Borehole MPU-6050 acceleration (m/s^2)
  float fused_val;    // 3rd Graph: Sensor Fusion output (m/s^2)
  float filtered_val; // 4th Graph: IIR High-Pass + Butterworth Low-Pass output (m/s^2)
};

SeismicSample sampleBuffer[RING_BUFFER_SIZE];
volatile uint32_t sampleSeq = 0;
uint32_t lastSampleMicros   = 0;
uint32_t lastSerialIpMillis = 0;
float peakFilteredAccel     = 0.0f;

// ============================================================================
// 5. EMBEDDED HIGHCHARTS WEB DASHBOARD (PROGMEM HTML/CSS/JS)
// ============================================================================
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1.0" />
  <title>ESP32 Dual MPU-6050 Seismograph — Highcharts DSP Dashboard</title>
  <script src="https://code.highcharts.com/highcharts.js"></script>
  <script src="https://code.highcharts.com/modules/exporting.js"></script>
  <style>
    :root {
      --bg: #0b1120;
      --card: #111827;
      --border: #1f2937;
      --text: #f3f4f6;
      --muted: #9ca3af;
      --accent1: #38bdf8;
      --accent2: #f59e0b;
      --accent3: #a855f7;
      --accent4: #10b981;
    }
    * { box-sizing: border-box; margin: 0; padding: 0; }
    body {
      font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif;
      background-color: var(--bg);
      color: var(--text);
      padding: 18px;
    }
    header {
      display: flex;
      flex-wrap: wrap;
      justify-content: space-between;
      align-items: center;
      gap: 12px;
      background: var(--card);
      border: 1px solid var(--border);
      border-radius: 12px;
      padding: 16px 20px;
      margin-bottom: 18px;
    }
    .title-group h1 {
      font-size: 1.35rem;
      font-weight: 700;
      letter-spacing: 0.3px;
    }
    .title-group p {
      color: var(--muted);
      font-size: 0.85rem;
      margin-top: 4px;
    }
    .controls {
      display: flex;
      flex-wrap: wrap;
      gap: 8px;
      align-items: center;
    }
    .badge {
      padding: 6px 12px;
      border-radius: 999px;
      font-size: 0.78rem;
      font-weight: 600;
      background: #1e293b;
      border: 1px solid #334155;
    }
    .badge.live { color: #34d399; border-color: #059669; }
    button {
      background: #1e293b;
      color: var(--text);
      border: 1px solid #334155;
      padding: 7px 13px;
      border-radius: 8px;
      cursor: pointer;
      font-size: 0.82rem;
      font-weight: 600;
      transition: background 0.15s;
    }
    button:hover { background: #334155; }
    .kpi-grid {
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
      gap: 12px;
      margin-bottom: 18px;
    }
    .kpi-card {
      background: var(--card);
      border: 1px solid var(--border);
      border-radius: 10px;
      padding: 12px 16px;
    }
    .kpi-label { font-size: 0.76rem; color: var(--muted); text-transform: uppercase; }
    .kpi-val { font-size: 1.4rem; font-weight: 700; margin-top: 4px; font-variant-numeric: tabular-nums; }
    .chart-card {
      background: var(--card);
      border: 1px solid var(--border);
      border-radius: 12px;
      padding: 14px;
      margin-bottom: 18px;
    }
    .chart-box { width: 100%; height: 320px; }
  </style>
</head>
<body>

  <header>
    <div class="title-group">
      <h1>Dual MPU-6050 Seismograph — Real-Time DSP Telemetry</h1>
      <p>ESP32 100 Hz Acquisition &bull; Borehole/Surface Sensor Fusion &bull; IIR High-Pass (&alpha;=0.985) + 2nd-Order Butterworth Low-Pass (fc=10 Hz)</p>
    </div>
    <div class="controls">
      <span id="statusBadge" class="badge live">LIVE 100 Hz</span>
      <span id="sensorBadge" class="badge">Checking I2C...</span>
      <button id="btnPause" onclick="togglePause()">Pause</button>
      <button onclick="toggleMode()">Toggle HW / Simulink Test Wave</button>
      <button onclick="clearCharts()">Clear Buffers</button>
    </div>
  </header>

  <div class="kpi-grid">
    <div class="kpi-card">
      <div class="kpi-label">MPU-6050 #1 (Surface 0x68)</div>
      <div class="kpi-val" id="valMpu1" style="color: var(--accent1);">0.000 m/s²</div>
    </div>
    <div class="kpi-card">
      <div class="kpi-label">MPU-6050 #2 (Borehole 0x69)</div>
      <div class="kpi-val" id="valMpu2" style="color: var(--accent2);">0.000 m/s²</div>
    </div>
    <div class="kpi-card">
      <div class="kpi-label">Sensor Fusion Output</div>
      <div class="kpi-val" id="valFused" style="color: var(--accent3);">0.000 m/s²</div>
    </div>
    <div class="kpi-card">
      <div class="kpi-label">IIR HPF + Butterworth LPF (Drift-Free)</div>
      <div class="kpi-val" id="valFiltered" style="color: var(--accent4);">0.000 m/s²</div>
    </div>
  </div>

  <!-- GRAPH 1: Combined Input Values of Both MPU-6050s -->
  <div class="chart-card">
    <div id="chartInputs" class="chart-box"></div>
  </div>

  <!-- GRAPH 2: Separate Graph for Sensor Fusion Output -->
  <div class="chart-card">
    <div id="chartFusion" class="chart-box"></div>
  </div>

  <!-- GRAPH 3: Separate Graph for IIR High-Pass + Butterworth Low-Pass Filter -->
  <div class="chart-card">
    <div id="chartFiltered" class="chart-box"></div>
  </div>

<script>
  Highcharts.setOptions({
    time: { useUTC: false },
    chart: {
      backgroundColor: 'transparent',
      style: { fontFamily: 'inherit' },
      animation: false,
      zoomType: 'x'
    },
    title: { style: { color: '#f3f4f6', fontSize: '15px', fontWeight: '600' }, align: 'left' },
    subtitle: { style: { color: '#9ca3af', fontSize: '12px' }, align: 'left' },
    xAxis: {
      type: 'datetime',
      gridLineColor: '#1f2937',
      lineColor: '#374151',
      tickColor: '#374151',
      labels: { style: { color: '#9ca3af' } },
      title: { text: 'Time (hh:mm:ss.ms)', style: { color: '#9ca3af' } }
    },
    yAxis: {
      gridLineColor: '#1f2937',
      labels: { style: { color: '#9ca3af' } },
      title: { text: 'Acceleration (m/s²)', style: { color: '#9ca3af' } }
    },
    legend: { itemStyle: { color: '#e5e7eb' }, itemHoverStyle: { color: '#ffffff' } },
    credits: { enabled: false },
    plotOptions: {
      series: {
        marker: { enabled: false },
        lineWidth: 2,
        states: { hover: { lineWidthPlus: 0 } }
      }
    }
  });

  // 1. Combined Graph: MPU-6050 #1 (Surface) & MPU-6050 #2 (Borehole)
  const chartInputs = Highcharts.chart('chartInputs', {
    title: { text: 'Graph 1 (Combined Inputs): MPU-6050 #1 (Surface) & MPU-6050 #2 (Borehole) Raw Values vs. Time' },
    subtitle: { text: 'Unprocessed dual-sensor vertical acceleration inputs (includes 9.81 m/s² gravity bias & surface interference)' },
    tooltip: { shared: true, valueDecimals: 4, valueSuffix: ' m/s²' },
    series: [
      { name: 'MPU-6050 #1 (Surface - 0x68)', color: '#38bdf8', data: [] },
      { name: 'MPU-6050 #2 (Borehole - 0x69)', color: '#f59e0b', data: [] }
    ]
  });

  // 2. Separate Graph: 3rd Value — Sensor Fusion Output
  const chartFusion = Highcharts.chart('chartFusion', {
    title: { text: 'Graph 2 (Sensor Fusion): Differential & Tilt-Compensated Fused Seismic Signal vs. Time' },
    subtitle: { text: 'Fused Borehole-Surface response isolating subterranean P/S waves and rejecting ambient surface vibrations' },
    tooltip: { valueDecimals: 4, valueSuffix: ' m/s²' },
    series: [
      { name: 'Sensor Fusion Output', color: '#a855f7', data: [] }
    ]
  });

  // 3. Separate Graph: 4th Value — IIR High-Pass + Butterworth Low-Pass Filtered Output
  const chartFiltered = Highcharts.chart('chartFiltered', {
    title: { text: 'Graph 3 (DSP Filtered): IIR High-Pass (α=0.985) + 2nd-Order Butterworth Low-Pass (fc=10 Hz) vs. Time' },
    subtitle: { text: 'Zero-drift, gravity-eliminated, anti-aliased seismic waveform: y[n] = α(y[n-1] + x[n] - x[n-1]) → Butterworth Biquad LPF' },
    tooltip: { valueDecimals: 4, valueSuffix: ' m/s²' },
    yAxis: {
      plotLines: [{ value: 0, color: '#4b5563', width: 1, dashStyle: 'Dash' }]
    },
    series: [
      { name: 'Filtered Seismic Signal (Drift Eliminated)', color: '#10b981', data: [] }
    ]
  });

  let isPaused = false;
  let lastSeq = 0;
  const MAX_POINTS = 300; // 3 seconds of 100 Hz high-resolution scrolling window

  function togglePause() {
    isPaused = !isPaused;
    document.getElementById('btnPause').innerText = isPaused ? 'Resume' : 'Pause';
    const badge = document.getElementById('statusBadge');
    badge.innerText = isPaused ? 'PAUSED' : 'LIVE 100 Hz';
    badge.className = isPaused ? 'badge' : 'badge live';
  }

  function clearCharts() {
    chartInputs.series[0].setData([], false);
    chartInputs.series[1].setData([], true);
    chartFusion.series[0].setData([], true);
    chartFiltered.series[0].setData([], true);
  }

  async function toggleMode() {
    try {
      await fetch('/api/toggle_sim', { method: 'POST' });
    } catch (e) {
      console.error(e);
    }
  }

  async function pollTelemetry() {
    if (!isPaused) {
      try {
        const res = await fetch('/api/data?since=' + lastSeq);
        if (res.ok) {
          const payload = await res.json();
          const sensorBadge = document.getElementById('sensorBadge');
          sensorBadge.innerText = payload.mode;

          const samples = payload.samples;
          if (samples && samples.length > 0) {
            const nowBase = Date.now();
            const latestSampleTime = samples[samples.length - 1].t;

            for (let i = 0; i < samples.length; i++) {
              const s = samples[i];
              if (s.seq <= lastSeq && lastSeq !== 0) continue;
              lastSeq = s.seq;

              // Map ESP32 sample timestamp onto browser timeline
              const tEpoch = nowBase - (latestSampleTime - s.t);
              const shift1 = chartInputs.series[0].data.length >= MAX_POINTS;
              const shift2 = chartFusion.series[0].data.length >= MAX_POINTS;
              const shift3 = chartFiltered.series[0].data.length >= MAX_POINTS;

              chartInputs.series[0].addPoint([tEpoch, s.m1], false, shift1);
              chartInputs.series[1].addPoint([tEpoch, s.m2], false, shift1);
              chartFusion.series[0].addPoint([tEpoch, s.fu], false, shift2);
              chartFiltered.series[0].addPoint([tEpoch, s.fi], false, shift3);
            }

            chartInputs.redraw();
            chartFusion.redraw();
            chartFiltered.redraw();

            const last = samples[samples.length - 1];
            document.getElementById('valMpu1').innerText = last.m1.toFixed(3) + ' m/s²';
            document.getElementById('valMpu2').innerText = last.m2.toFixed(3) + ' m/s²';
            document.getElementById('valFused').innerText = last.fu.toFixed(3) + ' m/s²';
            document.getElementById('valFiltered').innerText = last.fi.toFixed(4) + ' m/s²';
          }
        }
      } catch (err) {
        console.warn('Telemetry poll error:', err);
      }
    }
    setTimeout(pollTelemetry, 100);
  }

  pollTelemetry();
</script>
</body>
</html>
)rawliteral";

// ============================================================================
// 6. HELPER FUNCTIONS: SERIAL MONITOR IP BANNER, SENSOR FUSION & DSP FILTERS
// ============================================================================

/**
 * Prints the Webpage IP Address, Wi-Fi mode, and Dual MPU-6050 status clearly
 * to the PlatformIO Serial Monitor (115200 baud).
 */
void printSerialStatusBanner() {
  Serial.println(F("============================================================"));
  Serial.print(F("  WEBPAGE URL (Open in Browser) : http://"));
  Serial.println(activeIP);
  Serial.print(F("  Wi-Fi Mode                    : "));
  Serial.println(wifiModeStr);
  Serial.print(F("  MPU-6050 #1 (Surface  0x68)   : "));
  Serial.println(mpu1Connected ? F("CONNECTED (OK)") : F("NOT DETECTED (Simulating)"));
  Serial.print(F("  MPU-6050 #2 (Borehole 0x69)   : "));
  Serial.println(mpu2Connected ? F("CONNECTED (OK)") : F("NOT DETECTED (Simulating)"));
  Serial.print(F("  Peak Filtered Acceleration    : "));
  Serial.print(peakFilteredAccel, 4);
  Serial.println(F(" m/s^2"));
  Serial.println(F("  [Tip: Send 'c' in Serial Monitor to toggle 100Hz CSV stream]"));
  Serial.println(F("============================================================"));
}

/**
 * Computes tilt-compensated vertical acceleration & differential sensor fusion
 * between MPU-6050 #1 (Surface) and MPU-6050 #2 (Borehole).
 */
float computeSensorFusion(
    float ax1, float ay1, float az1, float gx1, float gy1,
    float ax2, float ay2, float az2, float gx2, float gy2,
    float dt, float &outMpu1Vert, float &outMpu2Vert) {

  // Complementary filter (98% gyro integration + 2% accelerometer angle)
  float accRoll1  = atan2f(ay1, az1);
  float accPitch1 = atan2f(-ax1, sqrtf(ay1 * ay1 + az1 * az1));
  fusedRoll1  = 0.98f * (fusedRoll1  + gx1 * dt) + 0.02f * accRoll1;
  fusedPitch1 = 0.98f * (fusedPitch1 + gy1 * dt) + 0.02f * accPitch1;

  float accRoll2  = atan2f(ay2, az2);
  float accPitch2 = atan2f(-ax2, sqrtf(ay2 * ay2 + az2 * az2));
  fusedRoll2  = 0.98f * (fusedRoll2  + gx2 * dt) + 0.02f * accRoll2;
  fusedPitch2 = 0.98f * (fusedPitch2 + gy2 * dt) + 0.02f * accPitch2;

  // Project 3-axis acceleration onto the true earth-vertical axis
  outMpu1Vert = -ax1 * sinf(fusedPitch1)
                + ay1 * sinf(fusedRoll1) * cosf(fusedPitch1)
                + az1 * cosf(fusedRoll1) * cosf(fusedPitch1);

  outMpu2Vert = -ax2 * sinf(fusedPitch2)
                + ay2 * sinf(fusedRoll2) * cosf(fusedPitch2)
                + az2 * cosf(fusedRoll2) * cosf(fusedPitch2);

  // Differential Borehole-Surface Fusion (Slide 3):
  // Cancels common-mode surface cultural noise (footsteps/traffic on Sensor 1)
  // while retaining subterranean P/S wave energy arriving at Borehole Sensor 2.
  float surfaceAC = outMpu1Vert - 9.80665f;
  float fusedSignal = outMpu2Vert - (0.5f * surfaceAC);
  return fusedSignal;
}

/**
 * Cascaded Embedded DSP Filter (Slide 4 & Slide 5):
 *   1. First-order IIR High-Pass Filter:
 *      y_hp[n] = alpha * (y_hp[n-1] + x[n] - x[n-1]),  alpha = 0.985
 *   2. Second-order Butterworth Low-Pass Filter (fc = 10 Hz @ fs = 100 Hz):
 *      y_lp[n] = b0*y_hp[n] + b1*y_hp[n-1] + b2*y_hp[n-2] - a1*y_lp[n-1] - a2*y_lp[n-2]
 */
float applyCascadedDSP(float x_in) {
  // Stage 1: IIR High-Pass Filter (eliminates 9.81 m/s^2 gravity & thermal drift)
  float y_hp = IIR_HP_ALPHA * (hp_prev_y + x_in - hp_prev_x);
  hp_prev_x = x_in;
  hp_prev_y = y_hp;

  // Stage 2: 2nd-Order Butterworth Low-Pass Filter (eliminates high-freq noise)
  float y_lp = (BW_B0 * y_hp) + (BW_B1 * lp_x1) + (BW_B2 * lp_x2)
             - (BW_A1 * lp_y1) - (BW_A2 * lp_y2);

  lp_x2 = lp_x1;
  lp_x1 = y_hp;
  lp_y2 = lp_y1;
  lp_y1 = y_lp;

  return y_lp;
}

/**
 * Acquires one 100 Hz sample from both MPU-6050 sensors (or generates the
 * exact Simulink benchmark signal from Slide 5 if sensors are unplugged).
 */
void sampleSeismograph() {
  const float dt = 1.0f / SAMPLE_RATE_HZ;
  uint32_t nowMs = millis();
  float tSec = nowMs * 0.001f;

  float mpu1Val  = 0.0f;
  float mpu2Val  = 0.0f;
  float fusedVal = 0.0f;

  bool useHardware = (mpu1Connected && mpu2Connected && !forceSimMode);

  if (useHardware) {
    sensors_event_t a1, g1, temp1;
    sensors_event_t a2, g2, temp2;
    mpuSurface.getEvent(&a1, &g1, &temp1);
    mpuBorehole.getEvent(&a2, &g2, &temp2);

    fusedVal = computeSensorFusion(
      a1.acceleration.x, a1.acceleration.y, a1.acceleration.z, g1.gyro.x, g1.gyro.y,
      a2.acceleration.x, a2.acceleration.y, a2.acceleration.z, g2.gyro.x, g2.gyro.y,
      dt, mpu1Val, mpu2Val
    );
  } else if (mpu1Connected && !forceSimMode) {
    sensors_event_t a1, g1, temp1;
    mpuSurface.getEvent(&a1, &g1, &temp1);
    mpu1Val  = a1.acceleration.z;
    mpu2Val  = a1.acceleration.z;
    fusedVal = a1.acceleration.z;
  } else {
    // Simulink Test Model (Slide 5):
    // - 5 Hz Seismic Wave: sin(10 * pi * t)
    // - High-Frequency Surface Noise + 9.8 m/s^2 Steady-State Gravity Bias
    float seismicWave     = 1.2f * sinf(10.0f * PI * tSec) * expf(-fmodf(tSec, 4.0f) * 0.6f);
    float highFreqNoise   = 0.45f * sinf(40.0f * PI * tSec);
    float surfaceFootstep = 0.8f * sinf(24.0f * PI * tSec);

    mpu1Val  = 9.80f + seismicWave + surfaceFootstep + highFreqNoise;
    mpu2Val  = 9.80f + (1.35f * seismicWave) + (0.15f * surfaceFootstep) + (0.3f * highFreqNoise);
    fusedVal = mpu2Val - 0.5f * (mpu1Val - 9.80f);
  }

  // Apply Cascaded IIR High-Pass + Butterworth Low-Pass Filter
  float filteredVal = applyCascadedDSP(fusedVal);

  // Track peak filtered acceleration
  if (fabsf(filteredVal) > peakFilteredAccel) {
    peakFilteredAccel = fabsf(filteredVal);
  } else {
    peakFilteredAccel *= 0.995f;
  }

  // Store in circular buffer for Highcharts web server
  uint32_t nextSeq = sampleSeq + 1;
  uint8_t idx = nextSeq % RING_BUFFER_SIZE;
  sampleBuffer[idx].seq          = nextSeq;
  sampleBuffer[idx].time_ms      = nowMs;
  sampleBuffer[idx].mpu1_raw     = mpu1Val;
  sampleBuffer[idx].mpu2_raw     = mpu2Val;
  sampleBuffer[idx].fused_val    = fusedVal;
  sampleBuffer[idx].filtered_val = filteredVal;
  sampleSeq = nextSeq;

  // Optional 100 Hz CSV output for Simulink (when toggled on with 'c' in Serial Monitor)
  if (serialCsvMode) {
    Serial.print(mpu1Val, 4);
    Serial.print(',');
    Serial.print(mpu2Val, 4);
    Serial.print(',');
    Serial.print(fusedVal, 4);
    Serial.print(',');
    Serial.println(filteredVal, 4);
  }
}

// ============================================================================
// 7. WEB SERVER HTTP ROUTES
// ============================================================================
void handleRoot() {
  server.send_P(200, "text/html", INDEX_HTML);
}

void handleApiData() {
  uint32_t sinceSeq = 0;
  if (server.hasArg("since")) {
    sinceSeq = (uint32_t)server.arg("since").toInt();
  }

  uint32_t currentSeq = sampleSeq;
  uint32_t startSeq = (currentSeq > 20) ? (currentSeq - 20) : 1;
  if (sinceSeq >= startSeq && sinceSeq < currentSeq) {
    startSeq = sinceSeq + 1;
  } else if (sinceSeq >= currentSeq) {
    startSeq = currentSeq + 1;
  }

  String json = "{\"mode\":\"";
  if (mpu1Connected && mpu2Connected && !forceSimMode) {
    json += "HW: Dual MPU-6050 (0x68 & 0x69)";
  } else if (mpu1Connected && !forceSimMode) {
    json += "HW: Single MPU-6050 (0x68)";
  } else {
    json += "SIMULINK TEST WAVE (Slide 5 Model)";
  }
  json += "\",\"samples\":[";

  bool first = true;
  for (uint32_t s = startSeq; s <= currentSeq; s++) {
    uint8_t idx = s % RING_BUFFER_SIZE;
    if (sampleBuffer[idx].seq != s) continue;

    if (!first) json += ",";
    first = false;

    json += "{\"seq\":";
    json += String(sampleBuffer[idx].seq);
    json += ",\"t\":";
    json += String(sampleBuffer[idx].time_ms);
    json += ",\"m1\":";
    json += String(sampleBuffer[idx].mpu1_raw, 4);
    json += ",\"m2\":";
    json += String(sampleBuffer[idx].mpu2_raw, 4);
    json += ",\"fu\":";
    json += String(sampleBuffer[idx].fused_val, 4);
    json += ",\"fi\":";
    json += String(sampleBuffer[idx].filtered_val, 4);
    json += "}";
  }
  json += "]}";

  server.send(200, "application/json", json);
}

void handleToggleSim() {
  forceSimMode = !forceSimMode;
  server.send(200, "application/json", "{\"ok\":true}");
}

// ============================================================================
// 8. ARDUINO SETUP & MAIN LOOP
// ============================================================================
void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println();
  Serial.println(F("============================================================"));
  Serial.println(F("  DUAL MPU-6050 SEISMOGRAPH DSP NODE — STARTING UP"));
  Serial.println(F("============================================================"));

  // Initialize ESP32 I2C bus at 400 kHz Fast Mode for 100 Hz dual-sensor readout
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(400000);

  // 1. Initialize MPU-6050 #1 (Surface Sensor at 0x68, AD0 = LOW)
  if (mpuSurface.begin(MPU1_I2C_ADDR, &Wire)) {
    mpu1Connected = true;
    mpuSurface.setAccelerometerRange(MPU6050_RANGE_2_G);
    mpuSurface.setGyroRange(MPU6050_RANGE_250_DEG);
    mpuSurface.setFilterBandwidth(MPU6050_BAND_21_HZ);
    Serial.println(F("[I2C] MPU-6050 #1 (Surface  0x68) : DETECTED"));
  } else {
    Serial.println(F("[I2C] MPU-6050 #1 (Surface  0x68) : NOT FOUND"));
  }

  // 2. Initialize MPU-6050 #2 (Borehole Sensor at 0x69, AD0 = HIGH)
  if (mpuBorehole.begin(MPU2_I2C_ADDR, &Wire)) {
    mpu2Connected = true;
    mpuBorehole.setAccelerometerRange(MPU6050_RANGE_2_G);
    mpuBorehole.setGyroRange(MPU6050_RANGE_250_DEG);
    mpuBorehole.setFilterBandwidth(MPU6050_BAND_21_HZ);
    Serial.println(F("[I2C] MPU-6050 #2 (Borehole 0x69) : DETECTED"));
  } else {
    Serial.println(F("[I2C] MPU-6050 #2 (Borehole 0x69) : NOT FOUND"));
  }

  // 3. Connect to Wi-Fi (or start fallback Access Point if router not found)
  Serial.print(F("[WiFi] Connecting to SSID: "));
  Serial.println(WIFI_SSID);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  uint32_t wifiStart = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - wifiStart) < 12000) {
    delay(300);
    Serial.print('.');
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    activeIP    = WiFi.localIP().toString();
    wifiModeStr = "Station (Connected to Router/Hotspot)";
  } else {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_FALLBACK_SSID, AP_FALLBACK_PASS);
    activeIP    = WiFi.softAPIP().toString();
    wifiModeStr = "Fallback AP (SSID: ESP32-Seismograph | Pass: seismic123)";
  }

  // Print Webpage IP Address to Serial Monitor immediately
  printSerialStatusBanner();

  // 4. Start Embedded Highcharts Web Server
  server.on("/", HTTP_GET, handleRoot);
  server.on("/api/data", HTTP_GET, handleApiData);
  server.on("/api/toggle_sim", HTTP_POST, handleToggleSim);
  server.begin();

  lastSampleMicros   = micros();
  lastSerialIpMillis = millis();
}

void loop() {
  // Handle incoming HTTP requests for Highcharts dashboard
  server.handleClient();

  // Check if user sent 'c' or 'C' in Serial Monitor to toggle 100 Hz CSV stream
  while (Serial.available() > 0) {
    char ch = (char)Serial.read();
    if (ch == 'c' || ch == 'C') {
      serialCsvMode = !serialCsvMode;
      if (!serialCsvMode) {
        Serial.println(F("\n[Serial] Switched to IP & Status Banner Mode (every 3s)."));
        printSerialStatusBanner();
      } else {
        Serial.println(F("\n[Serial] Switched to 100 Hz Simulink CSV Mode (mpu1,mpu2,fused,filtered). Send 'c' to stop."));
      }
    }
  }

  // Execute deterministic 100 Hz sampling & DSP filtering (every 10,000 us)
  uint32_t nowMicros = micros();
  if ((uint32_t)(nowMicros - lastSampleMicros) >= SAMPLE_INTERVAL_US) {
    lastSampleMicros += SAMPLE_INTERVAL_US;
    sampleSeismograph();
  }

  // Re-print the Webpage IP Address every 3 seconds in Serial Monitor (when not in CSV stream mode)
  uint32_t nowMillis = millis();
  if (!serialCsvMode && (uint32_t)(nowMillis - lastSerialIpMillis) >= SERIAL_IP_PRINT_MS) {
    lastSerialIpMillis = nowMillis;
    printSerialStatusBanner();
  }
}
