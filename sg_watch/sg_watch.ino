#include <Wire.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <WiFiUdp.h>
#include <Preferences.h>
#include <WebServer.h>
#include <strings.h>
#include <time.h>
#include <sys/time.h>
#include "MAX30105.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <string.h>
#include <stdint.h>
#include <limits.h>
#include <math.h>

// Set to 0 to turn off the Serial debug line once everything works.
#define DEBUG_SERIAL 1

// ---------- WiFi / time settings (EDIT THESE) ----------
const char *WIFI_SSID     = "wifi_name";
const char *WIFI_IDENTITY = "wifi_identity";   // the "identity" you log in with
const char *WIFI_PASS     = "wifi_password";
const char *TZ_INFO   = "IST-5:30";          // India Standard Time
const char *NTP_1     = "in.pool.ntp.org";
const char *NTP_2     = "samay1.nic.in";
const unsigned long WIFI_TIMEOUT_MS = 20000;   // campus (PEAP) logins can be slow

// Optional fallback: your phone's hotspot (2.4 GHz, normal WPA2 password).
// Leave the name empty ("") to skip it.
const char *HOTSPOT_SSID = "";
const char *HOTSPOT_PASS = "";

// The watch's own WiFi network for the live web page (join it with your
// phone, then open http://192.168.4.1).
// Change it: the page shows your heart rate to anyone who can join.
#define AP_SSID_STR "SGCo-Watch"
const char *AP_SSID = AP_SSID_STR;
const char *AP_PASS = "sg123456";

// ---------- I2C pins (ESP32 defaults) ----------
#define I2C_SDA 21
#define I2C_SCL 22

// ---------- OLED ----------
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C

// Last two arguments: I2C clock during / after a transfer. The default
// "after" value is 100 kHz, which would also slow down the sensor reads.
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET,
                         400000UL, 400000UL);
MAX30105 sensor;

int freeRam() { return (int)ESP.getFreeHeap(); }

// ---------- Sensor sampling ----------
// 400 samples/s averaged by 8 -> 50 samples/s.
// (Do not raise this much: the MAX30105 library only buffers 4 samples
// between sensor.check() calls and the OLED redraw blocks for ~25 ms.)
#define SAMPLE_AVG 8
const float NOMINAL_FS = 400.0f / SAMPLE_AVG;          // 50 Hz
const float NOMINAL_PERIOD_MS = 1000.0f / NOMINAL_FS;  // 20 ms

// Sample clock: time of the newest sample in ms. The period is corrected
// against millis() (the ESP32 crystal) because the sensor's internal
// oscillator is only accurate to a couple of percent.
double tMs = 0;
float periodMs = NOMINAL_PERIOD_MS;
unsigned long calStartMs = 0;
unsigned long calSamples = 0;
const unsigned long CAL_FIRST_MS = 4000;   // first window: learn the real sample rate
const unsigned long CAL_WINDOW_MS = 8000;  // later windows: fine-tune it
bool rateKnown = false;
byte calBadWindows = 0;
void applyRate();   // defined below: updates everything that depends on the rate

void calibrateClock() {
  if (calStartMs == 0) {
    calStartMs = millis();
    calSamples = 0;
    return;
  }
  calSamples++;
  unsigned long el = millis() - calStartMs;
  unsigned long win = rateKnown ? CAL_WINDOW_MS : CAL_FIRST_MS;
  if (el >= win) {
    float meas = (float)el / (float)calSamples;
    if (!rateKnown) {
      // Learn the real period (it can differ a lot from the nominal 20 ms).
      if (meas > 4.0f && meas < 200.0f) {
        periodMs = meas;
        rateKnown = true;
        applyRate();
      }
    } else if (meas > periodMs * 0.92f && meas < periodMs * 1.08f) {
      periodMs += 0.5f * (meas - periodMs);   // fine-tune against the crystal
      calBadWindows = 0;
    } else if (++calBadWindows >= 2) {
      // Two windows in a row disagree: the real rate changed, learn it again.
      periodMs = meas;
      calBadWindows = 0;
      applyRate();
    }
    calStartMs = millis();
    calSamples = 0;
  }
}

// ---------- LED brightness ----------
const byte IDLE_POWER = 40;   // LED level while waiting for a finger
byte irPower = IDLE_POWER;
byte redPower = IDLE_POWER;
unsigned long lastAdjustTime = 0;
const unsigned long ADJUST_FAST_MS = 50;   // while the signal is still too weak
const unsigned long ADJUST_SLOW_MS = 250;  // when only checking for saturation
unsigned long lastPowerChangeMs = 0;
bool dcReseed = false;                     // re-seed BPM high-pass after a LED change
const unsigned long SETTLE_MS = 400;       // no beats accepted this soon after

// Dead-band: LED power is only touched when the raw level leaves this range.
const int32_t LED_RAW_LOW  = 60000;        // below: ramp LED up
const int32_t LED_RAW_HIGH = 210000;       // above: ramp LED down gently
const int32_t LED_RAW_SAT  = 235000;       // above: ramp LED down faster

// ---------- Finger detection ----------
// Raise FINGER_IR_MIN if the watch face flips to the vitals screen by itself
// (e.g. under bright light); lower it if your finger is not detected.
const int32_t FINGER_IR_MIN = 20000;
const unsigned long FINGER_ON_MS = 300;    // must stay high this long to start
const unsigned long FINGER_LOST_MS = 1000; // must stay low this long to stop
bool fingerPresent = false;
unsigned long highSince = 0;
unsigned long lowSince = 0;

// ---------- Display refresh ----------
unsigned long lastDrawMs = 0;
const unsigned long DRAW_INTERVAL_VITALS_MS = 50;  // ~20 fps for the waveform
const unsigned long DRAW_INTERVAL_WATCH_MS = 250;  // watch face needs little

// ---------- Live PPG waveform ----------
// Bottom part of the screen: y = WAVE_TOP .. WAVE_TOP + WAVE_H - 1
#define WAVE_W 64           // stored points
#define WAVE_PX 2           // each point is drawn 2 pixels wide (64 * 2 = 128)
#define WAVE_TOP 38
#define WAVE_H 26
byte waveDecim = 3;  // set by applyRate() from the real sample rate. Old note:        // keep every 3rd sample (50 Hz -> ~17 Hz, ~3.8 s window)
uint8_t waveBuf[WAVE_W];    // already-scaled y positions (0..WAVE_H-1)
byte waveIdx = 0;           // next slot to write = oldest point
byte waveDecimCnt = 0;
float waveLP = 0;           // lightly smoothed AC signal
float wavePeak = 20;        // slow peak tracker for auto-scaling

void initWave() {
  for (byte i = 0; i < WAVE_W; i++) waveBuf[i] = WAVE_H / 2;
  waveIdx = 0;
  waveDecimCnt = 0;
  waveLP = 0;
  wavePeak = 20;
}

// Called for every sensor sample with the AC value (beats point up).
void pushWave(float v) {
  // Light smoothing to remove sample-to-sample noise.
  waveLP += 0.5f * (v - waveLP);

  // Auto-scale: fast attack, slow decay (~ a few seconds).
  float a = (waveLP < 0) ? -waveLP : waveLP;
  if (a > wavePeak) wavePeak = a;
  else wavePeak *= 0.997f;
  if (wavePeak < 10.0f) wavePeak = 10.0f;   // don't blow up pure noise

  if (++waveDecimCnt >= waveDecim) {
    waveDecimCnt = 0;
    float half = (WAVE_H - 1) / 2.0f;
    float y = half - (waveLP / wavePeak) * half;
    if (y < 0) y = 0;
    if (y > WAVE_H - 1) y = WAVE_H - 1;
    waveBuf[waveIdx] = (uint8_t)(y + 0.5f);
    waveIdx = (waveIdx + 1) % WAVE_W;
  }
}

void drawWave() {
  byte idx = waveIdx;                       // oldest point
  int16_t py = WAVE_TOP + waveBuf[idx];
  for (byte i = 1; i < WAVE_W; i++) {
    idx = (idx + 1) % WAVE_W;
    int16_t y = WAVE_TOP + waveBuf[idx];
    display.drawLine((i - 1) * WAVE_PX, py, i * WAVE_PX, y, SSD1306_WHITE);
    py = y;
  }
}

// ---------- SpO2 signal processing (unchanged) ----------
int32_t dcIR = 0, dcRED = 0;
const int SHIFT_DC = 4;

int32_t irMax = INT32_MIN, irMin = INT32_MAX;
int32_t redMax = INT32_MIN, redMin = INT32_MAX;

float spo2 = 0;
float Rratio = 0;
unsigned long lastSpo2DetectedMs = 0;
const unsigned long SPO2_TIMEOUT_MS = 10000;

// ---------- BPM signal processing ----------
// Band-pass 0.4 - 5 Hz: one-pole high-pass (DC removal) followed by two
// cascaded one-pole low-passes.
const float HP_FC = 0.4f;
const float LP_FC = 5.0f;
float hpAlpha = 0, lpAlpha = 0;   // set by applyRate() from the real sample rate
float bpDC = 0, lp1 = 0, lp2 = 0;

float env = 0;
const float envAlpha = 0.95f;
const float MIN_SIGNAL_QUALITY = 8.0f;

// Peak window: a sample is a beat peak if it is the maximum of
// +-PK_N samples around it (+-60 ms at 50 Hz).
#define PK_W_MAX 7
byte pkN = 3;                 // half window in samples, set by applyRate()
byte pkW = 7;
float pkBuf[PK_W_MAX];
byte pkFill = 0;

// Second opinion: BPM from the autocorrelation of the last ~8 s of signal.
#define AC_N 400
float acBuf[AC_N];
int acIdx = 0, acFill = 0;

// ---------- Respiratory rate ----------
// Breathing slowly raises and lowers the finger signal (about 0.1 - 0.5 Hz).
// That band is filtered out of the raw IR signal, reduced to ~4 samples/s,
// and the breathing rate is read from a spectrum peak over ~30 s.
#define RESP_N 120
float respBuf[RESP_N];
int respIdx = 0, respFill = 0;
float respHP = 0, respHP2 = 0, respL1 = 0, respL2 = 0, respL3 = 0;
float respHpA = 0.01f, respLpA = 0.05f;
byte respDecim = 5, respCnt = 0, respSince = 0;
int respSkip = 0;                  // decimated samples still to ignore (filter settling)
float respFs = 4.0f;               // decimated sample rate (Hz)
volatile float respRate = 0;       // breaths per minute, 0 = none yet
float respQ = 0;                   // autocorrelation peak / signal power (quality)
unsigned long respValidMs = 0;
bool respReseed = true;
float respHistEst[16];             // candidate rate of each of the last 16 updates (0 = none)
byte respHistIdx = 0;
int respUpd = 0;                   // updates made since the window first filled

void respRestart() {
  for (byte i = 0; i < 16; i++) respHistEst[i] = 0;
  respHistIdx = 0;
  respUpd = 0;
  respFill = respIdx = 0;
  respCnt = respSince = 0;
  respSkip = (int)(8.0f * respFs);
  respReseed = true;
}

double lastPeakT = 0;           // time of the last accepted beat (ms, sample clock)
bool lastPeakValid = false;
float beatAmp = 0;              // running average of beat peak height
const double MIN_IBI_MS = 300.0;     // 200 BPM
const double MAX_IBI_MS = 2000.0;    // 30 BPM (used only while acquiring)

unsigned long lastBeatDetectedMs = 0;   // last time a beat interval was accepted
unsigned long heartBeatTimer = 0;
const unsigned long BPM_HOLD_MS = 8000;          // keep showing last BPM this long
const unsigned long BEAT_AMP_DECAY_AFTER_MS = 2000;

// Interval validation (relative to the median of recent intervals).
const float GATE_LOW  = 0.70f;      // shorter than this = spurious early peak
const float GATE_HIGH = 1.40f;      // longer than this = missed beat / gap
const byte  RELOCK_AFTER = 3;       // this many bad intervals in a row = start over

// BPM smoothing.
const float BPM_ALPHA = 0.30f;      // weight of each new estimate
const float BPM_JUMP  = 15.0f;      // bigger change must persist to be believed
const float BPM_DISPLAY_HYST = 0.8f;

float bpm = 0;                  // smoothed BPM, 0 = none
int bpmInt = 0;                 // value shown on screen (with hysteresis)
byte bigDiffCnt = 0;
byte rejectCnt = 0;
unsigned long dbgPeaks = 0;     // beats detected since the finger was placed (debug)

// Debug counters.
unsigned long sampleCount = 0;
int32_t dbgIR = 0;
bool timeSynced = false;

// ---------- Rolling average of the last 10 SpO2 readings ----------
#define AVG_N 10
uint16_t spo2Buf[AVG_N];
byte spo2Idx = 0, spo2Cnt = 0;

// Adds a value to a ring buffer and returns the average of what it holds.
float pushAvg(uint16_t *buf, byte &idx, byte &cnt, float v) {
  buf[idx] = (uint16_t)(v * 10.0f + 0.5f);
  idx = (idx + 1) % AVG_N;
  if (cnt < AVG_N) cnt++;

  unsigned long sum = 0;
  for (byte i = 0; i < cnt; i++) sum += buf[i];
  return (float)sum / (cnt * 10.0f);
}

// ---------- Beat intervals, BPM and rhythm ----------
#define IBI_N 10
uint16_t ibiBuf[IBI_N];               // beat intervals in ms
byte ibiIdx = 0, ibiCnt = 0;
const float IRREG_DEV = 0.20f;        // interval is "off" if >20% from the mean
const byte IRREG_MAX_OUTLIERS = 3;    // this many "off" intervals = irregular
byte rhythm = 0;                      // 0 = unknown, 1 = regular, 2 = irregular

// ---------- HRV (heart-rate variability) ----------
// RMSSD = root mean square of the differences between successive beat
// intervals; SDNN = standard deviation of the intervals. Only intervals
// that follow each other directly are compared, and a jump of more than 20%
// between neighbours is treated as an artifact and skipped.
#define HRV_N 40
float hrvIbi[HRV_N];
bool hrvLinked[HRV_N];            // follows the previously stored interval directly
byte hrvIdx = 0, hrvCnt = 0;
bool hrvBreak = true;             // next interval does not follow the last one
const byte HRV_MIN_PAIRS = 20;
volatile float hrvRmssd = 0, hrvSdnn = 0;
volatile byte hrvPairs = 0;

void hrvUpdate() {
  float mean = 0;
  for (byte i = 0; i < hrvCnt; i++) mean += hrvIbi[i];
  mean /= hrvCnt;

  float var = 0;
  for (byte i = 0; i < hrvCnt; i++) { float d = hrvIbi[i] - mean; var += d * d; }

  float sumsq = 0;
  byte pairs = 0;
  for (byte k = 1; k < hrvCnt; k++) {
    byte pos = (hrvIdx + HRV_N - hrvCnt + k) % HRV_N;
    byte prev = (pos + HRV_N - 1) % HRV_N;
    if (!hrvLinked[pos]) continue;
    float d = hrvIbi[pos] - hrvIbi[prev];
    float avg = 0.5f * (hrvIbi[pos] + hrvIbi[prev]);
    if (fabsf(d) > 0.20f * avg) continue;        // artifact
    sumsq += d * d;
    pairs++;
  }

  hrvPairs = pairs;
  if (pairs >= HRV_MIN_PAIRS && hrvCnt > 2) {
    hrvRmssd = sqrtf(sumsq / pairs);
    hrvSdnn = sqrtf(var / (hrvCnt - 1));
  } else {
    hrvRmssd = 0;
    hrvSdnn = 0;
  }
}

void hrvPush(float ibi) {
  hrvIbi[hrvIdx] = ibi;
  hrvLinked[hrvIdx] = (!hrvBreak && hrvCnt > 0);
  hrvBreak = false;
  hrvIdx = (hrvIdx + 1) % HRV_N;
  if (hrvCnt < HRV_N) hrvCnt++;
  hrvUpdate();
}

// Median of the stored intervals (ms), 0 if none.
float ibiMedian() {
  if (ibiCnt == 0) return 0;

  uint16_t tmp[IBI_N];
  memcpy(tmp, ibiBuf, ibiCnt * sizeof(uint16_t));
  for (byte i = 1; i < ibiCnt; i++) {            // insertion sort
    uint16_t v = tmp[i];
    int j = i - 1;
    while (j >= 0 && tmp[j] > v) { tmp[j + 1] = tmp[j]; j--; }
    tmp[j + 1] = v;
  }

  if (ibiCnt & 1) return (float)tmp[ibiCnt / 2];
  return 0.5f * ((float)tmp[ibiCnt / 2 - 1] + (float)tmp[ibiCnt / 2]);
}

// BPM = 60000 / mean of the intervals that agree with the median (within
// 25%), so one missed or extra beat does not move the reading. The result
// is then low-pass filtered and the on-screen integer has hysteresis.
void updateBpm() {
  if (ibiCnt < 3) return;

  float med = ibiMedian();
  float sum = 0;
  byte n = 0;
  for (byte i = 0; i < ibiCnt; i++) {
    float d = (float)ibiBuf[i] - med;
    if (d < 0) d = -d;
    if (d <= 0.25f * med) { sum += ibiBuf[i]; n++; }
  }
  if (n < 3 || n * 2 < ibiCnt) return;    // intervals do not agree: keep old BPM

  float est = 60000.0f / (sum / n);
  if (est < 30.0f || est > 200.0f) return;

  if (bpm <= 0) {
    bpm = est;
    bigDiffCnt = 0;
  } else {
    float d = est - bpm;
    if (d > BPM_JUMP || d < -BPM_JUMP) {
      // Big change: only believe it if it persists for 3 beats in a row.
      if (++bigDiffCnt >= 3) { bpm = est; bigDiffCnt = 0; }
    } else {
      bigDiffCnt = 0;
      bpm += BPM_ALPHA * d;
    }
  }

  // Integer shown on screen only moves when the value is clearly past it.
  float dd = bpm - (float)bpmInt;
  if (dd < 0) dd = -dd;
  if (bpmInt == 0 || dd >= BPM_DISPLAY_HYST) bpmInt = (int)(bpm + 0.5f);
}

void updateRhythm() {
  if (ibiCnt < IBI_N) {       // wait until we have 10 intervals
    rhythm = 0;
    return;
  }

  float mean = 0;
  for (byte i = 0; i < IBI_N; i++) mean += ibiBuf[i];
  mean /= IBI_N;

  byte outliers = 0;
  for (byte i = 0; i < IBI_N; i++) {
    float d = (float)ibiBuf[i] - mean;
    if (d < 0) d = -d;
    if (d > IRREG_DEV * mean) outliers++;
  }

  rhythm = (outliers >= IRREG_MAX_OUTLIERS) ? 2 : 1;
}

// forHrv = false for an interval that was only reconstructed (a missed beat
// was split): it still feeds the BPM but is not used for HRV.
void addInterval(double dtMs, bool forHrv = true) {
  ibiBuf[ibiIdx] = (uint16_t)(dtMs + 0.5);
  ibiIdx = (ibiIdx + 1) % IBI_N;
  if (ibiCnt < IBI_N) ibiCnt++;
  if (forHrv) hrvPush((float)dtMs);
  else hrvBreak = true;
  updateBpm();
  updateRhythm();
}

// Soft reset: forget the stored intervals but keep showing the last BPM
// (it is replaced as soon as new valid intervals arrive).
void clearIntervalHistory() {
  ibiIdx = ibiCnt = 0;
  rhythm = 0;
  hrvBreak = true;                 // HRV: next interval is not adjacent to the last
}

// Full reset: no BPM at all (screen shows "--" / LOW SIGNAL).
void clearIntervals() {
  clearIntervalHistory();
  hrvCnt = hrvIdx = 0;
  hrvBreak = true;
  hrvRmssd = hrvSdnn = 0;
  hrvPairs = 0;
  bpm = 0;
  bpmInt = 0;
  bigDiffCnt = 0;
  rejectCnt = 0;
}

// Recomputes everything that depends on the REAL sample rate: filter
// coefficients, peak window width and waveform decimation. The old interval
// history used a different time base, so it is dropped.
void applyRate() {
  float dt = periodMs / 1000.0f;
  hpAlpha = dt / (1.0f / (6.2831853f * HP_FC) + dt);
  lpAlpha = dt / (1.0f / (6.2831853f * LP_FC) + dt);

  int n = (int)(80.0f / periodMs + 0.5f);      // peak window about +-80 ms
  if (n < 1) n = 1;
  if (n > 3) n = 3;
  pkN = (byte)n;
  pkW = (byte)(2 * n + 1);

  int d = (int)((1000.0f / periodMs) / 17.0f + 0.5f);   // ~17 waveform points/s
  if (d < 1) d = 1;
  if (d > 6) d = 6;
  waveDecim = (byte)d;

  // Respiratory band filters (2 x 0.08 Hz high-pass, 3 x 0.5 Hz low-pass) and
  // decimation to about 4 samples per second.
  respHpA = dt / (1.0f / (6.2831853f * 0.08f) + dt);
  respLpA = dt / (1.0f / (6.2831853f * 0.5f) + dt);
  int rd = (int)(250.0f / periodMs + 0.5f);
  if (rd < 1) rd = 1;
  if (rd > 40) rd = 40;
  respDecim = (byte)rd;
  respFs = 1000.0f / ((float)respDecim * periodMs);
  respRate = 0;
  respRestart();
  acFill = 0;
  acIdx = 0;

  pkFill = 0;
  lastPeakValid = false;
  beatAmp = 0;
  clearIntervals();
}

// ---------- Helpers ----------
float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

// Finds the breathing rate from the last ~30 s of the respiratory-band
// signal. A spectrum is computed at 0.01 Hz steps over 0.10 - 0.50 Hz
// (6 - 30 breaths/min) and a rate is only accepted when ONE sharp peak
// stands well above the background on both sides: random noise and slow
// drift have no such peak.
// Called every ~2 s once the window is full.
// Simulated test (noise only, 30 runs x 5 min): a ratio of 12 showed a false
// breathing rate in about 1 run in 6; 50 never did, and still found real
// breathing down to a very weak signal.
const float RESP_MIN_RATIO = 50.0f;     // peak power / background power next to it

void updateResp() {
  const int n = RESP_N;
  static float w[RESP_N];
  static float pw[48];

  float mean = 0;
  for (int i = 0; i < n; i++) mean += respBuf[(respIdx + i) % RESP_N];   // respIdx = oldest
  mean /= n;
  for (int i = 0; i < n; i++) {                                          // Hann window
    float x = respBuf[(respIdx + i) % RESP_N] - mean;
    w[i] = x * (0.5f - 0.5f * cosf(6.2831853f * (float)i / (float)(n - 1)));
  }

  const float fLo = 0.10f, step = 0.01f;
  const int nb = 41;                                                     // up to 0.50 Hz
  int pk = 0;
  for (int b = 0; b < nb; b++) {
    float ang = 6.2831853f * (fLo + b * step) / respFs;
    float c = cosf(ang), s = sinf(ang);
    float re = 0, im = 0, cr = 1, ci = 0;
    for (int i = 0; i < n; i++) {
      re += w[i] * cr;
      im += w[i] * ci;
      float ncr = cr * c - ci * s;
      ci = cr * s + ci * c;
      cr = ncr;
    }
    pw[b] = re * re + im * im;
    if (pw[b] > pw[pk]) pk = b;
  }

  // Background = mean power 8..16 steps (0.08 - 0.16 Hz) away on each side of
  // the peak; the higher side counts, so slow drift (which is strongest at the
  // low end) cannot pass as a peak.
  float sideL = 0, sideR = 0;
  int nL = 0, nR = 0;
  for (int d = 8; d <= 16; d++) {
    if (pk - d >= 0)  { sideL += pw[pk - d]; nL++; }
    if (pk + d < nb)  { sideR += pw[pk + d]; nR++; }
  }
  float bg = 0;
  if (nL >= 3) bg = sideL / nL;
  if (nR >= 3 && sideR / nR > bg) bg = sideR / nR;
  float ratio = (bg > 0) ? pw[pk] / bg : 0;
  respQ = ratio;

  // Candidate rate from this window (0 = no clear peak; a peak at the band
  // edge is drift, not breathing).
  float cand = 0;
  if (ratio >= RESP_MIN_RATIO && pk > 0 && pk < nb - 1) {
    float y0 = pw[pk - 1], y1 = pw[pk], y2 = pw[pk + 1];
    float den = y0 - 2.0f * y1 + y2;
    float frac = 0;
    if (den < -1e-9f) frac = clampf(0.5f * (y0 - y2) / den, -0.5f, 0.5f);
    cand = 60.0f * (fLo + ((float)pk + frac) * step);
  }

  // Accept only if the previous window (16 updates = 32 s ago, so it shares
  // no data with this one) found nearly the same rate. Real breathing
  // repeats; random drift and noise almost never do.
  float old = respHistEst[respHistIdx];
  respHistEst[respHistIdx] = cand;
  respHistIdx = (respHistIdx + 1) % 16;
  if (respUpd < 16) respUpd++;

  bool found = false;
  if (cand > 0 && old > 0 && fabsf(cand - old) <= 2.0f) {
    respRate = (respRate > 0) ? respRate + 0.35f * (cand - respRate) : cand;
    respValidMs = millis();
    found = true;
  }
  if (!found && respRate > 0 && millis() - respValidMs > 30000UL) respRate = 0;
}

// BPM from the autocorrelation of the last ~8 s of the filtered signal.
// Independent of peak picking, so it is a good cross-check. Returns 0 when
// there is not enough signal yet or the signal is not periodic.
float acBpm() {
  if (!rateKnown) return 0;
  int n = (int)(8000.0f / periodMs);
  if (n > AC_N) n = AC_N;
  if (acFill < n) return 0;

  int lagMin = (int)(300.0f / periodMs);    // 200 BPM
  if (lagMin < 2) lagMin = 2;
  int lagMax = (int)(1500.0f / periodMs);   // 40 BPM
  if (lagMax + 1 >= n / 2) return 0;

  static float w[AC_N];
  static float r[AC_N];

  int start = (acIdx + AC_N - n) % AC_N;
  float mean = 0;
  for (int i = 0; i < n; i++) mean += acBuf[(start + i) % AC_N];
  mean /= n;
  for (int i = 0; i < n; i++) w[i] = acBuf[(start + i) % AC_N] - mean;

  float r0 = 0;
  for (int i = 0; i < n; i++) r0 += w[i] * w[i];
  r0 /= n;
  if (r0 < 1e-3f) return 0;

  for (int lag = lagMin - 1; lag <= lagMax + 1; lag++) {
    float s = 0;
    for (int i = 0; i + lag < n; i++) s += w[i] * w[i + lag];
    r[lag] = s / (float)(n - lag);
  }

  float rmax = 0;
  for (int lag = lagMin; lag <= lagMax; lag++) if (r[lag] > rmax) rmax = r[lag];
  if (rmax < 0.2f * r0) return 0;           // not periodic enough

  // First local maximum that is nearly as strong as the best one = the
  // fundamental period (not a multiple of it).
  for (int lag = lagMin; lag <= lagMax; lag++) {
    if (r[lag] >= 0.8f * rmax && r[lag] >= r[lag - 1] && r[lag] > r[lag + 1]) {
      float y0 = r[lag - 1], y1 = r[lag], y2 = r[lag + 1];
      float den = y0 - 2.0f * y1 + y2;
      float frac = 0;
      if (den < -1e-9f) frac = clampf(0.5f * (y0 - y2) / den, -0.5f, 0.5f);
      return 60000.0f / (((float)lag + frac) * periodMs);
    }
  }
  return 0;
}

void drawHeart(int16_t x, int16_t y, uint16_t color) {
  display.fillCircle(x + 3, y + 3, 3, color);
  display.fillCircle(x + 9, y + 3, 3, color);
  display.fillTriangle(x, y + 4, x + 12, y + 4,
                       x + 6, y + 10, color);
}

// Full-screen message, one or two lines (pass nullptr for no second line).
// Size-2 text fits only 10 characters per line, so long messages are split.
void showMessage(const __FlashStringHelper *line1,
                 const __FlashStringHelper *line2) {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextWrap(false);
  display.setTextSize(2);
  display.setCursor(0, line2 ? 12 : 24);
  display.print(line1);
  if (line2) {
    display.setCursor(0, 34);
    display.print(line2);
  }
  display.display();
}

// Start-up / syncing screen: the "SG&Co" logo with a small status line.
// Size-3 text is 18 px per character, so "SG&Co" is 90 px wide.
void showLogo(const __FlashStringHelper *status) {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextWrap(false);

  display.drawRoundRect(2, 2, SCREEN_WIDTH - 4, SCREEN_HEIGHT - 4, 8, SSD1306_WHITE);

  display.setTextSize(3);
  display.setCursor((SCREEN_WIDTH - 5 * 18) / 2, 15);
  display.print(F("SG&Co"));

  display.setTextSize(1);
  int len = strlen_P((PGM_P)status);
  display.setCursor((SCREEN_WIDTH - len * 6) / 2, 46);
  display.print(status);

  display.display();
}

// Clears all measurement state and puts the LEDs back to the idle level.
void resetState() {
  spo2 = 0;
  env = 0;
  clearIntervals();
  dcIR = dcRED = 0;

  spo2Idx = spo2Cnt = 0;

  irMax = INT32_MIN;
  irMin = INT32_MAX;
  redMax = INT32_MIN;
  redMin = INT32_MAX;

  bpDC = lp1 = lp2 = 0;
  for (byte i = 0; i < PK_W_MAX; i++) pkBuf[i] = 0;
  pkFill = 0;
  lastPeakValid = false;
  beatAmp = 0;

  lastBeatDetectedMs = millis();
  lastSpo2DetectedMs = millis();
  heartBeatTimer = 0;
  dbgPeaks = 0;
  acFill = 0;
  acIdx = 0;
  respRate = 0;
  respRestart();

  highSince = 0;
  lowSince = 0;

  initWave();

  irPower = IDLE_POWER;
  redPower = IDLE_POWER;
  sensor.setPulseAmplitudeIR(irPower);
  sensor.setPulseAmplitudeRed(redPower);
  lastPowerChangeMs = millis();
}

void startMeasuring(int32_t irRaw, int32_t redRaw) {
  resetState();
  // Start the DC filters at the current level so there is no big
  // start-up transient on the waveform.
  dcIR = irRaw;
  dcRED = redRaw;
  bpDC = (float)irRaw;
  fingerPresent = true;
  Serial.println(F("Finger detected."));
}

void stopMeasuring() {
  resetState();
  fingerPresent = false;
  Serial.println(F("Finger removed."));
}

// ---------- Beat detection ----------
// Looks at the middle of the PK_W-sample window. Returns true and the beat
// time (ms on the sample clock) and height when it is a valid peak.
bool findPeak(double &tPeak, float &amp) {
  if (!rateKnown) return false;          // time base not measured yet
  if (pkFill < pkW) return false;
  if (env < MIN_SIGNAL_QUALITY) return false;

  float c = pkBuf[pkN];
  for (byte i = 0; i < pkW; i++) {
    if (i == pkN) continue;
    if (i < pkN) { if (pkBuf[i] > c) return false; }
    else         { if (pkBuf[i] >= c) return false; }
  }

  // Threshold: fraction of the mean signal level, and of recent beat height.
  float thresh = 0.5f * env;
  if (beatAmp > 0 && 0.4f * beatAmp > thresh) thresh = 0.4f * beatAmp;
  if (c <= thresh) return false;

  // Parabolic interpolation around the sample maximum -> sub-sample timing.
  float y0 = pkBuf[pkN - 1], y2 = pkBuf[pkN + 1];
  float denom = y0 - 2.0f * c + y2;
  float frac = 0;
  if (denom < -1e-6f) frac = clampf(0.5f * (y0 - y2) / denom, -0.5f, 0.5f);

  // The newest sample is PK_N samples after the window centre.
  tPeak = tMs - (double)(pkN - frac) * periodMs;
  amp = c;
  return true;
}

void updateSpo2(unsigned long nowMs) {
  // Experimental ratio-of-ratios SpO2 estimate.
  float acIRAmp = (float)(irMax - irMin);
  float acREDAmp = (float)(redMax - redMin);

  if (acIRAmp > 1 && acREDAmp > 1 && dcIR > 1 && dcRED > 1) {
    float nir = acIRAmp / (float)dcIR;
    float nred = acREDAmp / (float)dcRED;

    if (nir > 0.000001f) {
      Rratio = nred / nir;
      float spo2Inst = clampf(110.0f - 25.0f * Rratio, 70.0f, 100.0f);

      // Average of the last 10 SpO2 readings.
      spo2 = pushAvg(spo2Buf, spo2Idx, spo2Cnt, spo2Inst);
      lastSpo2DetectedMs = nowMs;
    }
  }
}

void handleBeat(double tPeak, float amp) {
  unsigned long nowMs = millis();
  if (nowMs - lastPowerChangeMs < SETTLE_MS) return;   // IR LED level just changed

  bool intervalOk = false;

  if (lastPeakValid) {
    double dt = tPeak - lastPeakT;

    // Too close to the previous beat (dicrotic notch / noise ripple):
    // ignore this peak completely and keep the old reference.
    if (dt < MIN_IBI_MS) return;

    if (ibiCnt < 3) {
      // ---- Acquiring: no trustworthy median yet ----
      if (dt > MAX_IBI_MS) {
        // Reference is stale: just restart from this peak.
        hrvBreak = true;
      } else {
        if (ibiCnt > 0) {
          // Each new interval must be close to the previous one,
          // otherwise start collecting again.
          float last = (float)ibiBuf[(ibiIdx + IBI_N - 1) % IBI_N];
          float diff = (float)dt - last;
          if (diff < 0) diff = -diff;
          if (diff > 0.3f * last) { ibiIdx = ibiCnt = 0; hrvBreak = true; }
        }
        addInterval(dt);
        intervalOk = true;
      }
    } else {
      // ---- Tracking: validate against the median ----
      float med = ibiMedian();
      float ratio = (float)dt / med;
      int k = (int)(ratio + 0.5f);

      if (ratio >= GATE_LOW && ratio <= GATE_HIGH) {
        addInterval(dt);                       // normal beat
        intervalOk = true;
      } else if (k >= 2 && k <= 3 &&
                 fabsf(ratio / (float)k - 1.0f) < 0.15f) {
        addInterval(dt / (double)k, false);           // 1-2 beats were missed
        intervalOk = true;
      } else {
        // Unusable interval.
        rejectCnt++;
        if (rejectCnt >= RELOCK_AFTER) {
          clearIntervalHistory();              // re-learn from here (BPM is held)
          rejectCnt = 0;
        } else if (ratio < GATE_LOW) {
          return;                              // spurious early peak: ignore it
        }
        // Otherwise (long odd gap): continue counting from this peak.
        hrvBreak = true;
      }
    }
  }

  if (intervalOk) {
    rejectCnt = 0;
    lastBeatDetectedMs = nowMs;
  }

  lastPeakT = tPeak;
  lastPeakValid = true;
  heartBeatTimer = nowMs + 150;   // heart icon flash
  dbgPeaks++;

  // Beat height average. A single huge spike (motion) is capped so it
  // cannot raise the detection threshold out of reach.
  if (beatAmp > 0) {
    float a = amp;
    if (a > 2.0f * beatAmp) a = 2.0f * beatAmp;
    beatAmp += 0.25f * (a - beatAmp);
  } else {
    beatAmp = amp;
  }

  if (intervalOk) updateSpo2(nowMs);

  irMax = INT32_MIN;
  irMin = INT32_MAX;
  redMax = INT32_MIN;
  redMin = INT32_MAX;
}

// ---------- Time ----------
// The ESP32 has no battery-backed clock, so the time is set at every boot.
// Several methods are tried in order until one works (see syncTime()):
//   1. NTP over UDP (by host name, then by raw IP address)
//   2. HTTP "Date" header from plain-HTTP sites (by name, then raw IP)
//   3. HTTPS "Date" header (the certificate is NOT checked: only the time
//      is read from the reply, nothing secret is sent)
//   4. worldtimeapi.org
//   5. the last time saved in flash (approximate), else the compile time
// You can always set the time by hand in the Serial Monitor (9600 baud,
// "Newline" line ending):
//   T 15:30                  sets the time (date unchanged)
//   T 2026-10-17 15:30:00    sets date and time (India time)
//   SYNC                     tries the network methods again

volatile bool webPaused = true;    // web server is paused while WiFi is switched around
void startAccessPoint();           // defined in the web section below

bool syncRequested = false;
unsigned long nextSyncMs = 0;       // next automatic retry (0 = none planned)
byte syncAttempts = 0;
unsigned long lastSaveMs = 0;
int wifiStatus = 0;
const char *timeSource = "none";

// Seconds since 1970 for a UTC date/time (no time-zone tricks needed).
time_t epochFromUtc(int y, int mo, int d, int h, int mi, int s) {
  y -= (mo <= 2);
  long era = (y >= 0 ? y : y - 399) / 400;
  unsigned yoe = (unsigned)(y - era * 400);
  unsigned doy = (153 * (mo + (mo > 2 ? -3 : 9)) + 2) / 5 + d - 1;
  unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  long days = era * 146097L + (long)doe - 719468L;
  return (time_t)(days * 86400L + h * 3600L + mi * 60L + s);
}

// The moment this sketch was compiled (needs TZ to be set first).
time_t compileEpoch() {
  static time_t cached = 0;
  if (cached == 0) {
    struct tm t;
    memset(&t, 0, sizeof(t));
    if (strptime(__DATE__ " " __TIME__, "%b %d %Y %H:%M:%S", &t)) {
      t.tm_isdst = -1;
      cached = mktime(&t);
    }
    if (cached <= 0) cached = 1700000000;
  }
  return cached;
}

// A real time reading can never be older than the moment this sketch was
// compiled (2 days of margin for time zones). Rejects garbage.
bool plausibleTime(time_t t) { return t >= compileEpoch() - 2 * 86400L; }

// Remember the time in flash so that, if every network method fails after a
// restart, the clock at least restarts from the last known time.
void saveTime() {
  time_t n = time(nullptr);
  if (!plausibleTime(n)) return;
  Preferences p;
  if (!p.begin("watch", false)) return;
  p.putUInt("t", (uint32_t)n);
  p.end();
}

// Sets the system clock from a UTC epoch value.
bool setClockUtc(time_t secs, const char *src) {
  if (!plausibleTime(secs)) {
    Serial.print(F("Ignored implausible time from ")); Serial.println(src);
    return false;
  }
  struct timeval tv;
  tv.tv_sec = secs;
  tv.tv_usec = 0;
  settimeofday(&tv, nullptr);
  timeSynced = true;
  timeSource = src;
  lastSaveMs = millis();
  saveTime();
  Serial.print(F("Time set via ")); Serial.println(src);
  return true;
}

// "Sat, 17 Oct 2026 06:56:08 GMT" -> epoch (HTTP dates are always GMT).
bool parseHttpDate(const String &d, time_t &out) {
  struct tm t;
  memset(&t, 0, sizeof(t));
  if (!strptime(d.c_str(), "%a, %d %b %Y %H:%M:%S", &t)) return false;
  out = epochFromUtc(t.tm_year + 1900, t.tm_mon + 1, t.tm_mday,
                     t.tm_hour, t.tm_min, t.tm_sec);
  return true;
}

// ---- Method 1: NTP over UDP ----
bool ntpQuery(IPAddress ip) {
  WiFiUDP udp;
  if (!udp.begin(2390)) return false;
  uint8_t pkt[48];
  memset(pkt, 0, sizeof(pkt));
  pkt[0] = 0x23;                      // leap 0, version 4, client mode
  udp.beginPacket(ip, 123);
  udp.write(pkt, sizeof(pkt));
  udp.endPacket();

  unsigned long t0 = millis();
  while (millis() - t0 < 1500) {
    if (udp.parsePacket() >= 48) {
      udp.read(pkt, sizeof(pkt));
      udp.stop();
      uint32_t secs = ((uint32_t)pkt[40] << 24) | ((uint32_t)pkt[41] << 16) |
                      ((uint32_t)pkt[42] << 8) | (uint32_t)pkt[43];
      if (secs < 2208988800UL) return false;   // not a valid answer
      return setClockUtc((time_t)(secs - 2208988800UL), "NTP");
    }
    delay(10);
  }
  udp.stop();
  return false;
}

bool timeViaNtp() {
  Serial.println(F("Method 1: NTP"));
  const char *hosts[] = { NTP_1, NTP_2, "pool.ntp.org", "time.google.com",
                          "time.cloudflare.com", "time.windows.com",
                          "time.nist.gov" };
  // Raw IPs work even if the campus DNS is broken (Cloudflare, Google, NIST).
  const char *ips[] = { "162.159.200.1", "216.239.35.0", "129.6.15.28" };
  byte timeouts = 0;

  for (byte i = 0; i < sizeof(hosts) / sizeof(hosts[0]); i++) {
    IPAddress ip;
    if (WiFi.hostByName(hosts[i], ip) != 1) continue;
    Serial.print(F("  ")); Serial.println(hosts[i]);
    if (ntpQuery(ip)) return true;
    if (++timeouts >= 3) {
      Serial.println(F("  UDP port 123 looks blocked, skipping NTP."));
      return false;
    }
  }
  for (byte i = 0; i < sizeof(ips) / sizeof(ips[0]); i++) {
    IPAddress ip;
    if (!ip.fromString(ips[i])) continue;
    Serial.print(F("  ")); Serial.println(ips[i]);
    if (ntpQuery(ip)) return true;
    if (++timeouts >= 3) {
      Serial.println(F("  UDP port 123 looks blocked, skipping NTP."));
      return false;
    }
  }
  return false;
}

// ---- Methods 2 and 3: "Date" header of a web reply ----
bool httpDate(const char *url, bool secure) {
  WiFiClientSecure sclient;     // declared first so it outlives "http"
  HTTPClient http;
  http.setConnectTimeout(2500);
  http.setTimeout(2500);

  bool began;
  if (secure) {
    sclient.setInsecure();      // only the time is read, nothing secret is sent
    began = http.begin(sclient, url);
  } else {
    began = http.begin(url);
  }
  if (!began) return false;

  const char *keys[] = { "Date" };
  http.collectHeaders(keys, 1);
  int code = http.sendRequest("HEAD");
  String d;
  if (code > 0 && http.hasHeader("Date")) d = http.header("Date");
  http.end();

  Serial.print(F("  ")); Serial.print(url);
  Serial.print(F(" -> ")); Serial.println(code);
  // Any reply counts, even a redirect or an error page: the Date header
  // is still the real time.
  if (d.length() == 0) return false;

  time_t secs;
  if (!parseHttpDate(d, secs)) {
    Serial.print(F("  unreadable Date: ")); Serial.println(d);
    return false;
  }
  return setClockUtc(secs, secure ? "HTTPS Date" : "HTTP Date");
}

bool timeViaHttp(bool secure) {
  static const char *plain[] = {
    "http://connectivitycheck.gstatic.com/generate_204",
    "http://clients3.google.com/generate_204",
    "http://www.google.com",
    "http://detectportal.firefox.com/success.txt",
    "http://captive.apple.com",
    "http://www.msftconnecttest.com/connecttest.txt",
    "http://1.1.1.1/",          // raw IPs: work even without DNS
    "http://1.0.0.1/",
    "http://208.67.222.222/"
  };
  static const char *tls[] = {
    "https://www.google.com",
    "https://www.cloudflare.com",
    "https://www.microsoft.com"
  };

  Serial.println(secure ? F("Method 3: HTTPS Date") : F("Method 2: HTTP Date"));
  if (secure) {
    for (byte i = 0; i < sizeof(tls) / sizeof(tls[0]); i++)
      if (httpDate(tls[i], true)) return true;
  } else {
    for (byte i = 0; i < sizeof(plain) / sizeof(plain[0]); i++)
      if (httpDate(plain[i], false)) return true;
  }
  return false;
}

// ---- Method 4: worldtimeapi.org (JSON with "unixtime") ----
bool timeViaWorldTimeApi() {
  Serial.println(F("Method 4: worldtimeapi.org"));
  HTTPClient http;
  http.setConnectTimeout(3000);
  http.setTimeout(3000);
  if (!http.begin("http://worldtimeapi.org/api/timezone/Etc/UTC")) return false;
  int code = http.GET();
  String body;
  if (code == 200) body = http.getString();
  http.end();
  Serial.print(F("  worldtimeapi -> ")); Serial.println(code);

  int i = body.indexOf("\"unixtime\":");
  if (i < 0) return false;
  long u = body.substring(i + 11).toInt();
  if (u <= 0) return false;
  return setClockUtc((time_t)u, "worldtimeapi");
}

// ---- WiFi ----
// WiFi status codes: 3 = connected, 1 = network not found (out of range, or
// 5 GHz only: the ESP32 only sees 2.4 GHz), 4 = login/password rejected,
// 6 = dropped.
bool connectWifi(bool enterprise, const char *ssid, const char *pass) {
  WiFi.disconnect(true);
  WiFi.mode(WIFI_STA);
  if (enterprise) WiFi.begin(ssid, WPA2_AUTH_PEAP, WIFI_IDENTITY, WIFI_IDENTITY, pass);
  else            WiFi.begin(ssid, pass);

  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < WIFI_TIMEOUT_MS) {
    delay(100);
  }
  wifiStatus = (int)WiFi.status();
  Serial.print(F("WiFi '")); Serial.print(ssid);
  Serial.print(F("' status=")); Serial.println(wifiStatus);
  return wifiStatus == WL_CONNECTED;
}

bool dnsOk() {
  IPAddress ip;
  return WiFi.hostByName("google.com", ip) == 1;
}

// If the network's own DNS does not work, switch to public DNS servers.
void ensureDns() {
  if (dnsOk()) { Serial.println(F("DNS ok")); return; }
  Serial.println(F("DNS failed - trying public DNS 8.8.8.8 / 1.1.1.1"));
  WiFi.config(WiFi.localIP(), WiFi.gatewayIP(), WiFi.subnetMask(),
              IPAddress(8, 8, 8, 8), IPAddress(1, 1, 1, 1));
  delay(300);
  if (dnsOk()) Serial.println(F("DNS ok now"));
  else Serial.println(F("DNS still failing: only raw-IP methods can work."));
}

// Runs all methods. boot = true: if everything fails, start from the last
// saved time (or the compile time). A later retry never overwrites a clock
// that is already running.
void syncTime(bool boot) {
  showLogo(F("Syncing time..."));
  webPaused = true;
  delay(40);                      // let the web task finish its current request

  setenv("TZ", TZ_INFO, 1);
  tzset();
  compileEpoch();                 // computed now, while TZ is set

  bool up = connectWifi(true, WIFI_SSID, WIFI_PASS);
  // Second try for the campus login, unless the network is simply not visible.
  if (!up && wifiStatus != WL_NO_SSID_AVAIL) up = connectWifi(true, WIFI_SSID, WIFI_PASS);
  if (!up && HOTSPOT_SSID[0]) up = connectWifi(false, HOTSPOT_SSID, HOTSPOT_PASS);

  bool ok = false;
  if (up) {
    Serial.print(F("IP: ")); Serial.println(WiFi.localIP());
    ensureDns();
    ok = timeViaNtp() || timeViaHttp(false) || timeViaHttp(true) ||
         timeViaWorldTimeApi();
  }

  // The campus WiFi is not needed any more: the ESP32 keeps time on its own.
  // Bring up the watch's own network (and the web page) instead.
  startAccessPoint();

  if (ok) {
    Serial.print(F("Time synced (")); Serial.print(timeSource);
    Serial.println(F(")."));
    nextSyncMs = 0;
    return;
  }

  if (boot) {
    // Last resort: the later of the compile time and the time saved in flash.
    time_t base = compileEpoch();
    const char *src = "compile time";
    Preferences p;
    if (p.begin("watch", true)) {
      uint32_t saved = p.getUInt("t", 0);
      p.end();
      if ((time_t)saved > base) { base = (time_t)saved; src = "last saved time"; }
    }
    struct timeval tv;
    tv.tv_sec = base;
    tv.tv_usec = 0;
    settimeofday(&tv, nullptr);
    Serial.print(F("All time methods failed - using ")); Serial.println(src);
    Serial.println(F("Set it by typing:  T 15:30   or   T 2026-10-17 15:30:00"));
    Serial.println(F("(Serial Monitor: 9600 baud, Newline). Type SYNC to retry."));
  } else {
    Serial.println(F("Retry failed."));
  }
  nextSyncMs = millis() + 60000UL;   // try again in a minute (see timeMaintenance)
}

// ---- Manual time setting and retries ----
void processCommand(char *s) {
  while (*s == ' ') s++;
  if (strcasecmp(s, "SYNC") == 0) {
    syncRequested = true;
    Serial.println(F("Sync requested (starts when no finger is on the sensor)."));
    return;
  }
  if (s[0] != 'T' && s[0] != 't') {
    Serial.println(F("Commands:  T 15:30   |   T 2026-10-17 15:30:00   |   SYNC"));
    return;
  }

  int Y = 0, M = 0, D = 0, h = 0, m = 0, sec = 0;
  struct tm t;
  memset(&t, 0, sizeof(t));

  int n = sscanf(s + 1, "%d-%d-%d %d:%d:%d", &Y, &M, &D, &h, &m, &sec);
  if (n >= 5) {
    t.tm_year = Y - 1900;
    t.tm_mon = M - 1;
    t.tm_mday = D;
    t.tm_hour = h;
    t.tm_min = m;
    t.tm_sec = (n >= 6) ? sec : 0;
  } else {
    h = m = sec = 0;
    if (sscanf(s + 1, "%d:%d:%d", &h, &m, &sec) < 2) {
      Serial.println(F("Use:  T 15:30   or   T 2026-10-17 15:30:00"));
      return;
    }
    time_t nowT = time(nullptr);
    localtime_r(&nowT, &t);          // keep today's date
    t.tm_hour = h;
    t.tm_min = m;
    t.tm_sec = sec;
  }
  if (t.tm_hour < 0 || t.tm_hour > 23 || t.tm_min < 0 || t.tm_min > 59 ||
      t.tm_sec < 0 || t.tm_sec > 59 || t.tm_mon < 0 || t.tm_mon > 11 ||
      t.tm_mday < 1 || t.tm_mday > 31) {
    Serial.println(F("That is not a valid time."));
    return;
  }
  t.tm_isdst = -1;
  if (setClockUtc(mktime(&t), "manual entry")) {
    char buf[32];
    time_t nowT = time(nullptr);
    struct tm lt;
    localtime_r(&nowT, &lt);
    strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &lt);
    Serial.print(F("Clock is now ")); Serial.println(buf);
  }
}

void handleSerialCommands() {
  static char line[40];
  static byte len = 0;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (len) {
        line[len] = 0;
        len = 0;
        processCommand(line);
      }
    } else if (len < sizeof(line) - 1) {
      line[len++] = c;
    }
  }
}

// Called from loop(): serial commands, saving the time to flash, and
// automatic retries while the clock is still not synced. Never blocks
// while a finger is on the sensor.
void timeMaintenance() {
  handleSerialCommands();
  unsigned long now = millis();

  if (timeSynced && now - lastSaveMs > 600000UL) {
    lastSaveMs = now;
    saveTime();
  }

  if (fingerPresent) return;

  bool due = !timeSynced && nextSyncMs != 0 && (long)(now - nextSyncMs) >= 0;
  if (syncRequested || due) {
    syncRequested = false;
    syncAttempts++;
    syncTime(false);
    // The sensor kept sampling meanwhile: drop the backlog.
    sensor.clearFIFO();
    calStartMs = 0;
    if (!timeSynced) {
      // 1 min, then 5 min, then every 15 min.
      nextSyncMs = millis() + (syncAttempts < 2 ? 300000UL : 900000UL);
    }
  }
}

// ---------- Web page (the ESP32's own WiFi network) ----------
// The watch creates its own WiFi network (AP_SSID / AP_PASS at the top).
// Join it with a phone or laptop and open http://192.168.4.1 to see the
// readings live. No cable and no campus WiFi needed.
// The web server runs on the ESP32's second core so it never delays the
// sensor readings. It pauses while the time is being synced (the ESP32
// switches WiFi mode then) and restarts afterwards.

WebServer server(80);

const char PAGE[] PROGMEM = R"HTML(<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>SG&amp;Co Watch</title>
<style>
:root{--bg:#f6f7f9;--card:#fff;--ink:#14171c;--mute:#6b7380;--line:#e3e6ea;--acc:#e5484d;--ok:#2a9d6f;color-scheme:light}
@media(prefers-color-scheme:dark){:root:not([data-theme=light]){--bg:#0e1013;--card:#171a1f;--ink:#eef0f3;--mute:#8d95a3;--line:#262b33;--acc:#ff6369;--ok:#3ddc97;color-scheme:dark}}
:root[data-theme=dark]{--bg:#0e1013;--card:#171a1f;--ink:#eef0f3;--mute:#8d95a3;--line:#262b33;--acc:#ff6369;--ok:#3ddc97;color-scheme:dark}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--ink);font:16px/1.4 system-ui,-apple-system,Segoe UI,Roboto,sans-serif}
main{max-width:640px;margin:0 auto;padding:16px}
header{display:flex;flex-wrap:wrap;gap:8px 12px;justify-content:space-between;align-items:center;margin-bottom:12px}
h1{font-size:18px;margin:0;letter-spacing:.04em}
.right{display:flex;align-items:center;gap:10px}
.mute{color:var(--mute);font-size:13px}
.grid{display:grid;grid-template-columns:1fr 1fr;gap:12px}
.card{background:var(--card);border:1px solid var(--line);border-radius:14px;padding:14px}
.wide{grid-column:1/-1}
.lbl{color:var(--mute);font-size:12px;text-transform:uppercase;letter-spacing:.08em}
.big{font-size:52px;font-weight:700;line-height:1.05;font-variant-numeric:tabular-nums}
.unit{font-size:15px;color:var(--mute);font-weight:500;margin-left:4px}
.row{display:flex;justify-content:space-between;align-items:center}
canvas{width:100%;height:110px;display:block}
.pill{display:inline-block;padding:2px 10px;border-radius:99px;font-size:13px;border:1px solid var(--line);color:var(--mute)}
.pill.on{color:var(--ok);border-color:var(--ok)}
button{font:inherit;padding:8px 14px;border-radius:10px;border:1px solid var(--line);background:var(--card);color:var(--ink);cursor:pointer}
.seg{display:inline-flex;border:1px solid var(--line);border-radius:10px;overflow:hidden;background:var(--card)}
.seg button{border:0;border-radius:0;padding:5px 11px;font-size:13px;background:transparent;color:var(--mute)}
.seg button[aria-pressed=true]{background:var(--ink);color:var(--bg)}
.note{margin:14px 0 0}
</style></head><body><main>
<header><h1>SG&amp;Co Watch</h1>
<div class="right"><span id="clock" class="mute">--:--:--</span>
<div class="seg" id="theme" role="group" aria-label="Colour theme">
<button data-t="auto" aria-pressed="true">Auto</button><button data-t="light" aria-pressed="false">Light</button><button data-t="dark" aria-pressed="false">Dark</button>
</div></div></header>
<div class="grid">
<div class="card"><div class="lbl">Heart rate</div>
<div><span id="bpm" class="big">--</span><span class="unit">BPM</span></div>
<div id="rhythm" class="mute">&nbsp;</div></div>
<div class="card"><div class="lbl">SpO&#8322;</div>
<div><span id="spo2" class="big">--</span><span class="unit">%</span></div>
<div class="mute">experimental estimate</div></div>
<div class="card"><div class="lbl">HRV (RMSSD)</div>
<div><span id="hrv" class="big">--</span><span class="unit">ms</span></div>
<div id="hrvsub" class="mute">&nbsp;</div></div>
<div class="card"><div class="lbl">Respiratory rate</div>
<div><span id="resp" class="big">--</span><span class="unit">br/min</span></div>
<div id="respsub" class="mute">&nbsp;</div></div>
<div class="card wide"><div class="row"><span class="lbl">Pulse waveform</span><span id="state" class="pill">connecting</span></div>
<canvas id="wave"></canvas></div>
<div class="card wide"><div class="lbl">Heart rate, last 5 minutes</div>
<canvas id="hist"></canvas><div class="mute" id="range">&nbsp;</div></div>
</div>
<p class="mute note"><button id="sync">Set watch time from this device</button> <span id="msg"></span></p>
<p class="mute">Estimates from a fingertip sensor sampled about 20 times a second, so they are approximate. HRV needs about 20 s of steady beats and breathing about 70 s; keep your hand still. Not a medical device.</p>
</main>
<script>
const $=id=>document.getElementById(id);
const css=n=>getComputedStyle(document.documentElement).getPropertyValue(n).trim();
const hist=[];let fails=0,auto=false,last=0,lastD=null;
function fit(c){const r=devicePixelRatio||1,w=c.clientWidth,h=c.clientHeight;
 if(c.width!==Math.round(w*r)){c.width=Math.round(w*r);c.height=Math.round(h*r)}
 const x=c.getContext('2d');x.setTransform(r,0,0,r,0,0);x.clearRect(0,0,w,h);return[x,w,h]}
function line(c,pts,lo,hi){const[x,w,h]=fit(c);if(pts.length<2)return;
 x.strokeStyle=css('--acc');x.lineWidth=2;x.lineJoin='round';x.beginPath();
 pts.forEach((v,i)=>{const px=i/(pts.length-1)*w,py=h-6-(v-lo)/(hi-lo)*(h-12);i?x.lineTo(px,py):x.moveTo(px,py)});
 x.stroke()}
function setTheme(t){const r=document.documentElement;
 if(t==='auto')r.removeAttribute('data-theme');else r.setAttribute('data-theme',t);
 document.querySelectorAll('#theme button').forEach(b=>b.setAttribute('aria-pressed',String(b.dataset.t===t)));
 try{localStorage.setItem('theme',t)}catch(e){}
 if(lastD)draw(lastD)}
async function setTime(){try{const r=await fetch('/settime?ms='+Date.now());
 $('msg').textContent=r.ok?'Time set.':'Rejected.'}catch(e){$('msg').textContent='Failed.'}}
function draw(d){
 const NB=' ';
 $('bpm').textContent=d.bpm||'--';$('spo2').textContent=d.spo2||'--';
 $('rhythm').textContent=d.bpm?(d.rhythm==2?'Irregular rhythm':d.rhythm==1?'Regular rhythm':'Measuring...'):NB;
 $('hrv').textContent=d.rmssd||'--';
 $('hrvsub').textContent=d.rmssd?('SDNN '+d.sdnn+' ms'):(d.finger&&d.bpm?('collecting '+Math.min(d.hrvn,20)+'/20 beats'):NB);
 $('resp').textContent=d.resp||'--';
 $('respsub').textContent=d.resp?'experimental':(d.finger?('collecting '+d.respp+'%'):NB);
 $('clock').textContent=d.time+(d.synced?'':' (not synced)');
 const st=$('state');
 if(!d.finger){st.textContent='place finger on sensor';st.className='pill'}
 else if(d.env<8){st.textContent='low signal';st.className='pill'}
 else{st.textContent='measuring';st.className='pill on'}
 line($('wave'),d.wave.map(v=>25-v),0,25);
 if(hist.length>1){const mn=Math.min(...hist),mx=Math.max(...hist),lo=mn-5,hi=Math.max(mx+5,lo+20);
  line($('hist'),hist,lo,hi);$('range').textContent='min '+mn+' · max '+mx+' BPM'}
 else line($('hist'),[],0,1)}
async function tick(){
 try{
  const d=await(await fetch('/data',{cache:'no-store'})).json();fails=0;lastD=d;
  const now=Date.now();
  if(d.bpm&&now-last>2000){last=now;hist.push(d.bpm);if(hist.length>150)hist.shift()}
  draw(d);
  if(!d.synced&&!auto){auto=true;setTime()}
 }catch(e){if(++fails>3){$('state').textContent='disconnected';$('state').className='pill'}}
 setTimeout(tick,400)}
document.querySelectorAll('#theme button').forEach(b=>b.onclick=()=>setTheme(b.dataset.t));
let saved='auto';try{saved=localStorage.getItem('theme')||'auto'}catch(e){}
setTheme(saved);
$('sync').onclick=setTime;tick();
</script></body></html>
)HTML";

void handleRoot() {
  server.sendHeader("Cache-Control", "no-store");
  server.send_P(200, "text/html", PAGE);
}

void handleData() {
  int bpmShow = (bpmInt >= 30 && bpmInt <= 200) ? bpmInt : 0;
  int spo2Show = (spo2 >= 70) ? (int)(spo2 + 0.5f) : 0;

  time_t nowT = time(nullptr);
  struct tm lt;
  localtime_r(&nowT, &lt);
  char tb[12];
  strftime(tb, sizeof(tb), "%H:%M:%S", &lt);

  // Progress towards the first breathing rate: settling time, one full 30 s
  // window, then a second window that has to confirm it (see updateResp).
  int respPct = 0;
  if (respSkip <= 0) {
    int done = respFill + (respFill == RESP_N ? respUpd * 8 + respSince : 0);
    respPct = done * 100 / (RESP_N + 16 * 8);
    if (respPct > 99) respPct = 99;
  }

  String j;
  j.reserve(520);
  j += F("{\"bpm\":");      j += bpmShow;
  j += F(",\"spo2\":");     j += spo2Show;
  j += F(",\"finger\":");   j += (fingerPresent ? 1 : 0);
  j += F(",\"rhythm\":");   j += (int)rhythm;
  j += F(",\"env\":");      j += (int)env;
  j += F(",\"synced\":");   j += (timeSynced ? 1 : 0);
  j += F(",\"rmssd\":");    j += (int)(hrvRmssd + 0.5f);
  j += F(",\"sdnn\":");     j += (int)(hrvSdnn + 0.5f);
  j += F(",\"hrvn\":");     j += (int)hrvPairs;
  j += F(",\"resp\":");     j += (respRate > 0 ? (int)(respRate + 0.5f) : 0);
  j += F(",\"respp\":");    j += respPct;
  j += F(",\"time\":\"");   j += tb;
  j += F("\",\"wave\":[");
  for (byte i = 0; i < WAVE_W; i++) {          // oldest point first
    if (i) j += ',';
    j += (int)waveBuf[(waveIdx + i) % WAVE_W];
  }
  j += F("]}");

  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", j);
}

// The page sends the phone's clock; used when the watch could not sync.
void handleSetTime() {
  uint64_t ms = strtoull(server.arg("ms").c_str(), nullptr, 10);
  if (ms == 0 || !setClockUtc((time_t)(ms / 1000ULL), "web page")) {
    server.send(400, "text/plain", "bad time");
    return;
  }
  server.send(200, "text/plain", "ok");
}

void webTask(void *) {
  for (;;) {
    if (!webPaused) server.handleClient();
    vTaskDelay(pdMS_TO_TICKS(3));
  }
}

// Registers the pages and starts the web task (it stays paused until the
// access point is up).
void startWeb() {
  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.on("/settime", handleSetTime);
  server.onNotFound([]() { server.send(404, "text/plain", "not found"); });
  xTaskCreatePinnedToCore(webTask, "web", 8192, nullptr, 1, nullptr, 0);
}

// Turns on the watch's own WiFi network and (re)starts the web server.
void startAccessPoint() {
  WiFi.disconnect(true);
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  delay(100);
  Serial.print(F("Web page: join WiFi '")); Serial.print(AP_SSID);
  Serial.print(F("' (password ")); Serial.print(AP_PASS);
  Serial.print(F("), then open http://")); Serial.println(WiFi.softAPIP());
  server.stop();
  server.begin();
  webPaused = false;
}

// ---------- Watch face (no finger) ----------
// Layout (128x64):
//   y  4..35  : HH:MM, size 4 (colon blinks every second)
//   y 38..39  : seconds progress bar
//   y 44      : date, centered
//   y 56      : hint
void drawWatch() {
  char buf[24];
  time_t nowT = time(nullptr);
  struct tm lt;
  localtime_r(&nowT, &lt);

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextWrap(false);

  // Time (colon blinks).
  strftime(buf, sizeof(buf), "%H:%M", &lt);
  if (lt.tm_sec % 2) buf[2] = ' ';
  display.setTextSize(4);
  display.setCursor(4, 4);
  display.print(buf);

  // Seconds progress bar.
  int w = ((lt.tm_sec + 1) * SCREEN_WIDTH) / 60;
  display.fillRect(0, 38, w, 2, SSD1306_WHITE);

  // Date, centered.
  strftime(buf, sizeof(buf), "%a, %d %b %Y", &lt);
  display.setTextSize(1);
  display.setCursor((SCREEN_WIDTH - (int)strlen(buf) * 6) / 2, 46);
  display.print(buf);

  // Hint.
  const char *hint = timeSynced ? "Place finger for BPM" : "Time not synced";
  display.setCursor((SCREEN_WIDTH - (int)strlen(hint) * 6) / 2, 56);
  display.print(hint);

  display.display();
}

// ---------- Vitals screen (finger present) ----------
// Layout (128x64):
//   y  0..7   : "BPM" / "SpO2" labels
//   y 10..25  : values (+ heart icon between them)
//   y 27..34  : rhythm status
//   y 36      : separator line
//   y 38..63  : live PPG waveform
void drawVitals() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextWrap(false);

  // Labels.
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print(F("BPM"));
  display.setCursor(72, 0);
  display.print(F("SpO2"));

  // BPM value (held for up to BPM_HOLD_MS if beats are briefly lost).
  display.setCursor(0, 10);
  if (bpmInt >= 30 && bpmInt <= 200) {
    display.setTextSize(2);
    display.print(bpmInt);
  } else if (env < MIN_SIGNAL_QUALITY) {
    display.setTextSize(1);
    display.setCursor(0, 14);
    display.print(F("LOW SIGNAL"));
  } else {
    display.setTextSize(2);
    display.print(F("--"));
  }

  // SpO2 value.
  display.setTextSize(2);
  display.setCursor(72, 10);
  if (spo2 >= 70) {
    int s = (int)(spo2 + 0.5f);
    display.print(s);
    display.setTextSize(1);
    display.setCursor(72 + (s >= 100 ? 36 : 24) + 2, 10);
    display.print(F("%"));
  } else {
    display.print(F("--"));
  }

  // Rhythm status.
  if (bpmInt >= 30) {
    display.setTextSize(1);
    display.setCursor(0, 27);
    if (rhythm == 2) display.print(F("IRREGULAR"));
    else if (rhythm == 1) display.print(F("Regular"));
  }

  // Beat indicator between the BPM and SpO2 values.
  if (millis() < heartBeatTimer) {
    drawHeart(47, 10, SSD1306_WHITE);
  }

  // Separator + live waveform.
  display.drawFastHLine(0, WAVE_TOP - 2, SCREEN_WIDTH, SSD1306_WHITE);
  drawWave();

  display.display();
}

// ---------- Setup ----------
void setup() {
  Serial.begin(9600);   // set the Serial Monitor to 9600 baud

  // ESP32: SDA = GPIO21, SCL = GPIO22
  Wire.begin(I2C_SDA, I2C_SCL);

  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("OLED not found. Check wiring/address."));
    while (1) { delay(10); }
  }

  showLogo(F("Starting..."));

  if (!sensor.begin(Wire, I2C_SPEED_FAST)) {
    showMessage(F("Sensor"), F("Error"));
    Serial.println(F("Sensor not found. Check wiring."));
    while (1) { delay(10); }
  }

  // LED brightness, sample average, LED mode,
  // sample rate, pulse width, ADC range
  sensor.setup(0, SAMPLE_AVG, 2, 400, 411, 4096);
  sensor.setPulseAmplitudeIR(irPower);
  sensor.setPulseAmplitudeRed(redPower);
  sensor.setPulseAmplitudeGreen(0);

  initWave();
  applyRate();
  startWeb();
  syncTime(true);
  showLogo(F("Wi-Fi: " AP_SSID_STR));   // tells you which network to join
  delay(2500);
  resetState();

  // The sensor kept sampling during the WiFi sync; drop that backlog so the
  // sample clock starts clean.
  sensor.clearFIFO();
  tMs = 0;
  calStartMs = 0;

  Serial.print(F("Free heap at start: "));
  Serial.println(freeRam());
}

// ---------- Main loop ----------
void loop() {
  sensor.check();

  // Process all newly available sensor samples.
  while (sensor.available()) {
    int32_t irRaw = sensor.getIR();
    int32_t redRaw = sensor.getRed();
    sensor.nextSample();

    sampleCount++;
    dbgIR = irRaw;

    // Sample clock: every sample is exactly one period after the last.
    tMs += periodMs;
    calibrateClock();

    // ----- Finger detection (debounced both ways) -----
    if (!fingerPresent) {
      if (irRaw >= FINGER_IR_MIN) {
        if (highSince == 0) highSince = millis();
        if (millis() - highSince >= FINGER_ON_MS) {
          startMeasuring(irRaw, redRaw);
        }
      } else {
        highSince = 0;
      }
      if (!fingerPresent) continue;   // watch mode: nothing else to process
    } else {
      if (irRaw < FINGER_IR_MIN) {
        if (lowSince == 0) lowSince = millis();
        if (millis() - lowSince > FINGER_LOST_MS) {
          stopMeasuring();
          continue;
        }
      } else {
        lowSince = 0;
      }
    }

    // Automatic LED adjustment with a wide dead-band: LED power is only
    // changed when the raw level is clearly too weak or too strong, so it
    // does not keep dithering and blocking beat detection.
    bool weak = (irRaw < LED_RAW_LOW || redRaw < LED_RAW_LOW);
    unsigned long adjustEvery = weak ? ADJUST_FAST_MS : ADJUST_SLOW_MS;
    if (millis() - lastAdjustTime > adjustEvery) {
      byte oldIr = irPower, oldRed = redPower;

      if (irRaw < LED_RAW_LOW && irPower < 248) irPower += 3;
      else if (irRaw > LED_RAW_SAT && irPower > 10) irPower -= 5;
      else if (irRaw > LED_RAW_HIGH && irPower > 5) irPower -= 2;

      if (redRaw < LED_RAW_LOW && redPower < 248) redPower += 3;
      else if (redRaw > LED_RAW_SAT && redPower > 10) redPower -= 5;
      else if (redRaw > LED_RAW_HIGH && redPower > 5) redPower -= 2;

      if (irPower != oldIr) sensor.setPulseAmplitudeIR(irPower);
      if (redPower != oldRed) sensor.setPulseAmplitudeRed(redPower);
      lastAdjustTime = millis();

      // Only an IR change affects the BPM signal: re-seed the BPM filter so
      // the step is not mistaken for a pulse, and hold off beat detection
      // briefly. A red-only change does not block beat detection.
      if (irPower != oldIr) {
        dcReseed = true;
        lastPowerChangeMs = millis();
        respRestart();
      }
    }

    // DC removal and AC extraction for the SpO2 estimate.
    dcIR += ((irRaw - dcIR) >> SHIFT_DC);
    dcRED += ((redRaw - dcRED) >> SHIFT_DC);

    int32_t irAC = dcIR - irRaw;
    int32_t redAC = dcRED - redRaw;

    if (irAC > irMax) irMax = irAC;
    if (irAC < irMin) irMin = irAC;
    if (redAC > redMax) redMax = redAC;
    if (redAC < redMin) redMin = redAC;

    // ----- BPM path: band-pass filter (beats point up) -----
    float xin = (float)irRaw;
    if (dcReseed) { bpDC = xin; dcReseed = false; }
    bpDC += hpAlpha * (xin - bpDC);
    float hp = bpDC - xin;
    lp1 += lpAlpha * (hp - lp1);
    lp2 += lpAlpha * (lp1 - lp2);
    float x = lp2 * 0.125f;

    // Feed the live waveform (same signal the beat detector uses).
    pushWave(x);

    // Keep the recent signal for the autocorrelation cross-check.
    acBuf[acIdx] = x;
    acIdx = (acIdx + 1) % AC_N;
    if (acFill < AC_N) acFill++;

    // Respiratory band (about 0.08 - 0.5 Hz) of the raw IR signal.
    if (respReseed) { respHP = xin; respHP2 = 0; respL1 = respL2 = respL3 = 0; respReseed = false; }
    respHP += respHpA * (xin - respHP);
    float rb = xin - respHP;
    respHP2 += respHpA * (rb - respHP2);          // second high-pass stage: less slow drift
    rb -= respHP2;
    respL1 += respLpA * (rb - respL1);
    respL2 += respLpA * (respL1 - respL2);
    respL3 += respLpA * (respL2 - respL3);
    if (++respCnt >= respDecim) {
      respCnt = 0;
      if (respSkip > 0) {
        respSkip--;                                  // let the filters settle
      } else {
        respBuf[respIdx] = respL3;
        respIdx = (respIdx + 1) % RESP_N;
        if (respFill < RESP_N) respFill++;
        if (respFill == RESP_N && ++respSince >= 8) { respSince = 0; updateResp(); }
      }
    }

    // Mean absolute signal level (signal quality + threshold).
    float absx = (x >= 0) ? x : -x;
    env = envAlpha * env + (1.0f - envAlpha) * absx;

    // Slide the peak window and look for a beat in its middle.
    for (byte i = 0; i < pkW - 1; i++) pkBuf[i] = pkBuf[i + 1];
    pkBuf[pkW - 1] = x;
    if (pkFill < pkW) pkFill++;

    double tPeak;
    float amp;
    if (findPeak(tPeak, amp)) handleBeat(tPeak, amp);

    // If no beat interval has been accepted for a while, let the beat height
    // average fade so a past spike cannot keep the threshold too high.
    if (beatAmp > 0 &&
        millis() - lastBeatDetectedMs > BEAT_AMP_DECAY_AFTER_MS) {
      beatAmp *= 0.99f;
    }

    // Only after a long time without any valid beat is the BPM dropped
    // (until then the last value is held on screen).
    if (bpm > 0 &&
        millis() - lastBeatDetectedMs > BPM_HOLD_MS) {
      clearIntervals();
      lastPeakValid = false;
      beatAmp = 0;
    }

    if (spo2 > 0 &&
        millis() - lastSpo2DetectedMs > SPO2_TIMEOUT_MS) {
      spo2 = 0;
      spo2Idx = spo2Cnt = 0;
    }
  }

  timeMaintenance();   // serial commands, time saving, sync retries

  // Redraw the OLED at a fixed rate, outside the sample loop,
  // so slow I2C drawing never makes the sensor FIFO overflow.
  unsigned long interval =
      fingerPresent ? DRAW_INTERVAL_VITALS_MS : DRAW_INTERVAL_WATCH_MS;
  if (millis() - lastDrawMs >= interval) {
    lastDrawMs = millis();
    if (fingerPresent) drawVitals();
    else drawWatch();
  }

#if DEBUG_SERIAL
  // Once a second, kept short so it does not block at 9600 baud
  // (a long line would stall the loop and the sensor would lose samples).
  // last = most recent accepted beat interval in ms (BPM = 60000 / last).
  static unsigned long lastDbgMs = 0;
  if (millis() - lastDbgMs >= 1000) {
    lastDbgMs = millis();
    int lastIbi = ibiCnt ? ibiBuf[(ibiIdx + IBI_N - 1) % IBI_N] : 0;
    Serial.print(F("env=")); Serial.print(env, 0);
    Serial.print(F(" amp=")); Serial.print(beatAmp, 0);
    Serial.print(F(" bpm=")); Serial.print(bpm, 0);
    Serial.print(F(" last=")); Serial.print(lastIbi);
    Serial.print(F(" n=")); Serial.print(ibiCnt);
    Serial.print(F(" rej=")); Serial.print(rejectCnt);
    Serial.print(F(" pk=")); Serial.print(dbgPeaks);   // total beats detected
    Serial.print(F(" ac=")); Serial.print(acBpm(), 0); // second opinion (autocorrelation)
    Serial.print(F(" hrv=")); Serial.print(hrvRmssd, 0);        // RMSSD in ms
    Serial.print('/'); Serial.print((int)hrvPairs);              // pairs used (need 20)
    Serial.print(F(" rr=")); Serial.print(respRate, 1);          // breaths/min
    Serial.print('/'); Serial.print(respQ, 0);                   // peak sharpness (needs >= 12)
    static unsigned long lastCnt = 0;
    Serial.print(F(" sps=")); Serial.print(sampleCount - lastCnt);   // should be ~50
    lastCnt = sampleCount;
    Serial.print(F(" T=")); Serial.println(periodMs, 1);             // should be ~20.0
  }



#endif
}