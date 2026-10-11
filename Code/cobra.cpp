// ======================================================
//  ESP32 DANCING COBRA  (PlatformIO / Arduino framework)
//
//  1. Sleeps coiled, servos relaxed.
//  2. Rises ONLY for MUSIC. Talking, claps and knocks don't wake it.
//     (music = its spectrum stays steady over time; speech changes every
//      syllable and keeps pausing between words)
//  3. Rises base-first like a cobra rearing up, then dances with a
//     traveling serpentine wave through all 7 joints.
//  4. 1st clap -> freezes straight up.  2nd clap -> dances again.
//  5. Music stops (silence OR people talking) -> lowers and sleeps.
//
//  At power-on the snake does a short "stretch": rises, dances silently
//  for ~4 s to measure its OWN servo noise, then lies down again.
//  Keep the room quiet for the first ~10 s after power-on.
//
//  Reliability fixes for "stops after some time":
//   - PCA9685 is checked every 2 s and re-initialised if it reset/locked up
//   - I2C at 100 kHz with a timeout (more robust next to servo wiring noise)
//   - Boot after a brown-out while music plays no longer breaks wake-up
//     (last quiet ambient level is saved in flash)
//   - Ambient level can't drift up during long music sessions
//   - Reset reason is printed at boot for diagnosis
// ======================================================

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <arduinoFFT.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <esp_system.h>

// ======================================================
// WiFi
// ======================================================
const char* ssid     = "Snake_Robot";
const char* password = "12345679";
WebServer server(80);

// ======================================================
// SERVOS (PCA9685)
// ======================================================
#define NUM_SERVOS 7
const int MIC_PIN = 34;
const uint8_t PCA_ADDR = 0x40;

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA_ADDR);

const int pcaChannels[NUM_SERVOS] = { 8, 9, 10, 11, 12, 13, 14 };
const int servoMins[NUM_SERVOS]   = { 75, 102, 102, 102, 102, 102, 102 };
const int servoMaxs[NUM_SERVOS]   = { 510, 500, 500, 500, 500, 500, 532 };

// Lying (coiled) pose - calibrated, do not change
const int restingAngles[NUM_SERVOS] = { 150, 27, 117, 0, 120, 180, 90 };

// Standing pose (dance center) - calibrated, do not change
const int centerAngles[NUM_SERVOS]  = { 72, 67, 66, 129, 76, 71, 119 };

int   currentAngles[NUM_SERVOS];
float smoothedServoAngles[NUM_SERVOS];
bool  servosPowered = false;
uint8_t expectedPrescale = 0;

// ======================================================
// FFT  (256 samples @ 10 kHz = 25.6 ms per frame)
// ======================================================
const uint16_t SAMPLES = 256;
const float SAMPLING_FREQ = 10000.0f;
const unsigned int SAMPLING_PERIOD_US = 100;

float vReal[SAMPLES];
float vImag[SAMPLES];

ArduinoFFT<float> FFT = ArduinoFFT<float>(vReal, vImag, SAMPLES, SAMPLING_FREQ);

struct AudioFrame {
  int   level;      // peak-to-peak amplitude
  float pitch;      // dominant frequency 80..1600 Hz
  float hfRatio;    // share of energy above 2 kHz (claps are "bright")
  float tonality;   // spectral peak / mean (melody high, noise ~2)
  float bass;       // FFT energy  78 -  234 Hz
  float mid;        // FFT energy 273 -  977 Hz
  float high;       // FFT energy  1.0 - 3.0 kHz
};

// ======================================================
// AUDIO PARAMETERS  (tune with the Serial Monitor output)
// ======================================================
float ambientLevel    = 80.0f;   // room level, servos OFF
float quietBase       = 80.0f;   // trusted quiet level (boot / flash)
float servoNoiseExtra = 0.0f;    // extra mic level while servos hold still
float danceNoiseLevel = 0.0f;    // mic level of the snake's own dancing (mean + 1 std)

const int   RISE_MARGIN          = 45;      // level above ambient = "loud" (sleeping)
const float DANCE_LOUD_MARGIN    = 20.0f;   // level above own dance noise = "loud" (dancing)
const float FROZEN_LOUD_MARGIN   = 25.0f;   // level above holding noise = "loud" (frozen)

// --- MUSIC SCORE (0..1) ---
// Every ~60 ms the spectrum is compared with the previous one (correlation).
// Music: notes/chords are held -> spectra stay similar -> score 0.6-0.8.
// Speech: sound changes every syllable + pauses     -> score 0.2-0.45.
// Silence or the snake's own servo noise            -> score ~0.
// The exact numbers depend on the mic, speaker and room, so the snake LEARNS
// them: web page -> "Learn MUSIC" (play a song) and "Learn TALKING" (talk).
// Learned values are saved in flash. These are only the starting defaults.
float musicScoreWake = 0.42f;   // score above this (for 1.5 s) -> rise
float musicScoreStay = 0.34f;   // score below this (for 3 s)   -> sleep
float learnedMusic = -1.0f;     // average score of your music  (-1 = not learned)
float learnedTalk  = -1.0f;     // average score of talking     (-1 = not learned)
const float SCORE_TAU_MS     = 1200.0f;     // smoothing time
const unsigned long SIM_INTERVAL_MS = 90;   // compare spectra this far apart

// Learning
int   learnMode = 0;            // 0 none, 1 music, 2 talking
unsigned long learnStart = 0;
double learnSum = 0;
int   learnN = 0;
char  learnMsg[120] = "Not learned yet - using default thresholds";
const unsigned long LEARN_WARMUP_MS = 2500;
const unsigned long LEARN_TOTAL_MS  = 10000;

const float MUSIC_CONFIRM_MS = 1500.0f;     // score above WAKE this long -> rise
const unsigned long MUSIC_OFF_TIMEOUT_MS = 3000;  // score below STAY this long -> sleep
const unsigned long WAKE_COOLDOWN_MS = 2500;

// --- Self-noise "stretch" at boot ---
const bool  CALIBRATE_DANCE_AT_BOOT = true;
const unsigned long STRETCH_MS = 4500;

// ======================================================
// RISE / LOWER
// ======================================================
// 0 = all joints move together; 0.4 = joints start one after another
// (base first when rising, head first when lowering). Also lowers the
// current peak on the servo supply compared with moving all at once.
const float STAGGER = 0.4f;

// ======================================================
// DANCE PARAMETERS
// ======================================================
const float MAX_BEND      = 15.0f;   // HARD LIMIT each side of centerAngles
const float DANCE_MIN_AMP = 5.0f;    // bend with very soft music
const float BEAT_ACCENT_DEG = 5.0f;  // extra head "strike" on each bass beat
const float DANCE_MAX_AMP = 15.0f;

const float DANCE_MIN_FREQ       = 0.35f;  // Hz, lowest pitch
const float DANCE_MAX_FREQ       = 1.20f;  // Hz, highest pitch
const float WAVE_PHASE_PER_JOINT = 0.90f;  // rad between joints
const float MAX_SLEW_DEG         = 4.0f;   // max deg per servo per update

// 0 = side-to-side joint, 1 = front-back joint (alternating = original wiring)
const int jointAxis[NUM_SERVOS] = { 0, 1, 0, 1, 0, 1, 0 };
// Flip to -1 for a joint that bends the wrong way in the wave
const int jointDir[NUM_SERVOS]  = { 1, 1, 1, 1, 1, 1, 1 };

float danceFreq = 0.5f;

// ======================================================
// CLAP DETECTOR
// Sudden bright jump above the recent level that dies away quickly.
// ======================================================
const int   CLAP_SPIKE_THRESHOLD = 260;
const float CLAP_HF_RATIO        = 0.40f;
const unsigned long CLAP_DEBOUNCE_MS = 1000;
const bool  CLAP_REQUIRE_SHORT   = false;  // true = ignore long bright sounds (drums)
const unsigned long CLAP_MAX_LEN_MS = 150;

unsigned long lastClapTime = 0;
bool  clapPending = false;
unsigned long clapCandTime = 0;
float clapCandBaseline = 0.0f;

// ======================================================
// STATE
// ======================================================
enum SnakeState {
  STATE_SLEEPING,
  STATE_RISING,
  STATE_DANCING,
  STATE_FROZEN,
  STATE_LOWERING,
  STATE_MANUAL
};
SnakeState currentState = STATE_SLEEPING;

enum Mode { MODE_AUDIO, MODE_MANUAL, MODE_SLEEP };
Mode activeMode = MODE_AUDIO;

// Web buttons ONLY change these flags. The main loop acts on them.
volatile bool manualMode  = false;
volatile bool forceSleep  = false;
volatile bool motionAbort = false;

// Music-score state
#define SPEC_LO 3     // bin 3  = 117 Hz
#define SPEC_HI 51    // bin 51 = 1992 Hz
#define SPEC_N  (SPEC_HI - SPEC_LO + 1)
float curSpec[SPEC_N];
float prevMag[SPEC_N];        // previous frame, averaged in to reduce noise
float refSpec[SPEC_N];
bool  refLoud = false;
unsigned long refTime = 0;
float musicScore = 0.0f;
float musicAboveMs = 0.0f;
unsigned long wakeBlockedUntil = 0;
unsigned long lastAudioMs = 0;
unsigned long lastMusicTime = 0;
unsigned long lastDanceUpdateMs = 0;
float phase = 0.0f;
float slowPhase = 0.0f;

// FFT dance analysis: each band is auto-scaled between its own running
// noise floor and running peak, so the dance works at any volume.
struct BandTracker {
  float floorV = 0, peakV = 1, norm = 0;
  void reset(float v) { floorV = v; peakV = v * 2.0f + 1.0f; norm = 0; }
  float update(float v) {
    floorV = (v < floorV) ? floorV + (v - floorV) * 0.30f : floorV + (v - floorV) * 0.004f;
    peakV  = (v > peakV)  ? v : peakV + (floorV - peakV) * 0.006f;
    float range = fmaxf(peakV - floorV, 1.0f);
    float x = constrain((v - floorV) / range, 0.0f, 1.0f);
    norm += (x - norm) * ((x > norm) ? 0.5f : 0.15f);    // fast attack, slow release
    return norm;
  }
};
BandTracker bassT, midT, highT;
float bassAvg = 0.0f;
unsigned long lastBeatMs = 0;
float beatPulse = 0.0f;
float jointAmp[NUM_SERVOS];
float smoothPitch = 220.0f;
float recentLevel = 0.0f;

Preferences prefs;

// ======================================================
// PROTOTYPES
// ======================================================
void sendJsonResponse();
void getAudioData(AudioFrame &f);
float measureAverageLevel(int frames);
bool moveTo(const int target[], int steps, int stepMs);
bool moveStaggered(const int target[], int steps, int stepMs, bool baseFirst);
bool goToSleep();
Mode requestedMode();
bool enterMode(Mode m);
void audioUpdate();
void danceStep(const AudioFrame &f);
const char* stateName();
void checkPca();

// ======================================================
// HELPERS
// ======================================================
void smartDelay(unsigned long ms) {
  unsigned long start = millis();
  while (millis() - start < ms) {
    server.handleClient();
    delay(1);
  }
}

void updateServo(int index, float angle) {
  if (index < 0 || index >= NUM_SERVOS) return;
  angle = constrain(angle, 0.0f, 180.0f);
  smoothedServoAngles[index] = angle;
  currentAngles[index] = (int)lroundf(angle);

  int pulse = servoMins[index] +
              (int)lroundf((servoMaxs[index] - servoMins[index]) * angle / 180.0f);
  pwm.setPWM(pcaChannels[index], 0, pulse);
}

void attachAllServos() {
  servosPowered = true;
  for (int i = 0; i < NUM_SERVOS; i++) updateServo(i, smoothedServoAngles[i]);
}

void detachAllServos() {
  servosPowered = false;
  for (int i = 0; i < NUM_SERVOS; i++) pwm.setPWM(pcaChannels[i], 0, 0);
}

// Level a sound must exceed to count as "loud" in the current state.
float loudThreshold() {
  switch (currentState) {
    case STATE_DANCING:
      return fmaxf(danceNoiseLevel + DANCE_LOUD_MARGIN, ambientLevel + DANCE_LOUD_MARGIN);
    case STATE_FROZEN:
      return ambientLevel + servoNoiseExtra + FROZEN_LOUD_MARGIN;
    default:
      return ambientLevel + RISE_MARGIN;
  }
}

// Correlation of the current spectrum with the one ~60 ms earlier,
// smoothed into musicScore. Quiet frames count as 0 (pauses = not music).
void updateMusicScore(bool loud, unsigned long now) {
  unsigned long el = now - refTime;
  if (el < SIM_INTERVAL_MS) return;

  float sim = 0.0f;
  if (loud && refLoud) {
    float ma = 0, mb = 0;
    for (int k = 0; k < SPEC_N; k++) { ma += curSpec[k]; mb += refSpec[k]; }
    ma /= SPEC_N; mb /= SPEC_N;
    float sab = 0, saa = 0, sbb = 0;
    for (int k = 0; k < SPEC_N; k++) {
      float a = curSpec[k] - ma, b = refSpec[k] - mb;
      sab += a * b; saa += a * a; sbb += b * b;
    }
    if (saa > 1e-6f && sbb > 1e-6f) sim = fmaxf(0.0f, sab / sqrtf(saa * sbb));
  }

  float alpha = constrain((float)min(el, 500UL) / SCORE_TAU_MS, 0.0f, 1.0f);
  musicScore += (sim - musicScore) * alpha;

  memcpy(refSpec, curSpec, sizeof(curSpec));
  refLoud = loud;
  refTime = now;
}

void resetMusicScore() {
  musicScore = 0.0f;
  musicAboveMs = 0.0f;
  refLoud = false;
}

// ---------- PCA9685 health ----------
void initI2CAndPca() {
  Wire.begin(21, 22);
  Wire.setClock(100000);     // 100 kHz is far more tolerant of servo noise
  Wire.setTimeOut(20);       // never block forever on a stuck bus
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50);
  delay(10);
  expectedPrescale = pwm.readPrescale();
}

// A PCA9685 that browned out wakes up asleep with all outputs off and the
// default prescale, so the snake just "stops". Detect and repair that.
void checkPca() {
  static unsigned long lastCheck = 0;
  if (millis() - lastCheck < 2000) return;
  lastCheck = millis();

  static int failures = 0;

  Wire.beginTransmission(PCA_ADDR);
  bool acked = (Wire.endTransmission() == 0);
  bool configured = acked && (pwm.readPrescale() == expectedPrescale);

  failures = configured ? 0 : failures + 1;

  if (failures >= 2) {                      // two checks in a row -> real problem
    Serial.println("!! PCA9685 lost or reset -> re-initialising");
    failures = 0;
    pwm.begin();
    pwm.setOscillatorFrequency(27000000);
    pwm.setPWMFreq(50);
    delay(10);
    if (servosPowered) attachAllServos();   // restore the pose we were holding
  }
}

// ---------- Moves ----------
// Smooth eased move, all joints together. Returns false if interrupted.
bool moveTo(const int target[], int steps, int stepMs) {
  motionAbort = false;

  float start[NUM_SERVOS];
  for (int i = 0; i < NUM_SERVOS; i++) start[i] = smoothedServoAngles[i];

  for (int s = 1; s <= steps; s++) {
    float t = (float)s / steps;
    t = t * t * (3.0f - 2.0f * t);

    for (int i = 0; i < NUM_SERVOS; i++) {
      updateServo(i, start[i] + (target[i] - start[i]) * t);
    }

    smartDelay(stepMs);
    if (motionAbort) return false;
  }
  return true;
}

// Eased move where joints start one after another (a ripple through the body).
bool moveStaggered(const int target[], int steps, int stepMs, bool baseFirst) {
  motionAbort = false;

  float start[NUM_SERVOS];
  for (int i = 0; i < NUM_SERVOS; i++) start[i] = smoothedServoAngles[i];

  const float window = 1.0f - STAGGER;

  for (int s = 1; s <= steps; s++) {
    float T = (float)s / steps;

    for (int i = 0; i < NUM_SERVOS; i++) {
      int order = baseFirst ? i : (NUM_SERVOS - 1 - i);
      float t0 = STAGGER * order / (float)(NUM_SERVOS - 1);
      float t = (window > 0.0f) ? constrain((T - t0) / window, 0.0f, 1.0f) : 1.0f;
      t = t * t * (3.0f - 2.0f * t);
      updateServo(i, start[i] + (target[i] - start[i]) * t);
    }

    smartDelay(stepMs);
    if (motionAbort) return false;
  }
  return true;
}

bool goToSleep() {
  currentState = STATE_LOWERING;
  attachAllServos();
  if (!moveStaggered(restingAngles, 110, 22, false)) return false;   // head first
  smartDelay(300);
  detachAllServos();

  resetMusicScore();
  clapPending = false;
  recentLevel = ambientLevel;
  wakeBlockedUntil = millis() + WAKE_COOLDOWN_MS;
  lastAudioMs = millis();
  currentState = STATE_SLEEPING;
  return true;
}

// ======================================================
// AUDIO
// ======================================================
void getAudioData(AudioFrame &f) {
  int signalMax = 0;
  int signalMin = 4095;

  unsigned long t = micros();
  for (int i = 0; i < SAMPLES; i++) {
    int sample = analogRead(MIC_PIN);
    vReal[i] = sample;
    vImag[i] = 0.0f;
    if (sample > signalMax) signalMax = sample;
    if (sample < signalMin) signalMin = sample;

    t += SAMPLING_PERIOD_US;
    while ((long)(micros() - t) < 0) { }
  }

  f.level = signalMax - signalMin;

  FFT.dcRemoval();
  FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
  FFT.compute(FFTDirection::Forward);
  FFT.complexToMagnitude();

  const float binHz = SAMPLING_FREQ / SAMPLES;
  const int MIN_BIN = (int)ceilf(80.0f / binHz);
  const int MAX_BIN = (int)floorf(1600.0f / binHz);
  const int HF_BIN  = (int)ceilf(2000.0f / binHz);

  int bestBin = MIN_BIN;
  float bestMag = 0.0f;
  float total = 0.0f;
  float hf = 0.0f;
  float bandSum = 0.0f;
  int bandCount = 0;

  for (int k = MIN_BIN; k < SAMPLES / 2; k++) {
    float m = vReal[k];
    total += m;
    if (k >= HF_BIN) hf += m;
    if (k <= MAX_BIN) {
      bandSum += m;
      bandCount++;
      if (m > bestMag) {
        bestMag = m;
        bestBin = k;
      }
    }
  }

  float peak = bestBin * binHz;
  if (bestBin > MIN_BIN && bestBin < MAX_BIN) {
    float l = vReal[bestBin - 1];
    float c = vReal[bestBin];
    float r = vReal[bestBin + 1];
    float d = l - 2.0f * c + r;
    if (fabsf(d) > 1e-6f) peak = (bestBin + 0.5f * (l - r) / d) * binHz;
  }

  float bandMean = (bandCount > 0) ? bandSum / bandCount : 0.0f;

  // Spectrum for the music score: average of this + previous frame, compressed
  for (int k = 0; k < SPEC_N; k++) {
    float m = vReal[SPEC_LO + k];
    curSpec[k] = sqrtf(0.5f * (m + prevMag[k]));
    prevMag[k] = m;
  }

  // Frequency bands for the dance
  float b = 0, md = 0, h = 0;
  for (int k = 2;  k <= 6;  k++) b  += vReal[k];
  for (int k = 7;  k <= 25; k++) md += vReal[k];
  for (int k = 26; k <= 76; k++) h  += vReal[k];
  f.bass = b; f.mid = md; f.high = h;

  f.pitch    = peak;
  f.hfRatio  = (total > 1e-6f) ? (hf / total) : 0.0f;
  f.tonality = (bandMean > 1e-6f) ? (bestMag / bandMean) : 0.0f;
}

float measureAverageLevel(int frames) {
  long sum = 0;
  for (int k = 0; k < frames; k++) {
    AudioFrame f;
    getAudioData(f);
    sum += f.level;
    server.handleClient();
  }
  return sum / (float)frames;
}

// ======================================================
// MODE HANDLING
// ======================================================
Mode requestedMode() {
  if (forceSleep) return MODE_SLEEP;
  if (manualMode) return MODE_MANUAL;
  return MODE_AUDIO;
}

bool enterMode(Mode m) {
  learnMode = 0;
  switch (m) {
    case MODE_SLEEP:
      if (currentState == STATE_SLEEPING) return true;
      return goToSleep();

    case MODE_MANUAL:
      attachAllServos();
      currentState = STATE_MANUAL;
      return true;

    case MODE_AUDIO:
      if (currentState == STATE_SLEEPING) {
        resetMusicScore();
        lastAudioMs = millis();
        return true;
      }
      return goToSleep();
  }
  return true;
}

// ======================================================
// DANCING  (traveling serpentine wave through every joint)
// ======================================================
// Moves the body one step of the traveling wave.
void danceMotion(const float targetAmp[NUM_SERVOS], float pitchHz, float pulse) {
  unsigned long now = millis();
  float dt = constrain((now - lastDanceUpdateMs) / 1000.0f, 0.0f, 0.15f);
  lastDanceUpdateMs = now;

  float safePitch = constrain(pitchHz, 80.0f, 1600.0f);
  float pitchNorm = constrain(logf(safePitch / 80.0f) / logf(20.0f), 0.0f, 1.0f);
  float targetFreq = DANCE_MIN_FREQ + pitchNorm * (DANCE_MAX_FREQ - DANCE_MIN_FREQ);
  danceFreq += (targetFreq - danceFreq) * constrain(dt * 4.0f, 0.0f, 1.0f);

  phase     = fmodf(phase     + TWO_PI * danceFreq * dt,         TWO_PI);
  slowPhase = fmodf(slowPhase + TWO_PI * danceFreq * 0.37f * dt, TWO_PI);

  float k = constrain(dt * 4.0f, 0.0f, 1.0f);
  for (int i = 0; i < NUM_SERVOS; i++) jointAmp[i] += (targetAmp[i] - jointAmp[i]) * k;

  // Wave travels base -> head. Side and front-back joints are 90 deg apart,
  // so the body sways and rolls like a cobra. Head moves more than the base.
  for (int i = 0; i < NUM_SERVOS; i++) {
    float theta = phase - i * WAVE_PHASE_PER_JOINT;
    float slow  = slowPhase - i * 0.45f;

    float wave = (jointAxis[i] == 0)
                   ? 0.8f * sinf(theta) + 0.2f * sinf(slow)
                   : 0.8f * cosf(theta) + 0.2f * cosf(slow);

    float taper  = 0.75f + 0.25f * (float)i / (NUM_SERVOS - 1);
    float bend   = wave * jointAmp[i] * taper;
    // beat accent: upper body snaps further in the direction it is moving
    float accentW = (i >= 4) ? 1.0f : (i >= 2 ? 0.4f : 0.0f);
    bend += pulse * BEAT_ACCENT_DEG * accentW * (wave >= 0 ? 1.0f : -1.0f);
    float offset = constrain(bend * jointDir[i], -MAX_BEND, MAX_BEND);
    float target = centerAngles[i] + offset;

    float cur  = smoothedServoAngles[i];
    float step = constrain(target - cur, -MAX_SLEW_DEG, MAX_SLEW_DEG);
    updateServo(i, cur + step);
  }
}

void resetDanceAnalysis(const AudioFrame &f) {
  bassT.reset(f.bass); midT.reset(f.mid); highT.reset(f.high);
  bassAvg = f.bass;
  beatPulse = 0.0f;
  for (int i = 0; i < NUM_SERVOS; i++) jointAmp[i] = 0.0f;   // wave eases in
}

// The dance follows the FFT:
//   dominant frequency (pitch) -> speed of the wave
//   BASS energy  -> how far the base of the body (servos 1-2) sways
//   MID energy   -> middle of the body (servos 3-5)
//   HIGH energy  -> head and neck (servos 6-7)
//   bass BEATS   -> quick head "strike" accent on every beat
void danceStep(const AudioFrame &f) {
  unsigned long now = millis();

  if (f.pitch >= 80.0f && f.pitch <= 1600.0f) {
    smoothPitch = smoothPitch * 0.92f + f.pitch * 0.08f;
  }

  float nb = bassT.update(f.bass);
  float nm = midT.update(f.mid);
  float nh = highT.update(f.high);

  // Beat = sudden rise of bass energy above its recent average
  bool beat = (f.bass > bassAvg * 1.45f) && (nb > 0.35f) && (now - lastBeatMs > 280);
  bassAvg += (f.bass - bassAvg) * 0.12f;
  if (beat) { lastBeatMs = now; beatPulse = 1.0f; }
  beatPulse *= 0.80f;                         // ~150 ms decay

  float overall = (nb + nm + nh) / 3.0f;
  float targetAmp[NUM_SERVOS];
  for (int i = 0; i < NUM_SERVOS; i++) {
    float band = (i <= 1) ? nb : (i <= 4 ? nm : nh);
    float drive = 0.4f * overall + 0.6f * band;
    targetAmp[i] = DANCE_MIN_AMP + drive * (DANCE_MAX_AMP - DANCE_MIN_AMP);
  }

  danceMotion(targetAmp, smoothPitch, beatPulse);

  static unsigned long lastDbg = 0;
  if (now - lastDbg > 300) {
    lastDbg = now;
    Serial.printf("   FFT  bass %.2f  mid %.2f  high %.2f  pitch %.0f Hz%s\n",
                  nb, nm, nh, smoothPitch, beat ? "  BEAT" : "");
  }
}

// ======================================================
// LEARNING  (web buttons "Learn MUSIC" / "Learn TALKING")
// ======================================================
void computeThresholds() {
  if (learnedMusic > 0 && learnedTalk >= 0) {
    float gap = learnedMusic - learnedTalk;
    if (gap < 0.04f) {
      snprintf(learnMsg, sizeof(learnMsg),
               "Music %.2f and talking %.2f are too similar - play music louder / closer, then learn again",
               learnedMusic, learnedTalk);
      return;
    }
    musicScoreWake = learnedTalk + 0.50f * gap;
    musicScoreStay = learnedTalk + 0.30f * gap;
  } else if (learnedMusic > 0) {
    musicScoreWake = learnedMusic * 0.80f;
    musicScoreStay = learnedMusic * 0.62f;
  } else if (learnedTalk >= 0) {
    musicScoreWake = learnedTalk + 0.12f;
    musicScoreStay = learnedTalk + 0.06f;
  } else {
    return;
  }
  snprintf(learnMsg, sizeof(learnMsg), "Music %.2f | talking %.2f -> rise above %.2f, sleep below %.2f",
           learnedMusic, learnedTalk, musicScoreWake, musicScoreStay);
}

void saveLearned() {
  prefs.begin("snake", false);
  prefs.putFloat("lmus", learnedMusic);
  prefs.putFloat("ltalk", learnedTalk);
  prefs.end();
}

void loadLearned() {
  prefs.begin("snake", true);
  learnedMusic = prefs.getFloat("lmus", -1.0f);
  learnedTalk  = prefs.getFloat("ltalk", -1.0f);
  prefs.end();
  computeThresholds();
}

// Called every audio frame while learning; the snake stays asleep.
void learnUpdate(unsigned long now) {
  unsigned long el = now - learnStart;
  if (el > LEARN_WARMUP_MS) { learnSum += musicScore; learnN++; }
  if (el < LEARN_TOTAL_MS) return;

  float avg = (learnN > 0) ? (float)(learnSum / learnN) : 0.0f;
  if (learnMode == 1) learnedMusic = fmaxf(avg, 0.01f);
  else                learnedTalk  = avg;
  Serial.printf("Learned %s score: %.2f\n", learnMode == 1 ? "MUSIC" : "TALKING", avg);
  learnMode = 0;
  computeThresholds();
  saveLearned();
}

// ======================================================
// MICROPHONE STATE MACHINE
// ======================================================
void audioUpdate() {
  AudioFrame f;
  getAudioData(f);

  unsigned long now = millis();
  float dtMs = constrain((float)(now - lastAudioMs), 0.0f, 200.0f);
  lastAudioMs = now;

  // ---- Clap: bright spike above the RECENT level that dies quickly ----
  float baseline = fmaxf(recentLevel, ambientLevel);
  bool spike = (f.level - baseline > CLAP_SPIKE_THRESHOLD) && (f.hfRatio > CLAP_HF_RATIO);
  bool clap = false;

  if (clapPending) {
    if (f.level < clapCandBaseline + CLAP_SPIKE_THRESHOLD * 0.5f) {
      clap = true;                         // short transient -> real clap
      clapPending = false;
    } else if (now - clapCandTime > CLAP_MAX_LEN_MS) {
      clapPending = false;                 // too long -> drum roll / shout
    }
  } else if (spike && (now - lastClapTime > CLAP_DEBOUNCE_MS)) {
    if (CLAP_REQUIRE_SHORT) {
      clapPending = true;
      clapCandTime = now;
      clapCandBaseline = baseline;
    } else {
      clap = true;
    }
  }

  if (!spike && !clapPending) recentLevel = recentLevel * 0.85f + f.level * 0.15f;

  // ---- Music score (claps are skipped so they don't disturb it) ----
  bool loud = f.level > loudThreshold();
  if (!spike && !clapPending) updateMusicScore(loud, now);

  bool musicPresent = musicScore >= musicScoreStay;

  static unsigned long lastPrint = 0;
  if (now - lastPrint > 300 || clap) {
    lastPrint = now;
    Serial.printf("%s | lvl %d (loud>%d) | score %.2f (wake>%.2f stay>%.2f) | %s | pitch %.0f | hf %.2f%s\n",
                  stateName(), f.level, (int)loudThreshold(),
                  musicScore, musicScoreWake, musicScoreStay,
                  musicScore >= musicScoreWake ? "MUSIC" : (musicScore >= musicScoreStay ? "music?" : "no music"),
                  f.pitch, f.hfRatio, clap ? "  <<< CLAP" : "");
  }

  switch (currentState) {

    case STATE_SLEEPING: {
      if (now < wakeBlockedUntil) {      // settling after lying down
        resetMusicScore();
        break;
      }

      if (learnMode != 0) {               // learning: listen only, don't rise
        learnUpdate(now);
        musicAboveMs = 0.0f;
        break;
      }

      if (musicScore >= musicScoreWake) musicAboveMs += dtMs;
      else                                musicAboveMs = 0.0f;

      if (!loud) {
        ambientLevel = ambientLevel * 0.997f + f.level * 0.003f;
        ambientLevel = constrain(ambientLevel, quietBase * 0.7f, quietBase * 1.6f + 10.0f);
      }

      if (musicAboveMs >= MUSIC_CONFIRM_MS) {
        Serial.println("Music detected -> rising");
        currentState = STATE_RISING;
      }
      break;
    }

    case STATE_RISING: {
      attachAllServos();
      if (!moveStaggered(centerAngles, 100, 20, true)) break;   // base first
      smartDelay(100);

      phase = 0.0f;
      slowPhase = 0.0f;
      resetDanceAnalysis(f);
      danceFreq = DANCE_MIN_FREQ;
      lastDanceUpdateMs = millis();
      lastMusicTime = millis();
      lastClapTime = millis();          // ignore noise right after rising
      lastAudioMs = millis();
      clapPending = false;
      recentLevel = f.level;
      if (f.pitch >= 80.0f && f.pitch <= 1600.0f) smoothPitch = f.pitch;

      currentState = STATE_DANCING;
      break;
    }

    case STATE_DANCING: {
      if (clap) {                        // FIRST CLAP -> freeze upright
        Serial.println("Clap -> freezing upright");
        currentState = STATE_FROZEN;
        moveTo(centerAngles, 30, 15);
        lastClapTime = millis();
        lastMusicTime = millis();
        lastAudioMs = millis();
        clapPending = false;
        break;
      }

      if (musicPresent) lastMusicTime = now;

      if (now - lastMusicTime > MUSIC_OFF_TIMEOUT_MS) {
        Serial.println("Music stopped -> lowering");
        currentState = STATE_LOWERING;
        break;
      }

      danceStep(f);
      break;
    }

    case STATE_FROZEN: {
      if (musicPresent) lastMusicTime = now;

      if (clap) {                        // SECOND CLAP -> dance again
        Serial.println("Clap -> resuming dance");
        lastClapTime = now;
        lastMusicTime = now;
        lastDanceUpdateMs = now;
        resetDanceAnalysis(f);
        currentState = STATE_DANCING;
        break;
      }

      if (now - lastMusicTime > MUSIC_OFF_TIMEOUT_MS) {
        Serial.println("Music stopped while frozen -> lowering");
        currentState = STATE_LOWERING;
      }
      break;
    }

    case STATE_LOWERING:
      goToSleep();
      break;

    default:
      break;
  }
}

// ======================================================
// WEB INTERFACE
// ======================================================
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Snake Robot Controller</title>
<style>
:root{
  --bg-color:#0f172a;--card-bg:#1e293b;--card-border:#334155;
  --text-main:#f8fafc;--text-muted:#94a3b8;
  --accent-mic:#10b981;--accent-manual:#f97316;
  --accent-sleep-off:#6366f1;--accent-sleep-on:#eab308;
}
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:-apple-system,BlinkMacSystemFont,"Segoe UI",Roboto,sans-serif;
  background:var(--bg-color);color:var(--text-main);padding:20px;display:flex;justify-content:center}
.container{width:100%;max-width:720px}
.header{text-align:center;margin-bottom:24px}
.header h1{font-size:28px;font-weight:700;margin-bottom:6px}
.header p{color:var(--text-muted);font-size:14px}
.card{background:var(--card-bg);border:1px solid var(--card-border);border-radius:16px;
  padding:20px;margin-bottom:20px;box-shadow:0 10px 25px -5px rgba(0,0,0,.3)}
.controls-grid{display:grid;grid-template-columns:1fr 1fr;gap:14px}
button{border:none;border-radius:12px;padding:16px;font-size:15px;font-weight:700;cursor:pointer;
  transition:all .2s ease;display:flex;align-items:center;justify-content:center;gap:8px}
button:hover{transform:translateY(-2px);filter:brightness(1.1)}
.btn-mic{background:var(--accent-mic);color:#022c22}
.btn-manual{background:var(--accent-manual);color:#fff}
.btn-sleep-off{background:var(--accent-sleep-off);color:#fff}
.btn-sleep-on{background:var(--accent-sleep-on);color:#422006}
.status-row{display:flex;justify-content:space-between;align-items:center;padding:12px 16px;
  background:#0f172a80;border-radius:10px;margin-bottom:8px}
.status-row:last-child{margin-bottom:0}
.status-label{color:var(--text-muted);font-size:14px}
.status-badge{font-weight:700;font-size:14px}
.section-title{font-size:16px;font-weight:700;margin-bottom:16px;color:var(--text-muted);text-transform:uppercase}
.servo-item{background:#0f172a60;border:1px solid var(--card-border);border-radius:12px;padding:14px;margin-bottom:12px}
.servo-header{display:flex;justify-content:space-between;align-items:center;margin-bottom:10px}
.servo-name{font-weight:600;font-size:15px}
.servo-val{font-weight:700;color:#38bdf8;font-size:15px}
input[type=range]{width:100%;height:8px;-webkit-appearance:none;appearance:none;background:#334155;border-radius:8px;outline:none}
input[type=range]::-webkit-slider-thumb{-webkit-appearance:none;width:22px;height:22px;border-radius:50%;background:#38bdf8;cursor:pointer}
input[type=range]:disabled{opacity:.35}
.meter{position:relative;height:16px;background:#0f172a;border-radius:8px;margin:10px 0 14px;overflow:hidden}
.meter-fill{height:100%;width:0;background:#38bdf8;transition:width .3s}
.mark{position:absolute;top:0;bottom:0;width:3px}
.mark.wake{background:#10b981}.mark.stay{background:#eab308}
.hint{color:var(--text-muted);font-size:13px;margin:8px 0 12px;line-height:1.5}
</style>
</head>
<body>
<div class="container">

<div class="header">
  <h1>🐍 Snake Robot Dashboard</h1>
  <p>ESP32 Control Portal</p>
</div>

<div class="card">
  <div class="controls-grid">
    <button id="btnManual" class="btn-mic" onclick="cmd('/toggleManual')">🎤 Microphone Active</button>
    <button id="btnSleep" class="btn-sleep-off" onclick="cmd('/toggleSleep')">🌙 Force Sleep: OFF</button>
  </div>
</div>

<div class="card">
  <div class="section-title">System Monitor</div>
  <div class="status-row">
    <span class="status-label">Control Source</span>
    <span id="controlStatus" class="status-badge" style="color:#10b981;">Microphone</span>
  </div>
  <div class="status-row">
    <span class="status-label">Robot State</span>
    <span id="robotStatus" class="status-badge" style="color:#38bdf8;">Sleeping</span>
  </div>
</div>

<div class="card">
  <div class="section-title">Music Detector</div>
  <div class="status-row">
    <span class="status-label">Music score</span>
    <span id="scoreTxt" class="status-badge">0.00</span>
  </div>
  <div class="meter"><div id="scoreBar" class="meter-fill"></div>
    <div id="stayMark" class="mark stay"></div><div id="wakeMark" class="mark wake"></div></div>
  <p class="hint">Green line = rises above it. Yellow line = sleeps below it.<br>
  Teach it in your room while the snake is <b>sleeping</b> (10 s each):</p>
  <div class="controls-grid">
    <button class="btn-mic" onclick="cmd('/learnMusic')">🎵 Learn MUSIC</button>
    <button class="btn-sleep-off" onclick="cmd('/learnTalk')">🗣️ Learn TALKING</button>
  </div>
  <p id="learnMsg" class="hint"></p>
</div>

<div class="card">
  <div class="section-title">Servo Positions (1 – 7)</div>
  <div id="sliders"></div>
</div>

</div>

<script>
let isDragging = false;

function initSliders() {
  let html = "";
  for (let i = 0; i < 7; i++) {
    html += `
      <div class="servo-item">
        <div class="servo-header">
          <span class="servo-name">Servo ${i + 1}</span>
          <span class="servo-val" id="v${i}">90°</span>
        </div>
        <input type="range" min="0" max="180" value="90" id="s${i}"
          oninput="updateVal(${i})" onchange="sendVal(${i})" disabled>
      </div>`;
  }
  document.getElementById("sliders").innerHTML = html;
}

function updateVal(i) {
  isDragging = true;
  document.getElementById("v" + i).innerHTML = document.getElementById("s" + i).value + "°";
}

function sendVal(i) {
  fetch(`/setAngle?id=${i}&val=${document.getElementById("s" + i).value}`);
  setTimeout(() => { isDragging = false; }, 250);
}

function cmd(path) {
  fetch(path).then(r => r.json()).then(updateUI).catch(e => console.log(e));
}

function updateUI(data) {
  const btnM = document.getElementById("btnManual");
  const cs = document.getElementById("controlStatus");

  if (data.manual) {
    btnM.className = "btn-manual";
    btnM.innerHTML = "🎛️ Manual Control Mode";
  } else {
    btnM.className = "btn-mic";
    btnM.innerHTML = "🎤 Microphone Active";
  }

  if (data.sleep)       { cs.innerHTML = "Force Sleep";     cs.style.color = "#eab308"; }
  else if (data.manual) { cs.innerHTML = "Manual Controls"; cs.style.color = "#f97316"; }
  else                  { cs.innerHTML = "Microphone FFT";  cs.style.color = "#10b981"; }

  const btnS = document.getElementById("btnSleep");
  btnS.className = data.sleep ? "btn-sleep-on" : "btn-sleep-off";
  btnS.innerHTML = data.sleep ? "☀️ Force Sleep: ACTIVE" : "🌙 Force Sleep: OFF";

  document.getElementById("robotStatus").innerHTML = data.state;

  const pct = v => Math.min(100, Math.max(0, v * 100)) + "%";
  document.getElementById("scoreBar").style.width = pct(data.score);
  document.getElementById("scoreBar").style.background = data.score >= data.wake ? "#10b981" : "#38bdf8";
  document.getElementById("wakeMark").style.left = pct(data.wake);
  document.getElementById("stayMark").style.left = pct(data.stay);
  document.getElementById("scoreTxt").innerHTML = data.score.toFixed(2);
  document.getElementById("learnMsg").innerHTML = data.learning
    ? "Listening... " + data.learning + " s left" : data.msg;

  for (let i = 0; i < 7; i++) {
    const slider = document.getElementById("s" + i);
    slider.disabled = !data.manual || data.sleep;
    if (!isDragging) {
      slider.value = data.angles[i];
      document.getElementById("v" + i).innerHTML = data.angles[i] + "°";
    }
  }
}

function fetchStatus() {
  if (!isDragging) {
    fetch("/status").then(r => r.json()).then(updateUI).catch(e => console.log(e));
  }
}

initSliders();
fetchStatus();
setInterval(fetchStatus, 500);
</script>
</body>
</html>
)rawliteral";

const char* stateName() {
  switch (currentState) {
    case STATE_SLEEPING:  return "Sleeping";
    case STATE_RISING:    return "Rising";
    case STATE_DANCING:   return "Dancing";
    case STATE_FROZEN:    return "Frozen (Clapped)";
    case STATE_LOWERING:  return "Lowering";
    case STATE_MANUAL:    return "Manual";
  }
  return "?";
}

// Fixed buffer instead of String concatenation (no heap fragmentation
// from the page polling every 500 ms for hours).
void sendJsonResponse() {
  char json[420];
  int learnLeft = 0;
  if (learnMode != 0) {
    long left = (long)LEARN_TOTAL_MS - (long)(millis() - learnStart);
    learnLeft = left > 0 ? (int)((left + 999) / 1000) : 0;
  }
  snprintf(json, sizeof(json),
           "{\"manual\":%s,\"sleep\":%s,\"state\":\"%s\",\"angles\":[%d,%d,%d,%d,%d,%d,%d],"
           "\"score\":%.3f,\"wake\":%.3f,\"stay\":%.3f,\"learning\":%d,\"msg\":\"%s\"}",
           manualMode ? "true" : "false",
           forceSleep ? "true" : "false",
           stateName(),
           currentAngles[0], currentAngles[1], currentAngles[2], currentAngles[3],
           currentAngles[4], currentAngles[5], currentAngles[6],
           musicScore, musicScoreWake, musicScoreStay, learnLeft, learnMsg);
  server.send(200, "application/json", json);
}

void startLearning(int mode) {
  if (activeMode != MODE_AUDIO || currentState != STATE_SLEEPING) {
    snprintf(learnMsg, sizeof(learnMsg), "Wait until the snake is sleeping (microphone mode), then try again");
    return;
  }
  learnMode = mode;
  learnStart = millis();
  learnSum = 0;
  learnN = 0;
  wakeBlockedUntil = 0;
}

void setupWebServer() {
  server.on("/", HTTP_GET, []() {
    server.send_P(200, "text/html", index_html);
  });

  server.on("/status", HTTP_GET, []() {
    sendJsonResponse();
  });

  server.on("/toggleManual", HTTP_GET, []() {
    manualMode = !manualMode;
    motionAbort = true;
    sendJsonResponse();
  });

  server.on("/toggleSleep", HTTP_GET, []() {
    forceSleep = !forceSleep;
    motionAbort = true;
    sendJsonResponse();
  });

  server.on("/learnMusic", HTTP_GET, []() {
    startLearning(1);
    sendJsonResponse();
  });

  server.on("/learnTalk", HTTP_GET, []() {
    startLearning(2);
    sendJsonResponse();
  });

  server.on("/setAngle", HTTP_GET, []() {
    if (server.hasArg("id") && server.hasArg("val") &&
        activeMode == MODE_MANUAL && manualMode && !forceSleep) {
      int id = server.arg("id").toInt();
      int val = server.arg("val").toInt();
      if (id >= 0 && id < NUM_SERVOS) updateServo(id, val);
    }
    server.send(200, "text/plain", "OK");
  });

  server.begin();
}

// ======================================================
// BOOT DIAGNOSTICS
// ======================================================
const char* resetReasonName(esp_reset_reason_t r) {
  switch (r) {
    case ESP_RST_POWERON:  return "power on";
    case ESP_RST_SW:       return "software restart";
    case ESP_RST_PANIC:    return "CRASH (panic)";
    case ESP_RST_INT_WDT:  return "CRASH (interrupt watchdog)";
    case ESP_RST_TASK_WDT: return "CRASH (task watchdog)";
    case ESP_RST_WDT:      return "CRASH (watchdog)";
    case ESP_RST_BROWNOUT: return "BROWN-OUT (power supply dipped - servos?)";
    case ESP_RST_EXT:      return "reset pin";
    default:               return "other";
  }
}

// "Stretch": rise, dance silently and measure how loud the snake's OWN
// servos are while dancing. Returns mean + 1 std of the mic level.
float measureDanceNoise() {
  Serial.println("Stretch: measuring my own dance noise - keep quiet...");
  attachAllServos();
  moveStaggered(centerAngles, 100, 20, true);

  phase = 0.0f; slowPhase = 0.0f;
  for (int i = 0; i < NUM_SERVOS; i++) jointAmp[i] = 0.0f;
  float fullAmp[NUM_SERVOS];
  for (int i = 0; i < NUM_SERVOS; i++) fullAmp[i] = DANCE_MAX_AMP;
  danceFreq = DANCE_MIN_FREQ;
  lastDanceUpdateMs = millis();

  double sum = 0, sum2 = 0;
  int n = 0;
  unsigned long start = millis();
  while (millis() - start < STRETCH_MS) {
    AudioFrame f;
    getAudioData(f);
    if (millis() - start > 1500) {           // after the wave has built up
      sum += f.level; sum2 += (double)f.level * f.level; n++;
    }
    danceMotion(fullAmp, 500.0f, 0.0f);       // full-size, fairly fast = loudest case
    server.handleClient();
  }

  moveStaggered(restingAngles, 110, 22, false);
  smartDelay(300);
  detachAllServos();

  if (n == 0) return ambientLevel;
  float mean = sum / n;
  float sd = sqrt(fmax(0.0, sum2 / n - (double)mean * mean));
  return mean + sd;
}

// Measure room noise. If the ESP32 rebooted while music was playing
// (e.g. brown-out), the measurement would be far too high and the snake
// would never wake again - so fall back to the last good value in flash.
void calibrateAudio() {
  prefs.begin("snake", false);
  float storedAmb = prefs.getFloat("amb", -1.0f);
  float storedSrv = prefs.getFloat("srv", 0.0f);
  float storedDnl = prefs.getFloat("dnl", -1.0f);

  float measuredOff = measureAverageLevel(20);

  attachAllServos();                         // servos holding rest pose
  smartDelay(300);
  float measuredOn = measureAverageLevel(15);
  detachAllServos();

  bool noisyBoot = (storedAmb > 0.0f && measuredOff > storedAmb * 1.5f + 10.0f);

  if (noisyBoot) {
    // Music/talking during boot: don't trust the measurements, use saved ones
    ambientLevel = storedAmb;
    servoNoiseExtra = storedSrv;
    danceNoiseLevel = (storedDnl > 0.0f) ? storedDnl : ambientLevel + servoNoiseExtra + 60.0f;
    Serial.printf("Boot was noisy (%.0f) - using saved values\n", measuredOff);
  } else {
    ambientLevel = measuredOff;
    servoNoiseExtra = fmaxf(0.0f, measuredOn - measuredOff);
    prefs.putFloat("amb", ambientLevel);
    prefs.putFloat("srv", servoNoiseExtra);

    if (CALIBRATE_DANCE_AT_BOOT) {
      danceNoiseLevel = measureDanceNoise();
      prefs.putFloat("dnl", danceNoiseLevel);
    } else {
      danceNoiseLevel = (storedDnl > 0.0f) ? storedDnl : ambientLevel + servoNoiseExtra + 60.0f;
    }
  }
  prefs.end();

  quietBase = ambientLevel;
  recentLevel = ambientLevel;
}

// ======================================================
// SETUP
// ======================================================
void setup() {
  Serial.begin(115200);
  delay(500);

  esp_reset_reason_t rr = esp_reset_reason();
  Serial.println();
  Serial.printf("Reset reason: %s\n", resetReasonName(rr));

  pinMode(MIC_PIN, INPUT);
  analogReadResolution(12);
  analogSetPinAttenuation(MIC_PIN, ADC_11db);

  initI2CAndPca();

  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);
  WiFi.setSleep(false);

  setupWebServer();

  for (int i = 0; i < NUM_SERVOS; i++) {
    smoothedServoAngles[i] = restingAngles[i];
    currentAngles[i] = restingAngles[i];
  }

  attachAllServos();
  smartDelay(800);
  detachAllServos();
  smartDelay(300);

  calibrateAudio();   // keep the room quiet for ~10 s after power-on (includes the stretch)
  loadLearned();      // thresholds learned earlier from your music / talking

  currentState = STATE_SLEEPING;
  activeMode = MODE_AUDIO;
  lastAudioMs = millis();
  wakeBlockedUntil = millis() + 500;

  Serial.println("=================================");
  Serial.println("Snake is ready and sleeping.");
  Serial.printf("Room noise %.0f | holding noise +%.0f | own dance noise %.0f\n",
                ambientLevel, servoNoiseExtra, danceNoiseLevel);
  Serial.printf("Music score: rise above %.2f, sleep below %.2f  (%s)\n",
                musicScoreWake, musicScoreStay, learnMsg);
  Serial.print("Web UI: http://");
  Serial.println(WiFi.softAPIP());
  Serial.println("=================================");
}

// ======================================================
// MAIN LOOP
// ======================================================
void loop() {
  server.handleClient();
  checkPca();

  Mode want = requestedMode();
  if (want != activeMode) {
    if (enterMode(want)) activeMode = want;
    return;
  }

  switch (activeMode) {
    case MODE_SLEEP:
    case MODE_MANUAL:
      smartDelay(10);
      break;

    case MODE_AUDIO:
      audioUpdate();
      break;
  }
}