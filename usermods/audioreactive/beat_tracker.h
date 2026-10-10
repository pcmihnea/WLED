#pragma once
// =============================================================================
//  ARBeatTracker - real-time onset detection + tempo / beat-phase tracking
//  Part of the AudioReactive usermod. process() runs inside the FFT task, once per
//  FFT frame (512 samples @ 22.05 kHz = 23.2 ms hop, ~43 frames/s).
//
//  Why: the stock "samplePeak" is a level trigger (bin magnitude > threshold, rate
//  limited to 80-100 ms). With any music playing it fires ~10x per second, so it is
//  useless as a beat source for anything that moves (moving heads, step effects).
//
//  Pipeline
//   1. Onset detection function (ODF): half-wave-rectified spectral flux of the
//      log-compressed magnitude spectrum. Weighted toward the kick region
//      (43-300 Hz) with a full-band share so snare/hat-driven music still registers.
//      Log compression makes it largely independent of mic gain and AGC.
//   2. Onsets: adaptive threshold (local mean + k * running std-dev), rising-edge
//      trigger with a refractory time.
//   3. Tempo: autocorrelation of the ODF over ~9 s with a harmonic comb
//      (L, 2L, 3L, 4L) and a log-Gaussian prior around 120 BPM to suppress octave
//      errors; parabolic interpolation for sub-frame resolution; hysteresis so the
//      tempo estimate does not flicker between candidates.
//   4. Beat phase: a PLL predicts every beat from the tempo and is pulled toward
//      onsets that land near a predicted beat. Beats keep coming through breaks and
//      fills, and stay on the grid instead of following every hi-hat.
//
//  Published values are plain 32-bit fields (atomic on Xtensa); readers on the other
//  core may see a value one frame old, never a torn one.
// =============================================================================
#include <math.h>
#include <string.h>
#include <stdint.h>

class ARBeatTracker {
  public:
    // ---- results (written by the FFT task) ----
    uint32_t beatCount     = 0;     // increments on every beat (tempo-locked when confident, raw onsets otherwise)
    uint32_t onsetCount    = 0;     // increments on every detected onset (kick/snare/...)
    uint32_t lastBeatMs    = 0;     // millis() timestamp of the last beat (predicted beat time, not detection time)
    uint32_t lastOnsetMs   = 0;     // millis() timestamp of the last onset
    float    bpm           = 0.0f;  // tempo estimate, 0 = unknown
    float    confidence    = 0.0f;  // 0..1 - how periodic the music is / how much to trust bpm + phase
    float    onsetStrength = 0.0f;  // 0..1 - strength of the last onset
    float    bands[3]      = {0.0f, 0.0f, 0.0f}; // bass / mid / high energy, self-normalized 0..1

    // ---- tuning ----
    float sensitivity = 0.5f;       // 0..1, higher = more onsets
    float minBpm = 70.0f, maxBpm = 180.0f;

    void begin(float frameRate) { _fps = frameRate; _hopMs = 1000.0f / frameRate; reset(); }

    void reset() {
      memset(_prevLog, 0, sizeof(_prevLog));
      memset(_hist, 0, sizeof(_hist));
      memset(_odfRecent, 0, sizeof(_odfRecent));
      _histPos = 0; _histFill = 0; _frame = 0; _recentPos = 0;
      _mu = 0.0f; _var = 0.0f; _armed = true; _lastOnsetFrame = -1000;
      _locked = false; _nextBeat = 0.0f; _gateWarm = 3; _candBpm = 0.0f; _candHits = 0;
      _silentFrames = 0; _phaseErrAvg = 0.0f; _offGrid = 0; _offGridErr = 0.0f;
      for (int b = 0; b < 3; b++) { _env[b] = 0.0f; _envLong[b] = 0.0f; bands[b] = 0.0f; }
      bpm = 0.0f; confidence = 0.0f; onsetStrength = 0.0f;
    }

    // called from the main loop while audio processing is suspended - fades out the outputs
    void idle() {
      confidence = 0.0f; onsetStrength = 0.0f;
      bands[0] = bands[1] = bands[2] = 0.0f;
      _locked = false; _gateWarm = 3;
    }

    // mag: FFT magnitude bins (at least NBINS valid entries). magScale maps the magnitudes into a
    // range where log1p() compresses nicely (~1 at the noise floor).
    template<typename T>
    void process(const T* mag, float magScale, bool gateOpen, uint32_t nowMs) {
      _frame++;
      const float tf = (float)_frame;

      // ---------- 1. spectral flux + band energies ----------
      float lowFlux = 0.0f, fullFlux = 0.0f;
      float e[3] = {0.0f, 0.0f, 0.0f};
      for (int k = 1; k < NBINS; k++) {
        float m = gateOpen ? (float)mag[k] : 0.0f;
        if (m < 0.0f) m = 0.0f;
        float lg = log1pf(m * magScale);
        float d = lg - _prevLog[k];
        _prevLog[k] = lg;
        if (d > 0.0f) { fullFlux += d; if (k < LOW_HI) lowFlux += d; }
        if (k < BASS_HI) e[0] += m; else if (k < MID_HI) e[1] += m; else e[2] += m;
      }
      e[0] /= float(BASS_HI - 1); e[1] /= float(MID_HI - BASS_HI); e[2] /= float(NBINS - MID_HI);
      updateBands(e);

      // kick-weighted ODF; the full-band part is rescaled to "LOW_HI-1 bins worth" so both halves are comparable
      float odf = 0.65f * lowFlux + 0.35f * fullFlux * (float(LOW_HI - 1) / float(NBINS - 1));
      if (!gateOpen) { _gateWarm = 3; _silentFrames++; }
      else _silentFrames = 0;
      if (_gateWarm > 0) {           // gate just (re)opened: the jump from silence is not an onset
        if (gateOpen) _gateWarm--;
        odf = 0.0f;
      }

      // ---------- 2. adaptive threshold + onset ----------
      float localMean = 0.0f;
      for (int i = 0; i < RECENT; i++) localMean += _odfRecent[i];
      localMean /= float(RECENT);
      _odfRecent[_recentPos] = odf; _recentPos = (_recentPos + 1) % RECENT;

      const float a = 1.0f / 86.0f;  // ~2 s running statistics
      float dm = odf - _mu;
      _mu  += a * dm;
      _var += a * (dm * dm - _var);
      float sd = sqrtf(_var > 0.0f ? _var : 0.0f);

      float k = 2.6f - 1.8f * clamp01(sensitivity);
      float thr = fmaxf(localMean, _mu) + k * sd + MIN_ODF;
      float periodF = (bpm > 0.0f) ? (_fps * 60.0f / bpm) : 0.0f;
      int refractory = (int)fmaxf(5.0f, 0.35f * periodF);   // >= 116 ms, or ~1/3 beat

      bool onset = false;
      if (_armed && odf > thr && (int)(_frame - _lastOnsetFrame) >= refractory) {
        onset = true; _armed = false; _lastOnsetFrame = (int)_frame;
        onsetStrength = clamp01((odf - _mu) / (4.0f * sd + 0.25f));
        onsetCount++;
        lastOnsetMs = nowMs;
      } else if (odf <= thr) {
        _armed = true;
      }

      // ---------- 3. tempo (autocorrelation of the onset envelope) ----------
      float x = odf - localMean; if (x < 0.0f) x = 0.0f;
      _hist[_histPos] = x; _hist[_histPos + HIST] = x;     // mirrored -> contiguous window without modulo
      _histPos = (_histPos + 1) % HIST;
      if (_histFill < HIST) _histFill++;
      if ((_frame % TEMPO_EVERY) == 0 && _histFill >= MIN_FILL) estimateTempo();

      if (_silentFrames > SILENCE_FRAMES) confidence *= 0.93f;   // long silence: fade trust, keep the tempo for a quick re-lock

      // ---------- 4. beat phase (PLL) ----------
      periodF = (bpm > 0.0f) ? (_fps * 60.0f / bpm) : 0.0f;
      if (periodF > 1.0f && confidence >= LOCK_CONF) {
        if (!_locked) {
          if (onset) { _locked = true; _nextBeat = tf + periodF; emitBeat(tf, tf, nowMs); }
        } else {
          if (onset) {
            float eNext = tf - _nextBeat;               // < 0: onset before the predicted beat
            float ePrev = tf - (_nextBeat - periodF);   // > 0: onset after the last beat
            float err = (fabsf(ePrev) < fabsf(eNext)) ? ePrev : eNext;
            if (fabsf(err) < 0.25f * periodF) {
              _nextBeat += (0.15f + 0.25f * onsetStrength) * err;   // phase pull
              _phaseErrAvg += 0.1f * (err - _phaseErrAvg);           // slow frequency pull (2nd order loop)
              _offGrid = 0;                                          // the grid is confirmed -> off-beat hats etc. never re-sync
            } else if (onsetStrength > 0.4f) {
              // strong onsets consistently off the grid (drums back after a breakdown, DJ nudged the track,
              // tracker locked on the off-beat): after 3 in a row at a similar offset, jump the phase
              if (_offGrid > 0 && fabsf(err - _offGridErr) < 0.12f * periodF) {
                _offGridErr = 0.5f * (_offGridErr + err);
                if (++_offGrid >= 3) { _nextBeat += _offGridErr; _offGrid = 0; _phaseErrAvg = 0.0f; }
              } else { _offGrid = 1; _offGridErr = err; }
            }
          }
          if (tf >= _nextBeat - 0.5f) {               // predicted beat falls in this frame
            emitBeat(_nextBeat, tf, nowMs);
            _nextBeat += periodF + 0.05f * _phaseErrAvg;
            if (_nextBeat < tf + 0.5f) _nextBeat = tf + periodF;     // never queue a beat in the past
          }
        }
      } else {
        _locked = false;
        if (onset) emitBeat(tf, tf, nowMs);             // no usable tempo: every onset is a beat
      }
    }

  private:
    static constexpr int   NBINS = 200;        // bins 1..199 => up to ~8.6 kHz @ 22050/512
    static constexpr int   LOW_HI = 7;         // kick band for the ODF: bins 1..6 (43..300 Hz)
    static constexpr int   BASS_HI = 6;        // band energies: bass 1..5 (43..258 Hz)
    static constexpr int   MID_HI = 47;        //                mid 6..46 (258..2020 Hz), high 47..199
    static constexpr int   HIST = 384;         // ODF history for tempo (~8.9 s)
    static constexpr int   MIN_FILL = 128;     // need ~3 s of history before estimating tempo
    static constexpr int   TEMPO_EVERY = 8;    // re-estimate tempo every ~186 ms
    static constexpr int   RECENT = 8;         // local mean window (~186 ms)
    static constexpr int   MAX_LAG = 200;      // >= 4 * lag(minBpm) + 1 for the harmonic comb (=> minBpm >= ~52)
    static constexpr int   SILENCE_FRAMES = 43;     // ~1 s of closed noise gate
    static constexpr float MIN_ODF = 0.15f;    // absolute floor - ignores flux from noise
    static constexpr float LOCK_CONF = 0.30f;  // confidence needed to run the tempo-locked beat clock

    float _fps = 43.066f, _hopMs = 23.22f;
    float _prevLog[NBINS];
    float _hist[2 * HIST];
    float _odfRecent[RECENT];
    float _ac[MAX_LAG + 2];
    float _xc[HIST];                           // mean-removed copy of the window
    int _histPos = 0, _histFill = 0, _recentPos = 0;
    uint32_t _frame = 0;
    int _lastOnsetFrame = -1000;
    float _mu = 0.0f, _var = 0.0f;
    bool _armed = true, _locked = false;
    float _nextBeat = 0.0f, _phaseErrAvg = 0.0f, _offGridErr = 0.0f;
    int   _offGrid = 0;
    int _gateWarm = 3, _silentFrames = 0;
    float _candBpm = 0.0f; int _candHits = 0;
    float _env[3] = {0}, _envLong[3] = {0};

    static inline float clamp01(float v) { return v < 0.0f ? 0.0f : (v > 1.0f ? 1.0f : v); }

    void emitBeat(float beatFrame, float tf, uint32_t nowMs) {
      beatCount++;
      lastBeatMs = nowMs - (int32_t)lroundf((tf - beatFrame) * _hopMs);
    }

    void updateBands(const float e[3]) {
      const float decay = 0.9923f;     // ~3 s peak follower
      const float decayLong = 0.9992f; // ~30 s reference -> floor, so pure noise is not stretched to full scale
      for (int b = 0; b < 3; b++) {
        _env[b] = fmaxf(e[b], _env[b] * decay);
        _envLong[b] = fmaxf(e[b], _envLong[b] * decayLong);
        float ref = fmaxf(_env[b], 0.15f * _envLong[b]);
        float n = (ref > 1e-6f) ? clamp01(e[b] / ref) : 0.0f;
        bands[b] += (n > bands[b] ? 0.6f : 0.15f) * (n - bands[b]);   // fast attack, ~150 ms release
      }
    }

    void estimateTempo() {
      const int n = _histFill;
      const float* x = &_hist[_histPos + HIST - n];   // the last n samples, oldest first
      float lagMin = _fps * 60.0f / maxBpm;
      float lagMax = _fps * 60.0f / minBpm;
      if (lagMin < 2.0f) lagMin = 2.0f;
      if (4.0f * lagMax + 1.0f > (float)MAX_LAG) lagMax = (float)(MAX_LAG - 1) / 4.0f;
      const int acMax = (int)(4.0f * lagMax) + 1;
      // mean-removed autocorrelation: an aperiodic (noise) onset envelope -> ~0 at every lag
      float mean = 0.0f;
      for (int i = 0; i < n; i++) mean += x[i];
      mean /= float(n);
      for (int i = 0; i < n; i++) _xc[i] = x[i] - mean;
      for (int L = 0; L <= acMax; L++) {
        float s = 0.0f;
        for (int i = L; i < n; i++) s += _xc[i] * _xc[i - L];
        _ac[L] = s / float(n - L);
      }
      if (_ac[0] <= 1e-9f) { confidence *= 0.9f; return; }

      // harmonic comb on a 1/4-frame lag grid (fractional multiples via interpolation -> no smearing
      // of the 2nd..4th harmonic when the period is not a whole number of frames)
      float bestL = -1.0f, bestScore = 0.0f;
      for (float L = lagMin; L <= lagMax; L += 0.25f) {
        float bpmL = _fps * 60.0f / L;
        float oct = log2f(bpmL / 120.0f);
        float prior = expf(-0.5f * oct * oct / (0.8f * 0.8f));   // log-Gaussian, sigma = 0.8 octave
        float s = comb(L) * prior;
        if (s > bestScore) { bestScore = s; bestL = L; }
      }
      if (bestL < 0.0f) { confidence *= 0.9f; return; }
      float est = _fps * 60.0f / bestL;
      float ratio = comb(bestL) / (_ac[0] * 2.08f);    // 2.08 = 1 + 1/2 + 1/3 + 1/4
      float c = clamp01((ratio - 0.10f) / 0.30f);

      if (bpm <= 0.0f) { bpm = est; _candHits = 0; }
      else {
        float dev = fabsf(logf(est / bpm));
        if (dev < 0.04f) { bpm += 0.25f * (est - bpm); _candHits = 0; }  // same tempo - refine
        else {
          if (_candBpm > 0.0f && fabsf(logf(est / _candBpm)) < 0.04f) _candHits++;
          else { _candBpm = est; _candHits = 1; }
          c *= 0.5f;                                   // disagreement lowers trust
          if (_candHits >= 4) { bpm = _candBpm; _candHits = 0; _locked = false; _phaseErrAvg = 0.0f; } // persistent: switch
        }
      }
      confidence += 0.25f * (c - confidence);
    }

    inline float acAt(float l) const {
      int i = (int)l; float f = l - (float)i;
      return _ac[i] * (1.0f - f) + _ac[i + 1] * f;
    }
    inline float comb(float L) const {
      return acAt(L) + 0.5f * acAt(2.0f * L) + 0.333f * acAt(3.0f * L) + 0.25f * acAt(4.0f * L);
    }
};
