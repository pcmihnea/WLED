// =============================================================================
//  movinghead - audio-reactive DMX moving heads, the WLED-native way
//
//  COLOUR  : a normal WLED (audio) effect colours a small source segment; its pixels are averaged
//            into each head's 3 LED rings (+ ring FX: beat flash, chase, spectrum, solid, pulse, rainbow).
//  MOTION  : driven by AudioReactive (volume, bass/mid/high, onsets, tempo + beat phase) or a tap tempo:
//              Auto - continuous path, the music speeds it up and grows it
//              Beat - one eased step per 1/2, 1, 2 or 4 beats, started early so it lands on the beat
//              Sync - path phase-locked to the tempo: one loop every N beats
//              Hold - fixed aim (pan/tilt sliders), beam + nod/shake still react
//            30 patterns with rotation / aspect / direction (incl. ping-pong), crossfaded pattern changes
//            (by hand, every N bars or on drops), drop detection, silence handling, beat nod + hit shake,
//            freeze, timing nudge, multi-head spread / mirror / dimmer chase.
//  OUTPUT  : critically-damped smoothing with per-axis speed limits, 16-bit pan/tilt,
//            non-blocking hardware-timed DMX (esp_dmx), up to 8 heads, built-in UKing 16/24/32-ch
//            profiles or a custom channel map + fixed channel values for any other fixture.
//
//  Show controls (0..10 sliders or short lists) live in the main UI "Mover" tab and are saved in presets
//  (WLED state). Fixture setup lives in Config -> Usermods (cfg.json, saved by WLED as usual).
//
//  DMX out via esp_dmx on UART2/GPIO1. WLED's own DMX output (WLED_ENABLE_DMX) is not used.
// =============================================================================
#include "wled.h"
#include <math.h>

#if defined(ARDUINO_ARCH_ESP32)
  #include <esp_dmx.h>
  #ifndef MH_DMX_PORT
    #define MH_DMX_PORT DMX_NUM_2          // UART2
  #endif
  #ifndef MH_DMX_TX_PIN
    #define MH_DMX_TX_PIN 1                // AtomS3R Port.A G1 = Unit-DMX Grove pin 4 (TXD)
  #endif
#endif

#ifndef MH_DEFAULT_ADDR
  #define MH_DEFAULT_ADDR 1
#endif
#ifndef USERMOD_ID_MOVINGHEAD
  #define USERMOD_ID_MOVINGHEAD 59
#endif

#define MH_MAX_FIX       8
#define MH_NUM_PATTERNS  30
#define MH_TWO_PI        6.2831853f
#define MH_XFADE_MS      1500              // pattern change crossfade

static const char mhPatNames[MH_NUM_PATTERNS][12] PROGMEM = {
  "Circle", "Figure-8", "Lissa 3:2", "Lissa 5:4", "Sweep H", "Sweep V", "Diagonal", "Spiral",
  "Sway", "Random", "Corners", "Figure-8 V", "Rose 4", "Rose 3", "Lissa 5:3", "Zigzag",
  "Saw sweep", "Breathe", "Spiral out", "Square", "Triangle", "Star", "Pendulum", "Ballyhoo",
  "Wave", "Crowd scan", "Bounce", "Cross", "Heart", "Flower"
};

// ---- live show settings: small integers (0..10 sliders or list indexes) -> presets ----
struct MHLive {
  uint8_t mode = 0, pattern = 0, cycle = 0, cycOrd = 0, speed = 4, size = 8, bpc = 4, stepDiv = 1, smooth = 5;
  uint8_t aimX = 5, aimY = 5, aspect = 5, rotate = 0, dir = 0, freeze = 0, motor = 10;
  uint8_t react = 0, sens = 5, depth = 6, accent = 5, nod = 0, shake = 0, drop = 1, silence = 1, tempo = 0, nudge = 5;
  uint8_t dimmer = 1, dimFloor = 3, strobe = 0, strobeSpd = 10, zoom = 1, zMan = 5, ringFx = 0, spread = 0, mirror = 0;
};
struct MHKey { const char* k; uint8_t MHLive::* p; uint8_t lo, hi; };
static const MHKey mhKeys[] = {
  {"mode", &MHLive::mode, 0, 3},       {"pattern", &MHLive::pattern, 0, MH_NUM_PATTERNS - 1},
  {"cycle", &MHLive::cycle, 0, 5},     {"cycOrd", &MHLive::cycOrd, 0, 1},
  {"speed", &MHLive::speed, 0, 10},    {"size", &MHLive::size, 0, 10},      {"bpc", &MHLive::bpc, 1, 16},
  {"stepDiv", &MHLive::stepDiv, 0, 3}, {"smooth", &MHLive::smooth, 0, 10},
  {"aimX", &MHLive::aimX, 0, 10},      {"aimY", &MHLive::aimY, 0, 10},      {"aspect", &MHLive::aspect, 0, 10},
  {"rotate", &MHLive::rotate, 0, 10},  {"dir", &MHLive::dir, 0, 2},         {"freeze", &MHLive::freeze, 0, 1},
  {"motor", &MHLive::motor, 0, 10},
  {"react", &MHLive::react, 0, 4},     {"sens", &MHLive::sens, 0, 10},      {"depth", &MHLive::depth, 0, 10},
  {"accent", &MHLive::accent, 0, 10},  {"nod", &MHLive::nod, 0, 10},        {"shake", &MHLive::shake, 0, 10},
  {"drop", &MHLive::drop, 0, 1},       {"silence", &MHLive::silence, 0, 2}, {"tempo", &MHLive::tempo, 0, 1},
  {"nudge", &MHLive::nudge, 0, 10},
  {"dimmer", &MHLive::dimmer, 0, 6},   {"dimFloor", &MHLive::dimFloor, 0, 10}, {"strobe", &MHLive::strobe, 0, 4},
  {"strobeSpd", &MHLive::strobeSpd, 0, 10}, {"zoom", &MHLive::zoom, 0, 3},  {"zMan", &MHLive::zMan, 0, 10},
  {"ringFx", &MHLive::ringFx, 0, 6},   {"spread", &MHLive::spread, 0, 10},  {"mirror", &MHLive::mirror, 0, 3},
};
#define MH_NUM_KEYS (sizeof(mhKeys) / sizeof(mhKeys[0]))

class MovingHead : public Usermod {
  private:
    // ================= fixture setup (Config -> Usermods, cfg.json) =================
    bool     enabled     = true;
    uint8_t  profile     = 24;               // 16 / 24 / 32 = UKing ZQ02253 modes, 0 = custom map
    uint8_t  chCount     = 24;               // custom profile: channels per head
    String   chMapStr    = F("1,21,2,22,3,4,17,18,5,6,7,8,9,10,11,12,13,14,15,16");
    String   fixedStr    = "";               // "ch=value,..." held constant (shutter open, colour wheel, ...)
    uint16_t dmxAddr     = MH_DEFAULT_ADDR;  // DMX address of head 1
    uint8_t  fixtures    = 1;                // number of identical heads (1..8)
    uint8_t  addrStep    = 0;                // address distance between heads, 0 = channel count
    uint8_t  sourceSeg   = 0;                // WLED segment whose pixels colour the rings
    uint8_t  ringMap     = 0;                // 0 = segment in thirds, 1 = physical 1:5:12 LEDs
    bool     colorSplit  = false;            // several heads: each gets its own slice of the segment
    uint16_t panDeg      = 540, tiltDeg = 270;            // fixture travel (DMX 0..65535 spans this)
    uint16_t panCenter   = 90,  panRange  = 180;          // safe motion box (deg)
    uint16_t tiltCenter  = 45,  tiltRange = 90;
    bool     panInvert   = false, tiltInvert = false;
    uint16_t maxPanSpd   = 300, maxTiltSpd = 200;         // deg/s, 0 = unlimited
    uint16_t lagMs       = 60;               // fixture lag; Beat/Sync moves start this early
    uint8_t  zoomNarrow  = 40,  zoomWide  = 220;
    uint8_t  strobeMin   = 20, strobeVal = 200;           // strobe channel: slowest .. fastest flash value
    uint8_t  ptSpeed     = 0;
    uint8_t  whiteMode   = 1;                // 0 Off / 1 Auto / 2 from RGB
    uint8_t  whiteMix    = 128;
    bool     parkOff     = true;             // WLED off -> centre, still, dark
    uint8_t  silenceSec  = 5;                // seconds without sound before "On silence" applies
    bool     fullFrame   = true;
    uint8_t  fps         = 40;

    // ================= live show settings =================
    MHLive live;

    // ================= resolved channel map =================
    struct ChMap { int8_t pan, panF, tilt, tiltF, ptSpd, dim, strobe, zoom; int8_t ring[3][4]; uint8_t count, rings; };
    ChMap cm;
    struct Fixed { uint8_t off, val; };
    Fixed fixedCh[8]; uint8_t nFixed = 0;

    // ================= runtime =================
    struct Head {
      float panS = 0, tiltS = 0, panV = 0, tiltV = 0, zoomS = 128;
      float fromX = 0, fromY = 0, toX = 0, toY = 0, curX = 0, curY = 0, rx = 0, ry = 0;
      bool  init = false;
    };
    Head hd[MH_MAX_FIX];
    unsigned long lastFrameUs = 0;
    float phase = 0.0f, beatPos = 0.0f, beatOffset = 0.0f, dirSign = 1.0f;
    float loopBeats = 0.0f, loopOff = 0.0f;                                 // Sync: effective beats per loop + phase continuity
    // per-pattern motion tables (built once): speed normalisation and peak axis speed per radian of phase
    float patK[MH_NUM_PATTERNS], patMx[MH_NUM_PATTERNS], patMy[MH_NUM_PATTERNS];
    // drop detection: kick activity (onset impulses, ~1.5 s) vs. the song's normal level (~20 s)
    float kickAct = 0.0f, kickRef = 0.0f;
    unsigned long breakStart = 0;
    bool  syncLocked = false;
    float volEnv = 0, bandEnv[3] = {0, 0, 0}, accentEnv = 0, nodEnv = 0, shakeEnv = 0, shakeSign = 1.0f;
    uint8_t  stepIdx = 0, prevMode = 255, curPattern = 0, prevPattern = 0, lastSetPattern = 255, clockSrc = 255;
    uint32_t lastBeats = 0, lastOnsets = 0, legacyPeaks = 0, barAnchor = 0, stepEdges = 0;
    int32_t  lastStepIdx = 0;
    bool     haveBeatRef = false, haveBarRef = false, stepRef = false, peakPrev = false, wasOutputting = false, wasBreak = false;
    unsigned long strobeUntil = 0, stepStart = 0, lastSound = 0, lastStrong = 0, dropAt = 0, lastBarMs = 0, xfStart = 0;
    uint16_t stepDur = 1;
    int16_t  barsSinceChange = 0, barsDir = 0;                               // -1 = the anchor beat is still to come
    // tap tempo
    unsigned long tapTimes[8] = {0}, tapRef = 0, tapRefSeen = 0;
    uint8_t  tapN = 0;
    float    tapPeriod = 0.0f;               // ms, 0 = no tap tempo

    // ================= exported for the display (getUMData) / info =================
    float    xPan = 0, xTilt = 0, xBpm = 0, xAccent = 0;
    float    xLimits[4] = {0, 540, 0, 270};
    float    xPos[2 * MH_MAX_FIX] = {0};
    uint8_t  xDim = 0, xZoom = 0, xLocked = 0, xFix = 1;
    uint32_t xRings[3] = {0, 0, 0};
    const char* xPatName = mhPatNames[0];
    bool     xDrop = false, xSilent = false;
    uint8_t  xSrc = 0, xDropFlash = 0, xBeatInBar = 255;                   // tempo source (0 audio, 1 tap), drop flash, beat 0..3 in the bar

    // ================= DMX =================
    uint8_t  _pkt[513] = {0};
    bool     _dmxReady = false;
    uint32_t _framesSent = 0, _framesSkipped = 0;

    static inline float clamp01(float v) { return v < 0.0f ? 0.0f : (v > 1.0f ? 1.0f : v); }
    static inline float s10(uint8_t v) { return v * 0.1f; }                                   // 0..10 -> 0..1
    static inline float mhtri(float x) { const float P=3.14159265f; return (2.0f/P)*asinf(sinf(x)); }
    static inline float mhsaw(float x) { float t=x/MH_TWO_PI; return 2.0f*(t-floorf(t+0.5f)); }
    static inline float ease(float t) { t = clamp01(t); return t * t * (3.0f - 2.0f * t); }
    static inline float frac1(float x) { return x - floorf(x); }

    // critically damped spring toward target with a max speed (Unity SmoothDamp)
    static float smoothDamp(float cur, float target, float &vel, float smoothTime, float maxSpeed, float dt) {
      if (smoothTime < 0.001f) smoothTime = 0.001f;
      float omega = 2.0f / smoothTime;
      float x = omega * dt;
      float ex = 1.0f / (1.0f + x + 0.48f * x * x + 0.235f * x * x * x);
      float change = cur - target;
      float origTarget = target;
      if (maxSpeed > 0.0f) { float m = maxSpeed * smoothTime; change = constrain(change, -m, m); }
      target = cur - change;
      float temp = (vel + omega * change) * dt;
      vel = (vel - omega * temp) * ex;
      float out = target + (change + temp) * ex;
      if ((origTarget - cur > 0.0f) == (out > origTarget)) { out = origTarget; vel = 0.0f; }
      if (maxSpeed > 0.0f) vel = constrain(vel, -maxSpeed, maxSpeed);
      return out;
    }

    // point on a closed polyline through n vertices, u in [0,1)
    static void poly(const float* vx, const float* vy, int n, float u, float &x, float &y) {
      u = frac1(u) * n; int i = (int)u; if (i >= n) i = n - 1; float f = u - i; int j = (i + 1) % n;
      x = vx[i] + (vx[j] - vx[i]) * f; y = vy[i] + (vy[j] - vy[i]) * f;
    }

    // normalized parametric path -> (nx,ny) in [-1,1]
    void path(uint8_t p, float th, float &nx, float &ny, const Head &h, uint8_t corner) {
      switch (p) {
        case 1:  nx = sinf(th);    ny = sinf(2*th);          break; // figure-8
        case 2:  nx = sinf(3*th);  ny = sinf(2*th);          break; // lissajous 3:2
        case 3:  nx = sinf(5*th);  ny = sinf(4*th + 0.785f); break; // lissajous 5:4
        case 4:  nx = sinf(th);    ny = 0.0f;                break; // sweep H
        case 5:  nx = 0.0f;        ny = sinf(th);            break; // sweep V
        case 6:  nx = sinf(th);    ny = sinf(th);            break; // diagonal
        case 7:  { float r = fabsf(sinf(0.11f*th)); nx = r*cosf(th); ny = r*sinf(th); } break; // spiral
        case 8:  nx = sinf(th);    ny = 0.25f*sinf(2*th);    break; // sway
        case 9:  nx = h.rx;        ny = h.ry;                break; // random target per beat
        case 10: { const float px[4]={-1,1,1,-1}, py[4]={-1,-1,1,1}; nx=px[corner&3]; ny=py[corner&3]; } break; // corners
        case 11: nx = sinf(2*th);  ny = sinf(th);            break; // figure-8 vertical
        case 12: { float r=cosf(2*th);    nx=r*cosf(th); ny=r*sinf(th); } break; // rose 4
        case 13: { float r=cosf(1.5f*th); nx=r*cosf(th); ny=r*sinf(th); } break; // rose 3
        case 14: nx = sinf(5*th);  ny = sinf(3*th);          break; // lissajous 5:3
        case 15: nx = mhtri(th);   ny = mhtri(2*th);         break; // zigzag
        case 16: nx = mhsaw(th);   ny = 0.0f;                break; // saw sweep
        case 17: { float r=0.35f+0.65f*(0.5f+0.5f*sinf(0.25f*th)); nx=r*cosf(th); ny=r*sinf(th); } break; // breathe
        case 18: { float r=frac1(0.1f*th); nx=r*cosf(th); ny=r*sinf(th); } break; // spiral out
        case 19: { const float vx[4]={-1,1,1,-1}, vy[4]={-1,-1,1,1}; poly(vx,vy,4,th/MH_TWO_PI,nx,ny); } break; // square
        case 20: { const float vx[3]={0,0.95f,-0.95f}, vy[3]={1,-0.8f,-0.8f}; poly(vx,vy,3,th/MH_TWO_PI,nx,ny); } break; // triangle
        case 21: { float vx[5], vy[5]; for (int k=0;k<5;k++){ float a=1.5708f+k*2.0f*MH_TWO_PI/5.0f; vx[k]=cosf(a); vy[k]=sinf(a);} poly(vx,vy,5,th/MH_TWO_PI,nx,ny); } break; // star
        case 22: { float a=1.1f*sinf(th); nx=sinf(a)/0.8912f; ny=-1.0f+2.0f*(1.0f-cosf(a))/0.5464f; } break; // pendulum
        case 23: nx = 0.6f*sinf(th)+0.4f*sinf(2.3f*th+1.0f); ny = 0.6f*sinf(1.7f*th+2.0f)+0.4f*sinf(3.1f*th); break; // ballyhoo
        case 24: nx = mhtri(th);   ny = sinf(3*th);          break; // wave
        case 25: nx = sinf(th);    ny = -0.6f+0.35f*fabsf(sinf(4*th)); break; // crowd scan (low, hopping)
        case 26: nx = mhtri(0.5f*th); ny = 2.0f*fabsf(sinf(2*th))-1.0f; break; // bounce
        case 27: { float u=frac1(th/(2*MH_TWO_PI)); if (u<0.5f){ nx=sinf(u*2*MH_TWO_PI); ny=0; } else { nx=0; ny=sinf((u-0.5f)*2*MH_TWO_PI);} } break; // cross
        case 28: { float s=sinf(th); nx=s*s*s; ny=((13*cosf(th)-5*cosf(2*th)-2*cosf(3*th)-cosf(4*th))/17.0f+0.353f)/0.647f; } break; // heart
        case 29: { float r=cosf(5*th); nx=r*cosf(th); ny=r*sinf(th); } break; // flower (rose 5)
        default: nx = cosf(th);    ny = sinf(th);            break; // circle
      }
    }

    // Measure every pattern once: mean path speed (circle = 1) and the peak |dx/dth|, |dy/dth|.
    // Jumps (saw snap, spiral-out reset) are skipped - the speed limits handle those.
    void buildPatternTables() {
      Head h0; const int N = 1024; const float span = 4.0f * MH_TWO_PI, dth = span / N;
      for (uint8_t p = 0; p < MH_NUM_PATTERNS; p++) {
        if (p == 9 || p == 10) { patK[p] = 1.0f; patMx[p] = patMy[p] = 1.0f; continue; }   // per-beat targets
        float px, py, x, y, sum = 0.0f, mx = 0.0f, my = 0.0f; int n = 0;
        path(p, 0.0f, px, py, h0, 0);
        for (int i = 1; i <= N; i++) {
          path(p, i * dth, x, y, h0, 0);
          float dx = fabsf(x - px), dy = fabsf(y - py);
          px = x; py = y;
          if (dx > 0.25f || dy > 0.25f) continue;
          sum += sqrtf(dx * dx + dy * dy) / dth; mx = fmaxf(mx, dx / dth); my = fmaxf(my, dy / dth); n++;
        }
        float mean = n ? sum / n : 1.0f;
        patK[p]  = 1.0f / fmaxf(mean, 0.1f);       // phase-rate scale so every pattern travels at the circle's beam speed
        patMx[p] = fmaxf(mx, 0.01f); patMy[p] = fmaxf(my, 0.01f);
      }
    }
    // highest phase rate (rad/s) at which pattern p stays within 85% of the pan/tilt speed limits
    float govRate(uint8_t p, float amp, float ax, float ay, float rc, float rs, float motorMul) const {
      float dxb = patMx[p] * fabsf(rc) + patMy[p] * fabsf(rs), dyb = patMx[p] * fabsf(rs) + patMy[p] * fabsf(rc);
      float aPan = amp * ax * panRange * 0.5f, aTilt = amp * ay * tiltRange * 0.5f;     // deg per unit
      float r = 1e9f;
      if (maxPanSpd  && aPan  * dxb > 0.01f) r = fminf(r, 0.85f * maxPanSpd  * motorMul / (aPan  * dxb));
      if (maxTiltSpd && aTilt * dyb > 0.01f) r = fminf(r, 0.85f * maxTiltSpd * motorMul / (aTilt * dyb));
      return r;
    }

    // ---------------- channel map ----------------
    void buildChMap() {
      ChMap m; memset(&m, 0xFF, sizeof(m));      // all offsets -1 (absent)
      if (profile == 32) {
        m = ChMap{ 0,1,2,3, 4, 6, 7, 5, {{9,10,11,12},{13,14,15,16},{17,18,19,20}}, 32, 3 };
      } else if (profile == 16) {
        m = ChMap{ 0,12,1,13, 2, 3, 8, 9, {{4,5,6,7},{-1,-1,-1,-1},{-1,-1,-1,-1}}, 16, 1 };
      } else if (profile == 24) {
        m = ChMap{ 0,20,1,21, 2, 3, 16, 17, {{4,5,6,7},{8,9,10,11},{12,13,14,15}}, 24, 3 };
      } else {                                   // custom: 1-based head channels, 0 = not present
        int8_t* slots[20] = { &m.pan,&m.panF,&m.tilt,&m.tiltF,&m.ptSpd,&m.dim,&m.strobe,&m.zoom,
          &m.ring[0][0],&m.ring[0][1],&m.ring[0][2],&m.ring[0][3],&m.ring[1][0],&m.ring[1][1],&m.ring[1][2],&m.ring[1][3],
          &m.ring[2][0],&m.ring[2][1],&m.ring[2][2],&m.ring[2][3] };
        m.count = constrain(chCount, 1, 64);
        const char* s = chMapStr.c_str();
        for (int i = 0; i < 20; i++) {
          while (*s == ' ' || *s == ',' || *s == ';') s++;
          if (!*s) { *slots[i] = -1; continue; }
          int v = atoi(s);
          *slots[i] = (v >= 1 && v <= m.count) ? (int8_t)(v - 1) : -1;
          while (*s && *s != ',' && *s != ';') s++;
        }
        m.rings = (m.ring[2][0] >= 0 || m.ring[2][1] >= 0) ? 3 : ((m.ring[1][0] >= 0 || m.ring[1][1] >= 0) ? 2 : 1);
      }
      cm = m;
      // fixed channels "ch=val,ch=val"
      nFixed = 0;
      const char* s = fixedStr.c_str();
      while (*s && nFixed < 8) {
        while (*s == ' ' || *s == ',' || *s == ';') s++;
        if (!*s) break;
        int ch = atoi(s); while (*s && *s != '=' && *s != ',' && *s != ';') s++;
        if (*s != '=') continue;
        s++; int v = atoi(s); while (*s && *s != ',' && *s != ';') s++;
        if (ch >= 1 && ch <= cm.count) fixedCh[nFixed++] = { (uint8_t)(ch - 1), (uint8_t)constrain(v, 0, 255) };
      }
    }
    inline uint16_t headAddr(uint8_t f) const { return dmxAddr + f * (addrStep ? addrStep : cm.count); }
    inline void wr(uint16_t base, int8_t off, uint8_t v) {
      if (off < 0) return;
      int ch = base + off;
      if (ch >= 1 && ch <= 512) _pkt[ch] = v;
    }

    // ---------------- inputs ----------------
    uint32_t avgPixels(Segment& sg, int a, int b) {
      int len = sg.length();
      a = constrain(a, 0, len - 1); b = constrain(b, a + 1, len);
      uint32_t ar=0, ag=0, ab=0, aw=0; int cnt = 0;
      for (int i = a; i < b; i++) { uint32_t c = sg.getPixelColor(i); ar+=R(c); ag+=G(c); ab+=B(c); aw+=W(c); cnt++; }
      if (cnt < 1) cnt = 1;
      return RGBW32(ar/cnt, ag/cnt, ab/cnt, aw/cnt);
    }
    // ring colours of head f (own slice of the segment when colorSplit)
    bool readRings(uint8_t f, uint32_t rc[3]) {
      rc[0] = rc[1] = rc[2] = 0;
      Segment& sg = (sourceSeg < strip.getSegmentsNum()) ? strip.getSegment(sourceSeg) : strip.getMainSegment();
      if (!sg.isActive()) return false;
      int len = sg.length();
      if (len < 1) return false;
      int a0 = 0, l = len;
      if (colorSplit && fixtures > 1 && len >= fixtures) { a0 = (f * len) / fixtures; l = ((f + 1) * len) / fixtures - a0; }
      int e[4];
      if (ringMap == 1) { e[0] = 0; e[1] = (l + 17) / 18; e[2] = (l * 6 + 17) / 18; e[3] = l; }
      else for (int r = 0; r < 4; r++) e[r] = (r * l) / 3;
      for (int r = 0; r < 3; r++) rc[r] = avgPixels(sg, a0 + e[r], a0 + max(e[r + 1], e[r] + 1));
      return true;
    }

    struct Audio {
      bool ok = false, tracker = false;
      float vol = 0, bands[3] = {0, 0, 0}, bpm = 0, conf = 0, onsetStr = 0;
      uint32_t beats = 0, onsets = 0, lastBeatMs = 0;
    };
    void readAudio(Audio &a) {
      um_data_t* um;
      if (!UsermodManager::getUMData(&um, USERMOD_ID_AUDIOREACTIVE) || !um) return;
      a.ok  = true;
      a.vol = clamp01(*(float*)um->u_data[0] / 255.0f);
      if (um->u_size >= 15) {                         // this fork's beat tracker
        a.tracker    = true;
        a.beats      = *(uint32_t*)um->u_data[8];
        a.bpm        = *(float*)um->u_data[9];
        a.conf       = *(float*)um->u_data[10];
        a.lastBeatMs = *(uint32_t*)um->u_data[11];
        a.onsets     = *(uint32_t*)um->u_data[12];
        a.onsetStr   = *(float*)um->u_data[13];
        float* b     = (float*)um->u_data[14];
        a.bands[0] = b[0]; a.bands[1] = b[1]; a.bands[2] = b[2];
      } else {                                        // stock AR: samplePeak edges = beats + onsets
        bool pk = (*(uint8_t*)um->u_data[3]) != 0;
        if (pk && !peakPrev) legacyPeaks++;
        peakPrev = pk;
        a.beats = a.onsets = legacyPeaks; a.onsetStr = 1.0f;
        a.bands[0] = a.bands[1] = a.bands[2] = a.vol;
      }
    }

    // tap tempo: >= 3 taps, gaps < 2 s; the last tap is a beat
    void registerTap(unsigned long now) {
      if (tapN && now - tapTimes[(tapN - 1) & 7] > 2000) tapN = 0;
      tapTimes[tapN & 7] = now; tapN++;
      if (tapN >= 3) {
        uint8_t n = tapN > 8 ? 8 : tapN;
        float per = (now - tapTimes[(tapN - n) & 7]) / float(n - 1);
        if (per >= 250.0f && per <= 1500.0f) { tapPeriod = per; tapRef = now; }   // 40..240 BPM
      }
    }

    // ---------------- live settings (presets) ----------------
    void writeLive(JsonObject t, bool forPreset) {
      for (size_t i = 0; i < MH_NUM_KEYS; i++) {
        if (forPreset && mhKeys[i].p == &MHLive::freeze) continue;          // a frozen preset makes no sense
        t[mhKeys[i].k] = live.*(mhKeys[i].p);
      }
      t["v"] = 2;
    }
    // returns true if anything was present. v1 presets (0..100 sliders, old key names) are converted.
    bool readLive(JsonObject t) {
      if (t.isNull()) return false;
      bool legacy = t["v"].isNull();
      bool any = false;
      for (size_t i = 0; i < MH_NUM_KEYS; i++) {
        int v;
        if (!getJsonValue(t[mhKeys[i].k], v)) continue;
        if (legacy && mhKeys[i].hi == 10) v = (v + 5) / 10;                    // 0..100 -> 0..10
        live.*(mhKeys[i].p) = (uint8_t)constrain(v, (int)mhKeys[i].lo, (int)mhKeys[i].hi);
        any = true;
      }
      if (legacy) {
        int v; bool b;
        if (getJsonValue(t["aDepth"], v)) { live.depth = constrain((v + 5) / 10, 0, 10); any = true; }
        if (getJsonValue(t["zReact"], b)) { live.zoom = b ? 1 : 0; any = true; }
      }
      return any;
    }

    // ---------------- DMX ----------------
    void sendFrame() {
      #if defined(ARDUINO_ARCH_ESP32)
      if (!_dmxReady) return;
      // Non-blocking: if the previous frame is still on the wire, skip this tick. Writing into the
      // driver buffer mid-frame would tear 16-bit pan/tilt (coarse new + fine old = visible jumps).
      if (!dmx_wait_sent(MH_DMX_PORT, 0)) { _framesSkipped++; return; }
      int last = headAddr(fixtures - 1) + cm.count - 1; if (last > 512) last = 512;
      size_t len = fullFrame ? 513 : (size_t)max(25, last + 1);
      _pkt[0] = 0;
      dmx_write(MH_DMX_PORT, _pkt, len);
      dmx_send(MH_DMX_PORT, len);
      _framesSent++;
      #endif
    }
    void blackoutAll() {
      for (uint8_t f = 0; f < fixtures; f++) for (int o = 0; o < cm.count; o++) wr(headAddr(f), (int8_t)o, 0);
    }

    // a motion step: new random targets / next corner
    void advanceStep() {
      float rx = (hw_random8() / 127.5f) - 1.0f, ry = (hw_random8() / 127.5f) - 1.0f;
      for (uint8_t f = 0; f < fixtures; f++) {
        if (live.spread == 0) { hd[f].rx = rx; hd[f].ry = ry; }                // unison: same target
        else { hd[f].rx = (hw_random8() / 127.5f) - 1.0f; hd[f].ry = (hw_random8() / 127.5f) - 1.0f; }
      }
      stepIdx++;
    }
    void setPattern(uint8_t p, unsigned long now) {
      if (p == curPattern) return;
      prevPattern = curPattern; curPattern = p; xfStart = now | 1;            // crossfade from the old path
      xPatName = mhPatNames[p]; barsSinceChange = 0;
    }
    void nextPattern(unsigned long now) {
      uint8_t p = curPattern;
      if (live.cycOrd) { do { p = (p + 1) % MH_NUM_PATTERNS; } while (p == 9 || p == 10); }   // sequential
      else for (int i = 0; i < 8 && (p == curPattern || p == 9 || p == 10); i++) p = hw_random8() % MH_NUM_PATTERNS;
      setPattern(p, now);
    }

  public:
    void setup() override {
      buildChMap();
      buildPatternTables();
      um_data = new um_data_t;
      um_data->u_size = 16;
      um_data->u_type = new um_types_t[um_data->u_size];
      um_data->u_data = new void*[um_data->u_size];
      um_data->u_data[0]  = &xPan;       um_data->u_type[0]  = UMT_FLOAT;      // head 1 pan, deg
      um_data->u_data[1]  = &xTilt;      um_data->u_type[1]  = UMT_FLOAT;      // head 1 tilt, deg
      um_data->u_data[2]  = &xDim;       um_data->u_type[2]  = UMT_BYTE;       // dimmer DMX
      um_data->u_data[3]  = &xZoom;      um_data->u_type[3]  = UMT_BYTE;       // zoom DMX
      um_data->u_data[4]  = xRings;      um_data->u_type[4]  = UMT_UINT32_ARR; // head 1 rings RGBW
      um_data->u_data[5]  = &live.mode;  um_data->u_type[5]  = UMT_BYTE;       // 0 Auto 1 Beat 2 Sync 3 Hold
      um_data->u_data[6]  = &xLocked;    um_data->u_type[6]  = UMT_BYTE;
      um_data->u_data[7]  = xLimits;     um_data->u_type[7]  = UMT_FLOAT_ARR;  // panMin, panMax, tiltMin, tiltMax
      um_data->u_data[8]  = &xBpm;       um_data->u_type[8]  = UMT_FLOAT;
      um_data->u_data[9]  = &xAccent;    um_data->u_type[9]  = UMT_FLOAT;
      um_data->u_data[10] = &xPatName;   um_data->u_type[10] = UMT_BYTE_ARR;   // const char** current pattern
      um_data->u_data[11] = &xFix;       um_data->u_type[11] = UMT_BYTE;       // number of heads
      um_data->u_data[12] = xPos;        um_data->u_type[12] = UMT_FLOAT_ARR;  // pan,tilt of every head
      um_data->u_data[13] = &xSrc;       um_data->u_type[13] = UMT_BYTE;       // 0 audio tempo, 1 tap tempo
      um_data->u_data[14] = &xDropFlash; um_data->u_type[14] = UMT_BYTE;       // 1 during the first 2 s of a drop
      um_data->u_data[15] = &xBeatInBar; um_data->u_type[15] = UMT_BYTE;       // 0..3, 255 = no tempo

      curPattern = prevPattern = live.pattern; lastSetPattern = live.pattern; xPatName = mhPatNames[curPattern];

      lastFrameUs = micros();
      #if defined(ARDUINO_ARCH_ESP32)
      if (!PinManager::allocatePin(MH_DMX_TX_PIN, true, PinOwner::UM_Unspecified)) {
        DEBUG_PRINTF_P(PSTR("MovingHead: DMX TX pin %d already in use - output disabled\n"), MH_DMX_TX_PIN);
        return;
      }
      if (!dmx_driver_is_installed(MH_DMX_PORT)) {
        dmx_config_t cfg = DMX_CONFIG_DEFAULT;            // 250k baud, 176us break, 12us MAB
        dmx_driver_install(MH_DMX_PORT, &cfg, DMX_INTR_FLAGS_DEFAULT);
      }
      dmx_set_pin(MH_DMX_PORT, MH_DMX_TX_PIN, -1, -1);    // TX only
      dmx_set_baud_rate(MH_DMX_PORT, DMX_BAUD_RATE);
      _dmxReady = dmx_driver_is_installed(MH_DMX_PORT);
      #endif
    }

    bool getUMData(um_data_t **data) override {
      if (!data || !enabled || !um_data) return false;
      *data = um_data;
      return true;
    }

    void loop() override {
      if (!enabled) {
        if (wasOutputting) { blackoutAll(); sendFrame(); wasOutputting = false; }
        return;
      }
      unsigned long nowUs = micros();
      unsigned long periodUs = 1000000UL / constrain(fps, 10, fullFrame ? 44 : 60);
      if (nowUs - lastFrameUs < periodUs) return;
      float dt = (nowUs - lastFrameUs) * 1e-6f; if (dt > 0.1f) dt = 0.1f;
      lastFrameUs = nowUs;
      const unsigned long now = millis();
      wasOutputting = true;
      if (live.pattern != lastSetPattern) { setPattern(live.pattern, now); lastSetPattern = live.pattern; }

      // ================= audio =================
      Audio au; readAudio(au);
      const bool off = (bri == 0);
      volEnv += ((au.ok ? au.vol : 0.0f) - volEnv) * (1.0f - expf(-dt / 0.12f));
      for (int b = 0; b < 3; b++) {
        float t = au.ok ? au.bands[b] : 0.0f;
        bandEnv[b] += (t - bandEnv[b]) * (1.0f - expf(-dt / (t > bandEnv[b] ? 0.04f : 0.18f)));   // punchy attack
      }
      float src;
      switch (live.react) {
        case 1:  src = bandEnv[0]; break;                                    // bass
        case 2:  src = bandEnv[1]; break;                                    // mid
        case 3:  src = bandEnv[2]; break;                                    // highs
        case 4:  src = volEnv; break;                                        // volume
        default: src = 0.45f * volEnv + 0.55f * bandEnv[0]; break;          // mix
      }
      const float sensMul = powf(2.0f, (live.sens - 5) / 2.5f);              // 0..10 -> 0.25x..4x
      const float drive = clamp01(src * sensMul);
      const float depth = s10(live.depth), accAmt = s10(live.accent);
      const int32_t nudgeMs = ((int)live.nudge - 5) * 40;                     // timing nudge, -200..+200 ms

      // ---- tempo source: the audio beat tracker, or the tap clock (Tempo = Tap, or while audio has no lock) ----
      const bool audioLocked = au.tracker && au.bpm > 1.0f && au.conf >= 0.3f
                               && (int32_t)(now - au.lastBeatMs) < (int32_t)(180000.0f / au.bpm);
      const bool useTap = tapPeriod > 0.0f && (live.tempo == 1 || !audioLocked);
      bool clockChanged = false;
      if ((uint8_t)useTap != clockSrc) { clockSrc = useTap; haveBeatRef = false; syncLocked = false; clockChanged = true; }
      xSrc = useTap;
      if (useTap) {
        uint32_t n = (now >= tapRef) ? (uint32_t)((now - tapRef) / tapPeriod) : 0;
        au.beats = n; au.lastBeatMs = tapRef + (unsigned long)(n * tapPeriod);
        au.bpm = 60000.0f / tapPeriod; au.conf = 1.0f; au.tracker = true;
        if (!au.ok) au.onsetStr = 0.8f;
      }

      // beat clock: tempo-locked when confident, otherwise raw beats/onsets
      float periodMs = (au.bpm > 1.0f) ? 60000.0f / au.bpm : 0.0f;
      bool locked = au.tracker && periodMs > 0.0f && au.conf >= 0.3f
                    && (int32_t)(now - au.lastBeatMs) < (int32_t)(3.0f * periodMs);
      xLocked = locked; xBpm = locked ? au.bpm : 0.0f;

      bool beatEdge  = haveBeatRef && (au.beats != lastBeats);
      bool onsetEdge = haveBeatRef && (au.onsets != lastOnsets);
      if (!haveBeatRef || (au.beats - lastBeats) > 8) { stepRef = false; beatEdge = onsetEdge = false; }
      lastBeats = au.beats; lastOnsets = au.onsets; haveBeatRef = au.ok || useTap;
      const bool strongHit = onsetEdge && au.onsetStr >= 0.6f;

      // silence
      const bool wasSilent = !useTap && (now - lastSound) > (unsigned long)silenceSec * 1000UL;
      if (au.ok && au.vol > 0.02f) lastSound = now;
      const bool silent   = !useTap && (now - lastSound) > (unsigned long)silenceSec * 1000UL;
      const bool silDrift = silent && live.silence == 1, silPark = silent && live.silence == 2;
      xSilent = silent;

      // drop = the kick returns after a real breakdown: kick activity (onset impulses, ~1.5 s) falls below 30% of
      // the song's normal level (~20 s average, frozen during the break) for >= 4 bars (8 s without tempo) while
      // music keeps playing, then a strong hit. At most one drop per 30 s.
      bool dropNow = false;
      kickAct = kickAct * expf(-dt / 1.5f) + (onsetEdge ? au.onsetStr : 0.0f);
      const bool sound = au.ok && !silent && volEnv > 0.03f;
      if (!breakStart && sound) kickRef += (kickAct - kickRef) * (1.0f - expf(-dt / 20.0f));
      if (breakStart) {
        const unsigned long minBreak = locked ? (unsigned long)(16.0f * periodMs) : 8000UL;
        if (sound && strongHit && now - breakStart >= minBreak && (!dropAt || now - dropAt > 30000)) { dropAt = now; dropNow = true; breakStart = 0; }
        else if (!sound || kickAct > 0.6f * kickRef) breakStart = 0;           // kick back without a drop moment, or silence
      } else if (sound && kickRef > 0.3f && kickAct < 0.3f * kickRef) breakStart = now | 1;
      if (strongHit) lastStrong = now;
      wasBreak = breakStart != 0;
      const unsigned long dropLen = locked ? (unsigned long)(8.0f * periodMs) : 4000UL;
      const bool dropOn    = dropAt && (now - dropAt) < dropLen;
      const bool dropFlash = dropAt && (now - dropAt) < 1200;
      xDrop = dropOn; xDropFlash = dropAt && (now - dropAt) < 2000;

      // ---- phrase alignment: bars count from a real downbeat. Anchors: a drop, the first strong hit after
      //      silence (music starts), a new tap tempo (the last tap). Pattern changes then land on phrase starts. ----
      bool anchorNow = dropNow || clockChanged || (strongHit && wasSilent) || (!haveBarRef && strongHit)
                       || (useTap && tapRef != tapRefSeen);                    // (re)tapped: the last tap is beat 1
      tapRefSeen = tapRef;
      if (anchorNow && (locked || useTap)) {
        bool beatAhead = locked && !useTap && (float)(now - au.lastBeatMs) > 0.5f * periodMs;   // hit is closer to the next beat
        barAnchor = au.beats + (beatAhead ? 1 : 0);
        barsSinceChange = barsDir = (beatAhead || beatEdge) ? -1 : 0;         // the anchor's own bar edge counts as bar 0
        haveBarRef = true;
      }
      bool barEdge = false;
      if (locked) { if (beatEdge && ((au.beats - barAnchor) & 3) == 0) barEdge = true; }
      else if (now - lastBarMs > 2000) barEdge = true;
      if (barEdge) lastBarMs = now;
      xBeatInBar = locked ? (uint8_t)((au.beats - barAnchor) & 3) : 255;
      // a drop starts a new section: new pattern in every auto-change mode
      if (dropNow && live.cycle >= 1 && !live.freeze) { int16_t b = barsSinceChange; nextPattern(now); barsSinceChange = b; }

      if (beatEdge) accentEnv = 0.6f + 0.4f * clamp01(au.onsetStr);
      accentEnv *= powf(0.05f, dt);
      xAccent = accentEnv;
      if (beatEdge) nodEnv = 1.0f;
      if (strongHit) { shakeEnv = 1.0f; shakeSign = -shakeSign; }
      const float fastDecay = powf(0.002f, dt);                               // ~0.16 s: crisp kicks that settle between beats
      nodEnv *= fastDecay; shakeEnv *= fastDecay;

      // bar counters: pattern auto-change, ping-pong direction
      if (barEdge && !live.freeze) {
        static const uint8_t bars[5] = {0, 4, 8, 16, 32};
        if (live.cycle >= 1 && live.cycle <= 4 && ++barsSinceChange >= (int16_t)bars[live.cycle]) nextPattern(now);
        if (live.dir == 2 && ++barsDir >= 4) { barsDir = 0; dirSign = -dirSign; }
      }
      if (live.dir == 0) dirSign = 1.0f; else if (live.dir == 1) dirSign = -1.0f;

      // strobe
      if (!off && !silPark) {
        switch (live.strobe) {
          case 1: if (beatEdge)  strobeUntil = now + 60; break;
          case 2: if (strongHit) strobeUntil = now + 60; break;
          case 3: if (barEdge && locked) strobeUntil = now + 80; break;
          case 4: if (dropFlash) strobeUntil = now + 30; break;
        }
        if (live.drop && live.strobe && dropFlash) strobeUntil = now + 30;
      }

      // ================= motion (shared time base) =================
      if (live.mode != prevMode) {
        for (uint8_t f = 0; f < fixtures; f++) { hd[f].fromX = hd[f].toX = hd[f].curX; hd[f].fromY = hd[f].toY = hd[f].curY; }
        stepStart = now; stepDur = 1; syncLocked = false; stepRef = false; prevMode = live.mode;
      }
      const float rate = 0.25f * powf(1.45f, live.speed);                      // rad/s, speed 0..10 = 0.25..10
      const float smoothTime = 0.02f + 0.5f * s10(live.smooth) * s10(live.smooth);
      const float motorMul = 0.2f + 0.08f * live.motor;                        // speed limit scale 0.2..1.0
      uint8_t mode = live.mode;
      if (silDrift && mode == 1) mode = 0;                                     // no beats to step on: drift
      const bool frozen = live.freeze;
      float amp = s10(live.size) * (1.0f + depth * drive) * (1.0f + 0.6f * accAmt * accentEnv);
      if (live.drop && dropOn) amp *= 1.3f;
      if (silDrift) amp *= 0.6f;
      amp = clamp01(amp);
      const float ax = live.aspect <= 5 ? 1.0f : (10 - live.aspect) * 0.2f;   // 0 = flat (pan only) .. 10 = tilt only
      const float ay = live.aspect >= 5 ? 1.0f : live.aspect * 0.2f;
      const float rotA = live.rotate * (3.14159265f / 10.0f), rc_ = cosf(rotA), rs_ = sinf(rotA);
      const float xf = (xfStart && now - xfStart < MH_XFADE_MS && prevPattern != curPattern) ? ease((now - xfStart) / (float)MH_XFADE_MS) : 1.0f;
      // speed governor: never ask for more than the fixture can follow (else it cuts the figure into small circles)
      float gov = govRate(curPattern, amp, ax, ay, rc_, rs_, motorMul);
      if (xf < 1.0f) gov = fminf(gov, govRate(prevPattern, amp, ax, ay, rc_, rs_, motorMul));
      const float kPat = patK[curPattern];                                     // equal beam speed for every pattern

      bool step = false;
      if (mode == 0) {
        float r = rate * kPat * (1.0f + depth * drive) * (silDrift ? 0.25f : 1.0f);
        if (!frozen) phase += dirSign * fminf(r, gov) * dt;
        if (beatEdge) advanceStep();
      } else if (mode == 1) {
        static const float divK[4] = {2.0f, 1.0f, 0.5f, 0.25f};              // steps per beat: 1/2, 1, 2, 4 beats
        static const uint8_t divN[4] = {1, 1, 2, 4};
        if (locked) {
          float bp = (float)au.beats + (float)(int32_t)(now + lagMs + nudgeMs - au.lastBeatMs) / periodMs;
          int32_t idx = (int32_t)floorf(bp * divK[live.stepDiv]);
          if (!stepRef || idx < lastStepIdx - 4) { lastStepIdx = idx; stepRef = true; }
          else if (idx > lastStepIdx) { step = true; lastStepIdx = idx; }
        } else if (beatEdge) {
          stepRef = false;
          if ((++stepEdges % divN[live.stepDiv]) == 0) step = true;
        }
        if (frozen) step = false;
        if (step) {
          advanceStep();
          phase += dirSign * (0.3f + 0.25f * live.speed) * kPat;               // Speed = step size (same beam distance for every pattern)
          stepStart = now;
          float base = locked ? periodMs / divK[live.stepDiv] : 400.0f * divN[live.stepDiv];
          stepDur = (uint16_t)constrain(base * (0.2f + 0.06f * live.smooth), 40.0f, 4000.0f);
        }
      } else if (mode == 2) {
        // loop length: complex patterns loop over more beats (x2/x4.., so loops stay on bar lines), and more again
        // if the path would still be faster than the fixture. Re-evaluated on bar lines; phase stays continuous.
        const float pMs = (locked && periodMs > 0.0f) ? periodMs : 500.0f;
        const float ampMax = clamp01(s10(live.size) * (1.0f + depth) * (1.0f + 0.6f * accAmt) * (live.drop ? 1.3f : 1.0f));
        const float govMax = govRate(curPattern, ampMax, ax, ay, rc_, rs_, motorMul);
        int k = (int)lroundf(log2f(1.0f / kPat)); k = constrain(k, 0, 3);
        while (k < 5 && MH_TWO_PI * 1000.0f / (live.bpc * (float)(1 << k) * pMs) > govMax) k++;
        const float wantLoop = live.bpc * (float)(1 << k);
        if (loopBeats <= 0.0f) loopBeats = wantLoop;
        else if (wantLoop != loopBeats && (barEdge || !locked || (prevPattern != curPattern && xf < 0.05f))) {
          loopOff += beatPos / loopBeats - beatPos / wantLoop;                // keep the phase continuous
          loopBeats = wantLoop;
        }
        float bpc = loopBeats;
        if (frozen) syncLocked = false;                                        // re-align smoothly after a freeze
        else if (locked && !silDrift) {
          float lead = lagMs + nudgeMs + smoothTime * 1000.0f;                 // fixture lag + nudge + filter delay
          float fr = ((float)(int32_t)(now - au.lastBeatMs) + lead) / periodMs;
          fr = constrain(fr, -1.0f, 3.0f);
          float raw = (float)au.beats + fr;
          if (!syncLocked) { beatOffset = roundf(beatPos - raw); syncLocked = true; }
          float err = raw + beatOffset - beatPos;
          if (fabsf(err) > 1.5f) { beatOffset -= roundf(err); err = raw + beatOffset - beatPos; }
          beatPos += dt * 1000.0f / periodMs + err * (1.0f - expf(-dt / 0.15f));
        } else {
          syncLocked = false;
          float r = rate * kPat * (1.0f + depth * drive) * (silDrift ? 0.25f : 1.0f);
          beatPos += dt * fminf(r, gov) * bpc / MH_TWO_PI;
        }
        float u = beatPos / bpc + loopOff;                                     // pattern loops
        if (live.dir == 1) u = -u;
        else if (live.dir == 2) { float lp = floorf(u), fr = u - lp; u = ((int32_t)lp & 1) ? 1.0f - fr : fr; }  // ping-pong per loop
        phase = MH_TWO_PI * u;
        if (beatEdge) advanceStep();
      }
      if (phase > 1e5f || phase < -1e5f) phase = fmodf(phase, MH_TWO_PI * 40.0f);
      if (beatPos > 1e5f) { loopOff += 1e5f / loopBeats; beatPos -= 1e5f; beatOffset -= 1e5f; }
      if (loopOff > 1e4f || loopOff < -1e4f) loopOff = fmodf(loopOff, 1.0f);

      const float nodY   = nodEnv * 0.6f * s10(live.nod);                     // tilt kick on beats
      const float shakeX = shakeSign * shakeEnv * 0.25f * s10(live.shake);    // pan flick on strong hits

      const float pMin = max(0.0f, panCenter - panRange * 0.5f),  pMax = min((float)panDeg,  panCenter + panRange * 0.5f);
      const float tMin = max(0.0f, tiltCenter - tiltRange * 0.5f), tMax = min((float)tiltDeg, tiltCenter + tiltRange * 0.5f);
      const float cPan  = panCenter  + (live.aimX - 5) * 0.2f * panRange  * 0.5f;   // aim shifts the centre
      const float cTilt = tiltCenter + (live.aimY - 5) * 0.2f * tiltRange * 0.5f;
      xLimits[0] = pMin; xLimits[1] = pMax; xLimits[2] = tMin; xLimits[3] = tMax;

      // ================= beam (shared by all heads; dimmer chase per head) =================
      const float floorLv = s10(live.dimFloor);
      float lv;
      switch (live.dimmer) {
        case 0:  lv = 1.0f; break;
        case 2:  lv = floorLv + (1.0f - floorLv) * accentEnv; break;
        case 3:  { float T = locked ? 4.0f * periodMs : 4000.0f; lv = floorLv + (1.0f - floorLv) * (0.5f - 0.5f * cosf(MH_TWO_PI * fmodf((float)now, T) / T)); } break;
        case 4:  lv = 0.0f; break;
        case 5: case 6: lv = 1.0f; break;                                     // chase / alternate: per head below
        default: lv = floorLv + (1.0f - floorLv) * clamp01(drive + 0.3f * accAmt * accentEnv); break;
      }
      if (live.drop && dropFlash && live.dimmer != 4) lv = 1.0f;
      float zT;
      const float zr = (float)zoomWide - (float)zoomNarrow;
      switch (live.zoom) {
        case 0:  zT = zoomWide - zr * s10(live.zMan); break;                  // 0 = wide, 10 = narrow
        case 2:  zT = zoomNarrow + zr * accentEnv; break;
        case 3:  zT = zoomWide - zr * clamp01(drive); break;
        default: zT = zoomNarrow + zr * clamp01(drive + 0.25f * accentEnv); break;
      }
      if (live.drop && dropOn && live.zoom != 0) zT = zoomWide;
      const bool strobeOn = !off && (int32_t)(strobeUntil - now) > 0;
      const uint8_t strobeDmx = strobeMin + (uint8_t)(((int)strobeVal - strobeMin) * live.strobeSpd / 10);

      // ================= per head =================
      const uint8_t nF = fixtures;
      const bool parked = (off && parkOff) || silPark;
      for (uint8_t f = 0; f < nF; f++) {
        Head &h = hd[f];
        const float offF = (nF > 1) ? f * s10(live.spread) * MH_TWO_PI / nF : 0.0f;
        const uint8_t corner = stepIdx + (live.spread ? f : 0);
        const bool mirX = (live.mirror == 1 || live.mirror == 3) && (f & 1);
        const bool mirY = (live.mirror == 2 || live.mirror == 3) && (f & 1);
        float panT, tiltT;

        if (parked || mode == 3) {
          float hx = panCenter, hy = tiltCenter;                               // park = centre of the box
          if (!parked) {                                                       // Hold: aim = position in the box
            hx = pMin + (pMax - pMin) * s10(live.aimX);
            hy = tMin + (tMax - tMin) * s10(live.aimY);
            if (nF > 1) hx += (f - (nF - 1) * 0.5f) * s10(live.spread) * (pMax - pMin) / nF;   // spread = fan
            if (mirX) hx = 2.0f * cPan - hx;
            hx += (panInvert ? -shakeX : shakeX) * panRange * 0.5f;            // nod / shake still apply
            hy += (tiltInvert ? -nodY : nodY) * tiltRange * 0.5f;
          }
          panT = hx; tiltT = hy;
          h.curX = h.curY = 0;
        } else {
          float nx, ny;
          if (mode == 1) {
            if (step) {
              h.fromX = h.curX; h.fromY = h.curY;
              path(curPattern, phase + offF, h.toX, h.toY, h, corner);
            }
            float e = ease((now - stepStart) / (float)stepDur);
            nx = h.fromX + (h.toX - h.fromX) * e;
            ny = h.fromY + (h.toY - h.fromY) * e;
          } else {
            path(curPattern, phase + offF, nx, ny, h, corner);
            if (xf < 1.0f) {                                                   // crossfade from the previous pattern
              float ox, oy; path(prevPattern, phase + offF, ox, oy, h, corner);
              nx = ox + (nx - ox) * xf; ny = oy + (ny - oy) * xf;
            }
          }
          h.curX = nx; h.curY = ny;
          // shape: rotation + aspect, then size, nod / shake, mirror, invert
          float rx = nx * rc_ - ny * rs_, ry = nx * rs_ + ny * rc_;
          float px = constrain(rx, -1.0f, 1.0f) * ax * amp + shakeX;
          float py = constrain(ry, -1.0f, 1.0f) * ay * amp + nodY;
          if (mirX) px = -px;
          if (mirY) py = -py;
          if (panInvert)  px = -px;
          if (tiltInvert) py = -py;
          panT  = cPan  + (panRange  * 0.5f) * px;
          tiltT = cTilt + (tiltRange * 0.5f) * py;
        }
        panT  = constrain(panT,  pMin, pMax);
        tiltT = constrain(tiltT, tMin, tMax);
        if (!h.init) { h.panS = panT; h.tiltS = tiltT; h.init = true; }
        const float st = (mode == 1 && !parked) ? 0.03f : smoothTime;         // Beat: the easing shapes the move
        h.panS  = smoothDamp(h.panS,  panT,  h.panV,  st, maxPanSpd  * motorMul, dt);
        h.tiltS = smoothDamp(h.tiltS, tiltT, h.tiltV, st, maxTiltSpd * motorMul, dt);
        h.zoomS += (zT - h.zoomS) * (live.zoom == 0 ? 1.0f : (1.0f - expf(-dt / 0.15f)));   // zoom motor is slow
        xPos[2 * f] = h.panS; xPos[2 * f + 1] = h.tiltS;

        // ---- dimmer (chase modes are per head) ----
        float lvH = lv;
        if (live.dimmer == 5) lvH = (nF > 1 ? (au.beats % nF) == f : (au.beats & 1)) ? 1.0f : floorLv;   // one head per beat
        if (live.dimmer == 6) lvH = ((au.beats + f) & 1) ? 1.0f : floorLv;                               // odd / even swap
        if (live.drop && dropFlash && live.dimmer != 4) lvH = 1.0f;
        const uint8_t dim = (off || silPark) ? 0 : (uint8_t)(bri * clamp01(lvH));

        // ---- rings ----
        uint32_t rc[3];
        bool haveColor = readRings(f, rc) && !off && !silPark;
        if (!haveColor) rc[0] = rc[1] = rc[2] = 0;
        switch (live.ringFx) {
          case 1: {                                                           // beat flash toward white
            uint8_t k = (uint8_t)(200.0f * accentEnv);
            if (haveColor) for (int r = 0; r < 3; r++) rc[r] = color_blend(rc[r], RGBW32(255, 255, 255, 255), k);
          } break;
          case 2: {                                                           // chase: colours rotate per beat
            uint32_t s3[3] = {rc[0], rc[1], rc[2]}; uint8_t sh = (au.beats + f) % 3;
            for (int r = 0; r < 3; r++) rc[r] = s3[(r + sh) % 3];
          } break;
          case 3: {                                                           // spectrum: centre=bass, mid=mid, outer=highs
            for (int r = 0; r < 3; r++) rc[r] = color_fade(rc[r], (uint8_t)(255.0f * clamp01(0.1f + 0.9f * bandEnv[r] * sensMul)));
          } break;
          case 4: {                                                           // solid: LED-count weighted average
            uint32_t r=0,g=0,b=0,w=0; const uint8_t wt[3] = {1, 5, 12};
            for (int q = 0; q < 3; q++) { r += R(rc[q])*wt[q]; g += G(rc[q])*wt[q]; b += B(rc[q])*wt[q]; w += W(rc[q])*wt[q]; }
            rc[0] = rc[1] = rc[2] = RGBW32(r/18, g/18, b/18, w/18);
          } break;
          case 5: {                                                           // pulse: brightness follows the beat
            for (int r = 0; r < 3; r++) rc[r] = color_fade(rc[r], (uint8_t)(40.0f + 215.0f * accentEnv));
          } break;
          case 6: {                                                           // rainbow: own colours, hue steps per beat
            if (!off && !silPark) for (int r = 0; r < 3; r++) {
              CRGBW c(CHSV((uint8_t)(au.beats * 24 + now / 64 + r * 85 + f * 32), 255, 255));
              rc[r] = c.color32;
            }
          } break;
        }

        // ---- DMX for this head ----
        const uint16_t base = headAddr(f);
        for (int o = 0; o < cm.count; o++) wr(base, (int8_t)o, 0);         // undriven channels stay 0
        uint16_t p16 = (uint16_t)constrain(h.panS  / panDeg  * 65535.0f + 0.5f, 0.0f, 65535.0f);
        uint16_t t16 = (uint16_t)constrain(h.tiltS / tiltDeg * 65535.0f + 0.5f, 0.0f, 65535.0f);
        wr(base, cm.pan, p16 >> 8);  wr(base, cm.panF, p16 & 0xFF);
        wr(base, cm.tilt, t16 >> 8); wr(base, cm.tiltF, t16 & 0xFF);
        wr(base, cm.ptSpd, ptSpeed);
        wr(base, cm.dim, dim);
        uint8_t z8 = (uint8_t)constrain(h.zoomS + 0.5f, 0.0f, 255.0f);
        wr(base, cm.zoom, z8);
        wr(base, cm.strobe, strobeOn ? strobeDmx : 0);
        for (uint8_t r = 0; r < 3; r++) {
          uint32_t c = rc[r];
          uint8_t cr = R(c), cg = G(c), cb = B(c), w = 0;
          uint8_t pk = cr>cg ? (cr>cb?cr:cb) : (cg>cb?cg:cb);
          switch (whiteMode) {
            case 0:  w = 0; break;
            case 2:  w = (uint16_t)whiteMix * pk / 255; break;
            default: w = W(c); if (w == 0 && whiteMix) w = (uint16_t)whiteMix * pk / 255; break;
          }
          if (f == 0) xRings[r] = (r < cm.rings) ? RGBW32(cr, cg, cb, w) : 0;
          if (r < cm.rings) { wr(base, cm.ring[r][0], cr); wr(base, cm.ring[r][1], cg); wr(base, cm.ring[r][2], cb); wr(base, cm.ring[r][3], w); }
        }
        for (uint8_t i = 0; i < nFixed; i++) wr(base, fixedCh[i].off, fixedCh[i].val);
        if (f == 0) { xPan = h.panS; xTilt = h.tiltS; xDim = dim; xZoom = z8; }
      }
      xFix = nF;
      sendFrame();
    }

    // ---- live show controls (Mover tab, presets, JSON API) ----
    void addToJsonState(JsonObject& root) override {
      writeLive(root.createNestedObject(F("MovingHead")), umStateForPreset);
    }
    void readFromJsonState(JsonObject& root) override {
      JsonObject t = root[F("MovingHead")];
      if (t.isNull()) return;
      int v;
      if (getJsonValue(t["tap"], v)) { if (v) registerTap(millis()); else { tapPeriod = 0.0f; tapN = 0; } }   // 1 = tap, 0 = clear
      if (getJsonValue(t["next"], v) && v) { nextPattern(millis()); live.pattern = curPattern; lastSetPattern = curPattern; }  // "next pattern" button
      readLive(t);
    }

    void addToJsonInfo(JsonObject& root) override {
      // structured status for the Mover tab
      JsonObject mh = root.createNestedObject(F("mh"));
      mh[F("bpm")]  = xLocked ? (int)(xBpm + 0.5f) : 0;
      mh[F("src")]  = (clockSrc == 1) ? F("tap") : F("audio");
      mh[F("tap")]  = tapPeriod > 0.0f ? (int)(60000.0f / tapPeriod + 0.5f) : 0;
      mh[F("pat")]  = xPatName;
      mh[F("drop")] = xDrop;
      mh[F("sil")]  = xSilent;
      mh[F("heads")] = fixtures;
      mh[F("skip")] = _framesSkipped;

      JsonObject u = root["u"]; if (u.isNull()) u = root.createNestedObject("u");
      JsonArray a = u.createNestedArray(F("Moving Head"));
      if (!enabled) { a.add(F("disabled")); return; }
      if (!_dmxReady) { a.add(F("DMX driver not ready")); return; }
      char buf[56];
      int last = min(512, (int)headAddr(fixtures - 1) + cm.count - 1);
      snprintf_P(buf, sizeof(buf), PSTR("%u head%s, ch %u-%d, %u fps"), fixtures, fixtures > 1 ? "s" : "", dmxAddr, last, fps);
      a.add(buf);
      if (xLocked) { snprintf_P(buf, sizeof(buf), PSTR(" | %d BPM%s"), (int)(xBpm + 0.5f), clockSrc == 1 ? " (tap)" : ""); a.add(buf); }
      else a.add(F(" | free-running"));
    }

    // ---- fixture setup -> Usermods menu (cfg.json) ----
    void addToConfig(JsonObject& root) override {
      JsonObject t = root.createNestedObject(F("MovingHead"));
      t[F("on")]=enabled;
      t[F("profile")]=profile; t[F("chCount")]=chCount; t[F("chMap")]=chMapStr; t[F("fixed")]=fixedStr;
      t[F("addr")]=dmxAddr; t[F("heads")]=fixtures; t[F("addrStep")]=addrStep;
      t[F("srcSeg")]=sourceSeg; t[F("ringMap")]=ringMap; t[F("colorSplit")]=colorSplit;
      t[F("panDeg")]=panDeg; t[F("tiltDeg")]=tiltDeg;
      t[F("panC")]=panCenter; t[F("panR")]=panRange; t[F("tiltC")]=tiltCenter; t[F("tiltR")]=tiltRange;
      t[F("panInv")]=panInvert; t[F("tiltInv")]=tiltInvert;
      t[F("panSpd")]=maxPanSpd; t[F("tiltSpd")]=maxTiltSpd; t[F("lagMs")]=lagMs;
      t[F("zMin")]=zoomNarrow; t[F("zMax")]=zoomWide;
      t[F("strobeMin")]=strobeMin; t[F("strobeVal")]=strobeVal; t[F("ptSpd")]=ptSpeed;
      t[F("wMode")]=whiteMode; t[F("white")]=whiteMix;
      t[F("parkOff")]=parkOff; t[F("silenceSec")]=silenceSec;
      t[F("fullFrame")]=fullFrame; t[F("fps")]=fps;
    }
    bool readFromConfig(JsonObject& root) override {
      JsonObject t = root[F("MovingHead")];
      bool ok = !t.isNull();
      bool wasEnabled = enabled;
      ok &= getJsonValue(t[F("on")],enabled,true);
      if (t[F("profile")].isNull() && t[F("chMode")].is<int>()) { profile = t[F("chMode")].as<int>(); ok = false; }   // v1 config
      else ok &= getJsonValue(t[F("profile")],profile,24);
      ok &= getJsonValue(t[F("chCount")],chCount,24);
      if (t[F("chMap")].is<const char*>()) chMapStr = t[F("chMap")].as<const char*>(); else ok = false;
      if (t[F("fixed")].is<const char*>()) fixedStr = t[F("fixed")].as<const char*>(); else ok = false;
      ok &= getJsonValue(t[F("addr")],dmxAddr,MH_DEFAULT_ADDR);
      ok &= getJsonValue(t[F("heads")],fixtures,1);       ok &= getJsonValue(t[F("addrStep")],addrStep,0);
      ok &= getJsonValue(t[F("srcSeg")],sourceSeg,0);     ok &= getJsonValue(t[F("ringMap")],ringMap,0);
      ok &= getJsonValue(t[F("colorSplit")],colorSplit,false);
      ok &= getJsonValue(t[F("panDeg")],panDeg,540);      ok &= getJsonValue(t[F("tiltDeg")],tiltDeg,270);
      ok &= getJsonValue(t[F("panC")],panCenter,90);      ok &= getJsonValue(t[F("panR")],panRange,180);
      ok &= getJsonValue(t[F("tiltC")],tiltCenter,45);    ok &= getJsonValue(t[F("tiltR")],tiltRange,90);
      ok &= getJsonValue(t[F("panInv")],panInvert,false); ok &= getJsonValue(t[F("tiltInv")],tiltInvert,false);
      ok &= getJsonValue(t[F("panSpd")],maxPanSpd,300);   ok &= getJsonValue(t[F("tiltSpd")],maxTiltSpd,200);
      ok &= getJsonValue(t[F("lagMs")],lagMs,60);
      ok &= getJsonValue(t[F("zMin")],zoomNarrow,40);     ok &= getJsonValue(t[F("zMax")],zoomWide,220);
      ok &= getJsonValue(t[F("strobeMin")],strobeMin,20); ok &= getJsonValue(t[F("strobeVal")],strobeVal,200);
      ok &= getJsonValue(t[F("ptSpd")],ptSpeed,0);
      ok &= getJsonValue(t[F("wMode")],whiteMode,1);      ok &= getJsonValue(t[F("white")],whiteMix,128);
      ok &= getJsonValue(t[F("parkOff")],parkOff,true);   ok &= getJsonValue(t[F("silenceSec")],silenceSec,5);
      ok &= getJsonValue(t[F("fullFrame")],fullFrame,true); ok &= getJsonValue(t[F("fps")],fps,40);
      // sanitize
      if (profile != 0 && profile != 16 && profile != 32) profile = 24;
      chCount    = constrain(chCount, 1, 64);
      dmxAddr    = constrain(dmxAddr, 1, 512);
      fixtures   = constrain(fixtures, 1, MH_MAX_FIX);
      panDeg     = constrain(panDeg, 90, 720);    tiltDeg   = constrain(tiltDeg, 90, 360);
      panCenter  = min(panCenter, panDeg);        panRange  = min(panRange,  panDeg);
      tiltCenter = min(tiltCenter, tiltDeg);      tiltRange = min(tiltRange, tiltDeg);
      lagMs      = min(lagMs, (uint16_t)500);
      if (strobeMin > strobeVal) strobeMin = strobeVal;
      if (ringMap > 1) ringMap = 0;
      if (whiteMode > 2) whiteMode = 1;
      fps = constrain(fps, 10, fullFrame ? 44 : 60);
      buildChMap();
      if (wasEnabled && !enabled) { blackoutAll(); sendFrame(); wasOutputting = false; }
      for (uint8_t f = 0; f < MH_MAX_FIX; f++) hd[f].init = false;        // re-seed smoothing after setup changes
      return ok;
    }
    void appendConfigData(Print& uiScript) override {
      uiScript.print(F("ux='MovingHead';"));
      uiScript.print(F("dd=addDropdown(ux,'profile');addOption(dd,'UKing 16-ch',16);addOption(dd,'UKing 24-ch (3 rings)',24);addOption(dd,'UKing 32-ch',32);addOption(dd,'Custom (chCount + chMap)',0);"));
      uiScript.print(F("dd=addDropdown(ux,'ringMap');addOption(dd,'Rings: segment in thirds',0);addOption(dd,'Rings: physical 1/5/12',1);"));
      uiScript.print(F("dd=addDropdown(ux,'wMode');addOption(dd,'W: Off (RGB only)',0);addOption(dd,'W: Auto (pixel, else from RGB)',1);addOption(dd,'W: from RGB',2);"));
      uiScript.print(F("addInfo(ux+':on',1,'enable the moving-head DMX output');"));
      uiScript.print(F("addInfo(ux+':chCount',1,'Custom: channels per head');"));
      uiScript.print(F("addInfo(ux+':chMap',1,'<br>Custom: head channel (1-based, 0 = none) of pan,panFine,tilt,tiltFine,ptSpeed,dimmer,strobe,zoom,R1,G1,B1,W1,R2,G2,B2,W2,R3,G3,B3,W3');"));
      uiScript.print(F("addInfo(ux+':fixed',1,'<br>head channels held at a value, e.g. 6=255,11=8 (shutter open, colour wheel...)');"));
      uiScript.print(F("addInfo(ux+':addr',1,'DMX address of head 1');"));
      uiScript.print(F("addInfo(ux+':heads',1,'identical heads, 1-8');"));
      uiScript.print(F("addInfo(ux+':addrStep',1,'address distance between heads, 0 = channel count');"));
      uiScript.print(F("addInfo(ux+':srcSeg',1,'segment whose pixels colour the rings - run any audio effect on it');"));
      uiScript.print(F("addInfo(ux+':colorSplit',1,'several heads: each takes its own slice of the segment');"));
      uiScript.print(F("addInfo(ux+':panDeg',1,'deg, full pan travel of the fixture');"));
      uiScript.print(F("addInfo(ux+':tiltDeg',1,'deg, full tilt travel');"));
      uiScript.print(F("addInfo(ux+':panC',1,'deg, centre of the safe motion box');"));
      uiScript.print(F("addInfo(ux+':panR',1,'deg, pan width of the box');"));
      uiScript.print(F("addInfo(ux+':tiltC',1,'deg');"));
      uiScript.print(F("addInfo(ux+':tiltR',1,'deg');"));
      uiScript.print(F("addInfo(ux+':panInv',1,'mirror pan (ceiling / flipped mount)');"));
      uiScript.print(F("addInfo(ux+':panSpd',1,'deg/s max pan speed, 0 = unlimited');"));
      uiScript.print(F("addInfo(ux+':tiltSpd',1,'deg/s max tilt speed');"));
      uiScript.print(F("addInfo(ux+':lagMs',1,'ms fixture lag - Beat/Sync moves start this early');"));
      uiScript.print(F("addInfo(ux+':zMin',1,'zoom DMX, narrowest');"));
      uiScript.print(F("addInfo(ux+':zMax',1,'zoom DMX, widest');"));
      uiScript.print(F("addInfo(ux+':strobeMin',1,'strobe channel value, slowest flash (Mover tab strobe speed 0)');"));
      uiScript.print(F("addInfo(ux+':strobeVal',1,'strobe channel value, fastest flash (strobe speed 10)');"));
      uiScript.print(F("addInfo(ux+':ptSpd',1,'fixture pan/tilt speed channel, 0 = fastest');"));
      uiScript.print(F("addInfo(ux+':white',1,'synthesized white amount (0-255)');"));
      uiScript.print(F("addInfo(ux+':parkOff',1,'WLED off: centre, stop, dark');"));
      uiScript.print(F("addInfo(ux+':silenceSec',1,'s without sound before the Mover tab On silence action');"));
      uiScript.print(F("addInfo(ux+':fullFrame',1,'send all 512 slots (off: only up to the last head, up to 60 fps)');"));
      uiScript.print(F("addInfo(ux+':fps',1,'DMX refresh (10-44 full frame, 10-60 short)');"));
    }

    uint16_t getId() override { return USERMOD_ID_MOVINGHEAD; }
};

static MovingHead movinghead;
REGISTER_USERMOD(movinghead);
