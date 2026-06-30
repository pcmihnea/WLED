// =============================================================================
//  movinghead - DMX moving-head driven the WLED-native way
//  UKing ZQ02253 (16/24/32-ch), 540deg pan / 270deg tilt, 16-bit pan/tilt,
//  mechanical zoom, 3x RGBW rings (central=1 LED, mid=5, outer=12).
//
//  Design: a normal WLED (audio-reactive) EFFECT colours a small segment; this
//  usermod is a PIXEL -> DMX BRIDGE. Each frame it reads that segment's pixels,
//  averages them into the 3 ring colours (weighted by ring size), and also derives
//  a motion "energy" from the same pixels so pan/tilt/zoom track the music in step
//  with the visible colour reaction - no separate, glitchy beat engine.
//   * COLOUR  -> pick any effect + palette on the source segment (main GUI page).
//   * MOTION / hardware / limits -> this usermod's menu (set per venue).
//
//  Ring channel order is central(1 LED) / mid(5) / outer(12); the central + mid
//  rings are visually minor, so the motion energy is size-weighted toward the
//  outer ring (the dominant one).
//
//  DMX out via esp_dmx on UART2/GPIO1 (real hardware break + MAB, DMA, continuously
//  driven line). WLED's SparkFunDMX is disabled (WLED_ENABLE_DMX off) so it never runs.
//
//  Folder: usermods/movinghead/ { library.json, movinghead.cpp }
// =============================================================================
#include "wled.h"
#include <math.h>

// Real, hardware-timed DMX (proper break/MAB, DMA, continuously driven line) via esp_dmx -
// supersedes WLED's SparkFunDMX (which fakes the break by baud-switching and tears the UART
// down every frame). WLED_ENABLE_DMX is left OFF for this build so SparkFun never runs.
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

class MovingHead : public Usermod {
  private:
    // ---- config (Usermods menu) ----
    bool     enabled     = true;
    uint16_t dmxAddr     = MH_DEFAULT_ADDR;   // DMX channel of the fixture's first channel (Pan)
    uint8_t  chMode      = 24;                // 16 / 24 / 32
    uint8_t  sourceSeg   = 0;                 // WLED segment whose pixels colour the rings
    uint8_t  pattern     = 0;                 // motion path (0..10)
    uint8_t  speed       = 90;                // base motion speed
    uint8_t  size        = 200;              // base movement amplitude
    uint8_t  smoothing   = 180;              // motion low-pass (higher = smoother)
    uint8_t  audioDepth  = 150;              // how strongly energy grows size+speed (0 = steady motion)
    uint8_t  sensitivity = 64;               // scales the pixel-derived energy
    uint16_t panCenter   = 90,  panRange  = 180;   // motion limits (deg, within 0..540)
    uint16_t tiltCenter  = 45,  tiltRange = 90;    //              (deg, within 0..270)
    uint8_t  zoomNarrow  = 40,  zoomWide  = 220;   // beam DMX limits (energy maps narrow..wide)
    bool     zoomReact   = true;             // true: zoom follows energy; false: hold zoomManual (use to test/aim)
    uint8_t  zoomManual  = 128;              // fixed zoom value when zoomReact is off (sweep this to verify the beam)
    uint8_t  strobeMode  = 0;                // 0 Off 1 Beat hits
    uint8_t  ptSpeed     = 0;                // fixture pan/tilt-speed channel (0 = fastest)
    uint8_t  whiteMode   = 1;                // ring W: 0 Off (pure RGB) / 1 Auto (pixel W, else synth) / 2 RGB->W (always synth)
    uint8_t  whiteMix    = 128;              // amount of synthesized white (from the RGB peak) for modes 1/2
    uint8_t  dimmerMode  = 1;                // 0 Full 1 Energy
    uint8_t  fps         = 40;               // DMX refresh cap

    // ---- runtime state ----
    float phase=0, panS=0, tiltS=0, accent=0, volEnv=0, prevE=0, rndx=0, rndy=0;
    uint8_t stepIdx=0;
    unsigned long lastFrame=0;
    bool inited=false;

    // ---- DMX frame buffer (index 0 = start code, 1..512 = channel values) ----
    uint8_t _pkt[513] = {0};
    bool    _dmxReady = false;

    // ---- channel map per mode (offset from dmxAddr; -1 = absent) ----
    struct ChMap { int8_t pan, panF, tilt, tiltF, ptSpd, dim, strobe, zoom; int8_t ring[3][4]; };
    ChMap chmap() const {
      if (chMode == 32) return ChMap{ 0,1,2,3, 4, 6, 7, 5, {{9,10,11,12},{13,14,15,16},{17,18,19,20}} };
      if (chMode == 16) return ChMap{ 0,12,1,13, 2, 3, 8, 9, {{4,5,6,7},{-1,-1,-1,-1},{-1,-1,-1,-1}} };
      return ChMap{ 0,20,1,21, 2, 3, 16, 17, {{4,5,6,7},{8,9,10,11},{12,13,14,15}} };  // 24-ch (default)
    }
    inline void wr(int8_t off, uint8_t v) {
      if (off < 0) return;
      int ch = dmxAddr + off;
      if (ch >= 1 && ch <= 512) _pkt[ch] = v;   // 1-indexed; _pkt[0] stays the start code (0)
    }
    inline void wr16(int8_t hi, int8_t lo, uint16_t v) { wr(hi, v >> 8); wr(lo, v & 0xFF); }

    // normalized parametric path -> (nx,ny) in [-1,1]
    void path(uint8_t p, float th, float &nx, float &ny) {
      switch (p) {
        case 1:  nx = sinf(th);    ny = sinf(2*th);          break; // figure-8
        case 2:  nx = sinf(3*th);  ny = sinf(2*th);          break; // lissajous 3:2
        case 3:  nx = sinf(5*th);  ny = sinf(4*th + 0.785f); break; // lissajous 5:4
        case 4:  nx = sinf(th);    ny = 0.0f;                break; // sweep H
        case 5:  nx = 0.0f;        ny = sinf(th);            break; // sweep V
        case 6:  nx = sinf(th);    ny = sinf(th);            break; // diagonal
        case 7:  { float r = fabsf(sinf(0.11f*th)); nx = r*cosf(th); ny = r*sinf(th); } break; // spiral
        case 8:  nx = sinf(th);    ny = 0.25f*sinf(2*th);    break; // sway
        case 9:  nx = rndx;        ny = rndy;                break; // random-ease (energy-beat target)
        case 10: { const float px[4]={-1,1,1,-1}, py[4]={-1,-1,1,1}; nx=px[stepIdx&3]; ny=py[stepIdx&3]; } break; // beat-step
        default: nx = cosf(th);    ny = sinf(th);            break; // circle
      }
    }

    // read the source segment's pixels, averaged into 3 ring colours (RGBW packed)
    bool readRings(uint32_t rc[3]) {
      rc[0]=rc[1]=rc[2]=0;
      if (sourceSeg >= strip.getSegmentsNum()) return false;
      Segment& sg = strip.getSegment(sourceSeg);
      if (!sg.isActive()) return false;
      int len = sg.length();
      if (len < 1) return false;
      for (int r = 0; r < 3; r++) {
        int a = (int)((uint32_t)r * len / 3);
        int b = (int)((uint32_t)(r + 1) * len / 3);
        if (b <= a) b = a + 1;
        if (b > len) b = len;
        uint32_t ar=0, ag=0, ab=0, aw=0; int cnt=0;
        for (int i = a; i < b; i++) { uint32_t c = sg.getPixelColor(i); ar+=R(c); ag+=G(c); ab+=B(c); aw+=W(c); cnt++; }
        if (cnt < 1) cnt = 1;
        rc[r] = RGBW32(ar/cnt, ag/cnt, ab/cnt, aw/cnt);
      }
      return true;
    }
    static inline float lum(uint32_t c) {
      uint8_t r=R(c), g=G(c), b=B(c), w=W(c);
      uint8_t m = r>g ? (r>b?r:b) : (g>b?g:b); if (w>m) m=w;
      return m / 255.0f;
    }

  public:
    void setup() override {
      lastFrame = millis(); inited = true;
      #if defined(ARDUINO_ARCH_ESP32)
      PinManager::allocatePin(MH_DMX_TX_PIN, true, PinOwner::UM_Unspecified);
      if (!dmx_driver_is_installed(MH_DMX_PORT)) {
        dmx_config_t cfg = DMX_CONFIG_DEFAULT;            // 250k baud, 176us break, 12us MAB
        dmx_driver_install(MH_DMX_PORT, &cfg, DMX_INTR_FLAGS_DEFAULT);   // esp_dmx 3.1.1 signature
      }
      dmx_set_pin(MH_DMX_PORT, MH_DMX_TX_PIN, -1, -1);    // TX only (rx/rts unused)
      dmx_set_baud_rate(MH_DMX_PORT, DMX_BAUD_RATE);
      _dmxReady = dmx_driver_is_installed(MH_DMX_PORT);
      #endif
    }

    void loop() override {
      if (!enabled) return;
      unsigned long now = millis();
      uint16_t period = 1000 / (fps ? fps : 40);
      if (now - lastFrame < period) return;
      float dt = (now - lastFrame) / 1000.0f; if (dt > 0.1f) dt = 0.1f;
      lastFrame = now;

      // ---- colour: read the source segment's pixels into 3 ring colours ----
      uint32_t rc[3]; bool haveColor = readRings(rc);

      // ---- energy: size-weighted toward the dominant outer ring (1 : 5 : 12) ----
      float e = 0.0f;
      if (haveColor) e = (lum(rc[0])*1.0f + lum(rc[1])*5.0f + lum(rc[2])*12.0f) / 18.0f;
      e = constrain(e * (sensitivity / 64.0f), 0.0f, 1.0f);
      volEnv += (e - volEnv) * 0.25f;                      // smoothed energy envelope
      bool beat = (e > prevE + 0.12f) && (e > 0.35f);      // rising-edge "beat" from the colour energy
      prevE += (e - prevE) * 0.5f;                          // fast-tracking reference for edge detection
      if (beat) { accent = 1.0f; rndx = (hw_random8()/127.5f)-1.0f; rndy = (hw_random8()/127.5f)-1.0f; stepIdx++; }
      accent *= powf(0.05f, dt);                            // frame-rate-independent decay

      // ---- motion (parametric path; energy modulates amplitude + speed) ----
      float spd = (speed / 90.0f) * (1.0f + (audioDepth/255.0f) * volEnv);
      phase += spd * dt * 2.0f;
      float nx, ny; path(pattern, phase, nx, ny);
      float amp = (size/255.0f) * (1.0f + (audioDepth/255.0f) * volEnv) * (1.0f + 0.6f*accent);
      amp = constrain(amp, 0.0f, 1.0f);

      float panT  = panCenter  + (panRange  * 0.5f) * nx * amp;
      float tiltT = tiltCenter + (tiltRange * 0.5f) * ny * amp;
      panT  = constrain(panT,  panCenter  - panRange*0.5f,  panCenter  + panRange*0.5f);
      tiltT = constrain(tiltT, tiltCenter - tiltRange*0.5f, tiltCenter + tiltRange*0.5f);
      panT  = constrain(panT, 0.0f, 540.0f);
      tiltT = constrain(tiltT, 0.0f, 270.0f);
      float k = 1.0f - (smoothing / 270.0f); k = constrain(k, 0.02f, 1.0f);
      panS  += (panT  - panS)  * k;
      tiltS += (tiltT - tiltS) * k;

      // ---- write DMX ----
      ChMap m = chmap();
      // blank the whole fixture block first -> every channel we don't drive (onboard macros,
      // effects, animations, secondary, mode, diagnostics) stays 0 = direct manual control
      for (int off = 0; off < chMode; off++) wr((int8_t)off, 0);
      wr16(m.pan,  m.panF,  (uint16_t)(panS  / 540.0f * 65535.0f));
      wr16(m.tilt, m.tiltF, (uint16_t)(tiltS / 270.0f * 65535.0f));
      wr(m.ptSpd, ptSpeed);
      wr(m.dim, dimmerMode == 1 ? (uint8_t)(bri * (0.3f + 0.7f*volEnv)) : bri);
      wr(m.zoom, zoomReact ? (uint8_t)(zoomNarrow + (zoomWide - zoomNarrow) * volEnv) : zoomManual);
      wr(m.strobe, (strobeMode == 1 && accent > 0.85f) ? 200 : 0);

      // ---- rings: straight from the source pixels (central=ring0, mid=ring1, outer=ring2) ----
      uint8_t nRings = (chMode == 16) ? 1 : 3;
      for (uint8_t i = 0; i < nRings; i++) {
        uint32_t c = haveColor ? rc[i] : 0;
        uint8_t r=R(c), g=G(c), b=B(c), w=0;
        uint8_t pk = r>g ? (r>b?r:b) : (g>b?g:b);
        switch (whiteMode) {
          case 0:  w = 0; break;                                                  // Off - pure RGB, W stays 0
          case 2:  w = (uint16_t)whiteMix * pk / 255; break;                       // RGB->W - always synthesize
          default: w = W(c); if (w == 0 && whiteMix) w = (uint16_t)whiteMix * pk / 255; break; // Auto
        }
        wr(m.ring[i][0], r); wr(m.ring[i][1], g); wr(m.ring[i][2], b); wr(m.ring[i][3], w);
      }
      // (all unused fixture channels were already blanked to 0 above)

      #if defined(ARDUINO_ARCH_ESP32)
      if (_dmxReady) {                       // hardware-timed frame: real break + continuously driven line
        _pkt[0] = 0;                          // DMX start code
        dmx_write(MH_DMX_PORT, _pkt, 513);    // full universe (matches a standard controller)
        dmx_send(MH_DMX_PORT, 513);
        dmx_wait_sent(MH_DMX_PORT, pdMS_TO_TICKS(50));
      }
      #endif
    }

    // ---- DYNAMIC show controls -> live via the main-page "Moving Head" group + presets/API.
    //      Relative/unitless params are exchanged as a human-friendly 0..100 here and scaled to
    //      the internal 0..255 range, so the motion math is unchanged. (NOT in the usermod menu.) ----
    static inline uint8_t to100(uint8_t v)  { return (uint8_t)((v*100 + 127) / 255); }
    static inline uint8_t from100(int v)    { v = v<0?0:(v>100?100:v); return (uint8_t)((v*255 + 50) / 100); }
    void addToJsonState(JsonObject& root) override {
      JsonObject t = root.createNestedObject(F("MovingHead"));
      t[F("speed")]=to100(speed); t[F("size")]=to100(size); t[F("pattern")]=pattern;
      t[F("sens")]=to100(sensitivity); t[F("aDepth")]=to100(audioDepth); t[F("smooth")]=to100(smoothing);
      t[F("zReact")]=zoomReact; t[F("zMan")]=to100(zoomManual); t[F("strobe")]=strobeMode; t[F("dimmer")]=dimmerMode;
    }
    void readFromJsonState(JsonObject& root) override {
      JsonObject t = root[F("MovingHead")];
      if (t.isNull()) return;
      int v;
      if (getJsonValue(t[F("speed")],  v)) speed       = from100(v);
      if (getJsonValue(t[F("size")],   v)) size        = from100(v);
      if (getJsonValue(t[F("sens")],   v)) sensitivity = from100(v);
      if (getJsonValue(t[F("aDepth")], v)) audioDepth  = from100(v);
      if (getJsonValue(t[F("smooth")], v)) smoothing   = from100(v);
      if (getJsonValue(t[F("zMan")],   v)) zoomManual  = from100(v);
      getJsonValue(t[F("pattern")], pattern);   // 0..10 index (passthrough)
      getJsonValue(t[F("zReact")],  zoomReact); // toggles passthrough
      getJsonValue(t[F("strobe")],  strobeMode);
      getJsonValue(t[F("dimmer")],  dimmerMode);
    }

    // ---- STATIC setup -> Usermods menu only (pinout via build flags; everything else here) ----
    void addToConfig(JsonObject& root) override {
      JsonObject t = root.createNestedObject(F("MovingHead"));
      t[F("on")]=enabled;
      t[F("addr")]=dmxAddr; t[F("chMode")]=chMode; t[F("srcSeg")]=sourceSeg;
      t[F("panC")]=panCenter; t[F("panR")]=panRange; t[F("tiltC")]=tiltCenter; t[F("tiltR")]=tiltRange;
      t[F("zMin")]=zoomNarrow; t[F("zMax")]=zoomWide; t[F("ptSpd")]=ptSpeed;
      t[F("wMode")]=whiteMode; t[F("white")]=whiteMix; t[F("fps")]=fps;
    }
    bool readFromConfig(JsonObject& root) override {
      JsonObject t = root[F("MovingHead")];
      if (t.isNull()) return false;
      bool ok = true;
      ok &= getJsonValue(t[F("on")],enabled,true);
      ok &= getJsonValue(t[F("addr")],dmxAddr,MH_DEFAULT_ADDR);
      ok &= getJsonValue(t[F("chMode")],chMode,24);       ok &= getJsonValue(t[F("srcSeg")],sourceSeg,0);
      ok &= getJsonValue(t[F("panC")],panCenter,90);      ok &= getJsonValue(t[F("panR")],panRange,180);
      ok &= getJsonValue(t[F("tiltC")],tiltCenter,45);    ok &= getJsonValue(t[F("tiltR")],tiltRange,90);
      ok &= getJsonValue(t[F("zMin")],zoomNarrow,40);     ok &= getJsonValue(t[F("zMax")],zoomWide,220);
      ok &= getJsonValue(t[F("ptSpd")],ptSpeed,0);
      ok &= getJsonValue(t[F("wMode")],whiteMode,1);      ok &= getJsonValue(t[F("white")],whiteMix,128);
      ok &= getJsonValue(t[F("fps")],fps,40);
      return ok;
    }
    void appendConfigData(Print& uiScript) override {
      uiScript.print(F("ux='MovingHead';"));
      uiScript.print(F("dd=addDropdown(ux,'chMode');addOption(dd,'16-ch',16);addOption(dd,'24-ch (3 rings)',24);addOption(dd,'32-ch',32);"));
      uiScript.print(F("dd=addDropdown(ux,'wMode');addOption(dd,'W: Off (RGB only)',0);addOption(dd,'W: Auto (pixel, else from RGB)',1);addOption(dd,'W: from RGB',2);"));
      uiScript.print(F("addInfo(ux+':on',1,'enable the moving-head DMX output');"));
      uiScript.print(F("addInfo(ux+':addr',1,'DMX start channel of the fixture (1-512)');"));
      uiScript.print(F("addInfo(ux+':srcSeg',1,'WLED segment whose pixels colour the rings - run any audio effect on it');"));
      uiScript.print(F("addInfo(ux+':panC',1,'pan centre, degrees (0-540)');"));
      uiScript.print(F("addInfo(ux+':panR',1,'pan travel, degrees peak-to-peak (0-540)');"));
      uiScript.print(F("addInfo(ux+':tiltC',1,'tilt centre, degrees (0-270)');"));
      uiScript.print(F("addInfo(ux+':tiltR',1,'tilt travel, degrees peak-to-peak (0-270)');"));
      uiScript.print(F("addInfo(ux+':zMin',1,'zoom DMX value at narrowest beam (0-255)');"));
      uiScript.print(F("addInfo(ux+':zMax',1,'zoom DMX value at widest beam (0-255)');"));
      uiScript.print(F("addInfo(ux+':ptSpd',1,'fixture pan/tilt-speed channel, 0 = fastest (0-255)');"));
      uiScript.print(F("addInfo(ux+':white',1,'synthesized white amount from RGB peak (0-255), for W modes Auto / from RGB');"));
      uiScript.print(F("addInfo(ux+':fps',1,'DMX refresh rate, frames per second (10-44)');"));
    }

    uint16_t getId() override { return USERMOD_ID_UNSPECIFIED; }
};

static MovingHead movinghead;
REGISTER_USERMOD(movinghead);
