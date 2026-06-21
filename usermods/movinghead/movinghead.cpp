// =============================================================================
//  movinghead - audioreactive DMX moving-head usermod
//  Designed for the UKing ZQ02253 (16/24/32-ch), 540deg pan / 270deg tilt,
//  16-bit pan/tilt, mechanical zoom, 3x RGBW rings.
//
//  Philosophy ("not spastic"):
//   * Motion = continuous parametric paths (circle / fig-8 / lissajous / sweep ...)
//     advanced by a phase accumulator -> inherently smooth.
//   * Audio (smoothed volume) modulates path AMPLITUDE and SPEED, not raw position.
//   * Beats trigger a decaying "accent" envelope used for colour/zoom/strobe pops,
//     never for position jumps.
//   * Final pan/tilt pass through an exponential low-pass -> hardware can't jerk.
//
//  DMX out: writes the fixture channels into WLED's global `dmx` (SparkFunDMX);
//  WLED's handleDMXOutput() transmits the frame. To avoid the LED->DMX mapping
//  fighting these channels, set DMX "Start LED" beyond your LED count (Settings ->
//  DMX Output) so it writes nothing, OR keep the fixture channels out of its range.
//
//  Folder: usermods/movinghead/ { library.json, movinghead.cpp }   custom_usermods += movinghead
// =============================================================================
#include "wled.h"
#include <math.h>

#ifndef MH_DEFAULT_ADDR
  #define MH_DEFAULT_ADDR 1
#endif

class MovingHead : public Usermod {
  private:
    // ---- config (all editable in the WLED GUI) ----
    bool     enabled       = true;
    uint16_t dmxAddr       = MH_DEFAULT_ADDR;  // DMX channel of the fixture's first channel (Pan)
    uint8_t  chMode        = 24;               // 16 / 24 / 32
    uint8_t  macro         = 2;                // 0 Manual 1 Chill 2 Pulse 3 Rave 4 Sweep 5 LockBeat 6 Auto
    uint8_t  pattern       = 0;                // motion path (Manual macro)
    uint8_t  colorMode     = 0;                // 0 Unison 1 Spread 2 PerBand 3 Chase 4 BeatPop 5 BeatStrobe
    uint8_t  palette       = 0;                // 0 Rainbow 1 Lava 2 Ocean 3 Forest 4 Sunset 5 Cyber 6 Warm
    uint16_t panCenter     = 90;               // deg (within 0..540)
    uint16_t panRange      = 180;              // deg peak-to-peak
    uint16_t tiltCenter    = 45;               // deg (within 0..270)
    uint16_t tiltRange     = 90;               // deg peak-to-peak
    uint8_t  speed         = 90;               // base motion speed
    uint8_t  size          = 200;              // base movement amplitude (0..255 -> 0..1)
    uint8_t  smoothing     = 180;              // motion low-pass (higher = smoother/slower)
    uint8_t  audioSize     = 150;              // how much volume grows the movement
    uint8_t  audioSpeed    = 90;               // how much volume speeds the movement
    uint8_t  sensitivity   = 64;               // audio input scaling
    uint8_t  zoomMode      = 1;                // 0 Off 1 Volume 2 Beat 3 Breathe
    uint8_t  zoomNarrow    = 40;               // DMX value for narrow beam
    uint8_t  zoomWide      = 220;              // DMX value for wide beam
    uint8_t  strobeMode    = 0;                // 0 Off 1 BeatHits
    uint8_t  dimmerMode    = 1;                // 0 Full 1 Volume
    uint8_t  whiteMix      = 0;                // RGBW white add (0 = pure RGB)
    uint8_t  ptSpeed       = 0;                // fixture Pan/Tilt-speed channel (0 = fastest tracking)
    uint8_t  fps           = 40;               // DMX update rate

    // ---- runtime state ----
    float    phase = 0.0f;       // motion phase (rad)
    float    cphase = 0.0f;      // colour phase
    float    panS = 0.0f, tiltS = 0.0f;   // smoothed pan/tilt (deg)
    float    accent = 0.0f;      // decaying beat accent 0..1
    float    volEnv = 0.0f;      // smoothed volume envelope 0..1
    float    rndx = 0.0f, rndy = 0.0f;    // random-ease target
    uint8_t  stepIdx = 0;        // beat-step index
    bool     beatPrev = false;
    uint8_t  beatHue = 0;        // BeatPop colour
    unsigned long lastFrame = 0, lastMacroSwap = 0;
    uint8_t  autoPattern = 0, autoColor = 0;
    bool     inited = false;

    // ---- channel map per mode (offset from dmxAddr; -1 = absent) ----
    struct ChMap { int8_t pan, panF, tilt, tiltF, ptSpd, dim, strobe, zoom; int8_t ring[3][4]; };
    ChMap chmap() const {
      if (chMode == 32) return ChMap{ 0,1,2,3, 4, 6, 7, 5,
              {{9,10,11,12},{13,14,15,16},{17,18,19,20}} };
      if (chMode == 16) return ChMap{ 0,12,1,13, 2, 3, 8, 9,
              {{4,5,6,7},{-1,-1,-1,-1},{-1,-1,-1,-1}} };
      return ChMap{ 0,20,1,21, 2, 3, 16, 17,    // 24-ch (default)
              {{4,5,6,7},{8,9,10,11},{12,13,14,15}} };
    }
    inline void wr(int8_t off, uint8_t v) {
      #ifdef WLED_ENABLE_DMX
      if (off >= 0) dmx.write(dmxAddr + off, v);
      #endif
    }
    inline void wr16(int8_t hi, int8_t lo, uint16_t v) { wr(hi, v >> 8); wr(lo, v & 0xFF); }

    // normalized parametric path -> (nx,ny) in [-1,1]
    void path(uint8_t p, float th, float &nx, float &ny) {
      switch (p) {
        case 1:  nx = sinf(th);       ny = sinf(2*th);            break; // figure-8
        case 2:  nx = sinf(3*th);     ny = sinf(2*th);            break; // lissajous 3:2
        case 3:  nx = sinf(5*th);     ny = sinf(4*th + 0.785f);   break; // lissajous 5:4
        case 4:  nx = sinf(th);       ny = 0.0f;                  break; // sweep H
        case 5:  nx = 0.0f;           ny = sinf(th);              break; // sweep V
        case 6:  nx = sinf(th);       ny = sinf(th);              break; // diagonal
        case 7:  { float r = fabsf(sinf(0.11f*th)); nx = r*cosf(th); ny = r*sinf(th); } break; // spiral
        case 8:  nx = sinf(th);       ny = 0.25f*sinf(2*th);      break; // sway
        case 9:  nx = rndx;           ny = rndy;                  break; // random-ease (target set on beats)
        case 10: { const float px[4]={-1,1,1,-1}, py[4]={-1,-1,1,1}; nx=px[stepIdx&3]; ny=py[stepIdx&3]; } break; // beat-step corners
        default: nx = cosf(th);       ny = sinf(th);              break; // circle
      }
    }

    // palette colour: index 0..255 -> RGB (hue-region "palettes", no deps)
    void palCol(uint8_t idx, uint8_t bri, uint8_t &r, uint8_t &g, uint8_t &b) {
      uint16_t hStart, hSpan; uint8_t sat = 255;
      switch (palette) {
        case 1: hStart = 64000; hSpan = 10000; break;             // Lava (red->orange->yellow)
        case 2: hStart = 28000; hSpan = 14000; break;             // Ocean (green->cyan->blue)
        case 3: hStart = 12000; hSpan = 12000; break;             // Forest (yellow->green)
        case 4: hStart = 60000; hSpan = 14000; break;             // Sunset (red->magenta->pink)
        case 5: hStart = 40000; hSpan = 26000; break;             // Cyber (blue->magenta->cyan)
        case 6: hStart = 6000;  hSpan = 4000; sat = 90; break;    // Warm white
        default: hStart = 0;    hSpan = 65535; break;             // Rainbow
      }
      uint16_t hue = hStart + (uint32_t)idx * hSpan / 255;
      byte rgb[4] = {0,0,0,0};
      colorHStoRGB(hue, sat, rgb);
      r = (rgb[0]*bri)/255; g = (rgb[1]*bri)/255; b = (rgb[2]*bri)/255;
    }

    void writeRing(const ChMap &m, uint8_t ring, uint8_t r, uint8_t g, uint8_t b) {
      uint8_t w = (uint16_t)whiteMix * ((r>g?(r>b?r:b):(g>b?g:b))) / 255;  // W from peak * mix
      wr(m.ring[ring][0], r); wr(m.ring[ring][1], g); wr(m.ring[ring][2], b); wr(m.ring[ring][3], w);
    }

    // map a macro to (pattern,color,zoom,strobe,speedMul,sizeMul); returns chosen pattern/color
    void applyMacro(uint8_t &pat, uint8_t &col, uint8_t &zm, uint8_t &sm, float &spdMul, float &szMul) {
      switch (macro) {
        case 1: pat=0;  col=1; zm=3; sm=0; spdMul=0.5f; szMul=0.8f; break;   // Chill
        case 2: pat=1;  col=4; zm=2; sm=0; spdMul=1.0f; szMul=1.0f; break;   // Pulse
        case 3: pat=2;  col=3; zm=1; sm=1; spdMul=1.7f; szMul=1.2f; break;   // Rave
        case 4: pat=4;  col=2; zm=0; sm=0; spdMul=1.0f; szMul=1.0f; break;   // Sweep
        case 5: pat=10; col=4; zm=2; sm=1; spdMul=1.0f; szMul=1.0f; break;   // LockBeat
        case 6: pat=autoPattern; col=autoColor; zm=1; sm=0; spdMul=1.0f; szMul=1.0f; break; // Auto
        default: pat=pattern; col=colorMode; zm=zoomMode; sm=strobeMode; spdMul=1.0f; szMul=1.0f; break;
      }
    }

  public:
    void setup() override {
      lastFrame = millis(); rndx = 0; rndy = 0; inited = true;
      #ifdef WLED_ENABLE_DMX
      dmx.initWrite(dmxAddr + 32);   // own the bus; only clock out up to the fixture's channels (fast frames)
      dmxOutputUsermod = enabled;    // built-in handleDMXOutput() stands down while we drive
      #endif
    }

    void loop() override {
      #ifdef WLED_ENABLE_DMX
      dmxOutputUsermod = enabled;    // own DMX only while enabled (built-in resumes if we turn off)
      #endif
      if (!enabled) return;
      unsigned long now = millis();
      uint16_t period = 1000 / (fps ? fps : 40);
      if (now - lastFrame < period) return;
      float dt = (now - lastFrame) / 1000.0f;
      if (dt > 0.1f) dt = 0.1f;
      lastFrame = now;

      // ---- audio ----
      float vol = 0.0f; uint8_t *fft = nullptr; bool beat = false;
      um_data_t *um;
      if (UsermodManager::getUMData(&um, USERMOD_ID_AUDIOREACTIVE)) {
        float volumeSmth = *(float*)um->u_data[0];
        fft  = (uint8_t*)um->u_data[2];
        beat = (*(uint8_t*)um->u_data[3]) != 0;
        vol  = constrain(volumeSmth / 255.0f * (sensitivity / 32.0f), 0.0f, 1.0f);
      }
      volEnv += (vol - volEnv) * 0.15f;                 // smooth envelope
      bool beatEdge = beat && !beatPrev; beatPrev = beat;
      if (beatEdge) {                                   // beat -> accents only (never position)
        accent = 1.0f;
        beatHue += 53;
        rndx = (hw_random8() / 127.5f) - 1.0f; rndy = (hw_random8() / 127.5f) - 1.0f;
        stepIdx++;
      }
      accent *= powf(0.05f, dt);                        // ~fast decay, frame-rate independent

      // Auto macro: rotate look every ~12s
      if (macro == 6 && now - lastMacroSwap > 12000) { lastMacroSwap = now; autoPattern = hw_random8(11); autoColor = hw_random8(5); }

      uint8_t pat, col, zm, sm; float spdMul, szMul;
      applyMacro(pat, col, zm, sm, spdMul, szMul);

      // ---- motion ----
      float spd = (speed / 90.0f) * spdMul * (1.0f + (audioSpeed/255.0f) * volEnv);   // rad/s-ish
      phase  += spd * dt * 2.0f;
      cphase += (0.15f + 0.6f*volEnv) * dt;
      float nx, ny; path(pat, phase, nx, ny);
      float amp = (size/255.0f) * szMul * (1.0f + (audioSize/255.0f) * volEnv) * (1.0f + 0.6f*accent);
      amp = constrain(amp, 0.0f, 1.0f);

      float panT  = panCenter  + (panRange  * 0.5f) * nx * amp;
      float tiltT = tiltCenter + (tiltRange * 0.5f) * ny * amp;
      panT  = constrain(panT,  panCenter  - panRange*0.5f,  panCenter  + panRange*0.5f);
      tiltT = constrain(tiltT, tiltCenter - tiltRange*0.5f, tiltCenter + tiltRange*0.5f);
      panT  = constrain(panT, 0.0f, 540.0f);
      tiltT = constrain(tiltT, 0.0f, 270.0f);
      float k = 1.0f - (smoothing / 270.0f);            // low-pass strength (smaller = smoother)
      k = constrain(k, 0.02f, 1.0f);
      panS  += (panT  - panS)  * k;
      tiltS += (tiltT - tiltS) * k;

      // ---- write DMX ----
      ChMap m = chmap();
      wr16(m.pan,  m.panF,  (uint16_t)(panS  / 540.0f * 65535.0f));
      wr16(m.tilt, m.tiltF, (uint16_t)(tiltS / 270.0f * 65535.0f));
      wr(m.ptSpd, ptSpeed);
      wr(m.dim, dimmerMode == 1 ? (uint8_t)(bri * (0.3f + 0.7f*volEnv)) : bri);  // master = WLED brightness (volume-pulsed in Volume mode)
      wr(m.zoom, zoomVal(zm));
      wr(m.strobe, (sm == 1 && accent > 0.85f && volEnv > 0.5f) ? 200 : 0);

      // ---- colour the 3 rings ----
      uint8_t nRings = (chMode == 16) ? 1 : 3;
      for (uint8_t i = 0; i < nRings; i++) {
        uint8_t r,g,b; uint8_t idx, bri = 255;
        switch (col) {
          case 1: idx = (uint8_t)(cphase*40) + i*85;          break;          // Spread
          case 2: { uint8_t lo=i*5, hi=lo+5, s=0; if (fft) for (uint8_t k2=lo;k2<hi;k2++) s+=fft[k2]/5;
                    idx = i*85 + (uint8_t)(cphase*20); bri = fft ? (40+s) : 255; } break;       // PerBand
          case 3: idx = (uint8_t)(cphase*60 + i*40);          break;          // Chase
          case 4: idx = beatHue + i*20; bri = (uint8_t)(60 + 195*(0.3f+0.7f*accent)); break;    // BeatPop
          case 5: idx = beatHue + i*30; bri = accent > 0.7f ? 255 : 0; break; // BeatStrobe
          default: idx = (uint8_t)(cphase*30);                break;          // Unison
        }
        palCol(idx, bri, r, g, b);
        writeRing(m, i, r, g, b);
      }
      // park the fixture's own macro/auto/mode channels so it obeys direct control
      if (chMode == 24)      { wr(18,0); wr(19,0); wr(23,0); }
      else if (chMode == 16) { wr(10,0); wr(11,0); wr(15,0); }
      else                   { wr(8,0); wr(21,0); wr(22,0); wr(24,0); wr(31,0); } // 32-ch: dimmer-mode/macro/effect/anim/mode

      #ifdef WLED_ENABLE_DMX
      dmx.update();   // transmit this frame ourselves (built-in path disabled via dmxOutputUsermod)
      #endif
    }

    uint8_t zoomVal(uint8_t zm) {
      switch (zm) {
        case 1: return zoomNarrow + (uint8_t)((zoomWide - zoomNarrow) * volEnv);            // Volume
        case 2: return zoomNarrow + (uint8_t)((zoomWide - zoomNarrow) * accent);            // Beat pulse
        case 3: return zoomNarrow + (uint8_t)((zoomWide - zoomNarrow) * (0.5f+0.5f*sinf(cphase))); // Breathe
        default: return (zoomNarrow + zoomWide) / 2;                                        // Off (mid)
      }
    }

    // ---- config (GUI) ----
    void addToConfig(JsonObject& root) override {
      JsonObject t = root.createNestedObject(F("MovingHead"));
      t[F("on")]=enabled; t[F("addr")]=dmxAddr; t[F("chMode")]=chMode; t[F("macro")]=macro;
      t[F("pattern")]=pattern; t[F("color")]=colorMode; t[F("palette")]=palette;
      t[F("panC")]=panCenter; t[F("panR")]=panRange; t[F("tiltC")]=tiltCenter; t[F("tiltR")]=tiltRange;
      t[F("speed")]=speed; t[F("size")]=size; t[F("smooth")]=smoothing;
      t[F("aSize")]=audioSize; t[F("aSpeed")]=audioSpeed; t[F("sens")]=sensitivity;
      t[F("zoom")]=zoomMode; t[F("zMin")]=zoomNarrow; t[F("zMax")]=zoomWide;
      t[F("strobe")]=strobeMode; t[F("dimmer")]=dimmerMode; t[F("white")]=whiteMix; t[F("ptSpd")]=ptSpeed; t[F("fps")]=fps;
    }
    bool readFromConfig(JsonObject& root) override {
      JsonObject t = root[F("MovingHead")];
      if (t.isNull()) return false;
      bool ok = true;
      ok &= getJsonValue(t[F("on")],enabled,true);     ok &= getJsonValue(t[F("addr")],dmxAddr,MH_DEFAULT_ADDR);
      ok &= getJsonValue(t[F("chMode")],chMode,24);    ok &= getJsonValue(t[F("macro")],macro,2);
      ok &= getJsonValue(t[F("pattern")],pattern,0);   ok &= getJsonValue(t[F("color")],colorMode,0);
      ok &= getJsonValue(t[F("palette")],palette,0);
      ok &= getJsonValue(t[F("panC")],panCenter,90);   ok &= getJsonValue(t[F("panR")],panRange,180);
      ok &= getJsonValue(t[F("tiltC")],tiltCenter,45); ok &= getJsonValue(t[F("tiltR")],tiltRange,90);
      ok &= getJsonValue(t[F("speed")],speed,90);      ok &= getJsonValue(t[F("size")],size,200);
      ok &= getJsonValue(t[F("smooth")],smoothing,180);
      ok &= getJsonValue(t[F("aSize")],audioSize,150); ok &= getJsonValue(t[F("aSpeed")],audioSpeed,90);
      ok &= getJsonValue(t[F("sens")],sensitivity,64);
      ok &= getJsonValue(t[F("zoom")],zoomMode,1);     ok &= getJsonValue(t[F("zMin")],zoomNarrow,40);
      ok &= getJsonValue(t[F("zMax")],zoomWide,220);
      ok &= getJsonValue(t[F("strobe")],strobeMode,0); ok &= getJsonValue(t[F("dimmer")],dimmerMode,1);
      ok &= getJsonValue(t[F("white")],whiteMix,0);    ok &= getJsonValue(t[F("ptSpd")],ptSpeed,0);
      ok &= getJsonValue(t[F("fps")],fps,40);
      return ok;
    }
    void appendConfigData(Print& uiScript) override {
      uiScript.print(F("ux='MovingHead';"));
      uiScript.print(F("dd=addDropdown(ux,'chMode');addOption(dd,'16-ch',16);addOption(dd,'24-ch (3 rings)',24);addOption(dd,'32-ch',32);"));
      uiScript.print(F("dd=addDropdown(ux,'macro');addOption(dd,'Manual',0);addOption(dd,'Chill',1);addOption(dd,'Pulse',2);addOption(dd,'Rave',3);addOption(dd,'Sweep',4);addOption(dd,'LockBeat',5);addOption(dd,'Auto',6);"));
      uiScript.print(F("dd=addDropdown(ux,'pattern');addOption(dd,'Circle',0);addOption(dd,'Figure-8',1);addOption(dd,'Lissajous 3:2',2);addOption(dd,'Lissajous 5:4',3);addOption(dd,'Sweep H',4);addOption(dd,'Sweep V',5);addOption(dd,'Diagonal',6);addOption(dd,'Spiral',7);addOption(dd,'Sway',8);addOption(dd,'Random-ease',9);addOption(dd,'Beat-step',10);"));
      uiScript.print(F("dd=addDropdown(ux,'color');addOption(dd,'Unison',0);addOption(dd,'Spread',1);addOption(dd,'Per-band',2);addOption(dd,'Chase',3);addOption(dd,'Beat-pop',4);addOption(dd,'Beat-strobe',5);"));
      uiScript.print(F("dd=addDropdown(ux,'palette');addOption(dd,'Rainbow',0);addOption(dd,'Lava',1);addOption(dd,'Ocean',2);addOption(dd,'Forest',3);addOption(dd,'Sunset',4);addOption(dd,'Cyber',5);addOption(dd,'Warm',6);"));
      uiScript.print(F("dd=addDropdown(ux,'zoom');addOption(dd,'Off',0);addOption(dd,'Volume',1);addOption(dd,'Beat',2);addOption(dd,'Breathe',3);"));
      uiScript.print(F("dd=addDropdown(ux,'strobe');addOption(dd,'Off',0);addOption(dd,'Beat hits',1);"));
      uiScript.print(F("dd=addDropdown(ux,'dimmer');addOption(dd,'Full',0);addOption(dd,'Volume',1);"));
      uiScript.print(F("addInfo(ux+':addr',1,'DMX channel of Pan (fixture start)');"));
      uiScript.print(F("addInfo(ux+':panR',1,'deg peak-to-peak (0-540 phys)');"));
      uiScript.print(F("addInfo(ux+':tiltR',1,'deg peak-to-peak (0-270 phys)');"));
      uiScript.print(F("addInfo(ux+':smooth',1,'higher = smoother motion');"));
    }

    uint16_t getId() override { return USERMOD_ID_UNSPECIFIED; }
};

static MovingHead movinghead;
REGISTER_USERMOD(movinghead);
