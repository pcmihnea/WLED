// AtomS3R (GC9107 128x128 IPS) WLED live screen - 3 pages:
//   STATUS : mic / fps / wifi / uptime / brightness / power / heap / psram, effect + palette,
//            live preview of the first two segments
//   AUDIO  : 16-band spectrum, volume, bass/mid/high energy, beat flash, BPM + lock confidence
//   MOVER  : moving-head pan/tilt XY plot with trail, ring colours, dimmer / zoom / section energy bars,
//            mode/pattern/BPM, beat-in-bar dots, tempo source, drop flash, box test progress
//
// Rendering: one 128x128 RGB565 canvas in internal (DMA-capable) RAM, pushed with SPI DMA while the
// main loop carries on (the SPI transaction is held open; only the display uses this bus). Falls back to
// a PSRAM canvas + blocking push if internal RAM is short. Animated pages run at ~25 fps, STATUS at 10 fps.
//
// Button (GPIO41): short press = WLED power toggle (brightness preserved), long press (>=0.6 s) = next page.
// While the screen is asleep, the first press only wakes it.
//
// Note: the AtomS3R has NO user RGB LED. The LP5562 drives only the LCD backlight (W).
//
// Folder (WLED 0.15+/0.16 custom_usermods):
//   usermods/atoms3r_display/ { library.json (dep m5stack/M5GFX, libArchive:false), atoms3r_display.cpp }

#include "wled.h"
#include <M5GFX.h>

#ifndef ATOMS3R_BTN_PIN
  #define ATOMS3R_BTN_PIN 41
#endif
#define ATOMS3R_LCD_MOSI 21
#define ATOMS3R_LCD_SCK  15
#define ATOMS3R_LCD_CS   14
#define ATOMS3R_LCD_DC   42
#define ATOMS3R_LCD_RST  48

#ifndef USERMOD_ID_MOVINGHEAD
  #define USERMOD_ID_MOVINGHEAD 59
#endif

#define STATUS_MS        100      // STATUS page refresh
#define ANIM_MS          40       // AUDIO / MOVER page refresh (25 fps)
#define LONGPRESS_MS     600
#define LINE_BUFFER_SIZE 28
#define DIM565           0x2104
#define GRID565          0x18E3
#define BASECOL          TFT_WHITE
#define NUM_PAGES        3
#define TRAIL_LEN        32

static M5GFX    display;
static M5Canvas canvas(&display);

extern int getSignalQuality(int rssi);

class Atoms3rDisplayUsermod : public Usermod {
  private:
    bool enabled = true, initDone = false, initTried = false, useCanvas = false, dmaHeld = false;

    uint8_t  rotation     = 0;
    uint8_t  backlight    = 80;     // 0..100 -> M5GFX backlight
    uint16_t sleepSeconds = 0;
    bool     showPreview  = true;
    uint8_t  startPage    = 0;
    uint8_t  page         = 0;

    bool     displayOff = false, btnDown = false, longFired = false;
    unsigned long cfgDirtyAt = 0;      // backlight / page changed from the Mover tab -> save cfg.json 5 s later
    // JSON state / settings arrive on the web server task: only record them here, the screen (SPI DMA + backlight
    // I2C) is touched from loop() only
    volatile int8_t reqPage = -1;
    volatile bool   reqBl = false, reqWake = false, reqCfg = false;
    unsigned long lastFrame = 0, lastInteract = 0, btnT0 = 0;
    uint8_t  kBri=0, kMode=0, kPal=0, kSpeed=0, kInt=0;

    // AUDIO page state
    uint32_t lastBeatSeen = 0; unsigned long beatFlashT = 0;
    uint8_t  peakHold[16] = {0}; unsigned long peakT = 0;
    // MOVER page state
    int16_t  trailX[TRAIL_LEN], trailY[TRAIL_LEN]; uint8_t trailPos = 0, trailN = 0;

    inline uint8_t blByte() const { return (uint16_t)backlight * 255 / 100; }
    inline lgfx::LovyanGFX* g() { return useCanvas ? (lgfx::LovyanGFX*)&canvas : (lgfx::LovyanGFX*)&display; }
    void wake() { if (displayOff && initDone && enabled) { display.setBrightness(blByte()); displayOff = false; } lastInteract = millis(); }

    static uint16_t fpsCol(int f) { return f<20 ? TFT_RED : f<40 ? TFT_ORANGE : TFT_GREEN; }
    static uint16_t sigCol(int q) { return q<10 ? TFT_RED : q<25 ? TFT_ORANGE : TFT_GREEN; }

    bool initDisplay() {
      initTried = true;
      if (!display.begin()) return false;
      display.setRotation(rotation);
      display.setBrightness(blByte());
      display.fillScreen(TFT_BLACK);

      canvas.setColorDepth(16);
      canvas.setPsram(false);                              // internal RAM -> pushSprite() uses SPI DMA
      useCanvas = (canvas.createSprite(display.width(), display.height()) != nullptr);
      bool internal = useCanvas;
      if (!useCanvas) {                                    // low internal heap: PSRAM canvas, blocking push
        canvas.setPsram(true);
        useCanvas = (canvas.createSprite(display.width(), display.height()) != nullptr);
      }
      if (useCanvas && internal) { display.startWrite(); dmaHeld = true; }   // keep the bus: pushes return immediately
      lastInteract = millis();
      return true;
    }

    void present() {
      if (useCanvas) canvas.pushSprite(0, 0);              // DMA when the canvas is in internal RAM
    }
    void beginDraw() {
      if (dmaHeld) display.waitDMA();                      // never draw into the buffer while it is being sent
    }

  public:
    void setup() override {
      uint8_t pins[] = { ATOMS3R_LCD_MOSI, ATOMS3R_LCD_SCK, ATOMS3R_LCD_CS, ATOMS3R_LCD_DC, ATOMS3R_LCD_RST };
      for (uint8_t p : pins) PinManager::allocatePin(p, true, PinOwner::UM_Unspecified);
      if (PinManager::allocatePin(ATOMS3R_BTN_PIN, false, PinOwner::UM_Unspecified)) pinMode(ATOMS3R_BTN_PIN, INPUT_PULLUP);
      page = startPage % NUM_PAGES;
      if (!enabled) return;
      initDone = initDisplay();
    }

    void connected() override { lastInteract = millis(); }

    void loop() override {
      if (cfgDirtyAt && millis() - cfgDirtyAt >= 5000) { cfgDirtyAt = 0; configNeedsWrite = true; }   // standard WLED config save
      if (!enabled) {                                      // disabled in Usermods: blank once
        if (initDone && !displayOff) { if (dmaHeld) display.waitDMA(); display.setBrightness(0); displayOff = true; }
        return;
      }
      if (!initDone) { if (!initTried) initDone = initDisplay(); return; }   // enabled at runtime

      applyRequests();
      handleButton41();

      unsigned long now = millis();
      Segment& seg = strip.getMainSegment();
      if (bri!=kBri || seg.speed!=kSpeed || seg.intensity!=kInt || seg.mode!=kMode || seg.palette!=kPal) {
        wake();
        kBri=bri; kMode=seg.mode; kPal=seg.palette; kSpeed=seg.speed; kInt=seg.intensity;
      }
      if (!displayOff && sleepSeconds && now - lastInteract > (unsigned long)sleepSeconds*1000) {
        display.setBrightness(0); displayOff = true;
      }
      if (displayOff) return;

      if (now - lastFrame < (page == 0 ? STATUS_MS : ANIM_MS)) return;
      lastFrame = now;

      beginDraw();
      lgfx::LovyanGFX* d = g();
      d->fillScreen(TFT_BLACK);
      d->setTextSize(1);
      switch (page) {
        case 1:  renderAudio(d);  break;
        case 2:  renderMover(d);  break;
        default: renderStatus(d, seg); break;
      }
      present();
    }

  private:
    // apply what readFromJsonState() / readFromConfig() asked for (main loop only)
    void applyRequests() {
      if (reqCfg) {
        reqCfg = false;
        if (dmaHeld) display.waitDMA();                    // no panel commands while a frame is on the bus
        display.setRotation(rotation);
        display.setBrightness(blByte());
        displayOff = false; lastInteract = millis(); lastFrame = 0;
      }
      int8_t p = reqPage;
      if (p >= 0) { reqPage = -1; page = p % NUM_PAGES; trailN = 0; lastFrame = 0; }
      if (reqBl)   { reqBl = false; if (!displayOff) display.setBrightness(blByte()); }
      if (reqWake) { reqWake = false; wake(); }
    }

    // short press (on release) = power toggle, long press (while held) = next page
    void handleButton41() {
      bool down = (digitalRead(ATOMS3R_BTN_PIN) == LOW);
      unsigned long now = millis();
      if (down && !btnDown) {                              // pressed
        if (now - btnT0 < 40) return;                      // debounce
        btnDown = true; btnT0 = now; longFired = false;
      } else if (down && btnDown) {
        if (!longFired && now - btnT0 >= LONGPRESS_MS) {   // long press
          longFired = true;
          bool wasOff = displayOff; wake();
          if (!wasOff) { page = (page + 1) % NUM_PAGES; trailN = 0; lastFrame = 0; }
        }
      } else if (!down && btnDown) {                       // released
        btnDown = false;
        if (now - btnT0 < 40) return;                      // bounce
        if (!longFired) {
          bool wasOff = displayOff; wake();
          if (!wasOff) { toggleOnOff(); stateUpdated(CALL_MODE_BUTTON); }
        }
        btnT0 = now;
      }
    }

    void field(lgfx::LovyanGFX* d, int x, int y, uint16_t col, const char* fmt, ...) {
      char buf[LINE_BUFFER_SIZE];
      va_list ap; va_start(ap, fmt); vsnprintf(buf, sizeof(buf), fmt, ap); va_end(ap);
      d->setTextColor(col); d->setTextDatum(TL_DATUM); d->drawString(buf, x, y);
    }
    void fieldR(lgfx::LovyanGFX* d, int x, int y, uint16_t col, const char* fmt, ...) {
      char buf[LINE_BUFFER_SIZE];
      va_list ap; va_start(ap, fmt); vsnprintf(buf, sizeof(buf), fmt, ap); va_end(ap);
      d->setTextColor(col); d->setTextDatum(TR_DATUM); d->drawString(buf, x, y);
    }
    void pageDots(lgfx::LovyanGFX* d) {
      for (int i = 0; i < NUM_PAGES; i++)
        d->fillCircle(d->width() - 4 - (NUM_PAGES - 1 - i) * 6, 4, 2, i == page ? TFT_WHITE : DIM565);
    }
    static uint16_t c565(lgfx::LovyanGFX* d, uint32_t c) {        // WLED RGBW -> 565 (W folded in)
      uint16_t w = (c >> 24) & 0xFF;
      uint16_t r = min(255, (int)((c >> 16) & 0xFF) + w), gg = min(255, (int)((c >> 8) & 0xFF) + w), b = min(255, (int)(c & 0xFF) + w);
      return d->color565(r, gg, b);
    }

    // ------------------------------------------------------------------ STATUS
    void renderStatus(lgfx::LovyanGFX* d, Segment& seg) {
      char nm[LINE_BUFFER_SIZE];
      //   row1: MIC | FPS    row2: RSSI | UP    row3: BRI | PWR    row4: HEAP | PSRAM
      const int RW = d->width()-2;
      {
        um_data_t* um = nullptr;   // getUMData() returns false when AR is off OR codec-faulted -> "MIC --"
        if (UsermodManager::getUMData(&um, USERMOD_ID_AUDIOREACTIVE) && um) {
          int pct = (int)(*(float*)um->u_data[0] / 255.0f * 100.0f);
          pct = pct < 0 ? 0 : (pct > 100 ? 100 : pct);
          field(d, 2, 1, pct > 2 ? TFT_GREEN : BASECOL, "MIC: %d%%", pct);
        } else {
          field(d, 2, 1, TFT_RED, "MIC: --");
        }
      }
      int fps = (int)strip.getFps();
      fieldR(d, RW, 1, fpsCol(fps), "FPS: %d", fps);

      unsigned long s = millis()/1000;
      if (apActive)            field(d, 2, 13, TFT_ORANGE, "NET: AP");
      else if (WLED_CONNECTED) { int rssi = WiFi.RSSI(); field(d, 2, 13, sigCol(getSignalQuality(rssi)), "RSSI: %d", rssi); }
      else                     field(d, 2, 13, TFT_RED,    "NET: --");
      fieldR(d, RW, 13, BASECOL, "UP: %lu:%02lu", s/3600, (s/60)%60);

      field(d, 2, 25, BASECOL, "BRI: %d%%", (int)bri*100/255);
      unsigned used = BusManager::currentMilliamps(), cap = BusManager::ablMilliampsMax();
      bool thr = cap && used >= cap;
      fieldR(d, RW, 25, thr?TFT_RED:BASECOL, "PWR:%umA", used);

      uint32_t fh = ESP.getFreeHeap(), fp = ESP.getFreePsram();
      field(d, 2, 37, fh<20480?TFT_RED:BASECOL, "HEAP:%uK", (unsigned)(fh/1024));
      fieldR(d, RW, 37, BASECOL, "PSR:%u.%01uM", (unsigned)(fp/1048576), (unsigned)((fp%1048576)*10/1048576));

      d->drawFastHLine(0, 49, d->width(), DIM565);

      if (realtimeMode) {
        field(d, 2, 52, TFT_MAGENTA, "LIVE: %s", realtimeIP.toString().c_str());
      } else {
        extractModeName(seg.mode, JSON_mode_names, nm, sizeof(nm)-1);
        field(d, 2, 52, BASECOL, "FX: %s", nm);
      }
      extractModeName(seg.palette, JSON_palette_names, nm, sizeof(nm)-1);
      field(d, 2, 64, BASECOL, "PAL: %s", nm);

      if (showPreview) {
        d->drawFastHLine(0, 76, d->width(), DIM565);
        int top = 79, rh = ((d->height()-top)/2) - 2;   // fixed 2-row height (single seg keeps same size)
        int shown = 0;
        for (int i = 0; i < (int)strip.getSegmentsNum() && shown < 2; i++) {
          Segment& sg = strip.getSegment(i);
          if (!sg.isActive()) continue;
          drawSegRow(d, sg, i, top + shown*(rh+2), rh);
          shown++;
        }
      }
    }

    // one row = one segment: draw exactly seg.length() cells tiled across full width
    void drawSegRow(lgfx::LovyanGFX* d, Segment& seg, int id, int y, int h) {
      if (h < 3) return;
      int W = d->width();
      uint16_t len = seg.length();
      if (!len) { d->drawRect(0, y, W, h, DIM565); return; }
      for (uint16_t i = 0; i < len; i++) {
        int x0 = (int)((uint32_t)i * W / len);
        int x1 = (int)((uint32_t)(i + 1) * W / len);
        if (x1 <= x0) continue;                       // sub-pixel cell, skip
        uint32_t c = seg.getPixelColor(i);
        d->fillRect(x0, y, x1 - x0, h, c565(d, c));
      }
      d->drawRect(0, y, W, h, DIM565);
      field(d, 2, y+1, BASECOL, "S%d", id+1);
    }

    // ------------------------------------------------------------------ AUDIO
    void renderAudio(lgfx::LovyanGFX* d) {
      const int W = d->width(), H = d->height();
      field(d, 2, 1, TFT_CYAN, "AUDIO");
      pageDots(d);
      um_data_t* um = nullptr;
      if (!UsermodManager::getUMData(&um, USERMOD_ID_AUDIOREACTIVE) || !um) {
        d->setTextDatum(MC_DATUM); d->setTextColor(TFT_RED); d->drawString("no audio", W/2, H/2);
        return;
      }
      unsigned long now = millis();
      float vol = *(float*)um->u_data[0];
      uint8_t* fft = (uint8_t*)um->u_data[2];
      bool tracker = um->u_size >= 15;

      // 16-band spectrum with falling peak markers
      const int top = 14, bot = 88, bw = 7, gap = 1, x0 = (W - 16*(bw+gap) + gap) / 2;
      bool decay = now - peakT > 30; if (decay) peakT = now;
      for (int i = 0; i < 16; i++) {
        int h = (fft[i] * (bot - top)) / 255;
        int x = x0 + i*(bw+gap);
        uint16_t col = d->color565(40 + i*13, 220 - i*10, 255 - i*12);
        if (h > 0) d->fillRect(x, bot - h, bw, h, col);
        if (fft[i] >= peakHold[i]) peakHold[i] = fft[i]; else if (decay && peakHold[i] > 3) peakHold[i] -= 3;
        int py = bot - (peakHold[i] * (bot - top)) / 255;
        d->drawFastHLine(x, py, bw, TFT_WHITE);
      }
      d->drawFastHLine(0, bot + 1, W, DIM565);

      // volume bar
      int vw = (int)(constrain(vol, 0.0f, 255.0f) * (W - 4) / 255.0f);
      d->fillRect(2, bot + 4, vw, 4, TFT_GREEN);
      d->drawRect(1, bot + 3, W - 2, 6, DIM565);

      if (tracker) {
        uint32_t beats = *(uint32_t*)um->u_data[8];
        float bpm  = *(float*)um->u_data[9];
        float conf = *(float*)um->u_data[10];
        float* bands = (float*)um->u_data[14];
        if (beats != lastBeatSeen) { lastBeatSeen = beats; beatFlashT = now; }
        // beat lamp
        unsigned long since = now - beatFlashT;
        uint8_t lv = since < 150 ? (uint8_t)(255 - since * 255 / 150) : 0;
        d->fillCircle(10, H - 13, 8, d->color565(lv, lv/3, 0));
        d->drawCircle(10, H - 13, 8, DIM565);
        // BPM + confidence
        bool locked = bpm > 0.0f && conf >= 0.3f;
        if (locked) fieldR(d, 76, H - 22, TFT_WHITE, "%d BPM", (int)(bpm + 0.5f));
        else        fieldR(d, 76, H - 22, DIM565 | 0x4208, "-- BPM");
        int cw = (int)(constrain(conf, 0.0f, 1.0f) * 54);
        d->fillRect(22, H - 10, cw, 4, locked ? TFT_GREEN : TFT_ORANGE);
        d->drawRect(21, H - 11, 56, 6, DIM565);
        // bass / mid / high energies
        const char* lbl[3] = {"B", "M", "H"};
        for (int b = 0; b < 3; b++) {
          int x = 84 + b * 15, h = (int)(constrain(bands[b], 0.0f, 1.0f) * 22);
          d->fillRect(x, H - 9 - h, 10, h, b == 0 ? TFT_RED : (b == 1 ? TFT_YELLOW : TFT_CYAN));
          d->setTextColor(DIM565 | 0x4208); d->setTextDatum(TC_DATUM); d->drawString(lbl[b], x + 5, H - 8);
        }
      } else {
        field(d, 2, H - 12, DIM565 | 0x4208, "beat tracker n/a");
      }
    }

    // ------------------------------------------------------------------ MOVER
    void renderMover(lgfx::LovyanGFX* d) {
      static const char* modeNames[4] = {"AUTO", "BEAT", "SYNC", "HOLD"};
      const int W = d->width(), H = d->height();
      pageDots(d);
      um_data_t* mh = nullptr;
      if (!UsermodManager::getUMData(&mh, USERMOD_ID_MOVINGHEAD) || !mh || mh->u_size < 11) {
        field(d, 2, 1, TFT_ORANGE, "MOVER");
        d->setTextDatum(MC_DATUM); d->setTextColor(TFT_RED); d->drawString("mover off", W/2, H/2);
        return;
      }
      float pan = *(float*)mh->u_data[0], tilt = *(float*)mh->u_data[1];
      uint8_t dim = *(uint8_t*)mh->u_data[2], zoom = *(uint8_t*)mh->u_data[3];
      uint32_t* rings = (uint32_t*)mh->u_data[4];
      uint8_t mode = *(uint8_t*)mh->u_data[5], locked = *(uint8_t*)mh->u_data[6];
      float* lim = (float*)mh->u_data[7];
      float bpm = *(float*)mh->u_data[8], acc = *(float*)mh->u_data[9];
      const char* pat = *(const char**)mh->u_data[10];

      uint8_t box = mh->u_size >= 18 ? *(uint8_t*)mh->u_data[16] : 0;
      if (box) field(d, 2, 1, TFT_MAGENTA, "BOX %u/6", box);                 // box test running
      else     field(d, 2, 1, TFT_ORANGE, "%s", modeNames[mode < 4 ? mode : 0]);
      if (!box) field(d, 32, 1, BASECOL, "%s", pat ? pat : "");

      // XY box: pan (x) / tilt (y) within the configured limits
      const int bx = 2, by = 13, bwid = 98, bht = 80;
      d->drawRect(bx, by, bwid, bht, DIM565);
      d->drawFastHLine(bx + 1, by + bht/2, bwid - 2, GRID565);
      d->drawFastVLine(bx + bwid/2, by + 1, bht - 2, GRID565);
      float pr = max(1.0f, lim[1] - lim[0]), tr = max(1.0f, lim[3] - lim[2]);
      int px = bx + 2 + (int)((pan  - lim[0]) / pr * (bwid - 5));
      int py = by + bht - 3 - (int)((tilt - lim[2]) / tr * (bht - 5));
      px = constrain(px, bx + 2, bx + bwid - 3); py = constrain(py, by + 2, by + bht - 3);
      trailX[trailPos] = px; trailY[trailPos] = py; trailPos = (trailPos + 1) % TRAIL_LEN;
      if (trailN < TRAIL_LEN) trailN++;
      for (int i = 1; i < trailN; i++) {                    // oldest -> newest, fading in
        int a = (trailPos + TRAIL_LEN - trailN + i - 1) % TRAIL_LEN, b = (a + 1) % TRAIL_LEN;
        uint8_t lv = 30 + (i * 160) / trailN;
        d->drawLine(trailX[a], trailY[a], trailX[b], trailY[b], d->color565(lv/2, lv/2, lv));
      }
      // other heads (multi-head rigs): small dots
      if (mh->u_size >= 13) {
        uint8_t nF = *(uint8_t*)mh->u_data[11];
        float* pos = (float*)mh->u_data[12];
        for (int f = 1; f < nF && f < 8; f++) {
          int qx = bx + 2 + (int)((pos[2*f]   - lim[0]) / pr * (bwid - 5));
          int qy = by + bht - 3 - (int)((pos[2*f+1] - lim[2]) / tr * (bht - 5));
          d->fillCircle(constrain(qx, bx + 2, bx + bwid - 3), constrain(qy, by + 2, by + bht - 3), 2, TFT_DARKGREY);
        }
      }
      uint16_t beamCol = (rings[2] & 0xFFFFFFu) || (rings[2] >> 24) ? c565(d, rings[2]) : c565(d, rings[0]);
      if (dim == 0) beamCol = DIM565;
      int rad = 3 + (int)(acc * 3);
      d->fillCircle(px, py, rad, beamCol);
      d->drawCircle(px, py, rad + 1, TFT_WHITE);

      // tempo strip under the box: beat-in-bar dots (downbeat orange), tempo source, drop flash
      if (mh->u_size >= 16) {
        uint8_t src = *(uint8_t*)mh->u_data[13], dropF = *(uint8_t*)mh->u_data[14], bib = *(uint8_t*)mh->u_data[15];
        const int ty = by + bht + 7;
        for (int i = 0; i < 4; i++) {
          int cx = 7 + i * 10;
          if (bib == i) d->fillCircle(cx, ty, 3, i == 0 ? TFT_ORANGE : TFT_WHITE);
          else          d->drawCircle(cx, ty, 3, bib == 255 ? GRID565 : DIM565);
        }
        if (locked) field(d, 46, ty - 4, src ? TFT_MAGENTA : TFT_CYAN, src ? "TAP" : "AUD");
        if (dropF) {
          bool on = (millis() / 100) & 1;                     // 5 Hz blink
          d->drawRect(bx - 1, by - 1, bwid + 2, bht + 2, on ? TFT_RED : TFT_WHITE);
          d->drawRect(bx, by, bwid, bht, on ? TFT_RED : TFT_WHITE);
          field(d, 70, ty - 4, on ? TFT_RED : TFT_WHITE, "DROP");
        }
      }

      // dimmer / zoom / section energy bars
      const int barTop = by, barH = bht;
      uint8_t en = mh->u_size >= 18 ? *(uint8_t*)mh->u_data[17] : 255;
      const int lvl[3] = { dim * (barH - 2) / 255, zoom * (barH - 2) / 255, en <= 100 ? en * (barH - 2) / 100 : 0 };
      const uint16_t bc[3] = { TFT_YELLOW, TFT_CYAN, TFT_MAGENTA };
      for (int i = 0; i < 3; i++) {
        const int x = 103 + i * 8;
        d->drawRect(x, barTop, 7, barH, DIM565);
        if (lvl[i] > 0) d->fillRect(x + 1, barTop + barH - 1 - lvl[i], 5, lvl[i], bc[i]);
      }

      // ring colours (centre / mid / outer) + pan/tilt readout + tempo
      const int ry = H - 16;
      for (int i = 0; i < 3; i++) {
        d->fillCircle(10 + i * 18, ry + 6, 3 + i * 2, c565(d, rings[i]));
        d->drawCircle(10 + i * 18, ry + 6, 3 + i * 2, DIM565);
      }
      field(d, 60, ry - 2, BASECOL, "P%3d T%3d", (int)(pan + 0.5f), (int)(tilt + 0.5f));
      if (locked) field(d, 60, ry + 8, TFT_GREEN, "%d BPM", (int)(bpm + 0.5f));
      else        field(d, 60, ry + 8, DIM565 | 0x4208, "free");
      d->setTextColor(DIM565 | 0x4208); d->setTextDatum(TC_DATUM);
      d->drawString("D", 106, by + bht + 1); d->drawString("Z", 114, by + bht + 1); d->drawString("E", 122, by + bht + 1);
    }

  public:
    // live control from the UI (Mover tab "Device"): screen page + backlight. Not in presets; written to the
    // usermod config (backlight, page = boot page) 5 s after the last change.
    void addToJsonState(JsonObject& root) override {
      if (umStateForPreset) return;
      JsonObject t = root.createNestedObject(F("Display"));
      t[F("page")] = page;
      t[F("bl")] = (backlight + 5) / 10;          // 0..10
      t[F("on")] = !displayOff;
    }
    void readFromJsonState(JsonObject& root) override {
      JsonObject t = root[F("Display")];
      if (t.isNull()) return;
      int v;
      if (getJsonValue(t[F("page")], v)) {
        v = constrain(v, 0, NUM_PAGES - 1);
        reqPage = v; reqWake = true;
        if (v != startPage) { startPage = v; cfgDirtyAt = millis() | 1; }
      }
      if (getJsonValue(t[F("bl")], v)) {
        v = constrain(v, 0, 10) * 10;
        if (v != backlight) { backlight = v; reqBl = true; cfgDirtyAt = millis() | 1; }
        reqWake = true;
      }
    }

    void addToJsonInfo(JsonObject& root) override {
      JsonObject u = root["u"]; if (u.isNull()) u = root.createNestedObject("u");
      JsonArray a = u.createNestedArray(F("AtomS3R Display"));
      if (!enabled) a.add(F("disabled"));
      else if (!initDone) a.add(F("init failed"));
      else a.add(dmaHeld ? F("DMA") : (useCanvas ? F("PSRAM canvas") : F("direct")));
    }
    void addToConfig(JsonObject& root) override {
      JsonObject t = root.createNestedObject(F("AtomS3R-Display"));
      t[F("enabled")]=enabled; t[F("rotation")]=rotation; t[F("backlight")]=backlight;
      t[F("sleepSeconds")]=sleepSeconds; t[F("preview")]=showPreview; t[F("page")]=startPage;
    }
    bool readFromConfig(JsonObject& root) override {
      JsonObject t = root[F("AtomS3R-Display")];
      if (t.isNull()) return false;
      bool ok = true;
      bool wasEnabled = enabled;
      ok &= getJsonValue(t[F("enabled")],enabled,true);
      ok &= getJsonValue(t[F("rotation")],rotation,0);
      ok &= getJsonValue(t[F("backlight")],backlight,80);
      ok &= getJsonValue(t[F("sleepSeconds")],sleepSeconds,0);
      ok &= getJsonValue(t[F("preview")],showPreview,true);
      ok &= getJsonValue(t[F("page")],startPage,0);
      rotation &= 3; backlight = min(backlight, (uint8_t)100); startPage %= NUM_PAGES;
      if (initDone) {
        if (enabled) { reqCfg = true; if (!wasEnabled) reqPage = startPage; }   // applied by loop() (disable: loop() blanks it)
      } else if (enabled && !wasEnabled) initTried = false;   // allow a (re)try from loop()
      return ok;
    }
    void appendConfigData(Print& uiScript) override {
      uiScript.print(F("ux='AtomS3R-Display';"));
      uiScript.print(F("dd=addDropdown(ux,'rotation');addOption(dd,'0',0);addOption(dd,'90',1);addOption(dd,'180',2);addOption(dd,'270',3);"));
      uiScript.print(F("dd=addDropdown(ux,'page');addOption(dd,'Status',0);addOption(dd,'Audio',1);addOption(dd,'Mover',2);"));
      uiScript.print(F("addInfo(ux+':backlight',1,'%');"));
      uiScript.print(F("addInfo(ux+':sleepSeconds',1,'s, 0 = always on');"));
      uiScript.print(F("addInfo(ux+':page',1,'page at boot - long-press the button to cycle');"));
    }
    uint16_t getId() override { return USERMOD_ID_UNSPECIFIED; }
};

static Atoms3rDisplayUsermod atoms3r_display;
REGISTER_USERMOD(atoms3r_display);
