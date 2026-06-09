// AtomS3R (GC9107 128x128 IPS) WLED live debug/performance screen.
// M5GFX display (backlight + GC9107), double-buffered via a PSRAM canvas.
// Rig-specific: 2 strings x 71 px (SM16703), previewed one per row.
//
// Note: the AtomS3R has NO user RGB LED. The LP5562 drives only the LCD
// backlight (W); its R/G/B outputs are unconnected, so there is no status LED
// to control here. Backlight is handled by M5GFX.
//
// Button (GPIO41): toggles WLED system power (preserves brightness).
//
// Folder (WLED 0.15+/0.16 custom_usermods):
//   usermods/atoms3r_display/ { library.json (dep m5stack/M5GFX, libArchive:false),
//                               atoms3r_display.cpp }

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

#define REFRESH_MS       120
#define LINE_BUFFER_SIZE 28
#define DIM565           0x2104
#define BASECOL          TFT_WHITE

static M5GFX    display;
static M5Canvas canvas(&display);

extern int getSignalQuality(int rssi);

class Atoms3rDisplayUsermod : public Usermod {
  private:
    bool enabled = true, initDone = false, useCanvas = false;

    uint8_t  rotation     = 0;
    uint8_t  backlight    = 80;     // 0..100 -> M5GFX backlight
    uint16_t sleepSeconds = 0;
    bool     showPreview  = true;

    bool     displayOff = false, btnPrev = false;
    unsigned long lastFrame = 0, lastInteract = 0, lastBtn = 0;
    uint8_t  kBri=0, kMode=0, kPal=0, kSpeed=0, kInt=0;

    inline uint8_t blByte() const { return (uint16_t)backlight * 255 / 100; }
    inline lgfx::LovyanGFX* g() { return useCanvas ? (lgfx::LovyanGFX*)&canvas : (lgfx::LovyanGFX*)&display; }
    void wake() { if (displayOff) { display.setBrightness(blByte()); displayOff = false; } }

    static uint16_t fpsCol(int f) { return f<20 ? TFT_RED : f<40 ? TFT_ORANGE : TFT_GREEN; }
    static uint16_t sigCol(int q) { return q<10 ? TFT_RED : q<25 ? TFT_ORANGE : TFT_GREEN; }

  public:
    void setup() override {
      uint8_t pins[] = { ATOMS3R_LCD_MOSI, ATOMS3R_LCD_SCK, ATOMS3R_LCD_CS, ATOMS3R_LCD_DC, ATOMS3R_LCD_RST };
      for (uint8_t p : pins) PinManager::allocatePin(p, true, PinOwner::UM_Unspecified);
      PinManager::allocatePin(ATOMS3R_BTN_PIN, false, PinOwner::UM_Unspecified);
      pinMode(ATOMS3R_BTN_PIN, INPUT_PULLUP);

      if (!enabled) return;
      if (!display.begin()) { enabled = false; return; }
      display.setRotation(rotation);
      display.setBrightness(blByte());
      display.fillScreen(TFT_BLACK);

      canvas.setPsram(true);
      canvas.setColorDepth(16);
      useCanvas = (canvas.createSprite(display.width(), display.height()) != nullptr);

      lastInteract = millis();
      initDone = true;
    }

    void connected() override { lastInteract = millis(); }

    void loop() override {
      if (!enabled || !initDone) return;

      // front button -> WLED power toggle (brightness preserved by toggleOnOff)
      bool down = (digitalRead(ATOMS3R_BTN_PIN) == LOW);
      if (down && !btnPrev && millis()-lastBtn > 250) {
        lastBtn = millis(); wake(); lastInteract = millis();
        toggleOnOff(); stateUpdated(CALL_MODE_BUTTON);
      }
      btnPrev = down;

      if (millis() - lastFrame < REFRESH_MS) return;
      lastFrame = millis();

      Segment& seg = strip.getMainSegment();
      if (bri!=kBri || seg.speed!=kSpeed || seg.intensity!=kInt || seg.mode!=kMode || seg.palette!=kPal) {
        wake(); lastInteract = millis();
        kBri=bri; kMode=seg.mode; kPal=seg.palette; kSpeed=seg.speed; kInt=seg.intensity;
      }
      if (!displayOff && sleepSeconds && millis()-lastInteract > (unsigned long)sleepSeconds*1000) {
        display.setBrightness(0); displayOff = true;
      }
      if (displayOff) return;

      render(seg);
    }

  private:
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

    void render(Segment& seg) {
      lgfx::LovyanGFX* d = g();
      char nm[LINE_BUFFER_SIZE];
      d->fillScreen(TFT_BLACK);
      d->setTextSize(1);

      int fps = (int)strip.getFps();
      field (d, 2,        1, fpsCol(fps), "FPS: %d", fps);
      fieldR(d, d->width()-2, 1, BASECOL, "BRI: %d%%", (int)bri*100/255);

      unsigned used = BusManager::currentMilliamps(), cap = BusManager::ablMilliampsMax();
      bool thr = cap && used >= cap;
      if (cap) field(d, 2, 13, thr?TFT_RED:BASECOL, "PWR: %u/%umA", used, cap);
      else     field(d, 2, 13, BASECOL,             "PWR: %umA", used);

      uint32_t fh = ESP.getFreeHeap();
      field(d, 2, 25, fh<20480?TFT_RED:BASECOL, "HEAP: %u/%uK", (unsigned)(fh/1024), (unsigned)(ESP.getMinFreeHeap()/1024));
      field(d, 2, 37, BASECOL, "PSRAM: %u.%02uM",
            (unsigned)(ESP.getFreePsram()/1048576), (unsigned)((ESP.getFreePsram()%1048576)*100/1048576));

      unsigned long s = millis()/1000;
      if (apActive)            field(d, 2, 49, TFT_ORANGE, "NET: AP");
      else if (WLED_CONNECTED) field(d, 2, 49, sigCol(getSignalQuality(WiFi.RSSI())), "RSSI: %d", (int)WiFi.RSSI());
      else                     field(d, 2, 49, TFT_RED,    "NET: --");
      fieldR(d, d->width()-2, 49, BASECOL, "UP: %lu:%02lu", s/3600, (s/60)%60);

      d->drawFastHLine(0, 61, d->width(), DIM565);

      if (realtimeMode) {
        field(d, 2, 64, TFT_MAGENTA, "LIVE: %s", realtimeIP.toString().c_str());
      } else {
        extractModeName(seg.mode, JSON_mode_names, nm, sizeof(nm)-1);
        field(d, 2, 64, BASECOL, "FX: %s", nm);
      }
      extractModeName(seg.palette, JSON_palette_names, nm, sizeof(nm)-1);
      field(d, 2, 76, BASECOL, "PAL: %s", nm);

      if (showPreview) {
        d->drawFastHLine(0, 88, d->width(), DIM565);
        int nb = (int)BusManager::getNumBusses();
        int rows = nb < 2 ? (nb<1?1:nb) : 2;
        int top = 91, rh = ((d->height()-top)/rows) - 2;
        for (int r = 0; r < rows; r++) drawBusRow(d, r, top + r*(rh+2), rh);
      }

      if (useCanvas) canvas.pushSprite(0, 0);
    }

    void drawBusRow(lgfx::LovyanGFX* d, int idx, int y, int h) {
      if (h < 3) return;
      Bus* b = BusManager::getBus(idx);
      int W = d->width();
      if (!b || !b->getLength()) { d->drawRect(0, y, W, h, DIM565); return; }
      uint16_t start = b->getStart(), len = b->getLength();
      for (int x = 0; x < W; x++) {
        uint32_t c = strip.getPixelColor(start + (uint32_t)x * len / W);
        d->drawFastVLine(x, y, h, d->color888((c>>16)&0xFF, (c>>8)&0xFF, c&0xFF));
      }
      d->drawRect(0, y, W, h, DIM565);
      field(d, 2, y+1, BASECOL, "S%d", idx+1);
    }

  public:
    void addToJsonInfo(JsonObject& root) override {
      JsonObject u = root["u"]; if (u.isNull()) u = root.createNestedObject("u");
      JsonArray a = u.createNestedArray(F("AtomS3R Display"));
      a.add(enabled ? F("installed") : F("disabled"));
    }
    void addToConfig(JsonObject& root) override {
      JsonObject t = root.createNestedObject(F("AtomS3R-Display"));
      t[F("enabled")]=enabled; t[F("rotation")]=rotation; t[F("backlight")]=backlight;
      t[F("sleepSeconds")]=sleepSeconds; t[F("preview")]=showPreview;
    }
    bool readFromConfig(JsonObject& root) override {
      JsonObject t = root[F("AtomS3R-Display")];
      if (t.isNull()) return false;
      bool ok = true;
      ok &= getJsonValue(t[F("enabled")],enabled,true);
      ok &= getJsonValue(t[F("rotation")],rotation,0);
      ok &= getJsonValue(t[F("backlight")],backlight,80);
      ok &= getJsonValue(t[F("sleepSeconds")],sleepSeconds,0);
      ok &= getJsonValue(t[F("preview")],showPreview,true);
      if (initDone) { display.setRotation(rotation); display.setBrightness(blByte()); lastInteract = millis(); }
      return ok;
    }
    uint16_t getId() override { return USERMOD_ID_UNSPECIFIED; }
};

static Atoms3rDisplayUsermod atoms3r_display;
REGISTER_USERMOD(atoms3r_display);