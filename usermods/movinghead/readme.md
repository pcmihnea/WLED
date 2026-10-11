# Moving Head (DMX) usermod

Drives one or more DMX moving heads from WLED: colours come from a WLED effect, movement and beam
react to the music (AudioReactive with the onset / tempo tracker in this fork).

## Quick start
1. Config → Usermods → **MovingHead**: set `addr` (DMX address), `profile` (UKing 16/24/32-ch or Custom),
   `heads`, and the safe motion box (`panC`/`panR`, `tiltC`/`tiltR` in degrees).
2. Make a small segment (e.g. 18 LEDs) and run any audio effect on it — its colours feed the head's rings
   (`srcSeg`, `ringMap`).
3. Open the **Mover** tab and pick a mode + pattern. Save a preset to recall the whole look.

## Mover tab (live, 0–10 sliders)
Saved in presets (save a preset to keep a look; set a boot preset to start with it).
The status line on top shows the tempo (audio or tap), the current pattern and drops. Section headers fold.
The screen's Mover page shows the same: beat-in-bar dots (orange = downbeat), AUD/TAP tempo source, a blinking DROP,
the dimmer / zoom / section energy bars (D Z E) and the box test progress.

| Section | Controls |
|---|---|
| Motion | **Mode** Auto / Beat / Sync / Hold · **Pattern** (30) + *Next pattern* · **Change pattern** every 4–32 bars or on drops, random or *in order* (patterns crossfade over 1.5 s; bars are phrase-aligned: counted from the drop, from the first hit after silence, or from your last tap, and every drop starts a new pattern) · **Speed** (equal beam speed for every pattern; a governor slows complex patterns down so the head can follow the whole figure; Sync lengthens their loop ×2/×4…) · **Size** · **Beats per loop** (Sync) · **Step every** ½ / 1 / 2 / 4 beats (Beat) · **Direction** forward / reverse / ping-pong · **Rotate** 0–180° · **Shape** flat ↔ tall · **Smoothing** · **Motor speed** (scales the speed limits) · **Aim** pan / tilt · **Freeze** |
| Audio & tempo | **React to** mix / bass / mids / highs / volume · **Sensitivity** · **Audio depth** · **Beat punch** (size) · **Beat nod** (tilt kick) · **Hit shake** (pan flick on strong hits) · **Drop reaction** (kick returns after ≥ 4 bars of breakdown; max one per 30 s) · **On silence** keep / drift / park · **Tempo** audio (tap as fallback) or tap only · **TAP** / Clear · **Timing nudge** ±200 ms · **Section energy** (loud parts of the song move bigger and faster, quiet parts calmer; 0 = off) |
| Beam | **Dimmer** full / audio / beat pulse / breathe / off / chase (one head per beat) / alternate · **Dimmer floor** · **Strobe** off / beat / strong hits / bar / drops · **Strobe speed** · **Zoom** manual (0 = wide, 10 = narrow) / audio / pulse / inverse · **Ring colours** segment / beat flash / chase / spectrum / solid / pulse / rainbow |
| Several heads (shown when `heads` ≥ 2) | **Spread** (phase fan; Hold: position fan) · **Mirror** off / pan / tilt / both |
| Audio input | AudioReactive on/off (part of presets) · ES8311 mic gain (3 dB steps) · **mic level meter** (raw 3 s peak in dBFS, CLIP warning; aim for −30..−6 dBFS) · auto gain · noise gate · beat sensitivity · onset peaks |
| Device | screen page (becomes the boot page) · backlight · **Box test** (head visits the centre and the 4 corners of the safe motion box, 2 s each) |

Audio input and Device are venue settings: they are not stored in presets; they are written to the usermod
setup (`cfg.json`, the same as pressing Save on the Usermods page) 5 s after the last change, so they survive reboots.

## Fixture setup (Usermods page)
- `lagMs`: fixture lag; Beat and Sync moves start this early so the beam lands on the beat (40–120 ms typical).
- `panSpd` / `tiltSpd`: speed limits in °/s — keep them below the fixture's real speed to avoid jerky catch-up.
- `panDeg` / `tiltDeg`: full travel, for fixtures other than 540°/270°.
- **Custom profile:** `chCount` channels per head; `chMap` lists the 1-based head channel of
  `pan,panFine,tilt,tiltFine,ptSpeed,dimmer,strobe,zoom,R1,G1,B1,W1,R2,G2,B2,W2,R3,G3,B3,W3` (0 = none).
  Example, generic 11-ch spot without zoom/rings: `chCount=11`, `chMap=1,2,3,4,5,6,7,0,8,9,10,11`.
- `fixed`: head channels held at a value, e.g. `6=255,11=8` (shutter open, colour wheel, gobo…).
- `heads` / `addrStep`: identical heads at `addr`, `addr+step`, … (step 0 = channel count).
- `strobeMin` / `strobeVal`: strobe channel value for the slowest / fastest flash (Mover tab *Strobe speed* picks in between).
- `fullFrame` off sends only up to the last head (shorter frames, up to 60 fps); on = 512 slots (most compatible).
- `parkOff`: when WLED is switched off, heads centre, stop and go dark.

## Section energy
Auto gain keeps the show lively at any volume, but it also makes a quiet verse and a loud chorus look the same.
Section energy uses the real microphone level (before auto gain, ~2 s average) and compares it with the loudest
and quietest parts heard in the last minute or so: near the top of that range (chorus, drop) moves get up to
1.35× bigger and 1.75× faster (Beat: longer steps), near the bottom 0.65× / 0.57×. Sync stays on the tempo; only
the size changes. Songs with less than ~2 dB of dynamics (heavily compressed) are left alone, and the range is
learnt again after a silence. The status line shows the current position as *energy %* (E bar on the screen).

## Box test
Mover tab → Device → **Box test**: centre, then the four corners of the safe motion box, then centre, 2 s each,
beam on and narrow (even with WLED off). Adjust `panC`/`panR`/`tiltC`/`tiltR` on the Usermods page, save, run it again.
