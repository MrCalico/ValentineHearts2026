## Purpose
Help AI coding agents be immediately productive in this repository: an ESP32/PlatformIO project that animates WS2812 LEDs and plays sounds from a DFPlayer Mini over UART.

Keep guidance short and actionable. When in doubt, change only `src/main.cpp` behavior in a conservative way and prefer preserving existing wiring, pin choices, and timing constants.

## Big picture
- Single microcontroller target (ESP32) using PlatformIO. Build config: `platformio.ini` in repo root.
- Main application is a single-file program in `src/main.cpp` that:
  - drives a long WS2812 LED strip (NUM_LEDS = 1088) using FastLED
  - controls a DFPlayer Mini via HardwareSerial (UART2) to play SD card tracks
  - reads two ADC pots for volume and speed control (GPIO34/35)
  - uses a BUSY pin (GPIO27) to monitor DFPlayer playback state

## Key files & locations
- `src/main.cpp` — primary implementation; contains all behavior and hardware pin mappings. Use this file as the canonical source for wiring, pin choices, and timing.
- `platformio.ini` — build/board settings (target platform, envs). Always consult it before changing build-related code.
- `assets/mp3/` — audio files expected on the DFPlayer Mini SD card; filenames are numeric (e.g. `0011.mp3`) and tracks are referenced by numeric index in code (e.g. `QueueTrack(11)` plays track 11).
- `.vscode/extensions.json` — recommended editor extensions (non-blocking) for local dev.

## Build / Flash / Debug workflows
- Use PlatformIO commands in VS Code or the terminal from repo root. Typical commands:
  ## Purpose
  Concise, actionable instructions to make AI coding agents productive in this ESP32 + PlatformIO project. The app drives WS2812 LEDs with FastLED and plays MP3s from a DFPlayer Mini.

  When unsure, make small, conservative edits to `src/main.cpp`. If you add modules or libraries, place them under `lib/` or `include/` and update `platformio.ini`.

  ## Big picture
  - Platform: ESP32 (Arduino framework) via PlatformIO. Config lives in `platformio.ini`.
  - App: single main program at `src/main.cpp`. Responsibilities:
    - LED animation with FastLED (primary strip and a small "house" strip)
    - DFPlayer Mini control over UART2 (HardwareSerial(2))
    - Two ADC pots for volume and speed (GPIO34 and GPIO35)
    - Playback state polled using BUSY pin (GPIO27)
  ## Big picture
  - Platform: ESP32 (Arduino framework) via PlatformIO. Config lives in `platformio.ini`.
  - App: single main program at `src/main.cpp`. Responsibilities:
    - LED animation with FastLED for a Valentine display: a large heart, 4 concentric heart levels, and an arrow accent (geometry mapped in `src/main.cpp`).
    - DFPlayer Mini control over UART2 (HardwareSerial(2)) for short voice or music cues.
    - Two ADC pots for volume and animation speed (GPIO34 and GPIO35).
    - Playback state polled using BUSY pin (GPIO27).

  ## Key files
  - `src/main.cpp` — canonical wiring, timing constants, and behavior. Read this first for any change.
  - `platformio.ini` — build/board/dependency configuration.
  - `assets/mp3/` — SD-card audio files. Filenames must be zero-padded numeric (e.g. `0011.mp3`). Code references numeric track indexes (e.g. `QueueTrack(11)`).
  ## Key files
  - `src/main.cpp` — canonical wiring, timing constants, LED geometry mapping (heart + concentric levels + arrow), and DFPlayer logic. Read this first for any change.
  - `platformio.ini` — build/board/dependency configuration.
  - `assets/mp3/` — SD-card audio files. Filenames must be zero-padded numeric (e.g. `0011.mp3`). Code references numeric track indexes (e.g. `QueueTrack(11)`). Update audio when swapping from train sounds to Valentine messages.

  ## Build / Flash / Debug (practical commands)
  - Build: `platformio run`
  - Upload/Flash: `platformio run --target upload`
  - Serial monitor: `platformio device monitor` (baud 9600)

  Notes: Serial debug prints are at 9600. DFPlayer uses `HardwareSerial ExtSerial(2)` with RX=16, TX=17 — don't change these pins without updating wiring and comments.

  ## Project-specific conventions & gotchas
  ## Project-specific conventions & gotchas
  - LED geometry: `src/main.cpp` maps LED indexes to shapes: one large heart, four concentric heart levels, and an arrow accent. Animations use those logical groups — if you change `NUM_LEDS` or remap pixel-to-shape, update every place that indexes groups.
  - NUM_LEDS is treated as circular where the code uses wrap-around indexing. In `src/main.cpp` NUM_LEDS is 800 — update references if you change it.
  - LED "house" (accent) strip uses `LED_HOUSE_PIN` and `NUM_HOUSE_LEDS` (15) for smaller, localized effects.
  - ADC pots: code sets 12-bit ADC (`analogReadResolution(12)`) and 11dB attenuation. The repo notes ADC loading/noise when two 10k pots are on the same rail — if you change pots, update mapping logic used for volume and speed.
  - DFPlayer behavior: `QueueTrack()` polls the BUSY pin and can block while waiting. DFPlayer init loops on failure — preserve a safe fallback (log and halt) instead of silent failures.
  - Timing: Animations depend on timing constants (animation speed, read intervals). Keep changes conservative; large timing changes can alter visual rhythm and audio sync.

  ## Integration points & dependencies
  ## Integration points & dependencies
  - Libraries: FastLED and DFRobotDFPlayerMini (declared via PlatformIO). If you add libraries, update `platformio.ini` and verify they build for the chosen ESP32 board.
  - Hardware: WS2812 strip arranged into a heart + concentric levels, DFPlayer Mini (UART2), two analog pots (GPIO34/35), BUSY pin (GPIO27). Keep wiring comments in `src/main.cpp` accurate.

  ## Safe edits an agent can do autonomously
  ## Safe edits an agent can do autonomously
  - Adjust animation constants near the top of `src/main.cpp` (animation speed, `TRAIN_VOLUME` name may still exist from the clone — rename carefully if desired), `NUM_HOUSE_LEDS`, and color palettes for heart levels.
  - Add audio: copy `assets/mp3/00NN.mp3` (e.g. `0001.mp3`) and call `QueueTrack(NN)`; include duration when needed. Rename tracks from train sounds to Valentine messages as needed.
  - Extract low-risk helpers: move heart-drawing logic to `src/leds.cpp` or `lib/leds/` but keep a compatible API (drawHeartLevel(level, progress), show()).

  Examples: change a concentric-heart color palette, animate a pulsing beat on the large heart, or add an arrow sweep effect using the existing `ledsHouse` accent strip.

  ## Testing & verification
  ## Testing & verification
  - There are no unit tests. Validate changes by building and flashing, then watching serial output for startup lines: DFPlayer init, "ADC configured", and periodic position/animation prints (the code prints status periodically — use those lines to sanity-check animation loops).
  - In PRs include: (1) build output, (2) short serial log showing DFPlayer online, and (3) one animation loop print (or a small set of prints showing heart-level activity).

  ## When to ask the maintainer
  - Any change to pin assignments, ADC attenuation, `NUM_LEDS`, or additions to `platformio.ini` must be confirmed with the maintainer before merge.

  If you'd like, I can expand this with an example patch that: adds a new LED helper file, or shows how to add a PlatformIO library and validate a build locally. Tell me which example you prefer.
