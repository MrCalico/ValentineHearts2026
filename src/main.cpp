#include <Arduino.h>
#include <FastLED.h>
#include <DFRobotDFPlayerMini.h>
#include <HardwareSerial.h>

const int MP3_STATION = 13;
const int MP3_HORN = 2;
const int MP3_RUNNING = 3;
const int MP3_BELL = 4;
const int MP3_WHISTLE = 5;
const int MP3_CROSSING_HORN = 6;
const int MP3_CROSSING_BELL = 7;
const int MP3_Whistle2 = 8;
const int MP3_Crossing_Bell2 = 9;
const int MP3_ABORD = 10;
const int MP3_MerryChristmas = 11;
const int MP3_Station = 12;
const int MP3_Parked = 13;

const uint8_t NUM_CARS = 11;  // number of train cars (not including locomotive and caboose) Odd # for green before cabose.
// START_OFFSET removed — track is circular starting at index 0

// initial speed (ms between frames). We'll copy this into a mutable variable so a pot can control it.
const uint16_t TRAIN_SPEED = 220;  // default speed (ms between frames)
const uint8_t  TRAIN_VOLUME = 30;  // Volume level for DFPlayer Mini (0-30)

//DFRobot DFPlayer Mini setup
// We'll use UART2 (index 2). You can use 1 for UART1 if you prefer.
HardwareSerial ExtSerial(2);
// Pick valid pins for your wiring:
constexpr int RX_PIN = 16;  // GPIO16 (safe for RX)
constexpr int TX_PIN = 17;  // GPIO17 (safe for TX)
constexpr unsigned long BAUD = 9600;
DFRobotDFPlayerMini myDFPlayer;
// Potentiometer pins (ADC1) - using GPIO34/35 (far from UART2 on GPIO16/17) to minimize interference
constexpr int VOLUME_POT_PIN = 34; // ADC1_CH6 (GPIO34) - far from UART2, less noisy
// NOTE: Two 10k pots in parallel on the same 3.3V rail create loading that limits ADC range to ~0–450.
// SOLUTION: Add a 10–100 nF capacitor from wiper to GND (reduces noise) and map 0–450 instead of 0–4095.
// FUTURE: Upgrade to 100k pots to eliminate parallel loading, then change mapping back to 0–4095.
constexpr int VOLUME_POT_READ_INTERVAL = 250; // ms between pot reads
constexpr uint8_t VOLUME_HYST = 1; // minimum volume change to apply
// Second pot for train speed control
constexpr int SPEED_POT_PIN = 35; // ADC1_CH7 (GPIO35) - far from UART2, less noisy
constexpr uint8_t SPEED_HYST = 5; // ms delta before applying new speed
constexpr int SPEED_MIN_MS = 1;  // fastest animation delay
constexpr int SPEED_MAX_MS = 500; // slowest animation delay

constexpr int BUSY_PIN = 27; // optional pin to monitor DFPlayer busy status

constexpr uint8_t LED_PIN = 4;
constexpr uint16_t NUM_LEDS = 800;

constexpr uint8_t LED_HOUSE_PIN = 19; // House Strip of LEDS
constexpr uint16_t NUM_HOUSE_LEDS = 15; // same number for simplicity

uint8_t playerState = 0;
  
CRGB leds[NUM_LEDS];
CRGB ledsHouse[NUM_HOUSE_LEDS];

// ---------------------------------------------------------------------------
// Heart & Arrow mapping constants
// The project was converted from a train to a Valentine display. The main strip
// is logically divided into a large heart + 4 concentric heart levels. The
// 'house' accent strip (ledsHouse) is used for an arrow/accent sweep.
//
// Assumption: by default we split the main strip into NUM_HEART_LEVELS equal
// contiguous regions. If your physical mapping differs, change HEART_START
// and HEART_LENGTH values to match the actual pixel ranges.
// ---------------------------------------------------------------------------

constexpr uint8_t NUM_HEART_LEVELS = 4; // 1 small/innermost + 3 concentric outward

// Sizes and start indices will be computed at setup time to match the
// physical wiring order (smallest heart first at index 0). This allows the
// layout to adapt if the strip mapping is not perfectly evenly partitioned.
uint16_t heartLevelSizes[NUM_HEART_LEVELS];
uint16_t heartStart[NUM_HEART_LEVELS];

// Default colors for each heart level (index 0 = smallest/innermost). You
// can change these at runtime if desired.
const CRGB HEART_COLORS[NUM_HEART_LEVELS] = {
  CRGB(255, 160, 180), // center / brightest
  CRGB(255, 100, 140),
  CRGB(255, 40, 90),
  CRGB(220, 20, 60)    // outer-most / darker
};

// Arrow mapping on the main strip: this project uses a single LED strip.
// The arrow lives at the start of the main strip (index 0). Adjust
// ARROW_MAIN_LENGTH to match the number of pixels wired to the arrow.
constexpr uint16_t ARROW_MAIN_START = 0;
constexpr uint16_t ARROW_MAIN_LENGTH = 284; // arrow occupies ~284 pixels at start of main strip
const CRGB ARROW_COLOR = CRGB(255, 100, 140);
// Flash timing for arrow accent at end of a show cycle
// Gap size (LEDs) between concentric heart regions that should remain dark
constexpr uint16_t HEART_GAP_SIZE = 6; // about 6 connected LEDs between hearts
constexpr uint32_t SHOW_CYCLE_MS = 30UL * 1000UL; // default 30 seconds per show cycle
constexpr uint32_t ARROW_FLASH_MS = 3UL * 1000UL; // flash arrow for 3 seconds
constexpr uint32_t ARROW_BLINK_MS = 250UL; // blink interval during flash


void QueueTrack(int track, bool waitForCompletion = true, uint8_t volume = TRAIN_VOLUME, uint16_t duration = std::numeric_limits<uint16_t>::max())
{
  static uint8_t lastVolume = 255;

  // Play an initial sound on startup (uncomment or change track as desired)
  int playerBusy = digitalRead(BUSY_PIN);
  if(playerBusy == LOW) {
    Serial.print("DFPlayer busy, stopping: ");
    myDFPlayer.stop();
    while((playerBusy=digitalRead(BUSY_PIN))!=HIGH) {
      delay(222); // wait for not busy
      Serial.print(playerBusy);
    }
    Serial.println("DFPlayer ready.");
  }
  // Only set volume if it has changed
  if(volume != lastVolume) {
    myDFPlayer.volume(volume); // Set volume value (0~30).
    lastVolume = volume;
  }
  uint32_t playerStarted = millis();
  Serial.print("Playing track: ");  
  myDFPlayer.play(track); // play track 11 on the SD card - Ho Ho Ho Merry Christmas
  while(digitalRead(BUSY_PIN)!=LOW) {
    delay(10); // wait for playback to start
  } 
  Serial.println(track);
  if(waitForCompletion) {
    uint32_t now = millis();
    uint32_t lastPlayCheck = now;
    Serial.print(F("Waiting, track state: "));
    delay(100);
    while(waitForCompletion && digitalRead(BUSY_PIN)==LOW) {
      now = millis();
      if (now - lastPlayCheck >= 500) {
        //waitForCompletion = !digitalRead(BUSY_PIN); // simulate readState() using BUSY pin
        if (now-playerStarted >= duration) {
          Serial.println(F(" Track duration exceeded, stopping playback."));
          myDFPlayer.stop();
          delay(222); // small delay to allow stop command to take effect
          //waitForCompletion = false;
        }
        Serial.print(waitForCompletion);
        lastPlayCheck = now;
      }
      delay(10); // small delay to avoid busy loop
    }
    Serial.print(F(" Track complete. Duration: "));
    Serial.println(now - playerStarted);
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
  FastLED.addLeds<WS2812, LED_HOUSE_PIN, GRB>(ledsHouse, NUM_HOUSE_LEDS); // same LED array for simplicity
  FastLED.setBrightness(255); // 0..255 (128 ~ 50%)
  
  // Initialize DFPlayer Mini
  ExtSerial.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN ); // Initialize Serial for debug output

  pinMode(BUSY_PIN, INPUT);
  Serial.println();
  Serial.println(F("DFRobot DFPlayer Mini Demo"));
  Serial.println(F("Initializing DFPlayer ... (May take 3~5 seconds)"));
  
  if (!myDFPlayer.begin(ExtSerial)) {  //Use softwareSerial to communicate with mp3.
    Serial.println(F("Unable to begin:"));
    Serial.println(F("1.Please recheck the connection!"));
    Serial.println(F("2.Please insert the SD card!"));
    while(true){
      delay(0); // Code to compatible with ESP8266 watch dog.
    }
  }
  Serial.println(F("DFPlayer Mini online."));

  /*
  */
  // configure ADC pin
  analogReadResolution(12); // 12-bit ADC (0-4095)
  // Ensure full-scale range up to 3.3V on the ADC pin
  analogSetPinAttenuation(VOLUME_POT_PIN, ADC_11db);
  // Configure attenuation for the speed pot as well
  analogSetPinAttenuation(SPEED_POT_PIN, ADC_11db);
  Serial.println(F("ADC configured: 12-bit resolution, 11dB attenuation"));
  // nothing else required for ADC1 pins on ESP32

  // Compute heart level sizes and start indices to match physical wiring
  {
    // Hard-coded heart level mapping (approximation). These values were
    // derived from the previous automatic computation but are easier to
    // tweak manually when the physical wiring is inconsistent.
    // Layout: arrow region (start 0 .. ARROW_MAIN_LENGTH-1), then gaps and
    // heart levels. Index 0 is the first (smallest/inner) heart.
    heartStart[0] = 284; heartLevelSizes[0] = 71; // inner
    heartStart[1] = 362; heartLevelSizes[1] = 99;
    heartStart[2] = 467; heartLevelSizes[2] = 144;
    heartStart[3] = 616; heartLevelSizes[3] = 184; // outer-most

    for (uint8_t L = 0; L < NUM_HEART_LEVELS; ++L) {
      Serial.print(F("Heart level "));
      Serial.print(L);
      Serial.print(F(" start="));
      Serial.print(heartStart[L]);
      Serial.print(F(" len="));
      Serial.println(heartLevelSizes[L]);
    }
  }

  // QueueTrack(11, true, TRAIN_VOLUME); // Play initial track 11 and wait for completion
}

void SantaStop() {
  Serial.println(F("Santa Stop Triggered!"));

  QueueTrack(MP3_MerryChristmas, false, TRAIN_VOLUME); // Play track 12 and wait for completion
  
  // Chase lights from ends to middle while Santa is talking
  FastLED.setBrightness(128); // 0..255 (128 ~ 50%)
  
  int step = 0;
  const int maxSteps = (NUM_HOUSE_LEDS + 1) / 2;

  while(digitalRead(BUSY_PIN) == LOW) {
    fill_solid(ledsHouse, NUM_HOUSE_LEDS, CRGB::Black);
    
    // Light up pixels from both ends moving inward
    ledsHouse[step] = CRGB(100, 100, 100);
    ledsHouse[NUM_HOUSE_LEDS - 1 - step] = CRGB(100, 100, 100);
    
    FastLED.show();
    delay(150);
    
    step++;
    if(step >= maxSteps) {
      step = 0;
    }
  }
  
  fill_solid(ledsHouse, NUM_HOUSE_LEDS, CRGB::Black);
  FastLED.show();
  FastLED.setBrightness(255); // 0..255 (128 ~ 50%)

}

const int SantaStopPosition = 700; // position index to trigger Santa stop

void loop() {
  // New heart animation loop
  static uint32_t lastPotRead = 0;
  static uint8_t currentVolume = TRAIN_VOLUME;
  static uint32_t lastSpeedRead = 0;
  static int frameDelayMs = TRAIN_SPEED; // controlled by speed pot
  static uint32_t showCycleStart = millis();
  static uint16_t arrowPos = 0; // retained for flash sequence

  uint32_t now = millis();

  // ----- Read volume pot (apply hysteresis) -----
  if (now - lastPotRead >= VOLUME_POT_READ_INTERVAL) {
    lastPotRead = now;
    int raw = analogRead(VOLUME_POT_PIN); // 0..4095
    uint8_t vol = map(raw, 0, 4095, 0, 30);
    if (vol > 30) vol = 30;
    if (vol + VOLUME_HYST < currentVolume || vol > currentVolume + VOLUME_HYST) {
      currentVolume = vol;
      myDFPlayer.volume(currentVolume);
      Serial.print(F("Raw ADC (vol): "));
      Serial.print(raw);
      Serial.print(F(" -> volume: "));
      Serial.println(currentVolume);
    }
  }

  // ----- Read speed pot and map to frame delay -----
  if (now - lastSpeedRead >= VOLUME_POT_READ_INTERVAL) {
    lastSpeedRead = now;
    int raw2 = analogRead(SPEED_POT_PIN);
    int mapped = map(raw2, 0, 4095, SPEED_MIN_MS, SPEED_MAX_MS);
    if (abs(mapped - frameDelayMs) > SPEED_HYST) {
      frameDelayMs = mapped;
      Serial.print(F("Raw ADC (spd): "));
      Serial.print(raw2);
      Serial.print(F(" -> frameDelayMs: "));
      Serial.println(frameDelayMs);
    }
  }

  // ----- Draw hearts -----
  fill_solid(leds, NUM_LEDS, CRGB::Black);

  // For each heart level, pulse brightness using a phase derived from millis()
  for (uint8_t level = 0; level < NUM_HEART_LEVELS; ++level) {
    uint16_t start = heartStart[level];
    uint16_t len = heartLevelSizes[level];

    // derive a phase; inner levels pulse slightly faster
    uint32_t speedFactor = max(10, frameDelayMs);
    uint8_t phase = (uint8_t)((now / speedFactor) + (level * 24));
    uint8_t wave = sin8(phase); // 0..255

    // scale the wave to a usable brightness (avoid full-zero)
    uint8_t bright = scale8(wave, 200) + 32; // 32..232

    CRGB col = HEART_COLORS[level];
    col.nscale8_video(bright);

    for (uint16_t p = 0; p < len; ++p) {
      uint16_t idx = (start + p) % NUM_LEDS;
      leds[idx] = col;
    }
  }

  // ----- Arrow accent on house strip (simple moving bright pixel) -----
  // Keep the arrow off during normal animation (house strip black)
  fill_solid(ledsHouse, NUM_HOUSE_LEDS, CRGB::Black);

  FastLED.show();

  // simple frame delay controlled by the speed pot
  delay(frameDelayMs);

  // Check show cycle elapsed; if so, flash the arrow for ARROW_FLASH_MS
  if (now - showCycleStart >= SHOW_CYCLE_MS) {
    uint32_t flashStart = millis();
    while (millis() - flashStart < ARROW_FLASH_MS) {
      // blink arrow on (main strip)
      for (uint16_t a = 0; a < ARROW_MAIN_LENGTH; ++a) {
        uint16_t idx = (ARROW_MAIN_START + a) % NUM_LEDS;
        leds[idx] = ARROW_COLOR;
      }
      FastLED.show();
      delay(ARROW_BLINK_MS);
      // blink arrow off
      for (uint16_t a = 0; a < ARROW_MAIN_LENGTH; ++a) {
        uint16_t idx = (ARROW_MAIN_START + a) % NUM_LEDS;
        leds[idx] = CRGB::Black;
      }
      FastLED.show();
      delay(ARROW_BLINK_MS);
    }
    // reset cycle
    showCycleStart = millis();
    // ensure house strip off after flash
    fill_solid(ledsHouse, NUM_HOUSE_LEDS, CRGB::Black);
    FastLED.show();
  }
}
