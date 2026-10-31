/*
 ============================================================================
  TRUCK BLACK BOX  v3  -  ESP32-S3
  150 sensors / 13 systems / 31 sub-systems (incl. fuses + relays), ECU-scan-tool style, white + blue UI
  Display : ST7735 128x160, NO display library (only SPI.h + own driver)
  Storage : internal flash (LittleFS, built into the ESP32 core)
  Data in : USB serial  AND  WiFi (HTTP)  -  both work at the same time
  Alerts  : Telegram bot with emojis, ONLY when a real problem happens
 ============================================================================

  ARDUINO IDE SETTINGS
    Board            : ESP32S3 Dev Module
    Partition Scheme : pick one with the BIGGEST "SPIFFS" / storage part (more storage = more black box history)
    USB CDC On Boot  : Enabled if the cable is in the port named "USB", Disabled if it is in the port named "UART"
    PSRAM            : OPI PSRAM if your board is N8R8 / N16R8. Works without PSRAM too (shorter recordings).
    No extra library needs to be installed.
    Sketch folder name must be the same as this file name (truck_black_box_v4).

  WIRING (module pin -> ESP32-S3 GPIO)
    TFT  VCC->3V3  GND->GND  CS->10  RESET->8  A0/DC->9  SDA/MOSI->11  SCK->12  LED->14
    Touch (own pins, NOT shared with the TFT): T_CLK->15 T_DIN->16 T_DO->17 T_CS->18 T_IRQ->21
    Buttons (other leg to GND): LEFT->4  RIGHT->5  MORSE->6        Buzzer (optional): GPIO 2

  BUTTONS
    Driver screen : hold LEFT+RIGHT 0.5 s = mechanic mode (only when the vehicle is stopped).
                    If a problem is active, mechanic mode opens straight on AUTO SCAN.
                    MORSE short = mute buzzer
    Other screens : LEFT = up / previous    RIGHT = down / next
                    MORSE short = OK / open    MORSE hold 0.7 s = back
    Black box search bar: select "Find", press MORSE -> typing mode
                    MORSE short = dot, MORSE long = dash (letter is added after a short pause)
                    LEFT = delete, RIGHT = done
  TOUCH: tap rows, "<" top-left = back, drag on a graph to move the cursor.
    Touch not working / taps land in the wrong place?  Menu -> STORAGE / TOOLS -> TOUCH TEST (shows raw numbers)
    and CALIBRATE TOUCH (4 taps). Hold the MORSE button while powering on = calibration straight away.
  BUZZER: STORAGE / TOOLS -> TEST BUZZER. Silent? change BUZZER_ACTIVE (0 <-> 1).

  MENU TREE (opens as deep as you like):
    AUTO SCAN  |  SYSTEM SCAN -> SYSTEM -> SUB-SYSTEM -> SENSOR -> problem history -> incident -> graph
    FAULT CODES | BLACK BOX | STORAGE / TOOLS (erase logs, reset limits, test Telegram) | DEVICE STATUS

  DATA PROTOCOL (one text line per sample, from USB serial 115200 OR  HTTP POST http://<ip>/d )
    D,v0,v1,...,v149,flags     values in catalog order, "x" = sensor gives no signal, flags bit0 = MIL lamp
    T,<epoch>                  set the clock (local epoch seconds)
    C,<idx>,<wLo>,<cLo>,<wHi>,<cHi>   change the limits of one sensor ("n" = no limit)
    CR                         reset all limits to default
    ?                          (serial) print the sensor catalog as JSON lines between #CATALOG_BEGIN / #CATALOG_END
  WiFi:
    The ESP32 makes its own hotspot  AP_SSID / AP_PASS  (http://192.168.4.1)  AND joins your WiFi (for Telegram).
    GET  /catalog   all systems, sub-systems, sensors, limits (JSON)  -> a dashboard builds itself from this
    GET  /status    live summary (JSON)
    POST /d         sensor lines (text/plain)
    POST /dash      store a dashboard page (HTML) in flash,  GET /  then serves it
  Sensor codes use J1939 SPN numbers + FMI (as on real trucks). SPN numbers are for demo / learning, verify
  against SAE J1939-71 before using on a real vehicle.

  TELEGRAM: fill TG_BOT_TOKEN / TG_CHAT_ID below. Chat id: send any message to your bot, open
    https://api.telegram.org/bot<TOKEN>/getUpdates and look for "chat":{"id":...
    NEVER share this file after you put your token / WiFi password in it.
 ============================================================================
*/

/*  v5 = ESP32-S3 BLACK BOX: Nokia-style app home, per-system pages with sensor icons, FULL SCAN + white REPORT page
      (what failed, when, at what speed / rpm; history kept in flash), menu works while the vehicle is running.
      Board: "ESP32S3 Dev Module". Pico (fake ECU) -> S3:  Pico GP0 (TX) -> S3 GPIO18 (RX),  S3 GPIO17 (TX) -> Pico GP1 (RX),  GND common, 115200 baud.
      Display/buttons/buzzer on the S3 are wired as in the S3 pin block below. Buttons: LEFT / RIGHT move, MORSE(OK) opens / selects, hold MORSE = back.
*/
/*  v4 pin arrangement for the soldered DevKit wiring (final):
      display : CS 5, A0 21, RESET 22, SCK 23, SDA 18, LED -> 3V3, VCC -> 3V3, GND -> GND
      buttons : LEFT 13, RIGHT 14, MORSE 12   (other leg of each button -> GND)
      buzzer  : GPIO 4 (+), GND (-)
      red LED : GPIO 27 -> 330 ohm -> LED long leg (+), short leg (-) -> GND
                tiny flash every 3 s = all fine | slow blink = warning | fast blink = critical
                double flash = ECU link lost | tiny flash every 2 s = waiting for ECU data
      S3 link : S3 TX -> DevKit GPIO 16, S3 RX -> DevKit GPIO 17, GND common, 115200 baud (later: CAN transceiver on the same pins)
      GPIO12 (MORSE) is a boot pin: a button to GND is fine, never pull it HIGH at power-on.
*/

/*  v4 (ESP32 DevKit + UART link from the ESP32-S3 fake ECU)
    - Pins for the classic ESP32 DevKit (your soldered wiring), touch OFF, S3 pins kept under CONFIG_IDF_TARGET_ESP32S3
    - NEW data input: UART2 (RX=GPIO16, TX=GPIO17, 115200) = wires from the S3 fake ECU. USB serial and WiFi still work too.
    - NEW FAULT LOG: every problem that starts / ends is written to flash AT ONCE (sensor, value, limit, speed, rpm, time).
      See it in SCAN TOOL > FAULT LOG, and AUTO SCAN shows it when nothing is active. Survives power off.
    - FIX: a problem that was already alerted could not alert again for 2 minutes even after it was fixed -> now it alerts again
      (8 s minimum gap) every time it comes back.
    - FIX: bad / short data lines are no longer dropped silently (counter on DEVICE STATUS + #BAD lines on USB serial).
    - FIX: plain ESP32 has no PSRAM -> shorter recording (15 s before, 8 s after) so Telegram (TLS) keeps enough free RAM.
    - NEW checks for sensors without limits: pedal 1 vs pedal 2, rail pressure vs rail desired. */
#include <SPI.h>
#include <LittleFS.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <WebServer.h>
#include <esp_system.h>
#include <driver/ledc.h>
#include <time.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>

// ============================ 1. YOUR SETTINGS ============================
#define WIFI_SSID     "realme P4 5G z7n5"        // your WiFi / phone hotspot (needed for Telegram). Leave as is = no STA WiFi
#define WIFI_PASS     "spcy5222"
#define AP_SSID       "TruckBlackBox"         // hotspot made by the ESP32 itself (dashboard works even with no internet)
#define AP_PASS       "truck1234"             // at least 8 characters
#define TG_BOT_TOKEN  "8798775739:AAEJJY8g-CLapzs9JbNFOmkzupNTDUJVIWs"   // from BotFather. This token was shown in a chat: /revoke it after the demo.
#define TG_CHAT_ID    "7867587516"
#define TIME_ZONE_SEC 19800                   // India = +5:30 = 19800 s

#define TG_SEND_BOOT        1        // one short message when the device starts (with reset reason)
#define TG_SEND_CLEARED     0        // 1 = also tell me when a problem goes away
#define ALERT_MIN_GAP_MS    8000UL   // a problem that comes back after it was fixed is sent again (not faster than this)
#define ALERT_BATCH_MS     1200UL    // problems that appear together go in ONE message
#define TG_REMIND_MS     300000UL    // while problems are still active, send a full list every 5 min (0 = off)

// ============================ 2. PINS =====================================
#if CONFIG_IDF_TARGET_ESP32S3
  // ---- ESP32-S3 board = the BLACK BOX with the display (Pico = fake ECU on GPIO18/17) ----
  #define PIN_TFT_CS     10
  #define PIN_TFT_DC      9
  #define PIN_TFT_RST     8
  #define PIN_TFT_SCK    12
  #define PIN_TFT_MOSI   11
  #define PIN_TFT_LED    14
  #define PIN_BTN_LEFT    4
  #define PIN_BTN_RIGHT   5
  #define PIN_BTN_MORSE   6
  #define PIN_BUZZER      2
  #define PIN_LINK_RX    18      // from the Raspberry Pi Pico (fake ECU) TX = GP0
  #define PIN_LINK_TX    17      // to the Pico RX = GP1
  #define PIN_T_CLK      15      // touch pins (only used when USE_TOUCH = 1)
  #define PIN_T_DIN      16
  #define PIN_T_DO       39
  #define PIN_T_CS       40
  #define PIN_T_IRQ      41
  #define USE_RGB_LED     1      // built-in RGB LED: green OK, amber warning, red problem
  #define PIN_LED_RED    -1      // plain red LED (not used on this board)
  #ifdef RGB_BUILTIN
    #define PIN_RGB RGB_BUILTIN
  #else
    #define PIN_RGB 48         // DevKitC-1 v1.0 = 48, v1.1 = 38. Wrong colour / nothing? try 38
  #endif
#else
  // ---- classic ESP32 DevKit (your soldered wiring) ----
  #define PIN_TFT_CS      5      // display CS
  #define PIN_TFT_DC     21      // display A0
  #define PIN_TFT_RST    22      // display RESET
  #define PIN_TFT_SCK    23      // display SCK
  #define PIN_TFT_MOSI   18      // display SDA
  #define PIN_TFT_LED    -1      // display LED pin is wired to 3V3 (no GPIO needed)
  #define PIN_BTN_LEFT   13      // other leg of every button -> GND
  #define PIN_BTN_RIGHT  14
  #define PIN_BTN_MORSE  12      // GPIO12 = boot pin: button to GND is fine, never pull it HIGH at power-on
  #define PIN_BUZZER      4      // set to -1 if no buzzer
  #define PIN_LINK_RX    16      // from the S3 fake ECU TX
  #define PIN_LINK_TX    17      // to the S3 fake ECU RX
  #define PIN_T_CLK      32      // touch pins (only used when USE_TOUCH = 1; your display has no touch chip)
  #define PIN_T_DIN      33
  #define PIN_T_DO       25
  #define PIN_T_CS       26
  #define PIN_T_IRQ      34
  #define USE_RGB_LED     0      // 1 + PIN_RGB = external WS2812 RGB LED instead of the plain red LED
  #define PIN_RGB        15
  #define PIN_LED_RED    27      // plain red LED: GPIO27 -> 330 ohm -> LED (+), LED (-) -> GND   (-1 = no LED)
#endif
#define TFT_LED_ON   HIGH      // backlight dark? change to LOW (only if PIN_TFT_LED is a GPIO)
#define LINK_BAUD   115200     // UART speed to the fake ECU (same number in the S3 firmware)
#define BUZZER_ACTIVE   0      // 0 = passive buzzer / piezo (sound is made by a PWM tone - works with most buzzers)
                               // 1 = "active" buzzer module that beeps by itself when the pin is ON
#define BUZZER_INVERT   0      // only for BUZZER_ACTIVE 1: set 1 if your module sounds when the pin is LOW
#define BUZZER_FREQ  2700      // tone frequency (Hz) for passive buzzers
#define BUZZER_CLICK    1      // short click when a button / the touch screen is pressed (0 = silent)

#define USE_TOUCH       0      // your display module has NO touch chip
#define TOUCH_USE_IRQ   0      // 0 = detect a touch by pressure (works even if the T_IRQ wire is NOT connected)
                               // 1 = use the T_IRQ pin to detect a touch (T_IRQ must be wired)
#define TOUCH_Z_MIN   250      // pressure needed to count as a touch (lower = more sensitive)

#define RGB_LEVEL      40      // brightness 0-255

#define SCREEN_W      128
#define SCREEN_H      160
#define SPI_HZ   16000000      // picture garbled? try 10000000
#define MADCTL_VALUE 0xC0      // red/blue swapped? 0xC8. upside down / mirrored? 0x00 0x08 0x40 0x80
#define COL_OFFSET      0
#define ROW_OFFSET      0
#define USE_INVERT      0      // colours look negative? set 1

#define TOUCH_DEBUG     0      // 1 = print raw touch numbers in Serial Monitor
#define TOUCH_SWAP_XY   0
#define TOUCH_X_MIN   300
#define TOUCH_X_MAX  3800
#define TOUCH_Y_MIN   300
#define TOUCH_Y_MAX  3800
#define TOUCH_INVERT_X  0
#define TOUCH_INVERT_Y  0

// ============================ 3. RULES / SIZES ============================
#define RUNNING_RPM        400   // "engine running" above this rpm
#define LINK_TIMEOUT_MS   3000   // no ECU data for this long = ECU link lost
#define DEBOUNCE_SAMPLES     3   // a limit must be crossed this many samples in a row
#define CLEAR_SAMPLES       15   // and stay OK this many samples to clear
#define WHEEL_MISMATCH_KMH  12.0f
#define PEDAL_MISMATCH_PCT  10.0f   // pedal 1 and pedal 2 must agree within this
#define RAIL_DEVIATION_BAR 250.0f   // rail pressure may differ from the desired value by this much (engine running)
#define MECH_SPEED_LIMIT     5.0f   // mechanic mode only below this speed (km/h)
#define ALLOW_MENU_WHILE_MOVING 1   // 1 = bench test with the fake ECU (menu works at any speed). Set 0 on the real truck.

#define SAMPLE_HZ            5      // the PC / dashboard should send 5 lines per second
#define PRE_SEC_PSRAM       60      // black box: seconds recorded BEFORE the fault  (board with PSRAM)
#define POST_SEC_PSRAM      30      //                           AFTER the fault
#define PRE_SEC_RAM         15      // same, board WITHOUT PSRAM
#define POST_SEC_RAM         8
#define MAX_INC             24      // oldest incident is deleted when full
#define FS_RESERVE       20000
#define FLASH_HIGH_PCT      85      // flash this full -> oldest black-box incidents are deleted automatically ...
#define FLASH_LOW_PCT       70      // ... until the flash is only this full
#define MAXROWS            160

// ============================ 4. COLOURS (white + blue) ===================
#define RGB565(r,g,b) ((uint16_t)((((r) & 0xF8) << 8) | (((g) & 0xFC) << 3) | ((b) >> 3)))
#define TRANSP    0x0001
#define C_BLACK   0x0000
#define C_WHITE   0xFFFF
#define C_INK     RGB565(10, 32, 80)       // main text (dark navy)
#define C_MUTED   RGB565(92, 112, 150)     // secondary text
#define C_GREY    RGB565(150, 160, 182)
#define BG_TOP    RGB565(252, 253, 255)
#define BG_BOT    RGB565(220, 234, 252)
#define CARD1     RGB565(255, 255, 255)
#define CARD2     RGB565(238, 245, 255)
#define CARD_LINE RGB565(186, 208, 244)
#define PANEL     RGB565(232, 241, 255)
#define BLUE      RGB565(22, 96, 214)
#define SELBG     RGB565(22, 110, 235)
#define COL_OK    RGB565(16, 158, 92)
#define COL_WARN  RGB565(232, 140, 0)
#define COL_CRIT  RGB565(214, 40, 52)

// ============================ 5. SYSTEMS / SUB-SYSTEMS ====================
#define NOLIM 1.0e9f
#define NOLO  (-NOLIM)
#define NOHI  (NOLIM)
#define SF_RUN  1      // low limits only checked while the engine runs
#define SF_CRIT 2      // a sensor failure counts as critical
#define SF_BOOL 4      // shown as NO / YES
#define SF_FUSE 8      // value 1 = OK, 0 = BLOWN
#define SF_RELAY 16    // value 1 = OK, 0 = FAILED

enum { SY_ENG, SY_AIR, SY_FUEL, SY_INJ, SY_COOL, SY_EXH, SY_TRN, SY_BRK, SY_STR, SY_TYR, SY_ELE, SY_VEH, SY_FUSE, NSYS };
enum { SB_SPDLOAD, SB_OIL, SB_INTAKE, SB_TURBO, SB_EGR, SB_FEED, SB_RAIL, SB_INJCORR, SB_CYLBAL, SB_INJTIM,
       SB_COOLANT, SB_FANPUMP, SB_EGTS, SB_DPF, SB_SCR, SB_GEARBOX, SB_DRIVELINE, SB_WHEELSPD, SB_AIRBRK,
       SB_PEDPAD, SB_STEER, SB_SUSP, SB_TYREP, SB_TYRET, SB_BATT, SB_CHARGE, SB_MOTION, SB_CAB, SB_FUSE_ENG, SB_FUSE_CHS, SB_RELAYS, NSUB };

#define NICONS 23
#define IC_ENGINE 0
#define IC_AIR 1
#define IC_FUEL 2
#define IC_INJ 3
#define IC_EXHAUST 4
#define IC_TRANS 5
#define IC_BRAKES 6
#define IC_STEER 7
#define IC_ELEC 8
#define IC_VEHICLE 9
#define IC_SCAN 10
#define IC_WARN 11
#define IC_BOX 12
#define IC_STATUS 13
#define IC_OK 14
#define IC_THERMO 15
#define IC_TYRE 16
#define IC_CAB 17
#define IC_FUSE 18
#define IC_DIAL 19
#define IC_BOLT 20
#define IC_LEVEL 21
#define IC_DOC 22

struct SysInfo { const char* name; uint16_t color; const char* emoji; uint8_t icon; };
static const SysInfo SYS[NSYS] = {
  {"ENGINE",        RGB565(236, 112, 40),  "⚙️", IC_ENGINE},
  {"AIR & TURBO",   RGB565(0, 150, 205),   "🌬️", IC_AIR},
  {"FUEL SYSTEM",   RGB565(222, 160, 10),  "⛽", IC_FUEL},
  {"INJECTORS",     RGB565(150, 80, 215),  "💉", IC_INJ},
  {"COOLING",       RGB565(0, 160, 170),   "🌡️", IC_THERMO},
  {"EXHAUST / SCR", RGB565(96, 116, 150),  "💨", IC_EXHAUST},
  {"TRANSMISSION",  RGB565(50, 96, 230),   "🔧", IC_TRANS},
  {"BRAKES / ABS",  RGB565(214, 48, 60),   "🛑", IC_BRAKES},
  {"STEER / SUSP",  RGB565(24, 160, 140),  "🧭", IC_STEER},
  {"TYRES",         RGB565(70, 84, 110),   "🛞", IC_TYRE},
  {"ELECTRICAL",    RGB565(36, 164, 84),   "🔋", IC_ELEC},
  {"VEHICLE / CAB", RGB565(222, 80, 146),  "🚛", IC_CAB},
  {"FUSES / RELAYS", RGB565(176, 126, 8),  "🧯", IC_FUSE},
};

struct SubInfo { uint8_t sys; const char* name; const char* adv; };
static const SubInfo SUBS[NSUB] = {
  {SY_ENG,  "Speed & load",       "Ease off the throttle. Shift up."},
  {SY_ENG,  "Oil system",         "Stop soon. Check oil level and pressure."},
  {SY_AIR,  "Air intake",         "Check air filter and intake hoses."},
  {SY_AIR,  "Turbocharger",       "Turbo issue. Reduce load, check hoses."},
  {SY_AIR,  "EGR system",         "EGR fault. Get it scanned soon."},
  {SY_FUEL, "Fuel feed (low P)",  "Check fuel filter and fuel level."},
  {SY_FUEL, "Common rail (high P)", "Rail pressure fault. Check filter and pump."},
  {SY_INJ,  "Injector correction", "Injector imbalance. Get injectors checked."},
  {SY_INJ,  "Cylinder balance",   "Possible misfire. Check the injectors."},
  {SY_INJ,  "Injection timing",   "Injection fault. Get the ECU scanned."},
  {SY_COOL, "Coolant",            "Engine hot. Pull over, let it idle."},
  {SY_COOL, "Fan & water pump",   "Cooling fan or pump fault. Check cooling."},
  {SY_EXH,  "Exhaust temps",      "Exhaust too hot. Reduce load."},
  {SY_EXH,  "DPF",                "DPF clogged. It needs regeneration."},
  {SY_EXH,  "SCR / AdBlue",       "AdBlue or NOx fault. Refill or service SCR."},
  {SY_TRN,  "Gearbox",            "Gearbox fault. Stop and let it cool."},
  {SY_TRN,  "Axles / driveline",  "Axle oil hot. Reduce load, check oil."},
  {SY_BRK,  "Wheel speed (ABS)",  "Check wheel speed sensor and ABS ring."},
  {SY_BRK,  "Air brake system",   "Low brake air! Stop safely, check leaks."},
  {SY_BRK,  "Pedal, pads, retarder", "Brake wear or heat. Slow down, service brakes."},
  {SY_STR,  "Steering",           "Heavy steering. Check EPS and linkage."},
  {SY_STR,  "Air suspension",     "Suspension fault. Check the air springs."},
  {SY_TYR,  "Tyre pressure",      "Tyre pressure bad. Stop and check the tyre."},
  {SY_TYR,  "Tyre temperature",   "Tyre too hot. Slow down, check the tyre."},
  {SY_ELE,  "Battery",            "Battery fault. Switch off extra loads."},
  {SY_ELE,  "Charging",           "Charging fault. Check alternator and belt."},
  {SY_VEH,  "Motion & economy",   "Drive carefully. Slow down."},
  {SY_VEH,  "Cab & status",       "Warning lamp active. Get the vehicle checked."},
  {SY_FUSE, "Engine fuses",       "Blown fuse. Find the short, then fit same rating."},
  {SY_FUSE, "Chassis fuses",      "Blown fuse. Find the short, then fit same rating."},
  {SY_FUSE, "Relays",             "Relay not switching. Check relay, wiring and its fuse."},
};

// ===== SENSOR CATALOG (single source: enum, table and the JSON for the dashboard are built from this list) =====
// X(id, system, sub-system, name, unit, scale, decimals, gaugeLo, gaugeHi, warnLow, critLow, warnHigh, critHigh, possibleMin, possibleMax, maxJump, flags, J1939 SPN)
#define SENSOR_LIST(X) \
  X(RPM, SY_ENG, SB_SPDLOAD, "Engine speed", "rpm", 1, 0, 0.0f, 2800.0f, NOLO, NOLO, 2300.0f, 2500.0f, 0.0f, 4000.0f, 0.0f, 0, 190) \
  X(LOAD, SY_ENG, SB_SPDLOAD, "Engine load", "%", 1, 0, 0.0f, 100.0f, NOLO, NOLO, NOHI, NOHI, 0.0f, 110.0f, 0.0f, 0, 92) \
  X(TORQ, SY_ENG, SB_SPDLOAD, "Actual torque", "%", 1, 0, -10.0f, 110.0f, NOLO, NOLO, NOHI, NOHI, -125.0f, 130.0f, 0.0f, 0, 513) \
  X(APP, SY_ENG, SB_SPDLOAD, "Accel pedal", "%", 1, 0, 0.0f, 100.0f, NOLO, NOLO, NOHI, NOHI, 0.0f, 105.0f, 0.0f, 0, 91) \
  X(APP2, SY_ENG, SB_SPDLOAD, "Accel pedal 2", "%", 1, 0, 0.0f, 100.0f, NOLO, NOLO, NOHI, NOHI, 0.0f, 105.0f, 0.0f, 0, 29) \
  X(OILP, SY_ENG, SB_OIL, "Oil pressure", "kPa", 1, 0, 0.0f, 800.0f, 150.0f, 100.0f, 600.0f, 700.0f, 0.0f, 1000.0f, 250.0f, 3, 100) \
  X(OILT, SY_ENG, SB_OIL, "Oil temp", "C", 1, 0, 0.0f, 160.0f, NOLO, NOLO, 120.0f, 135.0f, -40.0f, 180.0f, 20.0f, 0, 175) \
  X(OILL, SY_ENG, SB_OIL, "Oil level", "%", 1, 0, 0.0f, 100.0f, 30.0f, 15.0f, NOHI, NOHI, 0.0f, 110.0f, 0.0f, 0, 98) \
  X(OILD, SY_ENG, SB_OIL, "Oil filter dP", "kPa", 1, 0, 0.0f, 150.0f, NOLO, NOLO, 100.0f, 130.0f, 0.0f, 200.0f, 0.0f, 0, 99) \
  X(MAP, SY_AIR, SB_INTAKE, "Boost pressure", "kPa", 1, 0, 0.0f, 300.0f, NOLO, NOLO, 250.0f, 280.0f, 0.0f, 400.0f, 100.0f, 0, 102) \
  X(IAT, SY_AIR, SB_INTAKE, "Intake temp", "C", 1, 0, -20.0f, 120.0f, NOLO, NOLO, 80.0f, 100.0f, -40.0f, 150.0f, 15.0f, 0, 105) \
  X(MAF, SY_AIR, SB_INTAKE, "Air mass flow", "kg/h", 1, 0, 0.0f, 1200.0f, NOLO, NOLO, NOHI, NOHI, 0.0f, 2000.0f, 0.0f, 0, 132) \
  X(BARO, SY_AIR, SB_INTAKE, "Baro pressure", "kPa", 1, 0, 50.0f, 110.0f, NOLO, NOLO, NOHI, NOHI, 40.0f, 120.0f, 0.0f, 0, 108) \
  X(AFD, SY_AIR, SB_INTAKE, "Air filter dP", "kPa", 10, 1, 0.0f, 10.0f, NOLO, NOLO, 5.0f, 7.5f, 0.0f, 15.0f, 0.0f, 0, 107) \
  X(TSPD, SY_AIR, SB_TURBO, "Turbo speed", "krpm", 10, 1, 0.0f, 160.0f, NOLO, NOLO, 130.0f, 150.0f, 0.0f, 200.0f, 0.0f, 0, 103) \
  X(VGT, SY_AIR, SB_TURBO, "VGT position", "%", 1, 0, 0.0f, 100.0f, NOLO, NOLO, NOHI, NOHI, 0.0f, 105.0f, 0.0f, 0, 2795) \
  X(CACT, SY_AIR, SB_TURBO, "Charge air out", "C", 1, 0, -20.0f, 120.0f, NOLO, NOLO, 70.0f, 90.0f, -40.0f, 150.0f, 15.0f, 0, 2630) \
  X(EGR, SY_AIR, SB_EGR, "EGR valve", "%", 1, 0, 0.0f, 100.0f, NOLO, NOLO, NOHI, NOHI, 0.0f, 105.0f, 0.0f, 0, 27) \
  X(EGRT, SY_AIR, SB_EGR, "EGR gas temp", "C", 1, 0, 0.0f, 500.0f, NOLO, NOLO, 400.0f, 450.0f, -40.0f, 600.0f, 100.0f, 0, 412) \
  X(EGRD, SY_AIR, SB_EGR, "EGR delta P", "kPa", 10, 1, 0.0f, 30.0f, NOLO, NOLO, NOHI, NOHI, 0.0f, 50.0f, 0.0f, 0, 411) \
  X(EGRC, SY_AIR, SB_EGR, "EGR cooler out", "C", 1, 0, 0.0f, 300.0f, NOLO, NOLO, 220.0f, 260.0f, -40.0f, 400.0f, 80.0f, 0, 0) \
  X(FUEL, SY_FUEL, SB_FEED, "Fuel level", "%", 1, 0, 0.0f, 100.0f, 10.0f, 5.0f, NOHI, NOHI, 0.0f, 105.0f, 0.0f, 0, 96) \
  X(FUELT, SY_FUEL, SB_FEED, "Fuel temp", "C", 1, 0, 0.0f, 100.0f, NOLO, NOLO, 60.0f, 75.0f, -40.0f, 120.0f, 15.0f, 0, 174) \
  X(FEEDP, SY_FUEL, SB_FEED, "Fuel feed press", "kPa", 1, 0, 0.0f, 600.0f, 200.0f, 120.0f, NOHI, NOHI, 0.0f, 1000.0f, 200.0f, 1, 94) \
  X(FFILT, SY_FUEL, SB_FEED, "Fuel filter dP", "kPa", 1, 0, 0.0f, 100.0f, NOLO, NOLO, 50.0f, 80.0f, 0.0f, 150.0f, 0.0f, 0, 95) \
  X(WIF, SY_FUEL, SB_FEED, "Water in fuel", "", 1, 0, 0.0f, 1.0f, NOLO, NOLO, 0.5f, 0.5f, 0.0f, 1.0f, 0.0f, 4, 97) \
  X(FRP, SY_FUEL, SB_RAIL, "Rail pressure", "bar", 1, 0, 0.0f, 2000.0f, 200.0f, 120.0f, 1650.0f, 1750.0f, 0.0f, 2400.0f, 600.0f, 3, 157) \
  X(FRPD, SY_FUEL, SB_RAIL, "Rail desired", "bar", 1, 0, 0.0f, 2000.0f, NOLO, NOLO, NOHI, NOHI, 0.0f, 2400.0f, 0.0f, 0, 0) \
  X(PCV, SY_FUEL, SB_RAIL, "Rail PCV current", "mA", 1, 0, 0.0f, 2000.0f, NOLO, NOLO, 1700.0f, 1900.0f, 0.0f, 2500.0f, 0.0f, 0, 0) \
  X(FRATE, SY_FUEL, SB_RAIL, "Fuel rate", "L/h", 10, 1, 0.0f, 200.0f, NOLO, NOLO, NOHI, NOHI, 0.0f, 400.0f, 0.0f, 0, 183) \
  X(INJ1, SY_INJ, SB_INJCORR, "Injector 1 corr", "mm3", 10, 1, -5.0f, 5.0f, -2.5f, -4.0f, 2.5f, 4.0f, -20.0f, 20.0f, 0.0f, 0, 651) \
  X(INJ2, SY_INJ, SB_INJCORR, "Injector 2 corr", "mm3", 10, 1, -5.0f, 5.0f, -2.5f, -4.0f, 2.5f, 4.0f, -20.0f, 20.0f, 0.0f, 0, 652) \
  X(INJ3, SY_INJ, SB_INJCORR, "Injector 3 corr", "mm3", 10, 1, -5.0f, 5.0f, -2.5f, -4.0f, 2.5f, 4.0f, -20.0f, 20.0f, 0.0f, 0, 653) \
  X(INJ4, SY_INJ, SB_INJCORR, "Injector 4 corr", "mm3", 10, 1, -5.0f, 5.0f, -2.5f, -4.0f, 2.5f, 4.0f, -20.0f, 20.0f, 0.0f, 0, 654) \
  X(INJ5, SY_INJ, SB_INJCORR, "Injector 5 corr", "mm3", 10, 1, -5.0f, 5.0f, -2.5f, -4.0f, 2.5f, 4.0f, -20.0f, 20.0f, 0.0f, 0, 655) \
  X(INJ6, SY_INJ, SB_INJCORR, "Injector 6 corr", "mm3", 10, 1, -5.0f, 5.0f, -2.5f, -4.0f, 2.5f, 4.0f, -20.0f, 20.0f, 0.0f, 0, 656) \
  X(CYB1, SY_INJ, SB_CYLBAL, "Cyl 1 balance", "rpm", 10, 1, -20.0f, 20.0f, -8.0f, -14.0f, 8.0f, 14.0f, -60.0f, 60.0f, 0.0f, 0, 0) \
  X(CYB2, SY_INJ, SB_CYLBAL, "Cyl 2 balance", "rpm", 10, 1, -20.0f, 20.0f, -8.0f, -14.0f, 8.0f, 14.0f, -60.0f, 60.0f, 0.0f, 0, 0) \
  X(CYB3, SY_INJ, SB_CYLBAL, "Cyl 3 balance", "rpm", 10, 1, -20.0f, 20.0f, -8.0f, -14.0f, 8.0f, 14.0f, -60.0f, 60.0f, 0.0f, 0, 0) \
  X(CYB4, SY_INJ, SB_CYLBAL, "Cyl 4 balance", "rpm", 10, 1, -20.0f, 20.0f, -8.0f, -14.0f, 8.0f, 14.0f, -60.0f, 60.0f, 0.0f, 0, 0) \
  X(CYB5, SY_INJ, SB_CYLBAL, "Cyl 5 balance", "rpm", 10, 1, -20.0f, 20.0f, -8.0f, -14.0f, 8.0f, 14.0f, -60.0f, 60.0f, 0.0f, 0, 0) \
  X(CYB6, SY_INJ, SB_CYLBAL, "Cyl 6 balance", "rpm", 10, 1, -20.0f, 20.0f, -8.0f, -14.0f, 8.0f, 14.0f, -60.0f, 60.0f, 0.0f, 0, 0) \
  X(SOI, SY_INJ, SB_INJTIM, "Start of inj", "deg", 10, 1, -10.0f, 30.0f, NOLO, NOLO, NOHI, NOHI, -30.0f, 50.0f, 0.0f, 0, 0) \
  X(INJQ, SY_INJ, SB_INJTIM, "Inj quantity", "mg", 10, 1, 0.0f, 300.0f, NOLO, NOLO, NOHI, NOHI, 0.0f, 500.0f, 0.0f, 0, 0) \
  X(ECT, SY_COOL, SB_COOLANT, "Coolant temp", "C", 10, 1, 0.0f, 130.0f, NOLO, NOLO, 100.0f, 110.0f, -40.0f, 150.0f, 20.0f, 0, 110) \
  X(COOLL, SY_COOL, SB_COOLANT, "Coolant level", "%", 1, 0, 0.0f, 100.0f, 40.0f, 20.0f, NOHI, NOHI, 0.0f, 105.0f, 0.0f, 0, 111) \
  X(COOLP, SY_COOL, SB_COOLANT, "Coolant press", "kPa", 1, 0, 0.0f, 300.0f, NOLO, NOLO, 200.0f, 250.0f, 0.0f, 400.0f, 150.0f, 0, 109) \
  X(RADT, SY_COOL, SB_COOLANT, "Radiator out", "C", 1, 0, 0.0f, 120.0f, NOLO, NOLO, 95.0f, 105.0f, -40.0f, 150.0f, 25.0f, 0, 0) \
  X(FANS, SY_COOL, SB_FANPUMP, "Fan speed", "rpm", 1, 0, 0.0f, 2500.0f, NOLO, NOLO, NOHI, NOHI, 0.0f, 3500.0f, 0.0f, 0, 1639) \
  X(WPMP, SY_COOL, SB_FANPUMP, "Water pump", "rpm", 1, 0, 0.0f, 4000.0f, NOLO, NOLO, NOHI, NOHI, 0.0f, 6000.0f, 0.0f, 0, 0) \
  X(THERM, SY_COOL, SB_FANPUMP, "Thermostat", "%", 1, 0, 0.0f, 100.0f, NOLO, NOLO, NOHI, NOHI, 0.0f, 105.0f, 0.0f, 0, 0) \
  X(EGT1, SY_EXH, SB_EGTS, "EGT pre-turbo", "C", 1, 0, 0.0f, 900.0f, NOLO, NOLO, 700.0f, 780.0f, -40.0f, 1100.0f, 150.0f, 0, 173) \
  X(EGT2, SY_EXH, SB_EGTS, "EGT post-turbo", "C", 1, 0, 0.0f, 800.0f, NOLO, NOLO, 600.0f, 700.0f, -40.0f, 1000.0f, 150.0f, 0, 0) \
  X(DOCI, SY_EXH, SB_EGTS, "DOC inlet temp", "C", 1, 0, 0.0f, 800.0f, NOLO, NOLO, 600.0f, 700.0f, -40.0f, 1000.0f, 150.0f, 0, 4766) \
  X(DOCO, SY_EXH, SB_EGTS, "DOC outlet temp", "C", 1, 0, 0.0f, 800.0f, NOLO, NOLO, 600.0f, 700.0f, -40.0f, 1000.0f, 150.0f, 0, 4768) \
  X(DPF, SY_EXH, SB_DPF, "DPF diff press", "kPa", 10, 1, 0.0f, 40.0f, NOLO, NOLO, 20.0f, 30.0f, 0.0f, 60.0f, 0.0f, 0, 3251) \
  X(SOOT, SY_EXH, SB_DPF, "DPF soot load", "%", 1, 0, 0.0f, 120.0f, NOLO, NOLO, 80.0f, 100.0f, 0.0f, 130.0f, 0.0f, 0, 3719) \
  X(DPFI, SY_EXH, SB_DPF, "DPF inlet temp", "C", 1, 0, 0.0f, 800.0f, NOLO, NOLO, 650.0f, 720.0f, -40.0f, 1000.0f, 150.0f, 0, 3242) \
  X(DPFO, SY_EXH, SB_DPF, "DPF outlet temp", "C", 1, 0, 0.0f, 800.0f, NOLO, NOLO, 650.0f, 750.0f, -40.0f, 1000.0f, 150.0f, 0, 3246) \
  X(REGEN, SY_EXH, SB_DPF, "DPF regen", "", 1, 0, 0.0f, 1.0f, NOLO, NOLO, NOHI, NOHI, 0.0f, 1.0f, 0.0f, 4, 3700) \
  X(DEFL, SY_EXH, SB_SCR, "AdBlue level", "%", 1, 0, 0.0f, 100.0f, 20.0f, 8.0f, NOHI, NOHI, 0.0f, 105.0f, 0.0f, 0, 1761) \
  X(DEFT, SY_EXH, SB_SCR, "AdBlue temp", "C", 1, 0, -20.0f, 80.0f, NOLO, NOLO, 60.0f, 70.0f, -40.0f, 100.0f, 30.0f, 0, 3031) \
  X(DEFQ, SY_EXH, SB_SCR, "AdBlue conc", "%", 10, 1, 0.0f, 40.0f, 29.0f, 25.0f, 36.0f, 38.0f, 0.0f, 50.0f, 0.0f, 0, 3516) \
  X(DEFD, SY_EXH, SB_SCR, "AdBlue dosing", "mL/min", 10, 1, 0.0f, 100.0f, NOLO, NOLO, NOHI, NOHI, 0.0f, 150.0f, 0.0f, 0, 0) \
  X(NOXI, SY_EXH, SB_SCR, "NOx upstream", "ppm", 1, 0, 0.0f, 1500.0f, NOLO, NOLO, 1200.0f, 1400.0f, 0.0f, 3000.0f, 0.0f, 0, 3216) \
  X(NOXO, SY_EXH, SB_SCR, "NOx downstream", "ppm", 1, 0, 0.0f, 600.0f, NOLO, NOLO, 250.0f, 400.0f, 0.0f, 3000.0f, 0.0f, 0, 3226) \
  X(SCRE, SY_EXH, SB_SCR, "SCR efficiency", "%", 1, 0, 0.0f, 100.0f, 80.0f, 60.0f, NOHI, NOHI, 0.0f, 105.0f, 0.0f, 1, 0) \
  X(GEAR, SY_TRN, SB_GEARBOX, "Current gear", "", 1, 0, 0.0f, 12.0f, NOLO, NOLO, NOHI, NOHI, 0.0f, 16.0f, 0.0f, 0, 523) \
  X(TRNT, SY_TRN, SB_GEARBOX, "Trans oil temp", "C", 1, 0, 20.0f, 150.0f, NOLO, NOLO, 100.0f, 120.0f, -40.0f, 180.0f, 20.0f, 0, 177) \
  X(TRNP, SY_TRN, SB_GEARBOX, "Trans oil press", "kPa", 1, 0, 0.0f, 2000.0f, 600.0f, 400.0f, NOHI, NOHI, 0.0f, 3000.0f, 800.0f, 1, 127) \
  X(TRNI, SY_TRN, SB_GEARBOX, "Input shaft", "rpm", 1, 0, 0.0f, 3000.0f, NOLO, NOLO, NOHI, NOHI, 0.0f, 5000.0f, 0.0f, 0, 161) \
  X(TRNO, SY_TRN, SB_GEARBOX, "Output shaft", "rpm", 1, 0, 0.0f, 3000.0f, NOLO, NOLO, NOHI, NOHI, 0.0f, 5000.0f, 0.0f, 0, 191) \
  X(CLSL, SY_TRN, SB_GEARBOX, "Clutch slip", "%", 1, 0, 0.0f, 100.0f, NOLO, NOLO, 20.0f, 40.0f, 0.0f, 105.0f, 0.0f, 0, 0) \
  X(AXT1, SY_TRN, SB_DRIVELINE, "Axle 1 oil temp", "C", 1, 0, 0.0f, 150.0f, NOLO, NOLO, 110.0f, 130.0f, -40.0f, 180.0f, 25.0f, 0, 0) \
  X(AXT2, SY_TRN, SB_DRIVELINE, "Axle 2 oil temp", "C", 1, 0, 0.0f, 150.0f, NOLO, NOLO, 110.0f, 130.0f, -40.0f, 180.0f, 25.0f, 0, 0) \
  X(WS1, SY_BRK, SB_WHEELSPD, "Wheel FL", "km/h", 10, 1, 0.0f, 140.0f, NOLO, NOLO, NOHI, NOHI, 0.0f, 300.0f, 40.0f, 0, 905) \
  X(WS2, SY_BRK, SB_WHEELSPD, "Wheel FR", "km/h", 10, 1, 0.0f, 140.0f, NOLO, NOLO, NOHI, NOHI, 0.0f, 300.0f, 40.0f, 0, 906) \
  X(WS3, SY_BRK, SB_WHEELSPD, "Wheel RL1", "km/h", 10, 1, 0.0f, 140.0f, NOLO, NOLO, NOHI, NOHI, 0.0f, 300.0f, 40.0f, 0, 907) \
  X(WS4, SY_BRK, SB_WHEELSPD, "Wheel RR1", "km/h", 10, 1, 0.0f, 140.0f, NOLO, NOLO, NOHI, NOHI, 0.0f, 300.0f, 40.0f, 0, 908) \
  X(WS5, SY_BRK, SB_WHEELSPD, "Wheel RL2", "km/h", 10, 1, 0.0f, 140.0f, NOLO, NOLO, NOHI, NOHI, 0.0f, 300.0f, 40.0f, 0, 909) \
  X(WS6, SY_BRK, SB_WHEELSPD, "Wheel RR2", "km/h", 10, 1, 0.0f, 140.0f, NOLO, NOLO, NOHI, NOHI, 0.0f, 300.0f, 40.0f, 0, 910) \
  X(BAIR1, SY_BRK, SB_AIRBRK, "Air tank 1", "kPa", 1, 0, 0.0f, 1000.0f, 600.0f, 450.0f, NOHI, NOHI, 0.0f, 1200.0f, 300.0f, 2, 117) \
  X(BAIR2, SY_BRK, SB_AIRBRK, "Air tank 2", "kPa", 1, 0, 0.0f, 1000.0f, 600.0f, 450.0f, NOHI, NOHI, 0.0f, 1200.0f, 300.0f, 2, 118) \
  X(BSUP, SY_BRK, SB_AIRBRK, "Air supply", "kPa", 1, 0, 0.0f, 1000.0f, 600.0f, 450.0f, NOHI, NOHI, 0.0f, 1200.0f, 300.0f, 0, 0) \
  X(BAPP, SY_BRK, SB_AIRBRK, "Brake app press", "kPa", 1, 0, 0.0f, 1000.0f, NOLO, NOLO, NOHI, NOHI, 0.0f, 1200.0f, 0.0f, 0, 116) \
  X(PARK, SY_BRK, SB_AIRBRK, "Parking brake", "", 1, 0, 0.0f, 1.0f, NOLO, NOLO, NOHI, NOHI, 0.0f, 1.0f, 0.0f, 4, 70) \
  X(BPED, SY_BRK, SB_PEDPAD, "Brake pedal", "%", 1, 0, 0.0f, 100.0f, NOLO, NOLO, NOHI, NOHI, 0.0f, 105.0f, 0.0f, 0, 521) \
  X(PADF, SY_BRK, SB_PEDPAD, "Front pad life", "%", 1, 0, 0.0f, 100.0f, 20.0f, 10.0f, NOHI, NOHI, 0.0f, 105.0f, 0.0f, 0, 0) \
  X(PADR, SY_BRK, SB_PEDPAD, "Rear pad life", "%", 1, 0, 0.0f, 100.0f, 20.0f, 10.0f, NOHI, NOHI, 0.0f, 105.0f, 0.0f, 0, 0) \
  X(BTF, SY_BRK, SB_PEDPAD, "Front brake T", "C", 1, 0, 0.0f, 600.0f, NOLO, NOLO, 350.0f, 450.0f, -40.0f, 800.0f, 200.0f, 0, 0) \
  X(BTR, SY_BRK, SB_PEDPAD, "Rear brake T", "C", 1, 0, 0.0f, 600.0f, NOLO, NOLO, 350.0f, 450.0f, -40.0f, 800.0f, 200.0f, 0, 0) \
  X(RETD, SY_BRK, SB_PEDPAD, "Retarder torque", "%", 1, 0, 0.0f, 100.0f, NOLO, NOLO, NOHI, NOHI, 0.0f, 110.0f, 0.0f, 0, 520) \
  X(SANG, SY_STR, SB_STEER, "Steering angle", "deg", 1, 0, -540.0f, 540.0f, NOLO, NOLO, NOHI, NOHI, -720.0f, 720.0f, 0.0f, 0, 1807) \
  X(STOR, SY_STR, SB_STEER, "Steer torque", "Nm", 10, 1, -10.0f, 10.0f, -8.0f, -9.5f, 8.0f, 9.5f, -15.0f, 15.0f, 0.0f, 0, 0) \
  X(EPSA, SY_STR, SB_STEER, "EPS motor curr", "A", 10, 1, 0.0f, 60.0f, NOLO, NOLO, 45.0f, 55.0f, 0.0f, 80.0f, 0.0f, 0, 0) \
  X(SRATE, SY_STR, SB_STEER, "Steer rate", "deg/s", 1, 0, -400.0f, 400.0f, NOLO, NOLO, NOHI, NOHI, -900.0f, 900.0f, 0.0f, 0, 0) \
  X(ASFL, SY_STR, SB_SUSP, "Air spring FL", "kPa", 1, 0, 0.0f, 1000.0f, 250.0f, 150.0f, 800.0f, 900.0f, 0.0f, 1100.0f, 300.0f, 0, 0) \
  X(ASFR, SY_STR, SB_SUSP, "Air spring FR", "kPa", 1, 0, 0.0f, 1000.0f, 250.0f, 150.0f, 800.0f, 900.0f, 0.0f, 1100.0f, 300.0f, 0, 0) \
  X(ASRL, SY_STR, SB_SUSP, "Air spring RL", "kPa", 1, 0, 0.0f, 1000.0f, 250.0f, 150.0f, 800.0f, 900.0f, 0.0f, 1100.0f, 300.0f, 0, 0) \
  X(ASRR, SY_STR, SB_SUSP, "Air spring RR", "kPa", 1, 0, 0.0f, 1000.0f, 250.0f, 150.0f, 800.0f, 900.0f, 0.0f, 1100.0f, 300.0f, 0, 0) \
  X(RHF, SY_STR, SB_SUSP, "Ride height F", "mm", 1, 0, -100.0f, 100.0f, -40.0f, -70.0f, 40.0f, 70.0f, -200.0f, 200.0f, 0.0f, 0, 0) \
  X(RHR, SY_STR, SB_SUSP, "Ride height R", "mm", 1, 0, -100.0f, 100.0f, -40.0f, -70.0f, 40.0f, 70.0f, -200.0f, 200.0f, 0.0f, 0, 0) \
  X(TP1, SY_TYR, SB_TYREP, "Tyre FL press", "kPa", 1, 0, 0.0f, 1100.0f, 750.0f, 600.0f, 1000.0f, 1050.0f, 0.0f, 1400.0f, 200.0f, 0, 241) \
  X(TP2, SY_TYR, SB_TYREP, "Tyre FR press", "kPa", 1, 0, 0.0f, 1100.0f, 750.0f, 600.0f, 1000.0f, 1050.0f, 0.0f, 1400.0f, 200.0f, 0, 241) \
  X(TP3, SY_TYR, SB_TYREP, "Tyre RL1 press", "kPa", 1, 0, 0.0f, 1100.0f, 750.0f, 600.0f, 1000.0f, 1050.0f, 0.0f, 1400.0f, 200.0f, 0, 241) \
  X(TP4, SY_TYR, SB_TYREP, "Tyre RR1 press", "kPa", 1, 0, 0.0f, 1100.0f, 750.0f, 600.0f, 1000.0f, 1050.0f, 0.0f, 1400.0f, 200.0f, 0, 241) \
  X(TP5, SY_TYR, SB_TYREP, "Tyre RL2 press", "kPa", 1, 0, 0.0f, 1100.0f, 750.0f, 600.0f, 1000.0f, 1050.0f, 0.0f, 1400.0f, 200.0f, 0, 241) \
  X(TP6, SY_TYR, SB_TYREP, "Tyre RR2 press", "kPa", 1, 0, 0.0f, 1100.0f, 750.0f, 600.0f, 1000.0f, 1050.0f, 0.0f, 1400.0f, 200.0f, 0, 241) \
  X(TT1, SY_TYR, SB_TYRET, "Tyre FL temp", "C", 1, 0, -10.0f, 150.0f, NOLO, NOLO, 90.0f, 110.0f, -40.0f, 200.0f, 30.0f, 0, 242) \
  X(TT2, SY_TYR, SB_TYRET, "Tyre FR temp", "C", 1, 0, -10.0f, 150.0f, NOLO, NOLO, 90.0f, 110.0f, -40.0f, 200.0f, 30.0f, 0, 242) \
  X(TT3, SY_TYR, SB_TYRET, "Tyre RL1 temp", "C", 1, 0, -10.0f, 150.0f, NOLO, NOLO, 90.0f, 110.0f, -40.0f, 200.0f, 30.0f, 0, 242) \
  X(TT4, SY_TYR, SB_TYRET, "Tyre RR1 temp", "C", 1, 0, -10.0f, 150.0f, NOLO, NOLO, 90.0f, 110.0f, -40.0f, 200.0f, 30.0f, 0, 242) \
  X(TT5, SY_TYR, SB_TYRET, "Tyre RL2 temp", "C", 1, 0, -10.0f, 150.0f, NOLO, NOLO, 90.0f, 110.0f, -40.0f, 200.0f, 30.0f, 0, 242) \
  X(TT6, SY_TYR, SB_TYRET, "Tyre RR2 temp", "C", 1, 0, -10.0f, 150.0f, NOLO, NOLO, 90.0f, 110.0f, -40.0f, 200.0f, 30.0f, 0, 242) \
  X(BATV, SY_ELE, SB_BATT, "Battery volt", "V", 100, 1, 16.0f, 32.0f, 24.0f, 22.0f, 29.5f, 31.0f, 10.0f, 40.0f, 6.0f, 1, 168) \
  X(BATI, SY_ELE, SB_BATT, "Battery current", "A", 1, 0, -200.0f, 200.0f, NOLO, NOLO, NOHI, NOHI, -500.0f, 500.0f, 0.0f, 0, 114) \
  X(BATT, SY_ELE, SB_BATT, "Battery temp", "C", 1, 0, -20.0f, 80.0f, NOLO, NOLO, 55.0f, 65.0f, -40.0f, 100.0f, 15.0f, 0, 0) \
  X(SOC, SY_ELE, SB_BATT, "Battery SOC", "%", 1, 0, 0.0f, 100.0f, 40.0f, 20.0f, NOHI, NOHI, 0.0f, 105.0f, 0.0f, 0, 0) \
  X(ALTV, SY_ELE, SB_CHARGE, "Alternator volt", "V", 100, 1, 0.0f, 32.0f, 26.0f, 24.0f, 29.5f, 31.0f, 0.0f, 40.0f, 6.0f, 1, 167) \
  X(ALTC, SY_ELE, SB_CHARGE, "Alternator load", "A", 1, 0, 0.0f, 150.0f, NOLO, NOLO, 110.0f, 135.0f, 0.0f, 250.0f, 0.0f, 0, 115) \
  X(ALTT, SY_ELE, SB_CHARGE, "Alternator temp", "C", 1, 0, 0.0f, 160.0f, NOLO, NOLO, 110.0f, 130.0f, -40.0f, 200.0f, 40.0f, 0, 0) \
  X(SPEED, SY_VEH, SB_MOTION, "Vehicle speed", "km/h", 10, 1, 0.0f, 140.0f, NOLO, NOLO, 85.0f, 100.0f, 0.0f, 300.0f, 60.0f, 0, 84) \
  X(ACCEL, SY_VEH, SB_MOTION, "Acceleration", "m/s2", 100, 2, -10.0f, 5.0f, -4.0f, -6.0f, NOHI, NOHI, -15.0f, 10.0f, 0.0f, 0, 0) \
  X(GRADE, SY_VEH, SB_MOTION, "Road grade", "%", 10, 1, -20.0f, 20.0f, NOLO, NOLO, NOHI, NOHI, -40.0f, 40.0f, 0.0f, 0, 0) \
  X(FUELE, SY_VEH, SB_MOTION, "Fuel economy", "km/L", 10, 1, 0.0f, 10.0f, NOLO, NOLO, NOHI, NOHI, 0.0f, 30.0f, 0.0f, 0, 184) \
  X(AMBT, SY_VEH, SB_MOTION, "Ambient temp", "C", 1, 0, -10.0f, 60.0f, NOLO, NOLO, NOHI, NOHI, -40.0f, 85.0f, 0.0f, 0, 171) \
  X(CABT, SY_VEH, SB_CAB, "Cab temp", "C", 1, 0, 0.0f, 50.0f, NOLO, NOLO, NOHI, NOHI, -40.0f, 85.0f, 0.0f, 0, 170) \
  X(DERATE, SY_VEH, SB_CAB, "Power derate", "", 1, 0, 0.0f, 1.0f, NOLO, NOLO, 0.5f, 0.5f, 0.0f, 1.0f, 0.0f, 4, 0) \
  X(MIL, SY_VEH, SB_CAB, "MIL lamp", "", 1, 0, 0.0f, 1.0f, NOLO, NOLO, 0.5f, NOHI, 0.0f, 1.0f, 0.0f, 4, 0) \
  X(CRUISE, SY_VEH, SB_CAB, "Cruise control", "", 1, 0, 0.0f, 1.0f, NOLO, NOLO, NOHI, NOHI, 0.0f, 1.0f, 0.0f, 4, 0) \
  X(FU1, SY_FUSE, SB_FUSE_ENG, "ECU fuse F1", "", 1, 0, 0.0f, 1.0f, NOLO, 0.5f, NOHI, NOHI, 0.0f, 1.0f, 0.0f, 10, 0) \
  X(FU2, SY_FUSE, SB_FUSE_ENG, "Fuel pump F2", "", 1, 0, 0.0f, 1.0f, NOLO, 0.5f, NOHI, NOHI, 0.0f, 1.0f, 0.0f, 10, 0) \
  X(FU3, SY_FUSE, SB_FUSE_ENG, "Injector F3", "", 1, 0, 0.0f, 1.0f, NOLO, 0.5f, NOHI, NOHI, 0.0f, 1.0f, 0.0f, 10, 0) \
  X(FU4, SY_FUSE, SB_FUSE_ENG, "Glow plug F4", "", 1, 0, 0.0f, 1.0f, NOLO, 0.5f, NOHI, NOHI, 0.0f, 1.0f, 0.0f, 8, 0) \
  X(FU5, SY_FUSE, SB_FUSE_ENG, "Starter F5", "", 1, 0, 0.0f, 1.0f, NOLO, 0.5f, NOHI, NOHI, 0.0f, 1.0f, 0.0f, 8, 0) \
  X(FU6, SY_FUSE, SB_FUSE_ENG, "Alternator F6", "", 1, 0, 0.0f, 1.0f, NOLO, 0.5f, NOHI, NOHI, 0.0f, 1.0f, 0.0f, 8, 0) \
  X(FU7, SY_FUSE, SB_FUSE_CHS, "ABS fuse F7", "", 1, 0, 0.0f, 1.0f, NOLO, 0.5f, NOHI, NOHI, 0.0f, 1.0f, 0.0f, 10, 0) \
  X(FU8, SY_FUSE, SB_FUSE_CHS, "Headlight F8", "", 1, 0, 0.0f, 1.0f, NOLO, 0.5f, NOHI, NOHI, 0.0f, 1.0f, 0.0f, 8, 0) \
  X(FU9, SY_FUSE, SB_FUSE_CHS, "Brake light F9", "", 1, 0, 0.0f, 1.0f, NOLO, 0.5f, NOHI, NOHI, 0.0f, 1.0f, 0.0f, 8, 0) \
  X(FU10, SY_FUSE, SB_FUSE_CHS, "Wiper F10", "", 1, 0, 0.0f, 1.0f, NOLO, 0.5f, NOHI, NOHI, 0.0f, 1.0f, 0.0f, 8, 0) \
  X(FU11, SY_FUSE, SB_FUSE_CHS, "Gearbox TCU F11", "", 1, 0, 0.0f, 1.0f, NOLO, 0.5f, NOHI, NOHI, 0.0f, 1.0f, 0.0f, 10, 0) \
  X(FU12, SY_FUSE, SB_FUSE_CHS, "Horn F12", "", 1, 0, 0.0f, 1.0f, NOLO, 0.5f, NOHI, NOHI, 0.0f, 1.0f, 0.0f, 8, 0) \
  X(RL1, SY_FUSE, SB_RELAYS, "Main relay K1", "", 1, 0, 0.0f, 1.0f, NOLO, 0.5f, NOHI, NOHI, 0.0f, 1.0f, 0.0f, 18, 0) \
  X(RL2, SY_FUSE, SB_RELAYS, "Fuel pump K2", "", 1, 0, 0.0f, 1.0f, NOLO, 0.5f, NOHI, NOHI, 0.0f, 1.0f, 0.0f, 18, 0) \
  X(RL3, SY_FUSE, SB_RELAYS, "Starter K3", "", 1, 0, 0.0f, 1.0f, NOLO, 0.5f, NOHI, NOHI, 0.0f, 1.0f, 0.0f, 16, 0) \
  X(RL4, SY_FUSE, SB_RELAYS, "Cooling fan K4", "", 1, 0, 0.0f, 1.0f, NOLO, 0.5f, NOHI, NOHI, 0.0f, 1.0f, 0.0f, 16, 0) \
  X(RL5, SY_FUSE, SB_RELAYS, "Glow plug K5", "", 1, 0, 0.0f, 1.0f, NOLO, 0.5f, NOHI, NOHI, 0.0f, 1.0f, 0.0f, 16, 0) \
  X(RL6, SY_FUSE, SB_RELAYS, "Headlight K6", "", 1, 0, 0.0f, 1.0f, NOLO, 0.5f, NOHI, NOHI, 0.0f, 1.0f, 0.0f, 16, 0) \
  X(RL7, SY_FUSE, SB_RELAYS, "Brake light K7", "", 1, 0, 0.0f, 1.0f, NOLO, 0.5f, NOHI, NOHI, 0.0f, 1.0f, 0.0f, 16, 0) \
  X(RL8, SY_FUSE, SB_RELAYS, "A/C compress K8", "", 1, 0, 0.0f, 1.0f, NOLO, 0.5f, NOHI, NOHI, 0.0f, 1.0f, 0.0f, 16, 0)

#define X_ID(id, ...) SN_##id,
enum SensId { SENSOR_LIST(X_ID) NSENS };

struct SensInfo { uint8_t sys, sub; const char* key; const char* name; const char* unit; int16_t scale; uint8_t dec;
  float lo, hi, wLo, cLo, wHi, cHi, pmin, pmax, jump; uint8_t flags; uint16_t spn; };
#define X_ROW(id, sy, sb, nm, un, sc, dc, lo, hi, wl, cl, wh, ch, pmn, pmx, jm, fl, sp) \
  { (uint8_t)(sy), (uint8_t)(sb), #id, nm, un, sc, dc, lo, hi, wl, cl, wh, ch, pmn, pmx, jm, fl, sp },
static const SensInfo SENS[NSENS] = { SENSOR_LIST(X_ROW) };

static const uint16_t ICONS[NICONS][16] = {
  {0x0000, 0x0000, 0x0000, 0x07E0, 0x07E0, 0x07E0, 0x1FF8, 0x1FF8, 0xFDBF, 0xFDBF, 0xFDB8, 0x1DB8, 0x1FF8, 0x0FF0, 0x0FF0, 0x0000},  // ENGINE
  {0x0000, 0x03C0, 0x0D30, 0x11F8, 0x21E4, 0x21C4, 0x7982, 0x7FC2, 0x7FC2, 0x79E2, 0x20FC, 0x20E4, 0x1088, 0x0C30, 0x03C0, 0x0000},  // AIR
  {0x0000, 0x3FC0, 0x2040, 0x2040, 0x2040, 0x2078, 0x2048, 0x3FCA, 0x3FCA, 0x3FCA, 0x3FDC, 0x3FDC, 0x3FDC, 0x3FC0, 0xFFF0, 0xFFF0},  // FUEL
  {0x07E0, 0x07E0, 0x07E0, 0x07E0, 0x03C0, 0x03C0, 0x03C0, 0x03C0, 0x03C0, 0x03C0, 0x0380, 0x0990, 0x0520, 0x1008, 0x0240, 0x0100},  // INJ
  {0x0002, 0x0025, 0x0052, 0x002C, 0x0012, 0x0012, 0x000C, 0x0010, 0x00F0, 0xFFF0, 0xFFF0, 0xFFF0, 0xFFF0, 0x00F0, 0x0010, 0x0000},  // EXHAUST
  {0x0380, 0x7BBC, 0x7BFC, 0x7FFC, 0x7FFC, 0x1E78, 0xFC3F, 0xF81F, 0xF81F, 0x3C3C, 0x7E7C, 0x7FFC, 0x7FFC, 0x7BFC, 0x0380, 0x0380},  // TRANS
  {0x0000, 0x0F80, 0x1FC0, 0x307F, 0x70BF, 0xC707, 0xCF87, 0xCF87, 0xCF87, 0xD787, 0x603F, 0x307F, 0x1FC0, 0x0F80, 0x0000, 0x0000},  // BRAKES
  {0x07E0, 0x0BD0, 0x3C3C, 0x300C, 0x6006, 0xA185, 0xC3C3, 0xFFFF, 0xFFFF, 0xC3C3, 0xA185, 0x6186, 0x318C, 0x3DBC, 0x0BD0, 0x07E0},  // STEER
  {0x0000, 0x0000, 0x381C, 0x381C, 0xFFFF, 0xFFBF, 0xFF3F, 0xFE3F, 0xFC07, 0xF80F, 0xF01F, 0xFE7F, 0xFCFF, 0xFDFF, 0xFFFF, 0x0000},  // ELEC
  {0x0000, 0x0000, 0x0000, 0xFF80, 0xFF80, 0xFFF8, 0xFFE4, 0xFFE6, 0xFFE3, 0xFFFF, 0xFFFF, 0xFFFF, 0x3636, 0x3E3E, 0x1C1C, 0x0000},  // VEHICLE
  {0x1F00, 0x2E80, 0x71C0, 0xA0A0, 0xC060, 0xDF60, 0xC060, 0xBEA0, 0x71C0, 0x2EE0, 0x1F70, 0x0038, 0x001C, 0x000E, 0x0006, 0x0000},  // SCAN
  {0x0100, 0x0180, 0x0380, 0x03C0, 0x07C0, 0x0660, 0x0E60, 0x0E70, 0x1E78, 0x1E78, 0x3FFC, 0x3E7C, 0x7E7E, 0x7FFE, 0xFFFF, 0x0000},  // WARN
  {0x0000, 0x0000, 0xFFFF, 0xFFFF, 0xC003, 0xC183, 0xC3C3, 0xC3C3, 0xC183, 0xFFFF, 0xC203, 0xC203, 0xC203, 0xFFFF, 0xFFFF, 0x0000},  // BOX
  {0x0920, 0x0920, 0x0920, 0x1FF8, 0xFFFF, 0x1818, 0x1818, 0xF99F, 0x1998, 0x1818, 0xF81F, 0x1FF8, 0x1FF8, 0x0920, 0x0920, 0x0920},  // STATUS
  {0x0000, 0x0000, 0x0002, 0x0006, 0x000E, 0x001C, 0x0038, 0x4070, 0x60E0, 0x31C0, 0x1B80, 0x0F00, 0x0600, 0x0000, 0x0000, 0x0000},  // OK
  {0x0180, 0x0180, 0x01B0, 0x0180, 0x01B0, 0x0180, 0x01B0, 0x0180, 0x0180, 0x0180, 0x03C0, 0x07E0, 0x07E0, 0x07E0, 0x03C0, 0x0000},  // THERMO
  {0x07E0, 0x1FF8, 0x3FFC, 0x7C3E, 0x781E, 0xF18F, 0xE3C7, 0xE7E7, 0xE7E7, 0xE3C7, 0xF18F, 0x781E, 0x7C3E, 0x3FFC, 0x1FF8, 0x07E0},  // TYRE
  {0x0000, 0x1FF8, 0x3FFC, 0x300C, 0x300C, 0x300C, 0x3FFC, 0x7FFE, 0x7FFE, 0x7FFE, 0xFFFF, 0xFFFF, 0x7FFE, 0x6666, 0x6666, 0x0000},  // CAB
  {0x0000, 0x0000, 0x0000, 0x0000, 0x7FFE, 0xE007, 0xE3C7, 0xE7E7, 0xE3C7, 0xE007, 0x7FFE, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000},  // FUSE
  {0x03F0, 0x0C0C, 0x1002, 0x2001, 0x2009, 0x4011, 0x4021, 0x4041, 0x40C1, 0x4001, 0x4001, 0x2001, 0x2002, 0x1004, 0x0C0C, 0x03F0},  // DIAL
  {0x003C, 0x0078, 0x00F0, 0x01E0, 0x03C0, 0x07F0, 0x0FF8, 0x0078, 0x00F0, 0x01E0, 0x03C0, 0x0700, 0x0E00, 0x1800, 0x1000, 0x0000},  // BOLT
  {0x0000, 0x0FF0, 0x0810, 0x0816, 0x0810, 0x0810, 0x0816, 0x0810, 0x0BD0, 0x0BD6, 0x0BD0, 0x0BD0, 0x0BD6, 0x0BD0, 0x0FF0, 0x0000},  // LEVEL
  {0x3FF0, 0x2018, 0x2014, 0x201E, 0x2002, 0x2FC2, 0x2002, 0x2FC2, 0x2002, 0x2FC2, 0x2002, 0x2F02, 0x2002, 0x3FFE, 0x0000, 0x0000},  // DOC
};

// ============================ 6. TYPES ====================================
#define INV ((int16_t)-32768)               // "no signal" marker
struct __attribute__((packed)) Sample { uint32_t t; int16_t v[NSENS]; uint8_t flags; };
static_assert(sizeof(Sample) == 4 + 2 * NSENS + 1, "Sample size");
#define FLAG_MIL 1

#define MAXF 16
#define BB_MAGIC 0x33424F42UL
#define BB_VERSION 4
struct __attribute__((packed)) IncHeader {
  uint32_t magic; uint16_t version; uint16_t id; uint32_t epoch; uint32_t trigMs;
  uint16_t trigIndex; uint16_t count; uint16_t trigSpd10; uint16_t trigRpm; uint8_t nf; uint8_t sev;
  uint8_t  faults[2 * MAXF];                // { fault id, severity } x MAXF
  uint16_t fIdx[MAXF];                      // sample index where each fault crossed its limit
  uint16_t fRpm[MAXF];                      // engine rpm at that moment
  uint16_t fSpd10[MAXF];                    // vehicle speed x10 at that moment
  uint32_t fMs[MAXF];                       // time since boot (ms)
  uint32_t fEpoch[MAXF];                    // wall clock (0 = clock not set)
  float    fLim[MAXF];                      // the limit that was crossed
};
struct IncMeta { uint16_t id; uint32_t epoch; uint32_t trigMs; uint16_t spd10; uint16_t count; uint8_t nf; uint8_t sev;
  uint8_t faults[2 * MAXF]; uint16_t fRpm[MAXF]; uint16_t fSpd10[MAXF]; uint32_t fMs[MAXF]; uint32_t fEpoch[MAXF]; };

// A "fault id" = sensor*4 + kind
#define K_LOW      0
#define K_HIGH     1
#define K_SENSOR   2
#define K_MISMATCH 3
#define FID(s, k)  ((s) * 4 + (k))
#define NFAULT     ((NSENS + 1) * 4)
#define F_LINK     FID(NSENS, K_HIGH)       // virtual fault: ECU link lost
static const char* const KIND_NAME[4] = {"LOW", "HIGH", "SENSOR", "MISMATCH"};
static const char* const KIND_SHORT[4] = {"LOW", "HIGH", "SENSOR", "DIFF"};

enum Screen : uint8_t { S_DRIVER, S_MENU, S_SCAN, S_SYSTEMS, S_SUBS, S_SENSORS, S_SENSOR, S_HIST, S_CODES,
                        S_INC_LIST, S_INC_DETAIL, S_SNAP, S_REPLAY, S_STORAGE, S_STATUS, S_TOUCH, S_CAL, S_FLOG, S_FEVT, S_HOME, S_APP, S_SCANRUN, S_REPORT };
enum IncState : uint8_t { INC_IDLE, INC_REC };

struct Btn { uint8_t pin; bool raw; bool down; uint32_t tEdge; uint32_t tDown; uint32_t lastRep; };
struct TgMsg { char text[1500]; };
struct Row { char a[24]; char b[20]; uint16_t ac, bc, dot; int8_t icon; uint16_t ibg; int16_t tag; };
struct Nav { Screen s; int lcur, ltop, sys, sub, sens; };

// report entry (kept up here so Arduino's auto-generated prototypes can see it)
struct Rep { uint16_t fid; uint8_t sev, state; uint32_t epoch, ms; float val, lim; uint8_t hasVal; uint16_t spd10, rpm; uint32_t durS; };

static const char* const MORSE_TAB[36] = {
  ".-", "-...", "-.-.", "-..", ".", "..-.", "--.", "....", "..", ".---", "-.-", ".-..", "--",
  "-.", "---", ".--.", "--.-", ".-.", "...", "-", "..-", "...-", ".--", "-..-", "-.--", "--..",
  "-----", ".----", "..---", "...--", "....-", ".....", "-....", "--...", "---..", "----."
};

static const uint8_t font5x7[95][5] = {
  {0x00,0x00,0x00,0x00,0x00}, {0x00,0x00,0x5F,0x00,0x00}, {0x00,0x07,0x00,0x07,0x00}, {0x14,0x7F,0x14,0x7F,0x14},
  {0x24,0x2A,0x7F,0x2A,0x12}, {0x23,0x13,0x08,0x64,0x62}, {0x36,0x49,0x56,0x20,0x50}, {0x00,0x08,0x07,0x03,0x00},
  {0x00,0x1C,0x22,0x41,0x00}, {0x00,0x41,0x22,0x1C,0x00}, {0x2A,0x1C,0x7F,0x1C,0x2A}, {0x08,0x08,0x3E,0x08,0x08},
  {0x00,0x80,0x70,0x30,0x00}, {0x08,0x08,0x08,0x08,0x08}, {0x00,0x00,0x60,0x60,0x00}, {0x20,0x10,0x08,0x04,0x02},
  {0x3E,0x51,0x49,0x45,0x3E}, {0x00,0x42,0x7F,0x40,0x00}, {0x72,0x49,0x49,0x49,0x46}, {0x21,0x41,0x49,0x4D,0x33},
  {0x18,0x14,0x12,0x7F,0x10}, {0x27,0x45,0x45,0x45,0x39}, {0x3C,0x4A,0x49,0x49,0x31}, {0x41,0x21,0x11,0x09,0x07},
  {0x36,0x49,0x49,0x49,0x36}, {0x46,0x49,0x49,0x29,0x1E}, {0x00,0x00,0x14,0x00,0x00}, {0x00,0x40,0x34,0x00,0x00},
  {0x00,0x08,0x14,0x22,0x41}, {0x14,0x14,0x14,0x14,0x14}, {0x00,0x41,0x22,0x14,0x08}, {0x02,0x01,0x59,0x09,0x06},
  {0x3E,0x41,0x5D,0x59,0x4E}, {0x7C,0x12,0x11,0x12,0x7C}, {0x7F,0x49,0x49,0x49,0x36}, {0x3E,0x41,0x41,0x41,0x22},
  {0x7F,0x41,0x41,0x41,0x3E}, {0x7F,0x49,0x49,0x49,0x41}, {0x7F,0x09,0x09,0x09,0x01}, {0x3E,0x41,0x41,0x51,0x73},
  {0x7F,0x08,0x08,0x08,0x7F}, {0x00,0x41,0x7F,0x41,0x00}, {0x20,0x40,0x41,0x3F,0x01}, {0x7F,0x08,0x14,0x22,0x41},
  {0x7F,0x40,0x40,0x40,0x40}, {0x7F,0x02,0x1C,0x02,0x7F}, {0x7F,0x04,0x08,0x10,0x7F}, {0x3E,0x41,0x41,0x41,0x3E},
  {0x7F,0x09,0x09,0x09,0x06}, {0x3E,0x41,0x51,0x21,0x5E}, {0x7F,0x09,0x19,0x29,0x46}, {0x26,0x49,0x49,0x49,0x32},
  {0x03,0x01,0x7F,0x01,0x03}, {0x3F,0x40,0x40,0x40,0x3F}, {0x1F,0x20,0x40,0x20,0x1F}, {0x3F,0x40,0x38,0x40,0x3F},
  {0x63,0x14,0x08,0x14,0x63}, {0x03,0x04,0x78,0x04,0x03}, {0x61,0x59,0x49,0x4D,0x43}, {0x00,0x7F,0x41,0x41,0x41},
  {0x02,0x04,0x08,0x10,0x20}, {0x00,0x41,0x41,0x41,0x7F}, {0x04,0x02,0x01,0x02,0x04}, {0x40,0x40,0x40,0x40,0x40},
  {0x00,0x03,0x07,0x08,0x00}, {0x20,0x54,0x54,0x78,0x40}, {0x7F,0x28,0x44,0x44,0x38}, {0x38,0x44,0x44,0x44,0x28},
  {0x38,0x44,0x44,0x28,0x7F}, {0x38,0x54,0x54,0x54,0x18}, {0x00,0x08,0x7E,0x09,0x02}, {0x18,0xA4,0xA4,0x9C,0x78},
  {0x7F,0x08,0x04,0x04,0x78}, {0x00,0x44,0x7D,0x40,0x00}, {0x20,0x40,0x40,0x3D,0x00}, {0x7F,0x10,0x28,0x44,0x00},
  {0x00,0x41,0x7F,0x40,0x00}, {0x7C,0x04,0x78,0x04,0x78}, {0x7C,0x08,0x04,0x04,0x78}, {0x38,0x44,0x44,0x44,0x38},
  {0xFC,0x18,0x24,0x24,0x18}, {0x18,0x24,0x24,0x18,0xFC}, {0x7C,0x08,0x04,0x04,0x08}, {0x48,0x54,0x54,0x54,0x24},
  {0x04,0x04,0x3F,0x44,0x24}, {0x3C,0x40,0x40,0x20,0x7C}, {0x1C,0x20,0x40,0x20,0x1C}, {0x3C,0x40,0x30,0x40,0x3C},
  {0x44,0x28,0x10,0x28,0x44}, {0x4C,0x90,0x90,0x90,0x7C}, {0x44,0x64,0x54,0x4C,0x44}, {0x00,0x08,0x36,0x41,0x00},
  {0x00,0x00,0x77,0x00,0x00}, {0x00,0x41,0x36,0x08,0x00}, {0x02,0x01,0x02,0x04,0x02}
};

// ============================ 7. GLOBALS ==================================
static uint16_t* fb = nullptr;                 // frame buffer (byte-swapped RGB565)

// buffer sizes (decided at boot: PSRAM or not)
static int g_preN = 100, g_postN = 50, g_incMax = 150;

// live ECU data + rule state
static Sample   latest;
static bool     haveData = false;
static uint8_t  lastSrc = 0;                   // 1 = USB serial, 2 = WiFi, 3 = UART link (S3 fake ECU)
static uint32_t rxBad = 0; static int parseGot = 0;   // bad D lines and how many values the last bad line had
static uint32_t lastDataMs = 0, rxLines = 0, epochBase = 0;
static Sample*  ring = nullptr;
static int      ringHead = 0, ringCount = 0;
static float    thr[NSENS][4];                 // live limits: wLo, cLo, wHi, cHi (editable from the PC, kept in flash)
static bool     thrDirty = false; static uint32_t thrDirtyMs = 0;
#define WLO(i) thr[i][0]
#define CLO(i) thr[i][1]
#define WHI(i) thr[i][2]
#define CHI(i) thr[i][3]
static uint8_t  fSev[NFAULT], fCnt[NFAULT], fClr[NFAULT];
static uint16_t fHist[NFAULT];
static int      fLastMeta[NFAULT];
static uint32_t fAlertMs[NFAULT];
static uint8_t  fAlertSev[NFAULT];
static float    prevVal[NSENS]; static bool havePrev[NSENS]; static uint32_t glitchUntil[NSENS];
static uint8_t  raisedList[2 * 160]; static int nRaised = 0, raisedOverflow = 0;
static bool     fMuted = false;

// pending Telegram problems (batched into one message)
static uint8_t  pend[2 * 48]; static int nPend = 0, pendExtra = 0; static uint32_t pendSince = 0; static uint16_t pendId = 0;
static Sample   pendRef;
static uint32_t lastRemind = 0; static int lastCleaned = 0;

// incident recorder
static Sample*   incBuf = nullptr;
static IncHeader incHdr;
static int       incCount = 0, incRemain = 0;
static IncState  incState = INC_IDLE;
static bool      justStarted = false;
static IncMeta   meta[MAX_INC];
static int       metaN = 0;
static uint16_t  nextId = 1;

// viewer data (one incident)
static Sample* viewBuf = nullptr;
static IncHeader viewHdr;
static int     viewCount = 0, viewTrig = -1;

// UI
static Screen   screen = S_DRIVER;
static bool     dirty = true;
static uint32_t lastRender = 0;
static Row      rows[MAXROWS];
static int      nRows = 0, lcur = 0, ltop = 0;
static int      rowH = 17, listY = 16, listVis = 8;
static int      curSys = 0, curSub = 0, curSens = 0, detMeta = 0, fevtSel = 0;
static Nav      navStack[16]; static int navN = 0;
static int      vsig = 0, curCol = 0;
static int      iLcur = -1, iLtop = 0;         // incident list cursor (-1 = search bar)
static int      viewIdx[MAX_INC]; static int viewN = 0;
static bool     typing = false;
static char     search[13] = "";
static char     mbuf[8] = ""; static int mlen = 0; static uint32_t mLast = 0;
static int      confirmArm = 0;                // storage screen: tag+1 of the action waiting for a 2nd OK
static char     toastMsg[28] = ""; static uint32_t toastUntil = 0;

// inputs
static Btn bL = {PIN_BTN_LEFT, false, false, 0, 0, 0};
static Btn bR = {PIN_BTN_RIGHT, false, false, 0, 0, 0};
static Btn bM = {PIN_BTN_MORSE, false, false, 0, 0, 0};
static bool comboArmed = false, comboFired = false, supL = false, supR = false;
static uint32_t comboStart = 0;
static bool tDown = false, tMoved = false; static int tx0 = 0, ty0 = 0, tCnt = 0, tUp = 0; static uint32_t tStart = 0, tLastPoll = 0;

// wifi + telegram
static WebServer web(80);
static bool staEnabled = false;
static QueueHandle_t tgQ = nullptr;
static bool tgEnabled = false;
static volatile int tgState = 0;               // 0 off, 1 no wifi, 2 ok, 3 last send failed
static volatile int tgCode = 0;
static volatile uint16_t tgSent = 0, tgDropped = 0;
static uint32_t tgHash[4] = {0, 0, 0, 0}; static uint32_t tgHashMs[4] = {0, 0, 0, 0};

static char lineBuf[1700]; static int lineLen = 0;
static char lineBuf2[1700]; static int lineLen2 = 0;       // UART link line buffer

static void rebuildFilter();
static void finishIncident();
static void processLine(char* line, uint8_t src);
static void printCatalog(Print& out);

// ============================ 8. TFT DRIVER (no library) ==================
static inline uint16_t swap16(uint16_t c) { return (uint16_t)((c >> 8) | (c << 8)); }

static void sendCmd(uint8_t c) {
  digitalWrite(PIN_TFT_DC, LOW); digitalWrite(PIN_TFT_CS, LOW);
  SPI.transfer(c);
  digitalWrite(PIN_TFT_CS, HIGH);
}
static void sendData(const uint8_t* d, uint32_t n) {
  digitalWrite(PIN_TFT_DC, HIGH); digitalWrite(PIN_TFT_CS, LOW);
  SPI.writeBytes(d, n);
  digitalWrite(PIN_TFT_CS, HIGH);
}
static void sendCmdData(uint8_t c, const uint8_t* d, uint32_t n) { sendCmd(c); sendData(d, n); }

static void setWindow(int x0, int y0, int x1, int y1) {
  uint8_t d[4];
  x0 += COL_OFFSET; x1 += COL_OFFSET; y0 += ROW_OFFSET; y1 += ROW_OFFSET;
  d[0] = 0; d[1] = (uint8_t)x0; d[2] = 0; d[3] = (uint8_t)x1; sendCmdData(0x2A, d, 4);
  d[1] = (uint8_t)y0; d[3] = (uint8_t)y1; sendCmdData(0x2B, d, 4);
  sendCmd(0x2C);
}

static void tftInit() {
  pinMode(PIN_TFT_CS, OUTPUT); pinMode(PIN_TFT_DC, OUTPUT);
  pinMode(PIN_TFT_RST, OUTPUT);
#if PIN_TFT_LED >= 0
  pinMode(PIN_TFT_LED, OUTPUT); digitalWrite(PIN_TFT_LED, TFT_LED_ON);
#endif
  digitalWrite(PIN_TFT_CS, HIGH);
  SPI.begin(PIN_TFT_SCK, -1, PIN_TFT_MOSI, -1);
  SPI.beginTransaction(SPISettings(SPI_HZ, MSBFIRST, SPI_MODE0));
  digitalWrite(PIN_TFT_RST, HIGH); delay(10);
  digitalWrite(PIN_TFT_RST, LOW);  delay(20);
  digitalWrite(PIN_TFT_RST, HIGH); delay(150);
  sendCmd(0x01); delay(150);
  sendCmd(0x11); delay(255);
  const uint8_t frm1[] = {0x01, 0x2C, 0x2D};
  const uint8_t frm3[] = {0x01, 0x2C, 0x2D, 0x01, 0x2C, 0x2D};
  const uint8_t invc[] = {0x07};
  const uint8_t pw1[]  = {0xA2, 0x02, 0x84};
  const uint8_t pw2[]  = {0xC5};
  const uint8_t pw3[]  = {0x0A, 0x00};
  const uint8_t pw4[]  = {0x8A, 0x2A};
  const uint8_t pw5[]  = {0x8A, 0xEE};
  const uint8_t vm1[]  = {0x0E};
  const uint8_t madctl[] = {MADCTL_VALUE};
  const uint8_t colmod[] = {0x05};
  const uint8_t gam1[] = {0x02, 0x1C, 0x07, 0x12, 0x37, 0x32, 0x29, 0x2D, 0x29, 0x25, 0x2B, 0x39, 0x00, 0x01, 0x03, 0x10};
  const uint8_t gam2[] = {0x03, 0x1D, 0x07, 0x06, 0x2E, 0x2C, 0x29, 0x2D, 0x2E, 0x2E, 0x37, 0x3F, 0x00, 0x00, 0x02, 0x10};
  sendCmdData(0xB1, frm1, 3); sendCmdData(0xB2, frm1, 3); sendCmdData(0xB3, frm3, 6);
  sendCmdData(0xB4, invc, 1);
  sendCmdData(0xC0, pw1, 3); sendCmdData(0xC1, pw2, 1); sendCmdData(0xC2, pw3, 2);
  sendCmdData(0xC3, pw4, 2); sendCmdData(0xC4, pw5, 2); sendCmdData(0xC5, vm1, 1);
  sendCmd(USE_INVERT ? 0x21 : 0x20);
  sendCmdData(0x36, madctl, 1); sendCmdData(0x3A, colmod, 1);
  sendCmdData(0xE0, gam1, 16); sendCmdData(0xE1, gam2, 16);
  sendCmd(0x13); delay(10);
  sendCmd(0x29); delay(100);
}

static void flushScreen() {                     // push the whole frame buffer to the screen
  setWindow(0, 0, SCREEN_W - 1, SCREEN_H - 1);
  digitalWrite(PIN_TFT_DC, HIGH); digitalWrite(PIN_TFT_CS, LOW);
  const uint8_t* p = (const uint8_t*)fb;
  uint32_t n = (uint32_t)SCREEN_W * SCREEN_H * 2;
  while (n) { uint32_t c = n > 2048 ? 2048 : n; SPI.writeBytes(p, c); p += c; n -= c; }
  digitalWrite(PIN_TFT_CS, HIGH);
}

// ---- drawing into the frame buffer ----
static void fillRect(int x, int y, int w, int h, uint16_t color) {
  if (x < 0) { w += x; x = 0; }
  if (y < 0) { h += y; y = 0; }
  if (x + w > SCREEN_W) w = SCREEN_W - x;
  if (y + h > SCREEN_H) h = SCREEN_H - y;
  if (w <= 0 || h <= 0) return;
  uint16_t v = swap16(color);
  for (int yy = y; yy < y + h; yy++) {
    uint16_t* p = fb + yy * SCREEN_W + x;
    for (int i = 0; i < w; i++) p[i] = v;
  }
}
static void drawRect(int x, int y, int w, int h, uint16_t c) {
  fillRect(x, y, w, 1, c); fillRect(x, y + h - 1, w, 1, c);
  fillRect(x, y, 1, h, c); fillRect(x + w - 1, y, 1, h, c);
}
static void fillCard(int x, int y, int w, int h, uint16_t c) {          // rectangle with cut corners
  fillRect(x + 1, y, w - 2, h, c); fillRect(x, y + 1, w, h - 2, c);
}
static uint16_t lerp565(uint16_t a, uint16_t b, int t) {                 // t = 0..256
  int ar = (a >> 11) & 31, ag = (a >> 5) & 63, ab = a & 31;
  int br = (b >> 11) & 31, bg = (b >> 5) & 63, bb = b & 31;
  int r = ar + (((br - ar) * t) >> 8), g = ag + (((bg - ag) * t) >> 8), bl = ab + (((bb - ab) * t) >> 8);
  return (uint16_t)((r << 11) | (g << 5) | bl);
}
static void gradient(int y0, int y1, uint16_t c1, uint16_t c2) {
  int h = y1 - y0; if (h <= 0) return;
  for (int y = y0; y < y1; y++) fillRect(0, y, SCREEN_W, 1, lerp565(c1, c2, (y - y0) * 256 / h));
}
static uint16_t textOn(uint16_t c) {                                      // black or white text for a background
  int r = ((c >> 11) & 31) * 8, g = ((c >> 5) & 63) * 4, b = (c & 31) * 8;
  return ((r * 3 + g * 6 + b) / 10) > 150 ? C_INK : C_WHITE;
}
static void drawChar(int x, int y, char ch, int s, uint16_t fg, uint16_t bg) {
  if (ch < 32 || ch > 126) ch = '?';
  const uint8_t* g = font5x7[ch - 32];
  if (s == 1) {                                   // fast path: write pixels straight into the buffer
    uint16_t vf = swap16(fg), vb = swap16(bg);
    for (int gx = 0; gx < 6; gx++) {
      int px = x + gx; if (px < 0 || px >= SCREEN_W) continue;
      uint8_t col = (gx < 5) ? g[gx] : 0;
      for (int gy = 0; gy < 8; gy++) {
        int py = y + gy; if (py < 0 || py >= SCREEN_H) continue;
        if ((col >> gy) & 1) fb[py * SCREEN_W + px] = vf;
        else if (bg != TRANSP) fb[py * SCREEN_W + px] = vb;
      }
    }
    return;
  }
  for (int gx = 0; gx < 6; gx++) {
    uint8_t col = (gx < 5) ? g[gx] : 0;
    for (int gy = 0; gy < 8; gy++) {
      bool on = ((col >> gy) & 1);
      if (on) fillRect(x + gx * s, y + gy * s, s, s, fg);
      else if (bg != TRANSP) fillRect(x + gx * s, y + gy * s, s, s, bg);
    }
  }
}
static void drawText(int x, int y, const char* str, int s, uint16_t fg, uint16_t bg) {
  while (*str) { drawChar(x, y, *str, s, fg, bg); x += 6 * s; str++; }
}
static void drawTextN(int x, int y, const char* str, int maxChars, uint16_t fg) {   // clipped to maxChars
  for (int i = 0; str[i] && i < maxChars; i++) drawChar(x + i * 6, y, str[i], 1, fg, TRANSP);
}
static int textW(const char* str, int s) { return (int)strlen(str) * 6 * s; }
static void drawTextC(int cx, int y, const char* str, int s, uint16_t fg, uint16_t bg) {
  drawText(cx - textW(str, s) / 2, y, str, s, fg, bg);
}
static void drawTextR(int rx, int y, const char* str, uint16_t fg) { drawText(rx - textW(str, 1), y, str, 1, fg, TRANSP); }
static void drawWrapped(int x, int y, const char* text, int maxChars, int maxLines, uint16_t fg) {
  char line[32]; int ln = 0; const char* p = text;
  if (maxChars > 30) maxChars = 30;
  while (*p && ln < maxLines) {
    int len = 0, lastSpace = -1;
    while (p[len] && len < maxChars) { if (p[len] == ' ') lastSpace = len; len++; }
    if (p[len] && len == maxChars && lastSpace > 0) len = lastSpace;
    memcpy(line, p, len); line[len] = 0;
    drawText(x, y + ln * 10, line, 1, fg, TRANSP);
    p += len; while (*p == ' ') p++;
    ln++;
  }
}
static void drawIcon(int x, int y, int ic, uint16_t color, int sc) {
  if (ic < 0 || ic >= NICONS) return;
  for (int r = 0; r < 16; r++) {
    uint16_t bits = ICONS[ic][r];
    if (!bits) continue;
    for (int c = 0; c < 16; c++) if ((bits >> (15 - c)) & 1) fillRect(x + c * sc, y + r * sc, sc, sc, color);
  }
}

// ============================ 9. TOUCH (XPT2046, bit-banged) ==============
struct TouchCal { uint32_t magic; int32_t swap; float x0, x1, y0, y1; };
#define TCAL_MAGIC 0x54434132UL
static bool     tCalValid = false;
static TouchCal tCal;
static int      tRawX = 0, tRawY = 0, tRawZ = 0, tMapX = -1, tMapY = -1;
static bool     tIrq = false, tPress = false, tChipOk = true;
static int      calStep = 0, calRx[4], calRy[4]; static bool calLatched = false;
static const int CAL_PX[4] = {14, 113, 113, 14};
static const int CAL_PY[4] = {16, 16, 143, 143};

#if USE_TOUCH
static uint16_t xptRead(uint8_t cmd) {
  digitalWrite(PIN_T_CS, LOW);
  for (int i = 7; i >= 0; i--) {
    digitalWrite(PIN_T_DIN, (cmd >> i) & 1);
    digitalWrite(PIN_T_CLK, HIGH); digitalWrite(PIN_T_CLK, LOW);
  }
  uint16_t v = 0;
  for (int i = 0; i < 16; i++) {
    digitalWrite(PIN_T_CLK, HIGH); digitalWrite(PIN_T_CLK, LOW);
    v <<= 1; if (digitalRead(PIN_T_DO)) v |= 1;
  }
  digitalWrite(PIN_T_CS, HIGH);
  return (v >> 3) & 0x0FFF;
}
// reads the touch chip. returns true while the panel is pressed. rx/ry = raw numbers, z = pressure
static bool touchRaw(int& rx, int& ry, int& z) {
  tIrq = (digitalRead(PIN_T_IRQ) == LOW);
  int z1 = xptRead(0xB1), z2 = xptRead(0xC1);                 // pressure: Z1 / Z2
  tChipOk = !(z1 == z2 && (z1 == 0 || z1 == 4095));            // chip not answering = both read all-0 or all-1
  z = z1 + 4095 - z2; if (z < 0) z = 0;
  xptRead(0x90);                                               // back to normal mode
  bool pressed;
#if TOUCH_USE_IRQ
  pressed = tIrq;
#else
  pressed = tChipOk && z >= TOUCH_Z_MIN;
#endif
  if (!pressed) return false;
  xptRead(0xD0);                                               // dummy
  long sx = 0, sy = 0;
  for (int i = 0; i < 4; i++) { sx += xptRead(0xD0); sy += xptRead(0x90); }
  rx = (int)(sx / 4); ry = (int)(sy / 4);
  if ((rx == 0 && ry == 0) || (rx >= 4095 && ry >= 4095)) { tChipOk = false; return false; }
#if TOUCH_DEBUG
  Serial.printf("#TOUCH raw %d %d z=%d\n", rx, ry, z);
#endif
  return true;
}
#else
static bool touchRaw(int& rx, int& ry, int& z) { (void)rx; (void)ry; z = 0; return false; }
#endif

// raw -> screen pixels (uses the saved calibration, or the TOUCH_* numbers on top of this file)
static void touchMap(int rx, int ry, int& x, int& y) {
  float u = (float)rx, v = (float)ry, ax0, ax1, ay0, ay1; bool sw;
  if (tCalValid) { sw = tCal.swap != 0; ax0 = tCal.x0; ax1 = tCal.x1; ay0 = tCal.y0; ay1 = tCal.y1; }
  else {
    sw = TOUCH_SWAP_XY;
    ax0 = TOUCH_INVERT_X ? TOUCH_X_MAX : TOUCH_X_MIN; ax1 = TOUCH_INVERT_X ? TOUCH_X_MIN : TOUCH_X_MAX;
    ay0 = TOUCH_INVERT_Y ? TOUCH_Y_MAX : TOUCH_Y_MIN; ay1 = TOUCH_INVERT_Y ? TOUCH_Y_MIN : TOUCH_Y_MAX;
  }
  if (sw) { float t = u; u = v; v = t; }
  float px = (u - ax0) * (SCREEN_W - 1) / (ax1 - ax0);
  float py = (v - ay0) * (SCREEN_H - 1) / (ay1 - ay0);
  if (px < 0) px = 0; if (px > SCREEN_W - 1) px = SCREEN_W - 1;
  if (py < 0) py = 0; if (py > SCREEN_H - 1) py = SCREEN_H - 1;
  x = (int)px; y = (int)py;
}
static void touchCalLoad() {
  File f = LittleFS.open("/touch.bin", FILE_READ);
  if (!f) return;
  TouchCal c;
  if (f.read((uint8_t*)&c, sizeof(c)) == sizeof(c) && c.magic == TCAL_MAGIC) { tCal = c; tCalValid = true; }
  f.close();
}
static void touchCalSave() {
  File f = LittleFS.open("/touch.bin", FILE_WRITE);
  if (!f) return;
  f.write((const uint8_t*)&tCal, sizeof(tCal)); f.close();
}
// 4 taps (top-left, top-right, bottom-right, bottom-left) -> swap / invert / scale. false = taps were not sensible
static bool touchCalCompute() {
  float dx = fabsf((float)(calRx[1] - calRx[0])) + fabsf((float)(calRx[2] - calRx[3]));
  float dy = fabsf((float)(calRy[1] - calRy[0])) + fabsf((float)(calRy[2] - calRy[3]));
  bool sw = dy > dx;
  float u[4], v[4];
  for (int i = 0; i < 4; i++) { u[i] = (float)(sw ? calRy[i] : calRx[i]); v[i] = (float)(sw ? calRx[i] : calRy[i]); }
  float uL = (u[0] + u[3]) * 0.5f, uR = (u[1] + u[2]) * 0.5f, vT = (v[0] + v[1]) * 0.5f, vB = (v[2] + v[3]) * 0.5f;
  if (fabsf(uR - uL) < 500 || fabsf(vB - vT) < 500) return false;
  float perX = (uR - uL) / (float)(CAL_PX[1] - CAL_PX[0]), perY = (vB - vT) / (float)(CAL_PY[2] - CAL_PY[1]);
  tCal.magic = TCAL_MAGIC; tCal.swap = sw ? 1 : 0;
  tCal.x0 = uL - perX * CAL_PX[0];                       tCal.x1 = uR + perX * (SCREEN_W - 1 - CAL_PX[1]);
  tCal.y0 = vT - perY * CAL_PY[0];                       tCal.y1 = vB + perY * (SCREEN_H - 1 - CAL_PY[2]);
  tCalValid = true;
  return true;
}

// ============================ 10. SMALL HELPERS ===========================
static float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
static inline bool  svOk(const Sample& s, int i) { return s.v[i] != INV; }
static inline float sv(const Sample& s, int i)   { return s.v[i] / (float)SENS[i].scale; }

static int app(char* b, int len, int cap, const char* fmt, ...) {
  va_list ap; va_start(ap, fmt);
  int r = vsnprintf(b + len, cap - len, fmt, ap);
  va_end(ap);
  if (r < 0) return len;
  len += r; if (len >= cap) len = cap - 1;
  return len;
}

static void toast(const char* m) {
  strncpy(toastMsg, m, sizeof(toastMsg) - 1); toastMsg[sizeof(toastMsg) - 1] = 0;
  toastUntil = millis() + 2200; dirty = true;
}
static uint32_t epochAt(uint32_t ms) { return epochBase ? epochBase + ms / 1000 : 0; }
static void fmtClock(uint32_t epoch, uint32_t ms, char* out, size_t n) {
  if (epoch) {
    time_t t = (time_t)epoch; struct tm tmv; gmtime_r(&t, &tmv);
    strftime(out, n, "%d %b %H:%M:%S", &tmv);
  } else {
    uint32_t s = ms / 1000;
    snprintf(out, n, "boot+%02u:%02u:%02u", (unsigned)(s / 3600), (unsigned)((s / 60) % 60), (unsigned)(s % 60));
  }
}
static void fmtClockShort(uint32_t epoch, uint32_t ms, char* out, size_t n) {
  if (epoch) { time_t t = (time_t)epoch; struct tm tmv; gmtime_r(&t, &tmv); strftime(out, n, "%d %b %H:%M", &tmv); }
  else { uint32_t s = ms / 1000; snprintf(out, n, "boot+%02u:%02u", (unsigned)(s / 3600), (unsigned)((s / 60) % 60)); }
}
static void fmtClockTiny(uint32_t epoch, uint32_t ms, char* out, size_t n) {          // 09/10 08:55
  if (epoch) { time_t t = (time_t)epoch; struct tm tmv; gmtime_r(&t, &tmv); strftime(out, n, "%d/%m %H:%M", &tmv); }
  else { uint32_t s = ms / 1000; snprintf(out, n, "b+%02u:%02u", (unsigned)(s / 3600), (unsigned)((s / 60) % 60)); }
}
static void nowClock(char* out, size_t n) { uint32_t m = millis(); fmtClock(epochAt(m), m, out, n); }

static void fmtVal(int i, float v, char* out, size_t n, bool withUnit) {
  const SensInfo& s = SENS[i];
  if (s.flags & SF_BOOL) { snprintf(out, n, "%s", v > 0.5f ? "YES" : "NO"); return; }
  if (s.flags & SF_FUSE) { snprintf(out, n, "%s", v > 0.5f ? "OK" : "BLOWN"); return; }
  if (s.flags & SF_RELAY) { snprintf(out, n, "%s", v > 0.5f ? "OK" : "FAILED"); return; }
  if (withUnit && s.unit[0]) snprintf(out, n, "%.*f %s", (int)s.dec, v, s.unit);
  else snprintf(out, n, "%.*f", (int)s.dec, v);
}

// severity of a value against the limits of sensor i (0 ok, 1 warning, 2 critical)
static uint8_t sevOfValue(int i, float v, bool running) {
  const SensInfo& s = SENS[i];
  if (v >= CHI(i)) return 2;
  if (v >= WHI(i)) return 1;
  if (!(s.flags & SF_RUN) || running) {
    if (v <= CLO(i)) return 2;
    if (v <= WLO(i)) return 1;
  }
  return 0;
}
static bool engineRunning() { return haveData && latest.v[SN_RPM] != INV && sv(latest, SN_RPM) > RUNNING_RPM; }
static float curSpeed() { return (haveData && latest.v[SN_SPEED] != INV) ? sv(latest, SN_SPEED) : 0.0f; }
static const char* sevWord(uint8_t s) { return s >= 2 ? "CRIT" : (s == 1 ? "WARN" : "OK"); }
static uint16_t sevColor(uint8_t sev) { return sev >= 2 ? COL_CRIT : (sev == 1 ? COL_WARN : COL_OK); }

static uint8_t sensorSev(int i) {                        // worst active fault of one sensor
  uint8_t m = 0;
  for (int k = 0; k < 4; k++) if (fSev[FID(i, k)] > m) m = fSev[FID(i, k)];
  return m;
}
static uint8_t subSev(int sub, int* count) {
  uint8_t m = 0; int c = 0;
  for (int i = 0; i < NSENS; i++) if (SENS[i].sub == sub) {
    for (int k = 0; k < 4; k++) if (fSev[FID(i, k)]) { c++; if (fSev[FID(i, k)] > m) m = fSev[FID(i, k)]; }
  }
  if (count) *count = c;
  return m;
}
static uint8_t systemSev(int sysIdx, int* count) {
  uint8_t m = 0; int c = 0;
  for (int i = 0; i < NSENS; i++) if (SENS[i].sys == sysIdx) {
    for (int k = 0; k < 4; k++) if (fSev[FID(i, k)]) { c++; if (fSev[FID(i, k)] > m) m = fSev[FID(i, k)]; }
  }
  if (count) *count = c;
  return m;
}
static uint8_t overallSev() {
  uint8_t m = 0;
  for (int f = 0; f < NFAULT; f++) if (fSev[f] > m) m = fSev[f];
  return m;
}
static int activeCount() { int c = 0; for (int f = 0; f < NFAULT; f++) if (fSev[f]) c++; return c; }
static int topFault() {                                   // most severe active fault (-1 = none)
  int best = -1; uint8_t bs = 0;
  for (int f = 0; f < NFAULT; f++) if (fSev[f] > bs) { bs = fSev[f]; best = f; }
  return best;
}
static void faultName(int fid, char* out, size_t n, bool shortKey) {
  int s = fid / 4, k = fid % 4;
  if (s >= NSENS) { snprintf(out, n, "ECU LINK LOST"); return; }
  if (s == SN_DERATE) { snprintf(out, n, shortKey ? "LIMP MODE" : "Power reduced (limp)"); return; }
  const char* base = shortKey ? SENS[s].key : SENS[s].name;
  if (k == K_LOW && (SENS[s].flags & SF_FUSE))  { snprintf(out, n, "%s BLOWN", base); return; }
  if (k == K_LOW && (SENS[s].flags & SF_RELAY)) { snprintf(out, n, "%s FAILED", base); return; }
  if ((SENS[s].flags & SF_BOOL) && k == K_HIGH) snprintf(out, n, "%s", base);
  else snprintf(out, n, "%s %s", base, KIND_NAME[k]);
}
static void faultCode(int fid, int sev, char* out, size_t n) {       // J1939 style "SPN110 FMI0"
  out[0] = 0;
  int s = fid / 4, k = fid % 4;
  if (s >= NSENS) { snprintf(out, n, "U0100"); return; }
  if (!SENS[s].spn) return;
  int fmi = (k == K_HIGH) ? (sev >= 2 ? 0 : 16) : (k == K_LOW ? (sev >= 2 ? 1 : 17) : 2);
  snprintf(out, n, "SPN%u FMI%d", (unsigned)SENS[s].spn, fmi);
}
static float faultLimit(int fid, int sev) {            // the limit that belongs to this fault right now
  int s = fid / 4, k = fid % 4;
  if (s >= NSENS) return 0.0f;
  if (k == K_HIGH) return sev >= 2 ? CHI(s) : WHI(s);
  if (k == K_LOW)  return sev >= 2 ? CLO(s) : WLO(s);
  return 0.0f;
}
static void fmtLim(float v, char* out, size_t n) {
  if (v > 1e8f || v < -1e8f) snprintf(out, n, "-"); else snprintf(out, n, "%.4g", v);
}
static const char* adviceFor(int s) {
  if (s >= NSENS) return "Check the ECU cable and power.";
  switch (s) {
    case SN_WIF:    return "Water in fuel! Stop and drain the fuel filter.";
    case SN_DERATE: return "Engine in limp mode. Do NOT overtake.";
    case SN_DEFL:   return "Refill AdBlue soon or engine power will drop.";
    case SN_SPEED:  return "Overspeed. Slow down now.";
    case SN_ACCEL:  return "Harsh braking. Keep more distance.";
    case SN_FUEL:   return "Refuel at the next pump.";
    case SN_OILP:   return "Low oil pressure! Stop the engine soon.";
    default: break;
  }
  if (SENS[s].flags & SF_FUSE) return "Blown fuse. Find the short, then fit same rating.";
  if (SENS[s].flags & SF_RELAY) return "Relay not switching. Check relay, wiring and fuse.";
  return SUBS[SENS[s].sub].adv;
}
static bool linkOk() { return haveData && fSev[F_LINK] == 0; }
static bool canOpenMenu() { return ALLOW_MENU_WHILE_MOVING || !haveData || curSpeed() < MECH_SPEED_LIMIT; }

static void* bigAlloc(size_t n) {
  void* p = nullptr;
  if (psramFound()) p = ps_malloc(n);
  if (!p) p = malloc(n);
  return p;
}

// short description of a problem for lists:  "118.5>110"
static void faultLine(int fid, int sev, const Sample& s, char* out, size_t n) {
  int si = fid / 4, kind = fid % 4;
  char v[16], l[16];
  if (si >= NSENS) { snprintf(out, n, "no ECU data"); return; }
  if (kind == K_SENSOR) { snprintf(out, n, svOk(s, si) ? "bad reading" : "no signal"); return; }
  if (kind == K_MISMATCH) { snprintf(out, n, (si >= SN_WS1 && si <= SN_WS1 + 5) ? "wheel differs" : "does not match"); return; }
  if (SENS[si].flags & SF_FUSE) { snprintf(out, n, "blown"); return; }
  if (SENS[si].flags & SF_RELAY) { snprintf(out, n, "failed"); return; }
  if (SENS[si].flags & SF_BOOL) { snprintf(out, n, "detected"); return; }
  if (!svOk(s, si)) { snprintf(out, n, "no signal"); return; }
  fmtVal(si, sv(s, si), v, sizeof(v), false);
  fmtLim(faultLimit(fid, sev), l, sizeof(l));
  snprintf(out, n, "%s%s%s", v, kind == K_HIGH ? ">" : "<", l);
}

// ============================ 11. TELEGRAM ================================
static uint32_t hashStr(const char* s) { uint32_t h = 2166136261u; while (*s) { h ^= (uint8_t)*s++; h *= 16777619u; } return h; }

static void tgEnqueue(const char* text) {
  if (!tgEnabled || !tgQ) return;
  uint32_t now = millis(), h = hashStr(text);
  for (int i = 0; i < 4; i++) if (tgHash[i] == h && now - tgHashMs[i] < 4000UL) return;    // exact same text within 4 s: skip
  static int hi = 0; tgHash[hi] = h; tgHashMs[hi] = now; hi = (hi + 1) & 3;
  static TgMsg m, old;                              // static: keeps the main task stack small
  strncpy(m.text, text, sizeof(m.text) - 1); m.text[sizeof(m.text) - 1] = 0;
  if (xQueueSend(tgQ, &m, 0) != pdTRUE) {
    xQueueReceive(tgQ, &old, 0);
    xQueueSend(tgQ, &m, 0); tgDropped++;
  }
}

// returns: 1 = delivered (or assumed delivered), 0 = try again later, -1 = never retry
static int tgSendNow(const char* text) {
  static char body[3900];
  int n = snprintf(body, sizeof(body), "{\"chat_id\":\"%s\",\"text\":\"", TG_CHAT_ID);
  for (const char* p = text; *p && n < (int)sizeof(body) - 8; ++p) {
    unsigned char c = (unsigned char)*p;
    if (c == '"' || c == '\\') { body[n++] = '\\'; body[n++] = (char)c; }
    else if (c == '\n') { body[n++] = '\\'; body[n++] = 'n'; }
    else if (c < 0x20) { body[n++] = ' '; }
    else body[n++] = (char)c;                       // UTF-8 (emojis) passes through
  }
  n += snprintf(body + n, sizeof(body) - n, "\"}");
  WiFiClientSecure client; client.setInsecure();    // no certificate check (demo)
  HTTPClient http; http.setTimeout(12000);
  String url = String("https://api.telegram.org/bot") + TG_BOT_TOKEN + "/sendMessage";
  if (!http.begin(client, url)) return 0;
  http.addHeader("Content-Type", "application/json");
  int code = http.POST((uint8_t*)body, (size_t)n);
  tgCode = code;
  http.end();
  if (code == 200) return 1;
  if (code == -11) return 1;                        // read timeout: Telegram most likely got it. Do NOT resend (avoids repeats)
  if (code > 0 && code != 429 && code < 500) return -1;   // wrong token / chat id etc: retrying is pointless
  return 0;
}

static void tgTask(void*) {
  uint32_t lastTry = millis(); TgMsg cur; bool have = false; int tries = 0;
  for (;;) {
    if (WiFi.status() != WL_CONNECTED) {
      tgState = 1;
      if (millis() - lastTry > 15000) { WiFi.disconnect(); WiFi.begin(WIFI_SSID, WIFI_PASS); lastTry = millis(); }
      vTaskDelay(pdMS_TO_TICKS(500)); continue;
    }
    if (tgState == 1) tgState = 2;
    if (!have) {
      if (xQueueReceive(tgQ, &cur, pdMS_TO_TICKS(500)) == pdTRUE) { have = true; tries = 0; }
      else continue;
    }
    int r = tgSendNow(cur.text);
    Serial.printf("#TG result=%d http=%d heap=%u\n", r, (int)tgCode, (unsigned)ESP.getFreeHeap());
    if (r == 1) { have = false; tgSent++; tgState = 2; vTaskDelay(pdMS_TO_TICKS(1500)); }
    else if (r < 0) { have = false; tgDropped++; tgState = 3; vTaskDelay(pdMS_TO_TICKS(1500)); }
    else {
      tries++; tgState = 3;
      if (tries >= 3) { have = false; tgDropped++; }
      vTaskDelay(pdMS_TO_TICKS(tries * 6000));
    }
  }
}

static void unitTg(const char* u, char* out, size_t n) {          // "C" -> degree C for Telegram
  if (strcmp(u, "C") == 0) snprintf(out, n, "°C"); else snprintf(out, n, "%s", u);
}

// one Telegram message for a group of problems (what is wrong, how bad, how much, when, where it is saved)
// reminder = true: a list of EVERYTHING that is still wrong
static void notifyFaults(const uint8_t* fl, int n, int extra, const Sample& s, uint16_t id, bool reminder) {
  static char msg[1500];
  char tm[28], a[40], b[24], c[24], un[8], ln[24];
  int cap = (int)sizeof(msg), len = 0;
  nowClock(tm, sizeof(tm));
  uint8_t list[2 * 64]; int nl = 0; bool hadLink = false;
  for (int k = 0; k < n && nl < 64; k++) {
    if (!reminder && fl[2 * k] == F_LINK) { hadLink = true; continue; }
    list[2 * nl] = fl[2 * k]; list[2 * nl + 1] = fl[2 * k + 1]; nl++;
  }
  if (hadLink) {
    len = app(msg, len, cap, "📡❌ DEVICE ALERT\nNo data is coming from the ECU.\n"
                             "🔌 Check the ECU cable / ECU power.\n🕒 %s\n", tm);
    if (id) len = app(msg, len, cap, "💾 Last data saved as incident #%u\n", (unsigned)id);
    tgEnqueue(msg);
    if (nl == 0) return;
    len = 0;
  }
  if (nl == 0) return;
  if (reminder) len = app(msg, len, cap, "⏰ STILL NOT FIXED · %d active problem%s\n", nl, nl > 1 ? "s" : "");
  else len = app(msg, len, cap, "🚨 TRUCK BLACK BOX ALERT\n");
  int detailed = reminder ? 0 : (nl > 3 ? 3 : nl);
  for (int k = 0; k < detailed; k++) {
    int fid = list[2 * k], sev = list[2 * k + 1], si = fid / 4, kind = fid % 4;
    const SensInfo& se = SENS[si];
    unitTg(se.unit, un, sizeof(un));
    len = app(msg, len, cap, "\n%s %s · %s %s\n", sev >= 2 ? "🔴" : "🟡", sev >= 2 ? "CRITICAL" : "WARNING",
              SYS[se.sys].emoji, SYS[se.sys].name);
    faultName(fid, a, sizeof(a), false);
    len = app(msg, len, cap, "%s\n", a);
    if (kind == K_SENSOR) {
      if (svOk(s, si)) { fmtVal(si, sv(s, si), b, sizeof(b), false); len = app(msg, len, cap, "Reading %s %s looks wrong (sensor / wiring)", b, un); }
      else len = app(msg, len, cap, "No signal from this sensor");
    } else if (kind == K_MISMATCH) {
      fmtVal(si, sv(s, si), b, sizeof(b), false);
      len = app(msg, len, cap, "Reads %s %s, %s", b, un, (si >= SN_WS1 && si <= SN_WS1 + 5) ? "other wheels differ" : "does not match its partner / target");
    } else if (se.flags & SF_FUSE) {
      len = app(msg, len, cap, "⚡ Fuse blown - this circuit has no power");
    } else if (se.flags & SF_RELAY) {
      len = app(msg, len, cap, "⚡ Relay failed - it is not switching");
    } else if (se.flags & SF_BOOL) {
      len = app(msg, len, cap, "Detected");
    } else {
      fmtVal(si, svOk(s, si) ? sv(s, si) : 0.0f, c, sizeof(c), false);
      fmtLim(faultLimit(fid, sev), b, sizeof(b));
      len = app(msg, len, cap, "Now %s %s (limit %s)", c, un, b);
    }
    faultCode(fid, sev, c, sizeof(c));
    if (c[0]) len = app(msg, len, cap, " · %s", c);
    len = app(msg, len, cap, "\n");
  }
  int compactN = 0;                                   // every other problem: one short line each
  for (int k = detailed; k < nl && compactN < 15; k++) {
    int fid = list[2 * k], sev = list[2 * k + 1];
    faultName(fid, a, sizeof(a), false);
    faultLine(fid, sev, s, ln, sizeof(ln));
    len = app(msg, len, cap, "%s• %s %s · %s\n", compactN == 0 && detailed ? "\n" : "", sev >= 2 ? "🔴" : "🟡", a, ln);
    compactN++;
  }
  int more = (nl - detailed - compactN) + extra;
  if (more > 0) len = app(msg, len, cap, "➕ %d more problems\n", more);
  len = app(msg, len, cap, "\n🚚 %.0f km/h · ⚙️ %.0f rpm\n🕒 %s\n",
            svOk(s, SN_SPEED) ? sv(s, SN_SPEED) : 0.0f, svOk(s, SN_RPM) ? sv(s, SN_RPM) : 0.0f, tm);
  if (id && !reminder) len = app(msg, len, cap, "💾 Incident #%u saved on device\n", (unsigned)id);
  len = app(msg, len, cap, "💡 %s", adviceFor(list[0] / 4));
  tgEnqueue(msg);
}

static void pendAdd(int fid, int sev) {
  for (int k = 0; k < nPend; k++) if (pend[2 * k] == fid) { if (sev > pend[2 * k + 1]) pend[2 * k + 1] = (uint8_t)sev; return; }
  if (nPend < 48) { pend[2 * nPend] = (uint8_t)fid; pend[2 * nPend + 1] = (uint8_t)sev; nPend++; }
  else pendExtra++;
}
static void pendingTick(uint32_t now) {                 // send the batched problems after a short quiet time
  if (nPend > 0 && now - pendSince >= ALERT_BATCH_MS) {
    notifyFaults(pend, nPend, pendExtra, pendRef, pendId, false);
    nPend = 0; pendExtra = 0; pendId = 0; lastRemind = now;
  }
}
static void remindTick(uint32_t now) {                  // problems still active: send the full list again every TG_REMIND_MS
  if (!TG_REMIND_MS || !tgEnabled || nPend > 0 || now - lastRemind < TG_REMIND_MS) return;
  uint8_t list[2 * 64]; int n = 0;
  for (int f = 0; f < NFAULT && n < 64; f++) if (fSev[f]) { list[2 * n] = (uint8_t)f; list[2 * n + 1] = fSev[f]; n++; }
  lastRemind = now;
  if (n > 0) notifyFaults(list, n, 0, latest, 0, true);
}

// ============================ 12. FLASH STORAGE ===========================
static void incPath(uint16_t id, char* out, size_t n) { snprintf(out, n, "/inc/%04u.bin", (unsigned)id); }
static uint32_t fsFree() { return LittleFS.totalBytes() - LittleFS.usedBytes(); }
static uint32_t incFileBytes() { return (uint32_t)sizeof(IncHeader) + (uint32_t)(g_preN + g_postN) * sizeof(Sample); }

static void rebuildHistory() {
  memset(fHist, 0, sizeof(fHist));
  for (int f = 0; f < NFAULT; f++) fLastMeta[f] = -1;
  for (int i = 0; i < metaN; i++)
    for (int k = 0; k < meta[i].nf && k < MAXF; k++) {
      int fid = meta[i].faults[2 * k];
      if (fid < NFAULT) { fHist[fid]++; fLastMeta[fid] = i; }
    }
}
static void deleteOldestIncident() {
  if (metaN <= 0) return;
  char p[24]; incPath(meta[0].id, p, sizeof(p)); LittleFS.remove(p);
  for (int i = 1; i < metaN; i++) meta[i - 1] = meta[i];
  metaN--; rebuildHistory();
}
static int autoClean() {                               // flash nearly full -> the oldest incidents go, newest stay
  uint64_t tot = LittleFS.totalBytes(); int removed = 0;
  if (!tot) return 0;
  if ((uint64_t)LittleFS.usedBytes() * 100 > tot * FLASH_HIGH_PCT)
    while (metaN > 1 && (uint64_t)LittleFS.usedBytes() * 100 > tot * FLASH_LOW_PCT) { deleteOldestIncident(); removed++; }
  return removed;
}
static void ensureSpace(uint32_t need) {
  while (fsFree() < need + FS_RESERVE && metaN > 0) deleteOldestIncident();
}
static void metaFromHeader(const IncHeader& h, IncMeta& m) {
  m.id = h.id; m.epoch = h.epoch; m.trigMs = h.trigMs; m.spd10 = h.trigSpd10; m.count = h.count;
  m.nf = h.nf > MAXF ? MAXF : h.nf; m.sev = h.sev;
  memcpy(m.faults, h.faults, sizeof(m.faults)); memcpy(m.fRpm, h.fRpm, sizeof(m.fRpm)); memcpy(m.fSpd10, h.fSpd10, sizeof(m.fSpd10));
  memcpy(m.fMs, h.fMs, sizeof(m.fMs)); memcpy(m.fEpoch, h.fEpoch, sizeof(m.fEpoch));
}
static void scanIncidents() {
  metaN = 0; char bad[8][32]; int nbad = 0;
  File root = LittleFS.open("/inc");
  if (root && root.isDirectory()) {
    File f = root.openNextFile();
    while (f) {
      if (!f.isDirectory() && metaN < MAX_INC) {
        IncHeader h;
        if (f.size() >= (int)sizeof(h) && f.read((uint8_t*)&h, sizeof(h)) == sizeof(h) && h.magic == BB_MAGIC && h.version == BB_VERSION) {
          metaFromHeader(h, meta[metaN]); metaN++;
        } else if (nbad < 8) {                          // file from an older firmware: cannot be read, remove it
          const char* nm = f.name(); const char* bs = strrchr(nm, '/'); bs = bs ? bs + 1 : nm;
          snprintf(bad[nbad++], sizeof(bad[0]), "/inc/%s", bs);
        }
      }
      f.close(); f = root.openNextFile();
    }
  }
  for (int i = 0; i < nbad; i++) LittleFS.remove(bad[i]);
  for (int i = 1; i < metaN; i++) {                // sort by id (oldest first)
    IncMeta k = meta[i]; int j = i - 1;
    while (j >= 0 && meta[j].id > k.id) { meta[j + 1] = meta[j]; j--; }
    meta[j + 1] = k;
  }
  nextId = metaN ? (uint16_t)(meta[metaN - 1].id + 1) : 1;
  rebuildHistory();
}
static bool writeIncident() {
  uint32_t bytes = sizeof(IncHeader) + (uint32_t)incCount * sizeof(Sample);
  ensureSpace(bytes + 4096);
  char p[24]; incPath(incHdr.id, p, sizeof(p));
  File f = LittleFS.open(p, FILE_WRITE);
  if (!f) return false;
  size_t w = f.write((const uint8_t*)&incHdr, sizeof(incHdr));
  w += f.write((const uint8_t*)incBuf, (size_t)incCount * sizeof(Sample));
  f.close();
  if (w != bytes) { LittleFS.remove(p); return false; }
  if (metaN >= MAX_INC) deleteOldestIncident();
  metaFromHeader(incHdr, meta[metaN]); metaN++;
  rebuildHistory();
  lastCleaned = autoClean();
  return true;
}
static bool loadIncidentToView(int mi) {
  char p[24]; incPath(meta[mi].id, p, sizeof(p));
  File f = LittleFS.open(p, FILE_READ);
  if (!f) return false;
  IncHeader h;
  if (f.read((uint8_t*)&h, sizeof(h)) != sizeof(h) || h.magic != BB_MAGIC || h.count > g_incMax) { f.close(); return false; }
  int n = (int)(f.read((uint8_t*)viewBuf, (size_t)h.count * sizeof(Sample)) / sizeof(Sample));
  f.close();
  viewHdr = h; viewCount = n; viewTrig = h.trigIndex < n ? h.trigIndex : n - 1;
  curCol = (int)((long)viewTrig * SCREEN_W / (n > 0 ? n : 1));
  return n > 1;
}
// ---- FAULT LOG: every problem that starts / ends goes to flash AT ONCE (the black-box file is only saved after its post-window) ----
#define FLOG_MAX   150
#define FLOG_MAGIC 0x31474C46UL
struct __attribute__((packed)) FEvt {
  uint32_t epoch, ms; uint16_t fid; uint8_t sev, kind;      // kind: 0 = problem started (or got worse), 1 = cleared
  float val, lim; uint16_t spd10, rpm, inc; uint8_t hasVal;
};
static FEvt flog[FLOG_MAX]; static int flogN = 0;
static void flogRewrite() {
  File f = LittleFS.open("/flog.bin", FILE_WRITE);
  if (!f) return;
  uint32_t m = FLOG_MAGIC; f.write((const uint8_t*)&m, 4);
  if (flogN) f.write((const uint8_t*)flog, (size_t)flogN * sizeof(FEvt));
  f.close();
}
static void flogLoad() {
  flogN = 0;
  File f = LittleFS.open("/flog.bin", FILE_READ);
  if (!f) return;
  uint32_t m = 0; bool good = (f.read((uint8_t*)&m, 4) == 4 && m == FLOG_MAGIC);
  if (!good) { f.close(); LittleFS.remove("/flog.bin"); return; }
  int total = (int)((f.size() - 4) / sizeof(FEvt));
  int skip = total > FLOG_MAX ? total - FLOG_MAX : 0;
  f.seek(4 + (size_t)skip * sizeof(FEvt));
  while (flogN < FLOG_MAX && f.read((uint8_t*)&flog[flogN], sizeof(FEvt)) == sizeof(FEvt)) flogN++;
  f.close();
  if (skip) flogRewrite();
}
static void flogAdd(int fid, int sev, uint8_t kind, const Sample& ref, uint16_t inc) {
  FEvt e; memset(&e, 0, sizeof(e));
  uint32_t t = ref.t ? ref.t : millis();
  e.ms = t; e.epoch = epochAt(t);
  e.fid = (uint16_t)fid; e.sev = (uint8_t)sev; e.kind = kind; e.inc = inc;
  int si = fid / 4;
  if (si < NSENS && svOk(ref, si)) { e.val = sv(ref, si); e.hasVal = 1; }
  e.lim = (kind == 0) ? faultLimit(fid, sev) : 0.0f;
  e.spd10 = svOk(ref, SN_SPEED) ? (uint16_t)clampf(sv(ref, SN_SPEED) * 10.0f, 0, 65000) : 0;
  e.rpm = svOk(ref, SN_RPM) ? (uint16_t)clampf(sv(ref, SN_RPM), 0, 65000) : 0;
  if (flogN >= FLOG_MAX) {
    memmove(flog, flog + 1, sizeof(FEvt) * (FLOG_MAX - 1)); flogN = FLOG_MAX - 1;
    flog[flogN++] = e; flogRewrite(); return;
  }
  flog[flogN++] = e;
  File f = LittleFS.open("/flog.bin", FILE_APPEND);
  if (!f) return;
  if (f.size() == 0) { uint32_t m = FLOG_MAGIC; f.write((const uint8_t*)&m, 4); }
  f.write((const uint8_t*)&e, sizeof(e)); f.close();
}
static void eraseAllLogs() {
  for (int i = 0; i < metaN; i++) { char p[24]; incPath(meta[i].id, p, sizeof(p)); LittleFS.remove(p); }
  metaN = 0; nextId = 1; rebuildHistory();
  flogN = 0; LittleFS.remove("/flog.bin");
}

// ---- limits: defaults come from the catalog, the PC can change them, they are kept in flash ----
#define THR_MAGIC (0x54485200UL + NSENS)
struct ThrFile { uint32_t magic; float t[NSENS][4]; };
static void thrDefaults() {
  for (int i = 0; i < NSENS; i++) { thr[i][0] = SENS[i].wLo; thr[i][1] = SENS[i].cLo; thr[i][2] = SENS[i].wHi; thr[i][3] = SENS[i].cHi; }
}
static void thrLoad() {
  thrDefaults();
  File f = LittleFS.open("/thr.bin", FILE_READ);
  if (!f) return;
  ThrFile tf;
  if (f.read((uint8_t*)&tf, sizeof(tf)) == sizeof(tf) && tf.magic == THR_MAGIC) memcpy(thr, tf.t, sizeof(thr));
  f.close();
}
static void thrSave() {
  ThrFile tf; tf.magic = THR_MAGIC; memcpy(tf.t, thr, sizeof(thr));
  File f = LittleFS.open("/thr.bin", FILE_WRITE);
  if (!f) return;
  f.write((const uint8_t*)&tf, sizeof(tf)); f.close();
}
static float parseLim(const char* t, bool high) {
  if (!t || t[0] == 'n' || t[0] == 'N') return high ? NOLIM : -NOLIM;
  return (float)atof(t);
}

// ============================ 13. INCIDENT RECORDER =======================
static void incAddFault(int fid, int sev, int idx, float lim, const Sample& ref) {
  for (int k = 0; k < incHdr.nf; k++) {
    if (incHdr.faults[2 * k] == fid) { if (sev > incHdr.faults[2 * k + 1]) incHdr.faults[2 * k + 1] = (uint8_t)sev; if (sev > incHdr.sev) incHdr.sev = (uint8_t)sev; return; }
  }
  if (incHdr.nf < MAXF) {
    int k = incHdr.nf++;
    incHdr.faults[2 * k] = (uint8_t)fid; incHdr.faults[2 * k + 1] = (uint8_t)sev;
    incHdr.fIdx[k] = (uint16_t)(idx < 0 ? 0 : idx); incHdr.fLim[k] = lim;
    incHdr.fRpm[k] = svOk(ref, SN_RPM) ? (uint16_t)clampf(sv(ref, SN_RPM), 0, 65000) : 0;
    incHdr.fSpd10[k] = svOk(ref, SN_SPEED) ? (uint16_t)clampf(sv(ref, SN_SPEED) * 10.0f, 0, 65000) : 0;
    incHdr.fMs[k] = ref.t; incHdr.fEpoch[k] = epochAt(ref.t);
  }
  if (sev > incHdr.sev) incHdr.sev = (uint8_t)sev;
}

static uint16_t startIncident(const Sample& ref) {
  int start = (ringHead - ringCount + g_preN) % g_preN;
  incCount = 0;
  for (int i = 0; i < ringCount; i++) incBuf[incCount++] = ring[(start + i) % g_preN];
  memset(&incHdr, 0, sizeof(incHdr));
  incHdr.magic = BB_MAGIC; incHdr.version = BB_VERSION; incHdr.id = nextId++;
  incHdr.trigMs = ref.t; incHdr.epoch = epochAt(ref.t);
  incHdr.trigIndex = (uint16_t)(incCount - 1);
  incHdr.trigSpd10 = svOk(ref, SN_SPEED) ? (uint16_t)clampf(sv(ref, SN_SPEED) * 10.0f, 0, 65000) : 0;
  incHdr.trigRpm = svOk(ref, SN_RPM) ? (uint16_t)clampf(sv(ref, SN_RPM), 0, 65000) : 0;
  incRemain = g_postN; incState = INC_REC; justStarted = true;
  return incHdr.id;
}

static void finishIncident() {
  if (incState != INC_REC) return;
  incState = INC_IDLE;
  incHdr.count = (uint16_t)incCount;
  bool ok = writeIncident();
  Serial.printf("#INCIDENT %u %s (%d samples, %u KB flash free)\n", (unsigned)incHdr.id, ok ? "saved" : "SAVE FAILED", incCount, (unsigned)(fsFree() / 1024));
  if (!ok) {
    char msg[140];
    snprintf(msg, sizeof(msg), "💾❌ Storage error: incident #%u could NOT be saved!", (unsigned)incHdr.id);
    tgEnqueue(msg);
  }
  if (ok && lastCleaned > 0) {
    char msg[160];
    snprintf(msg, sizeof(msg), "🧹 Flash was almost full. %d oldest incident%s deleted automatically, newest kept.", lastCleaned, lastCleaned > 1 ? "s were" : " was");
    tgEnqueue(msg); lastCleaned = 0;
  }
  if (screen == S_INC_LIST) { rebuildFilter(); dirty = true; }
}

static void appendIncidentSample(const Sample& s) {
  if (incCount < g_incMax) incBuf[incCount++] = s;
  if (--incRemain <= 0 || incCount >= g_incMax) finishIncident();
}

// ============================ 14. RULE ENGINE =============================
static void faultRaise(int fid, int sev) {
  fSev[fid] = (uint8_t)sev;
  if (nRaised < 160) { raisedList[2 * nRaised] = (uint8_t)fid; raisedList[2 * nRaised + 1] = (uint8_t)sev; nRaised++; }
  else raisedOverflow++;
  fMuted = false; dirty = true;
}
static void faultClear(int fid) {
  int oldSev = fSev[fid];
  fSev[fid] = 0; fAlertSev[fid] = 0; dirty = true;          // fixed: the next time it comes back it alerts again
  flogAdd(fid, oldSev, 1, latest, 0);
  char a[40]; faultName(fid, a, sizeof(a), true);
  Serial.printf("#CLEAR %s\n", a);
#if TG_SEND_CLEARED
  char msg[140]; faultName(fid, a, sizeof(a), false);
  snprintf(msg, sizeof(msg), "✅ CLEARED: %s is back to normal", a);
  tgEnqueue(msg);
#endif
}

// after the rules ran for one sample: start / extend the black-box recording and queue the Telegram message
static void flushAlerts(const Sample& ref, bool noPost) {
  if (nRaised == 0) return;
  uint16_t id = 0;
  if (incState == INC_IDLE) { if (ringCount > 0) id = startIncident(ref); }
  else { id = incHdr.id; incRemain = g_postN; }
  if (incState == INC_REC) {
    int curIdx = (justStarted || noPost) ? incCount - 1 : incCount;      // index this sample has / will have in the recording
    if (curIdx >= g_incMax) curIdx = g_incMax - 1;
    for (int k = 0; k < nRaised; k++) incAddFault(raisedList[2 * k], raisedList[2 * k + 1], curIdx, faultLimit(raisedList[2 * k], raisedList[2 * k + 1]), ref);
  }

  uint32_t now = millis();
  for (int k = 0; k < nRaised; k++) {
    int fid = raisedList[2 * k], sev = raisedList[2 * k + 1];
    char a[40], c[24]; faultName(fid, a, sizeof(a), true); faultCode(fid, sev, c, sizeof(c));
    Serial.printf("#FAULT %s sev=%d code=%s incident=%u\n", a, sev, c, (unsigned)id);
    flogAdd(fid, sev, 0, ref, id);                                   // saved to flash at once
    bool allow = fAlertSev[fid] ? (sev > fAlertSev[fid])             // already alerted and still active: only if it got worse
                                : (fAlertMs[fid] == 0 || now - fAlertMs[fid] >= ALERT_MIN_GAP_MS);
    if (allow) {
      fAlertMs[fid] = now; fAlertSev[fid] = (uint8_t)sev;
      if (nPend == 0) { pendSince = now; pendRef = ref; }
      pendAdd(fid, sev);
      if (id) pendId = id;
    }
  }
  if (raisedOverflow) { pendExtra += raisedOverflow; raisedOverflow = 0; }
  nRaised = 0;
  if (noPost && incState == INC_REC) finishIncident();
}

static void evalRules(const Sample& s, uint32_t now) {
  static uint8_t w[NFAULT];
  memset(w, 0, sizeof(w));
  bool running = svOk(s, SN_RPM) && sv(s, SN_RPM) > RUNNING_RPM;
  float spd = svOk(s, SN_SPEED) ? sv(s, SN_SPEED) : 0.0f;

  for (int i = 0; i < NSENS; i++) {
    const SensInfo& si = SENS[i];
    if (!svOk(s, i)) { w[FID(i, K_SENSOR)] = (si.flags & SF_CRIT) ? 2 : 1; havePrev[i] = false; continue; }   // no signal
    float v = sv(s, i);
    if (v < si.pmin || v > si.pmax) { w[FID(i, K_SENSOR)] = (si.flags & SF_CRIT) ? 2 : 1; continue; }         // impossible value
    bool jumped = si.jump > 0 && havePrev[i] && fabsf(v - prevVal[i]) > si.jump;                             // impossible sudden jump
    prevVal[i] = v; havePrev[i] = true;
    if (jumped) glitchUntil[i] = now + 4000;
    if (now < glitchUntil[i]) w[FID(i, K_SENSOR)] = 1;                                                       // limits are still checked below
    if (v >= CHI(i)) w[FID(i, K_HIGH)] = 2; else if (v >= WHI(i)) w[FID(i, K_HIGH)] = 1;
    if (!(si.flags & SF_RUN) || running) {
      if (v <= CLO(i)) w[FID(i, K_LOW)] = 2; else if (v <= WLO(i)) w[FID(i, K_LOW)] = 1;
    }
  }
  if (spd > 15.0f) {                               // wheel speed plausibility (ABS), 6 wheels
    float ws[6]; int c = 0;
    for (int j = 0; j < 6; j++) if (svOk(s, SN_WS1 + j)) ws[c++] = sv(s, SN_WS1 + j);
    if (c == 6) {
      float t[6]; memcpy(t, ws, sizeof(t));
      for (int a = 0; a < 5; a++) for (int b = a + 1; b < 6; b++) if (t[b] < t[a]) { float x = t[a]; t[a] = t[b]; t[b] = x; }
      float med = (t[2] + t[3]) * 0.5f;
      for (int j = 0; j < 6; j++) if (fabsf(ws[j] - med) > WHEEL_MISMATCH_KMH) w[FID(SN_WS1 + j, K_MISMATCH)] = 1;
    }
  }
  // pedal 1 vs pedal 2 (two redundant sensors must agree)
  if (svOk(s, SN_APP) && svOk(s, SN_APP2) && fabsf(sv(s, SN_APP) - sv(s, SN_APP2)) > PEDAL_MISMATCH_PCT) w[FID(SN_APP2, K_MISMATCH)] = 1;
  // rail pressure must follow the desired value while the engine runs
  if (running && svOk(s, SN_FRP) && svOk(s, SN_FRPD) && fabsf(sv(s, SN_FRP) - sv(s, SN_FRPD)) > RAIL_DEVIATION_BAR) w[FID(SN_FRP, K_MISMATCH)] = 1;
  for (int fid = 0; fid < NFAULT; fid++) {
    if (fid == F_LINK) continue;
    uint8_t ww = w[fid];
    uint8_t deb = ((fid & 3) == K_SENSOR) ? 1 : DEBOUNCE_SAMPLES;
    if (ww) {
      fClr[fid] = 0; if (fCnt[fid] < 255) fCnt[fid]++;
      if (fCnt[fid] >= deb && (fSev[fid] == 0 || ww > fSev[fid])) faultRaise(fid, ww);
    } else {
      fCnt[fid] = 0;
      if (fSev[fid]) { if (++fClr[fid] >= CLEAR_SAMPLES) faultClear(fid); } else fClr[fid] = 0;
    }
  }
}

static void handleSample(const Sample& s, uint8_t src) {
  uint32_t now = millis();
  justStarted = false;
  latest = s; haveData = true; lastDataMs = now; rxLines++; lastSrc = src;
  ring[ringHead] = s; ringHead = (ringHead + 1) % g_preN; if (ringCount < g_preN) ringCount++;
  nRaised = 0;
  evalRules(s, now);
  flushAlerts(s, false);
  if (incState == INC_REC && !justStarted) appendIncidentSample(s);
  if (screen != S_REPLAY && screen != S_INC_DETAIL && screen != S_SNAP) dirty = true;
}

static void linkCheck(uint32_t now) {
  bool lost = haveData && (now - lastDataMs > LINK_TIMEOUT_MS);
  if (lost && fSev[F_LINK] == 0) {
    Sample ref; memset(&ref, 0, sizeof(ref)); ref.t = now;
    if (ringCount > 0) ref = ring[(ringHead - 1 + g_preN) % g_preN];
    nRaised = 0; justStarted = false;
    faultRaise(F_LINK, 2);
    flushAlerts(ref, true);                         // no more samples will come, save now
  } else if (!lost && fSev[F_LINK]) {
    fSev[F_LINK] = 0; fAlertSev[F_LINK] = 0; dirty = true;
    flogAdd(F_LINK, 2, 1, latest, 0);
    Serial.println("#CLEAR ECU LINK");
#if TG_SEND_CLEARED
    tgEnqueue("📡✅ ECU link restored");
#endif
  }
}

// ============================ 15. DATA INPUT (USB + WiFi) =================
static bool parseData(char* line, Sample& s) {
  // D,v0,...,v(NSENS-1),flags      "x" = sensor gives no signal
  char* save = nullptr;
  char* tok = strtok_r(line + 2, ",", &save);
  int n = 0;
  while (tok && n < NSENS) {
    if (tok[0] == 'x' || tok[0] == 'X' || tok[0] == 'N') s.v[n] = INV;
    else {
      float f = (float)atof(tok) * SENS[n].scale;
      long r = lroundf(f);
      if (r > 32000) r = 32000;
      if (r < -32000) r = -32000;
      s.v[n] = (int16_t)r;
    }
    n++; tok = strtok_r(nullptr, ",", &save);
  }
  parseGot = n;
  if (n < NSENS) return false;
  s.flags = tok ? (uint8_t)atoi(tok) : 0;
  s.t = millis();
  return true;
}

static void processLine(char* line, uint8_t src) {
  if (line[0] == 'D' && line[1] == ',') {
    static Sample s;
    if (parseData(line, s)) handleSample(s, src);
    else { rxBad++; if (rxBad <= 5 || rxBad % 50 == 0) Serial.printf("#BAD D line: got %d values, need %d (bad lines so far %u)\n", parseGot, (int)NSENS, (unsigned)rxBad); dirty = true; }
  } else if (line[0] == 'C' && line[1] == 'R') {      // reset all limits to the defaults
    thrDefaults(); LittleFS.remove("/thr.bin"); dirty = true; Serial.println("#THR reset to defaults");
  } else if (line[0] == 'C' && line[1] == ',') {      // C,sensor,warnLow,critLow,warnHigh,critHigh   (n = no limit)
    char* sv_ = nullptr; char* t[5]; int n = 0;
    char* tok = strtok_r(line + 2, ",", &sv_);
    while (tok && n < 5) { t[n++] = tok; tok = strtok_r(nullptr, ",", &sv_); }
    if (n == 5) {
      int i = atoi(t[0]);
      if (i >= 0 && i < NSENS) {
        thr[i][0] = parseLim(t[1], false); thr[i][1] = parseLim(t[2], false);
        thr[i][2] = parseLim(t[3], true);  thr[i][3] = parseLim(t[4], true);
        thrDirty = true; thrDirtyMs = millis(); dirty = true;
      }
    }
  } else if (line[0] == 'T' && line[1] == ',') {   // time sync from PC (local epoch seconds)
    uint32_t e = (uint32_t)strtoul(line + 2, nullptr, 10);
    if (e > 1700000000UL) { epochBase = e - millis() / 1000; dirty = true; }
  } else if (line[0] == '?') {
    if (src == 1) { Serial.print("#CATALOG_BEGIN\n"); printCatalog(Serial); Serial.print("\n#CATALOG_END\n"); }
  }
}

static void serialPump() {
  int guard = 0;
  while (Serial.available() && guard++ < 1500) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (lineLen > 0) { lineBuf[lineLen] = 0; processLine(lineBuf, 1); lineLen = 0; }
    } else if (lineLen < (int)sizeof(lineBuf) - 1) {
      lineBuf[lineLen++] = c;
    } else {
      lineLen = 0;
    }
  }
}

#if PIN_LINK_RX >= 0
static void uartPump() {                                  // wires from the S3 fake ECU
  int guard = 0;
  while (Serial2.available() && guard++ < 1500) {
    char c = (char)Serial2.read();
    if (c == '\n' || c == '\r') {
      if (lineLen2 > 0) { lineBuf2[lineLen2] = 0; processLine(lineBuf2, 3); lineLen2 = 0; }
    } else if (lineLen2 < (int)sizeof(lineBuf2) - 1) {
      lineBuf2[lineLen2++] = c;
    } else {
      lineLen2 = 0; rxBad++;
    }
  }
}
#else
static void uartPump() {}
#endif

// ---- JSON catalog (the dashboard builds itself from this) ----
static void jlim(char* b, size_t n, float v) { if (v > 1e8f || v < -1e8f) snprintf(b, n, "null"); else snprintf(b, n, "%g", v); }
static void printCatalog(Print& out) {
  char t[520], a[16], b[16], c[16], d[16];
  out.print("{\"v\":3,\"nsens\":"); out.print(NSENS); out.print(",\"systems\":[");
  for (int i = 0; i < NSYS; i++) {
    uint16_t col = SYS[i].color;
    snprintf(t, sizeof(t), "%s{\"name\":\"%s\",\"icon\":%d,\"color\":\"#%02x%02x%02x\"}", i ? "," : "", SYS[i].name, (int)SYS[i].icon,
             (((col >> 11) & 31) << 3), (((col >> 5) & 63) << 2), ((col & 31) << 3));
    out.print(t);
  }
  out.print("],\"subs\":[");
  for (int i = 0; i < NSUB; i++) {
    snprintf(t, sizeof(t), "%s{\"name\":\"%s\",\"sys\":%d}", i ? "," : "", SUBS[i].name, (int)SUBS[i].sys);
    out.print(t);
  }
  out.print("],\"sensors\":[\n");
  for (int i = 0; i < NSENS; i++) {
    const SensInfo& s = SENS[i];
    jlim(a, sizeof(a), WLO(i)); jlim(b, sizeof(b), CLO(i)); jlim(c, sizeof(c), WHI(i)); jlim(d, sizeof(d), CHI(i));
    snprintf(t, sizeof(t), "%s{\"i\":%d,\"key\":\"%s\",\"name\":\"%s\",\"unit\":\"%s\",\"sys\":%d,\"sub\":%d,\"scale\":%d,\"dec\":%d,"
             "\"lo\":%g,\"hi\":%g,\"wLo\":%s,\"cLo\":%s,\"wHi\":%s,\"cHi\":%s,\"pmin\":%g,\"pmax\":%g,\"jump\":%g,\"flags\":%d,\"spn\":%u}\n",
             i ? "," : "", i, s.key, s.name, s.unit, (int)s.sys, (int)s.sub, (int)s.scale, (int)s.dec,
             s.lo, s.hi, a, b, c, d, s.pmin, s.pmax, s.jump, (int)s.flags, (unsigned)s.spn);
    out.print(t);
  }
  out.print("]}");
}

// ---- WiFi web server ----
class WebPrint : public Print {
 public:
  char buf[480]; size_t n = 0;
  size_t write(uint8_t c) override { buf[n++] = (char)c; if (n >= sizeof(buf)) flushOut(); return 1; }
  size_t write(const uint8_t* b, size_t len) override { for (size_t i = 0; i < len; i++) write(b[i]); return len; }
  void flushOut() { if (n) { web.sendContent(buf, n); n = 0; } }
};
static void corsHeaders() {
  web.sendHeader("Access-Control-Allow-Origin", "*");
  web.sendHeader("Access-Control-Allow-Private-Network", "true");
  web.sendHeader("Access-Control-Allow-Methods", "GET,POST,OPTIONS");
  web.sendHeader("Access-Control-Allow-Headers", "*");
}
static void ingestText(const char* text, uint8_t src) {          // text may hold several lines
  static char wb[1700]; int n = 0;
  for (const char* p = text; ; ++p) {
    char c = *p;
    if (c == '\n' || c == '\r' || c == 0) {
      if (n > 0) { wb[n] = 0; processLine(wb, src); n = 0; }
      if (c == 0) break;
    } else if (n < (int)sizeof(wb) - 1) wb[n++] = c;
    else n = 0;
  }
}
static void hData() {
  corsHeaders();
  if (web.method() == HTTP_OPTIONS) { web.send(204); return; }
  ingestText(web.arg("plain").c_str(), 2);
  web.send(200, "text/plain", "ok");
}
static void hCatalog() {
  corsHeaders();
  web.setContentLength(CONTENT_LENGTH_UNKNOWN);
  web.send(200, "application/json", "");
  WebPrint wp; printCatalog(wp); wp.flushOut();
  web.sendContent("");
}
static void hStatus() {
  corsHeaders();
  static char b[2400]; int len = 0, cap = (int)sizeof(b);
  len = app(b, len, cap, "{\"link\":%s,\"src\":\"%s\",\"sev\":%d,\"active\":%d,\"rx\":%u,\"incidents\":%d,\"flashFreeKB\":%u,"
                         "\"tg\":%d,\"staIp\":\"%s\",\"apIp\":\"%s\",\"faults\":[",
            linkOk() ? "true" : "false", lastSrc == 1 ? "usb" : (lastSrc == 3 ? "uart" : (lastSrc == 2 ? "wifi" : "none")), (int)overallSev(), activeCount(),
            (unsigned)rxLines, metaN, (unsigned)(fsFree() / 1024), (int)tgState,
            WiFi.localIP().toString().c_str(), WiFi.softAPIP().toString().c_str());
  int c = 0;
  for (int f = 0; f < NFAULT && len < cap - 200; f++) if (fSev[f]) {
    char nm[40], code[24]; faultName(f, nm, sizeof(nm), false); faultCode(f, fSev[f], code, sizeof(code));
    len = app(b, len, cap, "%s{\"fid\":%d,\"sev\":%d,\"name\":\"%s\",\"code\":\"%s\"}", c ? "," : "", f, (int)fSev[f], nm, code);
    c++;
  }
  len = app(b, len, cap, "]}");
  web.send(200, "application/json", b);
}
static void hRoot() {
  corsHeaders();
  if (LittleFS.exists("/dash.html")) {
    File f = LittleFS.open("/dash.html", FILE_READ);
    if (f) { web.streamFile(f, "text/html"); f.close(); return; }
  }
  web.send(200, "text/html",
           "<html><body style='font-family:sans-serif;padding:20px'><h2>Truck Black Box is online</h2>"
           "<p>No dashboard stored yet. Send one with <code>POST /dash</code> (HTML body), or open your dashboard file on the PC.</p>"
           "<p><a href='/catalog'>/catalog</a> &nbsp; <a href='/status'>/status</a></p></body></html>");
}
static void hDash() {
  corsHeaders();
  if (web.method() == HTTP_OPTIONS) { web.send(204); return; }
  if (web.method() != HTTP_POST) { web.send(405, "text/plain", "POST html here"); return; }
  const String& body = web.arg("plain");
  if (body.length() < 20 || body.length() > 180000) { web.send(400, "text/plain", "bad size"); return; }
  File f = LittleFS.open("/dash.html", FILE_WRITE);
  if (!f) { web.send(500, "text/plain", "flash error"); return; }
  f.write((const uint8_t*)body.c_str(), body.length()); f.close();
  web.send(200, "text/plain", "saved");
}
static void hNotFound() {
  corsHeaders();
  if (web.method() == HTTP_OPTIONS) { web.send(204); return; }
  web.send(404, "text/plain", "not found");
}
static void webSetup() {
  web.on("/", HTTP_ANY, hRoot);
  web.on("/d", HTTP_ANY, hData);
  web.on("/catalog", HTTP_ANY, hCatalog);
  web.on("/status", HTTP_ANY, hStatus);
  web.on("/dash", HTTP_ANY, hDash);
  web.onNotFound(hNotFound);
  web.begin();
}

// ============================ 16. UI BUILDING BLOCKS ======================

// ---- buzzer: passive buzzer = PWM tone (LEDC), active buzzer = plain ON/OFF ----
#define BZ_TIMER LEDC_TIMER_3
#define BZ_CH    LEDC_CHANNEL_7
static bool bzReady = false, bzOn = false; static int bzFreqNow = 0;
static uint32_t bzUntil = 0; static int bzBeepFreq = BUZZER_FREQ;
static void buzzerInit() {
#if PIN_BUZZER >= 0
#if BUZZER_ACTIVE
  pinMode(PIN_BUZZER, OUTPUT); digitalWrite(PIN_BUZZER, BUZZER_INVERT ? HIGH : LOW);
#else
  ledc_timer_config_t t; memset(&t, 0, sizeof(t));
  t.speed_mode = LEDC_LOW_SPEED_MODE; t.duty_resolution = LEDC_TIMER_8_BIT; t.timer_num = BZ_TIMER;
  t.freq_hz = BUZZER_FREQ; t.clk_cfg = LEDC_AUTO_CLK;
  ledc_timer_config(&t);
  ledc_channel_config_t c; memset(&c, 0, sizeof(c));
  c.gpio_num = PIN_BUZZER; c.speed_mode = LEDC_LOW_SPEED_MODE; c.channel = BZ_CH; c.intr_type = LEDC_INTR_DISABLE;
  c.timer_sel = BZ_TIMER; c.duty = 0; c.hpoint = 0;
  ledc_channel_config(&c);
  bzFreqNow = BUZZER_FREQ;
#endif
  bzReady = true;
#endif
}
static void buzzerSet(bool on, int freq) {
#if PIN_BUZZER >= 0
  if (!bzReady) return;
#if BUZZER_ACTIVE
  (void)freq;
  digitalWrite(PIN_BUZZER, (on != (bool)BUZZER_INVERT) ? HIGH : LOW);
#else
  if (on) {
    if (freq != bzFreqNow) { ledc_set_freq(LEDC_LOW_SPEED_MODE, BZ_TIMER, (uint32_t)freq); bzFreqNow = freq; }
    ledc_set_duty(LEDC_LOW_SPEED_MODE, BZ_CH, 128);
  } else ledc_set_duty(LEDC_LOW_SPEED_MODE, BZ_CH, 0);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, BZ_CH);
#endif
#else
  (void)on; (void)freq;
#endif
}
static void beep(int ms, int freq) { bzUntil = millis() + (uint32_t)ms; bzBeepFreq = freq; }
static void buzzerTestTune() {                          // blocking, about half a second
  for (int i = 0; i < 3; i++) { buzzerSet(true, 1800 + i * 700); delay(130); buzzerSet(false, 0); delay(80); }
}
static void buzzerTick(uint32_t now) {
  bool on = false; int f = BUZZER_FREQ;
  if (now < bzUntil) { on = true; f = bzBeepFreq; }
  else {
    uint8_t sev = overallSev();
    if (sev && !fMuted) {
      if (sev >= 2) { uint32_t t = now % 900; on = (t < 130) || (t >= 230 && t < 360); f = 3200; }   // beep-beep ... beep-beep
      else          { on = (now % 3000) < 140; f = 2300; }                                         // single beep every 3 s
    }
  }
  if (on != bzOn || (on && f != bzFreqNow)) { buzzerSet(on, f); bzOn = on; }
}
static void drawBg() { gradient(0, SCREEN_H, BG_TOP, BG_BOT); }

static void drawHeader(const char* title, uint16_t color, bool back) {
  gradient(0, 15, color, lerp565(color, C_BLACK, 70));
  uint16_t tc = textOn(color);
  if (back) drawText(3, 4, "<", 1, tc, TRANSP);
  drawTextN(back ? 14 : 5, 4, title, 17, tc);
  fillRect(SCREEN_W - 10, 3, 8, 8, C_WHITE);
  fillRect(SCREEN_W - 9, 4, 6, 6, linkOk() ? COL_OK : (haveData ? COL_CRIT : C_GREY));
}
static void drawFooter(const char* t) { drawText(3, 151, t, 1, C_MUTED, TRANSP); }
static void drawToast() {
  if (millis() < toastUntil && toastMsg[0]) {
    int w = textW(toastMsg, 1) + 10;
    fillCard((SCREEN_W - w) / 2, SCREEN_H - 28, w, 15, C_INK);
    drawText((SCREEN_W - w) / 2 + 5, SCREEN_H - 24, toastMsg, 1, C_WHITE, TRANSP);
  }
}
static void infoCard(int x, int y, int w, int h) { fillCard(x, y, w, h, CARD1); drawRect(x, y, w, h, CARD_LINE); }

static void addRow(const char* a, const char* b, uint16_t ac, uint16_t bc, uint16_t dot, int icon, uint16_t ibg, int tag) {
  if (nRows >= MAXROWS) return;
  Row& r = rows[nRows++];
  snprintf(r.a, sizeof(r.a), "%s", a); snprintf(r.b, sizeof(r.b), "%s", b);
  r.ac = ac; r.bc = bc; r.dot = dot; r.icon = (int8_t)icon; r.ibg = ibg; r.tag = (int16_t)tag;
}
static void listClamp() {
  if (lcur >= nRows) lcur = nRows - 1;
  if (lcur < 0) lcur = 0;
  if (lcur < ltop) ltop = lcur;
  if (lcur >= ltop + listVis) ltop = lcur - listVis + 1;
  if (ltop < 0) ltop = 0;
}
static void drawList() {
  listClamp();
  bool tall = rowH >= 22;
  for (int r = 0; r < listVis; r++) {
    int idx = ltop + r;
    if (idx >= nRows) break;
    const Row& rw = rows[idx];
    int y = listY + r * rowH;
    bool sel = (idx == lcur);
    fillCard(2, y, SCREEN_W - 5, rowH - 2, sel ? SELBG : ((idx & 1) ? CARD2 : CARD1));
    drawRect(2, y, SCREEN_W - 5, rowH - 2, sel ? C_INK : CARD_LINE);
    int x = 7;
    if (rw.dot) { fillRect(3, y + 1, 3, rowH - 4, rw.dot); x = 9; }
    if (rw.icon >= 0) {
      int bh = rowH - 4; if (bh > 20) bh = 20;
      fillCard(x, y + 1, bh + 2, bh, rw.ibg);
      drawIcon(x + (bh + 2 - 16) / 2, y + 1 + (bh - 16) / 2, rw.icon, textOn(rw.ibg), 1);
      x += bh + 6;
    }
    uint16_t ac = sel ? C_WHITE : rw.ac;
    uint16_t bc = sel ? C_WHITE : rw.bc;
    if (tall) {
      drawTextN(x, y + 3, rw.a, (SCREEN_W - 8 - x) / 6, ac);
      drawTextN(x, y + rowH - 11, rw.b, (SCREEN_W - 8 - x) / 6, bc);
    } else {
      int bw = textW(rw.b, 1);
      drawTextN(x, y + (rowH - 10) / 2, rw.a, (SCREEN_W - 10 - x - bw) / 6, ac);
      drawTextR(SCREEN_W - 7, y + (rowH - 10) / 2, rw.b, bc);
    }
  }
  if (nRows > listVis) {
    int th = listVis * rowH;
    int bh = th * listVis / nRows; if (bh < 6) bh = 6;
    fillRect(SCREEN_W - 2, listY + (th - bh) * ltop / (nRows - listVis), 2, bh, BLUE);
  }
}

static void applyGeom(Screen s) {
  switch (s) {
    case S_MENU:       rowH = 24; listY = 18; listVis = 5; break;
    case S_SYSTEMS:    rowH = 24; listY = 17; listVis = 5; break;
    case S_SUBS:       rowH = 22; listY = 17; listVis = 6; break;
    case S_SENSORS:    rowH = 22; listY = 16; listVis = 6; break;
    case S_SCAN:       rowH = 22; listY = 30; listVis = 5; break;
    case S_CODES:
    case S_HIST:
    case S_FLOG:
    case S_SNAP:       rowH = 22; listY = 16; listVis = 6; break;
    case S_STORAGE:    rowH = 26; listY = 66; listVis = 3; break;
    case S_INC_DETAIL: rowH = 17; listY = 39; listVis = 4; break;
    default: break;
  }
}
static bool isListScreen(Screen s) {
  return s == S_MENU || s == S_SCAN || s == S_SYSTEMS || s == S_SUBS || s == S_SENSORS || s == S_HIST ||
         s == S_CODES || s == S_INC_DETAIL || s == S_SNAP || s == S_STORAGE || s == S_FLOG;
}
static void navForward(Screen s) {
  if (navN < 16) { Nav& n = navStack[navN++]; n.s = screen; n.lcur = lcur; n.ltop = ltop; n.sys = curSys; n.sub = curSub; n.sens = curSens; }
  screen = s; lcur = 0; ltop = 0; typing = false; confirmArm = 0; applyGeom(s); dirty = true;
}
static void navBack() {
  if (navN <= 0) { screen = S_DRIVER; dirty = true; return; }
  navN--;
  screen = navStack[navN].s; lcur = navStack[navN].lcur; ltop = navStack[navN].ltop;
  curSys = navStack[navN].sys; curSub = navStack[navN].sub; curSens = navStack[navN].sens;
  typing = false; confirmArm = 0; applyGeom(screen); dirty = true;
}
static void navHome() { navN = 0; screen = S_DRIVER; typing = false; confirmArm = 0; dirty = true; }
static void openMechanic() {                           // like an OBD scan tool: if something is wrong, show it immediately
  navN = 0; navForward(S_HOME);                          // Nokia-style app grid; the SCAN app starts the full scan
}

// ============================ 17. SCREENS =================================
static void tile(int x, int y, int w, const char* label, const char* val, uint16_t vc) {
  fillCard(x, y, w, 24, CARD1); drawRect(x, y, w, 24, CARD_LINE);
  fillRect(x + 1, y + 2, 2, 20, BLUE);
  drawText(x + 5, y + 3, label, 1, C_MUTED, TRANSP);
  drawText(x + 5, y + 13, val, 1, vc, TRANSP);
}
static uint16_t valColor(int i, bool ok) {
  if (!ok) return C_GREY;
  uint8_t s = sevOfValue(i, sv(latest, i), engineRunning());
  return s >= 2 ? COL_CRIT : (s == 1 ? COL_WARN : C_INK);
}

static void renderDriver() {
  uint8_t sev = overallSev();
  bool ok = linkOk();
  uint16_t t1, t2, pill;
  if (!haveData || !ok)  { t1 = RGB565(246, 248, 252); t2 = RGB565(212, 220, 234); pill = RGB565(120, 130, 152); }
  else if (sev >= 2)     { t1 = RGB565(255, 240, 240); t2 = RGB565(255, 176, 176); pill = COL_CRIT; }
  else if (sev == 1)     { t1 = RGB565(255, 250, 232); t2 = RGB565(255, 222, 140); pill = COL_WARN; }
  else                   { t1 = BG_TOP;                t2 = BG_BOT;                pill = COL_OK; }
  gradient(0, SCREEN_H, t1, t2);
  const char* st = !haveData ? "WAITING" : (!ok ? "NO ECU" : (sev >= 2 ? "DANGER" : (sev == 1 ? "WARNING" : "ALL GOOD")));
  fillCard(6, 4, 116, 22, pill);
  drawTextC(SCREEN_W / 2, 7, st, 2, textOn(pill), TRANSP);

  char b[44], a[32];
  int tf = topFault();
  if (!haveData) snprintf(b, sizeof(b), "waiting for ECU data");
  else if (tf >= 0) faultName(tf, b, sizeof(b), false);
  else snprintf(b, sizeof(b), "All systems normal");
  drawTextN(4, 30, b, 21, C_INK);

  float spd = curSpeed();
  snprintf(a, sizeof(a), "%d", (int)(spd + 0.5f));
  drawText(6, 42, a, 4, C_INK, TRANSP);
  drawText(6 + textW(a, 4) + 4, 64, "km/h", 1, C_MUTED, TRANSP);
  if (haveData && svOk(latest, SN_GEAR)) { snprintf(a, sizeof(a), "G%d", (int)(sv(latest, SN_GEAR) + 0.5f)); drawText(100, 44, a, 2, BLUE, TRANSP); }

  bool d = haveData;
  snprintf(a, sizeof(a), "%.0f", d ? sv(latest, SN_RPM) : 0.0f);               tile(3, 78, 38, "RPM", a, valColor(SN_RPM, d));
  snprintf(a, sizeof(a), "%.0fC", d ? sv(latest, SN_ECT) : 0.0f);              tile(45, 78, 38, "COOL", a, valColor(SN_ECT, d));
  snprintf(a, sizeof(a), "%.0f", d ? sv(latest, SN_OILP) : 0.0f);              tile(87, 78, 38, "OIL", a, valColor(SN_OILP, d));
  if (sev == 0 || !ok) {
    snprintf(a, sizeof(a), "%.1fV", d ? sv(latest, SN_BATV) : 0.0f);           tile(3, 104, 38, "BATT", a, valColor(SN_BATV, d));
    snprintf(a, sizeof(a), "%.0f%%", d ? sv(latest, SN_FUEL) : 0.0f);          tile(45, 104, 38, "FUEL", a, valColor(SN_FUEL, d));
    snprintf(a, sizeof(a), "%.0f%%", d ? sv(latest, SN_LOAD) : 0.0f);          tile(87, 104, 38, "LOAD", a, valColor(SN_LOAD, d));
    drawIcon(5, 134, IC_OK, COL_OK, 1);
    drawText(25, 138, ok ? "ALL SYSTEMS OK" : "NO ECU DATA", 1, ok ? COL_OK : C_GREY, TRANSP);
    drawText(4, 152, canOpenMenu() ? "OK button = scan" : "Stop to open menu", 1, C_MUTED, TRANSP);
  } else {
    infoCard(3, 104, 122, 34);
    drawWrapped(7, 108, adviceFor(tf >= 0 ? tf / 4 : 0), 20, 3, C_INK);
    int x = 4;                                           // icons of the systems that have a problem
    for (int s = 0; s < NSYS && x < SCREEN_W - 20; s++) {
      int c; uint8_t ss = systemSev(s, &c);
      if (!c) continue;
      fillCard(x, 140, 18, 18, sevColor(ss)); drawIcon(x + 1, 141, SYS[s].icon, textOn(sevColor(ss)), 1);
      x += 21;
    }
    if (fSev[F_LINK] && x < SCREEN_W - 20) { fillCard(x, 140, 18, 18, COL_CRIT); drawIcon(x + 1, 141, IC_WARN, C_WHITE, 1); }
  }
  drawToast();
}

static void renderMenu() {
  drawBg(); drawHeader("SCAN TOOL", BLUE, false);
  nRows = 0; char b[24]; int act = activeCount();
  if (act) snprintf(b, sizeof(b), "%d problem%s", act, act > 1 ? "s" : ""); else snprintf(b, sizeof(b), "no problems");
  addRow("AUTO SCAN", b, C_INK, act ? COL_CRIT : COL_OK, 0, IC_SCAN, act ? COL_CRIT : COL_OK, 0);
  snprintf(b, sizeof(b), "%d sensors", NSENS);
  addRow("SYSTEM SCAN", b, C_INK, C_MUTED, 0, IC_ENGINE, RGB565(22, 110, 235), 1);
  addRow("FAULT CODES", act ? "now + history" : "history", C_INK, C_MUTED, 0, IC_WARN, RGB565(230, 130, 20), 2);
  snprintf(b, sizeof(b), "%d events", flogN);
  addRow("FAULT LOG", b, C_INK, C_MUTED, 0, IC_WARN, RGB565(200, 60, 70), 6);
  snprintf(b, sizeof(b), "%d saved", metaN);
  addRow("BLACK BOX", b, C_INK, C_MUTED, 0, IC_BOX, RGB565(130, 80, 215), 3);
  addRow("STORAGE/TOOLS", "erase, test", C_INK, C_MUTED, 0, IC_STATUS, RGB565(70, 100, 150), 4);
  addRow("DEVICE STATUS", "wifi, ecu", C_INK, C_MUTED, 0, IC_OK, RGB565(40, 150, 110), 5);
  drawList();
  drawFooter("L/R move  OK open");
  drawToast();
}

static void renderScan() {
  drawBg(); drawHeader("AUTO SCAN", activeCount() ? COL_CRIT : BLUE, true);
  int act = activeCount(); char b[40];
  uint16_t chip; const char* msg;
  if (!haveData) { chip = C_GREY; msg = "WAITING FOR ECU DATA"; }
  else if (!act) { chip = COL_OK; msg = flogN ? "OK NOW - SEE HISTORY" : "NO FAULTS - ALL OK"; }
  else { chip = sevColor(overallSev()); snprintf(b, sizeof(b), "%d ACTIVE PROBLEM%s", act, act > 1 ? "S" : ""); msg = b; }
  fillCard(3, 17, 122, 12, chip);
  drawTextC(SCREEN_W / 2, 20, msg, 1, textOn(chip), TRANSP);
  nRows = 0;
  if (act) {
    char a[40], l[20], c[20];
    for (int pass = 2; pass >= 1; pass--)
      for (int f = 0; f < NFAULT; f++) {
        if (fSev[f] != pass) continue;
        faultName(f, a, sizeof(a), false);
        faultLine(f, pass, latest, l, sizeof(l));
        int si = f / 4;
        if (si >= NSENS) snprintf(c, sizeof(c), "%s  U0100", l);
        else if (SENS[si].spn) snprintf(c, sizeof(c), "%s SPN%u", l, (unsigned)SENS[si].spn);
        else snprintf(c, sizeof(c), "%s", l);
        addRow(a, c, C_INK, sevColor(pass), sevColor(pass), -1, 0, f);
      }
    drawList();
    drawFooter("L/R move  OK details");
  } else if (flogN > 0) {                                  // nothing active: show what happened before (newest first)
    char a2[40], b2[24], tm2[24];
    for (int i = flogN - 1; i >= 0 && nRows < 40; i--) {
      const FEvt& e = flog[i];
      faultName(e.fid, a2, sizeof(a2), false);
      fmtClockTiny(e.epoch, e.ms, tm2, sizeof(tm2));
      snprintf(b2, sizeof(b2), "%s %s", tm2, e.kind ? "CLEARED" : (e.sev >= 2 ? "CRIT" : "WARN"));
      addRow(a2, b2, e.kind ? C_MUTED : C_INK, C_MUTED, e.kind ? COL_OK : sevColor(e.sev), -1, 0, i);
    }
    drawList();
    drawFooter("OK: what happened");
  } else {
    int nosig = 0;
    if (haveData) for (int i = 0; i < NSENS; i++) if (latest.v[i] == INV) nosig++;
    drawIcon(40, 50, IC_OK, haveData ? COL_OK : C_GREY, 3);
    drawTextC(SCREEN_W / 2, 104, haveData ? "ALL SYSTEMS OK" : "NO DATA YET", 1, haveData ? COL_OK : C_MUTED, TRANSP);
    snprintf(b, sizeof(b), "%d sensors scanned", haveData ? NSENS - nosig : 0);
    drawTextC(SCREEN_W / 2, 118, b, 1, C_MUTED, TRANSP);
    snprintf(b, sizeof(b), "%d problems saved", fSev[F_LINK] ? 0 : (int)metaN);
    drawTextC(SCREEN_W / 2, 130, b, 1, C_MUTED, TRANSP);
    drawFooter("hold OK = back");
  }
  drawToast();
}

static void renderSystems() {
  drawBg(); drawHeader("SYSTEM SCAN", BLUE, true);
  nRows = 0; char b[24];
  for (int s = 0; s < NSYS; s++) {
    int c; uint8_t sv_ = systemSev(s, &c);
    int ns = 0; for (int i = 0; i < NSENS; i++) if (SENS[i].sys == s) ns++;
    if (c) snprintf(b, sizeof(b), "%d PROBLEM%s", c, c > 1 ? "S" : ""); else snprintf(b, sizeof(b), "OK  %d sensors", ns);
    addRow(SYS[s].name, b, C_INK, c ? sevColor(sv_) : COL_OK, 0, SYS[s].icon, SYS[s].color, s);
  }
  drawList();
  drawFooter("L/R move  OK open");
  drawToast();
}

static void renderSubs() {
  drawBg(); drawHeader(SYS[curSys].name, SYS[curSys].color, true);
  nRows = 0; char b[24];
  for (int s = 0; s < NSUB; s++) {
    if (SUBS[s].sys != curSys) continue;
    int c; uint8_t sv_ = subSev(s, &c);
    int ns = 0; for (int i = 0; i < NSENS; i++) if (SENS[i].sub == s) ns++;
    if (c) snprintf(b, sizeof(b), "%d PROBLEM%s", c, c > 1 ? "S" : ""); else snprintf(b, sizeof(b), "OK  %d sensors", ns);
    addRow(SUBS[s].name, b, C_INK, c ? sevColor(sv_) : COL_OK, c ? sevColor(sv_) : SYS[curSys].color, -1, 0, s);
  }
  drawList();
  drawFooter("L/R move  OK open");
  drawToast();
}

static void renderSensors() {
  drawBg(); drawHeader(SUBS[curSub].name, SYS[curSys].color, true);
  nRows = 0; char b[20], v[20];
  bool run = engineRunning();
  for (int i = 0; i < NSENS; i++) {
    if (SENS[i].sub != curSub) continue;
    bool ok = haveData && latest.v[i] != INV;
    uint8_t as = sensorSev(i);
    uint8_t vs = ok ? sevOfValue(i, sv(latest, i), run) : 0;
    uint8_t s = as > vs ? as : vs;
    if (!haveData) snprintf(b, sizeof(b), "--");
    else if (!ok) snprintf(b, sizeof(b), "NO SIGNAL");
    else { fmtVal(i, sv(latest, i), v, sizeof(v), true); snprintf(b, sizeof(b), "%s%s%s", v, s ? " " : "", s ? sevWord(s) : ""); }
    uint16_t col = (!ok && haveData) ? COL_CRIT : (s ? sevColor(s) : C_INK);
    addRow(SENS[i].name, b, C_INK, col, s ? sevColor(s) : SYS[curSys].color, -1, 0, i);
  }
  drawList();
  drawFooter("L/R move  OK details");
  drawToast();
}

static void drawGauge(int i, bool ok, float v) {
  const SensInfo& s = SENS[i];
  const int gx0 = 5, gw = 118, gy = 90, gh = 9;
  float lo = s.lo, hi = s.hi, span = hi - lo;
  if (span <= 0) span = 1;
  fillRect(gx0, gy, gw, gh, COL_OK);
  float cl = CLO(i), wl = WLO(i), wh = WHI(i), ch = CHI(i);
  struct Zn { float a, b; uint16_t c; bool on; } zn[4] = {
    {lo, wl, COL_WARN, wl > -1e8f}, {lo, cl, COL_CRIT, cl > -1e8f}, {wh, hi, COL_WARN, wh < 1e8f}, {ch, hi, COL_CRIT, ch < 1e8f}};
  for (int k = 0; k < 4; k++) {
    if (!zn[k].on) continue;
    float a = clampf(zn[k].a, lo, hi), b = clampf(zn[k].b, lo, hi);
    int xa = gx0 + (int)((a - lo) / span * (gw - 1)), xb = gx0 + (int)((b - lo) / span * (gw - 1));
    if (xb > xa) fillRect(xa, gy, xb - xa + 1, gh, zn[k].c);
  }
  drawRect(gx0 - 1, gy - 1, gw + 2, gh + 2, C_INK);
  if (ok) {
    int xm = gx0 + (int)((clampf(v, lo, hi) - lo) / span * (gw - 1));
    fillRect(xm - 1, gy - 4, 3, gh + 8, C_INK);
    fillRect(xm, gy - 3, 1, gh + 6, C_WHITE);
  }
  char t[16];
  snprintf(t, sizeof(t), "%.*f", (int)s.dec, lo); drawText(gx0, gy + gh + 3, t, 1, C_MUTED, TRANSP);
  snprintf(t, sizeof(t), "%.*f", (int)s.dec, hi); drawTextR(gx0 + gw, gy + gh + 3, t, C_MUTED);
}

static int lastProblemMeta(int sensor, int* kindOut, int* slotOut) {   // newest saved incident that mentions this sensor
  int best = -1, bk = 0;
  for (int k = 0; k < 4; k++) { int m = fLastMeta[FID(sensor, k)]; if (m > best) { best = m; bk = k; } }
  int slot = -1;
  if (best >= 0) for (int k = 0; k < meta[best].nf && k < MAXF; k++) if (meta[best].faults[2 * k] == FID(sensor, bk)) { slot = k; break; }
  if (kindOut) *kindOut = bk;
  if (slotOut) *slotOut = slot;
  return best;
}

static void renderSensor() {
  const SensInfo& si = SENS[curSens]; const SysInfo& sy = SYS[si.sys];
  drawBg(); drawHeader(SUBS[si.sub].name, sy.color, true);
  infoCard(3, 18, 122, 22);
  fillCard(5, 20, 18, 18, sy.color); drawIcon(6, 21, sy.icon, textOn(sy.color), 1);
  drawTextN(27, 21, si.name, 16, C_INK);
  drawTextN(27, 30, sy.name, 16, C_MUTED);

  bool ok = haveData && latest.v[curSens] != INV;
  float v = ok ? sv(latest, curSens) : 0.0f;
  uint8_t as = sensorSev(curSens);
  uint8_t vs = ok ? sevOfValue(curSens, v, engineRunning()) : 0;
  uint8_t sev = as > vs ? as : vs;
  char b[32], u[12];
  if (!haveData) snprintf(b, sizeof(b), "--"); else if (!ok) snprintf(b, sizeof(b), "NO SIGNAL"); else fmtVal(curSens, v, b, sizeof(b), false);
  uint16_t vc = (!ok && haveData) ? COL_CRIT : (sev ? sevColor(sev) : C_INK);
  int sz = (strlen(b) <= 6) ? 3 : 2;
  snprintf(u, sizeof(u), "%s", (si.flags & SF_BOOL) ? "" : si.unit);
  int tw = textW(b, sz) + (u[0] ? 6 + textW(u, 1) : 0);
  int x0 = (SCREEN_W - tw) / 2; if (x0 < 2) x0 = 2;
  drawText(x0, 45, b, sz, vc, TRANSP);
  if (u[0]) drawText(x0 + textW(b, sz) + 6, 45 + 8 * sz - 8, u, 1, C_MUTED, TRANSP);

  const char* msg = "NORMAL"; uint16_t chip = COL_OK;
  int worst = -1; for (int k = 0; k < 4; k++) if (fSev[FID(curSens, k)] && (worst < 0 || fSev[FID(curSens, k)] > fSev[FID(curSens, worst)])) worst = k;
  static char cm[28];
  if (!haveData) { msg = "WAITING FOR DATA"; chip = C_GREY; }
  else if (worst >= 0) {
    uint8_t ws = fSev[FID(curSens, worst)]; char code[20];
    faultCode(FID(curSens, worst), ws, code, sizeof(code));
    if (worst == K_LOW && (si.flags & SF_FUSE)) snprintf(cm, sizeof(cm), "FUSE BLOWN");
    else if (worst == K_LOW && (si.flags & SF_RELAY)) snprintf(cm, sizeof(cm), "RELAY FAILED");
    else if (worst == K_SENSOR) snprintf(cm, sizeof(cm), "%s SENSOR FAULT", ws >= 2 ? "CRIT" : "WARN");
    else if (worst == K_MISMATCH) snprintf(cm, sizeof(cm), "WHEEL MISMATCH");
    else snprintf(cm, sizeof(cm), "%s %s", ws >= 2 ? "CRITICAL" : "WARNING", KIND_NAME[worst]);
    msg = cm; chip = sevColor(ws);
  }
  fillCard(3, 72, 122, 14, chip);
  drawTextC(SCREEN_W / 2, 75, msg, 1, textOn(chip), TRANSP);
  bool fr = (si.flags & (SF_FUSE | SF_RELAY)) != 0;
  char l[44]; int y = 112;
  if (fr) {                                              // fuse / relay: no gauge, a plain explanation
    bool good = ok && v > 0.5f;
    drawText(5, 92, !haveData ? "Waiting for data" : (good ? "Circuit is powered." : "No power on circuit!"), 1,
             good ? COL_OK : (haveData ? COL_CRIT : C_MUTED), TRANSP);
    if (haveData && !good) drawWrapped(5, 104, adviceFor(curSens), 20, 3, C_INK);
    else drawWrapped(5, 104, (si.flags & SF_FUSE) ? "Fuse is good. It guards this circuit." : "Relay is switching normally.", 20, 3, C_MUTED);
  } else {
    drawGauge(curSens, ok, v);
    if (WHI(curSens) < 1e8f || CHI(curSens) < 1e8f) { char a1[12], a2[12]; fmtLim(WHI(curSens), a1, sizeof(a1)); fmtLim(CHI(curSens), a2, sizeof(a2)); snprintf(l, sizeof(l), "HI warn>%s crit>%s", a1, a2); drawTextN(4, y, l, 21, C_MUTED); y += 9; }
    if (WLO(curSens) > -1e8f || CLO(curSens) > -1e8f) { char a1[12], a2[12]; fmtLim(WLO(curSens), a1, sizeof(a1)); fmtLim(CLO(curSens), a2, sizeof(a2)); snprintf(l, sizeof(l), "LO warn<%s crit<%s", a1, a2); drawTextN(4, y, l, 21, C_MUTED); y += 9; }
    if (y == 112) { drawText(4, y, "No limits set", 1, C_MUTED, TRANSP); y += 9; }
    if (si.spn) snprintf(l, sizeof(l), "J1939 SPN %u", (unsigned)si.spn); else snprintf(l, sizeof(l), "No standard code");
    drawTextN(4, y, l, 21, C_MUTED);
  }

  int tot = 0; for (int k = 0; k < 4; k++) tot += fHist[FID(curSens, k)];
  uint16_t hc = tot ? COL_WARN : COL_OK;
  infoCard(3, 139, 122, 19);
  fillRect(4, 140, 3, 17, hc);
  if (tot) {
    int kk, slot; int mi = lastProblemMeta(curSens, &kk, &slot); char tm[28];
    snprintf(l, sizeof(l), "PROBLEMS %d  OK=list", tot); drawTextN(10, 141, l, 19, hc);
    if (mi >= 0 && slot >= 0) {
      fmtClockTiny(meta[mi].fEpoch[slot], meta[mi].fMs[slot], tm, sizeof(tm));
      snprintf(l, sizeof(l), "%s %urpm", tm, (unsigned)meta[mi].fRpm[slot]); drawTextN(10, 150, l, 19, C_INK);
    }
  } else {
    drawText(10, 141, "No problems saved", 1, COL_OK, TRANSP);
    drawText(10, 150, "in the black box", 1, C_MUTED, TRANSP);
  }
  drawToast();
}

static void renderHist() {                               // every saved problem of the selected sensor
  const SensInfo& si = SENS[curSens];
  drawBg(); drawHeader(si.name, SYS[si.sys].color, true);
  nRows = 0; char a[24], b[20], tm[28];
  for (int i = metaN - 1; i >= 0; i--)
    for (int k = 0; k < meta[i].nf && k < MAXF; k++) {
      int fid = meta[i].faults[2 * k];
      if (fid / 4 != curSens) continue;
      fmtClockTiny(meta[i].fEpoch[k], meta[i].fMs[k], tm, sizeof(tm));
      snprintf(a, sizeof(a), "%s %s", tm, KIND_SHORT[fid % 4]);
      snprintf(b, sizeof(b), "%urpm %.0fkm/h", (unsigned)meta[i].fRpm[k], meta[i].fSpd10[k] / 10.0f);
      addRow(a, b, C_INK, C_MUTED, sevColor(meta[i].faults[2 * k + 1]), -1, 0, i * MAXF + k);
    }
  if (nRows == 0) addRow("No problems saved", "", COL_OK, C_MUTED, COL_OK, -1, 0, -1);
  drawList();
  drawFooter("OK: open incident");
  drawToast();
}

static void renderCodes() {
  drawBg(); drawHeader("FAULT CODES", RGB565(230, 130, 20), true);
  nRows = 0; char a[40], b[20], cd[20];
  bool used[NFAULT]; memset(used, 0, sizeof(used));
  for (int pass = 0; pass < 2; pass++) {
    for (;;) {
      int best = -1; long bv = -1;
      for (int f = 0; f < NFAULT; f++) {
        if (used[f]) continue;
        long val;
        if (pass == 0) { if (!fSev[f]) continue; val = fSev[f] * 1000L + (NFAULT - f); }
        else { if (!fHist[f]) continue; val = fLastMeta[f] + 1; }
        if (val > bv) { bv = val; best = f; }
      }
      if (best < 0 || nRows >= 60) break;
      used[best] = true;
      faultName(best, a, sizeof(a), false);
      int hs = 1;
      if (fLastMeta[best] >= 0) { const IncMeta& m = meta[fLastMeta[best]]; for (int k = 0; k < m.nf && k < MAXF; k++) if (m.faults[2 * k] == best) hs = m.faults[2 * k + 1]; }
      faultCode(best, fSev[best] ? fSev[best] : hs, cd, sizeof(cd));
      if (fHist[best]) snprintf(b, sizeof(b), "%s x%u", cd[0] ? cd : "no code", (unsigned)fHist[best]);
      else snprintf(b, sizeof(b), "%s now", cd[0] ? cd : "no code");
      uint16_t dot = fSev[best] ? sevColor(fSev[best]) : C_GREY;
      addRow(a, b, fSev[best] ? C_INK : C_MUTED, C_MUTED, dot, -1, 0, best);
    }
  }
  if (nRows == 0) addRow("No faults recorded", "", COL_OK, C_MUTED, COL_OK, -1, 0, -1);
  drawList();
  drawFooter("xN = times recorded");
  drawToast();
}

// ---- black box: incident list with search bar ----
static bool metaMatches(const IncMeta& m) {
  if (!search[0]) return true;
  char hay[420]; int n = snprintf(hay, sizeof(hay), "%u %04u ", (unsigned)m.id, (unsigned)m.id);
  for (int k = 0; k < m.nf && k < MAXF; k++) {
    int fid = m.faults[2 * k], si = fid / 4;
    if (si < NSENS) n += snprintf(hay + n, sizeof(hay) - n, "%s %s %s %s %s ", SENS[si].key, SENS[si].name, KIND_NAME[fid % 4], SYS[SENS[si].sys].name, SUBS[SENS[si].sub].name);
    else n += snprintf(hay + n, sizeof(hay) - n, "ECU LINK ");
    if (n >= (int)sizeof(hay) - 80) break;
  }
  snprintf(hay + n, sizeof(hay) - n, "%s", m.sev >= 2 ? "CRIT RED" : "WARN YELLOW");
  for (char* p = hay; *p; p++) if (*p >= 'a' && *p <= 'z') *p -= 32;
  return strstr(hay, search) != nullptr;
}
static void rebuildFilter() {
  viewN = 0;
  for (int i = metaN - 1; i >= 0; i--) if (metaMatches(meta[i])) viewIdx[viewN++] = i;
  if (iLcur >= viewN) iLcur = viewN - 1;
  if (iLtop > iLcur) iLtop = iLcur < 0 ? 0 : iLcur;
}

static void renderIncList() {
  drawBg();
  char b[40];
  snprintf(b, sizeof(b), "BLACK BOX  %d", viewN);
  drawHeader(b, RGB565(130, 80, 215), true);
  bool sbSel = (iLcur == -1);
  fillCard(3, 17, 122, 12, typing ? C_WHITE : (sbSel ? SELBG : CARD1));
  drawRect(3, 17, 122, 12, typing ? BLUE : CARD_LINE);
  uint16_t sfg = typing ? C_INK : (sbSel ? C_WHITE : C_MUTED);
  snprintf(b, sizeof(b), "Find: %s%s", search, typing ? "_" : "");
  drawText(7, 20, b, 1, sfg, TRANSP);
  if (typing && mlen) drawText(SCREEN_W - 6 * (mlen + 1) - 4, 20, mbuf, 1, COL_CRIT, TRANSP);
  if (viewN == 0) drawText(10, 70, metaN ? "No match" : "No incidents yet", 1, C_MUTED, TRANSP);
  for (int r = 0; r < 5; r++) {
    int idx = iLtop + r;
    if (idx >= viewN) break;
    const IncMeta& m = meta[viewIdx[idx]];
    int y = 31 + r * 24;
    bool sel = (idx == iLcur);
    fillCard(2, y, SCREEN_W - 4, 22, sel ? SELBG : ((idx & 1) ? CARD2 : CARD1));
    drawRect(2, y, SCREEN_W - 4, 22, sel ? C_INK : CARD_LINE);
    fillRect(3, y + 1, 3, 20, sevColor(m.sev));
    char nm[24] = "";
    if (m.nf > 0) faultName(m.faults[0], nm, sizeof(nm), true);
    if (m.nf > 1) snprintf(b, sizeof(b), "#%03u %s +%d", (unsigned)m.id, nm, m.nf - 1); else snprintf(b, sizeof(b), "#%03u %s", (unsigned)m.id, nm);
    drawTextN(10, y + 3, b, 19, sel ? C_WHITE : C_INK);
    char tm[28]; fmtClockShort(m.epoch, m.trigMs, tm, sizeof(tm));
    drawTextN(10, y + 12, tm, 12, sel ? C_WHITE : C_MUTED);
    snprintf(b, sizeof(b), "%.0fkm/h", m.spd10 / 10.0f);
    drawTextR(SCREEN_W - 6, y + 12, b, sel ? C_WHITE : BLUE);
  }
  if (typing) drawFooter("./- morse L:del R:ok"); else drawFooter("L/R move  OK open");
  drawToast();
}

static void renderIncDetail() {
  const IncHeader& h = viewHdr;
  char b[80], tm[28], a[40], v[24];
  drawBg();
  snprintf(b, sizeof(b), "INCIDENT #%u", (unsigned)h.id);
  drawHeader(b, sevColor(h.sev), true);
  infoCard(3, 17, 122, 20);
  fmtClock(h.epoch, h.trigMs, tm, sizeof(tm));
  drawText(7, 20, tm, 1, C_INK, TRANSP);
  snprintf(b, sizeof(b), "%d problem%s %.0fkm/h", (int)h.nf, h.nf == 1 ? "" : "s", h.trigSpd10 / 10.0f);
  drawTextN(7, 28, b, 19, sevColor(h.sev));

  nRows = 0;
  for (int k = 0; k < h.nf && k < MAXF; k++) {
    int fid = h.faults[2 * k], si = fid / 4;
    int idx = h.fIdx[k] < viewCount ? h.fIdx[k] : viewCount - 1;
    faultName(fid, a, sizeof(a), true);
    if (si < NSENS && idx >= 0 && svOk(viewBuf[idx], si)) fmtVal(si, sv(viewBuf[idx], si), v, sizeof(v), true); else v[0] = 0;
    addRow(a, v, C_INK, sevColor(h.faults[2 * k + 1]), sevColor(h.faults[2 * k + 1]), -1, 0, k);
  }
  addRow("ALL SENSORS", ">", C_INK, BLUE, RGB565(90, 150, 255), -1, 0, 1000);
  addRow("GRAPH", ">", C_INK, BLUE, RGB565(90, 150, 255), -1, 0, 1001);
  drawList();

  // "what happened" card for the selected row
  infoCard(3, 110, 122, 40);
  int tag = (lcur >= 0 && lcur < nRows) ? rows[lcur].tag : -1;
  if (tag >= 0 && tag < MAXF && tag < h.nf) {
    int fid = h.faults[2 * tag], sev = h.faults[2 * tag + 1], si = fid / 4, kind = fid % 4;
    int idx = h.fIdx[tag] < viewCount ? h.fIdx[tag] : viewCount - 1; if (idx < 0) idx = 0;
    const Sample& s = viewBuf[idx];
    faultName(fid, a, sizeof(a), false);
    drawTextN(7, 113, a, 19, sevColor(sev));
    if (si >= NSENS) snprintf(b, sizeof(b), "No data from the ECU");
    else if (kind == K_SENSOR) snprintf(b, sizeof(b), svOk(s, si) ? "Reading looks wrong" : "No signal from sensor");
    else if (kind == K_MISMATCH) snprintf(b, sizeof(b), "Differs from other wheels");
    else {
      char lm[16]; fmtVal(si, svOk(s, si) ? sv(s, si) : 0.0f, v, sizeof(v), false); fmtLim(h.fLim[tag], lm, sizeof(lm));
      snprintf(b, sizeof(b), "Read %s, limit %s %s", v, lm, SENS[si].unit);
    }
    drawTextN(7, 122, b, 19, C_INK);
    snprintf(b, sizeof(b), "%.0frpm %.0fkm/h", svOk(s, SN_RPM) ? sv(s, SN_RPM) : 0.0f, svOk(s, SN_SPEED) ? sv(s, SN_SPEED) : 0.0f);
    drawTextN(7, 131, b, 19, BLUE);
    char cd[20]; faultCode(fid, sev, cd, sizeof(cd));
    snprintf(b, sizeof(b), cd[0] ? "Code %s" : "No standard code", cd);
    drawTextN(7, 140, b, 19, C_MUTED);
  } else if (tag == 1000) {
    drawText(7, 113, "ALL SENSORS", 1, C_INK, TRANSP);
    drawText(7, 123, "every sensor value", 1, C_MUTED, TRANSP);
    drawText(7, 132, "at the fault moment", 1, C_MUTED, TRANSP);
  } else {
    drawText(7, 113, "GRAPH", 1, C_INK, TRANSP);
    drawText(7, 123, "before and after the", 1, C_MUTED, TRANSP);
    drawText(7, 132, "fault. Scrub with L/R", 1, C_MUTED, TRANSP);
  }
  drawFooter("OK: open graph");
  drawToast();
}

static void renderSnap() {
  const IncHeader& m = viewHdr;
  char b[32], v[20];
  drawBg(); snprintf(b, sizeof(b), "SNAPSHOT #%u", (unsigned)m.id); drawHeader(b, RGB565(130, 80, 215), true);
  nRows = 0;
  if (viewCount > 0 && viewTrig >= 0) {
    const Sample& s = viewBuf[viewTrig];
    for (int i = 0; i < NSENS; i++) {
      uint8_t fs = 0;
      for (int k = 0; k < m.nf && k < MAXF; k++) if (m.faults[2 * k] / 4 == i && m.faults[2 * k + 1] > fs) fs = m.faults[2 * k + 1];
      if (!svOk(s, i)) snprintf(b, sizeof(b), "NO SIGNAL"); else { fmtVal(i, sv(s, i), v, sizeof(v), true); snprintf(b, sizeof(b), "%s", v); }
      addRow(SENS[i].name, b, C_INK, fs ? sevColor(fs) : (svOk(s, i) ? C_MUTED : C_GREY), fs ? sevColor(fs) : SYS[SENS[i].sys].color, -1, 0, i);
    }
  }
  drawList();
  drawFooter("values at fault time");
  drawToast();
}

static void renderReplay() {
  char b[40];
  drawBg();
  snprintf(b, sizeof(b), "GRAPH #%u", (unsigned)viewHdr.id);
  drawHeader(b, SYS[SENS[vsig].sys].color, true);
  if (viewCount < 2) { drawText(10, 60, "No data", 1, C_INK, TRANSP); drawToast(); return; }
  const int GY = 16, GH = 72;
  const SensInfo& si = SENS[vsig];
  uint16_t col = lerp565(SYS[si.sys].color, C_BLACK, 70);
  float lo = si.lo, hi = si.hi; if (hi <= lo) hi = lo + 1;
  fillRect(0, GY, SCREEN_W, GH, C_WHITE);                                  // white graph paper
  for (int g = 1; g < 4; g++) fillRect(0, GY + g * GH / 4, SCREEN_W, 1, CARD_LINE);
  for (int k = 0; k < 2; k++) {                                       // limit lines (warn = amber, crit = red)
    float lims[2] = {k == 0 ? WHI(vsig) : CHI(vsig), k == 0 ? WLO(vsig) : CLO(vsig)};
    for (int q = 0; q < 2; q++) {
      float t = lims[q];
      if (t > 1e8f || t < -1e8f) continue;
      int y = GY + GH - 1 - (int)((clampf(t, lo, hi) - lo) / (hi - lo) * (GH - 1));
      for (int x = 0; x < SCREEN_W; x += 4) fillRect(x, y, 2, 1, k == 0 ? COL_WARN : COL_CRIT);
    }
  }
  for (int c = 0; c < SCREEN_W; c++) {
    int i0 = (int)((long)c * viewCount / SCREEN_W), i1 = (int)((long)(c + 1) * viewCount / SCREEN_W);
    if (i1 <= i0) i1 = i0 + 1;
    if (i1 > viewCount) i1 = viewCount;
    if (i0 >= viewCount) i0 = viewCount - 1;
    float mn = 1e9f, mx = -1e9f; bool any = false;
    for (int i = i0; i < i1; i++) { if (!svOk(viewBuf[i], vsig)) continue; float v = sv(viewBuf[i], vsig); any = true; if (v < mn) mn = v; if (v > mx) mx = v; }
    if (!any) { fillRect(c, GY + GH - 3, 1, 2, COL_CRIT); continue; }             // no signal marker
    int y1 = GY + GH - 1 - (int)((clampf(mx, lo, hi) - lo) / (hi - lo) * (GH - 1));
    int y2 = GY + GH - 1 - (int)((clampf(mn, lo, hi) - lo) / (hi - lo) * (GH - 1));
    fillRect(c, y1, 1, (y2 - y1 + 1) < 2 ? 2 : (y2 - y1 + 1), col);
  }
  if (viewTrig >= 0) fillRect((int)((long)viewTrig * SCREEN_W / viewCount), GY, 1, GH, COL_CRIT);
  fillRect(curCol, GY, 1, GH, C_INK);
  drawTextN(3, GY + 2, si.name, 16, col);
  snprintf(b, sizeof(b), "%.*f", (int)si.dec, hi); drawTextR(SCREEN_W - 3, GY + 2, b, C_MUTED);
  snprintf(b, sizeof(b), "%.*f", (int)si.dec, lo); drawTextR(SCREEN_W - 3, GY + GH - 9, b, C_MUTED);

  int ci = (int)((long)curCol * viewCount / SCREEN_W); if (ci >= viewCount) ci = viewCount - 1;
  const Sample& s = viewBuf[ci];
  float dt = viewTrig >= 0 ? (float)((int32_t)(s.t - viewBuf[viewTrig].t)) / 1000.0f : 0.0f;
  snprintf(b, sizeof(b), "FAULT %+.1fs", dt);
  infoCard(3, 92, 122, 56);
  drawText(7, 95, b, 1, C_INK, TRANSP);
  char v[24];
  if (svOk(s, vsig)) fmtVal(vsig, sv(s, vsig), v, sizeof(v), true); else snprintf(v, sizeof(v), "NO SIGNAL");
  uint8_t vs = svOk(s, vsig) ? sevOfValue(vsig, sv(s, vsig), svOk(s, SN_RPM) && sv(s, SN_RPM) > RUNNING_RPM) : 2;
  drawTextR(SCREEN_W - 7, 95, v, vs ? sevColor(vs) : COL_OK);
  snprintf(b, sizeof(b), "SPD %.0f  RPM %.0f", svOk(s, SN_SPEED) ? sv(s, SN_SPEED) : 0.0f, svOk(s, SN_RPM) ? sv(s, SN_RPM) : 0.0f);
  drawText(7, 107, b, 1, BLUE, TRANSP);
  snprintf(b, sizeof(b), "CLT %.0f  OIL %.0f", svOk(s, SN_ECT) ? sv(s, SN_ECT) : 0.0f, svOk(s, SN_OILP) ? sv(s, SN_OILP) : 0.0f);
  drawText(7, 117, b, 1, C_INK, TRANSP);
  snprintf(b, sizeof(b), "BAT %.1f  FUEL %.0f", svOk(s, SN_BATV) ? sv(s, SN_BATV) : 0.0f, svOk(s, SN_FUEL) ? sv(s, SN_FUEL) : 0.0f);
  drawText(7, 127, b, 1, C_INK, TRANSP);
  drawText(7, 137, (s.flags & FLAG_MIL) ? "MIL lamp ON" : "MIL lamp off", 1, (s.flags & FLAG_MIL) ? COL_WARN : C_MUTED, TRANSP);
  drawText(3, 151, "L/R scrub  OK next", 1, C_MUTED, TRANSP);
  drawToast();
}

static void renderStorage() {
  drawBg(); drawHeader("STORAGE / TOOLS", BLUE, true);
  char b[40];
  uint32_t tot = LittleFS.totalBytes(), used = LittleFS.usedBytes();
  infoCard(3, 17, 122, 44);
  snprintf(b, sizeof(b), "FLASH %u / %u KB", (unsigned)(used / 1024), (unsigned)(tot / 1024));
  drawText(7, 21, b, 1, C_INK, TRANSP);
  fillRect(7, 32, 114, 7, CARD_LINE);
  int bw = tot ? (int)((uint64_t)used * 114 / tot) : 0; if (bw > 114) bw = 114;
  fillRect(7, 32, bw, 7, (tot && used * 10 > tot * 8) ? COL_WARN : BLUE);
  uint32_t room = incFileBytes() ? (fsFree() > FS_RESERVE ? (fsFree() - FS_RESERVE) / incFileBytes() : 0) : 0;
  snprintf(b, sizeof(b), "Saved %d, room ~%u", metaN, (unsigned)room);
  drawTextN(7, 43, b, 19, C_MUTED);
  snprintf(b, sizeof(b), "Auto-clean at %d%%", FLASH_HIGH_PCT);
  drawTextN(7, 52, b, 19, C_MUTED);
  nRows = 0;
  snprintf(b, sizeof(b), "%d saved", metaN);
  addRow("ERASE BLACK BOX", b, C_INK, C_MUTED, COL_CRIT, -1, 0, 0);
  addRow("RESET LIMITS", "back to defaults", C_INK, C_MUTED, COL_WARN, -1, 0, 1);
  addRow("TEST TELEGRAM", tgEnabled ? "send a test msg" : "not set up", C_INK, C_MUTED, BLUE, -1, 0, 2);
  addRow("TEST BUZZER", "3 beeps", C_INK, C_MUTED, BLUE, -1, 0, 3);
#if USE_TOUCH
  addRow("TOUCH TEST", "raw numbers", C_INK, C_MUTED, BLUE, -1, 0, 4);
  addRow("CALIBRATE TOUCH", tCalValid ? "saved" : "4 taps", C_INK, C_MUTED, BLUE, -1, 0, 5);
#endif
  drawList();
  if (confirmArm) {
    fillCard(4, 100, 120, 48, COL_CRIT);
    drawTextC(SCREEN_W / 2, 105, confirmArm == 1 ? "ERASE ALL LOGS?" : "RESET ALL LIMITS?", 1, C_WHITE, TRANSP);
    drawTextC(SCREEN_W / 2, 120, "OK again = YES", 1, C_WHITE, TRANSP);
    drawTextC(SCREEN_W / 2, 132, "any other key = no", 1, C_WHITE, TRANSP);
  } else drawFooter("L/R move  OK select");
  drawToast();
}

static void renderStatus() {
  drawBg(); drawHeader("DEVICE STATUS", RGB565(40, 150, 110), true);
  char b[44]; int y = 20;
  infoCard(3, 17, 122, 132);
  snprintf(b, sizeof(b), "ECU: %s", !haveData ? "no data yet" : (linkOk() ? (lastSrc == 1 ? "link OK (USB)" : (lastSrc == 3 ? "link OK (UART)" : "link OK (WiFi)")) : "LINK LOST"));
  drawText(7, y, b, 1, linkOk() ? COL_OK : (haveData ? COL_CRIT : C_MUTED), TRANSP); y += 11;
  snprintf(b, sizeof(b), "Lines %u  bad %u", (unsigned)rxLines, (unsigned)rxBad); drawText(7, y, b, 1, C_INK, TRANSP); y += 11;
  snprintf(b, sizeof(b), "FLASH %u/%u KB", (unsigned)(LittleFS.usedBytes() / 1024), (unsigned)(LittleFS.totalBytes() / 1024));
  drawText(7, y, b, 1, C_INK, TRANSP); y += 11;
  snprintf(b, sizeof(b), "BLACK BOX %d / %d", metaN, MAX_INC); drawText(7, y, b, 1, C_INK, TRANSP); y += 11;
  drawText(7, y, epochBase ? "TIME: synced" : "TIME: since boot", 1, epochBase ? COL_OK : COL_WARN, TRANSP); y += 11;
  if (WiFi.status() == WL_CONNECTED) { snprintf(b, sizeof(b), "WiFi %s", WiFi.localIP().toString().c_str()); drawTextN(7, y, b, 19, COL_OK); }
  else drawText(7, y, staEnabled ? "WiFi: not connected" : "WiFi: hotspot only", 1, staEnabled ? COL_WARN : C_MUTED, TRANSP);
  y += 11;
  snprintf(b, sizeof(b), "HOTSPOT %s", WiFi.softAPIP().toString().c_str()); drawTextN(7, y, b, 19, C_INK); y += 11;
  if (!tgEnabled) drawText(7, y, "TELEGRAM: off", 1, C_MUTED, TRANSP);
  else if (tgState == 2) drawText(7, y, "TELEGRAM: online", 1, COL_OK, TRANSP);
  else if (tgState == 3) { snprintf(b, sizeof(b), "TELEGRAM: error %d", (int)tgCode); drawText(7, y, b, 1, COL_CRIT, TRANSP); }
  else drawText(7, y, "TELEGRAM: no WiFi", 1, COL_WARN, TRANSP);
  y += 11;
  if (tgEnabled) { snprintf(b, sizeof(b), "sent %u  dropped %u", (unsigned)tgSent, (unsigned)tgDropped); drawText(7, y, b, 1, C_MUTED, TRANSP); }
  y += 11;
  snprintf(b, sizeof(b), "%s  rec %ds+%ds", psramFound() ? "PSRAM" : "no PSRAM", g_preN / SAMPLE_HZ, g_postN / SAMPLE_HZ);
  drawTextN(7, y, b, 19, C_MUTED); y += 11;
  snprintf(b, sizeof(b), "free RAM %u KB", (unsigned)(ESP.getFreeHeap() / 1024)); drawText(7, y, b, 1, C_MUTED, TRANSP);
  drawToast();
}

static void startCalibration() {
  calStep = 0; calLatched = false;
  if (screen != S_CAL) navForward(S_CAL);
}
static void renderTouchTest() {
  drawBg(); drawHeader("TOUCH TEST", BLUE, true);
  char b[44];
  infoCard(3, 17, 122, 82);
  bool irqMode = TOUCH_USE_IRQ;
  snprintf(b, sizeof(b), "Mode: %s", irqMode ? "T_IRQ pin" : "pressure"); drawText(7, 21, b, 1, C_MUTED, TRANSP);
  snprintf(b, sizeof(b), "T_IRQ pin: %s", tIrq ? "LOW (pressed)" : "HIGH"); drawText(7, 31, b, 1, C_INK, TRANSP);
  snprintf(b, sizeof(b), "Pressure Z: %d", tRawZ); drawText(7, 41, b, 1, C_INK, TRANSP);
  snprintf(b, sizeof(b), "Raw X %d  Y %d", tRawX, tRawY); drawText(7, 51, b, 1, C_INK, TRANSP);
  snprintf(b, sizeof(b), "Screen x %d  y %d", tMapX, tMapY); drawText(7, 61, b, 1, BLUE, TRANSP);
  uint16_t chip = !tChipOk ? COL_CRIT : (tPress ? COL_OK : C_GREY);
  const char* msg = !tChipOk ? "NO TOUCH CHIP" : (tPress ? "TOUCH DETECTED" : "touch the screen");
  fillCard(7, 73, 114, 14, chip); drawTextC(SCREEN_W / 2, 76, msg, 1, textOn(chip), TRANSP);
  if (!tChipOk) {
    drawText(5, 104, "Check these wires:", 1, C_INK, TRANSP);
    drawText(5, 114, "T_CLK T_DIN T_DO", 1, C_INK, TRANSP);
    drawText(5, 124, "T_CS 3V3 GND", 1, C_INK, TRANSP);
    drawText(5, 134, "(T_IRQ is optional)", 1, C_MUTED, TRANSP);
  } else {
    drawText(5, 106, "Red dot = where the", 1, C_MUTED, TRANSP);
    drawText(5, 116, "screen sees your", 1, C_MUTED, TRANSP);
    drawText(5, 126, "finger. Wrong place?", 1, C_MUTED, TRANSP);
    drawText(5, 136, "-> calibrate (OK)", 1, C_MUTED, TRANSP);
  }
  drawFooter("OK:calib  hold:back");
  if (tPress && tMapX >= 0) { fillRect(tMapX - 3, tMapY - 3, 7, 7, COL_CRIT); drawRect(tMapX - 4, tMapY - 4, 9, 9, C_WHITE); }
}
static void renderCal() {
  drawBg();
  char b[32];
  drawTextC(SCREEN_W / 2, 52, "TOUCH CALIBRATION", 1, C_INK, TRANSP);
  snprintf(b, sizeof(b), "Step %d / 4", calStep + 1 > 4 ? 4 : calStep + 1); drawTextC(SCREEN_W / 2, 66, b, 1, BLUE, TRANSP);
  drawTextC(SCREEN_W / 2, 82, "Touch the red +", 1, C_INK, TRANSP);
  drawTextC(SCREEN_W / 2, 94, "mark with a pen tip", 1, C_MUTED, TRANSP);
  drawTextC(SCREEN_W / 2, 124, "hold OK = cancel", 1, C_MUTED, TRANSP);
  if (calStep < 4) {
    int cx = CAL_PX[calStep], cy = CAL_PY[calStep];
    fillRect(cx - 9, cy, 19, 1, COL_CRIT); fillRect(cx, cy - 9, 1, 19, COL_CRIT);
    drawRect(cx - 3, cy - 3, 7, 7, COL_CRIT);
  }
  drawToast();
}

static void renderFlog() {
  drawBg(); drawHeader("FAULT LOG", RGB565(200, 60, 70), true);
  nRows = 0; char a[40], b[24], tm[24];
  for (int i = flogN - 1; i >= 0 && nRows < MAXROWS; i--) {
    const FEvt& e = flog[i];
    faultName(e.fid, a, sizeof(a), false);
    fmtClockTiny(e.epoch, e.ms, tm, sizeof(tm));
    snprintf(b, sizeof(b), "%s %s", tm, e.kind ? "CLEARED" : (e.sev >= 2 ? "CRIT" : "WARN"));
    addRow(a, b, e.kind ? C_MUTED : C_INK, C_MUTED, e.kind ? COL_OK : sevColor(e.sev), -1, 0, i);
  }
  if (nRows == 0) addRow("No faults logged", "", COL_OK, C_MUTED, COL_OK, -1, 0, -1);
  drawList();
  drawFooter("OK: details");
  drawToast();
}

static void renderFevt() {                               // one logged event: what, how much, at which speed, when
  if (fevtSel >= flogN) fevtSel = flogN - 1;
  if (fevtSel < 0) { drawBg(); drawHeader("FAULT LOG", BLUE, true); drawToast(); return; }
  const FEvt& e = flog[fevtSel];
  uint16_t col = e.kind ? COL_OK : sevColor(e.sev);
  drawBg(); drawHeader(e.kind ? "PROBLEM CLEARED" : (e.sev >= 2 ? "CRITICAL" : "WARNING"), col, true);
  infoCard(3, 17, 122, 132);
  char a[48], b[28], c[24], tm[28]; int y = 21; int si = e.fid / 4, kind = e.fid % 4;
  faultName(e.fid, a, sizeof(a), false);
  drawWrapped(7, y, a, 20, 2, C_INK); y += 24;
  if (si < NSENS) {
    snprintf(b, sizeof(b), "%s / %s", SYS[SENS[si].sys].name, SUBS[SENS[si].sub].name);
    drawTextN(7, y, b, 19, C_MUTED); y += 11;
    if (e.hasVal) fmtVal(si, e.val, c, sizeof(c), true); else snprintf(c, sizeof(c), "no signal");
    snprintf(b, sizeof(b), "Value %s", c); drawTextN(7, y, b, 19, C_INK); y += 11;
    if (!e.kind && (kind == K_HIGH || kind == K_LOW) && !(SENS[si].flags & (SF_BOOL | SF_FUSE | SF_RELAY))) {
      fmtLim(e.lim, c, sizeof(c)); snprintf(b, sizeof(b), "Limit %s%s", kind == K_HIGH ? ">" : "<", c); drawTextN(7, y, b, 19, C_INK); y += 11;
    } else if (!e.kind && kind == K_MISMATCH) { drawTextN(7, y, "Differs from partner", 19, C_INK); y += 11; }
    else if (!e.kind && kind == K_SENSOR) { drawTextN(7, y, "Sensor / wiring fault", 19, C_INK); y += 11; }
  } else { drawTextN(7, y, "No data from the ECU", 19, C_INK); y += 11; }
  snprintf(b, sizeof(b), "Speed %.0f km/h", e.spd10 / 10.0f); drawTextN(7, y, b, 19, C_INK); y += 11;
  snprintf(b, sizeof(b), "Engine %u rpm", (unsigned)e.rpm); drawTextN(7, y, b, 19, C_INK); y += 11;
  fmtClock(e.epoch, e.ms, tm, sizeof(tm)); drawTextN(7, y, tm, 19, C_MUTED); y += 11;
  faultCode(e.fid, e.sev, c, sizeof(c)); if (c[0]) { drawTextN(7, y, c, 19, C_MUTED); y += 11; }
  if (!e.kind) { if (e.inc) snprintf(b, sizeof(b), "Black box #%u", (unsigned)e.inc); else snprintf(b, sizeof(b), "Black box: none"); drawTextN(7, y, b, 19, C_MUTED); }
  snprintf(b, sizeof(b), "%d / %d", flogN - fevtSel, flogN); drawTextR(SCREEN_W - 7, 140, b, C_MUTED);
  drawText(7, 140, "L newer  R older", 1, C_MUTED, TRANSP);
  drawToast();
}

// ============================ APPS: Nokia-style home, system pages, scan, white report ============================
enum { AK_SCAN, AK_REPORT, AK_SYS, AK_BBOX, AK_TOOLS, AK_STATUS };
#define NAPPS (NSYS + 5)
#define GRID_COLS 4
#define GRID_ROWS 3
static const char* const SYS_SHORT[NSYS] = {"ENGIN", "AIR", "FUEL", "INJ", "COOL", "EXH", "TRANS", "BRAKE", "STEER", "TYRES", "ELEC", "CAB", "FUSES"};
static uint32_t scanT0 = 0;
static int appSens[24]; static int appN = 0;

// app list: SCAN, REPORT, the 13 systems, BLACK BOX, TOOLS, STATUS
static void appDesc(int idx, int& kind, int& arg, int& icon, uint16_t& col, const char*& label, const char*& name) {
  arg = 0;
  if (idx == 0)      { kind = AK_SCAN;   icon = IC_SCAN;   col = RGB565(16, 158, 92);  label = "SCAN"; name = "FULL SCAN"; }
  else if (idx == 1) { kind = AK_REPORT; icon = IC_DOC;    col = RGB565(22, 110, 235); label = "REPRT"; name = "SCAN REPORT"; }
  else if (idx < 2 + NSYS) { int s = idx - 2; kind = AK_SYS; arg = s; icon = SYS[s].icon; col = SYS[s].color; label = SYS_SHORT[s]; name = SYS[s].name; }
  else {
    int j = idx - 2 - NSYS;
    if (j == 0)      { kind = AK_BBOX;   icon = IC_BOX;    col = RGB565(130, 80, 215); label = "BOX";   name = "BLACK BOX"; }
    else if (j == 1) { kind = AK_TOOLS;  icon = IC_STATUS; col = RGB565(70, 100, 150); label = "TOOLS"; name = "STORAGE / TOOLS"; }
    else             { kind = AK_STATUS; icon = IC_OK;     col = RGB565(40, 150, 110); label = "STAT";  name = "DEVICE STATUS"; }
  }
}
static void gridClamp(int total) {
  if (lcur >= total) lcur = total - 1;
  if (lcur < 0) lcur = 0;
  int row = lcur / GRID_COLS, rows = (total + GRID_COLS - 1) / GRID_COLS;
  if (ltop > rows - GRID_ROWS) ltop = rows - GRID_ROWS;
  if (ltop < 0) ltop = 0;
  if (row < ltop) ltop = row;
  if (row >= ltop + GRID_ROWS) ltop = row - GRID_ROWS + 1;
}
// one app tile: coloured icon box, short text under it, highlight when selected, optional count badge
static void drawTile(int idx, int icon, uint16_t bg, const char* label, uint16_t labelCol, int badge, uint16_t badgeCol) {
  int row = idx / GRID_COLS, col = idx % GRID_COLS;
  if (row < ltop || row >= ltop + GRID_ROWS) return;
  int x = 1 + col * 32, y = 18 + (row - ltop) * 36;
  bool sel = (idx == lcur);
  if (sel) fillCard(x - 1, y - 1, 32, 37, SELBG);
  fillCard(x + 1, y + 1, 28, 26, bg);
  drawIcon(x + 7, y + 6, icon, textOn(bg), 1);
  if (badge > 0) {
    char b[6]; snprintf(b, sizeof(b), "%d", badge > 99 ? 99 : badge);
    int w = (int)strlen(b) * 6 + 4;
    fillCard(x + 29 - w, y - 1, w, 10, badgeCol);
    drawText(x + 31 - w, y + 1, b, 1, textOn(badgeCol), TRANSP);
  }
  drawText(x + 15 - textW(label, 1) / 2, y + 27, label, 1, sel ? C_WHITE : labelCol, TRANSP);
}
static void drawCaption(int total, const char* l1, uint16_t c1, const char* l2, uint16_t c2, const char* l3, uint16_t c3) {
  infoCard(2, 127, 124, 32);
  drawTextN(6, 130, l1, 17, c1); drawTextN(6, 140, l2, 19, c2); drawTextN(6, 150, l3, 19, c3);
  int rows = (total + GRID_COLS - 1) / GRID_COLS;
  if (ltop > 0) for (int r = 0; r < 3; r++) fillRect(118 - r, 130 + r, 1 + 2 * r, 1, BLUE);                  // more above
  if (ltop + GRID_ROWS < rows) for (int r = 0; r < 3; r++) fillRect(116 + r, 152 + r, 5 - 2 * r, 1, BLUE);   // more below
}
static int sysTopFault(int sys) {
  int best = -1; uint8_t bs = 0;
  for (int f = 0; f < NSENS * 4; f++) if (fSev[f] && SENS[f / 4].sys == sys && fSev[f] > bs) { bs = fSev[f]; best = f; }
  return best;
}
static int flogLastStart(int si) {                       // newest logged "problem started" event of one sensor
  for (int i = flogN - 1; i >= 0; i--) if (flog[i].kind == 0 && flog[i].fid / 4 == si) return i;
  return -1;
}
static void fmtShort(int i, float v, char* out, size_t n) {   // value for the tile label, at most 5 characters
  const SensInfo& s = SENS[i];
  if (s.flags & SF_FUSE)  { snprintf(out, n, "%s", v > 0.5f ? "OK" : "BLOWN"); return; }
  if (s.flags & SF_RELAY) { snprintf(out, n, "%s", v > 0.5f ? "OK" : "FAIL"); return; }
  if (s.flags & SF_BOOL)  { snprintf(out, n, "%s", v > 0.5f ? "YES" : "NO"); return; }
  snprintf(out, n, "%.*f", (int)s.dec, v);
  if (strlen(out) > 5) snprintf(out, n, "%.0f", v);
}
static int sensorIcon(int i) {                           // small picture that tells what kind of sensor it is
  const SensInfo& s = SENS[i];
  if (s.flags & (SF_FUSE | SF_RELAY)) return IC_FUSE;
  if (s.flags & SF_BOOL) return IC_WARN;
  if (!strcmp(s.unit, "C")) return IC_THERMO;
  if (!strcmp(s.unit, "V") || !strcmp(s.unit, "A") || !strcmp(s.unit, "mA")) return IC_BOLT;
  if (s.sub == SB_INJCORR) return IC_INJ;
  if (s.sub == SB_CYLBAL) return IC_ENGINE;
  if (s.sub == SB_WHEELSPD || s.sub == SB_TYREP || s.sub == SB_TYRET) return IC_TYRE;
  if (!strcmp(s.unit, "%") || !strcmp(s.unit, "mm")) return IC_LEVEL;
  if (!strcmp(s.unit, "kPa") || !strcmp(s.unit, "bar") || !strcmp(s.unit, "rpm") || !strcmp(s.unit, "krpm") ||
      !strcmp(s.unit, "km/h") || !strcmp(s.unit, "m/s2")) return IC_DIAL;
  if (!strcmp(s.unit, "deg") || !strcmp(s.unit, "deg/s") || !strcmp(s.unit, "Nm")) return IC_STEER;
  if (!strcmp(s.unit, "mm3") || !strcmp(s.unit, "mg") || !strcmp(s.unit, "L/h") || !strcmp(s.unit, "kg/h") ||
      !strcmp(s.unit, "mL/min") || !strcmp(s.unit, "ppm")) return IC_FUEL;
  if (s.sub == SB_GEARBOX || s.sub == SB_DRIVELINE) return IC_TRANS;
  return SYS[s.sys].icon;
}

// ---- report data: every problem that was logged, with the speed at that moment ----
#define REP_MAX 100
static Rep rep[REP_MAX]; static int repN = 0, repPage = 0, repScope = -1;     // state: 0 = still active, 1 = cleared, 2 = ended (no clear logged)
static int16_t repMap[NFAULT];
static void buildReport(int scope) {                     // scope = -1 : whole vehicle, else one system
  repScope = scope; repN = 0;
  for (int f = 0; f < NFAULT; f++) repMap[f] = -1;
  for (int i = 0; i < flogN; i++) {
    const FEvt& e = flog[i]; int fid = e.fid;
    if (fid >= NFAULT) continue;
    int si = fid / 4;
    if (scope >= 0 && !(si < NSENS && SENS[si].sys == scope)) continue;
    if (e.kind == 0) {
      if (repMap[fid] >= 0) { Rep& r = rep[repMap[fid]]; if (e.sev > r.sev) r.sev = e.sev; continue; }
      if (repN >= REP_MAX) {                              // full: forget the oldest one
        memmove(rep, rep + 1, sizeof(Rep) * (REP_MAX - 1)); repN = REP_MAX - 1;
        for (int f = 0; f < NFAULT; f++) if (repMap[f] >= 0) repMap[f]--;
      }
      Rep& r = rep[repN]; memset(&r, 0, sizeof(r));
      r.fid = (uint16_t)fid; r.sev = e.sev; r.state = 0; r.epoch = e.epoch; r.ms = e.ms; r.val = e.val; r.lim = e.lim;
      r.hasVal = e.hasVal; r.spd10 = e.spd10; r.rpm = e.rpm;
      repMap[fid] = (int16_t)repN++;
    } else if (repMap[fid] >= 0) {
      Rep& r = rep[repMap[fid]]; r.state = 1;
      if (e.epoch && r.epoch && e.epoch >= r.epoch) r.durS = e.epoch - r.epoch; else r.durS = (e.ms - r.ms) / 1000;
      repMap[fid] = -1;
    }
  }
  for (int f = 0; f < NFAULT; f++) if (repMap[f] >= 0) rep[repMap[f]].state = fSev[f] ? 0 : 2;
}
static int repActive() { int c = 0; for (int i = 0; i < repN; i++) if (rep[i].state == 0) c++; return c; }

static void renderHome() {
  drawBg(); drawHeader("SCAN TOOL", BLUE, false);
  gridClamp(NAPPS);
  int act = activeCount();
  for (int idx = 0; idx < NAPPS; idx++) {
    int kind, arg, icon; uint16_t col; const char* label; const char* name;
    appDesc(idx, kind, arg, icon, col, label, name);
    int badge = 0; uint16_t bcol = COL_CRIT;
    if (kind == AK_SCAN) { badge = act; bcol = sevColor(overallSev()); }
    else if (kind == AK_SYS) { int c; uint8_t s = systemSev(arg, &c); badge = c; bcol = sevColor(s); }
    drawTile(idx, icon, col, label, C_INK, badge, bcol);
  }
  int kind, arg, icon; uint16_t col; const char* label; const char* name;
  appDesc(lcur, kind, arg, icon, col, label, name);
  char l2[28] = "", l3[28] = ""; uint16_t c2 = C_MUTED, c3 = C_MUTED;
  switch (kind) {
    case AK_SCAN:
      if (act) { snprintf(l2, sizeof(l2), "%d problem%s now", act, act > 1 ? "s" : ""); c2 = sevColor(overallSev()); }
      else { snprintf(l2, sizeof(l2), haveData ? "all systems OK" : "waiting for ECU"); c2 = haveData ? COL_OK : C_MUTED; }
      snprintf(l3, sizeof(l3), "OK = start scan"); c3 = BLUE; break;
    case AK_REPORT: snprintf(l2, sizeof(l2), "%d events logged", flogN); snprintf(l3, sizeof(l3), "OK = open report"); c3 = BLUE; break;
    case AK_SYS: {
      int c; uint8_t s = systemSev(arg, &c); int ns = 0;
      for (int i = 0; i < NSENS; i++) if (SENS[i].sys == arg) ns++;
      if (c) { snprintf(l2, sizeof(l2), "%d PROBLEM%s", c, c > 1 ? "S" : ""); c2 = sevColor(s); } else { snprintf(l2, sizeof(l2), "OK  %d sensors", ns); c2 = COL_OK; }
      int tf = sysTopFault(arg);
      if (tf >= 0) { faultName(tf, l3, sizeof(l3), false); c3 = sevColor(fSev[tf]); } else { snprintf(l3, sizeof(l3), "OK = open"); c3 = BLUE; }
      break; }
    case AK_BBOX:   snprintf(l2, sizeof(l2), "%d saved incidents", metaN); snprintf(l3, sizeof(l3), "OK = open"); c3 = BLUE; break;
    case AK_TOOLS:  snprintf(l2, sizeof(l2), "erase, reset, test"); snprintf(l3, sizeof(l3), "OK = open"); c3 = BLUE; break;
    default:
      snprintf(l2, sizeof(l2), "%s", !haveData ? "no ECU data" : (linkOk() ? "ECU link OK" : "ECU LINK LOST"));
      c2 = !haveData ? C_MUTED : (linkOk() ? COL_OK : COL_CRIT);
      snprintf(l3, sizeof(l3), "%s", WiFi.status() == WL_CONNECTED ? "WiFi online" : "hotspot only"); break;
  }
  drawCaption(NAPPS, name, C_INK, l2, c2, l3, c3);
  drawToast();
}

static void buildApp() { appN = 0; for (int i = 0; i < NSENS && appN < 24; i++) if (SENS[i].sys == curSys) appSens[appN++] = i; }
static void renderApp() {
  buildApp();
  drawBg(); drawHeader(SYS[curSys].name, SYS[curSys].color, true);
  gridClamp(appN + 1);
  bool run = engineRunning(); char v[12];
  for (int k = 0; k < appN; k++) {
    int i = appSens[k];
    bool ok = haveData && latest.v[i] != INV;
    uint8_t as = sensorSev(i), vs = ok ? sevOfValue(i, sv(latest, i), run) : 0, s = as > vs ? as : vs;
    uint16_t bg = !ok ? C_GREY : (s >= 2 ? COL_CRIT : (s == 1 ? COL_WARN : SYS[curSys].color));
    if (!haveData) snprintf(v, sizeof(v), "--"); else if (!ok) snprintf(v, sizeof(v), "NOSIG"); else fmtShort(i, sv(latest, i), v, sizeof(v));
    drawTile(k, sensorIcon(i), bg, v, (!ok && haveData) ? COL_CRIT : (s ? sevColor(s) : C_INK), 0, 0);
  }
  drawTile(appN, IC_DOC, RGB565(22, 110, 235), "REPRT", C_INK, 0, 0);
  char l1[32], l2[32], l3[32]; uint16_t c2 = C_MUTED, c3 = C_MUTED;
  if (lcur < appN) {
    int i = appSens[lcur]; const SensInfo& si = SENS[i];
    bool ok = haveData && latest.v[i] != INV;
    uint8_t as = sensorSev(i), vs = ok ? sevOfValue(i, sv(latest, i), run) : 0, s = as > vs ? as : vs;
    snprintf(l1, sizeof(l1), "%s", si.name);
    if (!haveData) { snprintf(l2, sizeof(l2), "--"); }
    else if (!ok) { snprintf(l2, sizeof(l2), "NO SIGNAL"); c2 = COL_CRIT; }
    else { char vb[20]; fmtVal(i, sv(latest, i), vb, sizeof(vb), true); snprintf(l2, sizeof(l2), "%s  %s", vb, s ? sevWord(s) : "OK"); c2 = s ? sevColor(s) : COL_OK; }
    int e = flogLastStart(i);
    if (e >= 0) { char tm[24]; fmtClockTiny(flog[e].epoch, flog[e].ms, tm, sizeof(tm)); snprintf(l3, sizeof(l3), "%s %.0fkm/h", tm, flog[e].spd10 / 10.0f); c3 = s ? sevColor(s) : C_MUTED; }
    else snprintf(l3, sizeof(l3), "%s", SUBS[si.sub].name);
  } else {
    buildReport(curSys);
    snprintf(l1, sizeof(l1), "REPORT");
    snprintf(l2, sizeof(l2), "%d problem%s logged", repN, repN == 1 ? "" : "s"); c2 = repN ? COL_WARN : COL_OK;
    snprintf(l3, sizeof(l3), "OK = open"); c3 = BLUE;
  }
  drawCaption(appN + 1, l1, C_INK, l2, c2, l3, c3);
  drawToast();
}

static void renderScanRun() {
  uint32_t el = millis() - scanT0; int step = (int)(el / 280);
  drawBg(); drawHeader("FULL SCAN", BLUE, true);
  int done = step > NSYS ? NSYS : step;
  fillRect(4, 19, 120, 7, CARD_LINE); fillRect(4, 19, 120 * done / NSYS, 7, COL_OK); drawRect(4, 19, 120, 7, C_INK);
  for (int s = 0; s < NSYS; s++) {
    int y = 30 + s * 9; char b[16]; int c; uint8_t sv_ = systemSev(s, &c);
    fillRect(4, y, 7, 7, SYS[s].color);
    drawTextN(15, y, SYS[s].name, 13, s <= step ? C_INK : C_MUTED);
    if (s < step) { if (c) { snprintf(b, sizeof(b), "%d PROB", c); drawTextR(124, y, b, sevColor(sv_)); } else drawTextR(124, y, "OK", COL_OK); }
    else if (s == step) drawTextR(124, y, "scan", BLUE);
    else drawTextR(124, y, "-", C_MUTED);
  }
  drawText(4, 149, "reading all sensors...", 1, C_MUTED, TRANSP);
  drawToast();
}

static void repValueLine(const Rep& r, char* out, size_t n) {
  int si = r.fid / 4, kind = r.fid % 4; char v[20], l[16];
  if (si >= NSENS) { snprintf(out, n, "No data from the ECU"); return; }
  const SensInfo& s = SENS[si];
  if (kind == K_SENSOR) { snprintf(out, n, "Sensor / wiring fault"); return; }
  if (kind == K_MISMATCH) { snprintf(out, n, "Wheel speed differs"); return; }
  if (s.flags & SF_FUSE) { snprintf(out, n, "Fuse blown, no power"); return; }
  if (s.flags & SF_RELAY) { snprintf(out, n, "Relay failed"); return; }
  if (s.flags & SF_BOOL) { snprintf(out, n, "Detected"); return; }
  if (r.hasVal) fmtVal(si, r.val, v, sizeof(v), true); else snprintf(v, sizeof(v), "no signal");
  fmtLim(r.lim, l, sizeof(l));
  snprintf(out, n, "%s %s lim%s%s", kind == K_HIGH ? "HIGH" : "LOW", v, kind == K_HIGH ? ">" : "<", l);
}
static void drawRepEntry(int y, const Rep& r, int number) {
  char nm[40], line[48], code[24], tm[28];
  faultName(r.fid, nm, sizeof(nm), false);
  snprintf(line, sizeof(line), "%d. %s", number, nm); drawTextN(3, y, line, 15, C_INK);
  uint16_t bc = r.state == 0 ? sevColor(r.sev) : (r.state == 1 ? COL_OK : C_GREY);
  const char* bl = r.state == 0 ? (r.sev >= 2 ? "CRIT" : "WARN") : (r.state == 1 ? "FIXED" : "ENDED");
  fillRect(96, y - 1, 30, 9, bc); drawText(96 + (30 - textW(bl, 1)) / 2, y, bl, 1, textOn(bc), TRANSP);
  repValueLine(r, line, sizeof(line)); drawTextN(3, y + 10, line, 21, C_INK);
  int si = r.fid / 4; faultCode(r.fid, r.sev, code, sizeof(code));
  snprintf(line, sizeof(line), "%s  %s", code[0] ? code : "no code", si < NSENS ? SYS_SHORT[SENS[si].sys] : "ECU"); drawTextN(3, y + 19, line, 21, C_MUTED);
  fmtClock(r.epoch, r.ms, tm, sizeof(tm)); drawTextN(3, y + 28, tm, 21, C_INK);
  snprintf(line, sizeof(line), "Speed %.0fkm/h %urpm", r.spd10 / 10.0f, (unsigned)r.rpm); drawTextN(3, y + 37, line, 21, BLUE);
  if (r.state == 0) drawText(3, y + 46, "STILL ACTIVE NOW", 1, COL_CRIT, TRANSP);
  else if (r.state == 1) {
    if (r.durS < 60) snprintf(line, sizeof(line), "Cleared after %us", (unsigned)r.durS);
    else snprintf(line, sizeof(line), "Cleared after %um %us", (unsigned)(r.durS / 60), (unsigned)(r.durS % 60));
    drawTextN(3, y + 46, line, 21, COL_OK);
  } else drawText(3, y + 46, "Ended, no clear log", 1, C_MUTED, TRANSP);
  fillRect(3, y + 57, 120, 1, CARD_LINE);
}
static void renderReport() {
  char b[48], tm[28];
  fillRect(0, 0, SCREEN_W, SCREEN_H, C_WHITE);                 // plain white page
  drawHeader(repScope < 0 ? "SCAN REPORT" : SYS[repScope].name, repScope < 0 ? BLUE : SYS[repScope].color, true);
  int pages = 1 + (repN + 1) / 2;
  if (repPage >= pages) repPage = pages - 1;
  if (repPage < 0) repPage = 0;
  if (repPage == 0) {
    int act = repScope < 0 ? activeCount() : repActive();
    int nsens = 0; for (int i = 0; i < NSENS; i++) if (repScope < 0 || SENS[i].sys == repScope) nsens++;
    drawText(4, 19, "VEHICLE REPORT", 1, C_INK, TRANSP);
    nowClock(tm, sizeof(tm)); drawText(4, 29, tm, 1, C_MUTED, TRANSP);
    fillRect(4, 39, 120, 1, C_INK);
    snprintf(b, sizeof(b), "Sensors checked  %d", nsens); drawText(4, 44, b, 1, C_INK, TRANSP);
    snprintf(b, sizeof(b), "Problems active  %d", act); drawText(4, 54, b, 1, act ? COL_CRIT : COL_OK, TRANSP);
    snprintf(b, sizeof(b), "Problems logged  %d", repN); drawText(4, 64, b, 1, C_INK, TRANSP);
    snprintf(b, sizeof(b), "Speed now   %.0f km/h", curSpeed()); drawText(4, 74, b, 1, BLUE, TRANSP);
    snprintf(b, sizeof(b), "Engine now  %.0f rpm", (haveData && latest.v[SN_RPM] != INV) ? sv(latest, SN_RPM) : 0.0f); drawText(4, 84, b, 1, BLUE, TRANSP);
    fillRect(4, 95, 120, 1, CARD_LINE);
    uint8_t os = 0; for (int f = 0; f < NFAULT; f++) if (fSev[f] > os && (repScope < 0 || (f / 4 < NSENS && SENS[f / 4].sys == repScope))) os = fSev[f];
    const char* res = os >= 2 ? "CRITICAL" : (os == 1 ? "ATTENTION" : (repN ? "OK NOW" : "ALL NORMAL"));
    uint16_t rc = os ? sevColor(os) : COL_OK;
    drawText(4, 100, "RESULT", 1, C_MUTED, TRANSP);
    fillRect(52, 98, 72, 12, rc); drawTextC(88, 101, res, 1, textOn(rc), TRANSP);
    if (repScope < 0) {                                         // which systems have problems right now
      int n = 0; char list[64] = "";
      for (int s = 0; s < NSYS; s++) { int c; systemSev(s, &c); if (c) { n += snprintf(list + n, sizeof(list) - n, "%s %d  ", SYS_SHORT[s], c); if (n > 50) break; } }
      drawText(4, 116, n ? "Problems in:" : (repN ? "Past problems fixed." : "Nothing wrong found."), 1, n ? C_INK : COL_OK, TRANSP);
      if (n) drawWrapped(4, 126, list, 20, 2, COL_CRIT);
    }
  } else {
    for (int k = 0; k < 2; k++) {
      int e = repN - 1 - ((repPage - 1) * 2 + k);
      if (e < 0) break;
      drawRepEntry(19 + k * 62, rep[e], repN - e);
    }
  }
  snprintf(b, sizeof(b), "%d/%d  L/R%s", repPage + 1, pages, repPage ? "  OK:sensor" : " next");
  drawText(3, 151, b, 1, C_MUTED, TRANSP);
  drawToast();
}

static void render() {
  switch (screen) {
    case S_FLOG:       renderFlog(); break;
    case S_HOME:       renderHome(); break;
    case S_APP:        renderApp(); break;
    case S_SCANRUN:    renderScanRun(); break;
    case S_REPORT:     renderReport(); break;
    case S_FEVT:       renderFevt(); break;
    case S_DRIVER:     renderDriver(); break;
    case S_MENU:       renderMenu(); break;
    case S_SCAN:       renderScan(); break;
    case S_SYSTEMS:    renderSystems(); break;
    case S_SUBS:       renderSubs(); break;
    case S_SENSORS:    renderSensors(); break;
    case S_SENSOR:     renderSensor(); break;
    case S_HIST:       renderHist(); break;
    case S_CODES:      renderCodes(); break;
    case S_INC_LIST:   renderIncList(); break;
    case S_INC_DETAIL: renderIncDetail(); break;
    case S_SNAP:       renderSnap(); break;
    case S_REPLAY:     renderReplay(); break;
    case S_STORAGE:    renderStorage(); break;
    case S_STATUS:     renderStatus(); break;
    case S_TOUCH:      renderTouchTest(); break;
    case S_CAL:        renderCal(); break;
  }
  flushScreen();
}

// ============================ 18. UI ACTIONS ==============================
static void openIncident(int mi, int sensorForGraph) {
  if (!loadIncidentToView(mi)) { toast("Cannot read file"); return; }
  detMeta = mi;
  vsig = (sensorForGraph >= 0 && sensorForGraph < NSENS) ? sensorForGraph : 0;
  if (sensorForGraph < 0 && meta[mi].nf > 0 && meta[mi].faults[0] / 4 < NSENS) vsig = meta[mi].faults[0] / 4;
  navForward(S_INC_DETAIL);
}
static void openIncidentList() { iLcur = -1; iLtop = 0; mlen = 0; mbuf[0] = 0; rebuildFilter(); navForward(S_INC_LIST); }
static void openSensorOfFault(int fid) {
  int si = fid / 4;
  if (si >= NSENS) { toast("Check ECU cable/power"); return; }
  curSens = si; curSub = SENS[si].sub; curSys = SENS[si].sys;
  navForward(S_SENSOR);
}
static void sendTestTelegram() {
  if (!tgEnabled) { toast("Telegram not set up"); return; }
  char tm[28], msg[200]; nowClock(tm, sizeof(tm));
  snprintf(msg, sizeof(msg), "🧪 TEST from Truck Black Box\n✅ Telegram works\n🕒 %s\n💾 %d incidents stored", tm, metaN);
  tgEnqueue(msg); toast("Test message queued");
}

static void scanFinish() { buildReport(-1); repPage = 0; screen = S_REPORT; lcur = 0; ltop = 0; dirty = true; beep(70, 2800); }
static void startScan() { scanT0 = millis(); navForward(S_SCANRUN); }
static void homeOk() {
  int kind, arg, icon; uint16_t col; const char* label; const char* name;
  appDesc(lcur, kind, arg, icon, col, label, name);
  switch (kind) {
    case AK_SCAN:   startScan(); break;
    case AK_REPORT: buildReport(-1); repPage = 0; navForward(S_REPORT); break;
    case AK_SYS:    curSys = arg; navForward(S_APP); break;
    case AK_BBOX:   openIncidentList(); break;
    case AK_TOOLS:  navForward(S_STORAGE); break;
    default:        navForward(S_STATUS); break;
  }
}
static void appOk() {
  buildApp();
  if (lcur < appN) { curSens = appSens[lcur]; curSub = SENS[curSens].sub; navForward(S_SENSOR); }
  else { buildReport(curSys); repPage = 0; navForward(S_REPORT); }
}
static void reportOk() {                                 // on an entry page: open the sensor of the first entry
  if (repPage < 1) return;
  int e = repN - 1 - (repPage - 1) * 2;
  if (e < 0 || e >= repN) return;
  int si = rep[e].fid / 4; if (si >= NSENS) return;
  curSens = si; curSub = SENS[si].sub; curSys = SENS[si].sys; navForward(S_SENSOR);
}

static void selectRow() {
  if (lcur < 0 || lcur >= nRows) return;
  int tag = rows[lcur].tag;
  switch (screen) {
    case S_MENU:
      if (tag == 0) navForward(S_SCAN);
      else if (tag == 1) navForward(S_SYSTEMS);
      else if (tag == 2) navForward(S_CODES);
      else if (tag == 3) openIncidentList();
      else if (tag == 4) navForward(S_STORAGE);
      else if (tag == 6) navForward(S_FLOG);
      else navForward(S_STATUS);
      break;
    case S_SYSTEMS: curSys = tag; navForward(S_SUBS); break;
    case S_SUBS:    curSub = tag; navForward(S_SENSORS); break;
    case S_SENSORS: curSens = tag; navForward(S_SENSOR); break;
    case S_SCAN:
      if (activeCount() == 0) { if (tag >= 0 && tag < flogN) { fevtSel = tag; navForward(S_FEVT); } }
      else if (tag >= 0) openSensorOfFault(tag);
      break;
    case S_FLOG:    if (tag >= 0 && tag < flogN) { fevtSel = tag; navForward(S_FEVT); } break;
    case S_CODES:   if (tag >= 0) openSensorOfFault(tag); break;
    case S_HIST:
      if (tag >= 0) { int mi = tag / MAXF; if (mi < metaN) openIncident(mi, curSens); }
      break;
    case S_STORAGE:
      if (tag == 2) { sendTestTelegram(); break; }
      if (tag == 3) { buzzerTestTune(); toast(PIN_BUZZER >= 0 ? "Buzzer test" : "No buzzer pin set"); break; }
      if (tag == 4) { navForward(S_TOUCH); break; }
      if (tag == 5) { startCalibration(); break; }
      if (confirmArm == tag + 1) {
        if (tag == 0) { eraseAllLogs(); toast("Black box erased"); }
        else { thrDefaults(); LittleFS.remove("/thr.bin"); toast("Limits reset"); }
        confirmArm = 0;
      } else confirmArm = tag + 1;
      dirty = true;
      break;
    case S_INC_DETAIL:
      if (tag == 1000) navForward(S_SNAP);
      else if (tag == 1001) { curCol = viewTrig >= 0 && viewCount > 0 ? (int)((long)viewTrig * SCREEN_W / viewCount) : 0; navForward(S_REPLAY); }
      else if (tag >= 0 && tag < viewHdr.nf && viewHdr.faults[2 * tag] / 4 < NSENS) {
        vsig = viewHdr.faults[2 * tag] / 4;
        curCol = viewCount > 0 ? (int)((long)viewHdr.fIdx[tag] * SCREEN_W / viewCount) : 0;
        if (curCol > SCREEN_W - 1) curCol = SCREEN_W - 1;
        navForward(S_REPLAY);
      }
      break;
    case S_SNAP: vsig = tag; navForward(S_REPLAY); break;
    default: break;
  }
}

static void stepSensor(int dir) {                    // previous / next sensor inside the same sub-system
  int i = curSens;
  for (int n = 0; n < NSENS; n++) {
    i = (i + dir + NSENS) % NSENS;
    if (SENS[i].sub == SENS[curSens].sub) { curSens = i; break; }
  }
  dirty = true;
}
static void openSensorHistory() {
  int tot = 0; for (int k = 0; k < 4; k++) tot += fHist[FID(curSens, k)];
  if (!tot) { toast("No problem recorded"); return; }
  navForward(S_HIST);
}

static void uiBack() { if (screen != S_DRIVER) navBack(); }

static void uiLeft(bool rep) {
  if (isListScreen(screen)) { if (lcur > 0) lcur--; dirty = true; return; }
  switch (screen) {
    case S_HOME: lcur = lcur > 0 ? lcur - 1 : NAPPS - 1; break;
    case S_APP: buildApp(); lcur = lcur > 0 ? lcur - 1 : appN; break;
    case S_REPORT: if (repPage > 0) repPage--; break;
    case S_SENSOR: stepSensor(-1); break;
    case S_FEVT: if (fevtSel < flogN - 1) fevtSel++; break;      // newer
    case S_INC_LIST:
      if (typing) {
        if (mlen) { mlen = 0; mbuf[0] = 0; }
        else { int n = (int)strlen(search); if (n) { search[n - 1] = 0; rebuildFilter(); } }
      } else if (iLcur > -1) { iLcur--; if (iLcur < iLtop) iLtop = iLcur < 0 ? 0 : iLcur; }
      break;
    case S_REPLAY: curCol -= rep ? 4 : 1; if (curCol < 0) curCol = 0; break;
    default: break;
  }
  dirty = true;
}
static void uiRight(bool rep) {
  if (isListScreen(screen)) { if (lcur < nRows - 1) lcur++; dirty = true; return; }
  switch (screen) {
    case S_HOME: lcur = (lcur + 1) % NAPPS; break;
    case S_APP: buildApp(); lcur = (lcur + 1) % (appN + 1); break;
    case S_REPORT: repPage++; break;
    case S_SENSOR: stepSensor(1); break;
    case S_FEVT: if (fevtSel > 0) fevtSel--; break;               // older
    case S_INC_LIST:
      if (typing) { typing = false; mlen = 0; mbuf[0] = 0; }
      else if (iLcur < viewN - 1) { iLcur++; if (iLcur >= iLtop + 5) iLtop = iLcur - 4; }
      break;
    case S_REPLAY: curCol += rep ? 4 : 1; if (curCol > SCREEN_W - 1) curCol = SCREEN_W - 1; break;
    default: break;
  }
  dirty = true;
}
static void uiOk() {
  if (isListScreen(screen)) { selectRow(); return; }
  switch (screen) {
    case S_HOME: homeOk(); break;
    case S_APP: appOk(); break;
    case S_SCANRUN: scanFinish(); break;
    case S_REPORT: reportOk(); break;
    case S_SENSOR: openSensorHistory(); break;
    case S_INC_LIST:
      if (iLcur == -1) { typing = true; mlen = 0; mbuf[0] = 0; dirty = true; }
      else if (iLcur < viewN) openIncident(viewIdx[iLcur], -1);
      break;
    case S_REPLAY: vsig = (vsig + 1) % NSENS; dirty = true; break;
    case S_TOUCH: startCalibration(); break;
    default: break;
  }
}

static void morseKey(uint32_t dur) {
  if (dur >= 1500) { typing = false; mlen = 0; mbuf[0] = 0; dirty = true; return; }
  if (mlen < 6) { mbuf[mlen++] = dur < 300 ? '.' : '-'; mbuf[mlen] = 0; }
  mLast = millis(); dirty = true;
}
static void morseTick(uint32_t now) {
  if (!(screen == S_INC_LIST && typing && mlen > 0 && now - mLast > 800)) return;
  int found = -1;
  for (int i = 0; i < 36; i++) if (strcmp(MORSE_TAB[i], mbuf) == 0) { found = i; break; }
  if (found >= 0) {
    int n = (int)strlen(search);
    if (n < 12) { search[n] = found < 26 ? (char)('A' + found) : (char)('0' + found - 26); search[n + 1] = 0; }
    rebuildFilter();
  } else toast("Bad morse code");
  mlen = 0; mbuf[0] = 0; dirty = true;
}

static void uiTap(int x, int y) {
  if (screen != S_DRIVER && y < 15 && x < 28) { uiBack(); return; }
  if (confirmArm && screen == S_STORAGE) {
    if (y >= 100) uiOk(); else { confirmArm = 0; dirty = true; }
    return;
  }
  if (screen == S_DRIVER) {
    if (canOpenMenu()) openMechanic(); else toast("Stop vehicle first");
    return;
  }
  if (isListScreen(screen)) {
    if (y >= listY && y < listY + listVis * rowH) {
      int idx = ltop + (y - listY) / rowH;
      if (idx < nRows) { lcur = idx; selectRow(); }
    }
    return;
  }
  switch (screen) {
    case S_SENSOR:
      if (y >= 139) openSensorHistory();
      else if (y < 44) stepSensor(x < SCREEN_W / 2 ? -1 : 1);
      break;
    case S_INC_LIST:
      if (y >= 17 && y < 30) { iLcur = -1; typing = true; mlen = 0; mbuf[0] = 0; dirty = true; }
      else if (y >= 31 && y < 151) {
        int idx = iLtop + (y - 31) / 24;
        if (idx < viewN) { iLcur = idx; openIncident(viewIdx[idx], -1); }
      }
      break;
    case S_REPLAY:
      if (y >= 16 && y < 90) { curCol = x; dirty = true; }
      else if (y >= 90) { vsig = (vsig + (x < SCREEN_W / 2 ? NSENS - 1 : 1)) % NSENS; dirty = true; }
      break;
    default: break;
  }
}
static void uiDrag(int x, int y) {
  if (screen == S_REPLAY && y >= 10 && y < 92) { curCol = x < 0 ? 0 : (x > SCREEN_W - 1 ? SCREEN_W - 1 : x); dirty = true; }
}

// ============================ 19. INPUT POLLING ===========================
static int btnUpdate(Btn& b, uint32_t now) {         // 1 = pressed, 2 = released
  bool r = (digitalRead(b.pin) == LOW);
  if (r != b.raw) { b.raw = r; b.tEdge = now; }
  if ((now - b.tEdge) >= 30 && b.down != b.raw) {
    b.down = b.raw;
    if (b.down) { b.tDown = now; b.lastRep = now; return 1; }
    return 2;
  }
  return 0;
}

static void handleButtons(uint32_t now) {
  int el = btnUpdate(bL, now), er = btnUpdate(bR, now), em = btnUpdate(bM, now);
  if (supL && !bL.down) supL = false;
  if (supR && !bR.down) supR = false;

  if (screen == S_DRIVER) {
    if (bL.down && bR.down) {
      if (!comboArmed) { comboArmed = true; comboStart = now; }
      else if (!comboFired && now - comboStart > 500) {
        comboFired = true;
        if (canOpenMenu()) { supL = supR = true; openMechanic(); }
        else toast("Stop vehicle first");
      }
    } else { comboArmed = false; comboFired = false; }
    if (em == 2) { fMuted = true; if (canOpenMenu()) openMechanic(); }      // OK button = open the scan tool (also mutes the buzzer)
#if BUZZER_CLICK
    if (el == 1 || er == 1) beep(12, 3200);
#endif
    return;
  }

  bool repL = false, repR = false;
  if (bL.down && !supL && now - bL.tDown > 400 && now - bL.lastRep > 80) { bL.lastRep = now; repL = true; }
  if (bR.down && !supR && now - bR.tDown > 400 && now - bR.lastRep > 80) { bR.lastRep = now; repR = true; }
  if (!supL && (el == 1 || repL)) { confirmArm = 0; uiLeft(repL); }
  if (!supR && (er == 1 || repR)) { confirmArm = 0; uiRight(repR); }
#if BUZZER_CLICK
  if (el == 1 || er == 1 || em == 2) beep(12, 3200);
#endif
  if (em == 2) {
    uint32_t d = now - bM.tDown;
    if (screen == S_INC_LIST && typing) morseKey(d);
    else if (d >= 700) uiBack();
    else uiOk();
  }
}

static void calPoll(bool p, int rx, int ry) {            // calibration: one tap per target, taken when the finger is lifted
  static long sumX = 0, sumY = 0; static int cnt = 0;
  if (p) {
    if (cnt < 8) { sumX += rx; sumY += ry; cnt++; }
    calLatched = true;
  } else if (calLatched) {
    calLatched = false;
    if (cnt >= 3 && calStep < 4) {
      calRx[calStep] = (int)(sumX / cnt); calRy[calStep] = (int)(sumY / cnt);
      calStep++; dirty = true;
#if BUZZER_CLICK
      beep(30, 3000);
#endif
      if (calStep >= 4) {
        if (touchCalCompute()) { touchCalSave(); toast("Touch saved"); navBack(); }
        else { toast("Bad taps, try again"); calStep = 0; }
      }
    }
    sumX = sumY = 0; cnt = 0;
  }
}

static void touchPoll(uint32_t now) {
#if USE_TOUCH
  if (now - tLastPoll < 20) return;
  tLastPoll = now;
  int rx = 0, ry = 0, z = 0, x = 0, y = 0;
  bool p = touchRaw(rx, ry, z);
  tPress = p; tRawZ = z;
  if (p) { tRawX = rx; tRawY = ry; touchMap(rx, ry, x, y); tMapX = x; tMapY = y; }
  if (screen == S_CAL) { calPoll(p, rx, ry); return; }
  if (p) {
    tUp = 0; if (tCnt < 10) tCnt++;
    if (tCnt >= 2) {
      if (!tDown) { tDown = true; tStart = now; tx0 = x; ty0 = y; tMoved = false; }
      else if (abs(x - tx0) > 6 || abs(y - ty0) > 6) { tMoved = true; uiDrag(x, y); }
    }
  } else {
    tCnt = 0;
    if (tDown && ++tUp >= 2) {
      tDown = false; tUp = 0;
      if (!tMoved && now - tStart < 900) {
#if BUZZER_CLICK
        beep(12, 3200);
#endif
        uiTap(tx0, ty0);
      }
    }
  }
#else
  (void)now;
#endif
}

static void rgbTick(uint32_t now) {
#if USE_RGB_LED
  static uint32_t last = 0; static uint8_t lr = 1, lg = 1, lb = 1;
  if (now - last < 30) return;
  last = now;
  uint8_t sev = overallSev(), r = 0, g = 0, b = 0;
  if (!haveData)         { if (((now / 700) % 2) == 0) b = RGB_LEVEL; }                                  // waiting for ECU: blue blink
  else if (fSev[F_LINK]) { if (((now / 250) % 2) == 0) { r = RGB_LEVEL; b = RGB_LEVEL; } }               // ECU link lost: purple blink
  else if (sev >= 2)     { if (((now / 140) % 2) == 0) r = RGB_LEVEL; }                                  // critical: fast red blink
  else if (sev == 1)     { r = RGB_LEVEL; g = RGB_LEVEL / 2; }                                           // warning: amber
  else                   { g = RGB_LEVEL / 2; }                                                          // everything fine: green
  if (r != lr || g != lg || b != lb) { neopixelWrite(PIN_RGB, r, g, b); lr = r; lg = g; lb = b; }
#else
  (void)now;
#endif
}

static void redLedTick(uint32_t now) {                    // single red LED status patterns
#if PIN_LED_RED >= 0
  static int last = -1;
  uint8_t sev = overallSev(); bool on;
  if (!haveData)         on = (now % 2000) < 60;                                         // waiting for ECU data: tiny flash every 2 s
  else if (fSev[F_LINK]) { uint32_t t = now % 1000; on = (t < 100) || (t >= 200 && t < 300); }   // ECU link lost: double flash
  else if (sev >= 2)     on = ((now / 90) % 2) == 0;                                     // critical: fast blink
  else if (sev == 1)     on = ((now / 500) % 2) == 0;                                    // warning: slow blink
  else                   on = (now % 3000) < 60;                                         // all fine: heartbeat flash every 3 s
  if ((int)on != last) { digitalWrite(PIN_LED_RED, on ? HIGH : LOW); last = (int)on; }
#else
  (void)now;
#endif
}

// ============================ 20. SETUP / LOOP ============================
static const char* resetReasonText(char* buf, size_t n) {
  esp_reset_reason_t r = esp_reset_reason();
  switch (r) {
    case ESP_RST_POWERON: return "power on";
    case ESP_RST_SW:      return "software restart";
    case ESP_RST_PANIC:   return "CRASH (panic)";
    case ESP_RST_INT_WDT: case ESP_RST_TASK_WDT: case ESP_RST_WDT: return "watchdog reset";
    case ESP_RST_BROWNOUT: return "brown-out (power dip)";
    default: snprintf(buf, n, "reset code %d", (int)r); return buf;
  }
}

static bool allocBuffers() {
  bool ps = psramFound();
  g_preN = (ps ? PRE_SEC_PSRAM : PRE_SEC_RAM) * SAMPLE_HZ;
  g_postN = (ps ? POST_SEC_PSRAM : POST_SEC_RAM) * SAMPLE_HZ;
  for (int attempt = 0; attempt < 4; attempt++) {
    g_incMax = g_preN + g_postN;
    ring = (Sample*)bigAlloc(sizeof(Sample) * g_preN);
    incBuf = (Sample*)bigAlloc(sizeof(Sample) * g_incMax);
    viewBuf = (Sample*)bigAlloc(sizeof(Sample) * g_incMax);
    if (ring && incBuf && viewBuf) return true;
    if (ring) free(ring);
    if (incBuf) free(incBuf);
    if (viewBuf) free(viewBuf);
    ring = incBuf = viewBuf = nullptr;
    g_preN /= 2; g_postN /= 2;                      // not enough RAM: record shorter
  }
  return false;
}

void setup() {
  Serial.setRxBufferSize(4096);
  Serial.begin(115200);
#if PIN_LINK_RX >= 0
  Serial2.setRxBufferSize(4096);
  Serial2.begin(LINK_BAUD, SERIAL_8N1, PIN_LINK_RX, PIN_LINK_TX);
#endif
  delay(300);

  pinMode(PIN_BTN_LEFT, INPUT_PULLUP);
  pinMode(PIN_BTN_RIGHT, INPUT_PULLUP);
  pinMode(PIN_BTN_MORSE, INPUT_PULLUP);
  buzzerInit();
#if USE_TOUCH
  pinMode(PIN_T_CS, OUTPUT); digitalWrite(PIN_T_CS, HIGH);
  pinMode(PIN_T_CLK, OUTPUT); digitalWrite(PIN_T_CLK, LOW);
  pinMode(PIN_T_DIN, OUTPUT);
  pinMode(PIN_T_DO, INPUT);
  pinMode(PIN_T_IRQ, INPUT_PULLUP);
#endif

  fb = (uint16_t*)malloc((size_t)SCREEN_W * SCREEN_H * 2);
  if (!fb || !allocBuffers()) { Serial.println("#ERROR: not enough RAM"); while (true) delay(1000); }
  memset(fSev, 0, sizeof(fSev)); memset(fCnt, 0, sizeof(fCnt)); memset(fClr, 0, sizeof(fClr));
  memset(fAlertMs, 0, sizeof(fAlertMs)); memset(fAlertSev, 0, sizeof(fAlertSev));
  memset(glitchUntil, 0, sizeof(glitchUntil)); memset(havePrev, 0, sizeof(havePrev));
  for (int f = 0; f < NFAULT; f++) fLastMeta[f] = -1;

  tftInit();
  drawBg();
  fillRect(0, 0, SCREEN_W, 30, BLUE);
  drawIcon(56, 7, IC_ENGINE, C_WHITE, 1);
  drawIcon(40, 44, IC_VEHICLE, BLUE, 3);
  drawTextC(SCREEN_W / 2, 98, "TRUCK", 2, C_INK, TRANSP);
  drawTextC(SCREEN_W / 2, 116, "BLACK BOX", 2, BLUE, TRANSP);
  drawTextC(SCREEN_W / 2, 142, "starting...", 1, C_MUTED, TRANSP);
  flushScreen();

  bool fsOk = LittleFS.begin(true);
  thrDefaults();
  if (fsOk) { LittleFS.mkdir("/inc"); scanIncidents(); flogLoad(); thrLoad(); touchCalLoad(); lastCleaned = autoClean(); }
  Serial.printf("#BOOT v4 flash %s, %d incidents, %d log events, %u KB free, %d sensors, rec %d+%d samples, psram=%d\n",
                fsOk ? "OK" : "FAILED", metaN, flogN, fsOk ? (unsigned)(fsFree() / 1024) : 0u, (int)NSENS, g_preN, g_postN, psramFound() ? 1 : 0);

  staEnabled = (strncmp(WIFI_SSID, "YOUR_", 5) != 0);
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(AP_SSID, AP_PASS);
  if (staEnabled) {
    WiFi.setAutoReconnect(true);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    configTime(TIME_ZONE_SEC, 0, "pool.ntp.org", "time.google.com");
  }
  webSetup();

  tgEnabled = staEnabled && (strncmp(TG_BOT_TOKEN, "PASTE", 5) != 0);
  if (tgEnabled) {
    tgQ = xQueueCreate(4, sizeof(TgMsg));
    xTaskCreatePinnedToCore(tgTask, "tg", 16384, NULL, 1, NULL, 0);
#if TG_SEND_BOOT
    char rs[24], msg[260];
    snprintf(msg, sizeof(msg), "✅ Truck Black Box started (%s)\n💾 %d incidents stored, %u KB flash free\n📶 Data: USB + WiFi ready",
             resetReasonText(rs, sizeof(rs)), metaN, fsOk ? (unsigned)(fsFree() / 1024) : 0u);
    if (!fsOk) snprintf(msg, sizeof(msg), "💾❌ Truck Black Box started but FLASH STORAGE FAILED");
    tgEnqueue(msg);
#endif
  } else {
    Serial.println("#Telegram is OFF (WiFi name / bot token not set)");
  }
  memset(&latest, 0, sizeof(latest));
#if PIN_LED_RED >= 0
  pinMode(PIN_LED_RED, OUTPUT); digitalWrite(PIN_LED_RED, HIGH);
#endif
  buzzerSet(true, 2000); delay(90); buzzerSet(true, 3000); delay(90); buzzerSet(false, 0);   // start-up chirp + LED flash (proves both work)
#if PIN_LED_RED >= 0
  digitalWrite(PIN_LED_RED, LOW);
#endif
  delay(500);
  dirty = true;
#if USE_TOUCH
  if (digitalRead(PIN_BTN_MORSE) == LOW) startCalibration();     // MORSE held while powering on = touch calibration
#endif
}

void loop() {
  uint32_t now = millis();
  static uint32_t lastStat = 0, lastNtp = 0;
  serialPump();
  uartPump();
  web.handleClient();
  linkCheck(now);
  if (incState == INC_REC && fSev[F_LINK]) finishIncident();
  pendingTick(now);
  if (screen == S_SCANRUN && millis() - scanT0 > (uint32_t)(NSYS + 2) * 280) scanFinish();
  remindTick(now);
  handleButtons(now);
  touchPoll(now);
  morseTick(now);
  buzzerTick(now);
  rgbTick(now);
  redLedTick(now);
  if (thrDirty && now - thrDirtyMs > 2000) { thrSave(); thrDirty = false; }

  if (!epochBase && now - lastNtp > 5000) {          // clock from the internet when no PC has set it
    lastNtp = now;
    time_t t = time(nullptr);
    if (t > 1700000000L) epochBase = (uint32_t)t + TIME_ZONE_SEC - now / 1000;
  }
  if (screen != S_DRIVER && !ALLOW_MENU_WHILE_MOVING && haveData && curSpeed() >= MECH_SPEED_LIMIT + 3.0f) {
    navHome(); toast("Vehicle moving");
  }
  if (toastMsg[0] && millis() > toastUntil) { toastMsg[0] = 0; dirty = true; }
  if (now - lastStat > 5000) {
    lastStat = now;
    Serial.printf("#STAT rx=%u speed=%.0f link=%s wifi=%d tg=%d sent=%u heap=%u\n", (unsigned)rxLines, curSpeed(), linkOk() ? "ok" : "no",
                  (int)WiFi.status(), (int)tgState, (unsigned)tgSent, (unsigned)ESP.getFreeHeap());
  }

  bool live = (screen == S_DRIVER || screen == S_SCAN || screen == S_SYSTEMS || screen == S_SUBS || screen == S_SENSORS ||
               screen == S_SENSOR || screen == S_CODES || screen == S_STATUS || screen == S_MENU || screen == S_HOME || screen == S_APP);
  bool fast = (screen == S_TOUCH || screen == S_CAL || screen == S_SCANRUN);
  if ((dirty && now - lastRender >= 70) || (live && now - lastRender >= 400) || (fast && now - lastRender >= 60)) {
    render(); lastRender = now; dirty = false;
  }
  delay(1);
}
