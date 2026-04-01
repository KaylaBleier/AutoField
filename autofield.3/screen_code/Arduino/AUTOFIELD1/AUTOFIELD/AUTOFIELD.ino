/**
 * AUTOFIELD.ino
 *
 * Hardware  : Elecrow CrowPanel ESP32 7" (DIS08070H)  800x480
 * LVGL      : v9.x  (port file handles display init)
 * Input     : Serial monitor (laptop testing)
 * Comms     : WiFi/HTTP stub - fill in SSID/IP when ready
 *
 * ── File layout ──────────────────────────────────────────────
 *   AUTOFIELD.ino        <- this file
 *   autofield_ui.cpp     <- LVGL widget code
 *   autofield_ui.h       <- function declarations
 *   lvgl_v9_port.h       <- copy from ESP32_Display_Panel/examples/  (see note below)
 *   lvgl_v9_port.cpp     <- copy from ESP32_Display_Panel/examples/
 *
 * ── Finding the port files ───────────────────────────────────
 *   In your libraries folder open:
 *     ESP32_Display_Panel/examples/
 *   Find any example that has lvgl_v8_port.h / lvgl_v8_port.cpp
 *   (or lvgl_v9_port.* if present) and copy those two files into
 *   your AUTOFIELD folder alongside this .ino
 *
 * ── Serial monitor key mapping ───────────────────────────────
 *   0-9        numeric input
 *   # or Enter confirm / advance
 *   b          backspace
 *   x          cancel / abort to idle
 */

/* ════════════════════════════════════════════════════════════
 *  Includes
 * ════════════════════════════════════════════════════════════ */
#include <Arduino.h>
#include <esp_display_panel.hpp>   /* lowercase - matches example */
#include <lvgl.h>
#include "lvgl_v8_port.h"          /* copy from ESP32_Display_Panel examples */
#include "autofield_ui.h"

using namespace esp_panel::drivers;
using namespace esp_panel::board;

/* ── WiFi (stubbed - uncomment when ready) ───────────────── */
// #include <WiFi.h>
// #include <HTTPClient.h>
// #include <ArduinoJson.h>
// const char* WIFI_SSID     = "YOUR_SSID";
// const char* WIFI_PASSWORD = "YOUR_PASSWORD";
// const char* PI_URL        = "http://192.168.1.XX:5000/session";

/* ════════════════════════════════════════════════════════════
 *  WiFi helpers (stubbed)
 * ════════════════════════════════════════════════════════════ */
static void wifi_connect(void)
{
    /* Uncomment when ready:
    Serial.print("Connecting to WiFi");
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500); Serial.print(".");
    }
    Serial.println(" connected");
    Serial.println(WiFi.localIP());
    */
}

static void send_session_to_pi(void)
{
    /* Uncomment + fill in PI_URL when ready:

    if (WiFi.status() != WL_CONNECTED) return;

    StaticJsonDocument<128> doc;
    doc["user_id"]     = autofield_get_user_id();
    doc["field_width"] = autofield_get_field_width();
    doc["field_depth"] = autofield_get_field_depth();

    String body;
    serializeJson(doc, body);

    HTTPClient http;
    http.begin(PI_URL);
    http.addHeader("Content-Type", "application/json");
    int code = http.POST(body);
    Serial.printf("POST to Pi: HTTP %d\n", code);
    http.end();
    */

    Serial.printf("[STUB] Would POST to Pi: user_id=%s  width=%d  depth=%d\n",
                  autofield_get_user_id(),
                  autofield_get_field_width(),
                  autofield_get_field_depth());
}

/* ════════════════════════════════════════════════════════════
 *  Serial keyboard input
 *  Open Serial Monitor at 115200, line ending = "No line ending"
 * ════════════════════════════════════════════════════════════ */
static void poll_keyboard(void)
{
    if (!Serial.available()) return;

    char raw = Serial.read();
    char key = 0;

    if      (raw >= '0' && raw <= '9')                    key = raw;
    else if (raw == '#' || raw == '\r' || raw == '\n')    key = '#';
    else if (raw == 8 || raw == 127 || raw == 'b' || raw == 'B') key = '*';
    else if (raw == 27 || raw == 'x'  || raw == 'X')     key = '*';

    if (key == 0) return;

    Serial.printf("[KEY] '%c' -> '%c'\n", raw, key);

    ui_state_t prev = autofield_get_state();
    autofield_key_handler(key);
    ui_state_t next = autofield_get_state();

    /* Auto-fire when entering RUNNING state */
    if (prev != STATE_RUNNING && next == STATE_RUNNING) {
        send_session_to_pi();
    }
}

/* ════════════════════════════════════════════════════════════
 *  setup()
 * ════════════════════════════════════════════════════════════ */
void setup()
{
    Serial.begin(115200);
    Serial.println("\n-- AutoField booting --");

    /* 1. Init the board hardware (LCD, backlight, etc.) */
    Serial.println("Initializing board");
    Board *board = new Board();
    board->init();
    assert(board->begin());

    /* 2. Init LVGL + display port (handles buffers, flush, tick) */
    Serial.println("Initializing LVGL port");
    lvgl_port_init(board->getLCD(), nullptr);  /* nullptr = no touch */

    /* 3. Lock LVGL mutex before touching any widgets */
    lvgl_port_lock(-1);

    /* 4. Build the AutoField UI */
    Serial.println("Creating AutoField UI");
    autofield_ui_init();

    /* 5. Release mutex - port task takes over rendering */
    lvgl_port_unlock();

    /* 6. WiFi (stubbed) */
    wifi_connect();

    Serial.println("Ready. # = start/confirm  0-9 = digits  b = backspace  x = abort");
}

/* ════════════════════════════════════════════════════════════
 *  loop()
 *  The lvgl_port runs LVGL in its own FreeRTOS task, so we
 *  don't call lv_timer_handler() here. We just handle input
 *  and lock the mutex any time we touch LVGL objects.
 * ════════════════════════════════════════════════════════════ */
void loop()
{
    if (Serial.available()) {
        lvgl_port_lock(-1);      /* lock before touching UI */
        poll_keyboard();
        lvgl_port_unlock();
    }
    delay(20);
}
