/* Station_OS — CYD Station Display
 * NY&E Northern Lights Subdivision
 *
 * Subscribe: trains/clock/time    (retained, QoS 0)
 *            trains/clock/control (QoS 1)
 * Publish:   trains/station/{id}/status  (retained, QoS 1, LWT, 60 s heartbeat)
 *            trains/clock/sync_request   (QoS 0, on reconnect)
 *
 * Provisioning: connect via serial monitor (115200), type 'help'.
 * First boot (not provisioned) prints the prompt automatically.
 */

#include <Arduino.h>
#include <WiFi.h>
#include <AsyncMqttClient.h>
#include <Preferences.h>
#include <ArduinoJson.h>
#include <LittleFS.h>
#include <lvgl.h>
#include <TFT_eSPI.h>
#include <XPT2046_Touchscreen.h>
#include <vector>

extern "C" {
  #include "freertos/FreeRTOS.h"
  #include "freertos/timers.h"
}

// ── Hardware ──────────────────────────────────────────────────────────────
#define XPT2046_IRQ  36
#define XPT2046_MOSI 32
#define XPT2046_MISO 39
#define XPT2046_CLK  25
#define XPT2046_CS   33
// Logical display dimensions after rotation (landscape)
#define SCREEN_WIDTH  320
#define SCREEN_HEIGHT 240
#define DRAW_BUF_SIZE (SCREEN_WIDTH * SCREEN_HEIGHT / 10 * (LV_COLOR_DEPTH / 8))

static const int  MQTT_PORT           = 1883;
#define FIRMWARE_VER "1.0.0"
static const int  HEARTBEAT_SECS      = 60;
static const int  DISPLAY_UPDATE_MS   = 1000;
// Minutes before departure to consider a train "now" on display
static const int  NOW_WINDOW_MIN      = 3;

// ── NVS ───────────────────────────────────────────────────────────────────
static const char* NVS_NS    = "sta_os";
static const char* KEY_PROV  = "prov";
static const char* KEY_SSID  = "ssid";
static const char* KEY_WPAS  = "wpas";
static const char* KEY_MQTT  = "mqtt_ip";
static const char* KEY_MUSER = "muser";
static const char* KEY_MPAS  = "mpas";
static const char* KEY_STAID = "sta_id";

// ── Config ────────────────────────────────────────────────────────────────
struct Config {
    char ssid[64]    = {};
    char wpas[64]    = {};      // empty = open network
    char mqtt_ip[40] = {};
    char muser[32]   = "cyd_unit";
    char mpas[32]    = {};
    char sta_id[4]   = {};
};

static Config      cfg;
static Preferences prefs;
static bool        provisioned = false;

// ── Clock state ───────────────────────────────────────────────────────────
struct ClockState {
    int           base_hour  = 0;
    int           base_min   = 0;
    int           speed      = 3;
    bool          running    = false;
    bool          synced     = false;
    unsigned long tick_ms    = 0;   // millis() at last MQTT tick receipt
};

static ClockState         clk;
static SemaphoreHandle_t  clkMutex = nullptr;

// ── Schedule data ─────────────────────────────────────────────────────────
struct TrainEntry {
    char num[8];
    int  time;      // minutes since midnight, 0–1439
};

static char stationName[48]         = "---";
static std::vector<TrainEntry> schedN;   // northbound at this station
static std::vector<TrainEntry> schedS;   // southbound at this station

// ── LVGL / display ────────────────────────────────────────────────────────
static TFT_eSPI       tft;                          // global — initialized in setup()
static uint32_t       draw_buf[DRAW_BUF_SIZE / 4];
static lv_obj_t*      lbl_station = nullptr;
static lv_obj_t*      lbl_time    = nullptr;
static lv_obj_t*      lbl_nextN   = nullptr;
static lv_obj_t*      lbl_nextS   = nullptr;
static lv_obj_t*      dot_status  = nullptr;

static SPIClass             touchscreenSPI(VSPI);
static XPT2046_Touchscreen  touchscreen(XPT2046_CS, XPT2046_IRQ);

// Direct LVGL flush callback — bypasses lv_tft_espi_create wrapper
static void disp_flush(lv_display_t* disp, const lv_area_t* area, uint8_t* px_map) {
    uint32_t w = area->x2 - area->x1 + 1;
    uint32_t h = area->y2 - area->y1 + 1;
    tft.startWrite();
    tft.setAddrWindow(area->x1, area->y1, w, h);
    tft.pushColors((uint16_t*)px_map, w * h, true);
    tft.endWrite();
    lv_display_flush_ready(disp);
}

// ── MQTT ──────────────────────────────────────────────────────────────────
// LWT strings must outlive setup() — AsyncMqttClient stores raw pointers.
static char lwtTopic[48]   = {};
static char lwtPayload[64] = {};

static AsyncMqttClient mqttClient;
static TimerHandle_t   mqttReconTimer  = nullptr;
static TimerHandle_t   wifiReconTimer  = nullptr;
static TimerHandle_t   heartbeatTimer  = nullptr;
static bool            mqttReady       = false;
static volatile bool   needHeartbeat   = false;
static volatile bool   needSyncRequest = false;
// Set by MQTT callbacks (Core 0); read by LVGL timer (Core 1) — must not call LVGL from callbacks
static volatile bool   mqttOnline      = false;
static volatile bool   mqttStatusDirty = false;

// ── Forward declarations ──────────────────────────────────────────────────
static void loadConfig();
static void saveConfig();
static void loadSchedule();
static int  getInterpolatedMin();
static void findNextTrain(const std::vector<TrainEntry>& sched, int cur_min,
                          char* out, size_t n);
static void updateClockDisplay();
static void buildClockScreen();
static void publishStatus();
static void connectToWifi();
static void connectToMqtt();
static void onWifiGotIP(WiFiEvent_t, WiFiEventInfo_t);
static void onWifiDisconnect(WiFiEvent_t, WiFiEventInfo_t);
static void onMqttConnect(bool sessionPresent);
static void onMqttDisconnect(AsyncMqttClientDisconnectReason reason);
static void onMqttMessage(char* topic, char* payload,
                          AsyncMqttClientMessageProperties props,
                          size_t len, size_t index, size_t total);
static void touchscreen_read(lv_indev_t* indev, lv_indev_data_t* data);
static void handleSerialCLI();
static void printHelp();
static void printConfig();

// ═══════════════════════════════════════════════════════════════════════════
// NVS / Config
// ═══════════════════════════════════════════════════════════════════════════

static void loadConfig() {
    prefs.begin(NVS_NS, true);
    provisioned = prefs.getBool(KEY_PROV, false);
    prefs.getString(KEY_SSID,  cfg.ssid,    sizeof(cfg.ssid));
    prefs.getString(KEY_WPAS,  cfg.wpas,    sizeof(cfg.wpas));
    prefs.getString(KEY_MQTT,  cfg.mqtt_ip, sizeof(cfg.mqtt_ip));
    prefs.getString(KEY_MUSER, cfg.muser,   sizeof(cfg.muser));
    prefs.getString(KEY_MPAS,  cfg.mpas,    sizeof(cfg.mpas));
    prefs.getString(KEY_STAID, cfg.sta_id,  sizeof(cfg.sta_id));
    prefs.end();
    if (strlen(cfg.muser) == 0) strlcpy(cfg.muser, "cyd_unit", sizeof(cfg.muser));
}

static void saveConfig() {
    prefs.begin(NVS_NS, false);
    prefs.putBool(KEY_PROV,  true);
    prefs.putString(KEY_SSID,  cfg.ssid);
    prefs.putString(KEY_WPAS,  cfg.wpas);
    prefs.putString(KEY_MQTT,  cfg.mqtt_ip);
    prefs.putString(KEY_MUSER, cfg.muser);
    prefs.putString(KEY_MPAS,  cfg.mpas);
    prefs.putString(KEY_STAID, cfg.sta_id);
    prefs.end();
    provisioned = true;
}

// ═══════════════════════════════════════════════════════════════════════════
// Schedule (LittleFS)
// ═══════════════════════════════════════════════════════════════════════════

static void loadSchedule() {
    File f = LittleFS.open("/schedule.json", "r");
    if (!f) {
        Serial.println("schedule.json not found — upload filesystem image");
        return;
    }

    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, f);
    f.close();
    if (err) {
        Serial.printf("schedule.json parse error: %s\n", err.c_str());
        return;
    }

    const char* sta = cfg.sta_id;
    if (!doc[sta].is<JsonObject>()) {
        Serial.printf("Station %s not found in schedule.json\n", sta);
        return;
    }

    strlcpy(stationName, doc[sta]["name"] | "---", sizeof(stationName));

    for (JsonObject e : doc[sta]["N"].as<JsonArray>()) {
        TrainEntry te;
        strlcpy(te.num, e["num"] | "?", sizeof(te.num));
        te.time = e["time"] | 0;
        schedN.push_back(te);
    }
    for (JsonObject e : doc[sta]["S"].as<JsonArray>()) {
        TrainEntry te;
        strlcpy(te.num, e["num"] | "?", sizeof(te.num));
        te.time = e["time"] | 0;
        schedS.push_back(te);
    }

    Serial.printf("Loaded schedule for %s: %d NB, %d SB trains\n",
                  sta, schedN.size(), schedS.size());
}

// ═══════════════════════════════════════════════════════════════════════════
// Clock interpolation
// ═══════════════════════════════════════════════════════════════════════════

// Returns interpolated railroad time in minutes since midnight (0–1439).
static int getInterpolatedMin() {
    if (!clk.synced) return -1;
    int total = clk.base_hour * 60 + clk.base_min;
    if (clk.running) {
        unsigned long elapsed_ms = millis() - clk.tick_ms;
        total += (int)((float)elapsed_ms / 1000.0f * clk.speed / 60.0f);
    }
    return total % 1440;
}

static void minToTimeStr(int total_min, char* buf, size_t n) {
    if (total_min < 0) {
        snprintf(buf, n, "--:-- --");
        return;
    }
    total_min %= 1440;
    int h = total_min / 60;
    int m = total_min % 60;
    const char* ampm = (h < 12) ? "AM" : "PM";
    int h12 = h % 12;
    if (h12 == 0) h12 = 12;
    snprintf(buf, n, "%d:%02d %s", h12, m, ampm);
}

// Fills out[] with a display string for the next train after cur_min.
// If cur_min == -1 (not synced), writes "---".
static void findNextTrain(const std::vector<TrainEntry>& sched, int cur_min,
                          char* out, size_t n) {
    if (sched.empty()) {
        snprintf(out, n, "  ─────────");
        return;
    }
    if (cur_min < 0) {
        snprintf(out, n, "  ---");
        return;
    }

    // Search for first entry with time >= cur_min
    const TrainEntry* found = nullptr;
    for (const auto& e : sched) {
        if (e.time >= cur_min) { found = &e; break; }
    }
    // Wrap to next day
    if (!found) found = &sched[0];

    char tstr[12];
    int h = found->time / 60, m = found->time % 60;
    const char* ap = (h < 12) ? "AM" : "PM";
    int h12 = h % 12; if (h12 == 0) h12 = 12;
    snprintf(tstr, sizeof(tstr), "%d:%02d %s", h12, m, ap);

    bool isNow = (found->time >= cur_min && found->time - cur_min <= NOW_WINDOW_MIN);
    if (isNow)
        snprintf(out, n, "  No.%-3s  NOW", found->num);
    else
        snprintf(out, n, "  No.%-3s  %s", found->num, tstr);
}

// ═══════════════════════════════════════════════════════════════════════════
// LVGL display
// ═══════════════════════════════════════════════════════════════════════════

static void touchscreen_read(lv_indev_t* indev, lv_indev_data_t* data) {
    if (touchscreen.tirqTouched() && touchscreen.touched()) {
        TS_Point p = touchscreen.getPoint();
        float ax = -0.000f, bx = 0.090f, dx = -33.771f;
        float ay =  0.066f, by = 0.000f, dy = -14.632f;
        int xi = (int)(ay * p.x + by * p.y + dy);
        int yi = (int)(ax * p.x + bx * p.y + dx);
        data->point.x = constrain(xi, 0, SCREEN_WIDTH  - 1);
        data->point.y = constrain(yi, 0, SCREEN_HEIGHT - 1);
        data->state   = LV_INDEV_STATE_PRESSED;
    } else {
        data->state = LV_INDEV_STATE_RELEASED;
    }
}

static void buildClockScreen() {
    lv_obj_t* scr = lv_screen_active();
    lv_obj_set_style_bg_color(scr, lv_color_black(), LV_PART_MAIN);

    // Station name — top left
    lbl_station = lv_label_create(scr);
    lv_label_set_text(lbl_station, stationName);
    lv_obj_set_style_text_font(lbl_station, &lv_font_montserrat_20, LV_PART_MAIN);
    lv_obj_set_style_text_color(lbl_station, lv_palette_main(LV_PALETTE_YELLOW), LV_PART_MAIN);
    lv_obj_align(lbl_station, LV_ALIGN_TOP_LEFT, 8, 8);

    // MQTT status dot — top right
    dot_status = lv_obj_create(scr);
    lv_obj_set_size(dot_status, 12, 12);
    lv_obj_set_style_radius(dot_status, LV_RADIUS_CIRCLE, LV_PART_MAIN);
    lv_obj_set_style_border_width(dot_status, 0, LV_PART_MAIN);
    lv_obj_set_style_bg_color(dot_status, lv_palette_main(LV_PALETTE_RED), LV_PART_MAIN);
    lv_obj_align(dot_status, LV_ALIGN_TOP_RIGHT, -8, 12);

    // Railroad time — large, center
    lbl_time = lv_label_create(scr);
    lv_label_set_text(lbl_time, "--:-- --");
    lv_obj_set_style_text_font(lbl_time, &lv_font_montserrat_48, LV_PART_MAIN);
    lv_obj_set_style_text_color(lbl_time, lv_color_white(), LV_PART_MAIN);
    lv_obj_align(lbl_time, LV_ALIGN_CENTER, 0, -20);

    // Divider line
    lv_obj_t* line = lv_obj_create(scr);
    lv_obj_set_size(line, 300, 2);
    lv_obj_set_style_bg_color(line, lv_palette_darken(LV_PALETTE_GREY, 2), LV_PART_MAIN);
    lv_obj_set_style_border_width(line, 0, LV_PART_MAIN);
    lv_obj_set_style_radius(line, 0, LV_PART_MAIN);
    lv_obj_align(line, LV_ALIGN_CENTER, 0, 45);

    // Next northbound
    lbl_nextN = lv_label_create(scr);
    lv_label_set_text(lbl_nextN, "NB:  ---");
    lv_obj_set_style_text_font(lbl_nextN, &lv_font_montserrat_16, LV_PART_MAIN);
    lv_obj_set_style_text_color(lbl_nextN, lv_palette_lighten(LV_PALETTE_GREEN, 2), LV_PART_MAIN);
    lv_obj_align(lbl_nextN, LV_ALIGN_CENTER, 0, 65);

    // Next southbound
    lbl_nextS = lv_label_create(scr);
    lv_label_set_text(lbl_nextS, "SB:  ---");
    lv_obj_set_style_text_font(lbl_nextS, &lv_font_montserrat_16, LV_PART_MAIN);
    lv_obj_set_style_text_color(lbl_nextS, lv_palette_lighten(LV_PALETTE_CYAN, 2), LV_PART_MAIN);
    lv_obj_align(lbl_nextS, LV_ALIGN_CENTER, 0, 88);
}

static void updateClockDisplay() {
    if (!lbl_time) return;

    xSemaphoreTake(clkMutex, portMAX_DELAY);
    ClockState snap = clk;
    xSemaphoreGive(clkMutex);

    int cur_min = -1;
    if (snap.synced) {
        int total = snap.base_hour * 60 + snap.base_min;
        if (snap.running) {
            unsigned long elapsed_ms = millis() - snap.tick_ms;
            total += (int)((float)elapsed_ms / 1000.0f * snap.speed / 60.0f);
        }
        cur_min = total % 1440;
    }

    char timebuf[16];
    minToTimeStr(cur_min, timebuf, sizeof(timebuf));
    lv_label_set_text(lbl_time, timebuf);

    char nbuf[32], sbuf[32];
    findNextTrain(schedN, cur_min, nbuf, sizeof(nbuf));
    findNextTrain(schedS, cur_min, sbuf, sizeof(sbuf));

    char full[48];
    snprintf(full, sizeof(full), "NB:%s", nbuf);
    lv_label_set_text(lbl_nextN, full);
    snprintf(full, sizeof(full), "SB:%s", sbuf);
    lv_label_set_text(lbl_nextS, full);
}

static void onDisplayTimer(lv_timer_t* timer) {
    if (mqttStatusDirty && dot_status) {
        lv_color_t col = mqttOnline
            ? lv_palette_main(LV_PALETTE_GREEN)
            : lv_palette_main(LV_PALETTE_RED);
        lv_obj_set_style_bg_color(dot_status, col, LV_PART_MAIN);
        mqttStatusDirty = false;
    }
    updateClockDisplay();
}

// ═══════════════════════════════════════════════════════════════════════════
// MQTT publish helpers
// ═══════════════════════════════════════════════════════════════════════════

static void publishStatus() {
    char topic[48], payload[128];
    snprintf(topic, sizeof(topic), "trains/station/%s/status", cfg.sta_id);
    snprintf(payload, sizeof(payload),
             "{\"online\":true,\"station_id\":\"%s\","
             "\"firmware\":\"%s\",\"rssi\":%d,\"clock_sync\":%s}",
             cfg.sta_id, FIRMWARE_VER, WiFi.RSSI(),
             clk.synced ? "true" : "false");
    mqttClient.publish(topic, 1, true, payload);
}

static void publishSyncRequest() {
    char payload[32];
    snprintf(payload, sizeof(payload), "{\"station_id\":\"%s\"}", cfg.sta_id);
    mqttClient.publish("trains/clock/sync_request", 0, false, payload);
}

// ═══════════════════════════════════════════════════════════════════════════
// Network
// ═══════════════════════════════════════════════════════════════════════════

static void connectToWifi() {
    if (WiFi.isConnected()) return;
    if (strlen(cfg.wpas) > 0)
        WiFi.begin(cfg.ssid, cfg.wpas);
    else
        WiFi.begin(cfg.ssid);
}

static void connectToMqtt() {
    if (mqttClient.connected()) return;
    mqttClient.connect();
}

static void onWifiGotIP(WiFiEvent_t event, WiFiEventInfo_t info) {
    Serial.printf("WiFi connected: %s\n", WiFi.localIP().toString().c_str());
    connectToMqtt();
}

static void onWifiDisconnect(WiFiEvent_t event, WiFiEventInfo_t info) {
    Serial.printf("WiFi disconnected (reason %d)\n", info.wifi_sta_disconnected.reason);
    xTimerStop(mqttReconTimer, 0);
    xTimerStart(wifiReconTimer, 0);
}

static void onMqttConnect(bool sessionPresent) {
    Serial.println("MQTT connected");
    mqttReady     = true;
    mqttOnline    = true;
    mqttStatusDirty = true;   // dot update handled by LVGL timer on Core 1

    // Subscribe to clock topics
    mqttClient.subscribe("trains/clock/time",    0);
    mqttClient.subscribe("trains/clock/control", 1);

    // Publish online status with LWT already set; request clock sync
    publishStatus();
    needSyncRequest = true;
}

static void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
    Serial.println("MQTT disconnected");
    mqttReady       = false;
    mqttOnline      = false;
    mqttStatusDirty = true;   // dot update handled by LVGL timer on Core 1

    if (WiFi.isConnected())
        xTimerStart(mqttReconTimer, 0);
}

static void onMqttMessage(char* topic, char* payload,
                          AsyncMqttClientMessageProperties props,
                          size_t len, size_t index, size_t total) {
    // Null-terminate (payload buffer is not guaranteed null-terminated)
    char buf[256];
    size_t copy = min(len, sizeof(buf) - 1);
    memcpy(buf, payload, copy);
    buf[copy] = '\0';

    JsonDocument doc;
    if (deserializeJson(doc, buf)) return;

    if (strcmp(topic, "trains/clock/time") == 0) {
        xSemaphoreTake(clkMutex, portMAX_DELAY);
        clk.base_hour = doc["hour"] | clk.base_hour;
        clk.base_min  = doc["minute"] | clk.base_min;
        clk.speed     = doc["speed"] | clk.speed;
        clk.running   = doc["running"] | clk.running;
        clk.synced    = true;
        clk.tick_ms   = millis();
        xSemaphoreGive(clkMutex);
        return;
    }

    if (strcmp(topic, "trains/clock/control") == 0) {
        const char* action = doc["action"] | "";
        xSemaphoreTake(clkMutex, portMAX_DELAY);
        if (strcmp(action, "start") == 0) {
            clk.running = true;
        } else if (strcmp(action, "pause") == 0) {
            // Freeze at current interpolated value
            if (clk.synced && clk.running) {
                int total = clk.base_hour * 60 + clk.base_min;
                unsigned long elapsed_ms = millis() - clk.tick_ms;
                total += (int)((float)elapsed_ms / 1000.0f * clk.speed / 60.0f);
                clk.base_hour = (total / 60) % 24;
                clk.base_min  = total % 60;
                clk.tick_ms   = millis();
            }
            clk.running = false;
        } else if (strcmp(action, "speed") == 0) {
            clk.speed = doc["speed"] | clk.speed;
        } else if (strcmp(action, "set") == 0) {
            if (!doc["hour"].isNull())   clk.base_hour = doc["hour"];
            if (!doc["minute"].isNull()) clk.base_min  = doc["minute"];
            clk.tick_ms = millis();
        }
        xSemaphoreGive(clkMutex);
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// FreeRTOS timer callbacks
// ═══════════════════════════════════════════════════════════════════════════

static void cbMqttRecon(TimerHandle_t) { connectToMqtt(); }
static void cbWifiRecon(TimerHandle_t) { connectToWifi(); }
static void cbHeartbeat(TimerHandle_t) { needHeartbeat = true; }

// ═══════════════════════════════════════════════════════════════════════════
// Serial CLI (provisioning)
// ═══════════════════════════════════════════════════════════════════════════

static void printHelp() {
    Serial.println("Station_OS provisioning CLI");
    Serial.println("  set ssid <value>     WiFi SSID");
    Serial.println("  set wpas <value>     WiFi password (blank = open)");
    Serial.println("  set mqtt_ip <value>  MQTT broker IP");
    Serial.println("  set muser <value>    MQTT username  [default: cyd_unit]");
    Serial.println("  set mpas <value>     MQTT password");
    Serial.println("  set sta_id <value>   Station ID (2 chars, e.g. BB)");
    Serial.println("  show                 Print current config");
    Serial.println("  save                 Save to NVS and reboot");
    Serial.println("  clear                Erase NVS and reboot");
    Serial.println("  help                 Show this message");
}

static void printConfig() {
    Serial.printf("  ssid    = %s\n", cfg.ssid);
    Serial.printf("  wpas    = %s\n", strlen(cfg.wpas) ? "(set)" : "(empty/open)");
    Serial.printf("  mqtt_ip = %s\n", cfg.mqtt_ip);
    Serial.printf("  muser   = %s\n", cfg.muser);
    Serial.printf("  mpas    = %s\n", strlen(cfg.mpas) ? "(set)" : "(empty)");
    Serial.printf("  sta_id  = %s\n", cfg.sta_id);
    Serial.printf("  prov    = %s\n", provisioned ? "yes" : "no");
}

static void handleSerialCLI() {
    if (!Serial.available()) return;

    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.isEmpty()) return;

    if (line == "help") {
        printHelp();
    } else if (line == "show") {
        printConfig();
    } else if (line == "save") {
        saveConfig();
        Serial.println("Saved. Rebooting...");
        delay(500);
        ESP.restart();
    } else if (line == "clear") {
        prefs.begin(NVS_NS, false);
        prefs.clear();
        prefs.end();
        Serial.println("NVS cleared. Rebooting...");
        delay(500);
        ESP.restart();
    } else if (line.startsWith("set ")) {
        int sp = line.indexOf(' ', 4);
        if (sp < 0) { Serial.println("Usage: set <key> <value>"); return; }
        String key = line.substring(4, sp);
        String val = line.substring(sp + 1);
        val.trim();
        if      (key == "ssid")    strlcpy(cfg.ssid,    val.c_str(), sizeof(cfg.ssid));
        else if (key == "wpas")    strlcpy(cfg.wpas,    val.c_str(), sizeof(cfg.wpas));
        else if (key == "mqtt_ip") strlcpy(cfg.mqtt_ip, val.c_str(), sizeof(cfg.mqtt_ip));
        else if (key == "muser")   strlcpy(cfg.muser,   val.c_str(), sizeof(cfg.muser));
        else if (key == "mpas")    strlcpy(cfg.mpas,    val.c_str(), sizeof(cfg.mpas));
        else if (key == "sta_id")  strlcpy(cfg.sta_id,  val.c_str(), sizeof(cfg.sta_id));
        else { Serial.printf("Unknown key: %s\n", key.c_str()); return; }
        Serial.printf("Set %s = %s\n", key.c_str(), val.c_str());
    } else {
        Serial.printf("Unknown command: %s (type 'help')\n", line.c_str());
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// setup / loop
// ═══════════════════════════════════════════════════════════════════════════

void setup() {
    Serial.begin(115200);
    delay(100);
    Serial.println("\nStation_OS v" FIRMWARE_VER " starting");

    // ── Config ────────────────────────────────────────────────────────────
    loadConfig();
    if (!provisioned) {
        Serial.println("Not provisioned. Type 'help' to configure.");
        printHelp();
        Serial.println("\nProvision this unit then type 'save'.");
    }

    // ── LittleFS ──────────────────────────────────────────────────────────
    if (!LittleFS.begin(true)) {
        Serial.println("LittleFS mount failed");
    } else {
        loadSchedule();
    }

    // ── TFT + LVGL ────────────────────────────────────────────────────────
    tft.init();
    tft.setRotation(1);     // landscape — USB connector on right
    tft.fillScreen(TFT_BLACK);

    touchscreenSPI.begin(XPT2046_CLK, XPT2046_MISO, XPT2046_MOSI, XPT2046_CS);
    touchscreen.begin(touchscreenSPI);
    touchscreen.setRotation(2);

    lv_init();

    lv_display_t* disp = lv_display_create(SCREEN_WIDTH, SCREEN_HEIGHT);
    lv_display_set_flush_cb(disp, disp_flush);
    lv_display_set_buffers(disp, draw_buf, nullptr, sizeof(draw_buf),
                           LV_DISPLAY_RENDER_MODE_PARTIAL);

    lv_indev_t* indev = lv_indev_create();
    lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(indev, touchscreen_read);

    buildClockScreen();
    lv_timer_create(onDisplayTimer, DISPLAY_UPDATE_MS, nullptr);

    // ── Synchronization primitives ────────────────────────────────────────
    clkMutex = xSemaphoreCreateMutex();

    // ── MQTT / WiFi ───────────────────────────────────────────────────────
    if (provisioned) {
        // LWT: publish offline status if we disconnect unexpectedly
        snprintf(lwtTopic,   sizeof(lwtTopic),   "trains/station/%s/status", cfg.sta_id);
        snprintf(lwtPayload, sizeof(lwtPayload),
                 "{\"online\":false,\"station_id\":\"%s\"}", cfg.sta_id);

        mqttReconTimer = xTimerCreate("mqttRecon", pdMS_TO_TICKS(2000), pdFALSE,
                                      nullptr, cbMqttRecon);
        wifiReconTimer = xTimerCreate("wifiRecon", pdMS_TO_TICKS(5000), pdFALSE,
                                      nullptr, cbWifiRecon);
        heartbeatTimer = xTimerCreate("heartbeat",
                                      pdMS_TO_TICKS(HEARTBEAT_SECS * 1000UL),
                                      pdTRUE, nullptr, cbHeartbeat);

        WiFi.onEvent(onWifiGotIP,     ARDUINO_EVENT_WIFI_STA_GOT_IP);
        WiFi.onEvent(onWifiDisconnect, ARDUINO_EVENT_WIFI_STA_DISCONNECTED);

        mqttClient.onConnect(onMqttConnect);
        mqttClient.onDisconnect(onMqttDisconnect);
        mqttClient.onMessage(onMqttMessage);

        IPAddress broker;
        broker.fromString(cfg.mqtt_ip);
        mqttClient.setServer(broker, MQTT_PORT);
        mqttClient.setCredentials(cfg.muser, cfg.mpas);
        mqttClient.setClientId(cfg.sta_id);
        mqttClient.setKeepAlive(30);
        mqttClient.setWill(lwtTopic, 1, true, lwtPayload);

        WiFi.setAutoReconnect(false);  // our wifiReconTimer handles reconnection
        WiFi.setSleep(false);          // disable modem sleep
        connectToWifi();
        xTimerStart(heartbeatTimer, 0);
    } else {
        Serial.println("Skipping network (not provisioned)");
    }
}

void loop() {
    handleSerialCLI();

    if (mqttReady) {
        if (needHeartbeat) {
            publishStatus();
            needHeartbeat = false;
        }
        if (needSyncRequest) {
            publishSyncRequest();
            needSyncRequest = false;
        }
    }

    lv_task_handler();
    lv_tick_inc(5);
    delay(5);
}
