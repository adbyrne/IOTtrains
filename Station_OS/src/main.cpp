/* Station_OS — CYD Station Display
 * NY&E Northern Lights Subdivision
 *
 * Subscribe: trains/clock/time        (retained, QoS 0)
 *            trains/clock/control     (QoS 1)
 *            trains/to/{station_id}   (QoS 2 — train orders from dispatcher)
 * Publish:   trains/station/{id}/status      (retained, QoS 1, LWT, 60 s heartbeat)
 *            trains/clock/sync_request       (QoS 0, on reconnect)
 *            trains/os/{id}                  (QoS 1, on OS submit)
 *            trains/to/{id}/ack              (QoS 1, on order ACK)
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
#include <map>
#include <string>
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
#define FIRMWARE_VER "2.2.0"
static const int  HEARTBEAT_SECS      = 60;
static const int  DISPLAY_UPDATE_MS   = 1000;
// Minutes before departure to consider a train "now" on display
static const int  NOW_WINDOW_MIN      = 3;
// OS entry inactivity timeout (ms) and next-station screen timeout (ms)
static const int  OS_INACT_MS         = 15000;
static const int  NS_TIMEOUT_MS       = 30000;

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
    int           day        = 1;
    bool          running    = false;
    bool          synced     = false;
    unsigned long tick_ms    = 0;   // millis() at last MQTT tick receipt
};

static ClockState         clk;
static SemaphoreHandle_t  clkMutex = nullptr;

// ── Schedule data ─────────────────────────────────────────────────────────
struct TrainEntry {
    char num[8];
    int  arrive;    // minutes since midnight, -1 if absent (origin stops)
    int  depart;    // minutes since midnight, -1 if absent (terminus stops)
};

struct StationData {
    char name[48];
    std::vector<TrainEntry> N;
    std::vector<TrainEntry> S;
};

static std::map<std::string, StationData> allSchedules;
static std::vector<std::string>           stationOrder;
// Pointers into allSchedules for the provisioned station (set in loadSchedule)
static const std::vector<TrainEntry>* schedN = nullptr;
static const std::vector<TrainEntry>* schedS = nullptr;
static char stationName[48] = "---";
static const std::vector<TrainEntry>  kEmptyVec;

// ── Station name table (for TO text rendering) ────────────────────────────
struct StationNameEntry { const char* id; const char* name; };
static const StationNameEntry STATION_NAME_TABLE[] = {
    {"WP", "Williamsport"},
    {"XP", "Xina Pass"},
    {"BB", "Becs Bend"},
    {"JC", "Jacks Creek"},
    {"MC", "Michelles Cove"},
    {"SK", "Stans Knob"},
    {"HC", "Hemlock Crest"},
    {nullptr, nullptr}
};

static const char* stationNameForId(const char* id) {
    for (int i = 0; STATION_NAME_TABLE[i].id; ++i)
        if (strcmp(STATION_NAME_TABLE[i].id, id) == 0)
            return STATION_NAME_TABLE[i].name;
    return id;
}

// ── Pending Train Orders ──────────────────────────────────────────────────
struct PendingTo {
    int  seq;
    char to_type[16];
    char trains[64];      // comma-separated train/engine numbers for matching
    char fields_json[400]; // fields object as JSON string
};

static std::vector<PendingTo>  pendingTos;
static SemaphoreHandle_t       toMutex     = nullptr;
static int                     currentToSeq = -1;  // seq of TO on ORDERS screen

// ── Screen state machine ──────────────────────────────────────────────────
enum class Screen { CLOCK, OS_ENTRY, ORDERS, NEXT_STATION };
static Screen      currentScreen = Screen::CLOCK;
static lv_obj_t*   clockScr      = nullptr;  // permanent clock screen
static lv_obj_t*   tempScr       = nullptr;  // OS / ORDERS / NS screen (deleted on transition)

// OS entry state (reset on each entry)
static char os_train[8]    = {};
static char os_dir[4]      = {};  // "N", "S", or "" (unset)
static bool os_extra        = false;
static bool os_work_extra   = false;

// OS entry LVGL widget refs (valid while OS screen is active)
static lv_obj_t*   os_lbl_num  = nullptr;
static lv_obj_t*   os_lbl_dir  = nullptr;
static lv_obj_t*   os_lbl_extra = nullptr;
static lv_obj_t*   os_btnm_ref = nullptr;

// LVGL inactivity / timeout timers
static lv_timer_t* osInactTimer  = nullptr;
static lv_timer_t* nsTimeoutTimer = nullptr;

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
static void publishSyncRequest();
static void publishOs();
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
static void enterClock();
static void enterOsEntry();
static void updateOsDisplay();
static lv_obj_t* buildOsScreen();
static lv_obj_t* buildNextStationScreen(const char* train_num, const char* direction);
static lv_obj_t* buildOrdersScreen(const PendingTo& to);
static void checkAndShowNextTo();
static void publishToAck(int seq);
static void onClockTouch(lv_event_t* e);
static void onOsBtnm(lv_event_t* e);
static void onOsBtnmRelease(lv_event_t* e);
static void onNsTouch(lv_event_t* e);
static void onOrdersAck(lv_event_t* e);
static void cbOsInact(lv_timer_t* t);
static void cbNsTimeout(lv_timer_t* t);

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

    // Load station order
    for (JsonVariant v : doc["station_order"].as<JsonArray>()) {
        stationOrder.push_back(v.as<std::string>());
    }

    // Load all stations
    for (const std::string& sid : stationOrder) {
        if (!doc[sid.c_str()].is<JsonObject>()) continue;
        StationData sd;
        strlcpy(sd.name, doc[sid.c_str()]["name"] | "---", sizeof(sd.name));
        for (JsonObject e : doc[sid.c_str()]["N"].as<JsonArray>()) {
            TrainEntry te;
            strlcpy(te.num, e["num"] | "?", sizeof(te.num));
            te.arrive = e["arrive"] | -1;
            te.depart = e["depart"] | -1;
            sd.N.push_back(te);
        }
        for (JsonObject e : doc[sid.c_str()]["S"].as<JsonArray>()) {
            TrainEntry te;
            strlcpy(te.num, e["num"] | "?", sizeof(te.num));
            te.arrive = e["arrive"] | -1;
            te.depart = e["depart"] | -1;
            sd.S.push_back(te);
        }
        allSchedules[sid] = std::move(sd);
    }

    // Set current station name and schedule pointers
    const std::string staId(cfg.sta_id);
    auto it = allSchedules.find(staId);
    if (it != allSchedules.end()) {
        strlcpy(stationName, it->second.name, sizeof(stationName));
        schedN = &it->second.N;
        schedS = &it->second.S;
    } else {
        Serial.printf("Station %s not found in schedule.json\n", cfg.sta_id);
    }

    Serial.printf("Loaded schedule: %d stations, %s: %d NB %d SB\n",
                  stationOrder.size(), cfg.sta_id,
                  schedN ? (int)schedN->size() : 0,
                  schedS ? (int)schedS->size() : 0);
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

// Fills out[] with the next arriving train for the time screen.
// Comparison uses depart (keeps a holding train visible until it leaves).
// Display uses arrive; falls back to depart for origin-only stops.
// If cur_min == -1 (not synced), writes "---".
static void findNextTrain(const std::vector<TrainEntry>& sched, int cur_min,
                          char* out, size_t n) {
    if (sched.empty()) {
        snprintf(out, n, "  \xe2\x94\x80\xe2\x94\x80\xe2\x94\x80\xe2\x94\x80\xe2\x94\x80");
        return;
    }
    if (cur_min < 0) {
        snprintf(out, n, "  ---");
        return;
    }

    // Comparison key: depart if available, else arrive
    const TrainEntry* found = nullptr;
    for (const auto& e : sched) {
        int cmp = (e.depart >= 0) ? e.depart : e.arrive;
        if (cmp >= cur_min) { found = &e; break; }
    }
    if (!found) found = &sched[0];

    // Display time: arrive for time screen; fall back to depart for origin stops
    int disp_min = (found->arrive >= 0) ? found->arrive : found->depart;
    if (disp_min < 0) {
        snprintf(out, n, "  No.%-3s  --:--", found->num);
        return;
    }
    char tstr[12];
    int h = disp_min / 60, m = disp_min % 60;
    const char* ap = (h < 12) ? "AM" : "PM";
    int h12 = h % 12; if (h12 == 0) h12 = 12;
    snprintf(tstr, sizeof(tstr), "%d:%02d %s", h12, m, ap);

    bool isNow = (disp_min >= cur_min && disp_min - cur_min <= NOW_WINDOW_MIN);
    if (isNow)
        snprintf(out, n, "  No.%-3s  NOW", found->num);
    else
        snprintf(out, n, "  No.%-3s  %s", found->num, tstr);
}

// ═══════════════════════════════════════════════════════════════════════════
// LVGL display — clock screen
// ═══════════════════════════════════════════════════════════════════════════

static void touchscreen_read(lv_indev_t* indev, lv_indev_data_t* data) {
    if (touchscreen.tirqTouched() && touchscreen.touched()) {
        TS_Point p = touchscreen.getPoint();
        // Axes are physically swapped on this CYD unit in landscape (rotation=2):
        // rawX (p.x) drives screen Y; rawY (p.y) drives screen X.
        // Coefficients derived from 6-point affine calibration against button grid.
        int xi = (int)(344.3f - 0.0827f * p.y);   // inverted: left rawY → right screen X
        int yi = (int)(0.0666f * p.x - 16.4f);
        data->point.x = constrain(xi, 0, SCREEN_WIDTH  - 1);
        data->point.y = constrain(yi, 0, SCREEN_HEIGHT - 1);
        data->state   = LV_INDEV_STATE_PRESSED;
    } else {
        data->state = LV_INDEV_STATE_RELEASED;
    }
}

static void onClockTouch(lv_event_t* e) {
    if (currentScreen == Screen::CLOCK)
        enterOsEntry();
}

static void buildClockScreen() {
    clockScr = lv_screen_active();
    lv_obj_set_style_bg_color(clockScr, lv_color_black(), LV_PART_MAIN);
    lv_obj_remove_flag(clockScr, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_pad_all(clockScr, 0, LV_PART_MAIN);
    lv_obj_add_flag(clockScr, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(clockScr, onClockTouch, LV_EVENT_CLICKED, nullptr);

    // Station name — top left
    lbl_station = lv_label_create(clockScr);
    lv_label_set_text(lbl_station, stationName);
    lv_obj_set_style_text_font(lbl_station, &lv_font_montserrat_20, LV_PART_MAIN);
    lv_obj_set_style_text_color(lbl_station, lv_palette_main(LV_PALETTE_YELLOW), LV_PART_MAIN);
    lv_obj_align(lbl_station, LV_ALIGN_TOP_LEFT, 8, 8);

    // MQTT status dot — top right
    dot_status = lv_obj_create(clockScr);
    lv_obj_set_size(dot_status, 12, 12);
    lv_obj_set_style_radius(dot_status, LV_RADIUS_CIRCLE, LV_PART_MAIN);
    lv_obj_set_style_border_width(dot_status, 0, LV_PART_MAIN);
    lv_obj_set_style_bg_color(dot_status, lv_palette_main(LV_PALETTE_RED), LV_PART_MAIN);
    lv_obj_align(dot_status, LV_ALIGN_TOP_RIGHT, -8, 12);

    // Railroad time — large, center
    lbl_time = lv_label_create(clockScr);
    lv_label_set_text(lbl_time, "--:-- --");
    lv_obj_set_style_text_font(lbl_time, &lv_font_montserrat_48, LV_PART_MAIN);
    lv_obj_set_style_text_color(lbl_time, lv_color_white(), LV_PART_MAIN);
    lv_obj_align(lbl_time, LV_ALIGN_CENTER, 0, -20);

    // Divider line
    lv_obj_t* line = lv_obj_create(clockScr);
    lv_obj_set_size(line, 300, 2);
    lv_obj_set_style_bg_color(line, lv_palette_darken(LV_PALETTE_GREY, 2), LV_PART_MAIN);
    lv_obj_set_style_border_width(line, 0, LV_PART_MAIN);
    lv_obj_set_style_radius(line, 0, LV_PART_MAIN);
    lv_obj_align(line, LV_ALIGN_CENTER, 0, 45);

    // Next northbound
    lbl_nextN = lv_label_create(clockScr);
    lv_label_set_text(lbl_nextN, "NB:  ---");
    lv_obj_set_style_text_font(lbl_nextN, &lv_font_montserrat_16, LV_PART_MAIN);
    lv_obj_set_style_text_color(lbl_nextN, lv_palette_lighten(LV_PALETTE_GREEN, 2), LV_PART_MAIN);
    lv_obj_align(lbl_nextN, LV_ALIGN_CENTER, 0, 65);

    // Next southbound
    lbl_nextS = lv_label_create(clockScr);
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
    findNextTrain(schedN ? *schedN : kEmptyVec, cur_min, nbuf, sizeof(nbuf));
    findNextTrain(schedS ? *schedS : kEmptyVec, cur_min, sbuf, sizeof(sbuf));

    char full[48];
    snprintf(full, sizeof(full), "NB:%s", nbuf);
    lv_label_set_text(lbl_nextN, full);
    snprintf(full, sizeof(full), "SB:%s", sbuf);
    lv_label_set_text(lbl_nextS, full);
}

static void onDisplayTimer(lv_timer_t* timer) {
    if (currentScreen != Screen::CLOCK) return;
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
// LVGL display — OS entry screen
// ═══════════════════════════════════════════════════════════════════════════

/*
 * OS entry button map:
 *  col:  0      1      2      3
 *  row0: 1      2      3      N     (indices 0–3)
 *  row1: 4      5      6      S     (indices 4–7)
 *  row2: 7      8      9      X     (indices 8–11)
 *  row3: ⌫     0      ✓      WX    (indices 12–15)
 */
static const char* kOsBtnMap[] = {
    "1", "2", "3", "N", "\n",
    "4", "5", "6", "S", "\n",
    "7", "8", "9", "X", "\n",
    LV_SYMBOL_BACKSPACE, "0", LV_SYMBOL_OK, "WX", ""
};
static const int OS_BTN_N  = 3;
static const int OS_BTN_S  = 7;
static const int OS_BTN_X  = 11;
static const int OS_BTN_WX = 15;
static const int OS_BTN_OK = 14;

static void updateOsDisplay() {
    if (!os_lbl_num || !os_lbl_dir || !os_lbl_extra || !os_btnm_ref) return;

    // Train number
    char nbuf[16];
    if (os_train[0])
        snprintf(nbuf, sizeof(nbuf), "No. %s", os_train);
    else
        strlcpy(nbuf, "No. --", sizeof(nbuf));
    lv_label_set_text(os_lbl_num, nbuf);

    // Direction badge
    if (strcmp(os_dir, "N") == 0) {
        lv_label_set_text(os_lbl_dir, "[N]");
        lv_obj_set_style_text_color(os_lbl_dir, lv_palette_main(LV_PALETTE_GREEN), LV_PART_MAIN);
    } else if (strcmp(os_dir, "S") == 0) {
        lv_label_set_text(os_lbl_dir, "[S]");
        lv_obj_set_style_text_color(os_lbl_dir, lv_palette_main(LV_PALETTE_CYAN), LV_PART_MAIN);
    } else {
        lv_label_set_text(os_lbl_dir, "[ ]");
        lv_obj_set_style_text_color(os_lbl_dir, lv_color_hex(0x555555), LV_PART_MAIN);
    }

    // Extra badge
    if (os_work_extra) {
        lv_label_set_text(os_lbl_extra, "WX");
        lv_obj_set_style_text_color(os_lbl_extra, lv_palette_main(LV_PALETTE_AMBER), LV_PART_MAIN);
    } else if (os_extra) {
        lv_label_set_text(os_lbl_extra, "X");
        lv_obj_set_style_text_color(os_lbl_extra, lv_palette_main(LV_PALETTE_AMBER), LV_PART_MAIN);
    } else {
        lv_label_set_text(os_lbl_extra, "");
    }

    // Enable/disable ✓ button — green when valid, grey when not
    bool valid = (os_train[0] != '\0') && (os_dir[0] != '\0');
    if (valid) {
        lv_buttonmatrix_clear_button_ctrl(os_btnm_ref, OS_BTN_OK, LV_BUTTONMATRIX_CTRL_DISABLED);
        lv_buttonmatrix_set_button_ctrl(os_btnm_ref, OS_BTN_OK, LV_BUTTONMATRIX_CTRL_CHECKED);
    } else {
        lv_buttonmatrix_clear_button_ctrl(os_btnm_ref, OS_BTN_OK, LV_BUTTONMATRIX_CTRL_CHECKED);
        lv_buttonmatrix_set_button_ctrl(os_btnm_ref, OS_BTN_OK, LV_BUTTONMATRIX_CTRL_DISABLED);
    }
}

static bool s_btnmInPress = false;

static void onOsBtnmRelease(lv_event_t* e) {
    s_btnmInPress = false;
}

static void onOsBtnm(lv_event_t* e) {
    if (currentScreen != Screen::OS_ENTRY) return;
    if (s_btnmInPress) return;   // suppress drift events after first selection
    s_btnmInPress = true;

    lv_obj_t* btnm = (lv_obj_t*)lv_event_get_target(e);
    uint32_t  id   = lv_buttonmatrix_get_selected_button(btnm);
    const char* txt = lv_buttonmatrix_get_button_text(btnm, id);
    if (!txt) return;

    // Reset inactivity timer on every keypress
    if (osInactTimer) lv_timer_reset(osInactTimer);

    if (strcmp(txt, "N") == 0) {
        strlcpy(os_dir, "N", sizeof(os_dir));
        lv_buttonmatrix_set_button_ctrl(btnm, OS_BTN_N, LV_BUTTONMATRIX_CTRL_CHECKED);
        lv_buttonmatrix_clear_button_ctrl(btnm, OS_BTN_S, LV_BUTTONMATRIX_CTRL_CHECKED);

    } else if (strcmp(txt, "S") == 0) {
        strlcpy(os_dir, "S", sizeof(os_dir));
        lv_buttonmatrix_set_button_ctrl(btnm, OS_BTN_S, LV_BUTTONMATRIX_CTRL_CHECKED);
        lv_buttonmatrix_clear_button_ctrl(btnm, OS_BTN_N, LV_BUTTONMATRIX_CTRL_CHECKED);

    } else if (strcmp(txt, "X") == 0) {
        os_extra = !os_extra;
        if (os_extra) {
            os_work_extra = false;
            lv_buttonmatrix_set_button_ctrl(btnm, OS_BTN_X, LV_BUTTONMATRIX_CTRL_CHECKED);
            lv_buttonmatrix_clear_button_ctrl(btnm, OS_BTN_WX, LV_BUTTONMATRIX_CTRL_CHECKED);
        } else {
            lv_buttonmatrix_clear_button_ctrl(btnm, OS_BTN_X, LV_BUTTONMATRIX_CTRL_CHECKED);
        }

    } else if (strcmp(txt, "WX") == 0) {
        os_work_extra = !os_work_extra;
        if (os_work_extra) {
            os_extra = false;
            lv_buttonmatrix_set_button_ctrl(btnm, OS_BTN_WX, LV_BUTTONMATRIX_CTRL_CHECKED);
            lv_buttonmatrix_clear_button_ctrl(btnm, OS_BTN_X, LV_BUTTONMATRIX_CTRL_CHECKED);
        } else {
            lv_buttonmatrix_clear_button_ctrl(btnm, OS_BTN_WX, LV_BUTTONMATRIX_CTRL_CHECKED);
        }

    } else if (strcmp(txt, LV_SYMBOL_BACKSPACE) == 0) {
        size_t len = strlen(os_train);
        if (len > 0) os_train[len - 1] = '\0';

    } else if (strcmp(txt, LV_SYMBOL_OK) == 0) {
        // Only reachable when valid (button was enabled)
        if (os_train[0] != '\0' && os_dir[0] != '\0') {
            // Stop inactivity timer
            if (osInactTimer) { lv_timer_delete(osInactTimer); osInactTimer = nullptr; }
            // Publish OS report
            publishOs();
            // Check for a pending TO before proceeding to next-station screen
            lv_obj_t* oldScr = tempScr;
            tempScr = nullptr;
            lv_screen_load(clockScr);   // briefly show clock while building next screen
            lv_obj_delete_async(oldScr);
            checkAndShowNextTo();
        }
        return;

    } else {
        // Digit key
        size_t len = strlen(os_train);
        if (len < 4) {
            os_train[len]     = txt[0];
            os_train[len + 1] = '\0';
        }
    }

    updateOsDisplay();
}

// ═══════════════════════════════════════════════════════════════════════════
// Train Order rendering + ORDERS screen
// ═══════════════════════════════════════════════════════════════════════════

static void renderToText(const PendingTo& to, char* out, size_t n) {
    JsonDocument doc;
    deserializeJson(doc, to.fields_json);

    auto sn = [&](const char* key) -> const char* {
        const char* id = doc[key] | "";
        return stationNameForId(id);
    };
    auto dw = [&](const char* key) -> const char* {
        const char* d = doc[key] | "";
        return (strcmp(d, "N") == 0) ? "North" : (strcmp(d, "S") == 0) ? "South" : d;
    };

    if (strcmp(to.to_type, "meet") == 0) {
        bool bExtra = doc["train_b_is_extra"] | false;
        char bRef[64];
        if (bExtra)
            snprintf(bRef, sizeof(bRef), "Extra %s %s",
                     doc["train_b"] | "?", dw("direction_b"));
        else
            snprintf(bRef, sizeof(bRef), "No. %s Eng %s",
                     doc["train_b"] | "?", doc["engine_b"] | "?");
        snprintf(out, n,
            "No. %s Eng %s take siding at %s and wait for %s %s.",
            doc["train_a"] | "?", doc["engine_a"] | "?",
            sn("station"), bRef, dw("direction_b"));

    } else if (strcmp(to.to_type, "wait") == 0) {
        snprintf(out, n,
            "No. %s Eng %s take siding at %s and wait until %s.",
            doc["train"] | "?", doc["engine"] | "?",
            sn("station"), doc["until_time"] | "?");

    } else if (strcmp(to.to_type, "running_extra") == 0) {
        snprintf(out, n,
            "Engine %s run extra %s %s to %s.",
            doc["engine"] | "?", dw("direction"),
            sn("from_station"), sn("to_station"));

    } else if (strcmp(to.to_type, "work_extra") == 0) {
        snprintf(out, n,
            "Engine %s is authorized to work between %s and %s from %s to %s."
            " All trains must be clear of this section during this time.",
            doc["engine"] | "?",
            sn("from_station"), sn("to_station"),
            doc["start_rr_time"] | "?", doc["end_rr_time"] | "?");

    } else if (strcmp(to.to_type, "annulment") == 0) {
        const char* fromSta = doc["from_station"] | "";
        const char* toSta   = doc["to_station"] | "";
        if (fromSta[0] && toSta[0])
            snprintf(out, n, "No. %s is annulled %s to %s.",
                     doc["train"] | "?",
                     stationNameForId(fromSta), stationNameForId(toSta));
        else
            snprintf(out, n, "No. %s is annulled.", doc["train"] | "?");

    } else if (strcmp(to.to_type, "sections") == 0) {
        snprintf(out, n,
            "No. %s Eng %s will run in %d sections.",
            doc["train"] | "?", doc["engine"] | "?",
            doc["section_count"] | 2);

    } else {
        strlcpy(out, "(unknown order type)", n);
    }
}

static void publishToAck(int seq) {
    if (!mqttReady || cfg.sta_id[0] == '\0') return;
    char topic[48];
    snprintf(topic, sizeof(topic), "trains/to/%s/ack", cfg.sta_id);
    // Get current RR time
    char rrTime[12];
    xSemaphoreTake(clkMutex, portMAX_DELAY);
    int total = clk.base_hour * 60 + clk.base_min;
    if (clk.synced && clk.running) {
        unsigned long elapsed_ms = millis() - clk.tick_ms;
        total += (int)((float)elapsed_ms / 1000.0f * clk.speed / 60.0f);
    }
    xSemaphoreGive(clkMutex);
    total %= 1440;
    snprintf(rrTime, sizeof(rrTime), "%02d:%02d", total / 60, total % 60);

    char payload[128];
    snprintf(payload, sizeof(payload),
             "{\"seq\":%d,\"station_id\":\"%s\",\"rr_time\":\"%s\",\"copies\":2}",
             seq, cfg.sta_id, rrTime);
    mqttClient.publish(topic, 1, false, payload);
    Serial.printf("TO #%d ACK published\n", seq);
}

static void onOrdersAck(lv_event_t* e) {
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;

    int seqToAck = currentToSeq;
    publishToAck(seqToAck);

    // Remove the ACK'd TO from the queue
    xSemaphoreTake(toMutex, portMAX_DELAY);
    for (auto it = pendingTos.begin(); it != pendingTos.end(); ++it) {
        if (it->seq == seqToAck) {
            pendingTos.erase(it);
            break;
        }
    }
    xSemaphoreGive(toMutex);

    // Tear down current ORDERS screen before building next
    lv_obj_t* oldScr = tempScr;
    tempScr = nullptr;
    lv_screen_load(clockScr);
    lv_obj_delete_async(oldScr);
    currentToSeq = -1;

    // Show next pending TO for the same train, or advance to NEXT_STATION
    checkAndShowNextTo();
}

static lv_obj_t* buildOrdersScreen(const PendingTo& to) {
    char toText[480];
    renderToText(to, toText, sizeof(toText));

    lv_obj_t* scr = lv_obj_create(nullptr);
    lv_obj_set_style_bg_color(scr, lv_color_black(), LV_PART_MAIN);
    lv_obj_set_style_border_width(scr, 0, LV_PART_MAIN);
    lv_obj_remove_flag(scr, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_pad_all(scr, 0, LV_PART_MAIN);

    // Header: "Form 19  #seq"
    char header[32];
    snprintf(header, sizeof(header), "Form 19   #%d", to.seq);
    lv_obj_t* lbl_hdr = lv_label_create(scr);
    lv_label_set_text(lbl_hdr, header);
    lv_obj_set_style_text_font(lbl_hdr, &lv_font_montserrat_14, LV_PART_MAIN);
    lv_obj_set_style_text_color(lbl_hdr, lv_color_hex(0x888888), LV_PART_MAIN);
    lv_obj_align(lbl_hdr, LV_ALIGN_TOP_LEFT, 10, 8);

    // TO text — large, wrapped, yellow
    lv_obj_t* lbl_text = lv_label_create(scr);
    lv_label_set_long_mode(lbl_text, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(lbl_text, SCREEN_WIDTH - 20);
    lv_label_set_text(lbl_text, toText);
    lv_obj_set_style_text_font(lbl_text, &lv_font_montserrat_16, LV_PART_MAIN);
    lv_obj_set_style_text_color(lbl_text, lv_color_hex(0xFFDD44), LV_PART_MAIN);
    lv_obj_align(lbl_text, LV_ALIGN_TOP_LEFT, 10, 30);

    // ACK button — full width at bottom, green
    lv_obj_t* btn = lv_button_create(scr);
    lv_obj_set_size(btn, SCREEN_WIDTH - 20, 44);
    lv_obj_align(btn, LV_ALIGN_BOTTOM_MID, 0, -8);
    lv_obj_set_style_bg_color(btn, lv_color_hex(0x1b4020), LV_PART_MAIN);
    lv_obj_set_style_border_color(btn, lv_color_hex(0x2d6b35), LV_PART_MAIN);
    lv_obj_set_style_border_width(btn, 1, LV_PART_MAIN);
    lv_obj_t* lbl_ack = lv_label_create(btn);
    lv_label_set_text(lbl_ack, "ACK — Order Received");
    lv_obj_set_style_text_font(lbl_ack, &lv_font_montserrat_16, LV_PART_MAIN);
    lv_obj_set_style_text_color(lbl_ack, lv_color_hex(0x7dcc85), LV_PART_MAIN);
    lv_obj_center(lbl_ack);
    lv_obj_add_event_cb(btn, onOrdersAck, LV_EVENT_CLICKED, nullptr);

    return scr;
}

// Check pendingTos for a TO matching the current OS train; show ORDERS if found,
// else advance to NEXT_STATION.  Called from OS submit and after each ACK.
static void checkAndShowNextTo() {
    xSemaphoreTake(toMutex, portMAX_DELAY);
    PendingTo* match = nullptr;
    for (auto& pt : pendingTos) {
        // Check if os_train appears in the comma-separated trains[] string
        char needle[12];
        snprintf(needle, sizeof(needle), "%s", os_train);
        // Simple search: match whole token
        char haystack[64];
        strlcpy(haystack, pt.trains, sizeof(haystack));
        bool found = false;
        char* tok = strtok(haystack, ",");
        while (tok) {
            if (strcmp(tok, needle) == 0) { found = true; break; }
            tok = strtok(nullptr, ",");
        }
        if (found) { match = &pt; break; }
    }

    if (match) {
        PendingTo matchCopy = *match;   // copy before releasing mutex
        currentToSeq = matchCopy.seq;
        xSemaphoreGive(toMutex);
        tempScr = buildOrdersScreen(matchCopy);
        lv_screen_load(tempScr);
        currentScreen = Screen::ORDERS;
    } else {
        xSemaphoreGive(toMutex);
        // No matching TO — go directly to next-station screen
        tempScr = buildNextStationScreen(os_train, os_dir);
        lv_screen_load(tempScr);
        currentScreen = Screen::NEXT_STATION;
        nsTimeoutTimer = lv_timer_create(cbNsTimeout, NS_TIMEOUT_MS, nullptr);
    }
}

static lv_obj_t* buildOsScreen() {
    lv_obj_t* scr = lv_obj_create(nullptr);
    lv_obj_set_style_bg_color(scr, lv_color_black(), LV_PART_MAIN);
    lv_obj_set_style_border_width(scr, 0, LV_PART_MAIN);
    lv_obj_remove_flag(scr, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_pad_all(scr, 0, LV_PART_MAIN);

    // ── Display area (top 68px) ──────────────────────────────────────────
    // Train number — left
    os_lbl_num = lv_label_create(scr);
    lv_label_set_text(os_lbl_num, "No. --");
    lv_obj_set_style_text_font(os_lbl_num, &lv_font_montserrat_20, LV_PART_MAIN);
    lv_obj_set_style_text_color(os_lbl_num, lv_color_white(), LV_PART_MAIN);
    lv_obj_align(os_lbl_num, LV_ALIGN_TOP_LEFT, 10, 12);

    // Direction badge — right column top
    os_lbl_dir = lv_label_create(scr);
    lv_label_set_text(os_lbl_dir, "[ ]");
    lv_obj_set_style_text_font(os_lbl_dir, &lv_font_montserrat_20, LV_PART_MAIN);
    lv_obj_set_style_text_color(os_lbl_dir, lv_color_hex(0x555555), LV_PART_MAIN);
    lv_obj_align(os_lbl_dir, LV_ALIGN_TOP_RIGHT, -10, 12);

    // Extra badge — right column below direction
    os_lbl_extra = lv_label_create(scr);
    lv_label_set_text(os_lbl_extra, "");
    lv_obj_set_style_text_font(os_lbl_extra, &lv_font_montserrat_14, LV_PART_MAIN);
    lv_obj_align(os_lbl_extra, LV_ALIGN_TOP_RIGHT, -10, 40);

    // Divider line
    lv_obj_t* div = lv_obj_create(scr);
    lv_obj_set_size(div, 320, 2);
    lv_obj_set_style_bg_color(div, lv_palette_darken(LV_PALETTE_GREY, 2), LV_PART_MAIN);
    lv_obj_set_style_border_width(div, 0, LV_PART_MAIN);
    lv_obj_set_style_radius(div, 0, LV_PART_MAIN);
    lv_obj_set_pos(div, 0, 68);

    // ── Buttonmatrix (y=70 to y=240) ─────────────────────────────────────
    os_btnm_ref = lv_buttonmatrix_create(scr);
    lv_buttonmatrix_set_map(os_btnm_ref, kOsBtnMap);
    lv_obj_set_pos(os_btnm_ref, 0, 70);
    lv_obj_set_size(os_btnm_ref, 320, 170);
    lv_obj_set_style_border_width(os_btnm_ref, 0, LV_PART_MAIN);
    lv_obj_set_style_bg_color(os_btnm_ref, lv_color_black(), LV_PART_MAIN);
    lv_obj_set_style_pad_all(os_btnm_ref, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_gap(os_btnm_ref, 3, LV_PART_MAIN);
    // Button item style
    lv_obj_set_style_bg_color(os_btnm_ref, lv_color_hex(0x2a2a2a), LV_PART_ITEMS);
    lv_obj_set_style_text_color(os_btnm_ref, lv_color_white(), LV_PART_ITEMS);
    lv_obj_set_style_text_font(os_btnm_ref, &lv_font_montserrat_20, LV_PART_ITEMS);
    lv_obj_set_style_radius(os_btnm_ref, 4, LV_PART_ITEMS);
    // Checked item style — used by N/S/X/WX (selected) and ✓ (valid/enabled)
    lv_obj_set_style_bg_color(os_btnm_ref, lv_color_hex(0x1b4020), LV_PART_ITEMS | LV_STATE_CHECKED);
    lv_obj_set_style_text_color(os_btnm_ref, lv_color_hex(0x7dcc85), LV_PART_ITEMS | LV_STATE_CHECKED);

    // ✓ starts disabled until train# and direction are both set
    lv_buttonmatrix_set_button_ctrl(os_btnm_ref, OS_BTN_OK, LV_BUTTONMATRIX_CTRL_DISABLED);

    lv_obj_add_event_cb(os_btnm_ref, onOsBtnm, LV_EVENT_VALUE_CHANGED, nullptr);
    lv_obj_add_event_cb(os_btnm_ref, onOsBtnmRelease, LV_EVENT_RELEASED, nullptr);

    return scr;
}

// ═══════════════════════════════════════════════════════════════════════════
// LVGL display — next-station screen
// ═══════════════════════════════════════════════════════════════════════════

static void onNsTouch(lv_event_t* e) {
    enterClock();
}

static lv_obj_t* buildNextStationScreen(const char* train_num, const char* direction) {
    lv_obj_t* scr = lv_obj_create(nullptr);
    lv_obj_set_style_bg_color(scr, lv_color_black(), LV_PART_MAIN);
    lv_obj_set_style_border_width(scr, 0, LV_PART_MAIN);
    lv_obj_remove_flag(scr, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_pad_all(scr, 0, LV_PART_MAIN);
    lv_obj_add_flag(scr, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(scr, onNsTouch, LV_EVENT_CLICKED, nullptr);

    bool go_north = (strcmp(direction, "N") == 0);

    // Current RR time — for finding next opposing train
    xSemaphoreTake(clkMutex, portMAX_DELAY);
    ClockState clkSnap = clk;
    xSemaphoreGive(clkMutex);
    int cur_rr_min = -1;
    if (clkSnap.synced) {
        int total = clkSnap.base_hour * 60 + clkSnap.base_min;
        if (clkSnap.running) {
            unsigned long elapsed_ms = millis() - clkSnap.tick_ms;
            total += (int)((float)elapsed_ms / 1000.0f * clkSnap.speed / 60.0f);
        }
        cur_rr_min = total % 1440;
    }

    // Determine next station and build two info lines
    bool end_of_line = false;
    char ns_name[48] = {};
    // Opposing train departing from next station toward current station
    char line2[48]   = {};

    auto fmtMin = [](int t, char* buf, size_t n) {
        if (t < 0) { strlcpy(buf, "--:-- --", n); return; }
        int h = t / 60, m = t % 60;
        const char* ap = (h < 12) ? "AM" : "PM";
        int h12 = h % 12; if (h12 == 0) h12 = 12;
        snprintf(buf, n, "%d:%02d %s", h12, m, ap);
    };

    int cur_idx = -1;
    for (int i = 0; i < (int)stationOrder.size(); i++) {
        if (stationOrder[i] == cfg.sta_id) { cur_idx = i; break; }
    }

    if (cur_idx >= 0) {
        int next_idx = cur_idx + (go_north ? 1 : -1);
        if (next_idx < 0 || next_idx >= (int)stationOrder.size()) {
            end_of_line = true;
        } else {
            const std::string& next_id = stationOrder[next_idx];
            auto it = allSchedules.find(next_id);
            if (it != allSchedules.end()) {
                strlcpy(ns_name, it->second.name, sizeof(ns_name));

                        // Opposing train from next station (wraps to first if past last)
                if (cur_rr_min >= 0) {
                    const std::vector<TrainEntry>& opp = go_north ? it->second.S : it->second.N;
                    if (!opp.empty()) {
                        const TrainEntry* found = nullptr;
                        for (const auto& oe : opp) {
                            int cmp = (oe.depart >= 0) ? oe.depart : oe.arrive;
                            if (cmp >= cur_rr_min) { found = &oe; break; }
                        }
                        if (!found) found = &opp[0];  // wrap to first train of day
                        char tstr[16];
                        int t = (found->depart >= 0) ? found->depart : found->arrive;
                        fmtMin(t, tstr, sizeof(tstr));
                        snprintf(line2, sizeof(line2), "No. %s    Dp  %s", found->num, tstr);
                    }
                }
            } else {
                strlcpy(ns_name, next_id.c_str(), sizeof(ns_name));
            }
        }
    } else {
        strlcpy(ns_name, "Unknown", sizeof(ns_name));
    }

    // ── Direction label ──────────────────────────────────────────────────
    lv_obj_t* lbl_dir = lv_label_create(scr);
    lv_label_set_text(lbl_dir, go_north ? "NORTHBOUND" : "SOUTHBOUND");
    lv_obj_set_style_text_font(lbl_dir, &lv_font_montserrat_20, LV_PART_MAIN);
    lv_obj_set_style_text_color(lbl_dir,
        go_north ? lv_palette_main(LV_PALETTE_GREEN)
                 : lv_palette_main(LV_PALETTE_CYAN),
        LV_PART_MAIN);
    lv_obj_align(lbl_dir, LV_ALIGN_TOP_MID, 0, 12);
    lv_obj_add_flag(lbl_dir, LV_OBJ_FLAG_EVENT_BUBBLE);

    if (end_of_line) {
        lv_obj_t* lbl_eol = lv_label_create(scr);
        lv_label_set_text(lbl_eol, "End of line");
        lv_obj_set_style_text_font(lbl_eol, &lv_font_montserrat_20, LV_PART_MAIN);
        lv_obj_set_style_text_color(lbl_eol, lv_color_white(), LV_PART_MAIN);
        lv_obj_align(lbl_eol, LV_ALIGN_CENTER, 0, 0);
        lv_obj_add_flag(lbl_eol, LV_OBJ_FLAG_EVENT_BUBBLE);
    } else {
        // Station name
        lv_obj_t* lbl_sta = lv_label_create(scr);
        lv_label_set_text(lbl_sta, ns_name);
        lv_obj_set_style_text_font(lbl_sta, &lv_font_montserrat_20, LV_PART_MAIN);
        lv_obj_set_style_text_color(lbl_sta, lv_color_white(), LV_PART_MAIN);
        lv_obj_align(lbl_sta, LV_ALIGN_CENTER, 0, -20);
        lv_obj_add_flag(lbl_sta, LV_OBJ_FLAG_EVENT_BUBBLE);

        // Opposing train (cyan if going N, green if going S)
        if (line2[0]) {
            lv_obj_t* lbl_l2 = lv_label_create(scr);
            lv_label_set_text(lbl_l2, line2);
            lv_obj_set_style_text_font(lbl_l2, &lv_font_montserrat_16, LV_PART_MAIN);
            lv_obj_set_style_text_color(lbl_l2,
                go_north ? lv_palette_lighten(LV_PALETTE_CYAN, 1)
                         : lv_palette_lighten(LV_PALETTE_GREEN, 1),
                LV_PART_MAIN);
            lv_obj_align(lbl_l2, LV_ALIGN_CENTER, 0, +20);
            lv_obj_add_flag(lbl_l2, LV_OBJ_FLAG_EVENT_BUBBLE);
        }
    }

    // "touch to return" hint
    lv_obj_t* lbl_hint = lv_label_create(scr);
    lv_label_set_text(lbl_hint, "touch to return to clock");
    lv_obj_set_style_text_font(lbl_hint, &lv_font_montserrat_8, LV_PART_MAIN);
    lv_obj_set_style_text_color(lbl_hint, lv_color_hex(0x555555), LV_PART_MAIN);
    lv_obj_align(lbl_hint, LV_ALIGN_BOTTOM_MID, 0, -6);
    lv_obj_add_flag(lbl_hint, LV_OBJ_FLAG_EVENT_BUBBLE);

    return scr;
}

// ═══════════════════════════════════════════════════════════════════════════
// Screen transitions
// ═══════════════════════════════════════════════════════════════════════════

static void cbOsInact(lv_timer_t* t) {
    lv_timer_delete(t);
    osInactTimer = nullptr;
    enterClock();
}

static void cbNsTimeout(lv_timer_t* t) {
    lv_timer_delete(t);
    nsTimeoutTimer = nullptr;
    enterClock();
}

static void enterClock() {
    if (osInactTimer)   { lv_timer_delete(osInactTimer);  osInactTimer = nullptr; }
    if (nsTimeoutTimer) { lv_timer_delete(nsTimeoutTimer); nsTimeoutTimer = nullptr; }
    lv_obj_t* oldScr = tempScr;
    tempScr = nullptr;
    // Null out OS widget refs before deleting their screen
    os_lbl_num = os_lbl_dir = os_lbl_extra = nullptr;
    os_btnm_ref = nullptr;
    lv_screen_load(clockScr);
    if (oldScr) lv_obj_delete_async(oldScr);
    currentScreen = Screen::CLOCK;
}

static void enterOsEntry() {
    os_train[0]   = '\0';
    os_dir[0]     = '\0';
    os_extra      = false;
    os_work_extra = false;
    s_btnmInPress = false;  // clear any stuck debounce from previous screen

    lv_obj_t* oldScr = tempScr;
    tempScr = buildOsScreen();
    lv_screen_load(tempScr);
    if (oldScr) lv_obj_delete_async(oldScr);
    currentScreen = Screen::OS_ENTRY;

    osInactTimer = lv_timer_create(cbOsInact, OS_INACT_MS, nullptr);
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

static void publishOs() {
    if (!mqttReady) {
        Serial.println("OS report queued but MQTT not ready — dropped");
        return;
    }

    xSemaphoreTake(clkMutex, portMAX_DELAY);
    ClockState snap = clk;
    xSemaphoreGive(clkMutex);

    int rr_min = -1;
    if (snap.synced) {
        int total = snap.base_hour * 60 + snap.base_min;
        if (snap.running) {
            unsigned long elapsed_ms = millis() - snap.tick_ms;
            total += (int)((float)elapsed_ms / 1000.0f * snap.speed / 60.0f);
        }
        rr_min = total % 1440;
    }

    int rr_h = (rr_min >= 0) ? rr_min / 60 : 0;
    int rr_m = (rr_min >= 0) ? rr_min % 60 : 0;

    char topic[48], payload[192];
    snprintf(topic, sizeof(topic), "trains/os/%s", cfg.sta_id);
    snprintf(payload, sizeof(payload),
             "{\"station_id\":\"%s\",\"train\":\"%s\",\"section\":0,"
             "\"direction\":\"%s\",\"extra\":%s,\"work_extra\":%s,"
             "\"rr_time\":\"%02d:%02d\",\"day\":%d}",
             cfg.sta_id, os_train, os_dir,
             os_extra ? "true" : "false",
             os_work_extra ? "true" : "false",
             rr_h, rr_m, snap.day);
    mqttClient.publish(topic, 1, false, payload);
    Serial.printf("OS published: train=%s dir=%s rr=%02d:%02d day=%d\n",
                  os_train, os_dir, rr_h, rr_m, snap.day);
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

    // Subscribe to train orders addressed to this station
    if (cfg.sta_id[0] != '\0') {
        char toTopic[48];
        snprintf(toTopic, sizeof(toTopic), "trains/to/%s", cfg.sta_id);
        mqttClient.subscribe(toTopic, 2);
    }

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
    char buf[512];
    size_t copy = min(len, sizeof(buf) - 1);
    memcpy(buf, payload, copy);
    buf[copy] = '\0';

    // ── Train Order ───────────────────────────────────────────────────────
    // Topic: trains/to/{station_id}
    if (cfg.sta_id[0] != '\0') {
        char toTopic[48];
        snprintf(toTopic, sizeof(toTopic), "trains/to/%s", cfg.sta_id);
        if (strcmp(topic, toTopic) == 0) {
            JsonDocument doc;
            if (deserializeJson(doc, buf)) return;

            PendingTo pt;
            pt.seq = doc["seq"] | 0;
            strlcpy(pt.to_type, doc["to_type"] | "", sizeof(pt.to_type));

            // Build comma-separated trains[] string for matching
            pt.trains[0] = '\0';
            JsonArray trains = doc["trains"].as<JsonArray>();
            for (JsonVariant v : trains) {
                if (pt.trains[0]) strlcat(pt.trains, ",", sizeof(pt.trains));
                strlcat(pt.trains, v.as<const char*>(), sizeof(pt.trains));
            }

            // Serialise fields object back to string for later rendering
            serializeJson(doc["fields"], pt.fields_json, sizeof(pt.fields_json));

            xSemaphoreTake(toMutex, portMAX_DELAY);
            pendingTos.push_back(pt);
            xSemaphoreGive(toMutex);

            Serial.printf("TO #%d (%s) received; trains=[%s]\n",
                          pt.seq, pt.to_type, pt.trains);
            return;
        }
    }

    JsonDocument doc;
    if (deserializeJson(doc, buf)) return;

    if (strcmp(topic, "trains/clock/time") == 0) {
        xSemaphoreTake(clkMutex, portMAX_DELAY);
        clk.base_hour = doc["hour"]    | clk.base_hour;
        clk.base_min  = doc["minute"]  | clk.base_min;
        clk.speed     = doc["speed"]   | clk.speed;
        clk.running   = doc["running"] | clk.running;
        clk.day       = doc["day"]     | clk.day;
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
            if (!doc["day"].isNull())    clk.day       = doc["day"];
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
    toMutex  = xSemaphoreCreateMutex();

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
