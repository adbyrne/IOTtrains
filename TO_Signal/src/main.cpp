/* TO_Signal — Train Order Signal Controller
 * NY&E Northern Lights Subdivision
 *
 * Controls two TO signal arms (N=northbound, S=southbound) per station.
 *
 * Subscribe: trains/signal/{station}/to/{N|S}/cmd   (retained, QoS 1)
 * Publish:   trains/signal/{station}/to/{N|S}/state  (retained, QoS 1)
 *
 * Calibration: connect via serial monitor (115200), type 'help'.
 * First boot (not provisioned) prints this prompt automatically.
 */

#include <Arduino.h>
#include <WiFi.h>
#include <AsyncMqttClient.h>
#include <ESP32Servo.h>
#include <Preferences.h>
#include <ArduinoJson.h>

// ── Hardware ──────────────────────────────────────────────────────────────
static const int SERVO_PIN_N   = 13;
static const int SERVO_PIN_S   = 14;
static const int MQTT_PORT     = 1883;

// ── Sweep tuning ──────────────────────────────────────────────────────────
// 15 ms/degree → ~0.9 s for a 60° sweep; adjust if motion feels too fast/slow
static const int SWEEP_STEP_MS  = 15;
// Linkage bounce: overshoot this many degrees at end of travel, then return
static const int BOUNCE_DEG     = 3;
// Timing (ms) for bounce settle: pause → overshoot → settle
static const int BOUNCE_HOLD_MS = 60;
static const int BOUNCE_OVER_MS = 50;
static const int BOUNCE_BACK_MS = 30;

// ── NVS defaults ──────────────────────────────────────────────────────────
static const int RAISED_DEFAULT  = 45;
static const int LOWERED_DEFAULT = 90;

// ── NVS keys (≤ 15 chars) ─────────────────────────────────────────────────
static const char* NVS_NS       = "to_sig";
static const char* KEY_PROV     = "prov";
static const char* KEY_SSID     = "ssid";
static const char* KEY_WPAS     = "wpas";
static const char* KEY_MQTT_IP  = "mqtt_ip";
static const char* KEY_MUSER    = "muser";
static const char* KEY_MPAS     = "mpas";
static const char* KEY_STA_ID   = "sta_id";
static const char* KEY_N_RAISED  = "n_raised";
static const char* KEY_N_LOWERED = "n_lowrd";
static const char* KEY_S_RAISED  = "s_raised";
static const char* KEY_S_LOWERED = "s_lowrd";

// ── Config ────────────────────────────────────────────────────────────────
struct Config {
    char ssid[64]    = {};
    char wpas[64]    = {};   // WiFi password
    char mqtt_ip[40] = {};
    char muser[32]   = "to_signal";
    char mpas[32]    = {};   // MQTT password
    char sta_id[4]   = {};   // e.g. "BB"
    int  n_raised    = RAISED_DEFAULT;
    int  n_lowered   = LOWERED_DEFAULT;
    int  s_raised    = RAISED_DEFAULT;
    int  s_lowered   = LOWERED_DEFAULT;
};

Config      cfg;
Preferences prefs;
bool        provisioned = false;

// ── Arm ───────────────────────────────────────────────────────────────────
enum ArmState { ARM_UNKNOWN, ARM_LOWERED, ARM_RAISED };

struct Arm {
    Servo       servo;
    int         pin      = 0;
    const char* dir      = nullptr;   // "N" or "S"
    ArmState    state    = ARM_UNKNOWN;
    int         angle    = -1;        // -1 = position unknown (first boot)
    int*        pRaised  = nullptr;
    int*        pLowered = nullptr;
};

Arm armN, armS;

// ── MQTT ──────────────────────────────────────────────────────────────────
AsyncMqttClient mqttClient;
TimerHandle_t   mqttReconTimer = nullptr;
TimerHandle_t   wifiReconTimer = nullptr;
bool            mqttReady      = false;

// ── Servo task queue ──────────────────────────────────────────────────────
struct ServoCmd {
    uint8_t  idx;       // 0=N, 1=S
    int      toAngle;
    bool     bounce;
    ArmState newState;  // ARM_UNKNOWN = calibration sweep (no state publish)
};
static const int SERVO_QUEUE_DEPTH = 4;
QueueHandle_t servoQueue = nullptr;

// ── Railroad time ─────────────────────────────────────────────────────────
char              rrTime[8]     = "00:00";
SemaphoreHandle_t rrTimeMutex   = nullptr;

// ── Forward declarations ──────────────────────────────────────────────────
static void loadConfig();
static void saveConfig();
static void initArms();
static void queueMove(int idx, ArmState target);
static void queueAngle(int idx, int angle);
static void cmdTopic(char* buf, size_t n, int idx);
static void stateTopic(char* buf, size_t n, int idx);
static void publishState(int idx);
static void connectToWifi();
static void connectToMqtt();
static void startNetwork();
static void onWifiGotIP(WiFiEvent_t, WiFiEventInfo_t);
static void onWifiDisconnect(WiFiEvent_t, WiFiEventInfo_t);
static void onMqttConnect(bool sessionPresent);
static void onMqttDisconnect(AsyncMqttClientDisconnectReason reason);
static void onMqttMessage(char* topic, char* payload,
    AsyncMqttClientMessageProperties props,
    size_t len, size_t index, size_t total);
static void servoTask(void* param);
static void printHelp();
static void printConfig();
static void handleCliLine(char* line);
static void serialCliTask(void* param);

// ─────────────────────────────────────────────────────────────────────────
// Config
// ─────────────────────────────────────────────────────────────────────────

static void loadConfig() {
    prefs.begin(NVS_NS, /*readOnly=*/true);
    provisioned  = prefs.getBool(KEY_PROV, false);
    prefs.getString(KEY_SSID,    cfg.ssid,    sizeof(cfg.ssid));
    prefs.getString(KEY_WPAS,    cfg.wpas,    sizeof(cfg.wpas));
    prefs.getString(KEY_MQTT_IP, cfg.mqtt_ip, sizeof(cfg.mqtt_ip));
    prefs.getString(KEY_MUSER,   cfg.muser,   sizeof(cfg.muser));
    prefs.getString(KEY_MPAS,    cfg.mpas,    sizeof(cfg.mpas));
    prefs.getString(KEY_STA_ID,  cfg.sta_id,  sizeof(cfg.sta_id));
    cfg.n_raised  = prefs.getInt(KEY_N_RAISED,  RAISED_DEFAULT);
    cfg.n_lowered = prefs.getInt(KEY_N_LOWERED, LOWERED_DEFAULT);
    cfg.s_raised  = prefs.getInt(KEY_S_RAISED,  RAISED_DEFAULT);
    cfg.s_lowered = prefs.getInt(KEY_S_LOWERED, LOWERED_DEFAULT);
    prefs.end();
}

static void saveConfig() {
    prefs.begin(NVS_NS, /*readOnly=*/false);
    prefs.putBool(KEY_PROV,    true);
    prefs.putString(KEY_SSID,    cfg.ssid);
    prefs.putString(KEY_WPAS,    cfg.wpas);
    prefs.putString(KEY_MQTT_IP, cfg.mqtt_ip);
    prefs.putString(KEY_MUSER,   cfg.muser);
    prefs.putString(KEY_MPAS,    cfg.mpas);
    prefs.putString(KEY_STA_ID,  cfg.sta_id);
    prefs.putInt(KEY_N_RAISED,  cfg.n_raised);
    prefs.putInt(KEY_N_LOWERED, cfg.n_lowered);
    prefs.putInt(KEY_S_RAISED,  cfg.s_raised);
    prefs.putInt(KEY_S_LOWERED, cfg.s_lowered);
    prefs.end();
    provisioned = true;
}

// ─────────────────────────────────────────────────────────────────────────
// Arms
// ─────────────────────────────────────────────────────────────────────────

static void initArms() {
    armN.pin      = SERVO_PIN_N;
    armN.dir      = "N";
    armN.pRaised  = &cfg.n_raised;
    armN.pLowered = &cfg.n_lowered;
    armN.servo.attach(SERVO_PIN_N);

    armS.pin      = SERVO_PIN_S;
    armS.dir      = "S";
    armS.pRaised  = &cfg.s_raised;
    armS.pLowered = &cfg.s_lowered;
    armS.servo.attach(SERVO_PIN_S);
    // angle stays -1 (unknown) until first command; first command snaps to target
}

// ─────────────────────────────────────────────────────────────────────────
// Topic helpers
// ─────────────────────────────────────────────────────────────────────────

static void cmdTopic(char* buf, size_t n, int idx) {
    snprintf(buf, n, "trains/signal/%s/to/%s/cmd",   cfg.sta_id, idx == 0 ? "N" : "S");
}
static void stateTopic(char* buf, size_t n, int idx) {
    snprintf(buf, n, "trains/signal/%s/to/%s/state", cfg.sta_id, idx == 0 ? "N" : "S");
}

// ─────────────────────────────────────────────────────────────────────────
// Servo queue helpers (callable from any task)
// ─────────────────────────────────────────────────────────────────────────

static void queueMove(int idx, ArmState target) {
    Arm* arms[2] = { &armN, &armS };
    int angle = (target == ARM_RAISED) ? *arms[idx]->pRaised : *arms[idx]->pLowered;
    ServoCmd cmd = { (uint8_t)idx, angle, /*bounce=*/true, target };
    xQueueSend(servoQueue, &cmd, portMAX_DELAY);
}

// Calibration sweep: no bounce, does not update arm.state, no state publish
static void queueAngle(int idx, int angle) {
    ServoCmd cmd = { (uint8_t)idx, angle, /*bounce=*/false, ARM_UNKNOWN };
    xQueueSend(servoQueue, &cmd, portMAX_DELAY);
}

// ─────────────────────────────────────────────────────────────────────────
// Servo task — owns all servo writes
// ─────────────────────────────────────────────────────────────────────────

static void servoTask(void*) {
    Arm* arms[2] = { &armN, &armS };
    ServoCmd cmd;

    while (true) {
        if (xQueueReceive(servoQueue, &cmd, portMAX_DELAY) != pdTRUE) continue;

        Arm& arm  = *arms[cmd.idx];
        int  from = (arm.angle >= 0) ? arm.angle : cmd.toAngle; // snap on first command

        if (from != cmd.toAngle) {
            int step = (cmd.toAngle > from) ? 1 : -1;
            for (int a = from; a != cmd.toAngle; a += step) {
                arm.servo.write(a);
                vTaskDelay(pdMS_TO_TICKS(SWEEP_STEP_MS));
            }
            arm.servo.write(cmd.toAngle);

            if (cmd.bounce) {
                // Simulate mechanical linkage settling: slight overshoot, then return
                vTaskDelay(pdMS_TO_TICKS(BOUNCE_HOLD_MS));
                arm.servo.write(cmd.toAngle + step * BOUNCE_DEG);
                vTaskDelay(pdMS_TO_TICKS(BOUNCE_OVER_MS));
                arm.servo.write(cmd.toAngle);
                vTaskDelay(pdMS_TO_TICKS(BOUNCE_BACK_MS));
            }
        } else {
            arm.servo.write(cmd.toAngle);
        }

        arm.angle = cmd.toAngle;
        arm.state = cmd.newState;

        if (cmd.newState != ARM_UNKNOWN && mqttReady) {
            publishState(cmd.idx);
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────
// State publish
// ─────────────────────────────────────────────────────────────────────────

static void publishState(int idx) {
    Arm* arms[2] = { &armN, &armS };
    Arm& arm     = *arms[idx];

    char timeLocal[8] = "00:00";
    if (xSemaphoreTake(rrTimeMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        strncpy(timeLocal, rrTime, sizeof(timeLocal));
        xSemaphoreGive(rrTimeMutex);
    }

    JsonDocument doc;
    doc["state"]      = (arm.state == ARM_RAISED) ? "raised" : "lowered";
    doc["station_id"] = cfg.sta_id;
    doc["dir"]        = (idx == 0) ? "N" : "S";
    doc["rr_time"]    = timeLocal;

    char payload[128], topic[64];
    serializeJson(doc, payload, sizeof(payload));
    stateTopic(topic, sizeof(topic), idx);
    mqttClient.publish(topic, 1, /*retain=*/true, payload);
}

// ─────────────────────────────────────────────────────────────────────────
// WiFi / MQTT
// ─────────────────────────────────────────────────────────────────────────

static void connectToWifi() {
    Serial.printf("[WiFi] Connecting to %s\n", cfg.ssid);
    WiFi.begin(cfg.ssid, cfg.wpas);
}

static void connectToMqtt() {
    Serial.printf("[MQTT] Connecting to %s\n", cfg.mqtt_ip);
    mqttClient.connect();
}

static void onWifiGotIP(WiFiEvent_t, WiFiEventInfo_t) {
    Serial.println("[WiFi] Connected.");
    connectToMqtt();
}

static void onWifiDisconnect(WiFiEvent_t, WiFiEventInfo_t) {
    Serial.println("[WiFi] Disconnected.");
    mqttReady = false;
    xTimerStop(mqttReconTimer, 0);
    xTimerStart(wifiReconTimer, 0);
}

static void onMqttConnect(bool /*sessionPresent*/) {
    Serial.printf("[MQTT] Connected. Station: %s\n", cfg.sta_id);
    mqttReady = true;

    // Subscribe to both arm cmd topics (retained — restores state on reconnect)
    char topic[64];
    for (int i = 0; i < 2; i++) {
        cmdTopic(topic, sizeof(topic), i);
        mqttClient.subscribe(topic, 1);
        Serial.printf("[MQTT] Subscribed: %s\n", topic);
    }
    // Track railroad time for state reports
    mqttClient.subscribe("trains/clock/time", 0);
}

static void onMqttDisconnect(AsyncMqttClientDisconnectReason /*reason*/) {
    Serial.println("[MQTT] Disconnected.");
    mqttReady = false;
    if (WiFi.isConnected()) {
        xTimerStart(mqttReconTimer, 0);
    }
}

static void onMqttMessage(
    char* topic, char* payload,
    AsyncMqttClientMessageProperties /*props*/,
    size_t len, size_t /*index*/, size_t /*total*/)
{
    char buf[256];
    size_t n = (len < sizeof(buf) - 1) ? len : sizeof(buf) - 1;
    memcpy(buf, payload, n);
    buf[n] = '\0';

    // Railroad time update
    if (strcmp(topic, "trains/clock/time") == 0) {
        JsonDocument doc;
        if (deserializeJson(doc, buf) == DeserializationError::Ok) {
            char tmp[8];
            snprintf(tmp, sizeof(tmp), "%02d:%02d",
                (int)(doc["hour"] | 0), (int)(doc["minute"] | 0));
            if (xSemaphoreTake(rrTimeMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
                strncpy(rrTime, tmp, sizeof(rrTime));
                xSemaphoreGive(rrTimeMutex);
            }
        }
        return;
    }

    // Signal arm command
    JsonDocument doc;
    if (deserializeJson(doc, buf) != DeserializationError::Ok) return;
    const char* state = doc["state"] | "";

    ArmState target;
    if      (strcmp(state, "raised")  == 0) target = ARM_RAISED;
    else if (strcmp(state, "lowered") == 0) target = ARM_LOWERED;
    else return;

    char nCmd[64], sCmd[64];
    cmdTopic(nCmd, sizeof(nCmd), 0);
    cmdTopic(sCmd, sizeof(sCmd), 1);

    if      (strcmp(topic, nCmd) == 0) queueMove(0, target);
    else if (strcmp(topic, sCmd) == 0) queueMove(1, target);
}

static void startNetwork() {
    mqttReconTimer = xTimerCreate("mqttRec", pdMS_TO_TICKS(2000), pdFALSE, nullptr,
        reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
    wifiReconTimer = xTimerCreate("wifiRec", pdMS_TO_TICKS(5000), pdFALSE, nullptr,
        reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));

    WiFi.onEvent(onWifiGotIP,      ARDUINO_EVENT_WIFI_STA_GOT_IP);
    WiFi.onEvent(onWifiDisconnect, ARDUINO_EVENT_WIFI_STA_DISCONNECTED);

    char clientId[32];
    snprintf(clientId, sizeof(clientId), "to_sig_%s_%06X",
        cfg.sta_id, (uint32_t)(ESP.getEfuseMac() & 0xFFFFFF));

    mqttClient.setClientId(clientId);
    mqttClient.setServer(cfg.mqtt_ip, MQTT_PORT);
    if (cfg.muser[0] && cfg.mpas[0]) {
        mqttClient.setCredentials(cfg.muser, cfg.mpas);
    }
    mqttClient.onConnect(onMqttConnect);
    mqttClient.onDisconnect(onMqttDisconnect);
    mqttClient.onMessage(onMqttMessage);

    connectToWifi();
}

// ─────────────────────────────────────────────────────────────────────────
// Serial CLI
// ─────────────────────────────────────────────────────────────────────────

static void printHelp() {
    Serial.println(
        "\n--- TO_Signal CLI ---\n"
        "Network config:\n"
        "  set ssid <SSID>                  WiFi network name\n"
        "  set pass <password>              WiFi password\n"
        "  set mqtt <ip>                    MQTT broker IP\n"
        "  set station <XP|BB|JC|MC|SK>    station ID\n"
        "  set muser <user>                 MQTT username (default: to_signal)\n"
        "  set mpas <password>              MQTT password\n"
        "Servo calibration:\n"
        "  set angle N raised <0-180>       N arm raised angle\n"
        "  set angle N lowered <0-180>      N arm lowered angle\n"
        "  set angle S raised <0-180>       S arm raised angle\n"
        "  set angle S lowered <0-180>      S arm lowered angle\n"
        "  sweep N <0-180>                  move N arm to angle (no bounce)\n"
        "  sweep S <0-180>                  move S arm to angle (no bounce)\n"
        "  raise N  /  raise S              sweep to configured raised angle\n"
        "  lower N  /  lower S              sweep to configured lowered angle\n"
        "Other:\n"
        "  show                             print current config\n"
        "  save                             save config to NVS\n"
        "  restart                          restart device\n"
        "  help                             this message\n"
    );
}

static void printConfig() {
    Serial.printf(
        "\nConfig:\n"
        "  ssid:       %s\n"
        "  pass:       %s\n"
        "  mqtt_ip:    %s\n"
        "  muser:      %s\n"
        "  mpas:       %s\n"
        "  station_id: %s\n"
        "  N raised:   %d°   N lowered: %d°\n"
        "  S raised:   %d°   S lowered: %d°\n"
        "  provisioned: %s\n"
        "  mqtt_ready:  %s\n",
        cfg.ssid,
        strlen(cfg.wpas) ? "***" : "(not set)",
        cfg.mqtt_ip,
        cfg.muser,
        strlen(cfg.mpas) ? "***" : "(not set)",
        cfg.sta_id,
        cfg.n_raised,  cfg.n_lowered,
        cfg.s_raised,  cfg.s_lowered,
        provisioned ? "yes" : "no",
        mqttReady   ? "yes" : "no"
    );
}

static void handleCliLine(char* line) {
    char* tok = strtok(line, " \t");
    if (!tok) return;

    if (strcmp(tok, "help") == 0) {
        printHelp();

    } else if (strcmp(tok, "show") == 0) {
        printConfig();

    } else if (strcmp(tok, "save") == 0) {
        saveConfig();
        Serial.println("Config saved. Type 'restart' to apply network settings.");

    } else if (strcmp(tok, "restart") == 0) {
        Serial.println("Restarting...");
        vTaskDelay(pdMS_TO_TICKS(500));
        ESP.restart();

    } else if (strcmp(tok, "set") == 0) {
        char* key = strtok(nullptr, " \t");
        if (!key) { Serial.println("Usage: set <key> ..."); return; }

        if (strcmp(key, "ssid") == 0) {
            char* val = strtok(nullptr, " \t");
            if (val) { strncpy(cfg.ssid, val, sizeof(cfg.ssid) - 1); Serial.printf("ssid = %s\n", cfg.ssid); }
            else       Serial.println("Usage: set ssid <SSID>");

        } else if (strcmp(key, "pass") == 0) {
            char* val = strtok(nullptr, " \t");
            if (val) { strncpy(cfg.wpas, val, sizeof(cfg.wpas) - 1); Serial.println("WiFi password set."); }
            else       Serial.println("Usage: set pass <password>");

        } else if (strcmp(key, "mqtt") == 0) {
            char* val = strtok(nullptr, " \t");
            if (val) { strncpy(cfg.mqtt_ip, val, sizeof(cfg.mqtt_ip) - 1); Serial.printf("mqtt = %s\n", cfg.mqtt_ip); }
            else       Serial.println("Usage: set mqtt <ip>");

        } else if (strcmp(key, "station") == 0) {
            char* val = strtok(nullptr, " \t");
            if (val) { strncpy(cfg.sta_id, val, sizeof(cfg.sta_id) - 1); Serial.printf("station = %s\n", cfg.sta_id); }
            else       Serial.println("Usage: set station <XP|BB|JC|MC|SK>");

        } else if (strcmp(key, "muser") == 0) {
            char* val = strtok(nullptr, " \t");
            if (val) { strncpy(cfg.muser, val, sizeof(cfg.muser) - 1); Serial.printf("muser = %s\n", cfg.muser); }
            else       Serial.println("Usage: set muser <user>");

        } else if (strcmp(key, "mpas") == 0) {
            char* val = strtok(nullptr, " \t");
            if (val) { strncpy(cfg.mpas, val, sizeof(cfg.mpas) - 1); Serial.println("MQTT password set."); }
            else       Serial.println("Usage: set mpas <password>");

        } else if (strcmp(key, "angle") == 0) {
            char* dir     = strtok(nullptr, " \t");
            char* pos_str = strtok(nullptr, " \t");
            char* deg_str = strtok(nullptr, " \t");
            if (!dir || !pos_str || !deg_str) {
                Serial.println("Usage: set angle <N|S> <raised|lowered> <0-180>");
                return;
            }
            int deg = atoi(deg_str);
            if (deg < 0 || deg > 180) { Serial.println("Angle must be 0-180."); return; }

            bool isN      = (toupper((unsigned char)dir[0]) == 'N');
            bool isRaised = (strcmp(pos_str, "raised") == 0);
            if      ( isN &&  isRaised) cfg.n_raised  = deg;
            else if ( isN && !isRaised) cfg.n_lowered = deg;
            else if (!isN &&  isRaised) cfg.s_raised  = deg;
            else                        cfg.s_lowered = deg;
            Serial.printf("%s arm %s = %d°\n", isN ? "N" : "S", isRaised ? "raised" : "lowered", deg);

        } else {
            Serial.printf("Unknown key: %s\n", key);
            Serial.println("Keys: ssid, pass, mqtt, station, muser, mpas, angle");
        }

    } else if (strcmp(tok, "sweep") == 0) {
        char* dir     = strtok(nullptr, " \t");
        char* deg_str = strtok(nullptr, " \t");
        if (!dir || !deg_str) { Serial.println("Usage: sweep <N|S> <0-180>"); return; }
        int deg = atoi(deg_str);
        if (deg < 0 || deg > 180) { Serial.println("Angle 0-180."); return; }
        int idx = (toupper((unsigned char)dir[0]) == 'N') ? 0 : 1;
        queueAngle(idx, deg);
        Serial.printf("Sweep %s → %d° (queued)\n", idx == 0 ? "N" : "S", deg);

    } else if (strcmp(tok, "raise") == 0) {
        char* dir = strtok(nullptr, " \t");
        if (!dir) { Serial.println("Usage: raise <N|S>"); return; }
        int idx = (toupper((unsigned char)dir[0]) == 'N') ? 0 : 1;
        queueMove(idx, ARM_RAISED);
        Serial.printf("Raise %s (queued)\n", idx == 0 ? "N" : "S");

    } else if (strcmp(tok, "lower") == 0) {
        char* dir = strtok(nullptr, " \t");
        if (!dir) { Serial.println("Usage: lower <N|S>"); return; }
        int idx = (toupper((unsigned char)dir[0]) == 'N') ? 0 : 1;
        queueMove(idx, ARM_LOWERED);
        Serial.printf("Lower %s (queued)\n", idx == 0 ? "N" : "S");

    } else {
        Serial.printf("Unknown: '%s' — type 'help'\n", tok);
    }
}

static void serialCliTask(void*) {
    char   line[128];
    size_t pos = 0;

    vTaskDelay(pdMS_TO_TICKS(200)); // let Serial stabilize after setup

    if (!provisioned) {
        Serial.println(
            "\n=== TO_Signal: not configured ===\n"
            "Type 'help' for commands.\n"
            "Configure with 'set ...' commands, then 'save' and 'restart'.\n");
    } else {
        Serial.printf("\n=== TO_Signal %s === type 'help' for CLI\n", cfg.sta_id);
    }

    while (true) {
        while (Serial.available()) {
            char c = (char)Serial.read();
            if (c == '\r') continue;
            if (c == '\n') {
                line[pos] = '\0';
                if (pos > 0) handleCliLine(line);
                pos = 0;
            } else if (pos < sizeof(line) - 1) {
                line[pos++] = c;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// ─────────────────────────────────────────────────────────────────────────
// setup / loop
// ─────────────────────────────────────────────────────────────────────────

void setup() {
    Serial.begin(115200);

    loadConfig();
    initArms();

    servoQueue  = xQueueCreate(SERVO_QUEUE_DEPTH, sizeof(ServoCmd));
    rrTimeMutex = xSemaphoreCreateMutex();

    // Servo task at higher priority so sweep timing is not starved
    xTaskCreate(servoTask,    "servo", 4096, nullptr, 5, nullptr);
    xTaskCreate(serialCliTask, "cli",  4096, nullptr, 1, nullptr);

    if (provisioned) {
        startNetwork();
    }
}

void loop() {
    vTaskDelete(nullptr); // all work happens in FreeRTOS tasks
}
