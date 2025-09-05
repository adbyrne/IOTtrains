
#include <Arduino.h>
#include <WiFi.h>
#include <AsyncMqttClient.h>
#include <ESP32Servo.h>

// --- Configuration Placeholders ---
// (These will be loaded from non-volatile storage in the final version)
const char* WIFI_SSID = "YOUR_WIFI_SSID";
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";
const char* MQTT_HOST = "YOUR_MQTT_BROKER_IP";
const int MQTT_PORT = 1883;

// These settings are configurable via the config function
char mqttRootTopic[50] = "mqTrains";
char deviceId[50] = "station_os_01";
int servoClosedAngle = 30;
int servoThrownAngle = 150;

// --- Global Objects ---
AsyncMqttClient mqttClient;
Servo turnoutServo;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;

// --- State ---
enum TurnoutState {
  CLOSED,
  THROWN,
  UNKNOWN
};
TurnoutState currentTurnoutState = UNKNOWN;

// --- Function Declarations ---
void connectToWifi();
void connectToMqtt();
void onMqttConnect(bool sessionPresent);
void onMqttDisconnect(AsyncMqttClientDisconnectReason reason);
void onMqttSubscribe(uint16_t packetId, uint8_t qos);
void onMqttUnsubscribe(uint16_t packetId);
void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total);
void onMqttPublish(uint16_t packetId);
void setTurnoutState(TurnoutState newState);

void setup() {
  Serial.begin(115200);
  Serial.println("Booting...");

  // TODO: Implement configuration mode logic here
  // e.g., check for a button press on boot to enter config mode

  // Initialize servo
  turnoutServo.attach(13); // Default servo pin, can be made configurable

  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));

  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onSubscribe(onMqttSubscribe);
  mqttClient.onUnsubscribe(onMqttUnsubscribe);
  mqttClient.onMessage(onMqttMessage);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);

  connectToWifi();
}

void loop() {
  // The Async client handles everything in the background
  // No need to call client.loop()
}

void connectToWifi() {
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  // The onWifiConnect event will trigger connectToMqtt()
}

void onWifiConnect(const WiFiEventStationModeGotIP& event) {
  Serial.println("Connected to Wi-Fi.");
  connectToMqtt();
}

void onWifiDisconnect(const WiFiEventStationModeDisconnected& event) {
  Serial.println("Disconnected from Wi-Fi.");
  xTimerStop(mqttReconnectTimer, 0); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
  xTimerStart(wifiReconnectTimer, 0);
}

void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}

void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT.");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);

  char topic[100];
  snprintf(topic, 100, "%s/turnout/%s/state", mqttRootTopic, deviceId);
  
  uint16_t packetIdSub = mqttClient.subscribe(topic, 2);
  Serial.print("Subscribing at QoS 2, packetId: ");
  Serial.println(packetIdSub);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");
  if (WiFi.isConnected()) {
    xTimerStart(mqttReconnectTimer, 0);
  }
}

void onMqttSubscribe(uint16_t packetId, uint8_t qos) {
  Serial.println("Subscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
  Serial.print("  qos: ");
  Serial.println(qos);
}

void onMqttUnsubscribe(uint16_t packetId) {
  Serial.println("Unsubscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
  Serial.println("Publish received.");
  
  // Create a null-terminated version of the payload
  char payloadStr[len + 1];
  strncpy(payloadStr, payload, len);
  payloadStr[len] = '\0';

  Serial.print("  Topic: ");
  Serial.println(topic);
  Serial.print("  Payload: ");
  Serial.println(payloadStr);

  if (strcmp(payloadStr, "CLOSED") == 0) {
    setTurnoutState(CLOSED);
  } else if (strcmp(payloadStr, "THROWN") == 0) {
    setTurnoutState(THROWN);
  }
}

void onMqttPublish(uint16_t packetId) {
  Serial.println("Publish acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void setTurnoutState(TurnoutState newState) {
  if (currentTurnoutState == newState) {
    return; // Already in the desired state
  }

  Serial.print("Setting turnout state to: ");
  if (newState == CLOSED) {
    Serial.println("CLOSED");
    turnoutServo.write(servoClosedAngle);
    currentTurnoutState = CLOSED;
  } else if (newState == THROWN) {
    Serial.println("THROWN");
    turnoutServo.write(servoThrownAngle);
    currentTurnoutState = THROWN;
  }

  // Publish the new state back to the broker
  char topic[100];
  snprintf(topic, 100, "%s/turnout/%s/state", mqttRootTopic, deviceId);
  const char* statePayload = (newState == CLOSED) ? "CLOSED" : "THROWN";
  mqttClient.publish(topic, 0, true, statePayload);
}
