/* Station OS input and communications for MRR control*/

/*  Original code by Rui Santos & Sara Santos - Random Nerd Tutorials - https://RandomNerdTutorials.com/esp32-lvgl-ebook/
    THIS EXAMPLE WAS TESTED WITH THE FOLLOWING HARDWARE:
    1) ESP32-2432S028R 2.8 inch 240×320 also known as the Cheap Yellow Display (CYD): https://makeradvisor.com/tools/cyd-cheap-yellow-display-esp32-2432s028r/
      SET UP INSTRUCTIONS: https://RandomNerdTutorials.com/cyd-lvgl/
    2) REGULAR ESP32 Dev Board + 2.8 inch 240x320 TFT Display: https://makeradvisor.com/tools/2-8-inch-ili9341-tft-240x320/ and https://makeradvisor.com/tools/esp32-dev-board-wi-fi-bluetooth/
      SET UP INSTRUCTIONS: https://RandomNerdTutorials.com/esp32-tft-lvgl/
    Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files.
    The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
*/

#include <Arduino.h>

/*  Install the "lvgl" library version 9.X by kisvegabor to interface with the TFT Display - https://lvgl.io/
    *** IMPORTANT: lv_conf.h available on the internet will probably NOT work with the examples available at Random Nerd Tutorials ***
    *** YOU MUST USE THE lv_conf.h FILE PROVIDED IN THE LINK BELOW IN ORDER TO USE THE EXAMPLES FROM RANDOM NERD TUTORIALS ***
    FULL INSTRUCTIONS AVAILABLE ON HOW CONFIGURE THE LIBRARY: https://RandomNerdTutorials.com/cyd-lvgl/ or https://RandomNerdTutorials.com/esp32-tft-lvgl/   */
#include <lvgl.h>

/*  Install the "TFT_eSPI" library by Bodmer to interface with the TFT Display - https://github.com/Bodmer/TFT_eSPI
    *** IMPORTANT: User_Setup.h available on the internet will probably NOT work with the examples available at Random Nerd Tutorials ***
    *** YOU MUST USE THE User_Setup.h FILE PROVIDED IN THE LINK BELOW IN ORDER TO USE THE EXAMPLES FROM RANDOM NERD TUTORIALS ***
    FULL INSTRUCTIONS AVAILABLE ON HOW CONFIGURE THE LIBRARY: https://RandomNerdTutorials.com/cyd-lvgl/ or https://RandomNerdTutorials.com/esp32-tft-lvgl/   */
#include <TFT_eSPI.h>

#include <WiFi.h>
extern "C" {
  #include "freertos/FreeRTOS.h"
  #include "freertos/timers.h"
}
#include <AsyncMqttClient.h>

// Replace with your network credentials
const char* ssid = "SPROG-Pi4";
const char* password = "pi-sprog";

#define MQTT_HOST IPAddress(192, 168, 6, 1) // MQTT BROKER IP ADDRESS
//#define MQTT_HOST "example.com"   // MQTT BROKER URL
#define MQTT_PORT 1883
#define BROKER_USER "pi"
#define BROKER_PASS "pi-sprog"

char* station = "ostest";
#define MQTT_TOPIC_OS "trains/os/"
String topic_os = MQTT_TOPIC_OS + String(station);
#define MQTT_TOPIC_TO "trains/to"
#define MQTT_TOPIC_TO_MULTI "trains/to/#+"
String topic_to1 = String(MQTT_TOPIC_TO) + "/" + String(station) + "/N";
String topic_to2 = String(MQTT_TOPIC_TO) + "/" + String(station) + "/S";

AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;

// Install the "XPT2046_Touchscreen" library by Paul Stoffregen to use the Touchscreen - https://github.com/PaulStoffregen/XPT2046_Touchscreen - Note: this library doesn't require further configuration
#include <XPT2046_Touchscreen.h>

// Touchscreen pins
#define XPT2046_IRQ 36   // T_IRQ
#define XPT2046_MOSI 32  // T_DIN
#define XPT2046_MISO 39  // T_OUT
#define XPT2046_CLK 25   // T_CLK
#define XPT2046_CS 33    // T_CS

SPIClass touchscreenSPI = SPIClass(VSPI);
XPT2046_Touchscreen touchscreen(XPT2046_CS, XPT2046_IRQ);

#define SCREEN_WIDTH 240
#define SCREEN_HEIGHT 320

// Touchscreen coordinates: (x, y) and pressure (z)
int x, y, z;

#define DRAW_BUF_SIZE (SCREEN_WIDTH * SCREEN_HEIGHT / 10 * (LV_COLOR_DEPTH / 8))
uint32_t draw_buf[DRAW_BUF_SIZE / 4];

// setting PWM properties
const int freq = 5000;
const int ledChannel = 0;
const int resolution = 8;

// Include Wire Library for I2C
#include <Wire.h>

// Include Adafruit PCA9685 Servo Library
#include <Adafruit_PWMServoDriver.h>

#define I2C_SDA 27
#define I2C_SCL 22
TwoWire I2CPCA = TwoWire(0);

// Creat object to represent PCA9685 at default I2C address
Adafruit_PWMServoDriver pca9685 = Adafruit_PWMServoDriver(0x40, I2CPCA);

// Define maximum and minimum number of "ticks" for the servo motors
// Range from 0 to 4095
// This determines the pulse width

#define SERVOMIN  80  // Minimum value
#define SERVOMAX  600  // Maximum value

// Define servo motor connections (expand as required)
#define SER0  0   //Servo Motor 0 on connector 0
#define SER1  1   //Servo Motor 1 on connector 1

// Variables for Servo Motor positions (expand as required)
int pwm0;
int pwm1;

// If logging is enabled, it will inform the user about what is happening in the library
void log_print(lv_log_level_t level, const char * buf) {
  LV_UNUSED(level);
  Serial.println(buf);
  Serial.flush();
}

// Get the Touchscreen data
void touchscreen_read(lv_indev_t * indev, lv_indev_data_t * data) {
  // Checks if Touchscreen was touched, and prints X, Y and Pressure (Z)
  if(touchscreen.tirqTouched() && touchscreen.touched()) {
    // Get Touchscreen points
    TS_Point p = touchscreen.getPoint();

    // Advanced Touchscreen calibration, LEARN MORE » https://RandomNerdTutorials.com/touchscreen-calibration/
    float alpha_x, beta_x, alpha_y, beta_y, delta_x, delta_y;

    // REPLACE WITH YOUR OWN CALIBRATION VALUES » https://RandomNerdTutorials.com/touchscreen-calibration/
    alpha_x = -0.000;
    beta_x = 0.090;
    delta_x = -33.771;
    alpha_y = 0.066;
    beta_y = 0.000;
    delta_y = -14.632;

    x = alpha_y * p.x + beta_y * p.y + delta_y;
    // clamp x between 0 and SCREEN_WIDTH - 1
    x = max(0, x);
    x = min(SCREEN_WIDTH - 1, x);

    y = alpha_x * p.x + beta_x * p.y + delta_x;
    // clamp y between 0 and SCREEN_HEIGHT - 1
    y = max(0, y);
    y = min(SCREEN_HEIGHT - 1, y);

    // Basic Touchscreen calibration points with map function to the correct width and height
    //x = map(p.x, 200, 3700, 1, SCREEN_WIDTH);
    //y = map(p.y, 240, 3800, 1, SCREEN_HEIGHT);

    z = p.z;

    data->state = LV_INDEV_STATE_PRESSED;

    // Set the coordinates
    data->point.x = x;
    data->point.y = y;
  }
  else {
    data->state = LV_INDEV_STATE_RELEASED;
  }
}

void connectToWifi() {
  //Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(ssid, password);
}

void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}

void WiFiEvent(WiFiEvent_t event) {
  Serial.printf("[WiFi-event] event: %d\n", event);
  switch(event) {
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      //Serial.println("WiFi connected");
      //Serial.println("IP address: ");
      //Serial.println(WiFi.localIP());
      connectToMqtt();
      break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      //Serial.println("WiFi lost connection");
      xTimerStop(mqttReconnectTimer, 0); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
      xTimerStart(wifiReconnectTimer, 0);
      break;
  }
}

void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT.");
  /*Serial.print("Session present: ");
  Serial.println(sessionPresent);*/
  
  // Subscribe to topic "message" when it connects to the broker
  uint16_t packetIdSub = mqttClient.subscribe(MQTT_TOPIC_TO_MULTI, 2);
  Serial.print("Subscribing at QoS 2, packetId: ");
  Serial.println(packetIdSub);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");

  if (WiFi.isConnected()) {
    xTimerStart(mqttReconnectTimer, 0);
  }
}

void onMqttPublish(uint16_t packetId) {
  /*Serial.println("Publish acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);*/
}

void onMqttSubscribe(uint16_t packetId, uint8_t qos) {
  Serial.println("Subscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
  Serial.print("  qos: ");
  Serial.println(qos);
}

void onMqttUnsubscribe(uint16_t packetId) {
  /*Serial.println("Unsubscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);*/
}

void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
  // Do whatever you want when you receive a message
  // Save the message in a variable
  String received_message;
  for (int i = 0; i < len; i++) {
    Serial.println((char)payload[i]);
    received_message += (char)payload[i];
  }
  // Display the text message on the display receive from the MQTT topic "trainorder"
  Serial.println(topic);
    if (strcmp(topic, topic_to1.c_str()) == 0) {
      if (received_message == "ON"){
        // Move Motor 0 from 0 to 180 degrees
        for (int posDegrees = 0; posDegrees <= 180; posDegrees++) {

          // Determine PWM pulse width
          pwm0 = map(posDegrees, 0, 180, SERVOMIN, SERVOMAX);
          // Write to PCA9685
          pca9685.setPWM(SER0, 0, pwm0);
        }
      }
      if (received_message == "OFF"){
        // Move Motor 0 from 180 to 0 degrees
        for (int posDegrees = 180; posDegrees >= 0; posDegrees--) {

          // Determine PWM pulse width
          pwm0 = map(posDegrees, 0, 180, SERVOMIN, SERVOMAX);
          // Write to PCA9685
          pca9685.setPWM(SER0, 0, pwm0);
        }
      }
    }
    if (strcmp(topic, topic_to2.c_str()) == 0) {
      if (received_message == "ON"){
        // Move Motor 1 from 0 to 180 degrees
        for (int posDegrees = 0; posDegrees <= 180; posDegrees++) {
          // Determine PWM pulse width
          pwm1 = map(posDegrees, 0, 180, SERVOMIN, SERVOMAX);
          // Write to PCA9685
          pca9685.setPWM(SER1, 0, pwm1);
        }
      }
      if (received_message == "OFF"){
        // Move Motor 1 from 180 to 0 degrees
        for (int posDegrees = 180; posDegrees >= 0; posDegrees--) {

          // Determine PWM pulse width
          pwm1 = map(posDegrees, 0, 180, SERVOMIN, SERVOMAX);
          // Write to PCA9685
          pca9685.setPWM(SER1, 0, pwm1);
        }
      }
    }
}

static void text_area_event_handler(lv_event_t * e) {
  lv_obj_t * text_area = (lv_obj_t*) lv_event_get_target(e);
  LV_UNUSED(text_area);
  LV_LOG_USER("Enter was pressed. The current text is: %s", lv_textarea_get_text(text_area));

/* do MQTT */
  const char* os_string = lv_textarea_get_text(text_area);
  uint16_t packetIdPub1 = mqttClient.publish(topic_os.c_str(), 2, true, os_string);
  //Serial.printf("Publishing on topic %s at QoS 1, packetId: %s\n", MQTT_TOPIC_OS, packetIdPub1);
  //lv_create_os_screen();
}

static void button_matrix_event_handler(lv_event_t * e) {
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  lv_obj_t * text_area = (lv_obj_t*) lv_event_get_user_data(e);
  const char * txt = lv_buttonmatrix_get_button_text(obj, lv_buttonmatrix_get_selected_button(obj));

  if(lv_strcmp(txt, LV_SYMBOL_BACKSPACE) == 0) {
    lv_textarea_delete_char(text_area);
  }
  else if(lv_strcmp(txt, LV_SYMBOL_OK) == 0) {
    lv_obj_send_event(text_area, LV_EVENT_READY, NULL);
  }
  else {
    lv_textarea_add_text(text_area, txt);
  }
}

void lv_create_os_screen(void) {
  // Clear screen
  lv_obj_clean(lv_scr_act());

  lv_obj_t * text_label_station = lv_label_create(lv_screen_active());
  lv_label_set_text(text_label_station, station);
  lv_obj_align(text_label_station, LV_ALIGN_TOP_MID, 0, 10);

  lv_obj_t * text_area = lv_textarea_create(lv_screen_active());
  lv_textarea_set_one_line(text_area, true);
  lv_obj_align(text_area, LV_ALIGN_TOP_MID, 0, 40);
  //lv_textarea_set_password_mode(text_area, true);

  lv_obj_t * text_label = lv_label_create(lv_screen_active());
  lv_label_set_text(text_label, "Enter Train Number to OS");
  lv_obj_align_to(text_label, text_area, LV_ALIGN_TOP_MID, 0, -30);

  lv_obj_add_event_cb(text_area, text_area_event_handler, LV_EVENT_READY, text_area);
  lv_obj_add_state(text_area, LV_STATE_FOCUSED); // To be sure the cursor is visible

  static const char * keypad[]= {  "1", "2", "3", "N", "\n",
                                   "4", "5", "6", "S", "\n",
                                   "7", "8", "9", "X", "\n",
                                   LV_SYMBOL_BACKSPACE, "0", LV_SYMBOL_OK, "WX", ""
                                };

  lv_obj_t * button_matrix = lv_buttonmatrix_create(lv_screen_active());
  lv_obj_set_size(button_matrix, 200, 150);
  lv_obj_align(button_matrix, LV_ALIGN_BOTTOM_MID, 0, -10);
  lv_obj_add_event_cb(button_matrix, button_matrix_event_handler, LV_EVENT_VALUE_CHANGED, text_area);
  lv_obj_remove_flag(button_matrix, LV_OBJ_FLAG_CLICK_FOCUSABLE); // To keep the text area focused on button clicks
  lv_buttonmatrix_set_map(button_matrix, keypad);
}

// Callback that is triggered when the toggle switch changes state
static void toggle_switch_event_handler(lv_event_t * e) {
  lv_event_code_t code = lv_event_get_code(e);
  lv_obj_t * obj = (lv_obj_t*) lv_event_get_target(e);
  if(code == LV_EVENT_VALUE_CHANGED) {
    LV_UNUSED(obj);
    LV_LOG_USER("State: %s", lv_obj_has_state(obj, LV_STATE_CHECKED) ? "On" : "Off");
  }
}

void setup() {
  String LVGL_Arduino = String("LVGL Library Version: ") + lv_version_major() + "." + lv_version_minor() + "." + lv_version_patch();
  Serial.begin(115200);
  Serial.println(LVGL_Arduino);
  
  I2CPCA.begin(I2C_SDA, I2C_SCL, 100000);
  // Initialize PCA9685
  pca9685.begin();

  // Set PWM Frequency to 50Hz
  pca9685.setPWMFreq(50);

  // Start LVGL
  lv_init();
  // Register print function for debugging
  lv_log_register_print_cb(log_print);

  // Start the SPI for the touchscreen and init the touchscreen
  touchscreenSPI.begin(XPT2046_CLK, XPT2046_MISO, XPT2046_MOSI, XPT2046_CS);
  touchscreen.begin(touchscreenSPI);
  // Set the Touchscreen rotation in landscape mode
  // Note: in some displays, the touchscreen might be upside down, so you might need to set the rotation to 0: touchscreen.setRotation(0);
  touchscreen.setRotation(2);

  // Create a display object
  lv_display_t * disp;
  // Initialize the TFT display using the TFT_eSPI library
  disp = lv_tft_espi_create(SCREEN_WIDTH, SCREEN_HEIGHT, draw_buf, sizeof(draw_buf));
  lv_display_set_rotation(disp, LV_DISPLAY_ROTATION_270);
  
  // Initialize an LVGL input device object (Touchscreen)
  lv_indev_t * indev = lv_indev_create();
  lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
  // Set the callback function to read Touchscreen input
  lv_indev_set_read_cb(indev, touchscreen_read);

  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));
  WiFi.onEvent(WiFiEvent);
  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.onSubscribe(onMqttSubscribe);
  mqttClient.onUnsubscribe(onMqttUnsubscribe);
  mqttClient.onMessage(onMqttMessage);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setCredentials(BROKER_USER, BROKER_PASS);
  connectToWifi();

  // Function to draw the OS screen with keypad
  lv_create_os_screen();
}

void loop() {
  lv_task_handler();  // let the GUI do its work
  lv_tick_inc(5);     // tell LVGL how much time has passed
  delay(5);           // let this time pass
}