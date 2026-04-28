/* TO_Signal — Train Order Signal Controller
 * NY&E Northern Lights Subdivision Layout Control System
 *
 * Subscribes to trains/signal/{station_id}/to/cmd
 * Publishes to  trains/signal/{station_id}/to/state
 *
 * TODO Phase 2:
 *   - NVS provisioning (station ID, WiFi creds, MQTT broker, servo angles)
 *   - WiFi + MQTT connect/reconnect (port from Switch_Control)
 *   - Servo raise/lower on cmd receipt
 *   - State report publish after movement
 *   - Restore commanded state from retained topic on reconnect
 */

#include <Arduino.h>

void setup() {
  Serial.begin(115200);
  Serial.println("TO_Signal stub — Phase 2 implementation pending");
}

void loop() {
  delay(1000);
}
