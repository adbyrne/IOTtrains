
### **Product: JMRI-Compatible MQTT Turnout Controller**

**1. Introduction**
A network-enabled microcontroller solution using an ESP32 to control a model railroad turnout. The device is designed to be compatible with the JMRI (Java Model Railroad Interface) MQTT protocol for seamless integration.

**2. Core Components**
*   **Hardware:** ESP32 Microcontroller
*   **Actuator:** Standard hobby servo motor

**3. Features & Requirements**

*   **3.1. Servo Control**
    *   The device will control a servo to move the turnout between two positions.
    *   The speed of the servo movement should be slow and realistic, not instantaneous.

*   **3.2. Communication Protocol**
    *   **Connectivity:** The device must connect to a user-provided WiFi network.
    *   **Protocol:** All communication is handled via MQTT.
    *   **Reliability:** The device must automatically attempt to reconnect to the WiFi network and the MQTT broker if the connection is lost.
    *   **MQTT Root Topic:** The base topic for all communication is configurable, with a default of `mqTrains`.
    *   **MQTT Topics:** The device will follow JMRI conventions. It will subscribe and publish to a single topic for state and commands.
        *   **Topic Structure:** `[root_topic]/turnout/[device_id]/state`
        *   *Example:* `mqTrains/turnout/south_yard_01/state`
    *   **MQTT Payloads:**
        *   The device will listen for messages with payloads `CLOSED` or `THROWN` to trigger servo movement.
        *   After a successful movement, the device will publish a message with the new state (`CLOSED` or `THROWN`) to the same topic.

*   **3.3. Configuration**
    *   A "configuration mode" (e.g., triggered via the serial monitor) will allow the user to set and save parameters to the device's non-volatile memory.
    *   **Configurable Parameters:**
        1.  WiFi SSID
        2.  WiFi Password
        3.  MQTT Broker IP Address or Hostname
        4.  MQTT Root Topic (defaults to `mqTrains`)
        5.  Device ID / Location Identifier (e.g., `south_yard_01`)
        6.  Servo angle for the `CLOSED` position
        7.  Servo angle for the `THROWN` position