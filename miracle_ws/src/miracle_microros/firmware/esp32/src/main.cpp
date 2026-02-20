// ============================================================================
// MIRACLE ESP32 Sensor Bridge Firmware
// ============================================================================
// Acquires vibration (3-axis accelerometer) and spindle current data from
// analog sensors and publishes them to the MIRACLE ROS 2 system via
// micro-ROS over WiFi (UDP transport).
//
// Hardware:
//   - ESP32 DevKit V1
//   - GPIO34 (ADC1_CH6) : Vibration X-axis
//   - GPIO35 (ADC1_CH7) : Vibration Y-axis
//   - GPIO36 (ADC1_CH0) : Vibration Z-axis
//   - GPIO39 (ADC1_CH3) : Spindle current (ACS712)
//   - GPIO2             : Status LED
//
// Transport: WiFi UDP to micro-ROS agent
// ============================================================================

#include <Arduino.h>
#include <WiFi.h>
#include <esp_task_wdt.h>

#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/string.h>
#include <std_msgs/msg/u_int32.h>
#include <std_msgs/msg/float32.h>
#include <sensor_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/multi_array_dimension.h>
#include <std_msgs/msg/multi_array_layout.h>

// ---------------------------------------------------------------------------
// Compile-time configuration (set via build_flags in platformio.ini)
// ---------------------------------------------------------------------------
#ifndef MIRACLE_AGENT_IP
  #define MIRACLE_AGENT_IP "192.168.1.1"
#endif
#ifndef MIRACLE_AGENT_PORT
  #define MIRACLE_AGENT_PORT 8888
#endif
#ifndef MIRACLE_WIFI_SSID
  #define MIRACLE_WIFI_SSID "MIRACLE_FACTORY_NET"
#endif
#ifndef MIRACLE_WIFI_PASSWORD
  #define MIRACLE_WIFI_PASSWORD "CHANGE_ME_IN_ENV"
#endif
#ifndef MIRACLE_MACHINE_ID
  #define MIRACLE_MACHINE_ID "cnc_001"
#endif
#ifndef MIRACLE_ROS_DOMAIN_ID
  #define MIRACLE_ROS_DOMAIN_ID 42
#endif
#ifndef MIRACLE_VIBRATION_RATE_HZ
  #define MIRACLE_VIBRATION_RATE_HZ 1000
#endif
#ifndef MIRACLE_CURRENT_RATE_HZ
  #define MIRACLE_CURRENT_RATE_HZ 500
#endif
#ifndef MIRACLE_HEARTBEAT_RATE_HZ
  #define MIRACLE_HEARTBEAT_RATE_HZ 1
#endif
#ifndef MIRACLE_PIN_VIBRATION_X
  #define MIRACLE_PIN_VIBRATION_X 34
#endif
#ifndef MIRACLE_PIN_VIBRATION_Y
  #define MIRACLE_PIN_VIBRATION_Y 35
#endif
#ifndef MIRACLE_PIN_VIBRATION_Z
  #define MIRACLE_PIN_VIBRATION_Z 36
#endif
#ifndef MIRACLE_PIN_CURRENT
  #define MIRACLE_PIN_CURRENT 39
#endif
#ifndef MIRACLE_LED_PIN
  #define MIRACLE_LED_PIN 2
#endif
#ifndef MIRACLE_WDT_TIMEOUT_S
  #define MIRACLE_WDT_TIMEOUT_S 10
#endif

// ---------------------------------------------------------------------------
// Topic name helpers
// ---------------------------------------------------------------------------
#define TOPIC_PREFIX "/miracle/" MIRACLE_MACHINE_ID
#define TOPIC_VIBRATION   TOPIC_PREFIX "/raw_sensor/vibration"
#define TOPIC_CURRENT     TOPIC_PREFIX "/raw_sensor/current"
#define TOPIC_HEARTBEAT   TOPIC_PREFIX "/heartbeat"
#define TOPIC_COMMAND     TOPIC_PREFIX "/command"

// ---------------------------------------------------------------------------
// micro-ROS error checking macro
// ---------------------------------------------------------------------------
// On error, sets the global error flag and blinks the LED rapidly.
#define RCCHECK(fn) { \
  rcl_ret_t rc = (fn); \
  if (rc != RCL_RET_OK) { \
    error_loop(rc); \
  } \
}
#define RCSOFTCHECK(fn) { \
  rcl_ret_t rc = (fn); \
  if (rc != RCL_RET_OK) { \
    /* Non-fatal: log but continue */ \
  } \
}

// ---------------------------------------------------------------------------
// Global state
// ---------------------------------------------------------------------------

// micro-ROS entities
static rcl_allocator_t allocator;
static rclc_support_t  support;
static rcl_node_t      node;
static rclc_executor_t executor;

// Publishers
static rcl_publisher_t pub_vibration;
static rcl_publisher_t pub_current;
static rcl_publisher_t pub_heartbeat;

// Subscriber
static rcl_subscription_t sub_command;

// Timers
static rcl_timer_t timer_vibration;
static rcl_timer_t timer_current;
static rcl_timer_t timer_heartbeat;

// Messages
static sensor_msgs__msg__Float32MultiArray msg_vibration;
static float vibration_data[3];  // X, Y, Z
static std_msgs__msg__Float32 msg_current;
static std_msgs__msg__UInt32  msg_heartbeat;
static std_msgs__msg__String  msg_command;

// State tracking
static uint32_t heartbeat_counter = 0;
static volatile bool estop_active  = false;
static volatile bool agent_connected = false;

// Calibration offsets (ADC mid-scale for zero-g / zero-current)
static const float VIB_OFFSET = 2048.0f;
static const float VIB_SCALE  = 0.00161f;  // ADC counts -> g
static const float CUR_OFFSET = 2048.0f;
static const float CUR_SCALE  = 0.0264f;   // ADC counts -> Amperes

// ---------------------------------------------------------------------------
// LED status indicator
// ---------------------------------------------------------------------------
// Blink patterns indicate the current system state:
//   - Solid ON      : Normal operation, agent connected
//   - Slow blink    : Connecting to WiFi / agent
//   - Fast blink    : Error state
//   - OFF           : E-stop active

enum class LedState {
  OFF,
  SOLID,
  SLOW_BLINK,
  FAST_BLINK
};

static LedState led_state = LedState::SLOW_BLINK;
static unsigned long led_last_toggle_ms = 0;

/// Update the LED based on the current state. Call from loop().
static void update_led() {
  unsigned long now = millis();
  switch (led_state) {
    case LedState::OFF:
      digitalWrite(MIRACLE_LED_PIN, LOW);
      break;
    case LedState::SOLID:
      digitalWrite(MIRACLE_LED_PIN, HIGH);
      break;
    case LedState::SLOW_BLINK:
      if (now - led_last_toggle_ms >= 500) {
        digitalWrite(MIRACLE_LED_PIN, !digitalRead(MIRACLE_LED_PIN));
        led_last_toggle_ms = now;
      }
      break;
    case LedState::FAST_BLINK:
      if (now - led_last_toggle_ms >= 100) {
        digitalWrite(MIRACLE_LED_PIN, !digitalRead(MIRACLE_LED_PIN));
        led_last_toggle_ms = now;
      }
      break;
  }
}

// ---------------------------------------------------------------------------
// Error handling
// ---------------------------------------------------------------------------
/// Enter an error loop. The LED blinks rapidly to signal a fault.
/// In production firmware, this would log the error code and attempt
/// a controlled restart after a timeout.
static void error_loop(rcl_ret_t error_code) {
  (void)error_code;  // Could be logged to NVS or serial
  led_state = LedState::FAST_BLINK;
  while (true) {
    update_led();
    delay(10);
  }
}

// ---------------------------------------------------------------------------
// WiFi connection
// ---------------------------------------------------------------------------
/// Connect to the factory WiFi network. Blocks until connected or times out.
static bool connect_wifi() {
  Serial.printf("[MIRACLE] Connecting to WiFi SSID: %s\n", MIRACLE_WIFI_SSID);
  led_state = LedState::SLOW_BLINK;

  WiFi.mode(WIFI_STA);
  WiFi.begin(MIRACLE_WIFI_SSID, MIRACLE_WIFI_PASSWORD);

  int attempts = 0;
  const int max_attempts = 60;  // 30 seconds (500 ms per attempt)

  while (WiFi.status() != WL_CONNECTED && attempts < max_attempts) {
    delay(500);
    Serial.print(".");
    update_led();
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("\n[MIRACLE] WiFi connected. IP: %s\n",
                  WiFi.localIP().toString().c_str());
    return true;
  } else {
    Serial.println("\n[MIRACLE] WiFi connection FAILED.");
    return false;
  }
}

// ---------------------------------------------------------------------------
// Timer callbacks
// ---------------------------------------------------------------------------

/// Vibration timer callback - reads 3-axis ADC and publishes.
/// Called at MIRACLE_VIBRATION_RATE_HZ (default 1000 Hz).
static void vibration_timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  (void)last_call_time;
  if (timer == NULL || estop_active) return;

  // Read the three vibration ADC channels and convert to g-force
  int raw_x = analogRead(MIRACLE_PIN_VIBRATION_X);
  int raw_y = analogRead(MIRACLE_PIN_VIBRATION_Y);
  int raw_z = analogRead(MIRACLE_PIN_VIBRATION_Z);

  vibration_data[0] = (static_cast<float>(raw_x) - VIB_OFFSET) * VIB_SCALE;
  vibration_data[1] = (static_cast<float>(raw_y) - VIB_OFFSET) * VIB_SCALE;
  vibration_data[2] = (static_cast<float>(raw_z) - VIB_OFFSET) * VIB_SCALE;

  // Publish (best-effort: ignore transient failures)
  RCSOFTCHECK(rcl_publish(&pub_vibration, &msg_vibration, NULL));
}

/// Current timer callback - reads spindle current ADC and publishes.
/// Called at MIRACLE_CURRENT_RATE_HZ (default 500 Hz).
static void current_timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  (void)last_call_time;
  if (timer == NULL || estop_active) return;

  int raw_current = analogRead(MIRACLE_PIN_CURRENT);
  msg_current.data = (static_cast<float>(raw_current) - CUR_OFFSET) * CUR_SCALE;

  RCSOFTCHECK(rcl_publish(&pub_current, &msg_current, NULL));
}

/// Heartbeat timer callback - publishes monotonic counter at 1 Hz.
/// Also checks WiFi and agent connectivity.
static void heartbeat_timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  (void)last_call_time;
  if (timer == NULL) return;

  msg_heartbeat.data = heartbeat_counter++;

  RCSOFTCHECK(rcl_publish(&pub_heartbeat, &msg_heartbeat, NULL));

  // Check WiFi health
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[MIRACLE] WiFi lost! Attempting reconnection...");
    led_state = LedState::SLOW_BLINK;
    connect_wifi();
  }

  // Reset watchdog timer
  esp_task_wdt_reset();
}

// ---------------------------------------------------------------------------
// Subscriber callbacks
// ---------------------------------------------------------------------------

/// Command subscriber callback - handles E-STOP, CALIBRATE, RESET commands.
static void command_callback(const void *msg_in) {
  const std_msgs__msg__String *msg =
      static_cast<const std_msgs__msg__String *>(msg_in);

  if (msg == NULL || msg->data.data == NULL) return;

  String cmd = String(msg->data.data);
  Serial.printf("[MIRACLE] Command received: %s\n", cmd.c_str());

  if (cmd.equalsIgnoreCase("ESTOP")) {
    // Emergency stop: halt all sensor publishing and enter safe state
    estop_active = true;
    led_state = LedState::OFF;
    Serial.println("[MIRACLE] *** E-STOP ACTIVATED ***");
  }
  else if (cmd.equalsIgnoreCase("CALIBRATE")) {
    // Trigger sensor calibration (in a real system, this would sample
    // quiescent ADC values and update the offset constants)
    Serial.println("[MIRACLE] Calibration requested (not yet implemented).");
  }
  else if (cmd.equalsIgnoreCase("RESET")) {
    // Clear E-stop and resume normal operation
    estop_active = false;
    led_state = LedState::SOLID;
    Serial.println("[MIRACLE] E-STOP cleared. Resuming operation.");
  }
  else {
    Serial.printf("[MIRACLE] Unknown command: %s\n", cmd.c_str());
  }
}

// ---------------------------------------------------------------------------
// Message initialisation
// ---------------------------------------------------------------------------
/// Set up the Float32MultiArray message for vibration data.
static void init_vibration_message() {
  // Point the message data buffer to our static array
  msg_vibration.data.data = vibration_data;
  msg_vibration.data.size = 3;
  msg_vibration.data.capacity = 3;

  // Layout: single dimension "axis" with size 3
  // Note: micro-ROS on MCU typically uses pre-allocated static memory.
  // The layout is informational for downstream subscribers.
  msg_vibration.layout.data_offset = 0;
  msg_vibration.layout.dim.data = NULL;
  msg_vibration.layout.dim.size = 0;
  msg_vibration.layout.dim.capacity = 0;
}

/// Allocate and initialise the command message string buffer.
static void init_command_message() {
  // Pre-allocate a buffer for incoming command strings
  static char cmd_buffer[64];
  msg_command.data.data = cmd_buffer;
  msg_command.data.size = 0;
  msg_command.data.capacity = sizeof(cmd_buffer);
}

// ---------------------------------------------------------------------------
// micro-ROS entity creation
// ---------------------------------------------------------------------------
/// Create all publishers, subscribers, and timers for this node.
static void create_entities() {
  // --- Allocator & support ------------------------------------------------
  allocator = rcl_get_default_allocator();

  // Initialise support with the configured ROS domain ID
  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  RCCHECK(rcl_init_options_init(&init_options, allocator));
  RCCHECK(rcl_init_options_set_domain_id(&init_options, MIRACLE_ROS_DOMAIN_ID));
  RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options,
                                         &allocator));

  // --- Node ---------------------------------------------------------------
  RCCHECK(rclc_node_init_default(&node, "miracle_sensor_esp32",
                                 "/miracle", &support));

  // --- Publishers ---------------------------------------------------------

  // Vibration publisher (best-effort QoS for high-frequency data)
  RCCHECK(rclc_publisher_init_best_effort(
      &pub_vibration, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Float32MultiArray),
      TOPIC_VIBRATION));

  // Current publisher (best-effort QoS)
  RCCHECK(rclc_publisher_init_best_effort(
      &pub_current, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      TOPIC_CURRENT));

  // Heartbeat publisher (reliable QoS for diagnostics)
  RCCHECK(rclc_publisher_init_default(
      &pub_heartbeat, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt32),
      TOPIC_HEARTBEAT));

  // --- Subscriber ---------------------------------------------------------

  // Command subscriber (reliable QoS for safety-critical messages)
  RCCHECK(rclc_subscription_init_default(
      &sub_command, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
      TOPIC_COMMAND));

  // --- Timers -------------------------------------------------------------

  const unsigned int vibration_period_ms = 1000 / MIRACLE_VIBRATION_RATE_HZ;
  const unsigned int current_period_ms   = 1000 / MIRACLE_CURRENT_RATE_HZ;
  const unsigned int heartbeat_period_ms = 1000 / MIRACLE_HEARTBEAT_RATE_HZ;

  RCCHECK(rclc_timer_init_default(
      &timer_vibration, &support,
      RCL_MS_TO_NS(vibration_period_ms),
      vibration_timer_callback));

  RCCHECK(rclc_timer_init_default(
      &timer_current, &support,
      RCL_MS_TO_NS(current_period_ms),
      current_timer_callback));

  RCCHECK(rclc_timer_init_default(
      &timer_heartbeat, &support,
      RCL_MS_TO_NS(heartbeat_period_ms),
      heartbeat_timer_callback));

  // --- Executor -----------------------------------------------------------
  // The executor manages callbacks for 3 timers + 1 subscription = 4 handles.
  const unsigned int num_handles = 4;
  RCCHECK(rclc_executor_init(&executor, &support.context, num_handles,
                             &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer_vibration));
  RCCHECK(rclc_executor_add_timer(&executor, &timer_current));
  RCCHECK(rclc_executor_add_timer(&executor, &timer_heartbeat));
  RCCHECK(rclc_executor_add_subscription(
      &executor, &sub_command, &msg_command,
      &command_callback, ON_NEW_DATA));
}

/// Destroy all micro-ROS entities for a clean shutdown / reconnect.
static void destroy_entities() {
  rcl_publisher_fini(&pub_vibration, &node);
  rcl_publisher_fini(&pub_current, &node);
  rcl_publisher_fini(&pub_heartbeat, &node);
  rcl_subscription_fini(&sub_command, &node);
  rcl_timer_fini(&timer_vibration);
  rcl_timer_fini(&timer_current);
  rcl_timer_fini(&timer_heartbeat);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

// ---------------------------------------------------------------------------
// Arduino setup
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(2000);  // Allow serial monitor to connect

  Serial.println("============================================");
  Serial.println(" MIRACLE ESP32 Sensor Bridge");
  Serial.printf(" Machine ID : %s\n", MIRACLE_MACHINE_ID);
  Serial.printf(" Agent      : %s:%d\n", MIRACLE_AGENT_IP, MIRACLE_AGENT_PORT);
  Serial.printf(" Domain ID  : %d\n", MIRACLE_ROS_DOMAIN_ID);
  Serial.println("============================================");

  // Configure LED pin
  pinMode(MIRACLE_LED_PIN, OUTPUT);
  led_state = LedState::SLOW_BLINK;

  // Configure ADC pins as inputs (ESP32 ADC1 channels are input-only)
  analogReadResolution(12);  // 12-bit ADC resolution (0-4095)
  analogSetAttenuation(ADC_11db);  // Full 0-3.3 V range

  // Initialise message structures
  init_vibration_message();
  init_command_message();

  // Connect to WiFi
  if (!connect_wifi()) {
    Serial.println("[MIRACLE] FATAL: Cannot connect to WiFi. Restarting...");
    delay(5000);
    ESP.restart();
  }

  // Configure micro-ROS WiFi transport
  set_microros_wifi_transports(
      const_cast<char *>(MIRACLE_WIFI_SSID),
      const_cast<char *>(MIRACLE_WIFI_PASSWORD),
      const_cast<char *>(MIRACLE_AGENT_IP),
      MIRACLE_AGENT_PORT);

  // Wait for agent to become available
  Serial.println("[MIRACLE] Waiting for micro-ROS agent...");
  while (rmw_uros_ping_agent(1000, 5) != RMW_RET_OK) {
    Serial.print(".");
    update_led();
    delay(500);
  }
  Serial.println("\n[MIRACLE] Agent connected!");
  agent_connected = true;

  // Create all ROS entities
  create_entities();

  // Enable hardware watchdog timer
  esp_task_wdt_init(MIRACLE_WDT_TIMEOUT_S, true);  // true = panic on timeout
  esp_task_wdt_add(NULL);  // Add current task to WDT

  // All systems go
  led_state = LedState::SOLID;
  Serial.println("[MIRACLE] Setup complete. Publishing sensor data.");
}

// ---------------------------------------------------------------------------
// Arduino main loop
// ---------------------------------------------------------------------------
void loop() {
  // Spin the micro-ROS executor to process all pending callbacks.
  // The timeout controls how long to wait for new data (100 ms max).
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));

  // Update LED status indicator
  update_led();

  // Periodically verify agent connectivity (every ~5 seconds via heartbeat)
  // If the agent disappears, attempt to reconnect.
  static unsigned long last_ping_ms = 0;
  unsigned long now = millis();
  if (now - last_ping_ms > 5000) {
    last_ping_ms = now;
    if (rmw_uros_ping_agent(500, 3) != RMW_RET_OK) {
      Serial.println("[MIRACLE] Agent connection lost! Reconnecting...");
      led_state = LedState::SLOW_BLINK;
      agent_connected = false;

      // Tear down and re-create entities
      destroy_entities();

      // Re-attempt connection
      while (rmw_uros_ping_agent(1000, 5) != RMW_RET_OK) {
        update_led();
        delay(500);
      }

      create_entities();
      agent_connected = true;
      led_state = LedState::SOLID;
      Serial.println("[MIRACLE] Agent reconnected.");
    }
  }

  // Feed the watchdog
  esp_task_wdt_reset();
}
