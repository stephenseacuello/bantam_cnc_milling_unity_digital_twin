// ============================================================================
// MIRACLE STM32 Sensor Bridge Firmware
// ============================================================================
// Acquires temperature, acoustic emission, and coolant flow data from
// analog sensors and publishes them to the MIRACLE ROS 2 system via
// micro-ROS over serial (UART) transport.
//
// Hardware:
//   - STM32 Nucleo F446RE
//   - PA0 (ADC1_IN0) : Temperature (thermocouple / NTC via conditioning)
//   - PA1 (ADC1_IN1) : Acoustic emission sensor
//   - PA2 (ADC1_IN2) : Coolant flow sensor
//   - LED_BUILTIN    : Status LED (Nucleo onboard LD2)
//
// Transport: Serial UART to micro-ROS agent at 921600 baud
// ============================================================================

#include <Arduino.h>

#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/string.h>
#include <std_msgs/msg/u_int32.h>
#include <std_msgs/msg/float32.h>

// ---------------------------------------------------------------------------
// Compile-time configuration (set via build_flags in platformio.ini)
// ---------------------------------------------------------------------------
#ifndef MIRACLE_SERIAL_BAUDRATE
  #define MIRACLE_SERIAL_BAUDRATE 921600
#endif
#ifndef MIRACLE_MACHINE_ID
  #define MIRACLE_MACHINE_ID "cnc_001"
#endif
#ifndef MIRACLE_ROS_DOMAIN_ID
  #define MIRACLE_ROS_DOMAIN_ID 42
#endif
#ifndef MIRACLE_TEMPERATURE_RATE_HZ
  #define MIRACLE_TEMPERATURE_RATE_HZ 10
#endif
#ifndef MIRACLE_ACOUSTIC_RATE_HZ
  #define MIRACLE_ACOUSTIC_RATE_HZ 1000
#endif
#ifndef MIRACLE_ACOUSTIC_RAW_RATE_HZ
  #define MIRACLE_ACOUSTIC_RAW_RATE_HZ 44100
#endif
#ifndef MIRACLE_COOLANT_RATE_HZ
  #define MIRACLE_COOLANT_RATE_HZ 10
#endif
#ifndef MIRACLE_HEARTBEAT_RATE_HZ
  #define MIRACLE_HEARTBEAT_RATE_HZ 1
#endif
#ifndef MIRACLE_PIN_TEMPERATURE
  #define MIRACLE_PIN_TEMPERATURE PA0
#endif
#ifndef MIRACLE_PIN_ACOUSTIC
  #define MIRACLE_PIN_ACOUSTIC PA1
#endif
#ifndef MIRACLE_PIN_COOLANT_FLOW
  #define MIRACLE_PIN_COOLANT_FLOW PA2
#endif
#ifndef MIRACLE_LED_PIN
  #define MIRACLE_LED_PIN LED_BUILTIN
#endif
#ifndef MIRACLE_WDT_TIMEOUT_S
  #define MIRACLE_WDT_TIMEOUT_S 10
#endif
#ifndef MIRACLE_ACOUSTIC_DOWNSAMPLE_WINDOW
  #define MIRACLE_ACOUSTIC_DOWNSAMPLE_WINDOW 44
#endif

// ---------------------------------------------------------------------------
// Topic name helpers
// ---------------------------------------------------------------------------
#define TOPIC_PREFIX "/miracle/" MIRACLE_MACHINE_ID
#define TOPIC_TEMPERATURE   TOPIC_PREFIX "/raw_sensor/temperature"
#define TOPIC_ACOUSTIC      TOPIC_PREFIX "/raw_sensor/acoustic"
#define TOPIC_COOLANT_FLOW  TOPIC_PREFIX "/raw_sensor/coolant_flow"
#define TOPIC_HEARTBEAT     TOPIC_PREFIX "/heartbeat_stm32"
#define TOPIC_COMMAND       TOPIC_PREFIX "/command"

// ---------------------------------------------------------------------------
// micro-ROS error checking macros
// ---------------------------------------------------------------------------
#define RCCHECK(fn) { \
  rcl_ret_t rc = (fn); \
  if (rc != RCL_RET_OK) { \
    error_loop(rc); \
  } \
}
#define RCSOFTCHECK(fn) { \
  rcl_ret_t rc = (fn); \
  if (rc != RCL_RET_OK) { \
    /* Non-fatal: continue operation */ \
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
static rcl_publisher_t pub_temperature;
static rcl_publisher_t pub_acoustic;
static rcl_publisher_t pub_coolant_flow;
static rcl_publisher_t pub_heartbeat;

// Subscriber
static rcl_subscription_t sub_command;

// Timers
static rcl_timer_t timer_temperature;
static rcl_timer_t timer_acoustic;
static rcl_timer_t timer_coolant_flow;
static rcl_timer_t timer_heartbeat;

// Messages
static std_msgs__msg__Float32 msg_temperature;
static std_msgs__msg__Float32 msg_acoustic;
static std_msgs__msg__Float32 msg_coolant_flow;
static std_msgs__msg__UInt32  msg_heartbeat;
static std_msgs__msg__String  msg_command;

// State tracking
static uint32_t heartbeat_counter = 0;
static volatile bool estop_active  = false;
static volatile bool agent_connected = false;

// Calibration constants
static const float TEMP_OFFSET = 0.0f;
static const float TEMP_SCALE  = 0.0806f;   // ADC -> degrees Celsius
static const float ACOU_OFFSET = 0.0f;
static const float ACOU_SCALE  = 0.0244f;   // ADC -> dB (gain-adjusted)
static const float COOL_OFFSET = 0.0f;
static const float COOL_SCALE  = 0.0098f;   // ADC -> L/min

// ---------------------------------------------------------------------------
// Acoustic emission downsampling buffer
// ---------------------------------------------------------------------------
// The acoustic sensor is sampled at 44.1 kHz but published at 1 kHz.
// We accumulate raw samples into a ring buffer and compute the RMS value
// over each window of MIRACLE_ACOUSTIC_DOWNSAMPLE_WINDOW samples.
static float acoustic_sample_buffer[MIRACLE_ACOUSTIC_DOWNSAMPLE_WINDOW];
static uint16_t acoustic_sample_index = 0;
static float    acoustic_rms_value    = 0.0f;

/// Accumulate a raw acoustic ADC sample and compute RMS when the window
/// is full. This function should be called from a hardware timer ISR or
/// DMA completion callback at the raw sample rate (44.1 kHz).
///
/// For this skeleton, we simulate the accumulation in the acoustic timer
/// callback, reading multiple ADC samples in a burst.
static void acoustic_accumulate_sample(float sample) {
  acoustic_sample_buffer[acoustic_sample_index++] = sample;

  if (acoustic_sample_index >= MIRACLE_ACOUSTIC_DOWNSAMPLE_WINDOW) {
    // Compute RMS over the window
    float sum_sq = 0.0f;
    for (uint16_t i = 0; i < MIRACLE_ACOUSTIC_DOWNSAMPLE_WINDOW; i++) {
      sum_sq += acoustic_sample_buffer[i] * acoustic_sample_buffer[i];
    }
    acoustic_rms_value = sqrtf(sum_sq / MIRACLE_ACOUSTIC_DOWNSAMPLE_WINDOW);
    acoustic_sample_index = 0;  // Reset for next window
  }
}

// ---------------------------------------------------------------------------
// LED status indicator
// ---------------------------------------------------------------------------
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
static void error_loop(rcl_ret_t error_code) {
  (void)error_code;
  led_state = LedState::FAST_BLINK;
  while (true) {
    update_led();
    delay(10);
  }
}

// ---------------------------------------------------------------------------
// Timer callbacks
// ---------------------------------------------------------------------------

/// Temperature timer callback - reads ADC and publishes.
/// Called at MIRACLE_TEMPERATURE_RATE_HZ (default 10 Hz).
static void temperature_timer_callback(rcl_timer_t *timer,
                                       int64_t last_call_time) {
  (void)last_call_time;
  if (timer == NULL || estop_active) return;

  int raw = analogRead(MIRACLE_PIN_TEMPERATURE);
  msg_temperature.data =
      (static_cast<float>(raw) - TEMP_OFFSET) * TEMP_SCALE;

  RCSOFTCHECK(rcl_publish(&pub_temperature, &msg_temperature, NULL));
}

/// Acoustic timer callback - performs burst sampling and publishes RMS.
/// Called at MIRACLE_ACOUSTIC_RATE_HZ (default 1000 Hz).
/// Each invocation reads MIRACLE_ACOUSTIC_DOWNSAMPLE_WINDOW ADC samples
/// to simulate the 44.1 kHz raw rate (44 samples per 1 ms window).
static void acoustic_timer_callback(rcl_timer_t *timer,
                                    int64_t last_call_time) {
  (void)last_call_time;
  if (timer == NULL || estop_active) return;

  // Burst-read the acoustic ADC channel to fill one downsample window.
  // In production firmware, this would use DMA + hardware timer for
  // precise 44.1 kHz sampling. Here we use sequential analogRead calls
  // as a functional approximation.
  for (uint16_t i = 0; i < MIRACLE_ACOUSTIC_DOWNSAMPLE_WINDOW; i++) {
    int raw = analogRead(MIRACLE_PIN_ACOUSTIC);
    float sample = (static_cast<float>(raw) - ACOU_OFFSET) * ACOU_SCALE;
    acoustic_accumulate_sample(sample);
  }

  // Publish the RMS value computed during accumulation
  msg_acoustic.data = acoustic_rms_value;
  RCSOFTCHECK(rcl_publish(&pub_acoustic, &msg_acoustic, NULL));
}

/// Coolant flow timer callback - reads ADC and publishes.
/// Called at MIRACLE_COOLANT_RATE_HZ (default 10 Hz).
static void coolant_flow_timer_callback(rcl_timer_t *timer,
                                        int64_t last_call_time) {
  (void)last_call_time;
  if (timer == NULL || estop_active) return;

  int raw = analogRead(MIRACLE_PIN_COOLANT_FLOW);
  msg_coolant_flow.data =
      (static_cast<float>(raw) - COOL_OFFSET) * COOL_SCALE;

  RCSOFTCHECK(rcl_publish(&pub_coolant_flow, &msg_coolant_flow, NULL));
}

/// Heartbeat timer callback - publishes monotonic counter at 1 Hz.
/// Also monitors serial link health.
static void heartbeat_timer_callback(rcl_timer_t *timer,
                                     int64_t last_call_time) {
  (void)last_call_time;
  if (timer == NULL) return;

  msg_heartbeat.data = heartbeat_counter++;

  RCSOFTCHECK(rcl_publish(&pub_heartbeat, &msg_heartbeat, NULL));
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

  if (cmd.equalsIgnoreCase("ESTOP")) {
    // Emergency stop: halt all sensor publishing and enter safe state
    estop_active = true;
    led_state = LedState::OFF;
  }
  else if (cmd.equalsIgnoreCase("CALIBRATE")) {
    // Trigger sensor zero-offset calibration
    // In production, this would sample quiescent ADC values and update
    // the offset constants stored in flash.
  }
  else if (cmd.equalsIgnoreCase("RESET")) {
    // Clear E-stop and resume normal operation
    estop_active = false;
    led_state = LedState::SOLID;
  }
}

// ---------------------------------------------------------------------------
// Message initialisation
// ---------------------------------------------------------------------------
/// Allocate and initialise the command message string buffer.
static void init_command_message() {
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
  RCCHECK(rclc_node_init_default(&node, "miracle_sensor_stm32",
                                 "/miracle", &support));

  // --- Publishers ---------------------------------------------------------

  // Temperature publisher (best-effort QoS)
  RCCHECK(rclc_publisher_init_best_effort(
      &pub_temperature, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      TOPIC_TEMPERATURE));

  // Acoustic publisher (best-effort QoS)
  RCCHECK(rclc_publisher_init_best_effort(
      &pub_acoustic, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      TOPIC_ACOUSTIC));

  // Coolant flow publisher (best-effort QoS)
  RCCHECK(rclc_publisher_init_best_effort(
      &pub_coolant_flow, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      TOPIC_COOLANT_FLOW));

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

  const unsigned int temp_period_ms    = 1000 / MIRACLE_TEMPERATURE_RATE_HZ;
  const unsigned int acou_period_ms    = 1000 / MIRACLE_ACOUSTIC_RATE_HZ;
  const unsigned int coolant_period_ms = 1000 / MIRACLE_COOLANT_RATE_HZ;
  const unsigned int hb_period_ms      = 1000 / MIRACLE_HEARTBEAT_RATE_HZ;

  RCCHECK(rclc_timer_init_default(
      &timer_temperature, &support,
      RCL_MS_TO_NS(temp_period_ms),
      temperature_timer_callback));

  RCCHECK(rclc_timer_init_default(
      &timer_acoustic, &support,
      RCL_MS_TO_NS(acou_period_ms),
      acoustic_timer_callback));

  RCCHECK(rclc_timer_init_default(
      &timer_coolant_flow, &support,
      RCL_MS_TO_NS(coolant_period_ms),
      coolant_flow_timer_callback));

  RCCHECK(rclc_timer_init_default(
      &timer_heartbeat, &support,
      RCL_MS_TO_NS(hb_period_ms),
      heartbeat_timer_callback));

  // --- Executor -----------------------------------------------------------
  // The executor manages callbacks for 4 timers + 1 subscription = 5 handles.
  const unsigned int num_handles = 5;
  RCCHECK(rclc_executor_init(&executor, &support.context, num_handles,
                             &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer_temperature));
  RCCHECK(rclc_executor_add_timer(&executor, &timer_acoustic));
  RCCHECK(rclc_executor_add_timer(&executor, &timer_coolant_flow));
  RCCHECK(rclc_executor_add_timer(&executor, &timer_heartbeat));
  RCCHECK(rclc_executor_add_subscription(
      &executor, &sub_command, &msg_command,
      &command_callback, ON_NEW_DATA));
}

/// Destroy all micro-ROS entities for a clean shutdown / reconnect.
static void destroy_entities() {
  rcl_publisher_fini(&pub_temperature, &node);
  rcl_publisher_fini(&pub_acoustic, &node);
  rcl_publisher_fini(&pub_coolant_flow, &node);
  rcl_publisher_fini(&pub_heartbeat, &node);
  rcl_subscription_fini(&sub_command, &node);
  rcl_timer_fini(&timer_temperature);
  rcl_timer_fini(&timer_acoustic);
  rcl_timer_fini(&timer_coolant_flow);
  rcl_timer_fini(&timer_heartbeat);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

// ---------------------------------------------------------------------------
// Arduino setup
// ---------------------------------------------------------------------------
void setup() {
  // The debug serial port (typically Serial, USB CDC on Nucleo)
  Serial.begin(115200);
  delay(2000);

  Serial.println("============================================");
  Serial.println(" MIRACLE STM32 Sensor Bridge");
  Serial.print(" Machine ID : "); Serial.println(MIRACLE_MACHINE_ID);
  Serial.print(" Domain ID  : "); Serial.println(MIRACLE_ROS_DOMAIN_ID);
  Serial.print(" Baudrate   : "); Serial.println(MIRACLE_SERIAL_BAUDRATE);
  Serial.println("============================================");

  // Configure LED pin
  pinMode(MIRACLE_LED_PIN, OUTPUT);
  led_state = LedState::SLOW_BLINK;

  // Configure ADC resolution (STM32F446RE supports 12-bit ADC)
  analogReadResolution(12);

  // Initialise message structures
  init_command_message();

  // Configure micro-ROS serial transport.
  // On the Nucleo F446RE, the micro-ROS transport uses the serial port
  // connected via the ST-Link USB interface. A second UART can be used
  // if the debug port is needed separately.
  set_microros_serial_transports(Serial);

  // Wait for micro-ROS agent to become available on the serial link
  Serial.println("[MIRACLE] Waiting for micro-ROS agent on serial...");
  while (rmw_uros_ping_agent(1000, 5) != RMW_RET_OK) {
    Serial.print(".");
    update_led();
    delay(500);
  }
  Serial.println("\n[MIRACLE] Agent connected via serial!");
  agent_connected = true;

  // Create all ROS entities
  create_entities();

  // Enable independent watchdog (IWDG) for the STM32.
  // Note: STM32 Arduino framework provides IWatchdog in some board
  // packages. For portability, this is left as a placeholder.
  // IWatchdog.begin(MIRACLE_WDT_TIMEOUT_S * 1000000);  // microseconds

  // All systems go
  led_state = LedState::SOLID;
  Serial.println("[MIRACLE] Setup complete. Publishing sensor data.");
}

// ---------------------------------------------------------------------------
// Arduino main loop
// ---------------------------------------------------------------------------
void loop() {
  // Spin the micro-ROS executor to process all pending callbacks.
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));

  // Update LED status indicator
  update_led();

  // Periodically verify agent connectivity
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
      Serial.println("[MIRACLE] Agent reconnected via serial.");
    }
  }

  // Feed the watchdog
  // IWatchdog.reload();
}
