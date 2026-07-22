#include <micro_ros_arduino.h>
#include <ESP32Servo.h>
#include <HardwareSerial.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/vector3.h>
#include <std_msgs/msg/float64.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_BME280.h>
// #include <Adafruit_MPU6050.h>
// #include <SparkFun_Qwiic_OLED.h> //http://librarymanager/All#SparkFun_Qwiic_OLED
#include "MS5837.h"

#define SEALEVELPRESSURE_HPA (1013.25)
#define RCCHECK(fn) ((fn) == RCL_RET_OK)

#define INDICATOR_LED_PIN 0 // green LED: micro-ROS link status
#define RED_LED_PIN 5        // red LED: fault/attention by priority (leak > sensor fault > armed)
#define LEAK_PIN 15
#define DEBUG_TX_PIN 17
#define DEBUG_RX_PIN 16
#define ESC_MIN 1100
#define ESC_MAX 1900
#define LIGHT_MIN 1100
#define LIGHT_MAX 1900
#define CAM_SERVO_MIN 0
#define CAM_SERVO_MAX 180

//========================== Pinout =========================================================
Servo esc1, esc2, esc3, esc4, esc5, esc6;
Servo light_single, light_couple, light_single_pair;
Servo cam_servo;
MS5837 bar100;
Adafruit_BNO055 bno = Adafruit_BNO055(55);
Adafruit_BME280 bme;
HardwareSerial MainSerial(0);  // Declaring a Serial object on UART0
HardwareSerial DebugSerial(1); // Use UART1 for debugging

int esc_pins[] = {27, 26, 25, 33, 32, 14};
int light_pins[] = {18, 19}; // light_single, light_couple
int cam_servo_pin = 13;
int light_single_pair_pin = 4; // extra light, mirrors light_couple = the single light (B button)

//========================== Publishers definitions =========================================
rcl_publisher_t leak_pub;
std_msgs__msg__Float64 leak_msg;
rcl_publisher_t bno055_pub;
geometry_msgs__msg__Twist bno055_msg;
rcl_publisher_t bar100_pub;
geometry_msgs__msg__Vector3 bar100_msg;
rcl_publisher_t bme280_pub;
geometry_msgs__msg__Vector3 bme280_msg;
//========================== Subscribers definitions ========================================
rcl_subscription_t motors_sub;
rcl_subscription_t lights_servo_sub;
geometry_msgs__msg__Twist motors_msg;
geometry_msgs__msg__Vector3 lights_servo_msg;
//========================== ROS variables ==================================================
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t sensor_timer;
enum states {
    WAITING_AGENT,
    AGENT_AVAILABLE,
    AGENT_CONNECTED,
    AGENT_DISCONNECTED
} state;

unsigned long delayTime;
bool bar100_initialized = false;
bool bme280_initialized = false;
bool bno055_initialized = false;
volatile bool g_armed = false;   // any thruster commanded off-neutral (props may spin)

// Status-LED blink patterns; applyLed() renders them with millis() so nothing blocks.
enum LedPattern { LED_OFF, LED_SOLID, LED_SLOW, LED_FAST };


//========================== Subscribers Callbacks ==========================================
void subscription_callback_motors(const void *msgin) {
    const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
    int motor1_val = static_cast<int>(msg->linear.x);
    int motor2_val = static_cast<int>(msg->linear.y);
    int motor3_val = static_cast<int>(msg->linear.z);
    int motor4_val = static_cast<int>(msg->angular.x);
    int motor5_val = static_cast<int>(msg->angular.y);
    int motor6_val = static_cast<int>(msg->angular.z);

    // "Armed" for the red status LED: any valid motor command more than ~20 us
    // off neutral means a thruster is being driven (props may spin).
    int mvals[6] = {motor1_val, motor2_val, motor3_val, motor4_val, motor5_val, motor6_val};
    bool armed = false;
    for (int i = 0; i < 6; i++) {
        if (mvals[i] >= ESC_MIN && mvals[i] <= ESC_MAX && abs(mvals[i] - 1500) > 20) armed = true;
    }
    g_armed = armed;

    if (motor1_val >= ESC_MIN && motor1_val <= ESC_MAX) {
        esc1.writeMicroseconds(motor1_val);
    }
    if (motor2_val >= ESC_MIN && motor2_val <= ESC_MAX) {
        esc2.writeMicroseconds(motor2_val);
    }
    if (motor3_val >= ESC_MIN && motor3_val <= ESC_MAX) {
        esc3.writeMicroseconds(motor3_val);
    }
    if (motor4_val >= ESC_MIN && motor4_val <= ESC_MAX) {
        esc4.writeMicroseconds(motor4_val);
    }
    if (motor5_val >= ESC_MIN && motor5_val <= ESC_MAX) {
        esc5.writeMicroseconds(motor5_val);
    }
    if (motor6_val >= ESC_MIN && motor6_val <= ESC_MAX) {
        esc6.writeMicroseconds(motor6_val);
    }
}

void subscription_callback_lights_servo(const void *msgin) {
    const geometry_msgs__msg__Vector3 *msg = (const geometry_msgs__msg__Vector3 *)msgin;
    int light_single_val = static_cast<int>(msg->x);
    int light_couple_val = static_cast<int>(msg->y);
    int cam_servo_val = static_cast<int>(msg->z);

    if (light_single_val >= LIGHT_MIN && light_single_val <= LIGHT_MAX) {
        light_single.writeMicroseconds(light_single_val);
    }
    if (light_couple_val >= LIGHT_MIN && light_couple_val <= LIGHT_MAX) {
        light_couple.writeMicroseconds(light_couple_val);
        // GPIO 4 light pairs with the PHYSICAL single light (B button). Note the
        // code names are reversed vs the wiring: light_single (msg->x, A button)
        // is the 2-light pair; light_couple (msg->y, B button) is the single light.
        light_single_pair.writeMicroseconds(light_couple_val);
    }
    if (cam_servo_val >= CAM_SERVO_MIN && cam_servo_val <= CAM_SERVO_MAX) {
        cam_servo.write(cam_servo_val);
    }
}

void sensors_timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
    RCLC_UNUSED(last_call_time);
    sensors_event_t orientationData;
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);

    bno055_msg.linear.x = static_cast<double>(orientationData.orientation.x);
    bno055_msg.linear.y = static_cast<double>(orientationData.orientation.y);
    bno055_msg.linear.z = static_cast<double>(orientationData.orientation.z);
    bno055_msg.angular.x = 0.0;
    bno055_msg.angular.y = 0.0;
    bno055_msg.angular.z = 0.0;
    if (!RCCHECK(rcl_publish(&bno055_pub, &bno055_msg, NULL))) {
        DebugSerial.println("Failed publishing bno055 message");
    }

    leak_msg.data = (double)digitalRead(LEAK_PIN);
    if (!RCCHECK(rcl_publish(&leak_pub, &leak_msg, NULL))) {
        DebugSerial.println("Failed publishing leak message");
    }
    if (leak_msg.data) {
        DebugSerial.println("!!! LEAK DETECTED !!!");
    }

    if (bar100_initialized) {
        bar100.read();
        bar100_msg.x = static_cast<double>(bar100.depth());
        bar100_msg.y = static_cast<double>(bar100.pressure());
        bar100_msg.z = static_cast<double>(bar100.temperature());
        if (!RCCHECK(rcl_publish(&bar100_pub, &bar100_msg, NULL))) {
            DebugSerial.println("Failed publishing bar100 message");
        }
    }

    if (bme280_initialized) {
        bme280_msg.x = static_cast<double>(bme.readTemperature());
        bme280_msg.y = static_cast<double>(bme.readPressure() / 100.0);
        bme280_msg.z = static_cast<double>(bme.readHumidity());
        if (!RCCHECK(rcl_publish(&bme280_pub, &bme280_msg, NULL))) {
            DebugSerial.println("Failed publishing bme280 message");
        }
    }
}

void cleanup_ros_entities() {
    rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
    (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
    rcl_timer_fini(&sensor_timer);
    rcl_publisher_fini(&bno055_pub, &node);
    rcl_publisher_fini(&leak_pub, &node);
    rcl_publisher_fini(&bar100_pub, &node);
    rcl_publisher_fini(&bme280_pub, &node);
    rcl_subscription_fini(&motors_sub, &node);
    rcl_subscription_fini(&lights_servo_sub, &node);
    rclc_executor_fini(&executor);
    rclc_support_fini(&support);
    rcl_node_fini(&node);
}

bool setup_node_and_entities() {
    DebugSerial.println("____________________________________________\nStarting Setup:");
    set_microros_transports();
    allocator = rcl_get_default_allocator();
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, "esp32_node", "", &support));
    RCCHECK(rclc_publisher_init_default(&bno055_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/esp32/bno055_data"));
    RCCHECK(rclc_publisher_init_default(&bar100_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3), "/esp32/bar100_data"));
    RCCHECK(rclc_publisher_init_default(&leak_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64), "/esp32/leak"));
    RCCHECK(rclc_publisher_init_default(&bme280_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3), "/esp32/bme280_data"));
    RCCHECK(rclc_timer_init_default(&sensor_timer, &support, RCL_MS_TO_NS(50), sensors_timer_callback));
    RCCHECK(rclc_executor_init(&executor, &support.context, 6, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &sensor_timer));
    RCCHECK(rclc_subscription_init_default(&motors_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/motor_data"));
    RCCHECK(rclc_executor_add_subscription(&executor, &motors_sub, &motors_msg, subscription_callback_motors, ON_NEW_DATA));
    RCCHECK(rclc_subscription_init_default(&lights_servo_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3), "/lights_servo_data"));
    RCCHECK(rclc_executor_add_subscription(&executor, &lights_servo_sub, &lights_servo_msg, subscription_callback_lights_servo, ON_NEW_DATA));
    DebugSerial.println("____________________________________________");
    return true;
}

void setup() {
    set_microros_transports();
    pinMode(INDICATOR_LED_PIN, OUTPUT);
    pinMode(RED_LED_PIN, OUTPUT);
    pinMode(LEAK_PIN, INPUT);
    digitalWrite(INDICATOR_LED_PIN, LOW);  // updateStatusLeds() drives both from loop()
    digitalWrite(RED_LED_PIN, LOW);
    state = WAITING_AGENT;
    DebugSerial.begin(115200, SERIAL_8N1, DEBUG_RX_PIN, DEBUG_TX_PIN);
    MainSerial.begin(115200);
    DebugSerial.println("Starting setup...");
    // Give the ESP32Servo library all 4 hardware timers BEFORE attaching servos.
    // Without this, driving many servos (6 ESC + 2 lights + cam) leaves the later
    // attaches (e.g. cam_servo) without a timer, so they silently fail — a single
    // servo works, but the full 9-servo setup does not.
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);
    esc1.attach(esc_pins[0]);
    esc2.attach(esc_pins[1]);
    esc3.attach(esc_pins[2]);
    esc4.attach(esc_pins[3]);
    esc5.attach(esc_pins[4]);
    esc6.attach(esc_pins[5]);
    cam_servo.setPeriodHertz(50);
    cam_servo.attach(cam_servo_pin, 500, 2400);
    light_single.attach(light_pins[0]);
    light_couple.attach(light_pins[1]);
    light_single_pair.attach(light_single_pair_pin);
    esc1.writeMicroseconds(1500);
    esc2.writeMicroseconds(1500);
    esc3.writeMicroseconds(1500);
    esc4.writeMicroseconds(1500);
    esc5.writeMicroseconds(1500);
    esc6.writeMicroseconds(1500);
    light_single.writeMicroseconds(1100);
    light_couple.writeMicroseconds(1100);
    light_single_pair.writeMicroseconds(1100);
    cam_servo.write(90);

    Wire.begin();
    if (bar100.init()) {
        bar100_initialized = true;
        bar100.setFluidDensity(997);
        DebugSerial.println("Bar100 is connected.");
    } else {
        bar100_initialized = false;
        DebugSerial.println("Bar100 failed to initialize.");
    }
    if (!bno.begin()) {
        bno055_initialized = false;
        DebugSerial.println("Failed to find BNO055 chip.");
    } else {
        bno055_initialized = true;
        bno.setExtCrystalUse(true);
    }
    if (!bme.begin(0x77)) {
        bme280_initialized = false;
        DebugSerial.println("BME280 failed to initialize.");
    } else {
        bme280_initialized = true;
        DebugSerial.println("BME280 connected.");
    }
    DebugSerial.println("Finished Setup.");
    DebugSerial.println("Waiting for Agent...");
}

// Render a pattern on an LED using millis(), so it never blocks the loop.
void applyLed(int pin, LedPattern pat) {
    bool on = false;
    switch (pat) {
        case LED_SOLID: on = true; break;
        case LED_SLOW:  on = (millis() % 1000) < 500; break;   // ~1 Hz
        case LED_FAST:  on = (millis() % 250)  < 125; break;   // ~4 Hz
        case LED_OFF:
        default:        on = false; break;
    }
    digitalWrite(pin, on ? HIGH : LOW);
}

// Drive both status LEDs from the current state. Called every loop().
//   GREEN (GPIO0) = micro-ROS link: solid=connected, slow-blink=searching, off=down.
//   RED   (GPIO5) = fault/attention, highest priority wins:
//                   fast=LEAK, solid=sensor fault, slow=armed, off=all clear.
void updateStatusLeds() {
    LedPattern green;
    if (state == AGENT_CONNECTED)         green = LED_SOLID;
    else if (state == AGENT_DISCONNECTED) green = LED_OFF;
    else                                  green = LED_SLOW;   // WAITING / AVAILABLE = searching
    applyLed(INDICATOR_LED_PIN, green);

    // Leak is read straight off the pin (not via ROS), so the alarm still works
    // even when the ESP32 is disconnected from the agent.
    bool leak = digitalRead(LEAK_PIN);
    bool sensor_fault = !bno055_initialized || !bar100_initialized || !bme280_initialized;

    LedPattern red;
    if (leak)              red = LED_FAST;
    else if (sensor_fault) red = LED_SOLID;
    else if (g_armed)      red = LED_SLOW;
    else                   red = LED_OFF;
    applyLed(RED_LED_PIN, red);
}

void loop() {
    updateStatusLeds();
    switch (state) {
        case WAITING_AGENT:
            if (RMW_RET_OK == rmw_uros_ping_agent(1000, 3)) {
                state = AGENT_AVAILABLE;
                DebugSerial.println("Found Agent!");
            }
            break;
        case AGENT_AVAILABLE:
            setup_node_and_entities();
            state = AGENT_CONNECTED;
            DebugSerial.println("Connected to Agent!");
            break;
        case AGENT_CONNECTED:
            if (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) {
                rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
            } else {
                DebugSerial.println("Disconnected from Agent");
                state = AGENT_DISCONNECTED;
            }
            break;
        case AGENT_DISCONNECTED:
            cleanup_ros_entities();
            state = WAITING_AGENT;
            break;
        default:
            break;
    }
}
