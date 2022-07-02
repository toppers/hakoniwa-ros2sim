#include <stdio.h>

#include "ev3_msgs/msg/ev3_pdu_actuator.hpp"
#include "ev3_msgs/msg/ev3_pdu_sensor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#define ERR_CHECK(err)                                              \
  do {                                                              \
    if ((err) != 0) {                                               \
      printf("ERROR: %s %d err=%d", __FUNCTION__, __LINE__, (err)); \
    }                                                               \
  } while (0)

typedef int ER;

/*
 * 1.5 カラーセンサの値を見る
 */
typedef enum {
  COLOR_NONE = 0,
  COLOR_BLACK,
  COLOR_BLUE,
  COLOR_GREEN,
  COLOR_YELLOW,
  COLOR_RED,
  COLOR_WHITE,
  COLOR_BROWN,
} colorid_t;
colorid_t color = COLOR_NONE;
static const int left_motor = 0;
static const int right_motor = 1;
static const int arm_motor = 2;

static const int color_sensor = 0;
static const int ultrasonic_sensor = 0;
static const int touch_sensor0 = 0;
static const int touch_sensor1 = 1;

/******************
 * Actuator
 ******************/
typedef enum {
  LED_OFF = 0,
  LED_RED,
  LED_GREEN,
  LED_ORANGE,
} ledcolor_t;
typedef struct {
  int motor_power[3];
  ledcolor_t led;
} ActuatorDataType;
static ActuatorDataType actuator_data;
ER ev3_motor_set_power(int id, int power) {
  actuator_data.motor_power[id] = power;
  return 0;
}
static void ev3_led_set_color(ledcolor_t color) {
  actuator_data.led = color;
  return;
}

ER ev3_motor_stop(int id) {
  ev3_motor_set_power(id, 0);
  return 0;
}
static ER ev3_motor_steer(int id1, int id2, int power) {
  ev3_motor_set_power(id1, power);
  ev3_motor_set_power(id2, power);
  return 0;
}

/******************
 * Sensor
 ******************/
typedef short int16_t;
typedef struct {
  int16_t ultrasonic;
  colorid_t color;
  uint8_t reflect;
  bool touch[2];
} SensingDataType;
static SensingDataType sensing_data;
static int16_t ultrasonic_value = 0;
static int16_t ev3_ultrasonic_sensor_get_distance() {
  return sensing_data.ultrasonic / 10;
}

static colorid_t ev3_color_sensor_get_color() { return sensing_data.color; }
typedef unsigned char uint8_t;

static bool ev3_touch_sensor_is_pressed(int id) {
  return sensing_data.touch[id];
}

/*
 * 1.1 走る(前進操作)
 */
static void do_foward(int power) {
  ER err = ev3_motor_steer(left_motor, right_motor, power);
  ERR_CHECK(err);
  return;
}

typedef enum {
  Practice2_Phase_First = 0,
  Practice2_Phase_Second,
  Practice2_Phase_Third,
  Practice2_Phase_Fourth,
  Practice2_Phase_Fifth,
  Practice2_Phase_Sixth,
} Practice2_PhaseType;
static Practice2_PhaseType Practice2_Phase;
static void check_color_sensor(void) {
  color = ev3_color_sensor_get_color();
  printf("mode=%d :", Practice2_Phase + 1);
  switch (color) {
    case COLOR_NONE:
      break;
    case COLOR_BLACK:
      printf("BLACK\n");
      break;
    case COLOR_BLUE:
      printf("BLUE\n");
      break;
    case COLOR_GREEN:
      printf("GREEN\n");
      break;
    case COLOR_YELLOW:
      printf("YELLOW\n");
      break;
    case COLOR_RED:
      printf("RED\n");
      break;
    case COLOR_WHITE:
      printf("WHITE\n");
      break;
    case COLOR_BROWN:
      printf("BROWN\n");
      break;
    default:
      break;
  }
  return;
}
/*
 * 1.6 超音波センサの値を見る
 */
static void check_ultrasonic_sensor(void) {
  ultrasonic_value = ev3_ultrasonic_sensor_get_distance();
  return;
}
/*
 * 1.7 タッチセンサの値を見る
 */
bool is_pressed[2] = {false, false};
static void check_touch_sensor(int id) {
  int inx = (id == touch_sensor0) ? 0 : 1;
  is_pressed[inx] = ev3_touch_sensor_is_pressed(id);
  return;
}

/*
 * 1.3 止まる(停止操作)
 */
static void do_stop(void) {
  ER err = ev3_motor_stop(left_motor);
  ERR_CHECK(err);
  err = ev3_motor_stop(right_motor);
  ERR_CHECK(err);
  do_foward(0);
  printf("do_stop!!\n");
  return;
}
/*
 * 1.4 アームを動かす
 */
static void do_arm_move(bool up) {
  if (up) {
    ER err = ev3_motor_set_power(arm_motor, 5);
    ERR_CHECK(err);
  } else {
    ER err = ev3_motor_set_power(arm_motor, -5);
    ERR_CHECK(err);
  }
  return;
}
/*
 * 1.2 曲がる(ステアリング操作)
 */
static void do_turn(int turn_speed) {
  if (turn_speed > 0) {
    ER err = ev3_motor_set_power(left_motor, turn_speed);
    ERR_CHECK(err);
    err = ev3_motor_set_power(right_motor, 0);
    ERR_CHECK(err);
  } else {
    ER err = ev3_motor_set_power(left_motor, 0);
    ERR_CHECK(err);
    err = ev3_motor_set_power(right_motor, -turn_speed);
    ERR_CHECK(err);
  }
  return;
}

static void do_practice_2_first(void) {
  check_color_sensor();
  switch (color) {
    case COLOR_BLACK:
      do_foward(5);
      break;
    case COLOR_WHITE:
      do_turn(5);
      break;
    case COLOR_RED:
      do_stop();
      Practice2_Phase = Practice2_Phase_Second;
      break;
    default:
      do_turn(-5);
      break;
  }
  return;
}
static void do_practice_2_second(void) {
  check_ultrasonic_sensor();
  check_color_sensor();
  if (ultrasonic_value > 10) {
    do_foward(5);
    return;
  }

  switch (color) {
    case COLOR_BLACK:
      do_stop();
      Practice2_Phase = Practice2_Phase_Third;
      break;
    default:
      break;
  }
  return;
}
static void do_practice_2_third(void) {
  check_ultrasonic_sensor();
  if (ultrasonic_value < 12) {
    do_turn(5);
    return;
  }
  Practice2_Phase = Practice2_Phase_Fourth;
  return;
}
static void do_practice_2_Fourth(void) {
  check_color_sensor();
  switch (color) {
    case COLOR_BLACK:
      do_foward(5);
      break;
    case COLOR_WHITE:
      do_turn(-5);
      break;
    case COLOR_BLUE:
      do_stop();
      Practice2_Phase = Practice2_Phase_Fifth;
      break;
    default:
      do_turn(5);
      break;
  }
  return;
}
static void do_practice_2_Fifh(void) {
  check_ultrasonic_sensor();
  if (ultrasonic_value < 10) {
    do_stop();
    printf("GOAL!!\n");
    Practice2_Phase = Practice2_Phase_Sixth;
  } else {
    switch (color) {
      case COLOR_BLUE:
      case COLOR_BLACK:
        do_foward(5);
        break;
      case COLOR_WHITE:
        do_turn(-5);
        break;
      default:
        do_turn(5);
        break;
    }
  }
  return;
}
static void do_practice_2_Sixth(void) {
  check_ultrasonic_sensor();
  printf("distance=%d\n", ultrasonic_value);
  if (ultrasonic_value < 10) {
    do_stop();
  } else {
    do_foward(5);
  }
}
#define BLINK_CYCLE 20
#define BLINK_CYCLE_HALF (BLINK_CYCLE / 2)

static void blink_led(ledcolor_t b_color) {
  static int count = 0;
  check_touch_sensor(touch_sensor1);
  if (is_pressed[1] == true) {
    ev3_led_set_color(b_color);
    return;
  }

  if (count <= BLINK_CYCLE_HALF) {
    ev3_led_set_color(b_color);
  } else if (count > BLINK_CYCLE_HALF) {
    ev3_led_set_color(LED_OFF);
  }
  count++;
  if (count >= BLINK_CYCLE) {
    count = 0;
  }
  return;
}
static void do_practice_2(void) {
  switch (Practice2_Phase) {
    case Practice2_Phase_First:
      blink_led(LED_GREEN);
      do_practice_2_first();
      break;
    case Practice2_Phase_Second:
      blink_led(LED_ORANGE);
      do_practice_2_second();
      break;
    case Practice2_Phase_Third:
      blink_led(LED_RED);
      do_practice_2_third();
      break;
    case Practice2_Phase_Fourth:
      blink_led(LED_RED);
      do_practice_2_Fourth();
      break;
    case Practice2_Phase_Fifth:
      blink_led(LED_GREEN);
      do_practice_2_Fifh();
      break;
    case Practice2_Phase_Sixth:
      ev3_led_set_color(LED_OFF);
      do_practice_2_Sixth();
      break;
    default:
      do_stop();
      break;
  }
  return;
}

using namespace std::chrono_literals;

static void topic_callback(const ev3_msgs::msg::Ev3PduSensor::SharedPtr msg) {
  sensing_data.ultrasonic = msg->sensor_ultrasonic;
  sensing_data.color = (colorid_t)msg->color_sensors[0].color;
  sensing_data.touch[0] = msg->touch_sensors[0].value;
  sensing_data.touch[1] = msg->touch_sensors[1].value;
  return;
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("ev3_pubnode");
  auto publisher =
      node->create_publisher<ev3_msgs::msg::Ev3PduActuator>("ev3_actuator", 1);
  auto subscriber = node->create_subscription<ev3_msgs::msg::Ev3PduSensor>(
      "ev3_sensor", 1, topic_callback);

  rclcpp::WallRate rate(100ms);

  auto ros_actuator_data = ev3_msgs::msg::Ev3PduActuator();
  while (rclcpp::ok()) {
    {
      do_arm_move(false);
      check_ultrasonic_sensor();
      check_color_sensor();
      if (ultrasonic_value < 5) {
        do_stop();
      } else {
        do_practice_2();
      }
    }
    ros_actuator_data.leds[0] = actuator_data.led;
    ros_actuator_data.motors[0].power = actuator_data.motor_power[0];
    ros_actuator_data.motors[1].power = actuator_data.motor_power[1];
    ros_actuator_data.motors[2].power = actuator_data.motor_power[2];
    publisher->publish(ros_actuator_data);
    rclcpp::spin_some(node);
    rate.sleep();
  }
  rclcpp::shutdown();
  return 0;
}
