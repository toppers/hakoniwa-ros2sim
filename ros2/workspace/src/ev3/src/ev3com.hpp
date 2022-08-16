#ifndef _EV3COM_HPP_
#define _EV3COM_HPP_

#include "ev3_msgs/msg/ev3_pdu_actuator.hpp"
#include "ev3_msgs/msg/ev3_pdu_sensor.hpp"

#define left_motor  0
#define right_motor  1
#define arm_motor  2

#define color_sensor  0
#define ultrasonic_sensor  0
#define touch_sensor0  0
#define touch_sensor1  1


#define ERR_CHECK(err)                                              \
  do {                                                              \
    if ((err) != 0) {                                               \
      printf("ERROR: %s %d err=%d", __FUNCTION__, __LINE__, (err)); \
    }                                                               \
  } while (0)

typedef int ER;
typedef unsigned char uint8_t;
typedef short int16_t;

typedef enum {
  COLOR_NONE = 0,
  COLOR_BLACK,
  COLOR_BLUE,
  COLOR_GREEN,
  COLOR_YELLOW,
  COLOR_RED,
  COLOR_WHITE,
  COLOR_BROWN,
  TNUM_COLOR
} colorid_t;

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
extern ActuatorDataType actuator_data;

typedef struct {
  int16_t ultrasonic;
  colorid_t color;
  uint8_t reflect;
  bool touch[2];
} SensingDataType;
extern SensingDataType sensing_data;

extern ER ev3_motor_set_power(int id, int power);
extern void ev3_led_set_color(ledcolor_t color);
extern ER ev3_motor_stop(int id);
extern ER ev3_motor_steer(int id1, int id2, int power);
extern int16_t ev3_ultrasonic_sensor_get_distance();
extern colorid_t ev3_color_sensor_get_color();
extern bool ev3_touch_sensor_is_pressed(int id);


extern void topic_callback(const ev3_msgs::msg::Ev3PduSensor::SharedPtr msg);


#endif /* _EV3COM_HPP_ */