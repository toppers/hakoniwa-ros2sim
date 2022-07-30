#include "ev3com.hpp"


/******************
 * Actuator
 ******************/
ActuatorDataType actuator_data;
ER ev3_motor_set_power(int id, int power) {
  actuator_data.motor_power[id] = power;
  return 0;
}
void ev3_led_set_color(ledcolor_t color) {
  actuator_data.led = color;
  return;
}

ER ev3_motor_stop(int id) {
  ev3_motor_set_power(id, 0);
  return 0;
}
ER ev3_motor_steer(int id1, int id2, int power) {
  ev3_motor_set_power(id1, power);
  ev3_motor_set_power(id2, power);
  return 0;
}

/******************
 * Sensor
 ******************/
SensingDataType sensing_data;
int16_t ev3_ultrasonic_sensor_get_distance() {
  return sensing_data.ultrasonic / 10;
}

colorid_t ev3_color_sensor_get_color() { return sensing_data.color; }

bool ev3_touch_sensor_is_pressed(int id) {
  return sensing_data.touch[id];
}


void topic_callback(const ev3_msgs::msg::Ev3PduSensor::SharedPtr msg) {
  sensing_data.ultrasonic = msg->sensor_ultrasonic;
  sensing_data.color = (colorid_t)msg->color_sensors[0].color;
  sensing_data.touch[0] = msg->touch_sensors[0].value;
  sensing_data.touch[1] = msg->touch_sensors[1].value;
  return;
}
