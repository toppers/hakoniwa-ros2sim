#include <stdio.h>

#include "ev3_msgs/msg/ev3_pdu_actuator.hpp"
#include "ev3_msgs/msg/ev3_pdu_sensor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "ev3com.hpp"
#include "ev3ctrl_train.hpp"
#include "ev3ctrl_signal.hpp"
#include "ev3ctrl_base_practice.hpp"

using namespace std::chrono_literals;

int main(int argc, char **argv) {
  char buffer[3][4096];
  char *ctrl_op = (char*)"base_practice";

  if (argc > 1) {
    sprintf(buffer[0], "%s_ev3_node", argv[1]);
    sprintf(buffer[1], "%s_ev3_actuator", argv[1]);
    sprintf(buffer[2], "%s_ev3_sensor", argv[1]);
    if (argc == 3) {
      ctrl_op = argv[2];
    }
    printf("START: %s ctrl=%s\n", argv[1], ctrl_op);
  }
  else {
    sprintf(buffer[0], "ev3_node");
    sprintf(buffer[1], "ev3_actuator");
    sprintf(buffer[2], "ev3_sensor");
    printf("START\n");
  }
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared(buffer[0]);
  auto publisher =
      node->create_publisher<ev3_msgs::msg::Ev3PduActuator>(buffer[1], 1);
  auto subscriber = node->create_subscription<ev3_msgs::msg::Ev3PduSensor>(
      buffer[2], 1, topic_callback);

  // rclcpp::WallRate rate(5ms);
  rclcpp::WallRate rate(50ms);
    
  auto ros_actuator_data = ev3_msgs::msg::Ev3PduActuator();
  while (rclcpp::ok()) {

    if (strncmp("base_practice", ctrl_op, strlen("base_practice")) == 0) {
      do_base_practice();
    }
    else if (strncmp("train", ctrl_op, strlen("train")) == 0) {
      //printf("train-mode\n");
      do_train_ctrl();
    }
    else {
      //printf("signal-mode\n");
      do_signal_ctrl();
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
