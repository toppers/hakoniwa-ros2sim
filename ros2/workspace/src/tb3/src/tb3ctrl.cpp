#include <stdio.h>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

typedef struct {
  double ranges[360];
} ScanDataType;

static ScanDataType scan_data;

static void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  int i;
  for (i = 0; i < 360; i++) {
    scan_data.ranges[i] = msg->ranges[i];
  }
  return;
}

static geometry_msgs::msg::Twist cmd_vel;

static float get_forward_distance(void) {
  int i;
  float min = 100.0f;
  for (i = 0; i < 15; i++) {
    if (scan_data.ranges[i] < min) {
      min = scan_data.ranges[i];
    }
  }
  for (i = (360 - 15); i < 360; i++) {
    if (scan_data.ranges[i] < min) {
      min = scan_data.ranges[i];
    }
  }
  // printf("forward: %lf\n", min);
  return min;
}

static float get_right_distance(void) {
  int i;
  float min = 100.0f;
  for (i = (90 - 30); i < (90 + 30); i++) {
    if (scan_data.ranges[i] < min) {
      min = scan_data.ranges[i];
    }
  }
  // printf("right: %lf\n", min);
  return min;
}

static bool do_forward(void) {
  bool is_stop = false;
  cmd_vel.linear.x = 0;
  if (get_forward_distance() < 0.2f) {
    cmd_vel.linear.x = 0;
    is_stop = true;
  } else {
    cmd_vel.linear.x = 0.5f;
  }

  return is_stop;
}

static bool turn_right(void) {
  bool is_stop = false;
  cmd_vel.angular.z = 0;
  if (get_right_distance() < 0.05f) {
    cmd_vel.angular.z = -0.01;
    is_stop = true;
  } else {
    cmd_vel.angular.z = 0;
  }

  return is_stop;
}

static void do_control(void) {
  (void)do_forward();
  (void)turn_right();

  if (cmd_vel.linear.x == 0 && cmd_vel.angular.z == 0) {
    cmd_vel.angular.z = -1.0f;
  }
  return;
}

using namespace std::chrono_literals;

int main(int argc, char **argv) {
  char buffer[3][4096];

  if (argc > 1) {
    sprintf(buffer[0], "%s_tb3_node", argv[1]);
    sprintf(buffer[1], "%s_cmd_vel", argv[1]);
    sprintf(buffer[2], "%s_scan", argv[1]);
    printf("START: %s\n", argv[1]);
  }
  else {
    sprintf(buffer[0], "tb3_node");
    sprintf(buffer[1], "cmd_vel");
    sprintf(buffer[2], "scan");
    printf("START\n");
  }
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared(buffer[0]);
  auto publisher =
      node->create_publisher<geometry_msgs::msg::Twist>(buffer[1], 1);
  auto subscriber = node->create_subscription<sensor_msgs::msg::LaserScan>(
      buffer[2], 1, scanCallback);

  rclcpp::WallRate rate(10ms);

  while (rclcpp::ok()) {
    do_control();
    publisher->publish(cmd_vel);

    rclcpp::spin_some(node);
    rate.sleep();
  }
  return 0;
}
