#include <stdio.h>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"
#include <string.h>

using namespace std::chrono_literals;

static void clear_msg(geometry_msgs::msg::Twist *cmd)
{
  cmd->linear.x = 0;
  cmd->linear.y = 0;
  cmd->linear.z = 0;
  cmd->angular.x = 0;
  cmd->angular.y = 0;
  cmd->angular.z = 0;
}

int main(int argc, char **argv) {
  char buffer[20][4096];

  if (argc > 1) {
    sprintf(buffer[0], "%s_robo_node", argv[1]);
    sprintf(buffer[1], "%s_cmd_vel", argv[1]);
    sprintf(buffer[2], "%s_servo_base_angle", argv[1]);
    sprintf(buffer[3], "%s_servo_angle1", argv[1]);
    sprintf(buffer[4], "%s_servo_angle2", argv[1]);
    sprintf(buffer[5], "%s_servo_angle3", argv[1]);
    sprintf(buffer[6], "%s_servo_angle4", argv[1]);
    sprintf(buffer[7], "%s_pincher_cmd", argv[1]);
  }
  else {
    sprintf(buffer[0], "robo_node");
    sprintf(buffer[1], "cmd_vel");
    sprintf(buffer[2], "servo_base_angle");
    sprintf(buffer[3], "servo_angle1");
    sprintf(buffer[4], "servo_angle2");
    sprintf(buffer[5], "servo_angle3");
    sprintf(buffer[6], "servo_angle4");
    sprintf(buffer[7], "pincher_cmd");
  }
	printf("START\n");
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared(buffer[0]);
  auto publisher_motor =
      node->create_publisher<geometry_msgs::msg::Twist>(buffer[1], 1);
  auto publisher_servo_base =
      node->create_publisher<geometry_msgs::msg::Twist>(buffer[2], 1);
  auto publisher_servo1 =
      node->create_publisher<geometry_msgs::msg::Twist>(buffer[3], 1);
  auto publisher_servo2 =
      node->create_publisher<geometry_msgs::msg::Twist>(buffer[4], 1);
  auto publisher_servo3 =
      node->create_publisher<geometry_msgs::msg::Twist>(buffer[5], 1);
  auto publisher_servo4 =
      node->create_publisher<geometry_msgs::msg::Twist>(buffer[6], 1);
  auto publisher_pincher =
      node->create_publisher<geometry_msgs::msg::Twist>(buffer[7], 1);

  rclcpp::WallRate rate(10ms);

  geometry_msgs::msg::Twist cmd_vel_motor;
  geometry_msgs::msg::Twist cmd_vel_servo_base;
  geometry_msgs::msg::Twist cmd_vel_servo[4];
  geometry_msgs::msg::Twist cmd_vel_pinch;
  clear_msg(&cmd_vel_motor);
  clear_msg(&cmd_vel_servo_base);
  clear_msg(&cmd_vel_servo[0]);
  clear_msg(&cmd_vel_servo[1]);
  clear_msg(&cmd_vel_servo[2]);
  clear_msg(&cmd_vel_servo[3]);
  clear_msg(&cmd_vel_pinch);
	
  float param_angle[4];
  param_angle[0] = 5.0f;
  param_angle[1] = 3.0f;
  param_angle[2] = 0.5f;
  param_angle[3] = 1.0f;

  while (rclcpp::ok()) {
    printf("Up<n>: u<n>, Down<n>  : d<n>\n");
    printf("Left : l, Right : r, Stop : s\n");
    printf("PinchOpen : o, PinchClose : c\n");
    int key = getchar();
    printf("key=%c\n", key);
    switch (key) {
      case 'l':
        cmd_vel_servo_base.angular.y = -1.0f;
        break;
      case 'r':
        cmd_vel_servo_base.angular.y = 1.0f;
        break;
      case 's':
        cmd_vel_servo_base.angular.y = 0.0f;
        break;
      case 'u':
      case 'd':
    	{
		    int id = getchar();
		    printf("id=%c\n", id);
    		int index=id - '1';
	    	if (key == 'u') {
	        	cmd_vel_servo[index].angular.y = -1.0f * param_angle[index];
	    	}
	    	else {
		        cmd_vel_servo[index].angular.y = 1.0f * param_angle[index];
	    	}
    	}
        break;
      case 'o':
        cmd_vel_pinch.linear.x = 0.0f;
        break;
      case 'c':
        cmd_vel_pinch.linear.x = 1.0f;
        break;
      case ' ':
        clear_msg(&cmd_vel_servo_base);
        clear_msg(&cmd_vel_servo[0]);
        clear_msg(&cmd_vel_servo[1]);
        clear_msg(&cmd_vel_servo[2]);
        clear_msg(&cmd_vel_servo[3]);
        clear_msg(&cmd_vel_pinch);
        break;
      default:
        break;
    }

    publisher_motor->publish(cmd_vel_motor);
    publisher_servo_base->publish(cmd_vel_servo_base);
    publisher_servo1->publish(cmd_vel_servo[0]);
    publisher_servo2->publish(cmd_vel_servo[1]);
    publisher_servo3->publish(cmd_vel_servo[2]);
    publisher_servo4->publish(cmd_vel_servo[3]);
    publisher_pincher->publish(cmd_vel_pinch);
    rclcpp::spin_some(node);
    rate.sleep();
  }

	return 0;
}
