#include "ev3ctrl_base_practice.hpp"
#include "ev3com.hpp"

static int16_t ultrasonic_value = 0;

typedef enum {
  Practice2_Phase_First = 0,
  Practice2_Phase_Second,
  Practice2_Phase_Third,
  Practice2_Phase_Fourth,
  Practice2_Phase_Fifth,
  Practice2_Phase_Sixth,
} Practice2_PhaseType;
static Practice2_PhaseType Practice2_Phase;
static colorid_t color = COLOR_NONE;

/*
 * 走る(前進操作)
 */
void do_foward(int power) {
  ER err = ev3_motor_steer(left_motor, right_motor, power);
  ERR_CHECK(err);
  return;
}

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
  printf("ultrasonic_value=%d\n", ultrasonic_value);
  if (ultrasonic_value > 20) {
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
  if (ultrasonic_value < 15) {
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

void do_base_practice()
{
    do_arm_move(false);
    check_ultrasonic_sensor();
    check_color_sensor();
    if (ultrasonic_value < 5) {
        do_stop();
    } else {
        do_practice_2();
    }
    return;
}
