#include "ev3ctrl_train.hpp"
#include "ev3com.hpp"

static colorid_t color = COLOR_NONE;

static void do_move_foward(int power) {
    ER err = ev3_motor_set_power(arm_motor, power);
    ERR_CHECK(err);
    return;
}
static void do_stop()
{
    ER err = ev3_motor_stop(arm_motor);
    ERR_CHECK(err);
    return;
}

void do_train_ctrl()
{
    static int need_stop = 0;
    color = ev3_color_sensor_get_color();
    if (need_stop == 0 && color == COLOR_RED) {
        need_stop = 1;
        ev3_led_set_color(LED_RED);
        printf("STOP: RED\n");
    }
    if (need_stop) {
        do_stop();
        if (ev3_touch_sensor_is_pressed(0)) {
            need_stop = 0;
        }
    }
    else {
        ev3_led_set_color(LED_GREEN);
        do_move_foward(40);
    }
    return;
}
