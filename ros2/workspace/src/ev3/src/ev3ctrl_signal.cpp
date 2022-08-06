#include "ev3ctrl_signal.hpp"
#include "ev3com.hpp"


static void do_move_foward(int power) {
    ER err = ev3_motor_set_power(left_motor, power);
    ERR_CHECK(err);
    return;
}
static void do_stop()
{
    ER err = ev3_motor_stop(left_motor);
    ERR_CHECK(err);
    return;
}

static bool do_signal_move_until_changed(colorid_t current_color, colorid_t expect_color)
{
    static int touch_sensor_count = 0;
    static bool prev_touch_sensor = false;
    bool is_pressed = ev3_touch_sensor_is_pressed(1);
    if ((prev_touch_sensor == false) && is_pressed) {
        touch_sensor_count++;
        printf("touch_sensor_count=%d\n", touch_sensor_count);
    }
    prev_touch_sensor = is_pressed;
    if (touch_sensor_count > 0) {
        //TODO check color sensor
        touch_sensor_count = 0;
        if (current_color == expect_color) {
            do_stop();
            return true;
        }
        else {
            return false;
        }
    }
    else {
        do_move_foward(20);
        return false;
    }
}

static colorid_t get_expect_color(colorid_t color)
{
    if (color == COLOR_RED) {
        return COLOR_GREEN;
    }
    else {
        return COLOR_RED;
    }
}

static bool is_signal_changing = false;
static bool do_signal_change(bool start_trigger)
{
    colorid_t current_color = COLOR_RED; 
    static colorid_t expect_color = COLOR_RED;

    current_color = ev3_color_sensor_get_color();
    
    if (is_signal_changing == false) {
        if (start_trigger) {
            is_signal_changing = true;
            expect_color = get_expect_color(current_color);
            printf("START SIGNAL CHANGE\n");
        }
    }
    else {
        bool ret = do_signal_move_until_changed(current_color, expect_color);
        if (ret == true) {
            is_signal_changing = false;
            printf("END SIGNAL CHANGE\n");
            return true;
        }
    }
    return false;
}

void do_signal_ctrl()
{
    bool start_trigger = false;
    static bool is_stopping_mode = false;
    if (is_stopping_mode == false) {
        start_trigger = ev3_touch_sensor_is_pressed(0);
    }
    else {
        int16_t distance = ev3_ultrasonic_sensor_get_distance();
        printf("distance=%d\n", distance);
        if (distance < 10) {
            start_trigger = true;
        }
    }
    bool ret = do_signal_change(start_trigger);
    if (ret == true) {
        is_stopping_mode = (is_stopping_mode == true) ? false : true;
        printf("is_stopping_mode=%d\n", is_stopping_mode);
    }
    return;
}
