#include "ev3ctrl_train.hpp"
#include "ev3com.hpp"

#define STR(var)   #var

void clear_f(const int32_t line);
void init_f(const char *str);
void msg_f(const char *str, int32_t line);
void num_f(const int n, int32_t line);
void fmt_f(const char* fmt, const int n, int32_t line);

// 信号の種類
// 当座は進行と停止だけ
typedef enum _signal_type_t {
    SIGNAL_NONE,       // 未定
    SIGNAL_STOP,       // 停止
    SIGNAL_DEPARTURE,  // 進行
    SIGNAL_REDUCE,     // 減速
    SIGNAL_CAUTION,    // 注意
    SIGNAL_ALERT,      // 警戒
    TNUM_SIGNAL
} signal_type_t;


// static colorid_t color = COLOR_NONE;
#define drive_motor arm_motor 

// drive unit
#define DRIVE_UNIT_POWER 40
#define DRIVE_UNIT_SLOW_POWER 20
static int drive_unit_power = DRIVE_UNIT_POWER;

// static const motor_port_t drive_unit_motor_port = EV3_PORT_A;
// static const motor_type_t drive_unit_motor_type = MEDIUM_MOTOR;

void drive_unit_init(void) {
    // ev3_motor_config(drive_unit_motor_port, drive_unit_motor_type);
    drive_unit_power = DRIVE_UNIT_POWER;
}

void drive_unit_set_power(int power) {
    drive_unit_power = power;
}

void drive_unit_stop(void) {
    // 実装はpowerを0にして惰性で止まるだけ
    // なので、Powerが大きいとかなり進む
    ev3_motor_stop(drive_motor);
}

void drive_unit_forward(void) {
    // ev3_motor_set_power(drive_unit_motor_port, drive_unit_power);
    ev3_motor_set_power(drive_motor, drive_unit_power);
}

void drive_unit_back(void) {
    // ev3_motor_set_power(drive_unit_motor_port, -drive_unit_power);
    ev3_motor_set_power(drive_motor, -drive_unit_power);
}

// operation switch

// static const sensor_port_t operation_switch_sensor_port = EV3_PORT_2;
// static const sensor_type_t operation_switch_sensor_type = TOUCH_SENSOR;

// static bool saved = false;
#define operation_switch 0

void operation_switch_init(void) {
    // saved = false;
    // ev3_sensor_config(operation_switch_sensor_port, operation_switch_sensor_type);
}

bool operation_switch_is_pushed(void) {
    // return ev3_touch_sensor_is_pressed(operation_switch_sensor_port);
    return ev3_touch_sensor_is_pressed(operation_switch);    
}

// signal_reader

// static const sensor_port_t signal_reader_sensor_port = EV3_PORT_1;
// static const sensor_type_t signal_reader_sensor_type = COLOR_SENSOR;

void signal_reader_init(void) {
  // ev3_sensor_config(signal_reader_sensor_port, signal_reader_sensor_type);
  // dly_tsk(500U * 1000U);
}

static const char* color_names[TNUM_COLOR] = {
  " none", " black", " blue", " green",
  " yellow", " red", " white", " brown"
};

// 保存用の変数
// 初期値は列車の背景設定色（実機ではUNKNOWNになる）
// 列車のカラーセンサーの背景は白に変えてある
static colorid_t color_saved = COLOR_NONE;  

signal_type_t signal_reader_get_signal(void) {
    signal_type_t signal = SIGNAL_NONE;
    // colorid_t color = ev3_color_sensor_get_color(signal_reader_sensor_port);
    colorid_t color = ev3_color_sensor_get_color();
    // 下記で曖昧な色認識を都合に寄せる
    if(color != color_saved) {
        msg_f(color_names[color], 3);
    }
    color_saved = color;
    switch(color_saved) {
    case COLOR_RED:
        signal = SIGNAL_STOP;
        break;
    case COLOR_GREEN:
        signal = SIGNAL_DEPARTURE;
        break;
    case COLOR_YELLOW:
        signal = SIGNAL_REDUCE;
        break;
    default:
        signal = SIGNAL_NONE;
    }
    return signal;
}

typedef enum {
    TR_INIT,
    TR_FINISHED,
    TR_WAIT_FOR_DEPARTURE1, // 運転開始待ち
    TR_WAIT_FOR_DEPARTURE2, // 運転開始待ち
    TR_FORWARDING,          // 前進中
    TR_SLOW_DOWN,           // 減速走行中
    TR_STOP,
    TNUM_TRAIN_STATE
} train_state;

static const char* train_state_msg[TNUM_TRAIN_STATE] = {
    "TR_INIT", "TR_FINISHED", "TR_W_F_DEP1","TR_W_F_DEP2",
    "TR_FORWARDING", "TR_SLOW_DOWN", "TR_STOP"
};

static train_state tr_state = TR_INIT;
static bool tr_is_entry = true;

#define TR_ENTRY if(tr_is_entry){tr_is_entry=false;
#define TR_DO }{
#define TR_EVTCHK(f,s) if((f)){tr_state=(s);tr_is_entry=true;}
#define TR_EXIT }if(tr_is_entry){
#define TR_END }

void train_init(void) {
    drive_unit_init();
    signal_reader_init();
    operation_switch_init();
    tr_state = TR_INIT;
    tr_is_entry = true;
}

bool train_signal_is_stop(void) {
    return signal_reader_get_signal() == SIGNAL_STOP;
}

bool train_signal_is_slow_down(void) {
    return signal_reader_get_signal() == SIGNAL_REDUCE;
}

bool train_signal_is_departure(void) {
    return signal_reader_get_signal() == SIGNAL_DEPARTURE;
}

void train_run(void) {
    if( tr_is_entry ) {
        msg_f(train_state_msg[tr_state], 2);
    }

    switch( tr_state ) {
    case TR_INIT:
        TR_ENTRY
            train_init();
        TR_DO
        TR_EVTCHK(true,TR_WAIT_FOR_DEPARTURE1)
        TR_EXIT
        TR_END
        break;
    case TR_WAIT_FOR_DEPARTURE1:
        TR_ENTRY
            ev3_led_set_color(LED_ORANGE);
            // horn_warning();
        TR_DO
        TR_EVTCHK(operation_switch_is_pushed(),TR_WAIT_FOR_DEPARTURE2)
        TR_EXIT
        TR_END
        break;
    case TR_WAIT_FOR_DEPARTURE2:
        TR_ENTRY
        TR_DO
        TR_EVTCHK((!operation_switch_is_pushed()),TR_FORWARDING)
        TR_EXIT
        TR_END
        break;
    case TR_FORWARDING:
        TR_ENTRY
            ev3_led_set_color(LED_GREEN);
            // horn_confirmation();
            drive_unit_set_power(DRIVE_UNIT_POWER);
            drive_unit_forward();
        TR_DO
        TR_EVTCHK(train_signal_is_slow_down(),TR_SLOW_DOWN)
        TR_EVTCHK(train_signal_is_stop(),TR_STOP)
        TR_EVTCHK(operation_switch_is_pushed(),TR_FINISHED)
        TR_EXIT
        TR_END
        break;
    case TR_SLOW_DOWN:
        TR_ENTRY
            ev3_led_set_color(LED_ORANGE);
            drive_unit_set_power(DRIVE_UNIT_SLOW_POWER);
            drive_unit_forward();
            // horn_warning();
        TR_DO
        TR_EVTCHK(train_signal_is_stop(),TR_STOP)
        TR_EVTCHK(operation_switch_is_pushed(),TR_FINISHED)
        TR_EXIT
            drive_unit_stop();
        TR_END
        break;
    case TR_STOP:
        TR_ENTRY
            drive_unit_stop();
            ev3_led_set_color(LED_RED);
            // horn_arrived();
        TR_DO
        TR_EVTCHK(train_signal_is_departure(),TR_FORWARDING)
        TR_EVTCHK(operation_switch_is_pushed(),TR_FINISHED)
        TR_EXIT
        TR_END
        break;
    case TR_FINISHED:
        TR_ENTRY
            drive_unit_stop();
            ev3_led_set_color(LED_ORANGE);
            // horn_warning();
        TR_DO
        TR_EXIT
        TR_END
        break;
    default:
    case TNUM_TRAIN_STATE:
        break;
    }
}

void do_train_ctrl(void) {
    static bool is_initialized = false;
    if(! is_initialized ) {
        init_f("train");
        is_initialized = true;
    }

    train_run();
    
    // ext_tsk();
}
