#include "ev3ctrl_signal.hpp"
#include "ev3com.hpp"

// 演習用のユーティリティ

const int8_t util_line_height = 20;

// 初期処理用
void init_f(const char *str) {
    // フォントの設定と0行目の表示
//    ev3_lcd_set_font(EV3_FONT_MEDIUM);
//    ev3_lcd_draw_string(str, 0, 0);
//     syslog(LOG_NOTICE, str);
    puts(str);
}

/**
 * 引数の行を消去
 * @param line 20ドットごとの行番号（1から5）
 */
void clear_f(const int32_t line) {
//    ev3_lcd_fill_rect(0, line * line_height,
//                      EV3_LCD_WIDTH, line_height,
//                      EV3_LCD_WHITE);
    (void)line;  // dummy for unused
}

/**
 * 行単位で引数の文字列を表示
 * @param str 表示する文字列
 * @param line 20ドットごとの行番号（1から5）
 */
void msg_f(const char *str, int32_t line) {
//    clear_f(line);
//    ev3_lcd_draw_string(str, 0, line * line_height);
//    syslog(LOG_NOTICE, str);
    puts(str);
    (void)line;  // dummy for unused
}

/**
 * 行単位で引数の数値を表示
 * @param n 表示する数値
 * @param line 20ドットごとの行番号（1から5）
 */
void num_f(const int n, int32_t line) {
    static char buf[25] = {0};
    snprintf(buf, sizeof(buf), "%d", n);
//    clear_f(line);
//    ev3_lcd_draw_string(buf, 0, line * line_height);
//    syslog(LOG_NOTICE, buf);
    puts(buf);
    (void)line;  // dummy for unused
}

/**
 * 行単位でコメント付きで引数の数値を表示
 * @param fmt 編集用文字列(１つだけ%dを含められる）
 * @param n 表示する数値
 * @param line 20ドットごとの行番号（1から5）
 */
void fmt_f(const char* fmt, const int n, int32_t line) {
    static char buf[25] = {0};
    sprintf(buf, fmt, n);
//    clear_f(line);
//    ev3_lcd_draw_string(buf, 0, line * line_height);
//    syslog(LOG_NOTICE, buf);
    puts(buf);
    (void)line;  // dummy for unused
}

// manual_switch

// static const sensor_port_t manual_switch_sensor_port = EV3_PORT_1;
// static const sensor_type_t manual_switch_sensor_type = TOUCH_SENSOR;
#define manual_switch_sensor_port 0

static bool manual_switch_saved = false;

void manual_switch_init(void) {
    manual_switch_saved = false;
    // 本来は、手動スイッチに使う touch sensor の初期化
    // ev3_sensor_config(manual_switch_sensor_port, manual_switch_sensor_type);
}

bool manual_switch_is_pushed(void) {
    return ev3_touch_sensor_is_pressed(manual_switch_sensor_port);
}

// rotator (rotate signal display panel)

typedef enum _rotator_state {
    RS_INIT,
    RS_CHECKER_OFF,    // checkerがOFFで回転中
    RS_CHECKER_ON,     // checkerがONで回転中
    RS_STOP,           // 停止中
    TNUM_ROTATOR_STATE
} rotator_state;

static const char* rotator_state_msg[TNUM_ROTATOR_STATE] = {
    "RS_INIT", "RS_CHKR_OFF", "RS_CHKR_ON", "RS_STOP"
};

static rotator_state rs_state = RS_INIT;
static bool rs_is_entry = true;

#define RS_ENTRY if(rs_is_entry){rs_is_entry=false;
#define RS_DO }{
#define RS_EVTCHK(f,s) if((f)){rs_state=(s);rs_is_entry=true;}
#define RS_EXIT }if(rs_is_entry){
#define RS_END }

// static const sensor_port_t checker_port = EV3_PORT_2;
// static const sensor_type_t checker_type = TOUCH_SENSOR;
int checker_port = 1;

bool checker_is_pushed(void) {
    return ev3_touch_sensor_is_pressed(checker_port);
}

#define ROTATOR_POWER 10
int rotator_power = ROTATOR_POWER;

// static const motor_port_t motor_port = EV3_PORT_A;
// static const motor_type_t motor_type = LARGE_MOTOR;
#define motor_port left_motor

static bool rotator_rotating = false;

bool rotator_is_rotated(void) {
    return rs_state == RS_STOP;
}

void rotator_init(void) {
    // 本来は、motor と touch sensor の初期化
    // ev3_sensor_config(checker_port, checker_type);
    // ev3_motor_config(motor_port, motor_type);
    rotator_rotating = false;
}

void rotator_rotate(void) {
    rotator_rotating = true;
}

void rotator_stop(void) {
    rotator_rotating = false;
}

void rotator_run(void) {
    if( rs_is_entry ) {
        msg_f(rotator_state_msg[rs_state], 3);
    }
    switch(rs_state ) {
    case RS_INIT:
        RS_ENTRY
            rotator_init();
        RS_DO
        RS_EVTCHK(true, RS_CHECKER_OFF)
        RS_EXIT
        RS_END
        break;
    case RS_CHECKER_OFF:
        RS_ENTRY
            rotator_rotating = true;
            ev3_motor_set_power(motor_port, rotator_power);
        RS_DO
        RS_EVTCHK(checker_is_pushed(),RS_CHECKER_ON)
        RS_EXIT
        RS_END
        break;
    case RS_CHECKER_ON:
        RS_ENTRY
            rotator_rotating = true;
            ev3_motor_set_power(motor_port, rotator_power);
        RS_DO
        RS_EVTCHK((!checker_is_pushed()),RS_STOP)
        RS_EXIT
            // ev3_motor_stop(motor_port, true);
            ev3_motor_stop(motor_port);
        RS_END
        break;
    case RS_STOP:
        RS_ENTRY
            rotator_rotating = false;
        RS_DO
        RS_EVTCHK(rotator_rotating,RS_CHECKER_OFF)
        RS_EXIT
        RS_END
        break;
    default:
    case TNUM_ROTATOR_STATE:
        break;
    }
}

// Train Detector
#define TD_THRESHOLD 10
static int train_detector_threshold = TD_THRESHOLD;

static int16_t train_detector_dist_old = 0;

bool train_detector_init(void) {
    // 本来は、ultrasonic sensor の初期化処理
    train_detector_dist_old = ev3_ultrasonic_sensor_get_distance();
    if(train_detector_dist_old > train_detector_threshold) {
        return true;
    }
    return false;
}

bool train_detector_is_detected(void) {
    bool ret = false;
    int16_t distance = ev3_ultrasonic_sensor_get_distance();
    if (distance < train_detector_threshold) {
        ret = true;
    } 
    if(distance != train_detector_dist_old) {
        printf("distance=%d, %d\n", distance, train_detector_dist_old);
    }
    train_detector_dist_old = distance;
    return ret;
}

//  signal display

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

typedef enum _signal_display_state {
    SD_INIT,
    SD_WAIT_FOR_STOP,     // 停止へ切替中
    SD_STOP,              // 停止現示中
    SD_WAIT_FOR_DEP,      // 進行へ切替中
    SD_DEPARTURE,         // 進行現示中
    TNUM_SIGNAL_DISPLAY_STATE
} signal_display_state;

static const char* signal_display_state_msg[TNUM_SIGNAL_DISPLAY_STATE] = {
    "SD_INIT",
    "SD_W_F_STOP", "SD_STOP",
    "SD_W_F_DEP", "SD_DEPARTURE"
};

static const char* color_names[TNUM_COLOR] = {
  " none", " black", " blue", " green",
  " yellow", " red", " white", " brown"
};

static signal_display_state sd_state = SD_INIT;
static bool sd_is_entry = true;

#define SD_ENTRY if(sd_is_entry){sd_is_entry=false;
#define SD_DO }{
#define SD_EVTCHK(f,s) if((f)){sd_state=(s);sd_is_entry=true;}
#define SD_EXIT }if(sd_is_entry){
#define SD_END }

// static const sensor_port_t reader_sensor_port = EV3_PORT_3;
// static const sensor_type_t reader_sensor_type = COLOR_SENSOR;

static colorid_t last_color = COLOR_NONE;

signal_type_t signal_display_get_current_signal(void) {
    signal_type_t ret = SIGNAL_NONE;
    switch(last_color) {
    case COLOR_RED:
        ret = SIGNAL_STOP;
        break;
    case COLOR_GREEN:
        ret = SIGNAL_DEPARTURE;
        break;
    default:
        ret = SIGNAL_NONE;
    }
    return ret;
}

typedef enum _signal_display_oparation_t {
    SOP_STOP, SOP_DEP
} signal_display_oparation_t;

static signal_display_oparation_t signal_operation = SOP_STOP;

void signal_display_set_operation_stop(void) {
    signal_operation = SOP_STOP;
}

void signal_display_set_operation_departure(void) {
    signal_operation = SOP_DEP;
}

bool signal_display_is_operation_req_stop(void) {
    return signal_operation == SOP_STOP;
}

bool signal_display_is_operation_req_departure(void) {
    return signal_operation == SOP_DEP;
}

void signal_display_init(void) {
    // ev3_sensor_config(reader_sensor_port, reader_sensor_type);
    // dly_tsk(50U * 1000U);
    sd_state = SD_INIT;
    signal_operation = SOP_STOP;
}

void signal_display_run(void) {
    if( sd_is_entry ) {
        msg_f(signal_display_state_msg[sd_state], 2);
    }
    colorid_t color = ev3_color_sensor_get_color();
    // 経過の色を無視して都合の４色に寄せる
    switch(color) {
      case COLOR_RED:      case COLOR_GREEN:
      case COLOR_BLUE:     case COLOR_YELLOW:
        last_color = color;
        break;
      default:
        break;
    }

    switch( sd_state ) {
   case SD_INIT:
        SD_ENTRY
            signal_display_init();
        SD_DO
        SD_EVTCHK(true, SD_WAIT_FOR_STOP)
        SD_EXIT
        SD_END
        break;
    case SD_WAIT_FOR_STOP:
        SD_ENTRY
            rotator_rotate();
            ev3_led_set_color(LED_ORANGE);
        SD_DO
        SD_EVTCHK(signal_display_is_operation_req_departure(),SD_WAIT_FOR_DEP)
        // manual EVTCHK
            if(rotator_is_rotated()) {
                // last_color = ev3_color_sensor_get_color(reader_sensor_port);
                // last_color = ev3_color_sensor_get_color();
                msg_f(color_names[last_color], 4);
                if( last_color == COLOR_RED) {
                    sd_state= SD_STOP;
                } else {
                    sd_state= SD_WAIT_FOR_STOP;
                }
                sd_is_entry=true;
            }
        SD_EXIT
        SD_END
        break;
    case SD_STOP:
        SD_ENTRY
            rotator_stop();
            ev3_led_set_color(LED_RED);
        SD_DO
        SD_EVTCHK(signal_display_is_operation_req_departure(),SD_WAIT_FOR_DEP)
        SD_EXIT
        SD_END
        break;
    case SD_WAIT_FOR_DEP:
        SD_ENTRY
            rotator_rotate();
            ev3_led_set_color(LED_ORANGE);
        SD_DO
        SD_EVTCHK(signal_display_is_operation_req_stop(),SD_WAIT_FOR_STOP)
        // manual EVTCHK
            if(rotator_is_rotated()) {
                // last_color = ev3_color_sensor_get_color(reader_sensor_port);
                // last_color = ev3_color_sensor_get_color();
                msg_f(color_names[last_color], 4);
                if( last_color == COLOR_GREEN) {
                    sd_state= SD_DEPARTURE;
                } else {
                    sd_state= SD_WAIT_FOR_DEP;
                }
                sd_is_entry=true;
            }
        SD_EXIT
        SD_END
        break;
    case SD_DEPARTURE:
        SD_ENTRY
            rotator_stop();
            ev3_led_set_color(LED_GREEN);
        SD_DO
        SD_EVTCHK(signal_display_is_operation_req_stop(),SD_WAIT_FOR_STOP)
        SD_EXIT
        SD_END
        break;
    default:
    case TNUM_SIGNAL_DISPLAY_STATE:
        break;
    }

    rotator_run();

}

// block signal

typedef enum _block_signal_state {
    BS_INIT,
    BS_TO_STOP,     // 停止表示へ変更中
    BS_STOPPED,     // 停止を現時中
    BS_TO_DEP,      // 進行表示へ変更中
    BS_DEPARTURE,   // 進行を現時中
    TNUM_BLOCK_SIGNAL_STATE
} block_signal_state;

static const char* block_signal_state_msg[TNUM_BLOCK_SIGNAL_STATE] = {
    "BS_INIT", "BS_TO_STOP", "BS_STOPPED",
    "BS_TP_DEP", "BS_DEPARTURE"
};

static block_signal_state bs_state = BS_INIT;
static bool bs_is_entry = true;

#define BS_ENTRY if(bs_is_entry){bs_is_entry=false;
#define BS_DO }{
#define BS_EVTCHK(f,s) if((f)){bs_state=(s);bs_is_entry=true;}
#define BS_EXIT }if(bs_is_entry){
#define BS_END }

bool signal_display_is_stop(void) {
    return signal_display_get_current_signal() == SIGNAL_STOP;
}

bool signal_display_is_departure(void) {
    return signal_display_get_current_signal() == SIGNAL_DEPARTURE;
}

void block_signal_init(void) {
    manual_switch_init();
    train_detector_init();
    // dly_tsk(1000U * 100U);
    // horn_confirmation();
    bs_state = BS_INIT;
    bs_is_entry = true;
}

void block_signal_run(void) {
    if( bs_is_entry ) {
        msg_f(block_signal_state_msg[bs_state], 1);
    }

    switch( bs_state ) {
    case BS_INIT:
        BS_ENTRY
            block_signal_init();
        BS_DO
        BS_EVTCHK(true,BS_TO_STOP)
        BS_EXIT
        BS_END
        break;
    case BS_TO_STOP:
        BS_ENTRY
            signal_display_set_operation_stop();
        BS_DO
        BS_EVTCHK(signal_display_is_stop(),BS_STOPPED)
        // EVTCHK(司令室からの指示を受け取った(),BS_STOPPED)
        BS_EXIT
        BS_END
        break;
    case BS_STOPPED:
        BS_ENTRY
        BS_DO
        BS_EVTCHK(manual_switch_is_pushed(),BS_TO_DEP)
        // EVTCHK(司令室からの指示を受け取った(),BS_TO_DEP)
        BS_EXIT
        BS_END
        break;
    case BS_TO_DEP:
        BS_ENTRY
            signal_display_set_operation_departure();
        BS_DO
        BS_EVTCHK(signal_display_is_departure(),BS_DEPARTURE)
        // EVTCHK(司令室からの指示を受け取った(),BS_DEPARTURE)
        BS_END
        break;
    case BS_DEPARTURE:
        BS_ENTRY
        BS_DO
        BS_EVTCHK(manual_switch_is_pushed(),BS_TO_STOP)
        BS_EVTCHK(train_detector_is_detected(),BS_TO_STOP)
        // EVTCHK(司令室からの指示を受け取った(),BS_TO_STOP)
        BS_EXIT
        BS_END
        break;
    default:
    case TNUM_BLOCK_SIGNAL_STATE:
        break;
    }

    signal_display_run(); // 信号表示部のステートマシンの実行
}

// signal control 
void do_signal_ctrl(void) {
    static bool initialized = false;
    // static bool detected_old = false;
    static bool detected = false;
  
    if( !initialized ) {
        detected = train_detector_init();
        if( detected ) {
            initialized = true;
        }
        return;
    }

    block_signal_run();

    /***
    rotator_run();
    if(manual_switch_is_pushed()) {
        rotator_rotate();
    }
     ***/

    /***
    detected = train_detector_is_detected();
    if( detected && !detected_old) {
        printf("Train detected\n");
        printf("dist_old=%d\n", train_detector_dist_old);
    }
    detected_old = detected;
     ***/

    /*** for signal_display test
    signal_display_run();
    if(manual_switch_is_pushed()) {
        signal_type_t signal = signal_display_get_current_signal();
        switch(signal) {
        case SIGNAL_STOP:
            signal_display_set_operation_departure();
            break;
        case SIGNAL_DEPARTURE:
            signal_display_set_operation_stop();
            break;
        default:
            break;
        }
    }
    ***/

    // ext_tsk();
}
