/*
 * MEGA DRIVE CLASSIC CONTROLLER firmware
 * 
 * 
 * by IODUM
 * 
 * version 1.1.2
 *
*/

//************ Адреса ******************
#define CONSOLE_TYPE_REG 0xFE // W / 0x03 для SNES, 0x01 для WII

//********** Типы консолей deviceID[4] 0xFE ************
#define WII_TYPE 0x01
#define SNES_TYPE 0x03

// режим установки турбо кнопок
#define CONFIG_MODE_DELAY 4 //sec;
#define TURBO_INTERVAL 67 // msec

// состояния кнопок для переключения режимов
#define ENTER_CONFIG_0 0xED
#define ENTER_CONFIG_1 0xEF
#define RESET_CONFIG_0 0xAF
#define RESET_CONFIG_1 0xFF
#define EXIT_CONFIG_0 0xFB
#define EXIT_CONFIG_1 0xFF
#define nADD_TURBO_1 0x01
#define nREMOVE_TURBO_0 0x40

// порты подключения кнопок на плате type S/C
#define BTN_MODE 14 //PC0
#define BTN_UP 16   //PC2
#define BTN_LEFT 17 //PC3
#define BTN_RIGHT 0 //PD0
#define BTN_DOWN 1  //PD1
#define BTN_START 2 //PD2
#define BTN_A 5     //PD5
#define BTN_X 6     //PD6
#define BTN_B 7     //PD7
#define BTN_Y 8     //PB0
#define BTN_C 9     //PB1
#define BTN_Z 10    //PB2

// порты подключения пинов гейпада сеги (type L)
#define UP_Z_BTN 3       //PD3
#define DOWN_Y_BTN 4     //PD4
#define LEFT_X_BTN 5     //PD5
#define RIGHT_MODE_BTN 6 //PD6
#define A_B_BTN 7        //PD7
#define START_C_BTN 9    //PB1
#define SELECT_PIN 8     //PB0
// задержки для корректного чтения состояния кнопок
#define SCAN_LOOP_DELAY 17
#define SCAN_STEP_DELAY 10 //микросекунды

// задержка между циклами чтения состояния кнопок
#define BUTTONS_SCAN_DELAY 2 // миллисекунды

// задержка для устранения дребезга кнопок
#define DEBOUNCE_DELAY 3 //микросекунды