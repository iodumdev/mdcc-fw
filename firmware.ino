/*
 * MEGA DRIVE CLASSIC CONTROLLER firmware
 * by IODUM
 *
 * version 1.1.2
 *
*/

//#define TYPE_L // !!! ТИП ПЛАТЫ. Закомментить для type S/C!!!
//#define DEBUG

#include <wiimoteext.h>
#include <avr/wdt.h>

// ID classic controller
byte deviceID[6] = {0x00, 0x00, 0xA4, 0x20, 0x01, 0x01};

// calibration data
unsigned char cal_data[32] = {
    0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00};

//************ Адреса ******************
#define INIT1_REG 0xF0        // W / 0x55-начало иниц-и, 0xAA-включение кодир-я
#define INIT2_REG 0xFB        // W / 0x00
#define ID_REG 0xFA           // R
#define CONSOLE_TYPE_REG 0xFE // W / 0x03 для SNES, 0x01 для WII
#define KEY1_REG 0x40         // W
#define KEY2_REG 0x46         // W
#define KEY3_REG 0x4C         // W
#define CONFIRM1_REG 0x20     // R
#define CONFIRM2_REG 0x30     // R
#define READ_BUTTONS_REG 0x00 // R

//********** Типы консолей deviceID[4] 0xFE ************
#define WII_TYPE 0x01
#define SNES_TYPE 0x03

// состояние кнопок
//byte buttons_state_mini[8] = {0x7F, 0x7F, 0x7F, 0x7F, 0x00, 0x00, 0xFF, 0xFF};
//byte buttons_state_wii[6] = {0x5F, 0xDF, 0x8F, 0x00, 0xFF, 0xFF};
byte controller_report[8] = {0x5F, 0xDF, 0x8F, 0x00, 0xFF, 0xFF, 0x00, 0x00};
byte buttons_state[2] = {0xFF, 0xFF};

// режим установки турбо кнопок
bool config_mode = false;
uint8_t config_push_time = 0;
#define CONFIG_MODE_DELAY 4 //sec

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
#define BUTTONS_SCAN_DELAY 10 // миллисекунды

/***********************************************************
 * Инициализация геймпада
************************************************************/
int gamepadInit()
{

//--- type S/C ---------------
#ifndef TYPE_L
  pinMode(BTN_UP, INPUT_PULLUP);
  pinMode(BTN_DOWN, INPUT_PULLUP);
  pinMode(BTN_LEFT, INPUT_PULLUP);
  pinMode(BTN_RIGHT, INPUT_PULLUP);
  pinMode(BTN_START, INPUT_PULLUP);
  pinMode(BTN_MODE, INPUT_PULLUP);
  pinMode(BTN_A, INPUT_PULLUP);
  pinMode(BTN_B, INPUT_PULLUP);
  pinMode(BTN_C, INPUT_PULLUP);
  pinMode(BTN_X, INPUT_PULLUP);
  pinMode(BTN_Y, INPUT_PULLUP);
  pinMode(BTN_Z, INPUT_PULLUP);
#endif

//--- type L ---------------
#ifdef TYPE_L
  pinMode(UP_Z_BTN, INPUT);
  pinMode(DOWN_Y_BTN, INPUT);
  pinMode(LEFT_X_BTN, INPUT);
  pinMode(RIGHT_MODE_BTN, INPUT);
  pinMode(A_B_BTN, INPUT);
  pinMode(SELECT_PIN, OUTPUT);
  pinMode(START_C_BTN, INPUT);

  digitalWrite(SELECT_PIN, HIGH);
#endif

  return 0;
}

/***********************************************************
 * Проверка контактов
************************************************************/
void debugInit()
{
#ifdef DEBUG
  pinMode(BTN_UP, OUTPUT);
  pinMode(BTN_DOWN, OUTPUT);
  pinMode(BTN_LEFT, OUTPUT);
  pinMode(BTN_RIGHT, OUTPUT);
  pinMode(BTN_START, OUTPUT);
  pinMode(BTN_MODE, OUTPUT);
  pinMode(BTN_A, OUTPUT);
  pinMode(BTN_B, OUTPUT);
  pinMode(BTN_C, OUTPUT);
  pinMode(BTN_X, OUTPUT);
  pinMode(BTN_Y, OUTPUT);
  pinMode(BTN_Z, OUTPUT);

  digitalWrite(BTN_UP, HIGH);
  digitalWrite(BTN_DOWN, HIGH);
  digitalWrite(BTN_LEFT, HIGH);
  digitalWrite(BTN_RIGHT, HIGH);
  digitalWrite(BTN_START, HIGH);
  digitalWrite(BTN_MODE, HIGH);
  digitalWrite(BTN_A, HIGH);
  digitalWrite(BTN_B, HIGH);
  digitalWrite(BTN_C, HIGH);
  digitalWrite(BTN_X, HIGH);
  digitalWrite(BTN_Y, HIGH);
  digitalWrite(BTN_Z, HIGH);
#endif
}

/***********************************************************
 * Чтение состояния выводов контроллера в буфер
 * Используется только для Type L
************************************************************/
unsigned char controllerPortRead()
{
  unsigned char res;
  res = PIND & 0xFC;
  res |= PINB & 0x02;
  return res;
}

/***********************************************************
 * Опрос состояния кнопок
************************************************************/
void pollController(byte *state_buf)
{

  //--- type S/C ---------------
#ifndef TYPE_L
  //if (!digitalRead(BTN_Z)) state_b1 &= B11111101;
  state_buf[0] &= (PINB >> 1) | B11111101;
  //if (!digitalRead(BTN_START)) state_b1 &= B11111011;
  state_buf[0] &= PIND | B11111011;
  //if (!digitalRead(BTN_MODE)) state_b1 &= B11101111;
  state_buf[0] &= (PINC << 4) | B11101111;
  //if (!digitalRead(BTN_X)) state_b1 &= B11011111;
  state_buf[0] &= (PIND >> 1) | B11011111;
  //if (!digitalRead(BTN_DOWN)) state_b1 &= B10111111;
  state_buf[0] &= (PIND << 5) | B10111111;
  //if (!digitalRead(BTN_RIGHT)) state_b1 &= B01111111;
  state_buf[0] &= (PIND << 7) | B01111111;

  //if (!digitalRead(BTN_UP)) state_b2 &= B11111110;
  //if (!digitalRead(BTN_LEFT)) state_b2 &= B11111101;
  state_buf[1] &= (PINC >> 2) | B11111100;
  //if (!digitalRead(BTN_Y)) state_b2 &= B11110111;
  //if (!digitalRead(BTN_C)) state_b2 &= B11101111;
  state_buf[1] &= (PINB << 3) | B11100111;
  //if (!digitalRead(BTN_A)) state_b2 &= B11011111;
  state_buf[1] &= PIND | B11011111;
  //if (!digitalRead(BTN_B)) state_b2 &= B10111111;
  state_buf[1] &= (PIND >> 1) | B10111111;
#endif

//--- type L ---------------
#ifdef TYPE_L

  unsigned char buf[3] = {0xff, 0xff, 0xff};
  //buf[0]:  A /  x  / x  / x  / x / x / Start/ x
  //buf[1]:  B /Right/Left/Down/ Up/ x / C    / x
  //buf[2]:  x / Mode/ X  / Y  / Z / x / x    / x

  for (unsigned char i = 0; i <= 3; i++)
  {
    digitalWrite(SELECT_PIN, LOW);
    delayMicroseconds(SCAN_STEP_DELAY);

    if (i == 1)
      buf[0] = controllerPortRead();

    digitalWrite(SELECT_PIN, HIGH);
    delayMicroseconds(SCAN_STEP_DELAY);

    if (i == 1)
      buf[1] = controllerPortRead();
    if (i == 2)
      buf[2] = controllerPortRead();
  }

  if (!(buf[0] & (1 << 7)))
    state_buf[0] &= B11011111; // A
  if (!(buf[0] & (1 << 1)))
    state_buf[0] &= B11111011; // Start

  if (!(buf[1] & (1 << 3)))
    state_buf[1] &= B11111110; // Up
  if (!(buf[1] & (1 << 4)))
    state_buf[0] &= B10111111; // Down
  if (!(buf[1] & (1 << 5)))
    state_buf[1] &= B11111101; // Left
  if (!(buf[1] & (1 << 6)))
    state_buf[0] &= B01111111; // Right
  if (!(buf[1] & (1 << 7)))
    state_buf[1] &= B10111111; // B
  if (!(buf[1] & (1 << 1)))
    state_buf[1] &= B11101111; // C

  if (!(buf[2] & (1 << 3)))
    state_buf[0] &= B11111101; // Z
  if (!(buf[2] & (1 << 4)))
    state_buf[1] &= B11110111; // Y
  if (!(buf[2] & (1 << 5)))
    state_buf[0] &= B11011111; // X
  if (!(buf[2] & (1 << 6)))
    state_buf[0] &= B11101111; // Mode

  delay(SCAN_LOOP_DELAY);
#endif
}

/***********************************************************
 * Убирает дребезг кнопок
************************************************************/
void buttonsScan()
{

  byte state1[2] = {0xFF, 0xFF};
  byte state2[2] = {0xFF, 0xFF};

  pollController(state1);
  delayMicroseconds(3);
  pollController(state2);

  buttons_state[0] = state1[0] | state2[0];
  buttons_state[1] = state1[1] | state2[1];
}

/***********************************************************
 * Формирование ответа для консоли
************************************************************/
void wiimoteQuery()
{

  wdt_reset();

  if (wm_get_reg(CONSOLE_TYPE_REG) == WII_TYPE)
  { // data format

    controller_report[0] = 0x5F;
    controller_report[1] = 0xDF;
    controller_report[2] = 0x8F;
    controller_report[3] = 0x00;
    controller_report[4] = buttons_state[0];
    controller_report[5] = buttons_state[1];
    controller_report[6] = 0x00;
    controller_report[7] = 0x00;
  }
  else if (wm_get_reg(CONSOLE_TYPE_REG) == SNES_TYPE)
  { // data format

    controller_report[0] = 0x7F;
    controller_report[1] = 0x7F;
    controller_report[2] = 0x7F;
    controller_report[3] = 0x7F;
    controller_report[4] = 0x00;
    controller_report[5] = 0x00;
    controller_report[6] = buttons_state[0];
    controller_report[7] = buttons_state[1];
  }

  if (!config_mode)
  {
    wm_newaction(controller_report);
  }
}

/***********************************************************
 * MAIN
************************************************************/
void setup()
{

#ifdef DEBUG
  debugInit();
#endif

#ifndef DEBUG

  wdt_disable(); // disable watchdog reset
  gamepadInit();
  wm_init(deviceID, cal_data, wiimoteQuery);
  wdt_enable(WDTO_2S); //enable watchdog
  wdt_reset();

#endif
}

void loop()
{
#ifndef DEBUG

  buttonsScan();

  // выход из режима настройки турбо кнопок
  if (buttons_state[0] == 0xF9 && buttons_state[1] == 0xFF && config_mode)
  {
    config_mode = false;
  }

  // вход в режим настройки настройки турбо кнопок
  if (buttons_state[0] == 0xFB && buttons_state[1] == 0xF7 && !config_mode)
  {
    config_push_time++;

    if (config_push_time > CONFIG_MODE_DELAY)
    {
      config_mode = true;
    }
    delay(1000);
  }
  else
  {
    config_push_time = 0;
  }

  delay(BUTTONS_SCAN_DELAY);

#endif
}
