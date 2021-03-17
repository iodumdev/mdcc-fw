/*
 * MEGA DRIVE CLASSIC CONTROLLER firmware
 * by IODUM
 *
 * version 1.1.2
 *
*/

//#define TYPE_L // !!! ТИП ПЛАТЫ. Закомментить для type S/C!!!
//#define DEBUG

#include <WMExtention.h>
#include <avr/wdt.h>
#include "defines.h"

// состояние кнопок
//byte buttons_state_mini[8] = {0x7F, 0x7F, 0x7F, 0x7F, 0x00, 0x00, 0xFF, 0xFF};
//byte buttons_state_wii[6] = {0x5F, 0xDF, 0x8F, 0x00, 0xFF, 0xFF};
byte controller_report[8] = {0x5F, 0xDF, 0x8F, 0x00, 0xFF, 0xFF, 0x00, 0x00};
byte buttons_state[2] = {0xFF, 0xFF};
byte turbo_mask[2] = {0x00, 0x00};

// режим установки турбо кнопок
bool config_mode = false;
uint8_t config_push_time = 0;
unsigned long last_turbo_push = 0;

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
  delayMicroseconds(DEBOUNCE_DELAY);
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
  byte state[2] = {0xFF, 0xFF};

  if (!config_mode)
  {
    state[0] = buttons_state[0];
    state[1] = buttons_state[1];

    if (millis() - last_turbo_push >= TURBO_INTERVAL)
    {
      state[0] |= turbo_mask[0];
      state[1] |= turbo_mask[1];

      last_turbo_push = millis();
    }
  }

  if (WME.getReg(CONSOLE_TYPE_REG) == WII_TYPE)
  { // data format
    controller_report[0] = 0x5F;
    controller_report[1] = 0xDF;
    controller_report[2] = 0x8F;
    controller_report[3] = 0x00;
    controller_report[4] = state[0];
    controller_report[5] = state[1];
    controller_report[6] = 0x00;
    controller_report[7] = 0x00;
  }
  else if (WME.getReg(CONSOLE_TYPE_REG) == SNES_TYPE)
  { // data format
    controller_report[0] = 0x7F;
    controller_report[1] = 0x7F;
    controller_report[2] = 0x7F;
    controller_report[3] = 0x7F;
    controller_report[4] = 0x00;
    controller_report[5] = 0x00;
    controller_report[6] = state[0];
    controller_report[7] = state[1];
  }

  WME.newAction(controller_report);
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
  WME.init(wiimoteQuery);
  wdt_enable(WDTO_2S); //enable watchdog
  wdt_reset();

#endif
}

void loop()
{
#ifndef DEBUG

  buttonsScan();

  // настройка турбо кнопок
  if (config_mode)
  {
    if (!(buttons_state[1] & nADD_TURBO_1))
    {
      // добавление турбо
      turbo_mask[0] &= 0x22 & !(buttons_state[0]);
      turbo_mask[1] &= 0x7B & !(buttons_state[1]);
    }
    else if(!(buttons_state[0] & nREMOVE_TURBO_0))
    {
      // удаление турбо
      turbo_mask[0] |= 0xDD | buttons_state[0];
      turbo_mask[1] |= 0x84 | buttons_state[1];
    }

    // сброс настроек турбо кнопок
    if (buttons_state[0] == RESET_CONFIG_0 && buttons_state[1] == RESET_CONFIG_1)
    {
      turbo_mask[0] = 0xFF;
      turbo_mask[1] = 0xFF;
    }
    // выход из режима настройки турбо кнопок
    if (buttons_state[0] == EXIT_CONFIG_0 && buttons_state[1] == EXIT_CONFIG_1)
    {
      config_mode = false;
    }
  }
  else
  {
    // вход в режим настройки турбо кнопок
    if (buttons_state[0] == ENTER_CONFIG_0 && buttons_state[1] == ENTER_CONFIG_1)
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
  }

  delay(BUTTONS_SCAN_DELAY);

#endif
}
