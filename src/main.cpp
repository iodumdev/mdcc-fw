/*
  MEGA DRIVE CLASSIC CONTROLLER firmware
  version 2.0
  
  Copyright (C) 2020-2021 Denis Radiontsev.  All rights reserved.

  This program is free software; you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation; either version 2 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License along
  with this program; if not, write to the Free Software Foundation, Inc.,
  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.

  Contact information
  -------------------
  Denis Radiontsev
  e-mail   :  iodumdevices@gmail.com
*/

//#include <Arduino.h> // используется только функция millis()
extern "C" {
#include <millis.h>
}

//#define TYPE_L // !!! ТИП ПЛАТЫ. Закомментить для type S/C!!!

#include <WMExtention.h>
#include <util/delay.h>
#include <avr/wdt.h>
#include <defines.h>

// состояние кнопок
//byte buttons_state_mini[8] = {0x7F, 0x7F, 0x7F, 0x7F, 0x00, 0x00, 0xFF, 0xFF};
//byte buttons_state_wii[6] = {0x5F, 0xDF, 0x8F, 0x00, 0xFF, 0xFF};
uint8_t controller_report[8] = {0x5F, 0xDF, 0x8F, 0x00, 0xFF, 0xFF, 0x00, 0x00};
uint8_t buttons_state[2] = {0xFF, 0xFF};
uint8_t turbo_mask[2] = {0x00, 0x00};

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
  DDRB &= 0b11111000;
  PORTB |= 0b00000111;
  DDRC &= 0b11110010;
  PORTC |= 0b00001101;
  DDRD &= 0b00011000;
  PORTD |= 0b11100111;
#endif

//--- type L ---------------
#ifdef TYPE_L

  DDRD &= 0b00000111;
  DDRB |= 0b00000001;
  DDRB &= 0b11111101;
  PORTB |= _BV(PB0);

#endif

  return 0;
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
void pollController(uint8_t *state_buf)
{

  //--- type S/C ---------------
#ifndef TYPE_L
  //if (!digitalRead(BTN_Z)) state_b1 &= B11111101;
  state_buf[0] &= (PINB >> 1) | 0b11111101;
  //if (!digitalRead(BTN_START)) state_b1 &= B11111011;
  state_buf[0] &= PIND | 0b11111011;
  //if (!digitalRead(BTN_MODE)) state_b1 &= B11101111;
  state_buf[0] &= (PINC << 4) | 0b11101111;
  //if (!digitalRead(BTN_X)) state_b1 &= B11011111;
  state_buf[0] &= (PIND >> 1) | 0b11011111;
  //if (!digitalRead(BTN_DOWN)) state_b1 &= B10111111;
  state_buf[0] &= (PIND << 5) | 0b10111111;
  //if (!digitalRead(BTN_RIGHT)) state_b1 &= B01111111;
  state_buf[0] &= (PIND << 7) | 0b01111111;

  //if (!digitalRead(BTN_UP)) state_b2 &= B11111110;
  //if (!digitalRead(BTN_LEFT)) state_b2 &= B11111101;
  state_buf[1] &= (PINC >> 2) | 0b11111100;
  //if (!digitalRead(BTN_Y)) state_b2 &= B11110111;
  //if (!digitalRead(BTN_C)) state_b2 &= B11101111;
  state_buf[1] &= (PINB << 3) | 0b11100111;
  //if (!digitalRead(BTN_A)) state_b2 &= B11011111;
  state_buf[1] &= PIND | 0b11011111;
  //if (!digitalRead(BTN_B)) state_b2 &= B10111111;
  state_buf[1] &= (PIND >> 1) | 0b10111111;
#endif

//--- type L ---------------
#ifdef TYPE_L

  unsigned char buf[3] = {0xff, 0xff, 0xff};
  //buf[0]:  A /  x  / x  / x  / x / x / Start/ x
  //buf[1]:  B /Right/Left/Down/ Up/ x / C    / x
  //buf[2]:  x / Mode/ X  / Y  / Z / x / x    / x

  for (unsigned char i = 0; i <= 3; i++)
  {
    PORTB &= ~_BV(PB0);
    _delay_us(SCAN_STEP_DELAY_us);

    if (i == 1)
      buf[0] = controllerPortRead();

    PORTB |= _BV(PB0);
    _delay_us(SCAN_STEP_DELAY_us);

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

  _delay_ms(SCAN_LOOP_DELAY_ms);
#endif
}

/***********************************************************
 * Убирает дребезг кнопок
************************************************************/
void buttonsScan()
{

  uint8_t state1[2] = {0xFF, 0xFF};
  uint8_t state2[2] = {0xFF, 0xFF};

  pollController(state1);
  _delay_us(DEBOUNCE_DELAY_us);
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
  uint8_t state[2] = {0xFF, 0xFF};

  if (!config_mode)
  {
    state[0] = buttons_state[0];
    state[1] = buttons_state[1];

    if (millis() - last_turbo_push >= TURBO_INTERVAL_ms)
    {
      state[0] |= turbo_mask[0];
      state[1] |= turbo_mask[1];

      last_turbo_push = millis();
    }
  }

  if (WME.getReg(CONSOLE_TYPE_REG) == WII_TYPE)
  { // Wii data format
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
  { // NES/SNES data format
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
int main(void)
{

  wdt_disable(); // disable watchdog reset
  millis_init();
  gamepadInit();
  WME.init(wiimoteQuery);
  wdt_enable(WDTO_2S); //enable watchdog
  wdt_reset();

  while (1)
  {

    buttonsScan();

    // настройка турбо кнопок
    if (config_mode)
    {
      if (!(buttons_state[1] & nADD_TURBO_1))
      {
        // добавление турбо
        turbo_mask[0] |= 0x22 & ~(buttons_state[0]);
        turbo_mask[1] |= 0x78 & ~(buttons_state[1]);
      }
      else if (!(buttons_state[0] & nREMOVE_TURBO_0))
      {
        // удаление турбо
        turbo_mask[0] &= 0x22 & buttons_state[0];
        turbo_mask[1] &= 0x78 & buttons_state[1];
      }

      // сброс настроек турбо кнопок
      if (buttons_state[0] == RESET_CONFIG_0 && buttons_state[1] == RESET_CONFIG_1)
      {
        turbo_mask[0] = 0x00;
        turbo_mask[1] = 0x00;
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

        if (config_push_time > CONFIG_MODE_DELAY_s)
        {
          config_mode = true;
        }
        _delay_ms(1000);
      }
      else
      {
        config_push_time = 0;
      }
    }

    _delay_ms(BUTTONS_SCAN_DELAY_ms);
  }
}