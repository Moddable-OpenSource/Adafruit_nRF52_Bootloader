/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2018 Ha Thach for Adafruit Industries
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "boards.h"
#include "app_scheduler.h"
#include "app_timer.h"

//--------------------------------------------------------------------+
// MACRO TYPEDEF CONSTANT ENUM DECLARATION
//--------------------------------------------------------------------+
#define SCHED_MAX_EVENT_DATA_SIZE           sizeof(app_timer_event_t)        /**< Maximum size of scheduler events. */
#define SCHED_QUEUE_SIZE                    30                               /**< Maximum number of events in the scheduler queue. */

void led_init(uint32_t led_index, uint32_t led_pin);
void led_terminate();

//------------- IMPLEMENTATION -------------//
void button_init(uint32_t pin)
{
  if ( BUTTON_PULL == NRF_GPIO_PIN_PULLDOWN )
  {
    nrf_gpio_cfg_sense_input(pin, BUTTON_PULL, NRF_GPIO_PIN_SENSE_HIGH);
  }
  else
  {
    nrf_gpio_cfg_sense_input(pin, BUTTON_PULL, NRF_GPIO_PIN_SENSE_LOW);
  }
}

bool button_pressed(uint32_t pin)
{
  uint32_t const active_state = (BUTTON_PULL == NRF_GPIO_PIN_PULLDOWN ? 1 : 0);
  return nrf_gpio_pin_read(pin) == active_state;
}

void board_init(void)
{
  led_terminate();

  // stop LF clock just in case we jump from application without reset
  NRF_CLOCK->TASKS_LFCLKSTOP = 1UL;

  // Use Internal OSC to compatible with all boards
  NRF_CLOCK->LFCLKSRC = CLOCK_LFCLKSRC_SRC_RC;
  NRF_CLOCK->TASKS_LFCLKSTART = 1UL;

#if defined(BUTTON_DFU)
  button_init(BUTTON_DFU);
#endif
#if defined(BUTTON_FRESET)
  button_init(BUTTON_FRESET);
#endif
#if defined(BUTTON_5)
  button_init(BUTTON_5);
#endif
  NRFX_DELAY_US(100); // wait for the pin state is stable

  // use PMW0 for LED RED
  led_init(LED_PRIMARY, LED_PRIMARY_PIN);

  // Init scheduler
  APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);

  // Init app timer (use RTC1)
  app_timer_init();

  // Configure Systick for led blinky
  NVIC_SetPriority(SysTick_IRQn, 7);
  SysTick_Config(SystemCoreClock/1000);
}

void board_teardown(void)
{
  // Disable systick, turn off LEDs
  SysTick->CTRL = 0;

  // Button

  // Stop RTC1 used by app_timer
  NVIC_DisableIRQ(RTC1_IRQn);
  NRF_RTC1->EVTENCLR    = RTC_EVTEN_COMPARE0_Msk;
  NRF_RTC1->INTENCLR    = RTC_INTENSET_COMPARE0_Msk;
  NRF_RTC1->TASKS_STOP  = 1;
  NRF_RTC1->TASKS_CLEAR = 1;

  // Stop LF clock
  NRF_CLOCK->TASKS_LFCLKSTOP = 1UL;
}

static uint32_t _systick_count = 0;
void SysTick_Handler(void)
{
  _systick_count++;

  led_tick();
}


uint32_t tusb_hal_millis(void)
{
  return ( ( ((uint64_t)app_timer_cnt_get())*1000*(APP_TIMER_CONFIG_RTC_FREQUENCY+1)) / APP_TIMER_CLOCK_FREQ );
}


static uint32_t g_led_pin = LED_PRIMARY_PIN;

void led_init(uint32_t led_index, uint32_t led_pin) {
	g_led_pin = led_pin;
	nrf_gpio_cfg_output(led_pin);
	nrf_gpio_pin_write(led_pin, !LED_STATE_ON);
}

void led_terminate() {
#ifdef LED_PRIMARY_PIN
	nrf_gpio_pin_write(LED_PRIMARY_PIN, !LED_STATE_ON);
#endif
#ifdef LED_SECONDARY_PIN
	nrf_gpio_pin_write(LED_SECONDARY_PIN, !LED_STATE_ON);
#endif
}

static uint32_t ledCycle = 0;
static uint32_t ledNext = 0;
static uint32_t ledDelays[4] = { 500, 500, 500, 500 };
#define setLEDDelays(a, b, c, d) {	\
	ledDelays[0] = a; \
	ledDelays[1] = b; \
	ledDelays[2] = c; \
	ledDelays[3] = d; }

void led_tick() {
	if (_systick_count > ledNext) {
		ledNext += ledDelays[ledCycle++];
		if (ledCycle > 3)
			ledCycle = 0;
		nrf_gpio_pin_write(g_led_pin, ledCycle % 2);
	}
}

void led_state(uint32_t state)
{
    switch (state) {
        case STATE_USB_MOUNTED:
			setLEDDelays(500, 500, 500, 500);
          break;

        case STATE_BOOTLOADER_STARTED:
        case STATE_USB_UNMOUNTED:
			setLEDDelays(700, 500, 700, 500);
          break;

        case STATE_WRITING_STARTED:
			setLEDDelays(30, 100, 50, 100);
          break;

        case STATE_WRITING_FINISHED:
			setLEDDelays(500, 500, 500, 500);
          break;

        case STATE_BLE_CONNECTED:
			setLEDDelays(50, 50, 50, 750);
          break;

        case STATE_BLE_DISCONNECTED:
			setLEDDelays(50, 50, 50, 250);
          break;

        default:
        break;
    }
}

