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

#ifndef TEST_BOARD_H
#define TEST_BOARD_H

#define BOOTLOADER_VER_MEM 0x200041D0

/*------------------------------------------------------------------*/
/* LED
 *------------------------------------------------------------------*/
#define LEDS_NUMBER         2
#define LED_PRIMARY_PIN     7
#define LED_SECONDARY_PIN   22
#define LED_STATE_ON        0

/*------------------------------------------------------------------*/
/* BUTTON
 *------------------------------------------------------------------*/
#define BUTTONS_NUMBER      2
//#define BUTTON_1            11		// encoder switch
#define BUTTON_1            13	// side switch
//#define BUTTON_2            24		// random pin
#define BUTTON_DFU			11
#define BUTTON_PULL         NRF_GPIO_PIN_PULLUP

/*------------------------------------------------------------------*/
/* QSPI
 *------------------------------------------------------------------*/
// #define USE_QSPI_XIP		1

#define USE_QSPI		0
#define USE_DFU_UART	0		// special device<->device update

#define QSPI_SCK_PIN   		NRF_GPIO_PIN_MAP(1,1)
#define QSPI_CSN_PIN   		NRF_GPIO_PIN_MAP(1,2)
#define QSPI_IO0_PIN   		NRF_GPIO_PIN_MAP(1,3)
#define QSPI_IO1_PIN   		NRF_GPIO_PIN_MAP(1,4)
#define QSPI_IO2_PIN   		NRF_GPIO_PIN_MAP(1,5)
#define QSPI_IO3_PIN   		NRF_GPIO_PIN_MAP(1,6)

#define QSPI_XIP_START_ADDR      0x12000000

//--------------------------------------------------------------------+
// UART update
//--------------------------------------------------------------------+
#define RX_PIN_NUMBER      31
#define TX_PIN_NUMBER      30
#define CTS_PIN_NUMBER     0
#define RTS_PIN_NUMBER     0
#define HWFC               false

//--------------------------------------------------------------------+
// BLE OTA
//--------------------------------------------------------------------+
#define BLEDIS_MANUFACTURER    "Moddable Tech, Inc."
#define BLEDIS_MODEL           "Test item"

//--------------------------------------------------------------------+
// USB
//--------------------------------------------------------------------+


//#define USB_DESC_VID           0x1915
//#define USB_DESC_UF2_PID       0x520F
#define USB_DESC_VID           0xbeef
#define USB_DESC_UF2_PID       0xcafe
#define USB_DESC_CDC_ONLY_PID  0x002A

#define UF2_PRODUCT_NAME    "Moddable Four"
#define UF2_BOARD_ID        "moddable_four-v1"
#define UF2_INDEX_URL       "https://www.moddable.com/product.php"

#define UF2_VOLUME_LABEL   "MODDABLE4  "

// Interface number, string index, EP Out & IN address, EP size
#define TUD_MOD_VENDOR_DESCRIPTOR(_itfnum, _stridx, _epout, _epin, _epsize) \
	/* Interface */\
	9, TUSB_DESC_INTERFACE, _itfnum, 0, 2, TUSB_CLASS_VENDOR_SPECIFIC, CFG_TUD_VENDOR_SUBCLASS, 0x00, _stridx,\
	/* Endpoint Out */\
	7, TUSB_DESC_ENDPOINT, _epout, TUSB_XFER_BULK, U16_TO_U8S_LE(_epsize), 0,\
	/* Endpoint In */\
	7, TUSB_DESC_ENDPOINT, _epin, TUSB_XFER_BULK, U16_TO_U8S_LE(_epsize), 0
#endif // TEST_BOARD_H
