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

#ifndef USB_DESC_H_
#define USB_DESC_H_

#include "tusb.h"
#include "boards.h"

void usb_desc_init(bool cdc_only);

#ifndef USB_DESC_VID
//#define USB_DESC_VID            0x239A
//#define USB_DESC_VID            0x1915
#define USB_DESC_VID            0xbeef
#endif

#ifndef USB_DESC_UF2_PID
//#define USB_DESC_UF2_PID        0x0029
//#define USB_DESC_UF2_PID        0x520F
#define USB_DESC_UF2_PID        0xcafe
#endif

#ifndef USB_DESC_CDC_ONLY_PID
#define USB_DESC_CDC_ONLY_PID   0x002A
#endif

// Interface number, string index, EP Out & IN address, EP size
#define TUD_MOD_VENDOR_DESCRIPTOR(_itfnum, _stridx, _epout, _epin, _epsize) \
    /* Interface */\
    9, TUSB_DESC_INTERFACE, _itfnum, 0, 2, TUSB_CLASS_VENDOR_SPECIFIC, CFG_TUD_VENDOR_SUBCLASS, 0x00, _stridx,\
    /* Endpoint Out */\
    7, TUSB_DESC_ENDPOINT, _epout, TUSB_XFER_BULK, U16_TO_U8S_LE(_epsize), 0,\
    /* Endpoint In */\
    7, TUSB_DESC_ENDPOINT, _epin, TUSB_XFER_BULK, U16_TO_U8S_LE(_epsize), 0

#endif /* USB_DESC_H_ */
