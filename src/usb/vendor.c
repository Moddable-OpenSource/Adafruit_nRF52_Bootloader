#include "uf2/uf2.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "boards.h"
#include "tusb.h"
#include "usb_desc.h"

#include "bootloader.h"

uint32_t vendor_connected = 0;

bool tud_vendor_control_request_cb(uint8_t rhport, tusb_control_request_t const * request) {
	switch (request->bRequest) {
		case 0x22:
			// simulate CDC_REQUEST_SET_CONTROL_LINE_STATE
			vendor_connected = (request->wValue != 0);

			if (vendor_connected) {
				led_state(STATE_USB_MOUNTED);
//				tud_vendor_write_str("\r\n VENDOR CONTROL_LINE_STATE\r\n");
			}
			else {
				led_state(STATE_USB_UNMOUNTED);
			}
			return tud_control_status(rhport, request);
		default:
			return false;
	}
	
	return true;
}

// Invoked when DATA stage of VENDOR request is complete
bool tud_vendor_control_complete_cb(uint8_t rhport, tusb_control_request_t const *request) {
		// do something with the data here
	return true;
}

#define VENDOR_READ_BUF_SIZE	1024
static uint8_t vendor_read_buffer[VENDOR_READ_BUF_SIZE];
static uint32_t vendor_read_pos = 0;

static WriteState _vendor_wr_state = { 0 };
extern int write_block(uint32_t block_no, uint8_t *data, WriteState *state);

uint32_t data[2];
char response[128];

char debugbuf[1024];
void vendor_eat_data(uint8_t *buf, uint32_t len) {
	uint32_t amt, rem;
	uint8_t *p;

	while (len) {
		rem = VENDOR_READ_BUF_SIZE - vendor_read_pos;
		amt = len > rem ? rem : len;
		memcpy(vendor_read_buffer + vendor_read_pos, buf, amt);
		vendor_read_pos += amt;
		len -= amt;

		amt = vendor_read_pos;
		p = vendor_read_buffer;
		while (amt >= 512) {
			UF2_Block *bl = (void*)p;
			int wr_ret;

			wr_ret = write_block(0, p, &_vendor_wr_state);
			if (wr_ret > 0) {
				p += 512;
				amt -= 512;
				sprintf(response, "%ld\r\n", bl->blockNo);
//				data[0] = 0xbeefcafe;
//				data[1] = bl->blockNo;
			}
			else {
				sprintf(response, "fail %ld\r\n", bl->blockNo);
//				data[0] = 0xffffffff;
//				data[1] = 0xffffffff;
			}

//			tud_vendor_write((void*)data, 8);
			tud_vendor_write_str(response);
		}

		if (amt) { 			// remainder
			memcpy(vendor_read_buffer, p, amt);
			vendor_read_pos = amt;
		}
		else
			vendor_read_pos = 0;
	}

	if (_vendor_wr_state.numBlocks && (_vendor_wr_state.numWritten >= _vendor_wr_state.numBlocks)) {
		led_state(STATE_WRITING_FINISHED);

//		data[0] = _vendor_wr_state.numBlocks;
//		data[1] = _vendor_wr_state.numBlocks;
//		tud_vendor_write(data, 8);
		sprintf(response, "%ld finished\r\n", _vendor_wr_state.numBlocks);
		tud_vendor_write_str(response);

		dfu_update_status_t update_status;
		memset(&update_status, 0, sizeof(dfu_update_status_t));
		update_status.status_code = DFU_UPDATE_APP_COMPLETE;
		update_status.app_crc = 0;
		update_status.app_size = _vendor_wr_state.numBlocks * 256;

		bootloader_dfu_update_process(update_status);
	}
}

void tud_vendor_rx_cb(uint8_t itf) {
//void tud_vendor_task(void) {
//	if (vendor_connected) {
//		if (tud_vendor_available() ) {
			uint8_t buf[512];
			uint32_t count;
			
			while ((count = tud_vendor_read(buf, sizeof(buf))) > 0) {
				vendor_eat_data(buf, count);
			}
//		}
//	}
}


