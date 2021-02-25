#include "uf2/uf2.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "boards.h"
#include "tusb.h"
#include "usb_desc.h"
#include "crc16.h"
#include "flash_nrf5x.h"

#include "bootloader.h"

extern uint32_t tusb_hal_millis(void);

extern uint32_t *dbl_reset_mem;
#define DFU_MODDABLE_VENDOR_MAGIC	0xf00dcafe

#define PROGRAMMING_TRIGGER_DELAY	500
#define USB_DRAIN_TIME	50			// some time to allow USB to drain

uint32_t vendor_connected = 0;
uint16_t last_state = 0;
uint32_t last_state_ms = 0;
uint32_t resetTime = 0xffffffff;
uint32_t finishTime = 0xffffffff;

bool tud_vendor_control_request_cb(uint8_t rhport, tusb_control_request_t const * request) {
	uint32_t now = tusb_hal_millis();
	switch (request->bRequest) {
		case 0x22: {
			// simulate CDC_REQUEST_SET_CONTROL_LINE_STATE
			uint8_t reboot_seq = request->wValue;	// DTR: 0b01, RTS: 0b10

			vendor_connected = (request->wValue != 0);

			if (vendor_connected) {
				led_state(STATE_USB_MOUNTED);
//				tud_vendor_write_str("\r\n VENDOR CONTROL_LINE_STATE\r\n");
			}
			else {
				led_state(STATE_USB_UNMOUNTED);
			}

			if (last_state != reboot_seq) {
				if (1 == last_state && 0 == reboot_seq
					&& ((now - last_state_ms) < PROGRAMMING_TRIGGER_DELAY)) {
					// reboot as vendor
					*dbl_reset_mem = DFU_MODDABLE_VENDOR_MAGIC;
					resetTime = tusb_hal_millis() + USB_DRAIN_TIME;
					tud_vendor_write_str("REBOOT VENDOR");
				}
/*
				else if (0 == reboot_seq) {
					// reboot
					*dbl_reset_mem = 0;
					resetTime = tusb_hal_millis() + USB_DRAIN_TIME;
					tud_vendor_write_str("REBOOT");
				}
*/
			}

			last_state = reboot_seq;
			last_state_ms = now;
			return tud_control_status(rhport, request);
		}
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

static WriteState _vendor_wr_state = { 0 };
extern int write_block(uint32_t block_no, uint8_t *data, WriteState *state);

char response[128];

void vendor_debug(char *str) {
	tud_vendor_write_str(str);
}

int vendor_eat_data(uint8_t *buf, uint32_t len) {
	UF2_Block *bl = (void*)buf;
	int wr_ret;

	wr_ret = write_block(0, buf, &_vendor_wr_state);	// block is in buf
	if (wr_ret > 0) {
		sprintf(response, "rcv: %ld\n", bl->blockNo);
	}
	else if (wr_ret == 0) {					 // busy
		sprintf(response, "BUSY %ld\n", bl->blockNo);
	}
	else {									 // not a UF2 block
		sprintf(response, "not uf2 %ld\n", bl->blockNo);
	}

	tud_vendor_write_str(response);


	return wr_ret;
}

uint8_t block_buffer[512];
uint32_t block_buffer_amt = 0;
int finished_sent = 0;

void tud_vendor_task(void) {
	int ret;
	if (block_buffer_amt) {
		ret = vendor_eat_data(block_buffer, 512);
		if (0 == ret) {
			return;					// was busy flashing
		}
		block_buffer_amt = 0;
	}

//	if (tud_vendor_available() >= 512 ) 
	if (tud_vendor_available() >= 520 ) {
		uint32_t check, calc;
		tud_vendor_read(&check, 4);
		if (0xbeefc4f3 != check) {
tud_vendor_write_str("no 0xbeefc4f3\n");
			return;
		}
		tud_vendor_read(&check, 4);
		
		led_state(STATE_WRITING_STARTED);
		
		block_buffer_amt = tud_vendor_read(block_buffer, 512);
		calc = crc16_compute(block_buffer, 512, NULL);
		if (check != calc) {
			tud_vendor_write_str("resend\n");
			block_buffer_amt = 0;
			return;
		}
		ret = vendor_eat_data(block_buffer, 512);
		if (0 == ret) {
			return;					// was busy flashing - try again later
		}
		block_buffer_amt = 0;
	}

	dfu_update_status_t update_status;
	if (_vendor_wr_state.aborted) {
		memset(&update_status, 0, sizeof(dfu_update_status_t));

		*dbl_reset_mem = 0;
		update_status.status_code = DFU_RESET;
		bootloader_dfu_update_process(update_status);
		led_state(STATE_WRITING_FINISHED);
	}
	else if (!finished_sent && _vendor_wr_state.numBlocks && (_vendor_wr_state.numWritten >= _vendor_wr_state.numBlocks)) {
		memset(&update_status, 0, sizeof(dfu_update_status_t));
		*dbl_reset_mem = 0;

		led_state(STATE_WRITING_FINISHED);
		sprintf(response, "%ld finished\r\n", _vendor_wr_state.numBlocks);
		tud_vendor_write_str(response);

//		flash_nrf5x_flush(true);
		finishTime = tusb_hal_millis() + USB_DRAIN_TIME;

		finished_sent = 1;
	}

	if (tusb_hal_millis() >= finishTime) {
		finishTime = 0xffffffff;
		if (_vendor_wr_state.update_bootloader) {
			update_status.status_code = DFU_RESET;
			uint32_t *new_bootloader = (uint32_t*)BOOTLOADER_ADDR_NEW_RECIEVED;
			// skip if there is no bootloader change
			if ( memcmp(new_bootloader, (uint8_t*) BOOTLOADER_ADDR_START, DFU_BL_IMAGE_MAX_SIZE) ) {
				// copy new bootloader
				sd_mbr_command_t command = {
		            .command = SD_MBR_COMMAND_COPY_BL,
		            .params.copy_bl.bl_src = new_bootloader,
		            .params.copy_bl.bl_len = DFU_BL_IMAGE_MAX_SIZE/4 // size in words
				};
				sd_mbr_command(&command);
			}
		}
		else {
			update_status.status_code = DFU_UPDATE_APP_COMPLETE;
			update_status.app_crc = 0;
			update_status.app_size = _vendor_wr_state.numBlocks * 256;
		}

		bootloader_dfu_update_process(update_status);
	}

	if (tusb_hal_millis() >= resetTime)
		NVIC_SystemReset();
}


