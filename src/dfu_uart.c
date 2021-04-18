
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "boards.h"
#include "nrfx_uart.h"
#include "app_fifo.h"

#include "uf2.h"
#include "crc16.h"

#include "bootloader.h"
#include "nrf_mbr.h"

#include "dfu_uart.h"

#if USE_DFU_UART

extern uint32_t tusb_hal_millis(void);
extern int write_block(uint32_t block_no, uint8_t *data, WriteState *state);

#define SERIAL_DRAIN_TIME 1000		// 

extern uint32_t *dbl_reset_mem;

static uint32_t finish_time = 0xffffffff;
static int finished_sent = 0;

static WriteState _serial_wr_state = {0};

static uint8_t ser_blk_buffer[512];
static uint32_t ser_blk_buffer_amt = 0;

//#define BOOTLOADER_SERIAL_TX_PIN 12
//#define BOOTLOADER_SERIAL_RX_PIN 7
//#define BOOTLOADER_SERIAL_BAUDRATE NRF_UART_BAUDRATE_921600
// #define BOOTLOADER_SERIAL_BAUDRATE NRF_UART_BAUDRATE_115200

// Priority default is 7

nrfx_uart_config_t gUartConfig = NRFX_UART_DEFAULT_CONFIG(DFU_UART_TX_PIN, DFU_UART_RX_PIN);

nrfx_uart_t gUart = {
    .p_reg        = NRFX_CONCAT_2(NRF_UART, 0),
    .drv_inst_idx = NRFX_CONCAT_3(NRFX_UART, 0, _INST_IDX),
};

static app_fifo_t	ser_rx_fifo;
static uint8_t		*ser_rx_fifo_buffer;

#define MOD_SERIAL_RX_FIFO_SIZE	2048

char ser_response[32];

static __INLINE uint32_t fifo_length(app_fifo_t *const fifo) {
	return fifo->write_pos - fifo->read_pos;
}

static void serial_write(const char* buf) {
	int ret;
	ret = nrfx_uart_tx(&gUart, (uint8_t*)buf, strlen((char*)buf));
	if (NRFX_SUCCESS != ret)
		ret = nrfx_uart_tx(&gUart, (uint8_t*)buf, strlen((char*)buf));
}

void setup_uart() {
	int ret;

	gUartConfig.baudrate = DFU_UART_BAUDRATE;
	ret = nrfx_uart_init(&gUart, &gUartConfig, NULL);
	if (NRFX_SUCCESS != ret)
		return;

    nrf_gpio_cfg(gUartConfig.pseltxd,
            NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT,
            NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_H0H1, NRF_GPIO_PIN_NOSENSE);

	ser_rx_fifo_buffer = malloc(MOD_SERIAL_RX_FIFO_SIZE);
    ret = app_fifo_init(&ser_rx_fifo, ser_rx_fifo_buffer, MOD_SERIAL_RX_FIFO_SIZE);
	nrfx_uart_rx_enable(&gUart);
}


static int serial_eat_data(uint8_t *buf, uint32_t len) {
	int ret;
	UF2_Block *bl = (UF2_Block*)buf;

	if (bl->blockNo >= 2083)
		bl->flags = 0x2000;

	ret = write_block(0, ser_blk_buffer, &_serial_wr_state);

	if (ret > 0)
		sprintf(ser_response, "RCV:%05ld", bl->blockNo);
	else if (ret == 0)
		sprintf(ser_response, "BSY:%05ld", bl->blockNo);
	else
		sprintf(ser_response, "BAD:%05ld", bl->blockNo);

	serial_write(ser_response);
	return ret;
}

void dfu_uart_task() {
	uint32_t check, calc, sz;
	int ret;
	dfu_update_status_t	update_status;

	if (ser_blk_buffer_amt >= 512) {
		ret = serial_eat_data(ser_blk_buffer, 512);
		if (0 == ret) {
			return;			// was busy flashing
		}
		ser_blk_buffer_amt = 0;
	}

//receive:
	while (nrfx_uart_rx_ready(&gUart) && (fifo_length(&ser_rx_fifo) < MOD_SERIAL_RX_FIFO_SIZE)) {
		uint8_t ch;

		ret = nrfx_uart_rx(&gUart, &ch, 1);
		if (NRFX_SUCCESS != ret)
			break;

		ret = app_fifo_put(&ser_rx_fifo, ch);
		if (NRFX_SUCCESS != ret)
			break;
	}
		
	if (fifo_length(&ser_rx_fifo) >= 520) {
		sz = 4;
		app_fifo_read(&ser_rx_fifo, (uint8_t*)&check, &sz);
		if (SERIAL_TOKEN != check) {
			serial_write("BAD:token");
			return;
		}
		sz = 4;
		app_fifo_read(&ser_rx_fifo, (uint8_t*)&check, &sz);
		led_state(STATE_WRITING_STARTED);

		sz = 512;
		app_fifo_read(&ser_rx_fifo, ser_blk_buffer, &sz);
//		calc = crc16_compute(ser_blk_buffer, 512, NULL);
		calc = 0x12345678;
		if (check != calc) {
			serial_write("BAD:check");
			ser_blk_buffer_amt = 0;
			return;
		}

		ret =  serial_eat_data(ser_blk_buffer, 512);
		if (0 == ret)
			goto process;			// was busy flashing - try again later

		ser_blk_buffer_amt = 0;
	}

process:
	if (_serial_wr_state.aborted) {
		memset(&update_status, 0, sizeof(dfu_update_status_t));
		*dbl_reset_mem = 0;

		update_status.status_code = DFU_RESET;
		bootloader_dfu_update_process(update_status);
		led_state(STATE_WRITING_FINISHED);
	}
	else if (!finished_sent && _serial_wr_state.numBlocks && (_serial_wr_state.numWritten >= _serial_wr_state.numBlocks)) {
		memset(&update_status, 0, sizeof(dfu_update_status_t));
		*dbl_reset_mem = 0;

		led_state(STATE_WRITING_FINISHED);
		sprintf(ser_response, "FIN:%05ld", _serial_wr_state.numBlocks);
		serial_write(ser_response);

		finish_time = tusb_hal_millis() + SERIAL_DRAIN_TIME;
		finished_sent = 1;
	}

	if (tusb_hal_millis() >= finish_time) {
		finish_time = 0xffffffff;
		if (_serial_wr_state.update_bootloader) {
			uint32_t *new_bootloader = (uint32_t*)BOOTLOADER_ADDR_NEW_RECIEVED;
			update_status.status_code = DFU_RESET;
			// skip if ther eis no bootloader change
			if (memcmp(new_bootloader, (uint8_t*)BOOTLOADER_ADDR_START, DFU_BL_IMAGE_MAX_SIZE) ) {
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
			update_status.app_size = _serial_wr_state.numBlocks * 256;
		}

		bootloader_dfu_update_process(update_status);
	}

}

#endif	/* USE_DFU_UART */
