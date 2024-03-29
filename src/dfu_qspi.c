#include <nrfx_qspi.h>
#include <boards.h>
//#include "ftdi_trace.h"

#if USE_QSPI

#define QSPI_STD_CMD_WRSR        0x01
#define QSPI_STD_CMD_RSTEN       0x66
#define QSPI_STD_CMD_RST         0x99

#define QSPI_SR_QUAD_ENABLE_BYTE 0x40

//#define QSPI_XIP_START_ADDR      0x12000000		// in boards.h


static void configure_memory(bool writable)
{
	uint32_t err_code;
	uint8_t data;
	nrf_qspi_cinstr_conf_t cinstr_cfg = {
		.opcode		= QSPI_STD_CMD_RSTEN,
		.length		= NRF_QSPI_CINSTR_LEN_1B,
		.io2_level	= true,
		.io3_level	= true,
		.wipwait	= false, 		// was true - set up for readonly for faster
		.wren		= false			// was true
	};

	if (writable) {
		cinstr_cfg.wipwait = true;
		cinstr_cfg.wren = true;
	}

	// Send reset enable
	err_code = nrfx_qspi_cinstr_xfer(&cinstr_cfg, NULL, NULL);
	if (NRFX_SUCCESS != err_code)
		return;
//		ftdiTraceAndHex("qspi_reset enable - qspi_cinstr_xfer", err_code);

	// Send reset command
	cinstr_cfg.opcode = QSPI_STD_CMD_RST;
	err_code = nrfx_qspi_cinstr_xfer(&cinstr_cfg, NULL, NULL);
//ftdiTrace("qspi_reset - ");
//if (NRFX_SUCCESS != err_code) ftdiTraceAndHex("qspi_reset - qspi_cinstr_xfer", err_code);

	// Switch to qspi mode
	data = QSPI_SR_QUAD_ENABLE_BYTE;
	cinstr_cfg.opcode = QSPI_STD_CMD_WRSR;
	cinstr_cfg.length = NRF_QSPI_CINSTR_LEN_2B;
	err_code = nrfx_qspi_cinstr_xfer(&cinstr_cfg, &data, NULL);
//ftdiTrace("qspi_mode - ");
//if (NRFX_SUCCESS != err_code) ftdiTraceAndHex("qspi_mode - qspi_cinstr_xfer", err_code);

	while (NRFX_ERROR_BUSY == nrfx_qspi_mem_busy_check())
	{}
//ftdiTrace("qspi memory configured");
}

void setup_qspi(bool writable)
{
	uint32_t err_code;

	// Set QSPI peripheral with default config
	nrfx_qspi_config_t config = NRFX_QSPI_DEFAULT_CONFIG (
		config.pins.sck_pin = QSPI_SCK_PIN,
		config.pins.csn_pin = QSPI_CSN_PIN,
		config.pins.io0_pin = QSPI_IO0_PIN,
		config.pins.io1_pin = QSPI_IO1_PIN,
		config.pins.io2_pin = QSPI_IO2_PIN,
		config.pins.io3_pin = QSPI_IO3_PIN);
	config.prot_if.readoc = NRFX_QSPI_CONFIG_READOC;
	config.prot_if.writeoc = NRFX_QSPI_CONFIG_WRITEOC;
	config.prot_if.addrmode = NRFX_QSPI_CONFIG_ADDRMODE;
	config.phy_if.spi_mode = NRFX_QSPI_CONFIG_MODE;
	config.phy_if.sck_freq = NRFX_QSPI_CONFIG_FREQUENCY;
	config.phy_if.sck_delay = 1;

	// use high drive strength
	nrf_gpio_cfg(config.pins.sck_pin,
			NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT,
			NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_H0H1, NRF_GPIO_PIN_NOSENSE);
	nrf_gpio_cfg(config.pins.csn_pin,
			NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT,
			NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_H0H1, NRF_GPIO_PIN_NOSENSE);
	nrf_gpio_cfg(config.pins.io0_pin,
			NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT,
			NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_H0H1, NRF_GPIO_PIN_NOSENSE);
	nrf_gpio_cfg(config.pins.io1_pin,
			NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT,
			NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_H0H1, NRF_GPIO_PIN_NOSENSE);
	nrf_gpio_cfg(config.pins.io2_pin,
			NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT,
			NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_H0H1, NRF_GPIO_PIN_NOSENSE);
	nrf_gpio_cfg(config.pins.io3_pin,
			NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT,
			NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_H0H1, NRF_GPIO_PIN_NOSENSE);

	// Initialize QSPI in blocking mode
	err_code = nrfx_qspi_init(&config, NULL, NULL);
	if (NRFX_SUCCESS != err_code)
		return;
		//ftdiTraceAndHex("qspi_init fails", err_code);

	// Restart and configure external memory to use quad line mode
	configure_memory(writable);

}

#define FLASH_PAGE_SIZE				4096
#define FLASH_CACHE_INVALID_ADDR	0xffffffff
static uint32_t _qsp_addr = FLASH_CACHE_INVALID_ADDR;
static uint8_t	_qsp_buf[FLASH_PAGE_SIZE] __attribute__((aligned(4)));

void qspi_flush(bool need_erase)
{
	uint32_t err_code;
	if (_qsp_addr == FLASH_CACHE_INVALID_ADDR) return;
	
	// skip write if content matches
	if (memcmp(_qsp_buf, (void*)_qsp_addr, FLASH_PAGE_SIZE) != 0)
	{
		if (need_erase) {
			err_code = nrfx_qspi_erase(NRF_QSPI_ERASE_LEN_4KB, _qsp_addr);
			if (NRFX_SUCCESS != err_code)
				return;
		}
		err_code = nrfx_qspi_write(_qsp_buf, FLASH_PAGE_SIZE, _qsp_addr);
		if (NRFX_SUCCESS != err_code)
			return;
	}

	_qsp_addr = FLASH_CACHE_INVALID_ADDR;

	while (NRFX_ERROR_BUSY == nrfx_qspi_mem_busy_check())
	{}
}

void qspi_write(uint32_t dst, void const *src, int len, bool need_erase)
{
	uint32_t newAddr = dst & ~(FLASH_PAGE_SIZE - 1);

	if (newAddr != _qsp_addr)
	{
		qspi_flush(need_erase);
		_qsp_addr = newAddr;
		memcpy(_qsp_buf, (void*)newAddr, FLASH_PAGE_SIZE);
	}
	memcpy(_qsp_buf + (dst & (FLASH_PAGE_SIZE - 1)), src, len);
}


#else
void setup_qspi(bool writable) {
}

#endif
