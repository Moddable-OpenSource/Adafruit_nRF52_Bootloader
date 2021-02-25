#include <nrfx_qspi.h>
#include <boards.h>
//#include "ftdi_trace.h"

#define QSPI_STD_CMD_WRSR        0x01
#define QSPI_STD_CMD_RSTEN       0x66
#define QSPI_STD_CMD_RST         0x99

#define QSPI_SR_QUAD_ENABLE_BYTE 0x40

//#define QSPI_XIP_START_ADDR      0x12000000		// in boards.h


static void configure_memory(void)
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

void setup_qspi(void)
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
	configure_memory();

/*
	nrfx_irq_handler_t reset_handler = (nrfx_irq_handler_t)((volatile uint32_t const *)QSPI_XIP_START_ADDR)[1];


	SCB->VTOR = QSPI_XIP_START_ADDR;

	// Boot app
	reset_handler();
*/
}
