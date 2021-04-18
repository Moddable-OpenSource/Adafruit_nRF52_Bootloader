
#ifndef __DFU_UART__
#define __DFU_UART__

#define DFU_MODDABLE_SERIAL_MAGIC       0x347f00d5
#define SERIAL_TOKEN					0xbeefcafe

void setup_uart();
void dfu_uart_task();

#endif
