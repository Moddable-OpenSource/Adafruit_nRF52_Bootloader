
#ifndef __DFU_QSPI__
#define __DFU_QSPI__

void setup_qspi(bool writable);
void qspi_flush(bool need_erase);
void qspi_write(uint32_t dst, void const *src, int len, bool need_erase);

#endif
