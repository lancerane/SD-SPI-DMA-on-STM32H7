# SPI-SD card writes using DMA on STM32H7

Demonstrates near-10x speedup over standard CPU writes, with additional resource saving resulting from unblocking. Initialisation code generated using STCube MX.

### DMA 
DMA transfers are non-blocking, but on SD cards they must generally be done block by block. After commencing the DMA transfer, the function stack is exited. The CPU is recalled at the completion of each block transfer, to either start the next block or end the transfer. This is mediated via callbacks triggered at DMA completion.

### Call stack
Based on FatFS's f_write and associated SD/SPI drivers generated by CubeMX. 

f_write is called from main. The call stack then goes:
f_write -> disk_write -> (fnc_ptr in ff_gen_drv) -> user_write -> user_spi_write -> several more calls eventually culminating in HAL_SPI_Transmit which sends the data byte by byte.

DMA version replaces this with a multi-byte HAL_SPI_Transmit_DMA call. This is non-blocking, so after invocation the function stack is exited.

The public interface is f_write_dma which through conditionals handles both the transfer start and subsequent callbacks. Internally, the write logic required is handled by USER_diskio_SPI.

### Performance
Significant efficiency gains: at 480MHz and SPI baud at max, DMA transfers take just over 1/10 the time of equivalent CPU transfers, and during some of this time the CPU is free (it's not 100% because of CPU-mediated handshakes between blocks, etc).