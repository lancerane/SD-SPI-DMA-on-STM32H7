/*
 * DMA.h
 *
 *  Created on: 17 Sep 2020
 *      Author: lance
 */

#include "main.h"
#include "ff.h"
#include "diskio.h"


#ifndef SRC_DMA_H_
#define SRC_DMA_H_

class FatDMA {
  public:
    FatDMA(){};

    void initialise();
    FRESULT f_write (FIL* fp, const void* buff, UINT btw, UINT* bw);
    FRESULT f_write_dma_cplt ();
    int on_block_f_written();

    ~FatDMA(){};

    bool DMAReady = true;
    int writeError = 0;
    UINT* bw;

  private:
    DMA_HandleTypeDef hdma_spi3_tx;
    FRESULT res;
	FATFS *fs;
	DWORD clst, sect;

	const BYTE* nextBuff;
	const BYTE *wbuff;
	UINT btw;

	FIL* fp;
	int blocksLeft = 0;
	bool multi; // ?multi-block write
	UINT wcnt, cc, csect;


    FRESULT f_write_dma_start(FIL* fp, const void* buff, UINT btw);
    FRESULT f_write_dma_loop();
    int USER_SPI_write_dma_start (BYTE drv, const BYTE *buff, DWORD sector, UINT count);

    DRESULT USER_SPI_write_dma_cplt ();
    int xmit_datablock (const BYTE *buff, BYTE token);
    static int xmit_datablock_cplt();

};

#endif /* SRC_DMA_H_ */
