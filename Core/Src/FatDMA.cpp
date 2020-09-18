/*
 * DMA.cpp
 *
 *  Created on: 17 Sep 2020
 *      Author: lance
 */

#include "FatDMA.h"
#include "diskio.h"


void FatDMA::initialise() {

  // MX DMA Init stuff
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);



}

FRESULT FatDMA::f_write (FIL* fp, const void* buff, UINT btw, UINT* bw){
  DMAReady = false;
  this->bw = bw;
  return f_write_dma_start(fp, buff, btw);


}

// Called from the DMA transfer cplt callback. Determine if there are further blocks to transfer. If so,
// need to call the lower level funcs to handshake and start the next block. When the final block is done,
// the necessary logic is handled by higher level f_write_dma_cplt, which checks to see if the transfer was
// mmultiblock, in which case an additional stop token must be sent

int FatDMA::on_block_written(){ //1: ok; 0 err

  if (blocksLeft == 1) {
    blocksLeft --;

	FRESULT res = f_write_dma_cplt();
	DMAReady = true;
	return res == FR_OK ? 1 : 0;
  }


  else {
	xmit_datablock_cplt(); //handshake
	blocksLeft--;
	return xmit_datablock(nextBuff, 0xFC);

  }

}


