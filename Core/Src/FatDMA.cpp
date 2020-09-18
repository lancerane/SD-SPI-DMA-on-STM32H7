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


