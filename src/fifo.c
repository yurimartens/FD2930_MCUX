/**
  ******************************************************************************
  * @file    fifo.c
  * @author  Yuri Martenstev <yurimartens@gmail.com>
  * @brief   Simple static memory FIFO
  ******************************************************************************
  */ 
#include <fifo.h>
#include <string.h>


/**
  * @brief  
  * @param  
  * @retval  
  */
void FIFOInit(FIFO_t *fifo, void *mem, uint32_t itemSize, uint32_t size)
{
	fifo->mem = mem;
	fifo->head = mem;
	fifo->tail = mem;
	fifo->size = size;
	fifo->itemSize = itemSize;
	fifo->end = (uint32_t)fifo->mem + (size * itemSize);
	fifo->items = 0;
}

/**
  * @brief
  * @param
  * @retval
  */
FIFOError_t FIFOPush(FIFO_t *fifo, void *data)
{
	if (fifo->size == fifo->items) return FIFO_ERROR_FULL;
	if ((uint32_t)(fifo->head + fifo->itemSize) > fifo->end) {
		fifo->head = fifo->mem;
	}
	memcpy(fifo->head, data, fifo->itemSize);
	fifo->head += fifo->itemSize;
	fifo->items++;
	return FIFO_ERROR_NONE;
}

/**
  * @brief
  * @param
  * @retval
  */
FIFOError_t FIFOPop(FIFO_t *fifo, void *data)
{
	if (fifo->items == 0) return FIFO_ERROR_EMPTY;
	if ((uint32_t)(fifo->tail + fifo->itemSize) > fifo->end) {
		fifo->tail = fifo->mem;
	}
	memcpy(data, fifo->tail, fifo->itemSize);
	fifo->tail += fifo->itemSize;
	fifo->items--;
	return FIFO_ERROR_NONE;
}

/**
  * @brief
  * @param
  * @retval
  */
uint8_t FIFOIsEmpty(FIFO_t *fifo)
{
	if (fifo->items == 0) return 1;
	else return 0;
}

//------------------------------------------------------------------------------
// end
//------------------------------------------------------------------------------


