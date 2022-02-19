/**
  ******************************************************************************
  * @file    fifo.h
  * @author  Yuri Martenstev <yurimartens@gmail.com>
  * @brief   Simple static memory FIFO
  ******************************************************************************
  */ 
  
#ifndef _FIFO_H
#define _FIFO_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>


typedef struct {
	void			*mem;
	void			*head;
	void			*tail;
	uint32_t		size;			// in items
	uint32_t		end;
	uint32_t		itemSize;
	uint32_t		items;
} FIFO_t;

typedef enum {FIFO_ERROR_NONE = 0, FIFO_ERROR_EMPTY, FIFO_ERROR_FULL} FIFOError_t;


void FIFOInit(FIFO_t *fifo, void *mem, uint32_t itemSize, uint32_t size);
FIFOError_t FIFOPush(FIFO_t *fifo, void *data);
FIFOError_t FIFOPop(FIFO_t *fifo, void *data);
uint8_t FIFOIsEmpty(FIFO_t *fifo);



#ifdef __cplusplus
}
#endif

#endif // _FIFO_H
//------------------------------------------------------------------------------
// END
//------------------------------------------------------------------------------
