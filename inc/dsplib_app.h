/**
  ******************************************************************************
  * @file    dsplib_app.h
  * @author  Yuri Martenstev <yurimartens@gmail.com>
  * @brief
  ******************************************************************************
  */
#ifndef _DSPLIB_APP_H
#define _DSPLIB_APP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define FFT_POINTS						1024
#define FFT_OUTPUT_POINTS				40


extern int16_t 		*FFTInputData;
extern int16_t 		*FFTOutputData;

extern uint16_t 	FFTMagnitude[FFT_OUTPUT_POINTS];


uint16_t FFTCalculate(float coeff, uint16_t gain, uint16_t *out);

#endif // DSPLIB_APP

