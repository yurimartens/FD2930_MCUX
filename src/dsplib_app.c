/**
  ******************************************************************************
  * @file    dsplib_app.c
  * @author  Yuri Martenstev <yurimartens@gmail.com>
  * @brief
  ******************************************************************************
  */
#include <dsplib_app.h>
#include <cr_dsplib.h>
#include <math.h>


int16_t 		*FFTInputData = (int16_t *)0x2007C000; /* AHB SRAM0 */
int16_t 		*FFTOutputData = (int16_t *)0x20080000; /* AHB SRAM1 */


uint32_t Noise = 0;

static void vTest_PerformFFT(uint32_t arrayIdx);

/**
  * @brief
  * @param
  * @retval
  */
uint16_t FFTCalculate(float coeff, uint16_t gain, uint16_t i, uint16_t *out)
{	
	vTest_PerformFFT(i);
  
  	uint16_t FFTExceeded = 0;
  	int offsetOut = i * FFT_OUTPUT_POINTS, offsetFFT = i * 2 * FFT_POINTS;
  
  	for(int j = 0; j < FFT_OUTPUT_POINTS; j++) {
  		out[offsetOut + j] = (int)sqrt(FFTOutputData[offsetFFT + 2 * j] * FFTOutputData[offsetFFT + 2 * j] + FFTOutputData[offsetFFT + 2 * j + 1] * FFTOutputData[offsetFFT + 2 * j + 1]);
  		Noise += out[offsetOut + j];
  	}
  	Noise = (float)Noise / FFT_OUTPUT_POINTS;

  	for (int j = 0; j < FFT_OUTPUT_POINTS; j++) {
  		if (fabs(out[offsetOut + j] - Noise) * (gain / coeff) > 2 * Noise) FFTExceeded++;
  	}
  	return FFTExceeded;
}

/*****************************************************************************
** Function name:   vTest_PerformFFT
**
** Descriptions:    Perform one of the FFT functions
**
** Parameters:	    None
**
** Returned value:  None
**
******************************************************************************/
static void vTest_PerformFFT(uint32_t arrayIdx)
{
	#ifdef NPOINTS
	{
		#if (NPOINTS == NPOINTS_64)
		{
			vF_dspl_fftR4b16N64(psi_Output, psi_Input);
		}
		#elif (NPOINTS == NPOINTS_256)
		{
			vF_dspl_fftR4b16N256(psi_Output, psi_Input);
		}
		#elif (NPOINTS == NPOINTS_1024)
		{
			vF_dspl_fftR4b16N1024(FFTOutputData + arrayIdx * 2 * NPOINTS_1024, FFTInputData + arrayIdx * 2 * NPOINTS_1024);
		}
		#elif (NPOINTS == NPOINTS_4096)
		{
			vF_dspl_fftR4b16N4096(psi_Output, psi_Input);
		}
		#else
		{
			#error "NPOINTS Not Valid!"
		}
		#endif
	}
	#else
	{
		#error "NPOINTS Not Defined!"
	}
	#endif
}

/*****************************************************************************
 **                            End Of File
 *****************************************************************************/
