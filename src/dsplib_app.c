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

static uint16_t Magnitude[FFT_OUTPUT_POINTS];


uint32_t Noise = 0;

static void vTest_PerformFFT(void);

/**
  * @brief
  * @param
  * @retval
  */
uint16_t FFTCalculate(float coeff, uint16_t gain, uint16_t *out)
{	
	vTest_PerformFFT();
  
  	uint16_t FFTExceeded = 0;
  
  	for(int j = 0; j < FFT_OUTPUT_POINTS; j++) {
  		Magnitude[j] = (int)sqrt(FFTOutputData[2 * j] * FFTOutputData[2 * j] + FFTOutputData[2 * j + 1] * FFTOutputData[2 * j + 1]);
  		Noise += Magnitude[j];
  		out[j] = Magnitude[j];
  	}
  	Noise = (float)Noise / FFT_OUTPUT_POINTS;

  	for (int j = 0; j < FFT_OUTPUT_POINTS; j++) {
  		if (fabs(Magnitude[j] - Noise) * (gain / coeff) > 2 * Noise) FFTExceeded++;
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
static void vTest_PerformFFT(void)
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
			vF_dspl_fftR4b16N1024(FFTOutputData, FFTInputData);
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
