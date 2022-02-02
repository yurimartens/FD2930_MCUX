
#ifndef  __ARMCM3_TYPE_DEF_H
#define  __ARMCM3_TYPE_DEF_H




/*
*********************************************************************************************************
*                                              DATA TYPES
*                                         (Compiler Specific)
*********************************************************************************************************
*/

typedef unsigned char  BOOLEAN;
typedef unsigned char  INT8U;                    /* Unsigned  8 bit quantity                           */
typedef signed   char  INT8S;                    /* Signed    8 bit quantity                           */
typedef unsigned short INT16U;                   /* Unsigned 16 bit quantity                           */
typedef signed   short INT16S;                   /* Signed   16 bit quantity                           */
typedef unsigned int   INT32U;                   /* Unsigned 32 bit quantity                           */
typedef signed   int   INT32S;                   /* Signed   32 bit quantity                           */
typedef unsigned long  INT64U;                   /* Unsigned 64 bit quantity                           */
typedef signed   long  INT64S;                   /* Signed   64 bit quantity                           */
typedef float          FP32;                     /* Single precision floating point                    */
typedef double         FP64;                     /* Double precision floating point                    */


typedef struct {
   INT8U lo;
   INT8U hi;
} HILO;


typedef union {
   INT16U w;
   INT16U ival;
   INT8U b[2];
   HILO bval;
} TWO_BYTES;


typedef union {
   INT32U       dw;
   FP32         f;
   INT16U       w[2];
   TWO_BYTES    db[2];
   INT8U        b[4];
} FOUR_BYTES;

typedef union
{				// use this version for float data initializations
   float 	   f;
   INT32U       dw;
   INT16U        w[2];
   TWO_BYTES   db[2];
   INT8U        b[4];
} FOUR_BYTES_F;

#endif //ARMCM3_TYPE_DEF_H
