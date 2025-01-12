/**********************************************************************/
/* This file has been generated automatically by Beremiz4Pico Manager */
/* WARNING : DO NOT EDIT MANUALLY THIS FILE                           */
/**********************************************************************/

#include "iec_types.h"
#include "../xplc.h"

IEC_BOOL* __IX0_8;
IEC_BOOL* __IX0_9;
IEC_BOOL* __QX0_0;
IEC_BOOL* __QX0_7;

void ConfigurePLCIO (xplc_t * xplc, uint32_t millis)
{
  __IX0_8 = &xplc->IO.di_int[DI_INTERN0].DI[DI_BTN1]; 
  __IX0_9 = &xplc->IO.di_int[DI_INTERN0].DI[DI_BTN2]; 
  __QX0_0 = &xplc->IO.do_int[DO_INTERN0].DO[DO_LED1_GN];
  __QX0_7 = &xplc->IO.do_int[DO_INTERN0].DO[DO_RELAY4];
}

void AcquirePLCInputs (xplc_t *xplc)
{
  *__IX0_8 = xplc->IO.di_int[0].DI[DI_BTN1]; 
  *__IX0_9 = xplc->IO.di_int[0].DI[DI_BTN2]; 
}

void UpdatePLCOutputs (xplc_t *xplc)
{
  xplc->IO.do_int[DO_INTERN0].DO[DO_LED1_GN] = *__QX0_0;
  xplc->IO.do_int[DO_INTERN0].DO[DO_RELAY4] = *__QX0_7;
}
