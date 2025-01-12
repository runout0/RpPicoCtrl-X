#ifndef _MAX31865_H
#define _MAX31865_H

/*
  Author:     Nima Askari
  WebSite:    http://www.github.com/NimaLTD
  Instagram:  http://instagram.com/github.NimaLTD
  Youtube:    https://www.youtube.com/channel/UCUhY7qY1klJm1d2kulr9ckw
  
  Version:    1.0.0
  
  
  Reversion History:
  
  (1.0.0)
  First Release.

*/

#ifdef __cplusplus
extern "C" {
#endif

	//#include <stdbool.h>
#include "pico/stdlib.h"
#include "..\xspi.h"
	
#define MAX31856_CONFIG_REG             0x00
#define MAX31856_CONFIG_BIAS            0x80
#define MAX31856_CONFIG_MODEAUTO        0x40
#define MAX31856_CONFIG_MODEOFF         0x00
#define MAX31856_CONFIG_1SHOT           0x20
#define MAX31856_CONFIG_3WIRE           0x10
#define MAX31856_CONFIG_24WIRE          0x00
#define MAX31856_CONFIG_FAULTSTAT       0x02
#define MAX31856_CONFIG_FILT50HZ        0x01
#define MAX31856_CONFIG_FILT60HZ        0x00

#define MAX31856_RTDMSB_REG             0x01
#define MAX31856_RTDLSB_REG             0x02
#define MAX31856_HFAULTMSB_REG          0x03
#define MAX31856_HFAULTLSB_REG          0x04
#define MAX31856_LFAULTMSB_REG          0x05
#define MAX31856_LFAULTLSB_REG          0x06
#define MAX31856_FAULTSTAT_REG          0x07


#define MAX31865_FAULT_HIGHTHRESH       0x80
#define MAX31865_FAULT_LOWTHRESH        0x40
#define MAX31865_FAULT_REFINLOW         0x20
#define MAX31865_FAULT_REFINHIGH        0x10
#define MAX31865_FAULT_RTDINLOW         0x08
#define MAX31865_FAULT_OVUV             0x04
  
	  //#########################################################################################################################
	
	typedef enum max31865_st {
		st_idle  = 0,
		st_run0,
		st_run1,
		st_rdy,
		st_err,
		st_COUNT
	} max31865_st_t;
		
	typedef struct
	{
		spi_inst_t *spi;
		enum spi_csn cs_pin;
		uint8_t lock;	  
		max31865_st_t state;
		uint8_t cnt_run0;
		uint8_t cnt_run1;
		bool first_run;
		float tempC;
		float tempC_filtered;
		uint8_t reg_fault;
	}Max31865_t;
	//#########################################################################################################################
	void  Max31865_init(Max31865_t *max31865, spi_inst_t *spi, enum spi_csn cs_pin, uint8_t  numwires, uint8_t filterHz);
	max31865_st_t Max31865_readTempC(Max31865_t *max31865, const bool start);
	void Max31865_Cyclic(Max31865_t *max31865);
	bool  Max31865_readTempF(Max31865_t *max31865, float *readTemp);
	float Max31865_Filter(float	newInput, float	lastOutput, float efectiveFactor);
   
	// TIM erweitert
	uint8_t Max31865_readRegister8(Max31865_t *max31865, uint8_t addr);  
 
	//#########################################################################################################################
#ifdef __cplusplus
}
#endif



#endif
