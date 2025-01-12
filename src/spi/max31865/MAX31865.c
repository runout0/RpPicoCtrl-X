
#include <math.h>
//#include  "main.h"
#include "MAX31865.h"
#include "MAX31865Conf.h"

#define MAX31865_WRITE_BIT 0x80
#define WAIT_RUN0 1 //  * 10ms
#define WAIT_RUN1 7 //  * 10ms

#define RTD_A 3.9083e-3
#define RTD_B -5.775e-7
//#########################################################################################################################
void  Max31865_delay(uint32_t delay_ms)
{	
	sleep_ms(delay_ms);
}
//#########################################################################################################################
void Max31865_readRegisterN(Max31865_t *max31865, uint8_t addr, uint8_t *buffer, uint8_t n)
{
	addr &= 0x7F;
	asm volatile("nop \n nop \n nop");
	asm volatile("nop \n nop \n nop");	
	CSn_Set(max31865->cs_pin);
	spi_write_blocking(max31865->spi, &addr, 1);
	asm volatile("nop \n nop \n nop");
	asm volatile("nop \n nop \n nop");	
	spi_read_blocking(max31865->spi, 0, buffer, n);	
	CSn_Set(csn_disable);
	asm volatile("nop \n nop \n nop");
	asm volatile("nop \n nop \n nop");	
}
//#########################################################################################################################
uint8_t Max31865_readRegister8(Max31865_t *max31865, uint8_t addr)
{
	uint8_t ret = 0;
	Max31865_readRegisterN(max31865, addr, &ret, 1);
	return ret;  
}
//#########################################################################################################################
uint16_t Max31865_readRegister16(Max31865_t *max31865, uint8_t addr)
{
	uint8_t buffer[2] = { 0, 0 };
	Max31865_readRegisterN(max31865, addr, buffer, 2);
	uint16_t ret = buffer[0];
	ret <<= 8;
	ret |=  buffer[1];
	return ret;
}
//#########################################################################################################################
void Max31865_writeRegister8(Max31865_t *max31865, uint8_t addr, uint8_t data)
{
	uint8_t buf[2];
	buf[0] = MAX31865_WRITE_BIT | addr;
	buf[1] = data;
	asm volatile("nop \n nop \n nop");
	asm volatile("nop \n nop \n nop");	
	CSn_Set(max31865->cs_pin);
	spi_write_blocking(max31865->spi, &buf[0], 2);
	CSn_Set(csn_disable);
	asm volatile("nop \n nop \n nop");
	asm volatile("nop \n nop \n nop");
}
//#########################################################################################################################
uint8_t Max31865_readFault(Max31865_t *max31865)
{
	return Max31865_readRegister8(max31865, MAX31856_FAULTSTAT_REG);
}
//#########################################################################################################################
void Max31865_clearFault(Max31865_t *max31865)
{
	uint8_t t = Max31865_readRegister8(max31865, MAX31856_CONFIG_REG);
	t &= ~0x2C;
	t |= MAX31856_CONFIG_FAULTSTAT;
	Max31865_writeRegister8(max31865, MAX31856_CONFIG_REG, t);
}
//#########################################################################################################################
void Max31865_enableBias(Max31865_t *max31865, uint8_t enable)
{
	uint8_t t = Max31865_readRegister8(max31865, MAX31856_CONFIG_REG);
	if (enable)
		t |= MAX31856_CONFIG_BIAS;
	else
		t &= ~MAX31856_CONFIG_BIAS;
	Max31865_writeRegister8(max31865, MAX31856_CONFIG_REG, t);
}
//#########################################################################################################################
void Max31865_autoConvert(Max31865_t *max31865, uint8_t enable)
{
	uint8_t t = Max31865_readRegister8(max31865, MAX31856_CONFIG_REG);
	if (enable)
		t |= MAX31856_CONFIG_MODEAUTO;
	else
		t &= ~MAX31856_CONFIG_MODEAUTO; 
	Max31865_writeRegister8(max31865, MAX31856_CONFIG_REG, t);
}
//#########################################################################################################################
void Max31865_setWires(Max31865_t *max31865, uint8_t numWires)
{
	uint8_t t = Max31865_readRegister8(max31865, MAX31856_CONFIG_REG);
	if (numWires == 3)
		t |= MAX31856_CONFIG_3WIRE;
	else
		t &= ~MAX31856_CONFIG_3WIRE;
	Max31865_writeRegister8(max31865, MAX31856_CONFIG_REG, t);
}
//#########################################################################################################################
void Max31865_setFilter(Max31865_t *max31865, uint8_t filterHz)
{
	uint8_t t = Max31865_readRegister8(max31865, MAX31856_CONFIG_REG);
	if (filterHz == 50)
		t |= MAX31856_CONFIG_FILT50HZ;
	else
		t &= ~MAX31856_CONFIG_FILT50HZ;
	Max31865_writeRegister8(max31865, MAX31856_CONFIG_REG, t);
}
//#########################################################################################################################
uint16_t Max31865_readRTD(Max31865_t *max31865)
{
	Max31865_clearFault(max31865);
	Max31865_enableBias(max31865, 1);
	Max31865_delay(5);
	uint8_t t = Max31865_readRegister8(max31865, MAX31856_CONFIG_REG);
	t |= MAX31856_CONFIG_1SHOT;
	Max31865_writeRegister8(max31865, MAX31856_CONFIG_REG, t);
	Max31865_delay(65);
	uint16_t rtd = Max31865_readRegister16(max31865, MAX31856_RTDMSB_REG);
	rtd >>= 1;
	return rtd;
}
//#########################################################################################################################
//#########################################################################################################################
//#########################################################################################################################
void  Max31865_init(Max31865_t *max31865, spi_inst_t *spi, enum spi_csn cs_pin, uint8_t  numwires, uint8_t filterHz)
{
	//	if (max31865->lock == 1)
	//		Max31865_delay(1);
	max31865->lock = 1;
	max31865->spi = spi;
	max31865->cs_pin = cs_pin; 
	Max31865_setWires(max31865, numwires);
	Max31865_enableBias(max31865, 0);
	Max31865_autoConvert(max31865, 0);
	Max31865_clearFault(max31865);
	Max31865_setFilter(max31865, filterHz);  
}

//#########################################################################################################################
max31865_st_t Max31865_readTempC(Max31865_t *max31865, const bool start)
{	
	if (max31865->state == st_idle  && start)
	{
		max31865->cnt_run0 = WAIT_RUN0;
		max31865->cnt_run1 = WAIT_RUN1;
		max31865->lock = 1;
		Max31865_clearFault(max31865);
		Max31865_enableBias(max31865, 1);
		max31865->state = st_run0; // Count 10ms (default)
	}
	
	if (max31865->state == st_run0  && !max31865->cnt_run0)
	{
		uint8_t t = Max31865_readRegister8(max31865, MAX31856_CONFIG_REG);
		t |= MAX31856_CONFIG_1SHOT;
		Max31865_writeRegister8(max31865, MAX31856_CONFIG_REG, t);
		max31865->state = st_run1; // Count 70ms (default)
	}

	if (max31865->state == st_run1  && !max31865->cnt_run1)
	{
		float Z1, Z2, Z3, Z4, Rt, temp;
		uint16_t rtd = Max31865_readRegister16(max31865, MAX31856_RTDMSB_REG);
		rtd >>= 1;
		Rt = rtd;
		Rt /= 32768;
		Rt *= _MAX31865_RREF;
		Z1 = -RTD_A;
		Z2 = RTD_A * RTD_A - (4 * RTD_B);
		Z3 = (4 * RTD_B) / _MAX31865_RNOMINAL;
		Z4 = 2 * RTD_B;
		temp = Z2 + (Z3 * Rt);
		temp = (sqrtf(temp) + Z1) / Z4;

		if (temp >= 0)
		{
			max31865->tempC = temp;
			max31865->reg_fault = Max31865_readFault(max31865);
			if (max31865->reg_fault == 0) max31865->state = st_rdy;
			else max31865->state = st_err;
		}
		else
		{
			Rt /= _MAX31865_RNOMINAL;
			Rt *= 100;    
			float rpoly = Rt;
			temp = -242.02;
			temp += 2.2228 * rpoly;
			rpoly *= Rt; // square
			temp += 2.5859e-3 * rpoly;
			rpoly *= Rt; // ^3
			temp -= 4.8260e-6 * rpoly;
			rpoly *= Rt; // ^4
			temp -= 2.8183e-8 * rpoly;
			rpoly *= Rt; // ^5
			temp += 1.5243e-10 * rpoly;
			max31865->tempC = temp;
			
			max31865->reg_fault = Max31865_readFault(max31865);
			if (max31865->reg_fault == 0) max31865->state = st_rdy;
			else max31865->state = st_err;
		}

		//init filter
		if (!max31865->first_run)
		{
			max31865->first_run = true;
			max31865->tempC_filtered = max31865->tempC;
		}
	}
	if (max31865->state == st_rdy  && !start)
	{
		max31865->lock = 0;
		max31865->state = st_idle;		
	}
	
	if (max31865->state == st_err  && !start)
	{
		max31865->lock = 0;
		max31865->state = st_idle;		
	}	
	return max31865->state;
}

// Call in 10ms-Task
void Max31865_Cyclic(Max31865_t *max31865)
{
	if (max31865->state == st_run0 && max31865->cnt_run0) max31865->cnt_run0--;
	if (max31865->state == st_run1 && max31865->cnt_run1) max31865->cnt_run1--;
}

//#########################################################################################################################
bool  Max31865_readTempF(Max31865_t *max31865, float *readTemp)
{  
	bool isOk = Max31865_readTempC(max31865, readTemp);
	*readTemp = (*readTemp * 9.0f / 5.0f) + 32.0f;
	return isOk;
}
//#########################################################################################################################
float Max31865_Filter(float	newInput, float	lastOutput, float efectiveFactor)
{
	return ((float)lastOutput*(1.0f - efectiveFactor)) + ((float)newInput*efectiveFactor) ;
}
//#########################################################################################################################

