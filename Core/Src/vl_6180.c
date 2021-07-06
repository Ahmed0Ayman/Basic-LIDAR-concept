/*
 * vl_6180.c
 *
 *  Created on: Jul 6, 2021
 *      Author: Ahmed_Ayman
 */


#include "vl_6180.h"


/*
 * brief this function used to write to VL6180 registers
 * param : RegAddr The specific register address
 * Data  : pointer to the required data to be write into the register the data size must be equal the register size
 * return : void
 */
void VL6180_setRegister(uint16_t RegAddr,uint8_t Data)
{
	uint8_t data = Data ;
	HAL_I2C_Mem_Write(&hi2c1,0x52,RegAddr,2,(uint8_t *)&data,1,2);

}



/*
 * brief this function used to write to VL6180 registers
 * param : RegAddr The specific register address
 * Data  : pointer to the required data to be write into the register the data size must be equal the register size
 * return : void
 */
void VL6180_setRegister16bit(uint16_t RegAddr,uint32_t Data)
{
	uint32_t data = (((Data>>8)&0xff)|((Data<<8)&0xff00)) ;
	uint16_t addr = (((RegAddr>>8)&0xff)|((RegAddr<<8)&0xff00));
	HAL_I2C_Mem_Write(&hi2c1,0x52,addr,2,(uint8_t *)&data,2,2);

}


/*
 * brief this function used to read from VL6180 registers
 * param : RegAddr The specific register address
 * return : the read value from the specific register
 */
uint8_t VL6180_getRegister(uint16_t registerAddr)
{

	uint8_t pData =0;
	HAL_I2C_Mem_Read(&hi2c1,0x52,registerAddr,2,(uint8_t *) &pData, 1,2);
	return pData ;


}



/*
 * brief this function used to read from VL6180 registers
 * param : RegAddr The specific register address
 * return : the read value from the specific register
 */
uint16_t VL6180_getRegister16bit(uint16_t registerAddr)
{

	uint16_t pData =0;
	HAL_I2C_Mem_Read(&hi2c1,0x52,registerAddr,2,(uint8_t *) &pData, 2,2);
	pData = (((pData>>8)&0xff)|((pData<<8)&0xff00)) ;
	return pData ;


}


/*
 * brief this function used to read from VL6180 registers
 * param : void
 * return : the read distance value from 0 - 255 mm
 */
uint8_t VL6180_GetDistance(void)
{
	uint8_t pData =0;
	VL6180_setRegister(VL6180_SYSRANGE_START_R, 0x03); //Start Single shot mode
	HAL_Delay(100);

	VL6180_setRegister(VL6180_SYSTEM_INTERRUPT_CLEAR_R, 0x07);
	HAL_I2C_Mem_Read(&hi2c1,0x52,VL6180_RESULT_RANGE_VAL,2, &pData, 1,2);
	return pData ;


}


void   init_vl6180(void)
{

	  VL6180_setRegister(VL6180_SYSTEM_INTERRUPT_CONFIG_GPIO_R, (4 << 3)|(4) ); // Set GPIO1 high when sample complete


	  VL6180_setRegister(VL6180_READOUT_AVERAGING_SAMPLE_PERIOD_R, 0x30); //Set Avg sample period
	  VL6180_setRegister(VL6180_SYSRANGE_VHV_REPEAT_RATE_R, 0xFF); // Set auto calibration period (Max = 255)/(OFF = 0)
	  VL6180_setRegister(VL6180_SYSRANGE_VHV_RECALIBRATE_R, 0x01); // perform a single temperature calibration

	  VL6180_setRegister(VL6180_SYSRANGE_INTERMEASUREMENT_PERIOD_R, 0x09); // Set default ranging inter-measurement period to 100ms
	  //Additional settings defaults from community
	  VL6180_setRegister(VL6180_SYSRANGE_MAX_CONVERGENCE_TIME_R, 0x32);
	  VL6180_setRegister(VL6180_SYSRANGE_RANGE_CHECK_ENABLES_R, 0x10 | 0x01);
	  VL6180_setRegister16bit(VL6180_SYSRANGE_EARLY_CONVERGENCE_ESTIMATE_16Bit_R, 0x7B );

	  VL6180_setRegister(VL6180_READOUT_AVERAGING_SAMPLE_PERIOD_R,0x30);

}


