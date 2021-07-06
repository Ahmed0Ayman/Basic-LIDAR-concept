/*
 * vl_6180.h
 *
 *  Created on: Jul 6, 2021
 *      Author: Ahmed_Ayman
 */

#ifndef INC_VL_6180_H_
#define INC_VL_6180_H_



#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "i2c.h"


#define VL6180_IDENTIFICATION_MODEL_ID_R				        0x000
#define VL6180_IDENTIFICATION_MODEL_REV_MAJOR_R				    0x001
#define VL6180_IDENTIFICATION_MODEL_REV_MINOR_R       			0x002
#define VL6180_IDENTIFICATION_MODULE_REV_MAJOR_R      			0x003
#define VL6180_IDENTIFICATION_MODULE_REV_MINOR_R      			0x004
#define VL6180_IDENTIFICATION_DATE_HI_R               			0x006
#define VL6180_IDENTIFICATION_DATE_LO_R               			0x007
#define VL6180_IDENTIFICATION_TIME_HI_R               			0x008
#define VL6180_IDENTIFICATION_TIME_LO_R               			0x009


#define VL6180_SYSTEM_MODE_GPIO0_R                  			0x010
#define VL6180_SYSTEM_MODE_GPIO1_R                  			0x011
#define VL6180_SYSTEM_GPIO_MODE_INT								0x000
#define VL6180_SYSTEM_GPIO_MODE_GPIO							0X010






#define VL6180_SYSTEM_HISTORY_CTRL_R                  			0x012
#define VL6180_SYSTEM_HISTORY_CTRL_EN				 			0x001
#define VL6180_SYSTEM_HISTORY_CTRL_CLEAR			  			0x005



#define VL6180_SYSTEM_INTERRUPT_CONFIG_GPIO_R         			0x014
#define VL6180_SYSTEM_INTERRUPT_CONFIG_GPIO_NEW_SAMPLE      	0x004

#define VL6180_SYSTEM_INTERRUPT_CLEAR_R               			0x015
#define VL6180_SYSTEM_INTERRUPT_CLEAR               			0x007

#define VL6180_SYSTEM_FRESH_OUT_OF_RESET_R            			0x016
#define VL6180_SYSTEM_GROUPED_PARAMETER_HOLD_R        			0x017

#define VL6180_SYSRANGE_START_R                       			0x018
#define VL6180_SYSRANGE_START_ONE_SHOT_START                    0x001
#define VL6180_SYSRANGE_START_CONTINUOUS_START                  0x003



#define VL6180_SYSRANGE_THRESH_HIGH_R                 			0x019  /* threshold high value from 0 -255 mm */
#define VL6180_SYSRANGE_THRESH_LOW_R                  			0x01A    /* threshold low value from 0 -255 mm */
#define VL6180_SYSRANGE_INTERMEASUREMENT_PERIOD_R     			0x01B   /* delay between each measurment step size = 10 ms */
#define VL6180_SYSRANGE_MAX_CONVERGENCE_TIME_R        			0x01C
#define VL6180_SYSRANGE_CROSSTALK_COMPENSATION_RATE_16Bit_R 	0x01E   /* 16 bit register */
#define VL6180_SYSRANGE_CROSSTALK_VALID_HEIGHT     	 			0x021
#define VL6180_SYSRANGE_EARLY_CONVERGENCE_ESTIMATE_16Bit_R  	0x022
#define VL6180_SYSRANGE_PART_TO_PART_RANGE_OFFSET_R   			0x024
#define VL6180_SYSRANGE_RANGE_IGNORE_VALID_HEIGHT_R   			0x025
#define VL6180_SYSRANGE_RANGE_IGNORE_THRESHOLD_16Bit_R      	0x026
#define VL6180_SYSRANGE_MAX_AMBIENT_LEVEL_MULT_R      			0x02C
#define VL6180_SYSRANGE_RANGE_CHECK_ENABLES_R         			0x02D
#define VL6180_SYSRANGE_VHV_RECALIBRATE_R             			0x02E
#define VL6180_SYSRANGE_VHV_REPEAT_RATE_R             			0x031



#define VL6180_RESULT_RANGE_STATUS_R                  			0x04D  /* Read only register */
#define VL6180_RESULT_INTERRUPT_STATUS_GPIO_R        			0x04F  /* interrupt status */
#define VL6180_RESULT_HISTORY_BUFFER_16Bit_R                 	0x052
#define VL6180_RESULT_HISTORY_BUFFER_READ(n)        			( VL6180_RESULT_HISTORY_BUFFER_16Bit_R  + (0x2 * n))
#define VL6180_RESULT_RANGE_VAL                     			0x062  /* final result for user to use in unit of mm */
#define VL6180_RESULT_RANGE_RAW                     			0x064  /* result value with offset applied */
#define VL6180_RESULT_RANGE_RETURN_RATE_16Bit_R              	0x066
#define VL6180_RESULT_RANGE_REFERENCE_RATE_16Bit_R           	0x068
#define VL6180_RESULT_RANGE_RETURN_SIGNAL_COUNT_16Bit_R      	0x06C
#define VL6180_RESULT_RANGE_REFERENCE_SIGNAL_COUNT_32Bit_R   	0x070
#define VL6180_RESULT_RANGE_RETURN_AMB_COUNT_32Bit_R        	0x074
#define VL6180_RESULT_RANGE_REFERENCE_AMB_COUNT_32Bit_R     	0x078
#define VL6180_RESULT_RANGE_RETURN_CONV_TIME_32Bit_R        	0x07C
#define VL6180_RESULT_RANGE_REFERENCE_CONV_TIME_32Bit_R     	0x080
#define VL6180_READOUT_AVERAGING_SAMPLE_PERIOD_R      			0x10A  /* increase the sampling period to decrease noise 0 - 255 */
#define VL6180_FIRMWARE_BOOTUP_R                      			0x119
#define VL6180_I2C_SLAVE_DEVICE_ADDRESS_R             			0x212  /* change device programmable 7-bit  slave address */


#define VL6180_REG_SIZE_8_bit                            		0x001
#define VL6180_REG_SIZE_16_bit 									0x002
#define VL6180_REG_SIZE_32_bit 									0x003




extern I2C_HandleTypeDef hi2c1;






void   init_vl6180(void);




/*
 * brief this function used to write to VL6180 registers
 * param : RegAddr The specific register address
 * Data  : pointer to the required data to be write into the register the data size must be equal the register size
 * return : void
 */
void VL6180_setRegister(uint16_t RegAddr,uint8_t Data);




/*
 * brief this function used to write to VL6180 registers
 * param : RegAddr The specific register address
 * Data  : pointer to the required data to be write into the register the data size must be equal the register size
 * return : void
 */
void VL6180_setRegister16bit(uint16_t RegAddr,uint32_t Data);




/*
 * brief this function used to read from VL6180 registers
 * param : RegAddr The specific register address
 * return : the read value from the specific register
 */
uint8_t VL6180_getRegister(uint16_t registerAddr);





/*
 * brief this function used to read from VL6180 registers
 * param : RegAddr The specific register address
 * return : the read value from the specific register
 */
uint16_t VL6180_getRegister16bit(uint16_t registerAddr);

/*
 * brief this function used to read from VL6180 registers
 * param : void
 * return : the read distance value from 0 - 255 mm
 */
uint8_t VL6180_GetDistance(void);









#endif /* INC_VL_6180_H_ */
