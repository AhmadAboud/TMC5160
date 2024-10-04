/*
 * inputsCat9555.hpp
 *
 *  Created on: Sep 26, 2023
 *      Author: ahmad.alaboud
 */

#ifndef INC_INPUTSCAT9555_HPP_
#define INC_INPUTSCAT9555_HPP_

#include "stm32h7xx_hal.h"

class inputsCat9555
{
public:
	inputsCat9555 (I2C_HandleTypeDef i2c);
	uint16_t mem_read_reg(uint8_t register_pointer);

private:

    I2C_HandleTypeDef mI2C;

	#define IN_PORT0    0
	#define IN_PORT1    1
	#define OUT_PORT0   2
	#define OUT_PORT1   3
	#define PI_PORT0    4
	#define PI_PORT1    5
	#define CFG_PORT0   6
	#define CFG_PORT1   7
	unsigned char p0_val = 0x00;
	unsigned char p1_val = 0xff;
	unsigned int t = 0;
	unsigned char int_flag = 0;
	uint16_t DEV_ADDR  = 0X40;
	uint16_t  DEV_ADDR2  = 0X41;



};







#endif /* INC_INPUTSCAT9555_HPP_ */
