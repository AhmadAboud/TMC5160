/*
 * AD7794.hpp
 *
 *  Created on: Oct 5, 2023
 *      Author: ahmad.alaboud
 */

#ifndef INC_AD7794_HPP_
#define INC_AD7794_HPP_
#include "stm32h7xx_hal.h"

class AD7794 {


public :

	    AD7794(SPI_HandleTypeDef spi);

		void begin();
		void reset();

		void setBipolar(uint8_t ch, bool isBipolar);
		void setInputBuffer(uint8_t ch, bool isBuffered);
		void setGain(uint8_t ch, uint8_t gain);
		void setEnabled(uint8_t ch, bool enabled);
		void setVBias(uint8_t ch, bool enabled);
		void setRefMode(uint8_t ch, uint8_t mode);

		//void setUpdateRate(uint8_t bitMask);
		void setUpdateRate(double rate);
		//void setConvMode(bool isSingle); //Deprecated
		void setMode(); //Added 11-15-2021
		void setChopEnabled(bool enabled = true); //Added 11-14-2021
		void setActiveCh(uint8_t ch);

		uint32_t getReadingRaw(uint8_t ch);
		//float getReadingVolts(uint8_t ch);
		float TempSensorRawToDegC(uint32_t rawData);

		void read(float *buf, uint8_t bufSize); //experimental
		float read(uint8_t ch);
		void zero(uint8_t ch);  //Single channel
		void zero();            //All enabled, external channels (not internal temperature or VCC monitor)
		float offset(uint8_t ch);

private:


	 //Private helper functions
	    void startConv();
	    uint32_t getConvResult();
	    void writeConfReg();
	    void writeModeReg();
	    void buildConfReg();
	    //void setActiveCh(uint8_t ch);
	    uint8_t getGainBits(uint8_t gain);

	    int waitForConvReady(uint32_t timeout); //Added 11-14-2021
	    uint8_t CS;
	    uint8_t currentCh;
	    float vRef;


	    uint16_t modeReg; //holds value for 16 bit mode register
	    uint16_t confReg; //holds value for 16 bit configuration register

	    bool isSnglConvMode;

	    const uint16_t convTimeout = 480; // This should be set based on update rate eventually
	    SPI_HandleTypeDef nSPI;







};




#endif /* INC_AD7794_HPP_ */
