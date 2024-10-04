/*
 * AD7794.cpp
 *
 *  Created on: 05.10.2023
 *      Author: ahmad.alaboud
 */

#include "AD7794.hpp"
#include "main.h"
#include "stm32h7xx_hal.h"



AD7794 :: AD7794(SPI_HandleTypeDef spi)
{
	 nSPI = spi;
}

void AD7794 :: begin()
{
  HAL_GPIO_WritePin(GPIOB, CS_ADC_Pin,GPIO_PIN_SET);

  reset();

  HAL_Delay(2); //4 times the recomended period

  //Apply the defaults that were set up in the constructor
  //Should add a begin(,,) method that lets you override the defaults
  for(uint8_t i = 0; i < 6; i++){ //<-- Channel count stuff needs to be handled better!!
    setActiveCh(i);

    writeModeReg();
    writeConfReg();
  }

  setActiveCh(0); //Set channel back to 0

  read(0); //Take a through away reading because the very first value read is usually junk
}

void AD7794 :: reset()
{
  // Speed set to 4MHz, SPI mode set to MODE 3 and Bit order set to MSB first.
	HAL_StatusTypeDef ResetHA;
  HAL_GPIO_WritePin(GPIOB, CS_ADC_Pin,GPIO_PIN_RESET);
  for(uint8_t i=0;i<4;i++)
  {
   	uint8_t dataToSend[1] = { 0xFF};
    ResetHA = HAL_SPI_Transmit(&nSPI, dataToSend , 1, 100);
  }
  HAL_GPIO_WritePin(GPIOB, CS_ADC_Pin,GPIO_PIN_SET);
}

void  AD7794 ::  setActiveCh(uint8_t ch)
{
  if(ch < 8){
    currentCh = ch;
    buildConfReg();
    writeConfReg();
  }
}

//Sets bipolar/unipolar mode for currently selected channel
void AD7794:: setBipolar(uint8_t ch, bool isBipolar)
{
  setActiveCh(ch);
  buildConfReg();
  writeConfReg();
}

void AD7794::setInputBuffer(uint8_t ch, bool isBuffered)
{

}

void AD7794::setGain(uint8_t ch, uint8_t gain)
{
  setActiveCh(ch);
  buildConfReg();
  writeConfReg();
}

void AD7794::setEnabled(uint8_t ch, bool enabled)
{

}


/******************************************************
Sets low byte of the mode register. The high nibble
must be 0, the low nibble sets the mode (FS3->FS0)
[EXAMPLE '\x01' = 470Hz]
refer to datasheet for modes
  FS3 | FS2 | FS1 | FS0 | frequency (hz) | Tsettle (ms)
  0   |  0  |  0  |  0  |  X             |  X
  0   |  0  |  0  |  1  |  470           |  4
  0   |  0  |  1  |  0  |  242           |  8
  0   |  0  |  1  |  1  |  123           |  16
  0   |  1  |  0  |  0  |   62           |  32
  ....
*/
void AD7794::setUpdateRate(double rate)
{
  uint8_t bitMask;
  //Map requested to next available range
  if(rate <= 4.17)
    {bitMask = 0x0F;}     //0x0F = 74 dB (50 Hz and 60 Hz)
  else if(rate > 4.17 && rate <= 6.25)
    {bitMask = 0x0E;}     //0x0E = 72 dB (50 Hz and 60 Hz)
  else if(rate > 6.25 && rate <= 8.33)
    {bitMask = 0x0D;}     //0x0D = 70 dB (50 Hz and 60 Hz)
  else if(rate > 8.33 && rate <= 10)
    {bitMask = 0x0C;}     //0x0C = 69 dB (50 Hz and 60 Hz)
  else if(rate > 10 && rate <= 12.5)
    {bitMask = 0x0B;}     //0x0B = 66 dB (50 Hz and 60 Hz)
  else if(rate > 12.5 && rate <= 16.7) //This ones wierd, have to pick one
    {bitMask = 0x0A;}     //0x0A = 65 dB(50Hx and 60Hz)
    //{bitMask = 0x09;}   //0x09 = 80 dB(50Hz)65 dB(50Hx and 60Hz)
  else if(rate > 16.7 && rate <= 19.6)
    {bitMask = 0x08;}     //0x08 = 90 dB(60 Hz only) *Should be best option in US
  else if(rate > 19.6 && rate <= 33.2)
    {bitMask = 0x07;}
  else if(rate > 33.2 && rate <= 39)
    {bitMask = 0x06;}
  else if(rate > 39 && rate <= 50)
    {bitMask = 0x05;}
  else if(rate > 50 && rate <= 62)
    {bitMask = 0x04;}
  else if(rate > 62 && rate <= 123)
    {bitMask = 0x03;}
  else if(rate > 123 && rate <= 242)
    {bitMask = 0x02;}
    else if(rate > 242)
      {bitMask = 0x01;} //anything above 242 set to 470 Hz

  modeReg &= 0xFFF0; //Zero off low nibble
  modeReg |= bitMask;

  writeModeReg();
  //TODO: Put table from datasheet in comments, in header file
}


//Enable internal bias voltage for specified channel
void AD7794::setVBias(uint8_t ch, bool isEnabled)
{
  setActiveCh(ch);

  buildConfReg();
  writeConfReg();
}

//Set the votage reference source for the specified channel
void AD7794::setRefMode(uint8_t ch, uint8_t mode){

  setActiveCh(ch);

  //Only 0,1,2 are valid
  if(mode < 3){
    buildConfReg();
    writeConfReg();
  }

}


uint32_t AD7794::getConvResult()
{
  uint8_t inByte;
  uint32_t result  = 0;
  uint8_t send[1] = {0x58};

  //SPI.beginTransaction(spiSettings); //We should still be in our transaction
  HAL_GPIO_WritePin(GPIOB, CS_ADC_Pin,GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(&nSPI, send, &inByte, 1, 1000);

  //Read 24 bits one byte at a time, and put in an unsigned long

  HAL_SPI_TransmitReceive(&nSPI, send, &inByte, 1, 1000);
  result = inByte;
  HAL_SPI_TransmitReceive(&nSPI, send, &inByte, 1, 1000);
  result = result << 8;
  result = result | inByte;
  HAL_SPI_TransmitReceive(&nSPI, send, &inByte, 1, 1000);
  result = result << 8;
  result = result | inByte;

  //De-assert CS if not in continous conversion mode
  if(isSnglConvMode){
	  HAL_GPIO_WritePin(GPIOB, CS_ADC_Pin,GPIO_PIN_SET);
  }

  return result;
}

//Experiment with reading all active channels, this may be the way I go in the future UNTESTED
void AD7794::read(float *buf, uint8_t bufSize)
{
  uint8_t readingCnt = 0 ;

  for(int i = 0;  i <8; i++){

      if(readingCnt < bufSize){
        buf[readingCnt] = read(i);
      }else{
        //buffer too small!
        //Maybe return an error code?
      }
      readingCnt++;
    }
  }


float AD7794::read(uint8_t ch)
{
  //Lets the conversion result
  uint32_t adcRaw = getReadingRaw(ch);
  //Serial.print(adcRaw);
  //Serial.print(' ');
  float result;

	  return (((float)adcRaw / 16777216- 1) * 1.17)*100; //Bipolar, not sure what mode for sensor



}

/* zero - Record offsets to apply to the channel (if active) */
void AD7794::zero(uint8_t ch)
{

  if(true){
    //read(ch); //Take a throw away reading first -This is now done in begin,shouldn't be needed here anymore

   read(ch);
  }
}

/* zero - Record offsets to apply to all active
   channels. (NOT temperature an AVDD monitor)
*/
void AD7794::zero()
{
  for(int i = 0;  i < 8-2 ; i++){
    zero(i);
  }
}


/* offset - Returns channel offset value. I'm not sure if this will be needed
   in the long run, but it's usefull for testing now.
*/
float AD7794::offset(uint8_t ch)
{


}

//Added 11-14-2021
int AD7794::waitForConvReady (uint32_t timeout){
    uint8_t inByte;
    uint8_t send = 0x40;
    HAL_SPI_TransmitReceive(&nSPI, &send, &inByte, 1, 1000);

    //Read status register
    uint8_t send1 = 0xFF;
    HAL_SPI_TransmitReceive(&nSPI, &send1, &inByte, 1, 1000);

    if((inByte & 0x80) == 0){
      //bit cleared, conversion is ready
      return 0;
    }

  return -1;
}
//_______________________________________________________

uint32_t AD7794::getReadingRaw(uint8_t ch)
{
  static bool contConvStarted = false;
  // setActiveCh() also calls SPI.beginTransaction(), This causes a lockup on esp32
  // so the call has been moved down.
  //SPI.beginTransaction(spiSettings);

  if(isSnglConvMode || !contConvStarted){ //Added 11-14-2021

    setActiveCh(ch);

    startConv();
    contConvStarted = true;
  }

  if(!isSnglConvMode){ //Added 11-14-2021
	  HAL_GPIO_WritePin(GPIOB, CS_ADC_Pin,GPIO_PIN_RESET);
  }
  //uint32_t t = millis();

  // #if defined (ESP8266) //Workaround until I figure out how to read the MISO pin status on the ESP8266
  //   delay(10);           //This delay is only appropriate for the fastest rate (470 Hz)
  // #else

    //Commented out to test status reg read method
    // while(digitalRead(MISO) == HIGH){
    //   if((millis() - t) >= convTimeout){
    //     //Serial.print("getReadingRaw Timeout");
    //     break;
    //   }
    // }

    //Test status read method NOTE: Should do something with return value (-1 if timeout)
    waitForConvReady(convTimeout);

  // #endif

  uint32_t adcRaw = getConvResult();

  if(isSnglConvMode){ //Added 11-14-2021

  }
  return adcRaw;
}


// OK, for now I'm not going to handle the spi transaction inside the startConversion()
// and getConvResult() functions. They are very low level and the behavior is different
// depending on conversion mode (and other things ?). think I will make these private
// and create a higher level functions to get readings.
void AD7794::startConv()
{
  //Write out the mode reg, but leave CS asserted (LOW)
  //SPI.beginTransaction(spiSettings);
	HAL_GPIO_WritePin(GPIOB, CS_ADC_Pin,GPIO_PIN_RESET);
	uint8_t send = 0x08;
	uint8_t Recv[2] = {0,0};
	HAL_SPI_TransmitReceive(&nSPI, &send, Recv, 2, 1000);
   modeReg |= (Recv[0] & 0xFF00);
   modeReg |= (Recv[1]& 0x00FF);


  //Don't end the transaction yet. for now this is going to have to hold on
  //to the buss until the conversion is complete. not sure if there is a way around this
}


//////// Private helper functions/////////////////

//This has been changed and is untested
void AD7794::buildConfReg()
{
  confReg = 0x0010; //wipe it back to default

  confReg = (getGainBits(64) << 8) | currentCh;
}

void AD7794::writeConfReg()
{

	HAL_GPIO_WritePin(GPIOB, CS_ADC_Pin,GPIO_PIN_RESET);
	uint8_t send =  0x10;
	uint8_t Recv[2] = {0,0};

	HAL_SPI_TransmitReceive(&nSPI, &send, Recv, 2, 1000);
	HAL_GPIO_WritePin(GPIOB, CS_ADC_Pin,GPIO_PIN_SET);
	   modeReg |= (Recv[0] & 0xFF00);
	   modeReg |= (Recv[1]& 0x00FF);

}

void AD7794::writeModeReg()
{
	HAL_GPIO_WritePin(GPIOB, CS_ADC_Pin,GPIO_PIN_RESET);

	uint8_t send =  0x10;
	uint8_t Recv[2] = {0,0};

	HAL_SPI_TransmitReceive(&nSPI, &send, Recv, 2, 1000);
	HAL_GPIO_WritePin(GPIOB, CS_ADC_Pin,GPIO_PIN_SET);
	   modeReg |= (Recv[0] & 0xFF00);
	   modeReg |= (Recv[1]& 0x00FF);
}

uint8_t AD7794::getGainBits(uint8_t gain)
{
  uint8_t gainBits = 0;

  switch(gain){
    case 1:
      gainBits |= 0x00;
      break;
    case 2:
      gainBits |= 0x01;
      break;
    case 4:
      gainBits |= 0x02;
      break;
    case 8:
      gainBits |= 0x03;
      break;
    case 16:
      gainBits |= 0x04;
      break;
    case 32:
      gainBits |= 0x05;
      break;
    case 64:
      gainBits |= 0x06;
      break;
    case 128:
      gainBits |= 0x07;
      break;
    default:
      //OOPS!, shouldn't be here, well just set it to 0x00 (gain = 1)
      gainBits |= 0x00;
      break;
  }
  return gainBits;
}

