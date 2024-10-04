/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "EventLoop.hpp"
#include "hx711.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim16;
TIM_HandleTypeDef htim17;
TIM_HandleTypeDef htim24;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM17_Init(void);
static void MX_TIM24_Init(void);
static void MX_TIM16_Init(void);
static void MX_SPI1_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
double calcFrequency(double m_dCycle, double m_dRampTimeMS,double m_dSollSpeedDriveHZ )
{
	double PI =3.14159999999478589672;
	double m_dCurrentlySpeedDriveHz;
    m_dCurrentlySpeedDriveHz = m_dSollSpeedDriveHZ*pow(sin(m_dCycle*(1/m_dRampTimeMS)-PI/2),10);
 	return m_dCurrentlySpeedDriveHz;
}

void Thread(double m_dSollSpeedDriveHZ, double m_dCycle, double m_dRampTimeMS, double m_dCurrentlySpeedDriveHz)
{
	int m_bStart = 0, m_bReverseRun = 0;

    if(m_bReverseRun)
    {
    	HAL_GPIO_WritePin(GPIOC, DirSM40_Pin,GPIO_PIN_SET);
	}

    calcFrequency( m_dCycle, m_dRampTimeMS, m_dSollSpeedDriveHZ);
	m_dCycle++;
	__HAL_TIM_SET_AUTORELOAD (&htim17,calcFrequency( m_dCycle, m_dRampTimeMS, 1/m_dSollSpeedDriveHZ)); // 31khz
	__HAL_TIM_SET_PRESCALER (&htim17,5);
    __HAL_TIM_SET_COMPARE (&htim17,TIM_CHANNEL_1,50);
	HAL_TIM_PWM_Start(&htim17, TIM_CHANNEL_1);
	TmcStart();

	if (m_dCycle > m_dRampTimeMS)
	{
		m_dCycle = m_dRampTimeMS;
	 }

	if (!m_bStart)
	{
	calcFrequency(m_dCycle, m_dRampTimeMS, m_dSollSpeedDriveHZ);
	m_dCycle--;
	}

	if (m_dCycle <= 0)
	{
		HAL_TIM_PWM_Stop(&htim17, TIM_CHANNEL_1);
		m_dCycle = 0;

	}
}

void TmcStart() // Chopper Config. für TMC5160
{
	uint8_t pDataSend1[5]={0xEC, 0, 0x1, 0, 0xC3};
	uint8_t pDataRecv1[5]={0, 0, 0, 0, 0};
	HAL_GPIO_WritePin(csSM40_GPIO_Port, csSM40_Pin,GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi1, pDataSend1, pDataRecv1, 5, 1000);
	HAL_GPIO_WritePin(csSM40_GPIO_Port, csSM40_Pin,GPIO_PIN_SET);

}
volatile uint32_t timerOverflow = 0;

uint32_t ReadHardwareTimer()
{
    uint32_t count = htim17.Instance->CNT;
    return count;
}

uint32_t ReadTimerOverflow()
{

    return timerOverflow;
}


uint16_t mem_read_reg(uint16_t register_pointer)

{    HAL_StatusTypeDef status = HAL_OK;    uint16_t return_value = 0;
     while (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(0x40), 3, 100) != HAL_OK) {}
     uint8_t* Data;
     status = HAL_I2C_Mem_Read(&hi2c1,(uint16_t)(0x40),(uint16_t)register_pointer, I2C_MEMADD_SIZE_16BIT,  &return_value, 2,   100);
    /* Check the communication status */
    if(status != HAL_OK) {
    }
    return return_value;
}

void writeAd7790 (uint8_t ui8address, uint8_t ui8value)
{
    uint8_t Data[2]={ui8address,ui8value};
    HAL_StatusTypeDef ret3;
	uint8_t Data1[2]={ui8address,ui8value};
	if (ui8address != RESET)
		{
			HAL_GPIO_WritePin(GPIOB, CS_ADC_Pin,GPIO_PIN_RESET);
			ret3  = HAL_SPI_Transmit(&hspi2, Data, 2, 1000);
		   	HAL_GPIO_WritePin(GPIOB, CS_ADC_Pin,GPIO_PIN_SET);
		} else
		{
			HAL_GPIO_WritePin(GPIOB, CS_ADC_Pin,GPIO_PIN_RESET);
			ret3  = HAL_SPI_Transmit(&hspi2, Data1, 2, 1000);
			ret3  = HAL_SPI_Transmit(&hspi2, Data1, 2, 1000);
			ret3  = HAL_SPI_Transmit(&hspi2, Data1, 2, 1000);
			ret3  = HAL_SPI_Transmit(&hspi2, Data1, 2, 1000);
			HAL_GPIO_WritePin(GPIOB, CS_ADC_Pin,GPIO_PIN_SET);
		}

}
uint16_t readAd7790 (uint8_t ui8address)
{
	uint16_t ui16AdcCodes = 0;
	uint8_t ui8AdcUpperCodes = 0;			// Data register read MSB
	uint8_t ui8AdcLowerCodes = 0;			// Data register read LSB
	uint8_t DataRecive[3]={ui8address,ui8AdcUpperCodes,ui8AdcLowerCodes};
	uint8_t DataSent[3]={ui8address,0x00,0x00};
	HAL_StatusTypeDef ret4;
	HAL_GPIO_WritePin(GPIOB, CS_ADC_Pin,GPIO_PIN_RESET);

	ret4  = HAL_SPI_Receive(&hspi2, DataRecive, 3, 1000);


	HAL_GPIO_WritePin(GPIOB, CS_ADC_Pin,GPIO_PIN_SET);
	return ui16AdcCodes;
}



void writeModeReg(uint8_t AD7794_WRITE_MODE_REG,uint8_t modeReg1,uint8_t modeReg2)
{
	uint8_t Data[3] = {0,0,0};
	Data[0] = AD7794_WRITE_MODE_REG;
	if(Data==0)
	{
		Data[0] = AD7794_WRITE_MODE_REG =  0x08;

	}
	HAL_GPIO_WritePin(GPIOB, CS_ADC_Pin,GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2, Data, 3, 1000);
	HAL_GPIO_WritePin(GPIOB, CS_ADC_Pin,GPIO_PIN_SET);

	uint8_t Data1[3] = {AD7794_WRITE_MODE_REG, modeReg1,modeReg2};

	HAL_GPIO_WritePin(GPIOB, CS_ADC_Pin,GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2, Data1, 3, 1000);
	HAL_GPIO_WritePin(GPIOB, CS_ADC_Pin,GPIO_PIN_SET);

}

float readADC7794(uint8_t ch,uint8_t AD7794_WRITE_MODE_REG)
{
		uint32_t adcRaw;
		uint16_t modeReg = 0;
		HAL_GPIO_WritePin(GPIOB, CS_ADC_Pin,GPIO_PIN_RESET);
		HAL_SPI_TransmitReceive(&hspi2, AD7794_WRITE_MODE_REG, modeReg, 3, 1000);
		HAL_GPIO_WritePin(GPIOB, CS_ADC_Pin,GPIO_PIN_SET);
}

unsigned long AD7799_GetRegisterValue(unsigned char regAddress, unsigned char size)
{
	HAL_StatusTypeDef GetR;
    #define AD7799_COMM_ADDR(x)	(((x) & 0x7) << 3)	/* Register Address */
    #define AD7799_COMM_READ    (1 << 6) 			/* Read Operation */
	unsigned char data[5] = {0x03, 0x00, 0x00, 0x00, 0x00};
	unsigned char dataRe[5] = {0x00, 0x00, 0x00, 0x00, 0x00};
	unsigned long receivedData = 0x00;
	data[1] = AD7799_COMM_READ | (((regAddress) & 0x7) << 3);
	HAL_GPIO_WritePin(GPIOB, CS_ADC_Pin,GPIO_PIN_RESET);

	GetR = HAL_SPI_TransmitReceive(&hspi2, data, dataRe, size, 1000);
	/*GetR = HAL_SPI_Transmit(&hspi2, data, 1, 10);
	GetR = HAL_SPI_Receive(&hspi2, data, size, 10);*/

	HAL_GPIO_WritePin(GPIOB, CS_ADC_Pin,GPIO_PIN_SET);
	if(size == 1)
	{
		receivedData += (dataRe[0] << 0);
	}
	if(size == 2)
	{
		receivedData += (dataRe[0] << 8);
		receivedData += (dataRe[1] << 0);
	}
	if(size == 3)
	{
		receivedData += (dataRe[0] << 16);
		receivedData += (dataRe[1] << 8);
		receivedData += (dataRe[2] << 0);
	}
    return receivedData;
}


void AD7799_Reset(void)
{
	HAL_StatusTypeDef ResT;
	unsigned char dataToSend[5] = {0x03, 0xff, 0xff, 0xff, 0xff};
	HAL_GPIO_WritePin(GPIOB, CS_ADC_Pin,GPIO_PIN_RESET);
	ResT = HAL_SPI_Transmit(&hspi2, dataToSend, 4, 10);
	HAL_GPIO_WritePin(GPIOB, CS_ADC_Pin,GPIO_PIN_SET);
}

void AD7799_SetRegisterValue(unsigned char regAddress,
                             unsigned long regValue,
                             unsigned char size)
{
    #define AD7799_COMM_WRITE	(0 << 6) 			/* Write Operation */
	unsigned char data[5] = {0x03, 0x00, 0x00, 0x00, 0x00};
	data[1] = AD7799_COMM_WRITE |  (((regAddress) & 0x7) << 3);
    if(size == 1)
    {
        data[2] = (unsigned char)regValue;
    }
    if(size == 2)
    {
		data[3] = (unsigned char)((regValue & 0x0000FF) >> 0);
        data[2] = (unsigned char)((regValue & 0x00FF00) >> 8);
    }
    if(size == 3)
    {
		data[4] = (unsigned char)((regValue & 0x0000FF) >> 0);
		data[3] = (unsigned char)((regValue & 0x00FF00) >> 8);
        data[2] = (unsigned char)((regValue & 0xFF0000) >> 16);
    }
    HAL_StatusTypeDef SetH;
	HAL_GPIO_WritePin(GPIOB, CS_ADC_Pin,GPIO_PIN_RESET);
	SetH = HAL_SPI_Transmit(&hspi2, data,(1+ size), 10);
	HAL_GPIO_WritePin(GPIOB, CS_ADC_Pin,GPIO_PIN_SET);

}

unsigned char AD7799_Ready(void)
{
    #define AD7799_REG_STAT	    0 /* Status Register	    (RO, 8-bit) */
    unsigned char rdy = 0;
    rdy = (AD7799_GetRegisterValue( AD7799_REG_STAT,1) & 0x80);

	return(!rdy);
}

void AD7799_SetMode(unsigned long mode)
{

	#define AD7799_REG_MODE	    1 /* Mode Register	     	(RW, 16-bit */
    unsigned long command;
    command = AD7799_GetRegisterValue(AD7799_REG_MODE,2);
    command &= ~(((0xFF) & 0x7) << 13);
    command |= (((mode) & 0x7) << 13);
    AD7799_SetRegisterValue(
            AD7799_REG_MODE,
            command,
            2
    );
}

void AD7799_SetChannel(unsigned long channel)
{
	#define AD7799_REG_CONF	    2 /* Configuration Register (RW, 16-bit)*/
    unsigned long command;
    command = AD7799_GetRegisterValue(AD7799_REG_CONF,2);
    command &= ~((0xFF) & 0x7);
    command |= ((channel) & 0x7);
    AD7799_SetRegisterValue(
            AD7799_REG_CONF,
            command,
            2
    );
}

void AD7799_SetGain(unsigned long gain)
{
	#define AD7799_REG_CONF	    2 /* Configuration Register (RW, 16-bit)*/
    unsigned long command;
    command = AD7799_GetRegisterValue(AD7799_REG_CONF,2);
    command &= ~(((0xFF) & 0x7) << 8);
    command |= (((gain) & 0x7) << 8);
    AD7799_SetRegisterValue(
            AD7799_REG_CONF,
            command,
            2
    );
}

void AD7799_SetReference(unsigned char state)
{
	#define AD7799_REG_CONF	    2 /* Configuration Register (RW, 16-bit)*/
    unsigned long command = 0;
    command = AD7799_GetRegisterValue(AD7799_REG_CONF,2);
    command &= ~(((1) & 0x1) << 5);
    command |= (((state) & 0x1) << 5);
    AD7799_SetRegisterValue(AD7799_REG_CONF,
							command,
							2);
}
unsigned char AD7799_Init(void)
{
	#define AD7799_ID			0x9
	#define AD7799_ID_MASK		0xF
	#define AD7799_REG_ID	    4 /* ID Register	     	(RO, 8-bit) */
	unsigned char status = 0x1;
	if((AD7799_GetRegisterValue(AD7799_REG_ID, 1) & 0x0F) != AD7799_ID)
	{
		status = 0x0;
	}

	return(status);
}


enum AD7794_OperatingModes {
    AD7794_OpMode_Continuous = 0,           // Continuous conversion mode (default). Only 1 channel
    AD7794_OpMode_SingleConv,               // Single conversion mode.

};



void setActiveCh(uint8_t ch)
{
    uint8_t currentCh;
  if(ch < 6){

    currentCh = ch;
    uint8_t AD7794_WRITE_MODE_REG = 0x08;
    uint16_t confReg;
    confReg =  0x0010 ;

    HAL_GPIO_WritePin(GPIOB, CS_ADC_Pin,GPIO_PIN_RESET);

   //  SPI.transfer(0x10);

     // SPI.transfer16(confReg);

     HAL_GPIO_WritePin(GPIOB, CS_ADC_Pin,GPIO_PIN_SET);
  }
}
uint32_t getConvResult()
{
  uint8_t inByte;
  uint32_t result = 0;

  //SPI.beginTransaction(spiSettings); //We should still be in our transaction
  uint8_t AD7794_READ_DATA_REG = 0x58;
HAL_StatusTypeDef ret11;
  HAL_GPIO_WritePin(GPIOB, CS_ADC_Pin,GPIO_PIN_RESET);
  uint8_t Data[3] = {0xFF,0,0};
  uint32_t DataRec[3] = {0,0,0};
  ret11 =  HAL_SPI_TransmitReceive(&hspi2, Data, DataRec, 4, 200);
  HAL_GPIO_WritePin(GPIOB, CS_ADC_Pin,GPIO_PIN_SET);
   result = DataRec [0];
   result |= (DataRec [1] & 0xFF0000 );
   result |= (DataRec [2] & 0x00FF00 );


  return result;
}

getReadingRaw(ch)
{
	uint32_t adcRaw = getConvResult();
}
// Class AD7794
uint8_t SPI2_Read_OneByte(uint8_t reg)
{
		uint8_t i;
		uint8_t SendReg[4]={reg,0xFF,0x00,0x00};
		uint8_t RecVReg[4]={0x00,0x00,0x00,0x00};
		HAL_GPIO_WritePin(GPIOB, CS_ADC_Pin,GPIO_PIN_RESET);
		HAL_SPI_Receive(&hspi2, RecVReg, 4, 100);
	    // Register und Data zusammen ?
		HAL_SPI_TransmitReceive(&hspi2, SendReg , RecVReg, 4, 100);


		HAL_GPIO_WritePin(GPIOB, CS_ADC_Pin,GPIO_PIN_SET);
		i = RecVReg[2];
		return i;

}

void SPI2_Read_buf(uint8_t reg, uint8_t *pbuf, uint8_t len)
{
	uint8_t i;
	HAL_GPIO_WritePin(GPIOB, CS_ADC_Pin,GPIO_PIN_RESET);
	uint8_t RecvReg[2]= {reg, 0x00 };
	HAL_SPI_Receive(&hspi2, RecvReg, 1, 100);
	for(i = 0; i < len; i++)
	{
		// Register und Data zusammen ?
	uint8_t RecvReg1[2]= {reg, 0xFF};
	HAL_SPI_TransmitReceive(&hspi2, RecvReg1 , pbuf[i], 1, 100);
	}

	HAL_GPIO_WritePin(GPIOB, CS_ADC_Pin,GPIO_PIN_SET);

}


void SPI2_Write_buf(uint8_t reg, uint8_t *pbuf, uint8_t len)
{
	uint8_t status , i;

	// SPI_CS_Enable();
	 HAL_GPIO_WritePin(GPIOB, CS_ADC_Pin,GPIO_PIN_RESET);
	 HAL_StatusTypeDef Write;
uint8_t Send[4] = {reg, 0xFF, 0xFF,0xFF};
uint8_t Recv[4] = {0x00,0x00, 0x00,0x00};

	Write = HAL_SPI_Transmit(&hspi2, Send,4, 100);

	 uint8_t Send1[4] = {reg, 0xFF, 0xFF,0xFF};
	 uint8_t Recv1[4] = {0x00,0x00, 0x00,0x00};

	// for (int i=0 ; i< len; i++)
		{
		 // HAL_SPI_Transmit(&hspi2, *pbuf++, 1, 100);
		 // Write = HAL_SPI_Transmit(&hspi2, pbuf, len, 100);
		 Write = HAL_SPI_Transmit(&hspi2, Send1,4, 100);
		 // Write = HAL_SPI_Receive(&hspi2, Recv1, 4, 100);
		 Write = HAL_SPI_TransmitReceive(&hspi2, Send1, Recv1, 4, 100);

		}

	 HAL_GPIO_WritePin(GPIOB, CS_ADC_Pin,GPIO_PIN_SET);
}

void AD7794_Reset(void)
{
	uint8_t xtemp[3] = {0xFF,0xFF,0xFF};
	SPI2_Write_buf(0xFF, xtemp, 3);
}

uint8_t xDevice_ID;
void AD7794_Init(void)
{

	uint8_t ModeRegisterMsg[2] = {0x00,0x07};

	uint8_t ConfigRegisterMsg[2] = {0x00,0x91};  // 00 ch1 - 01 ch2 - 10 ch3 - 11 ch4

	uint8_t IoRegisterMsg[1] = {0x00};

	AD7794_Reset();

	HAL_Delay(1);

	xDevice_ID = SPI2_Read_OneByte(0x60);

	//if((xDevice_ID & 0x0F)==0x0B)
	{
		HAL_Delay(1);
		 HAL_StatusTypeDef Write1;
		 //SPI2_Write_buf(0x08, ModeRegisterMsg, 2);   // Mode register
		 HAL_GPIO_WritePin(GPIOB, CS_ADC_Pin,GPIO_PIN_RESET);
		 uint8_t Send2[5] = {0x00,0x50,0x00,0x00,0x00};
	     uint8_t Recv2[5] = {0x00,0x00,0x00,0x00,0x00};
		 Write1 = HAL_SPI_Transmit(&hspi2, Send2,5, 100);
		 Write1 = HAL_SPI_TransmitReceive(&hspi2, Send2, Recv2, 5, 100);
		 HAL_GPIO_WritePin(GPIOB, CS_ADC_Pin,GPIO_PIN_SET);

		HAL_Delay(1);
		// SPI2_Write_buf(0x08, ConfigRegisterMsg, 2);   // Config register
		 HAL_GPIO_WritePin(GPIOB, CS_ADC_Pin,GPIO_PIN_RESET);
		 uint8_t Send3[5] = {0x00, 0x11,0x00,0x00,0x00};
	     uint8_t Recv3[5] = {0x00,0x00,0x00,0x00,0x00};
		 Write1 = HAL_SPI_Transmit(&hspi2, Send3,5, 100);
		 Write1 = HAL_SPI_TransmitReceive(&hspi2, Send3, Recv3, 5, 100);
		 HAL_GPIO_WritePin(GPIOB, CS_ADC_Pin,GPIO_PIN_SET);

		HAL_Delay(1);

		// SPI2_Write_buf(0x28, IoRegisterMsg, 1);   // IO register
		 HAL_GPIO_WritePin(GPIOB, CS_ADC_Pin,GPIO_PIN_RESET);
		 uint8_t Send4[4] = {0x00, 0x00,0x00,0x00};
	     uint8_t Recv4[4] = {0x00,0x00,0x00,0x00};
		 Write1 = HAL_SPI_Transmit(&hspi2, Send4,4, 100);
		 Write1 = HAL_SPI_TransmitReceive(&hspi2, Send4, Recv4, 4, 100);
		 HAL_GPIO_WritePin(GPIOB, CS_ADC_Pin,GPIO_PIN_SET);

		HAL_Delay(1);
	}
}


int32_t Read_AD7794_Data(void)
{
	uint8_t xStatus = 1;
	uint8_t xRDY = 0x80;
	uint8_t xtemp[3];
	int32_t adValue =0;

/*	while(1)
	{
		xStatus = SPI2_Read_OneByte(0x40);
		xRDY = xStatus60x80;
		HAL_Delay(1);
	}*/

//SPI2_Read_buf(0x58, xtemp,3);
	HAL_GPIO_WritePin(GPIOB, CS_ADC_Pin,GPIO_PIN_RESET);
	uint8_t RecvDataReg[4]= {0x3,0x00,0x00,0x00};
	HAL_SPI_Receive(&hspi2, RecvDataReg, 4, 100);

	xtemp[0] = RecvDataReg[1];
	xtemp[1] = RecvDataReg[2];
	xtemp[2] = RecvDataReg[3];

    adValue = ((int32_t)xtemp[0]<<16) | (((int32_t)xtemp[1]) <<8)|(((int32_t)xtemp[2]));

return adValue;

}
uint32_t AD7794_adValue;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM17_Init();
  MX_TIM24_Init();
  MX_TIM16_Init();
  MX_SPI1_Init();
  MX_I2C1_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */

// Wägezellen mit SPI AD7795

    #define AD7794_CHANNEL_COUNT           8

	//Communications register settings
	#define AD7794_WRITE_MODE_REG       0x08    //selects mode reg for writing
	#define AD7794_WRITE_CONF_REG       0x10    //selects conf reg for writing
	#define AD7794_READ_DATA_REG        0x58    //selects data reg for reading
	#define AD7794_READ_STATUS_REG      0x40    //selects status register for reading //Added 11-14-2021

	#define AD7794_DEFAULT_MODE_REG   0x2001    //Single conversion mode, Fadc = 470Hz
	#define AD7794_DEFAULT_CONF_REG   0x0010    //CH 0 - Bipolar, Gain = 1, Input buffer enabled
	#define AD7794_CHOP_DISABLE       0x0210    //Chop disable bits in mode register

	#define AD7794_ADC_MAX_UP     16777216
	#define AD7794_ADC_MAX_BP     8388608

	#define AD7794_INTERNAL_REF_V  1.17
	#define AD7794_REF_EXT_1          0
	#define AD7794_REF_EXT_2          1
	#define AD7794_REF_INT            2
    uint8_t ch;
    uint8_t currentCh;
	uint8_t inByte;
	uint32_t result = 0;

	// Config Reg// ___________________________________

/*AD7794_Init();
while(1)
{
	AD7794_adValue = Read_AD7794_Data();
	HAL_Delay(200);

}*/


		HAL_GPIO_WritePin(csSM40_GPIO_Port, csSM40_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(V24Vor_GPIO_Port, V24Vor_Pin, GPIO_PIN_SET);

		HAL_Delay(200);

		HAL_GPIO_WritePin(V24ON_GPIO_Port, V24ON_Pin, GPIO_PIN_SET);
		HAL_Delay(200);

		HAL_GPIO_WritePin(GPIOD, Weigh_Cell_On_Pin, GPIO_PIN_SET);
		HAL_Delay(1000);

		HAL_GPIO_WritePin(GPIOD, Weigh_Cell_On_Pin, GPIO_PIN_RESET);
		HAL_Delay(1000);
		GPIO_PinState Flag;
		Flag = HAL_GPIO_ReadPin(U_Weigh_Cell_Flag_GPIO_Port, U_Weigh_Cell_Flag_Pin);
		HAL_Delay(200);



	        HAL_SPI_StateTypeDef ret5;

		    //RESET AD7795
		    /*uint8_t DATASEN[4]={ 0xFF, 0xFF, 0xFF, 0xFF};
		    uint8_t DATARECV[4]={ 0, 0, 0, 0};
			HAL_GPIO_WritePin(GPIOB, CS_ADC_Pin,GPIO_PIN_RESET);
			//ret5 = HAL_SPI_Transmit(&hspi2, DATASEN, 4, 200);
			ret5 = HAL_SPI_TransmitReceive(&hspi2, DATASEN, DATARECV, 1, 100);
			HAL_GPIO_WritePin(GPIOB, CS_ADC_Pin,GPIO_PIN_SET);
			HAL_Delay(50);*/





			// Communication Register write to Mode Register 0x8
		    uint8_t DATASEN6[1]={0x8};
		    //uint8_t DATAREC6[2]={ 0x00, 0x00};
			HAL_GPIO_WritePin(GPIOB, CS_ADC_Pin,GPIO_PIN_RESET);
			ret5 = HAL_SPI_Transmit(&hspi2, DATASEN6, 1, 1000);
			HAL_GPIO_WritePin(GPIOB, CS_ADC_Pin,GPIO_PIN_SET);
			HAL_Delay(50);

			// Mode Register      // fADC = 61.7 Hz  120 ms settle
			// 1010 0000 0000 1010
			// 0010 0000 0101 0001
			uint8_t DATASEN2[2]={ 0x20, 0x0A};
			HAL_GPIO_WritePin(GPIOB, CS_ADC_Pin,GPIO_PIN_RESET);
			ret5 = HAL_SPI_Transmit(&hspi2, DATASEN2, 2, 1000);
			HAL_GPIO_WritePin(GPIOB, CS_ADC_Pin,GPIO_PIN_SET);
			HAL_Delay(50);

while(1)
{

	// Communication Register to read Data Register
				uint8_t DATASEN8[1]={ 0x58};
				HAL_GPIO_WritePin(GPIOB, CS_ADC_Pin,GPIO_PIN_RESET);
				ret5 = HAL_SPI_Transmit(&hspi2, DATASEN8, 1, 1000);
				HAL_GPIO_WritePin(GPIOB, CS_ADC_Pin,GPIO_PIN_SET);
				HAL_Delay(500);

				 //read out AD7795   Data Register
				//uint8_t DATASEN10[3]={ 0x58, 0x00, 0x00};
				uint8_t DATRECV10[3]={0,0,0};
				HAL_GPIO_WritePin(GPIOB, CS_ADC_Pin,GPIO_PIN_RESET);
				//ret5 = HAL_SPI_TransmitReceive(&hspi2, DATASEN3, DATRECV3, 3, 100);
				ret5 = HAL_SPI_Receive(&hspi2, DATRECV10, 2,1000);
				HAL_GPIO_WritePin(GPIOB, CS_ADC_Pin,GPIO_PIN_SET);
				//uint8_t Wert = DATRECV10[2] + DATRECV10[1] + DATRECV10[0];
				HAL_Delay(500);

}


			// Communication Register to write Configuration Regi
			uint8_t DATASEN4[1]={0x10};
			uint8_t DATAREC4[2];
			HAL_GPIO_WritePin(GPIOB, CS_ADC_Pin,GPIO_PIN_RESET);
			ret5 = HAL_SPI_Transmit(&hspi2, DATASEN4, 1, 1000);
			HAL_GPIO_WritePin(GPIOB, CS_ADC_Pin,GPIO_PIN_SET);
			HAL_Delay(50);

			// Configuration Register
			// 0000 0000 0001 0000
			// 0000 0111 1100 0000
			uint8_t DATASEN1[2]={ 0x07, 0xC0};   // Ch4 = {0x00, 0x13 } // CH1 = {0x00, 0x10} // CH2 = {0x00, 0x11} // CH3 = {0x00, 0x12}
			uint8_t DATARE1[2]={ 0x00, 0x00};
			HAL_GPIO_WritePin(GPIOB, CS_ADC_Pin,GPIO_PIN_RESET);
			// ret5 = HAL_SPI_Transmit(&hspi2, DATASEN1, 3, 200);
			ret5 = HAL_SPI_Transmit(&hspi2, DATASEN1, 2, 1000);
			HAL_GPIO_WritePin(GPIOB, CS_ADC_Pin,GPIO_PIN_SET);
			HAL_Delay(50);


			// Communication Register to write Mode Register
			uint8_t DATASEN5[1]={ 0x8};
			uint8_t DATAREC5[2]={ 0x00, 0x00};
			HAL_GPIO_WritePin(GPIOB, CS_ADC_Pin,GPIO_PIN_RESET);
			ret5 = HAL_SPI_Transmit(&hspi2, DATASEN5, 1, 1000);
			HAL_GPIO_WritePin(GPIOB, CS_ADC_Pin,GPIO_PIN_SET);
			HAL_Delay(50);


			// Mode Register     zero scale calibration
			uint8_t DATASEN3[2]={ 0x80, 0x0A};
			HAL_GPIO_WritePin(GPIOB, CS_ADC_Pin,GPIO_PIN_RESET);
			ret5 = HAL_SPI_Transmit(&hspi2, DATASEN3, 2, 1000);
			HAL_GPIO_WritePin(GPIOB, CS_ADC_Pin,GPIO_PIN_SET);
			HAL_Delay(50);

			int d = 1;
			while(d=0){
				 HAL_StatusTypeDef ret5;

			// Communication Register to write Mode Register
			uint8_t DATASEN5[1]={ 0x8};
			uint8_t DATAREC5[2]={ 0x00, 0x00};
			HAL_GPIO_WritePin(GPIOB, CS_ADC_Pin,GPIO_PIN_RESET);
			ret5 = HAL_SPI_Transmit(&hspi2, DATASEN5, 1, 1000);
			HAL_GPIO_WritePin(GPIOB, CS_ADC_Pin,GPIO_PIN_SET);
			HAL_Delay(500);


			// Mode Register     zero scale calibration
			uint8_t DATASEN3[2]={ 0x00, 0x0A};
			HAL_GPIO_WritePin(GPIOB, CS_ADC_Pin,GPIO_PIN_RESET);
			ret5 = HAL_SPI_Transmit(&hspi2, DATASEN3, 2, 1000);
			HAL_GPIO_WritePin(GPIOB, CS_ADC_Pin,GPIO_PIN_SET);
			HAL_Delay(500);


			// Communication Register to read Data Register
			uint8_t DATASEN8[1]={ 0x58};
			HAL_GPIO_WritePin(GPIOB, CS_ADC_Pin,GPIO_PIN_RESET);
			ret5 = HAL_SPI_Transmit(&hspi2, DATASEN8, 1, 1000);
			HAL_GPIO_WritePin(GPIOB, CS_ADC_Pin,GPIO_PIN_SET);
			HAL_Delay(500);

			 //read out AD7795   Data Register
			uint8_t DATASEN10[3]={ 0x58, 0x00, 0x00};
			uint8_t DATRECV10[3]={0,0,0};
			HAL_GPIO_WritePin(GPIOB, CS_ADC_Pin,GPIO_PIN_RESET);
			//ret5 = HAL_SPI_TransmitReceive(&hspi2, DATASEN3, DATRECV3, 3, 100);
			ret5 = HAL_SPI_Receive(&hspi2, DATRECV10, 2,1000);
			HAL_GPIO_WritePin(GPIOB, CS_ADC_Pin,GPIO_PIN_SET);
			uint8_t Wert = DATRECV10[2] + DATRECV10[1] + DATRECV10[0];
			HAL_Delay(500);


			//__________________________________________________________________________________________________
			// Communication Register to read Status
			uint8_t DATASEN9[1]={ 0x40};
			HAL_GPIO_WritePin(GPIOB, CS_ADC_Pin,GPIO_PIN_RESET);
			ret5 = HAL_SPI_Transmit(&hspi2, DATASEN9, 1, 1000);
			HAL_GPIO_WritePin(GPIOB, CS_ADC_Pin,GPIO_PIN_SET);
			HAL_Delay(500);

			// Read Status
			uint8_t DATASEN40[1]={0};
			uint8_t DATRECV40[1];
			HAL_GPIO_WritePin(GPIOB, CS_ADC_Pin,GPIO_PIN_RESET);
			ret5 = HAL_SPI_Receive(&hspi2, DATRECV40, 1, 1000);
			HAL_GPIO_WritePin(GPIOB, CS_ADC_Pin,GPIO_PIN_SET);
			HAL_Delay(500);

			}

// :::::::::::::::::::::::::::::::ENDE::::::::::::::::::::::::::::::::::::::::::::::::://
			//::::::::::::::::::::AHMAD::::::::::::::::::::::::::::::::::://


/*
  // Weigh Cell hx711:
  HAL_GPIO_WritePin(csSM40_GPIO_Port, csSM40_Pin,GPIO_PIN_SET);
  HAL_GPIO_WritePin(V24Vor_GPIO_Port, V24Vor_Pin, GPIO_PIN_SET);

  HAL_Delay(200);

  HAL_GPIO_WritePin(V24ON_GPIO_Port, V24ON_Pin, GPIO_PIN_SET);
  HAL_Delay(200);

  hx711_t loadcell;
  float weight;
  // HX_SCK1 = CLK PIN
  // DATA_PIN = HX_DOUT1
  hx711_init(&loadcell,GPIOD, HX_SCK4_Pin, GPIOE, HX_DOUT4_Pin);
  hx711_coef_set(&loadcell, 354.5); // read afer calibration
  hx711_tare(&loadcell, 10);
  // HX_DOUT1_GPIO_Port
  for(int i=0;i<50; i++)
  {
  HAL_Delay(500);
  weight = hx711_weight(&loadcell, 10);
  }
*/


  double m_dSollSpeedDriveHZ= 700;
  double m_dCycle=1;
  double m_dRampTimeMS =100;
  double m_dCurrentlySpeedDriveHz=0;
  int m_bStart = 0, m_bReverseRun = 0;
  HAL_GPIO_WritePin(csSM40_GPIO_Port, csSM40_Pin,GPIO_PIN_SET);
  HAL_GPIO_WritePin(V24Vor_GPIO_Port, V24Vor_Pin, GPIO_PIN_SET);

  HAL_Delay(200);

  HAL_GPIO_WritePin(V24ON_GPIO_Port, V24ON_Pin, GPIO_PIN_SET);
  HAL_Delay(200);

// start Stepper direct!
  while(1)
  {
      EventCreateC();
  }



  // Eingänge und Temp. Sensor

//when A2=1 A1=1 A0=1 =>0100 0000  0 write operation = 0x40 or 0100 0001  read operation = 0x41
//CAT9555 regsiter
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
EventCreateC();


mem_read_reg(CFG_PORT0);
mem_read_reg(CFG_PORT1);

while(1)
{
	  HAL_Delay(20);
	   t++;
	   if(t>49)// Invert the level of the P0 port for about 1 second(20*50=1000ms)
	   {
	      t=0;
	      p0_val =~p0_val;
	     // Write_reg(OUT_PORT0, p0_val);
	   }
	      uint16_t res = mem_read_reg(IN_PORT1);     // Read_reg(IN_PORT1);

 		  if(/* X6 : */(res == 0xffef)||(res == 0xffbf)||(res == 0xffdf)||(res == 0xff7f)||(res == 0xfffe)||(res == 0xfffb||(res == 0xfffd))||(res == 0xfff7)
 				/* X5 : */||(res == 0xf7ff)||(res == 0xfeff)||(res == 0xfbff)||(res == 0xfdff)||(res == 0x7fff)||(res == 0xefff)||(res == 0xbfff)||(res == 0xdfff))
 		   {
 			   EventCreateC();
 		   }else
 		   {
 			   HAL_TIM_PWM_Stop(&htim17, TIM_CHANNEL_1);
 		   }

}


  static const uint8_t REG_TEMP = 0xc1;
  uint8_t TMP102_ADDR = 0x20;
  uint8_t buf[12];

  HAL_StatusTypeDef ret1;

  buf[0] = REG_TEMP;
  ret1 = HAL_I2C_Master_Transmit(&hi2c1, TMP102_ADDR, buf, 1, 1000);
  HAL_Delay(100);
  if ( ret1 != HAL_OK ) {
      strcpy((char*)buf, "Error Tx\r\n");
  } else {

      ret1 = HAL_I2C_Master_Receive(&hi2c1, TMP102_ADDR, buf, 2, 1000);
      if ( ret1 != HAL_OK ) {
      strcpy((char*)buf, "Error Rx\r\n");
      }
  }








 /* TIM24->CCR2 = 65535/2;
  HAL_TIMEx_PWMN_Start(&htim24, TIM_CHANNEL_2);*/

// Motoren TMC5160
  TIM16->CCR1 = 65535/2;
  HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);



  TIM24->CCR1 = 65535/2;
  HAL_TIM_PWM_Start(&htim24, TIM_CHANNEL_1);

  TIM24->CCR2 = 65535/2;
  HAL_TIM_PWM_Start(&htim24, TIM_CHANNEL_2);

    uint8_t pDataSend10[5]={0xEC, 0, 0x1, 0, 0xC3};
  	uint8_t pDataRecv10[5]={0, 0, 0, 0, 0};
  	HAL_GPIO_WritePin(csSM40_2_GPIO_Port, csSM40_2_Pin,GPIO_PIN_RESET);
  	HAL_SPI_TransmitReceive(&hspi1, pDataSend10, pDataRecv10, 5, 1000);
  	HAL_GPIO_WritePin(csSM40_2_GPIO_Port, csSM40_2_Pin,GPIO_PIN_SET);



while(1)
{
   for(int i=1; i<330;i+=60)
	{
		//InitializeInterface(ReadHardwareTimer, ReadTimerOverflow);
	   TIM17->CCR1 = 65535/2;
	   HAL_TIM_PWM_Start(&htim17, TIM_CHANNEL_1);
		/*m_dCurrentlySpeedDriveHz = 10+round(calcFrequency(i, m_dRampTimeMS,m_dSollSpeedDriveHZ));
		timerOverflow = 60 + m_dCurrentlySpeedDriveHz;
		__HAL_TIM_SET_AUTORELOAD (&htim17, 60 + m_dCurrentlySpeedDriveHz); // 31khz
		__HAL_TIM_SET_PRESCALER (&htim17,5);
		__HAL_TIM_SET_COMPARE (&htim17,TIM_CHANNEL_1,50);
		HAL_TIM_PWM_Start(&htim17, TIM_CHANNEL_1);*/
		TmcStart();
		HAL_Delay(80);
  	}
}

  HAL_GPIO_WritePin(GPIOA, OUT10_Pin,GPIO_PIN_SET );
  HAL_GPIO_WritePin(GPIOA, OUT10_Pin,GPIO_PIN_RESET );


	int tt =10000;
	while(tt>0)
	{
		tt--;
		 //Dir 1
	   	  HAL_GPIO_WritePin(GPIOC, DirSM40_Pin,GPIO_PIN_RESET);
		  /* TIM17->CCR1 = 65535/2;
			 HAL_TIM_Base_Start(&htim17);*/
		  // Rampe : PW 50, Prescaler 5, Overload: bei 40 ist Freq=16 und bei 1000 ist Freq 300 Hz

		  // 0xEC000100C3;
			uint8_t pDataSend2[5]={0xEC, 0, 0x1, 0, 0xC3};
			uint8_t pDataRecv2[5]={0, 0, 0, 0, 0};
			uint8_t pDataSend3[5]={0x90, 0, 0x06, 0x1F, 0x0A};
			uint8_t pDataRecv3[5]={0, 0, 0, 0, 0};
			uint32_t uhCapture = 0;
			uint32_t TIM17_pulses = 12;
			uint16_t autoreload_value;
			autoreload_value = __HAL_TIM_GET_AUTORELOAD(&htim17);

		  for(int i=1; i<330;i+=60)
		  {
			  m_dCurrentlySpeedDriveHz = 10+round(calcFrequency(i, m_dRampTimeMS,m_dSollSpeedDriveHZ));
			__HAL_TIM_SET_AUTORELOAD (&htim17, 60 + m_dCurrentlySpeedDriveHz); // 31khz
			__HAL_TIM_SET_PRESCALER (&htim17,5);
			__HAL_TIM_SET_COMPARE (&htim17,TIM_CHANNEL_1,50);
			HAL_TIM_PWM_Start(&htim17, TIM_CHANNEL_1);

			HAL_GPIO_WritePin(csSM40_GPIO_Port, csSM40_Pin,GPIO_PIN_RESET);
			HAL_SPI_TransmitReceive(&hspi1, pDataSend2, pDataRecv2, 5, 1000);
			HAL_GPIO_WritePin(csSM40_GPIO_Port, csSM40_Pin,GPIO_PIN_SET);
			HAL_Delay(80);
		  }


		  HAL_TIM_PWM_Stop(&htim17, TIM_CHANNEL_1);
		  HAL_Delay(3000);
		   /*HAL_GPIO_WritePin(csSM40_GPIO_Port, csSM40_Pin,GPIO_PIN_RESET);
		   HAL_SPI_TransmitReceive(&hspi1, pDataSend3, pDataRecv3, 5, 1000);
		   HAL_GPIO_WritePin(csSM40_GPIO_Port, csSM40_Pin,GPIO_PIN_SET);
		   HAL_Delay(23);*/
	  }









  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = 64;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 12;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x20303E5D;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 0x0;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi1.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 0x0;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  hspi2.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi2.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi2.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi2.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi2.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi2.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi2.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi2.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi2.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 0;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 65535;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim16, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim16, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */
  HAL_TIM_MspPostInit(&htim16);

}

/**
  * @brief TIM17 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM17_Init(void)
{

  /* USER CODE BEGIN TIM17_Init 0 */

  /* USER CODE END TIM17_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 0;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 65535;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim17, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim17, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM17_Init 2 */

  /* USER CODE END TIM17_Init 2 */
  HAL_TIM_MspPostInit(&htim17);

}

/**
  * @brief TIM24 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM24_Init(void)
{

  /* USER CODE BEGIN TIM24_Init 0 */

  /* USER CODE END TIM24_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM24_Init 1 */

  /* USER CODE END TIM24_Init 1 */
  htim24.Instance = TIM24;
  htim24.Init.Prescaler = 0;
  htim24.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim24.Init.Period = 4294967295;
  htim24.Init.ClockDivision = TIM_CLOCKDIVISION_DIV4;
  htim24.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim24) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim24, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim24, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  if (HAL_TIM_PWM_ConfigChannel(&htim24, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM24_Init 2 */

  /* USER CODE END TIM24_Init 2 */
  HAL_TIM_MspPostInit(&htim24);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_ADC_GPIO_Port, CS_ADC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, Weigh_Cell_On_Pin|V24ON_Pin|HX_SCK3_Pin|HX_SCK2_Pin
                          |HX_SCK4_Pin|HX_SCK1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, OUT10_Pin|V24Vor_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(csSM40_GPIO_Port, csSM40_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, DirSM40_2_Pin|DirSM40_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, OUT7_Pin|csSM40_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CS_ADC_Pin */
  GPIO_InitStruct.Pin = CS_ADC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_ADC_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Weigh_Cell_On_Pin V24ON_Pin HX_SCK3_Pin HX_SCK2_Pin
                           HX_SCK4_Pin HX_SCK1_Pin */
  GPIO_InitStruct.Pin = Weigh_Cell_On_Pin|V24ON_Pin|HX_SCK3_Pin|HX_SCK2_Pin
                          |HX_SCK4_Pin|HX_SCK1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : OUT10_Pin V24Vor_Pin */
  GPIO_InitStruct.Pin = OUT10_Pin|V24Vor_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : csSM40_Pin */
  GPIO_InitStruct.Pin = csSM40_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(csSM40_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : U_Weigh_Cell_Flag_Pin */
  GPIO_InitStruct.Pin = U_Weigh_Cell_Flag_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(U_Weigh_Cell_Flag_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DirSM40_2_Pin DirSM40_Pin */
  GPIO_InitStruct.Pin = DirSM40_2_Pin|DirSM40_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : HX_DOUT4_Pin HX_DOUT3_Pin HX_DOUT1_Pin HX_DOUT2_Pin */
  GPIO_InitStruct.Pin = HX_DOUT4_Pin|HX_DOUT3_Pin|HX_DOUT1_Pin|HX_DOUT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : OUT7_Pin csSM40_2_Pin */
  GPIO_InitStruct.Pin = OUT7_Pin|csSM40_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


/* USER CODE END 4 */

/* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  if(htim == &htim17)
      {
          timerOverflow++;
      }

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
