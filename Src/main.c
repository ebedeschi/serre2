/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l4xx_hal.h"
#include "adc.h"
#include "i2c.h"
#include "rtc.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */

#include "version.h"
#include "OneWire/OneWire_C.h"
#include "DsThermometer/DsThermometer_C.h"
#include "SHT2x/SHT2x.h"
#include "Vbat/Vbat.h"
#include "BH1750/BH1750c.h"
#include "crc32/crc32.h"
#include "timeServer.h"
#include "radio.h"
#include "vcom.h"
#include "lora.h"
#include "hw.h"

/*!
 * Defines the application data transmission duty cycle. 5s, value in [ms].
 */
#define APP_TX_DUTYCYCLE                            60000
/*!
 * LoRaWAN Adaptive Data Rate
 * @note Please note that when ADR is enabled the end-device should be static
 */
#define LORAWAN_ADR_ON                              0
/*!
 * LoRaWAN confirmed messages
 */
#define LORAWAN_CONFIRMED_MSG                    ENABLE
/*!
 * LoRaWAN application port
 * @note do not use 224. It is reserved for certification
 */
#define LORAWAN_APP_PORT							1

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

extern RTC_HandleTypeDef RtcHandle;
extern RTC_HandleTypeDef hrtc;

/* call back when LoRa will transmit a frame*/
static void LoraTxData( lora_AppData_t *AppData, FunctionalState* IsTxConfirmed);

/* call back when LoRa has received a frame*/
static void LoraRxData( lora_AppData_t *AppData);

/* load call backs*/
static LoRaMainCallback_t LoRaMainCallbacks ={ HW_GetBatteryLevel,
                                               HW_GetUniqueId,
                                               HW_GetRandomSeed,
                                               LoraTxData,
                                               LoraRxData};

/*!
 * Specifies the state of the application LED
 */
static uint8_t AppLedStateOn = RESET;

/* !
 *Initialises the Lora Parameters
 */
static  LoRaParam_t LoRaParamInit= {TX_ON_TIMER,
                                    APP_TX_DUTYCYCLE,
                                    CLASS_A,
                                    LORAWAN_ADR_ON,
									DR_5,
                                    LORAWAN_PUBLIC_NETWORK };

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

void fai()
{}

TimerEvent_t wakeup;

static void OnWakeup( void )
{
	TimerStop( &wakeup );
	PRINTF("OnWakeup\n");
//	HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
//	HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
//	HAL_Delay(5000); //delay
//	HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
//  TimerInit( &wakeup, OnWakeup );
//	TimerSetValue( &wakeup, 2500 );
//	TimerStart( &wakeup );
}

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART3_UART_Init();
  MX_I2C1_Init();
  MX_RTC_Init();
  MX_SPI1_Init();
  MX_ADC3_Init();

  /* USER CODE BEGIN 2 */

  HAL_GPIO_WritePin(RFPOWER_GPIO_Port, RFPOWER_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(RADIO_NRESET_GPIO_Port, RADIO_NRESET_Pin, GPIO_PIN_SET);
//  HAL_GPIO_WritePin(RADIO_NSS_GPIO_Port, RADIO_NSS_Pin, GPIO_PIN_SET);

  /* Configure the hardware*/
  HW_Init( );

//  HAL_GPIO_WritePin(RADIO_NRESET_GPIO_Port, RADIO_NRESET_Pin, GPIO_PIN_RESET);
//  HAL_GPIO_WritePin(RFPOWER_GPIO_Port, RFPOWER_Pin, GPIO_PIN_SET);

//    lora_Init( &LoRaMainCallbacks, &LoRaParamInit);

//    PRINTF("VERSION: %X\n", VERSION);

//	TimerInit( &wakeup, OnWakeup );
//	TimerSetValue( &wakeup, 2500 );
//	TimerStart( &wakeup );

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  /* run the LoRa class A state machine*/
//	  lora_fsm( );

		HOneWire _wire;
		HOneWire* wire;
		//handle bus OneWire
		_wire = owCreateHandle(OneWireBus_GPIO_Port, OneWireBus_Pin);
		wire = &_wire;
		owBegin(wire);

		//Test termometro
		dsSearchResult sr = dsSearchThermometers(wire);
		HDsThermometer _ds = dsCreateHandle(wire, sr.devices[0]);
		dsDeleteSearchResult(&sr);
		HDsThermometer* ds = &_ds;
		dsBegin(ds);
		float temp = 0;
		dsGetTemperature(ds, &temp);
		dsRequestTemperature(ds);
		dsGetTemperature(ds, &temp);

		BH1750 luxMeter;
		//crea handle luxMeter. Lo collega all'indirizzo basso
		luxMeter = BH1750_CreateHandle(&hi2c1, BH1750_LOW_ADDRESS);
		//inizializza la periferica in CONTINUOUS_HIGH_RES_MODE_1
		BH1750_Begin(luxMeter, CONTINUOUS_HIGH_RES_MODE_1);
		HAL_Delay(10);

		uint16_t vbat = 0;
		vbat = getVbat(hadc3);

		uint16_t sT, sH;
		float   temperatureC, humidityH;           //variable for temperature[°C] as float
		uint8_t  error = 0;              //variable for error code. For codes see system.h
		char ctemp[20];
		error |= SHT2x_MeasureHM(TEMP, &sT);
		temperatureC = SHT2x_CalcTemperatureC(sT);
		int d1 = temperatureC;
		float f2 = ((float)temperatureC) - d1;
		int d2 = trunc(f2 * 100);
		sprintf(ctemp,"%d.%d", d1, d2);
		char chum[20];
		error |= SHT2x_MeasureHM(HUMIDITY, &sH);
		humidityH = SHT2x_CalcRH(sH);
		d1 = humidityH;
		f2 = ((float)humidityH) - d1;
		d2 = trunc(f2 * 100);
		sprintf(chum,"%d.%d", d1, d2);

		int lux = BH1750_ReadLightLevel(luxMeter);

		char b[100]={'\0'};
		uint8_t blen = 0;
		RadioEvents_t re;
		re.TxDone = fai;
		re.RxDone = fai;
		re.RxError = fai;
		re.TxTimeout = fai;
		re.RxTimeout = fai;
		Radio.Init( &re );
		Radio.SetChannel( 868000000 );

		b[0]=100;
		b[1]=1;
		b[2]=100;
		b[3]=0;
		b[4]='\0';
		sprintf(&b[4], "%s:%s:%d:%d", ctemp, chum, lux, vbat);
		blen = strlen(&b[4]) + 4;

		uint32_t crc = crc32(0, &b[4], blen - 4);
		memcpy(&b[blen], &crc, 4);
		blen += 4;

		Radio.SetTxConfig( MODEM_LORA, 5, 0, 0, 7, 1, 8, false, true, 0, 0, false, 3000 );
		Radio.Send( b, blen );
		HAL_Delay(1000);

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

	TimerInit( &wakeup, OnWakeup );
	TimerSetValue( &wakeup, 5000 );
	TimerStart( &wakeup );

	DISABLE_IRQ( );
//    if ( lora_getDeviceState( ) == DEVICE_STATE_SLEEP )
//    {
		  LowPower_Handler( );
//    }
	ENABLE_IRQ();


//	  HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
//	  HAL_Delay(5000);
	  PRINTF("Prova \n");
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_USART3
                              |RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_ADC;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 16;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_ADC1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure LSE Drive Capability
    */
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

    /**Configure the main internal regulator output voltage
    */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the Systick interrupt time
    */
//  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

    /**Enable MSI Auto calibration
    */
  HAL_RCCEx_EnableMSIPLLMode();

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

static void LoraTxData( lora_AppData_t *AppData, FunctionalState* IsTxConfirmed)
{
  uint8_t batteryLevel;

  uint32_t i = 0;

  batteryLevel = HW_GetBatteryLevel( );                     /* 1 (very low) to 254 (fully charged) */

  AppData->Port = LORAWAN_APP_PORT;

  *IsTxConfirmed =  LORAWAN_CONFIRMED_MSG;

	char word[20];
//	error |= SHT2x_MeasureHM(TEMP, &sT);
//	temperatureC = SHT2x_CalcTemperatureC(sT);
//	int d1 = temperatureC;
//	float f2 = temperatureC - d1;
//	int d2 = trunc(f2 * 10000);
//	sprintf(word,"%d.%04d", d1, d2);
//	for(i = 0; i<strlen(word); i++){
//	sprintf(Buffer+i*2, "%02X", word[i]);
//	}
//	PRINTF("%s\n", Buffer);
    sprintf(word,"ciao");
	strcpy(AppData->Buff, word);
	PRINTF("%s\n", AppData->Buff);

//  AppData->Buff[i++] = 'c';
//  AppData->Buff[i++] = 'i';
//  AppData->Buff[i++] = 'a';
//  AppData->Buff[i++] = 'o';
//  AppData->Buff[i++] = ' ';
//  AppData->Buff[i++] = 'm';
//  AppData->Buff[i++] = 'o';
//  AppData->Buff[i++] = 'n';
//  AppData->Buff[i++] = 'd';
//  AppData->Buff[i++] = 'o';

  AppData->BuffSize = strlen(AppData->Buff);
}

static void LoraRxData( lora_AppData_t *AppData )
{
  switch (AppData->Port)
  {
  case LORAWAN_APP_PORT:
	  for(int i=0; i<AppData->BuffSize; i++)
		  PRINTF("%c", (char)(AppData->Buff[i]));
    break;
  default:
    break;
  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
