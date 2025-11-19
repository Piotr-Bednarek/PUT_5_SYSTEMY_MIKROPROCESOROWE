/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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
#include "string.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>
#include <stdbool.h>

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
#if defined ( __ICCARM__ ) /*!< IAR Compiler */
#pragma location=0x2007c000
ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
#pragma location=0x2007c0a0
ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

#elif defined ( __CC_ARM )  /* MDK ARM Compiler */

__attribute__((at(0x2007c000))) ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
__attribute__((at(0x2007c0a0))) ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

#elif defined ( __GNUC__ ) /* GNU Compiler */

ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT] __attribute__((section(".RxDecripSection"))); /* Ethernet Rx DMA Descriptors */
ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT] __attribute__((section(".TxDecripSection")));   /* Ethernet Tx DMA Descriptors */
#endif

ETH_TxPacketConfig TxConfig;

ETH_HandleTypeDef heth;

SPI_HandleTypeDef hspi4;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ETH_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_SPI4_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define ZADANIE 3

void SetPWM(TIM_HandleTypeDef *htim, uint32_t channel, float duty) {
	if (duty < 0.0f)
		duty = 0.0f;
	if (duty > 100.0f)
		duty = 100.0f;

	uint32_t ARR = __HAL_TIM_GET_AUTORELOAD(htim);
	uint32_t compare = (uint32_t) ((duty / 100.0f) * (ARR + 1));
	__HAL_TIM_SET_COMPARE(htim, channel, compare);
}

// wyslanie wiadomosci uart'em
void UART_TransmitMessage(char *msg) {
	HAL_UART_Transmit(&huart3, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
}

// z dokumentacji
# define BMP2_SPI_BUFFER_LEN 28
# define BMP2_DATA_INDEX 1
# define BMP2_REG_ADDR_INDEX 0
# define BMP2_REG_ADDR_LEN 1

#define BMP_CS_PORT GPIOE
#define BMP_CS_PIN  GPIO_PIN_4  // SPI4_NSS - PE4

// aktywacja linii - start komunikacji
void CS_LOW() {
	HAL_GPIO_WritePin(BMP_CS_PORT, BMP_CS_PIN, GPIO_PIN_RESET);
}

// dezaktywacja linii - koniec komunikacji
void CS_HIGH() {
	HAL_GPIO_WritePin(BMP_CS_PORT, BMP_CS_PIN, GPIO_PIN_SET);
}

// zapis do rejestru czujnika
void BMP280_Write(uint8_t reg, uint8_t value) {
	uint8_t tx[2] = { reg & 0x7F, value };
	CS_LOW();
	HAL_SPI_Transmit(&hspi4, tx, 2, HAL_MAX_DELAY);
	CS_HIGH();
}

// odczyt rejestru czujnika
void BMP280_Read(uint8_t reg, uint8_t *data, uint8_t len) {
	uint8_t addr = reg | 0x80;
	CS_LOW();
	HAL_SPI_Transmit(&hspi4, &addr, 1, HAL_MAX_DELAY);
	HAL_SPI_Receive(&hspi4, data, len, HAL_MAX_DELAY);
	CS_HIGH();
}

// XD - kumpel witam, niektore chyba niepotrzebne
#define BMP280_ADDR_CTRL_MEAS  0xF4
#define BMP280_ADDR_CONFIG     0xF5
#define BMP280_ADDR_RESET      0xE0
#define BMP280_ADDR_ID         0xD0

#define BMP280_RESET_VALUE     0xB6
#define BMP280_MODE_NORMAL     0x03
#define BMP280_OSRS_TEMP_2X    (0x02 << 5)

#define BMP280_CTRL_MEAS_INIT  0x27 // oversampling x1 i normal mode

_Bool BMP280_IsDetected() {
	uint8_t id = 0;
	BMP280_Read(BMP280_ADDR_ID, &id, 1);

	// standardowe id czujnika
	if (id == 0x58)
		return true;
	else
		return false;
}

void BMP280_Init() {
	// czytamy id czujnika
	uint8_t id;
	BMP280_Read(BMP280_ADDR_ID, &id, 1);

	// sprawdzamy czy wykryty czujnik
	if (!BMP280_IsDetected()) {
		UART_TransmitMessage("CZUJNIK NIEWYKRYTY\r\n");
		return;
	}

	// zapisujemy reset do adresu reseta czujnika
	// ustawienia domyslne czujnika
	BMP280_Write(BMP280_ADDR_RESET, BMP280_RESET_VALUE);
	HAL_Delay(10);

	// ustawiamy w adresie configu rozne rzeczy (filtry, czas pomiedzy pomiarami) - tutaj wylaczamy wszystko
	BMP280_Write(BMP280_ADDR_CONFIG, 0x00);
	// ustawiamy tryb pracy czujnika,
	BMP280_Write(BMP280_ADDR_CTRL_MEAS, BMP280_CTRL_MEAS_INIT);
}

// bajty tempretury czujnika
// most significant byte
// least significant byte
// extra least significant byte
#define BMP280_ADDR_TEMP_MSB 0xFA
#define BMP280_ADDR_TEMP_LSB 0xFB
#define BMP280_ADDR_TEMP_XLSB 0xFC

// bajty cisnienia czujnika
#define BMP280_ADDR_PRESS_MSB  0xF7
#define BMP280_ADDR_PRESS_LSB  0xF8
#define BMP280_ADDR_PRESS_XLSB 0xF9

// odczyt temperatury...
int32_t BMP280_ReadRawTemperature() {
	uint8_t data[3];
	BMP280_Read(BMP280_ADDR_TEMP_MSB, data, 3);
	int32_t rawTemp = ((int32_t) data[0] << 12) | ((int32_t) data[1] << 4) | ((int32_t) (data[2] >> 4));
	return rawTemp;
}

int32_t BMP280_ReadRawPressure() {
	uint8_t data[3];
	BMP280_Read(BMP280_ADDR_PRESS_MSB, data, 3);
	int32_t rawPress = ((int32_t) data[0] << 12) | ((int32_t) data[1] << 4) | ((int32_t) (data[2] >> 4));
	return rawPress;
}

// trzeba odczytac kalibracje zeby dobrze zamienic raw temp na C
uint16_t dig_T1;
int16_t dig_T2, dig_T3;

// to samo dla cisnienia
uint16_t dig_P1;
int16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;

void BMP280_ReadCalibration() {
	uint8_t calib[24];
	BMP280_Read(0x88, calib, 24);

	dig_T1 = (uint16_t) ((calib[1] << 8) | calib[0]);
	dig_T2 = (int16_t) ((calib[3] << 8) | calib[2]);
	dig_T3 = (int16_t) ((calib[5] << 8) | calib[4]);

	dig_P1 = (uint16_t) ((calib[7] << 8) | calib[6]);
	dig_P2 = (int16_t) ((calib[9] << 8) | calib[8]);
	dig_P3 = (int16_t) ((calib[11] << 8) | calib[10]);
	dig_P4 = (int16_t) ((calib[13] << 8) | calib[12]);
	dig_P5 = (int16_t) ((calib[15] << 8) | calib[14]);
	dig_P6 = (int16_t) ((calib[17] << 8) | calib[16]);
	dig_P7 = (int16_t) ((calib[19] << 8) | calib[18]);
	dig_P8 = (int16_t) ((calib[21] << 8) | calib[20]);
	dig_P9 = (int16_t) ((calib[23] << 8) | calib[22]);
}

//podobno algorytm przeliczenia z dokumentacji czujnika, ale kto by w to wierzyl
int32_t t_fine;

// zwracamy temperature w C
float BMP280_CalcTemperature(int32_t adc_T) {
	int32_t var1, var2;
	var1 = ((((adc_T >> 3) - ((int32_t) dig_T1 << 1))) * ((int32_t) dig_T2)) >> 11;
	var2 = (((((adc_T >> 4) - ((int32_t) dig_T1)) * ((adc_T >> 4) - ((int32_t) dig_T1))) >> 12) * ((int32_t) dig_T3))
			>> 14;
	t_fine = var1 + var2;
	float T = (t_fine * 5 + 128) >> 8;
	return T / 100.0f;
}

int32_t BMP280_CalcTemperature_x100(int32_t adc_T) {
	int32_t var1, var2, T;
	var1 = ((((adc_T >> 3) - ((int32_t) dig_T1 << 1))) * ((int32_t) dig_T2)) >> 11;

	var2 = (((((adc_T >> 4) - ((int32_t) dig_T1)) * ((adc_T >> 4) - ((int32_t) dig_T1))) >> 12) * ((int32_t) dig_T3))
			>> 14;

	t_fine = var1 + var2;
	T = (t_fine * 5 + 128) >> 8;

	return T;
}

int32_t BMP280_CalcPressure(int32_t adc_P) {
	int64_t var1, var2, p;

	var1 = ((int64_t) t_fine) - 128000;
	var2 = var1 * var1 * (int64_t) dig_P6;
	var2 = var2 + ((var1 * (int64_t) dig_P5) << 17);
	var2 = var2 + (((int64_t) dig_P4) << 35);
	var1 = ((var1 * var1 * (int64_t) dig_P3) >> 8) + ((var1 * (int64_t) dig_P2) << 12);
	var1 = (((((int64_t) 1) << 47) + var1)) * ((int64_t) dig_P1) >> 33;

	if (var1 == 0)
		return 0; // avoid exception

	p = 1048576 - adc_P;
	p = (((p << 31) - var2) * 3125) / var1;
	var1 = (((int64_t) dig_P9) * (p >> 13) * (p >> 13)) >> 25;
	var2 = (((int64_t) dig_P8) * p) >> 19;

	p = ((p + var1 + var2) >> 8) + (((int64_t) dig_P7) << 4);

	return (int32_t) p;  // Uwaga: wynik w Q24.8 -> Pa = p/256
}

float tempC;

int32_t rawTemp;

int32_t temp_mC;
int32_t pressPa;


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	// zad 2 - odpal debuggera i odczytaj temp
	// zad 3 - temperatura po uarcie do terminala
	// zad 6 - odkomentuj
	// 1 HZ
	if (htim->Instance == TIM2) {
		// sprawdzamy czy wykryty czujnik
		if (!BMP280_IsDetected()) {
			UART_TransmitMessage("CZUJNIK NIEWYKRYTY\r\n");
			return;
		}


		rawTemp = BMP280_ReadRawTemperature();
		tempC = BMP280_CalcTemperature(rawTemp);

		static uint32_t counter = 0; //zad 6
		float time_s = counter; // zad 6

		char buf[50];
		int integer = (int) tempC;
		int decimal = (int) ((tempC - integer) * 100);

#if ZADANIE == 3
			sprintf(buf, "Temperetura = %d.%02d C\r\n", integer, decimal);
			UART_TransmitMessage(buf);
		#endif

#if ZADANIE == 6
			sprintf(buf, "Temperetura = %d.%02d C\r\n", integer, decimal);
			UART_TransmitMessage(buf);
			sprintf(buf, "%.2f,%.2f\r\n", time_s, tempC);
			UART_TransmitMessage(buf);
		#endif

		counter++;
	}

	// nie trzeba w uarcie, ale dla zasady
	// odpal SWV
	// 4 Hz
	if (htim->Instance == TIM3) {

		// sprawdzamy czy wykryty czujnik
		if (!BMP280_IsDetected()) {
			UART_TransmitMessage("CZUJNIK NIEWYKRYTY\r\n");
			return;
		}

		// raw odczyty
		int32_t rawTemp = BMP280_ReadRawTemperature();
		int32_t rawPress = BMP280_ReadRawPressure();

		// temperatura w setnych stopnia
		temp_mC = BMP280_CalcTemperature_x100(rawTemp) * 10;

		// cisnienie w Pa
		pressPa = BMP280_CalcPressure(rawPress);

		//  temperatury na

		// Formatowanie tekstu bez floatów
		char buf[80];
		sprintf(buf, "Temperatura = %ld mC   Cisnienie = %ld Pa\r\n", temp_mC, pressPa);

		UART_TransmitMessage(buf);
	}
}

uint8_t rxBuffer[4];

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {

	int value = atoi((char*) rxBuffer);

	SetPWM(&htim4, TIM_CHANNEL_1, (float) value);

	HAL_UART_Receive_IT(&huart3, rxBuffer, 4);

//	UART_TransmitMessage("TEST");
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

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
  MX_ETH_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_SPI4_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

#if ZADANIE == 2 || ZADANIE == 3
	HAL_TIM_Base_Start_IT(&htim2);
	BMP280_Init();
	BMP280_ReadCalibration();
#endif

#if ZADANIE == 4
	HAL_TIM_Base_Start_IT(&htim3);
	BMP280_Init();
	BMP280_ReadCalibration();
#endif

#if ZADANIE == 5
	UART_TransmitMessage("czas[s], temperatura[C]\r\n");
	HAL_TIM_Base_Start(&htim4);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    HAL_TIM_Base_Start_IT(&htim2);
    BMP280_Init();
    BMP280_ReadCalibration();
    SetPWM(&htim4, TIM_CHANNEL_1, 0.0);
    HAL_UART_Receive_IT(&huart3, rxBuffer, 4);
//    UART_TransmitMessage("TES2T");
#endif

#if ZADANIE == 6
    UART_TransmitMessage("czas[s], temperatura[C]\r\n");

//    HAL_UART_Transmit(&huart3, "TES424T", strlen("TES424T"), HAL_MAX_DELAY);
    HAL_TIM_Base_Start(&htim4);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_Base_Start_IT(&htim2);
	BMP280_Init();
	BMP280_ReadCalibration();
	SetPWM(&htim4, TIM_CHANNEL_1, 0.0);
	HAL_UART_Receive_IT(&huart3, rxBuffer, 4);
    UART_TransmitMessage("czas[s], temperatura[C]\r\n");
#endif

	// zad 2 i 3
//	HAL_TIM_Base_Start_IT(&htim2);
//	BMP280_Init();
//	BMP280_ReadCalibration();

	// zad 4
//	HAL_TIM_Base_Start_IT(&htim3);
//	BMP280_Init();
//	BMP280_ReadCalibration();

	// zad 5
//	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
//	BMP280_Init();
//	BMP280_ReadCalibration();

	// zad 6, PWM NA 90%, ODPAL "START LOG" W TERMINALU
//	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
//	SetPWM(&htim4, TIM_CHANNEL_1, 90.0);
//	UART_TransmitMessage("czas[s], temperatura[C]\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {

		// delay bo bez tego receive z uarta dziala randomowo
		HAL_Delay(10);
//		HAL_UART_Transmit(&huart3, "TES424T", strlen("TES424T"), HAL_MAX_DELAY);
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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ETH Initialization Function
  * @param None
  * @retval None
  */
static void MX_ETH_Init(void)
{

  /* USER CODE BEGIN ETH_Init 0 */

  /* USER CODE END ETH_Init 0 */

   static uint8_t MACAddr[6];

  /* USER CODE BEGIN ETH_Init 1 */

  /* USER CODE END ETH_Init 1 */
  heth.Instance = ETH;
  MACAddr[0] = 0x00;
  MACAddr[1] = 0x80;
  MACAddr[2] = 0xE1;
  MACAddr[3] = 0x00;
  MACAddr[4] = 0x00;
  MACAddr[5] = 0x00;
  heth.Init.MACAddr = &MACAddr[0];
  heth.Init.MediaInterface = HAL_ETH_RMII_MODE;
  heth.Init.TxDesc = DMATxDscrTab;
  heth.Init.RxDesc = DMARxDscrTab;
  heth.Init.RxBuffLen = 1524;

  /* USER CODE BEGIN MACADDRESS */

  /* USER CODE END MACADDRESS */

  if (HAL_ETH_Init(&heth) != HAL_OK)
  {
    Error_Handler();
  }

  memset(&TxConfig, 0 , sizeof(ETH_TxPacketConfig));
  TxConfig.Attributes = ETH_TX_PACKETS_FEATURES_CSUM | ETH_TX_PACKETS_FEATURES_CRCPAD;
  TxConfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
  TxConfig.CRCPadCtrl = ETH_CRC_PAD_INSERT;
  /* USER CODE BEGIN ETH_Init 2 */

  /* USER CODE END ETH_Init 2 */

}

/**
  * @brief SPI4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI4_Init(void)
{

  /* USER CODE BEGIN SPI4_Init 0 */

  /* USER CODE END SPI4_Init 0 */

  /* USER CODE BEGIN SPI4_Init 1 */

  /* USER CODE END SPI4_Init 1 */
  /* SPI4 parameter configuration*/
  hspi4.Instance = SPI4;
  hspi4.Init.Mode = SPI_MODE_MASTER;
  hspi4.Init.Direction = SPI_DIRECTION_2LINES;
  hspi4.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi4.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi4.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi4.Init.NSS = SPI_NSS_SOFT;
  hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi4.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi4.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi4.Init.CRCPolynomial = 7;
  hspi4.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi4.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI4_Init 2 */

  /* USER CODE END SPI4_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 10799;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 9999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 23999;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 95;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 9999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PE4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
