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
ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT] __attribute__((section(".TxDecripSection"))); /* Ethernet Tx DMA Descriptors */
#endif

ETH_TxPacketConfig TxConfig;

ETH_HandleTypeDef heth;

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
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define BMP2_SPI_BUFFER_LEN 28
#define BMP2_DATA_INDEX 1
#define BMP2_REG_ADDR_INDEX 0
#define BMP2_REG_ADDR_LEN 1

#define BMP2_TIMEOUT 100
#define BMP2_NUM_OF_SENSORS 1

/* wskaznik SPI */
#define BMP2_SPI (&hspi4)

GPIO_TypeDef *BMP2_CS_Ports[BMP2_NUM_OF_SENSORS] = { GPIOE };
uint16_t BMP2_CS_Pins[BMP2_NUM_OF_SENSORS] = { GPIO_PIN_4 };

extern SPI_HandleTypeDef hspi4;
extern UART_HandleTypeDef huart3;

/* ----------------- BMP280 ADRESY REJESTRÓW ----------------- */
#define BMP280_ADDR_ID         0xD0
#define BMP280_ADDR_RESET      0xE0
#define BMP280_ADDR_STATUS     0xF3
#define BMP280_ADDR_CTRL_MEAS  0xF4
#define BMP280_ADDR_CONFIG     0xF5

#define BMP280_RESET_VALUE     0xB6
#define BMP280_ID_VALUE        0x58

/* temperatury */
#define BMP280_ADDR_TEMP_MSB  0xFA
#define BMP280_ADDR_TEMP_LSB  0xFB
#define BMP280_ADDR_TEMP_XLSB 0xFC

/* cisnienie */
#define BMP280_ADDR_PRESS_MSB  0xF7
#define BMP280_ADDR_PRESS_LSB  0xF8
#define BMP280_ADDR_PRESS_XLSB 0xF9

uint16_t dig_T1;
int16_t dig_T2, dig_T3;

uint16_t dig_P1;
int16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;

int32_t t_fine;

int8_t bmp2_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr) {
	HAL_StatusTypeDef status = HAL_OK;
	int8_t iError = 0;
	uint8_t cs = *(uint8_t*) intf_ptr;

	/* CS LOW – wybór slave */
	HAL_GPIO_WritePin(BMP2_CS_Ports[cs], BMP2_CS_Pins[cs], GPIO_PIN_RESET);

	/* zapis adresu rejestru */
	status = HAL_SPI_Transmit(BMP2_SPI, &reg_addr, BMP2_REG_ADDR_LEN, BMP2_TIMEOUT);
	/* zapis danych */
	status |= HAL_SPI_Transmit(BMP2_SPI, (uint8_t*) reg_data, length, BMP2_TIMEOUT);

	/* CS HIGH – koniec transmisji */
	HAL_GPIO_WritePin(BMP2_CS_Ports[cs], BMP2_CS_Pins[cs], GPIO_PIN_SET);

	if (status != HAL_OK)
		iError = -1;

	return iError;
}

int8_t bmp2_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr) {
	HAL_StatusTypeDef status = HAL_OK;
	int8_t iError = 0;
	uint8_t cs = *(uint8_t*) intf_ptr;

	/* adres rejestru OR | 0x80 dla odczytu */
	reg_addr |= 0x80;

	/* CS LOW */
	HAL_GPIO_WritePin(BMP2_CS_Ports[cs], BMP2_CS_Pins[cs], GPIO_PIN_RESET);

	/* wysyłamy adres rejestru */
	status = HAL_SPI_Transmit(BMP2_SPI, &reg_addr, BMP2_REG_ADDR_LEN, BMP2_TIMEOUT);
	/* odbieramy dane */
	status |= HAL_SPI_Receive(BMP2_SPI, reg_data, length, BMP2_TIMEOUT);

	/* CS HIGH */
	HAL_GPIO_WritePin(BMP2_CS_Ports[cs], BMP2_CS_Pins[cs], GPIO_PIN_SET);

	if (status != HAL_OK)
		iError = -1;

	return iError;
}

uint8_t sensor_index = 0;  // jeden sensor

void BMP_Write(uint8_t reg, uint8_t val) {
	bmp2_spi_write(reg, &val, 1, &sensor_index);
}

void BMP_Read(uint8_t reg, uint8_t *buf, uint8_t len) {
	bmp2_spi_read(reg, buf, len, &sensor_index);
}

_Bool BMP280_IsDetected() {
	uint8_t id = 0;
	BMP_Read(BMP280_ADDR_ID, &id, 1);
	return id == BMP280_ID_VALUE;
}

void BMP280_ReadCalibration() {
	uint8_t calib[24];
	BMP_Read(0x88, calib, 24);

	dig_T1 = (uint16_t) (calib[1] << 8 | calib[0]);
	dig_T2 = (int16_t) (calib[3] << 8 | calib[2]);
	dig_T3 = (int16_t) (calib[5] << 8 | calib[4]);

	dig_P1 = (uint16_t) (calib[7] << 8 | calib[6]);
	dig_P2 = (int16_t) (calib[9] << 8 | calib[8]);
	dig_P3 = (int16_t) (calib[11] << 8 | calib[10]);
	dig_P4 = (int16_t) (calib[13] << 8 | calib[12]);
	dig_P5 = (int16_t) (calib[15] << 8 | calib[14]);
	dig_P6 = (int16_t) (calib[17] << 8 | calib[16]);
	dig_P7 = (int16_t) (calib[19] << 8 | calib[18]);
	dig_P8 = (int16_t) (calib[21] << 8 | calib[20]);
	dig_P9 = (int16_t) (calib[23] << 8 | calib[22]);
}

void BMP280_Init() {
	if (!BMP280_IsDetected()) {
		char msg[] = "Czujnik niewykryty!\r\n";
		HAL_UART_Transmit(&huart3, (uint8_t*) msg, strlen(msg), 100);
		return;
	}

	BMP_Write(BMP280_ADDR_RESET, BMP280_RESET_VALUE);
	HAL_Delay(10);

	BMP_Write(BMP280_ADDR_CONFIG, 0x00);        // filtry OFF
	BMP_Write(BMP280_ADDR_CTRL_MEAS, 0x27);     // oversampling x1 + normal mode
}

/* odczyt raw temperatury */
int32_t BMP280_ReadRawTemperature() {
	uint8_t d[3];
	BMP_Read(BMP280_ADDR_TEMP_MSB, d, 3);
	return ((int32_t) d[0] << 12) | ((int32_t) d[1] << 4) | (d[2] >> 4);
}

/* odczyt raw cisnienia */
int32_t BMP280_ReadRawPressure() {
	uint8_t d[3];
	BMP_Read(BMP280_ADDR_PRESS_MSB, d, 3);
	return ((int32_t) d[0] << 12) | ((int32_t) d[1] << 4) | (d[2] >> 4);
}

/* przeliczanie temperatury */
float BMP280_CalcTemperature(int32_t adc_T) {
	int32_t var1, var2;
	var1 = ((((adc_T >> 3) - ((int32_t) dig_T1 << 1))) * dig_T2) >> 11;
	var2 = (((((adc_T >> 4) - dig_T1) * ((adc_T >> 4) - dig_T1)) >> 12) * dig_T3) >> 14;
	t_fine = var1 + var2;
	return ((t_fine * 5 + 128) >> 8) / 100.0f;
}

/* przeliczanie cisnienia */
int32_t BMP280_CalcPressure(int32_t adc_P) {
	int64_t var1, var2, p;

	var1 = ((int64_t) t_fine) - 128000;
	var2 = var1 * var1 * (int64_t) dig_P6;
	var2 += (var1 * dig_P5) << 17;
	var2 += ((int64_t) dig_P4) << 35;

	var1 = (var1 * var1 * dig_P3) >> 8;
	var1 += (var1 * dig_P2) << 12;
	var1 = (((((int64_t) 1) << 47) + var1) * dig_P1) >> 33;

	if (var1 == 0)
		return 0;

	p = 1048576 - adc_P;
	p = (((p << 31) - var2) * 3125) / var1;

	var1 = ((int64_t) dig_P9 * (p >> 13) * (p >> 13)) >> 25;
	var2 = ((int64_t) dig_P8 * p) >> 19;

	p = ((p + var1 + var2) >> 8) + ((int64_t) dig_P7 << 4);

	return (int32_t) p;
}

void SetPWM(TIM_HandleTypeDef *htim, uint32_t channel, float duty) {
	if (duty < 0)
		duty = 0;
	if (duty > 100)
		duty = 100;

	uint32_t ARR = __HAL_TIM_GET_AUTORELOAD(htim);
	uint32_t cmp = (uint32_t) ((duty / 100.0f) * (ARR + 1));
	__HAL_TIM_SET_COMPARE(htim, channel, cmp);
}

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	/* 1 Hz – temp/CSV/UART */
	if (htim->Instance == TIM2) {
		int32_t rt = BMP280_ReadRawTemperature();
		float tempC = BMP280_CalcTemperature(rt);

		char buf[64];
		sprintf(buf, "TEMP=%.2f C\r\n", tempC);
		HAL_UART_Transmit(&huart3, (uint8_t*) buf, strlen(buf), 10);
	}

	/* 4 Hz – temp + pressure */
	if (htim->Instance == TIM3) {
		int32_t rt = BMP280_ReadRawTemperature();
		int32_t rp = BMP280_ReadRawPressure();

		int32_t temp100 = (int32_t) (BMP280_CalcTemperature(rt) * 100);
		int32_t press = BMP280_CalcPressure(rp);

		char buf[80];
		sprintf(buf, "T=%ld cC  P=%ld Pa\r\n", temp100, press);
		HAL_UART_Transmit(&huart3, (uint8_t*) buf, strlen(buf), 10);
	}
}

/* ------------------- UART RX = sterowanie PWM ------------------- */
uint8_t rxBuf[4];

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	int pwm = atoi((char*) rxBuf);
	SetPWM(&htim4, TIM_CHANNEL_1, pwm);

	HAL_UART_Receive_IT(&huart3, rxBuf, 4);
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

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
	/* USER CODE BEGIN 2 */

	/* inicjalizacja sensora */
	BMP280_Init();
	BMP280_ReadCalibration();

	/* start timerów */
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_Base_Start_IT(&htim3);

	/* PWM */
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	SetPWM(&htim4, TIM_CHANNEL_1, 0);

	/* UART RX */
	HAL_UART_Receive_IT(&huart3, rxBuf, 4);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {

		HAL_Delay(10);
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

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
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Activate the Over-Drive mode
	 */
	if (HAL_PWREx_EnableOverDrive() != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief ETH Initialization Function
 * @param None
 * @retval None
 */
static void MX_ETH_Init(void) {

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

	if (HAL_ETH_Init(&heth) != HAL_OK) {
		Error_Handler();
	}

	memset(&TxConfig, 0, sizeof(ETH_TxPacketConfig));
	TxConfig.Attributes = ETH_TX_PACKETS_FEATURES_CSUM | ETH_TX_PACKETS_FEATURES_CRCPAD;
	TxConfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
	TxConfig.CRCPadCtrl = ETH_CRC_PAD_INSERT;
	/* USER CODE BEGIN ETH_Init 2 */

	/* USER CODE END ETH_Init 2 */

}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void) {

	/* USER CODE BEGIN USART3_Init 0 */

	/* USER CODE END USART3_Init 0 */

	/* USER CODE BEGIN USART3_Init 1 */

	/* USER CODE END USART3_Init 1 */
	huart3.Instance = USART3;
	huart3.Init.BaudRate = 115200;
	huart3.Init.WordLength = UART_WORDLENGTH_8B;
	huart3.Init.StopBits = UART_STOPBITS_1;
	huart3.Init.Parity = UART_PARITY_NONE;
	huart3.Init.Mode = UART_MODE_TX_RX;
	huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart3.Init.OverSampling = UART_OVERSAMPLING_16;
	huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart3) != HAL_OK) {
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
static void MX_USB_OTG_FS_PCD_Init(void) {

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
	if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK) {
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
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */

	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOG_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, LD1_Pin | LD3_Pin | LD2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : USER_Btn_Pin */
	GPIO_InitStruct.Pin = USER_Btn_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
	GPIO_InitStruct.Pin = LD1_Pin | LD3_Pin | LD2_Pin;
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
void Error_Handler(void) {
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
