#include "main.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"  // CDC 전송 함수 사용

/* -- Private defines -- */
#define BURST_SIZE 9

/* -- SPI, GPIO, USB Handles -- */
extern SPI_HandleTypeDef hspi1;
extern USBD_HandleTypeDef hUsbDeviceFS;

/* -- PMW3320 Burst Read Data Struct -- */
#pragma pack(push, 1)
typedef struct {
	uint8_t motion;
	int8_t  delta_x;
	int8_t  delta_y;
	uint8_t squal;
	uint8_t shutter_upper;
	uint8_t shutter_lower;
	uint8_t pix_max;
	uint8_t pix_accum;
	uint8_t pix_min;
} PMW3320_BurstData;
#pragma pack(pop)

/* -- Function Prototypes -- */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);

/* -- Delay Helpers -- */
void delay_us(uint32_t time);
volatile uint32_t us_tick = 0;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
	if (us_tick > 0)
		us_tick--;
}

void delay_us(uint32_t time) {
	us_tick = time / 10;
	while (us_tick != 0) __NOP();
}

/* -- PMW3320 SPI Helpers -- */
void spi_write(uint8_t addr, uint8_t data) {
	HAL_GPIO_WritePin(SPI_NCS_GPIO_Port, SPI_NCS_Pin, GPIO_PIN_RESET);
	addr |= 0x80;
	HAL_SPI_Transmit(&hspi1, &addr, 1, 100);
	HAL_SPI_Transmit(&hspi1, &data, 1, 100);
	delay_us(180);
	HAL_GPIO_WritePin(SPI_NCS_GPIO_Port, SPI_NCS_Pin, GPIO_PIN_SET);
}

void PMW3320_ReadBurst(PMW3320_BurstData* burst) {
	uint8_t addr = 0x63;
	HAL_GPIO_WritePin(SPI_NCS_GPIO_Port, SPI_NCS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, &addr, 1, HAL_MAX_DELAY);
	HAL_SPI_Receive(&hspi1, (uint8_t*)burst, sizeof(PMW3320_BurstData), HAL_MAX_DELAY);
	HAL_GPIO_WritePin(SPI_NCS_GPIO_Port, SPI_NCS_Pin, GPIO_PIN_SET);
}

/* -- Main Function -- */
int main(void) {
	HAL_Init();
	SystemClock_Config();
	MX_GPIO_Init();
	MX_SPI1_Init();
	MX_USB_DEVICE_Init();

	HAL_Delay(3);

	// 센서 초기화
	spi_write(0x3b, 0xb6);   // Shutdown
	HAL_Delay(300);
	spi_write(0x3a, 0x5a);   // Power-up reset
	delay_us(50);
	spi_write(0x0d, 0x20);   // Resolution
	spi_write(0x41, 0x00);   // Motion control
	spi_write(0x43, 0x08);   // Auto LED
	spi_write(0x40, 0x04);   // LED always on
	spi_write(0x45, 0x00);   // Rest mode disable
	spi_write(0x22, 0x80);   // Force awake

	PMW3320_BurstData burst_data;
	uint8_t usb_buffer[64];

	while (1) {
		// Read Burst data
		PMW3320_ReadBurst(&burst_data);

		// Build USB output string
		int len = snprintf((char*)usb_buffer, sizeof(usb_buffer),
			"M:%02X DX:%d DY:%d SQUAL:%u SHUT:%u PIX:%u/%u/%u\r\n",
			burst_data.motion,
			burst_data.delta_x, burst_data.delta_y,
			burst_data.squal,
			(burst_data.shutter_upper << 8) | burst_data.shutter_lower,
			burst_data.pix_min,
			burst_data.pix_accum,
			burst_data.pix_max);

		// Transmit over USB CDC
		CDC_Transmit_FS(usb_buffer, len);

		HAL_Delay(10); // 100 Hz update
	}
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
		| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
	{
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
	PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
	{
		Error_Handler();
	}
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
	hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi1.Init.NSS = SPI_NSS_SOFT;
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN SPI1_Init 2 */

	/* USER CODE END SPI1_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, SPI_NCS_Pin | USB_EN_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, LED6_Pin | LED3_Pin | LED4_Pin | LED5_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : SPI_NCS_Pin */
	GPIO_InitStruct.Pin = SPI_NCS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(SPI_NCS_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : LED6_Pin LED3_Pin LED4_Pin LED5_Pin */
	GPIO_InitStruct.Pin = LED6_Pin | LED3_Pin | LED4_Pin | LED5_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : USB_EN_Pin */
	GPIO_InitStruct.Pin = USB_EN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(USB_EN_GPIO_Port, &GPIO_InitStruct);

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
void assert_failed(uint8_t* file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	 /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */