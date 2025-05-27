/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "spi.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"
#include "usbd_cdc_if.h"

/* Private define ------------------------------------------------------------*/
#define ARRAY_SIZE   26
#define IMG_PIXELS   (ARRAY_SIZE * ARRAY_SIZE)
#define BURST_READ_FIRST  0x42
#define FRAME_HEADER      0x01

/* Private variables ---------------------------------------------------------*/
volatile uint32_t us_tick;

/* Frame buffers -------------------------------------------------------------*/
uint8_t pixel_buf[IMG_PIXELS];
uint8_t frame_buf[1 + IMG_PIXELS];

/* External SPI handle -------------------------------------------------------*/
extern SPI_HandleTypeDef hspi1;

/* GPIO for CS from CubeMX ---------------------------------------------------*/
#define CS_GPIO_Port  SPI_NCS_GPIO_Port
#define CS_Pin        SPI_NCS_Pin

/* Function prototypes -------------------------------------------------------*/
void SystemClock_Config(void);
void delay_us(uint32_t time);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim);

/* Select/Deselect CS --------------------------------------------------------*/
static void CS_Select(void) {
    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
}
static void CS_Deselect(void) {
    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
}

/* Microsecond delay based on TIM4 interrupt --------------------------------*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
    if (htim == &htim4 && us_tick > 0) {
        us_tick--;
    }
}
void delay_us(uint32_t time) {
    us_tick = time / 10;
    while (us_tick != 0) __asm__("nop");
}

/* SPI register write (addr bit7=1 for write) ----------------------------------*/
void spi_write(uint8_t addr, uint8_t data) {
    CS_Select();
    addr |= 0x80;
    HAL_SPI_Transmit(&hspi1, &addr, 1, 10);
    HAL_SPI_Transmit(&hspi1, &data, 1, 10);
    delay_us(180);
    CS_Deselect();
}

uint8_t spi_read(uint8_t addr)
{
    uint8_t data = 0;

    HAL_GPIO_WritePin(SPI_NCS_GPIO_Port, SPI_NCS_Pin, GPIO_PIN_RESET);

    HAL_SPI_Transmit(&hspi1, (uint8_t*)&addr, 1, 10);

    delay_us(160);

    HAL_SPI_Receive(&hspi1, (uint8_t*)&data, 1, 10);

    delay_us(20);

    HAL_GPIO_WritePin(SPI_NCS_GPIO_Port, SPI_NCS_Pin, GPIO_PIN_SET);

    return data;
}

/* Burst read all pixels -----------------------------------------------------*/
static void Sensor_BurstRead(uint8_t* pixels) {
    uint8_t cmd;

    // 1) Write BURST_READ_FIRST register with 0x03 (recommended start)
    spi_write(0x42, 0x03);  // <== ¿©±â¸¦ »õ·Î Ãß°¡

    // 2) Read burst data
    CS_Select();
    cmd = BURST_READ_FIRST | 0x80;   // 0x42 | 0x80 = 0xC2 (read mode)
    HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY);
    delay_us(10);  // ´Ã·ÁÁÜ

    for (int i = 0; i < IMG_PIXELS; i++) {
        uint8_t dummy = 0;
        HAL_SPI_TransmitReceive(&hspi1, &dummy, &pixels[i], 1, HAL_MAX_DELAY);
    }
    CS_Deselect();
}

static void Sensor_ReadOneByOne(uint8_t* pixels) {
    spi_write(0x0b, 0x01);  // reset pixel grabber
    for (int i = 0; i < IMG_PIXELS;) {
        uint8_t val = spi_read(0x0b);
        if (!(val & 0x80)) continue;
        pixels[i++] = val & 0x7F;
    }
}


int main(void) {
    /* HAL & peripherals init */
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_SPI1_Init();
    MX_TIM4_Init();
    MX_USB_DEVICE_Init();

    /* Enable USB, start timer for microsecond delays */
    HAL_GPIO_WritePin(USB_EN_GPIO_Port, USB_EN_Pin, GPIO_PIN_RESET);
    HAL_TIM_Base_Start_IT(&htim4);

    /* Sensor initialization sequence ------------------------------------------*/
    HAL_Delay(3);
    spi_write(0x3b, 0xb6);  // shutdown
    HAL_Delay(300);
    delay_us(40);
    delay_us(40);
    spi_write(0x3a, 0x5a);  // power-up reset
    delay_us(50);
    spi_write(0x0d, 0x20);  // set resolution
    spi_write(0x41, 0x00);  // motion control
    spi_write(0x43, 0x08);  // auto LED switching
    spi_write(0x40, 0x04);  // LED always on
    spi_write(0x45, 0x00);  // rest mode off

    /* Prepare frame header */
    frame_buf[0] = FRAME_HEADER;

    while (1) {
        /* Trigger burst read */
        spi_write(0x22, 0x80);  // disable rest
        spi_write(0x0b, 0x01);  // reset pixel grabber
        delay_us(1);

        /* Read full image */
        //Sensor_BurstRead(pixel_buf);
        Sensor_ReadOneByOne(pixel_buf);

        /* Build frame buffer and transmit */
        memcpy(&frame_buf[1], pixel_buf, IMG_PIXELS);
        while (CDC_Transmit_FS(frame_buf, sizeof(frame_buf)) == USBD_BUSY) {
            HAL_Delay(1);
        }

        // Optional throttle
        // HAL_Delay(1);
    }
}

/* SystemClock_Config() and Error_Handler() remain unchanged */

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
