/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "spi.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"
#include "usbd_cdc_if.h"

/* Private define ------------------------------------------------------------*/
#define ARRAY_SIZE 26


/* Private typedef -----------------------------------------------------------*/
struct __attribute__((__packed__)) FRAME {
    uint8_t start;
    uint8_t line;
    uint8_t linebuff[ARRAY_SIZE * 2]; // 52 bytes
};

/* Private variables ---------------------------------------------------------*/
uint8_t tmp;
volatile uint32_t us_tick;

/* Function prototypes -------------------------------------------------------*/
void SystemClock_Config(void);
void delay_us(uint32_t time);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim);

/* Functions ------------------------------------------------------------------*/

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
    if (htim == &htim4) {
        if (us_tick > 0)
            us_tick--;
    }
}

void delay_us(uint32_t time) {
    us_tick = time / 10;
    while (us_tick != 0) {
        __asm__("nop");
    }
}

void spi_write(uint8_t addr, uint8_t data) {
    HAL_GPIO_WritePin(SPI_NCS_GPIO_Port, SPI_NCS_Pin, GPIO_PIN_RESET);
    addr |= 0x80;
    HAL_SPI_Transmit(&hspi1, &addr, 1, 10);
    HAL_SPI_Transmit(&hspi1, &data, 1, 10);
    delay_us(180);
    HAL_GPIO_WritePin(SPI_NCS_GPIO_Port, SPI_NCS_Pin, GPIO_PIN_SET);
}

uint8_t spi_read(uint8_t addr) {
    uint8_t data = 0;
    HAL_GPIO_WritePin(SPI_NCS_GPIO_Port, SPI_NCS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, &addr, 1, 10);
    delay_us(40);
    HAL_SPI_Receive(&hspi1, &data, 1, 10);
    delay_us(10);
    HAL_GPIO_WritePin(SPI_NCS_GPIO_Port, SPI_NCS_Pin, GPIO_PIN_SET);
    return data;
}

int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_SPI1_Init();
    MX_TIM4_Init();
    MX_USB_DEVICE_Init();

    HAL_GPIO_WritePin(USB_EN_GPIO_Port, USB_EN_Pin, GPIO_PIN_RESET);
    HAL_TIM_Base_Start_IT(&htim4);

    HAL_Delay(3);
    spi_write(0x3b, 0xb6);  // sensor shutdown
    HAL_Delay(300);
    delay_us(40);
    delay_us(40);
    spi_write(0x3a, 0x5a);  // power-up reset
    delay_us(50);
    spi_write(0x0d, 0x20);  // set resolution
    spi_write(0x41, 0x00);  // motion control
    spi_write(0x43, 0x08);  // auto led switching
    spi_write(0x40, 0x04);  // LED always on
    spi_write(0x45, 0x00);  // rest mode off

    struct FRAME frame;

    while (1) {
        spi_write(0x22, 0x80);  // disable rest
        spi_write(0x0b, 0x01);  // reset pixel grabber

        for (uint8_t line = 0; line < ARRAY_SIZE / 2; line++) {
            frame.start = (line == 0) ? 1 : 0;
            frame.line = line;

            uint8_t count = 0;
            while (count < ARRAY_SIZE * 2) {
                tmp = spi_read(0x0b);
                if (!(tmp & 0x80)) continue;
                frame.linebuff[count++] = tmp & 0x7F;
            }

            while (CDC_Transmit_FS((uint8_t*)&frame, sizeof(frame)) == USBD_BUSY) {
                HAL_Delay(1);
            }
        }

        // Optional frame delay
        // HAL_Delay(1);
    }
}

/* System Clock Configuration (same as before) */
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
    RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
    RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
        RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
        Error_Handler();
    }

    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
    PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
        Error_Handler();
    }
}

void Error_Handler(void) {
    __disable_irq();
    while (1) {}
}
