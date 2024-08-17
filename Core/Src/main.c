/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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
#include "i2c.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "MPU9250.h"
#include "bmp280.h"

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

/* USER CODE BEGIN PV */

BMP280_HandleTypedef bmp280;

float pressure, temperature, humidity;

uint16_t size;
uint8_t  Data[ 256 ];

float Acc_x, Acc_y, Acc_z;
float Mag_x, Mag_y, Mag_z;
float Gyro_x, Gyro_y, Gyro_z;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config( void );
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main( void ) {

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
    MX_SPI1_Init();
    MX_I2C1_Init();
    MX_USART1_UART_Init();
    /* USER CODE BEGIN 2 */

    // bmp280_init_default_params( &bmp280.params );
    // bmp280.addr = BMP280_I2C_ADDRESS_0;
    // bmp280.i2c  = &hi2c1;

    //    while ( !bmp280_init( &bmp280, &bmp280.params ) ) {
    //        // size = sprintf( ( char* )Data, "BMP280 initialization failed\n" );
    //        HAL_UART_Transmit( &huart1, Data, size, 1000 );
    //        HAL_Delay( 2000 );
    //    }
    // bool bme280p = bmp280.id == BME280_CHIP_ID;

    // size = sprintf( ( char* )Data, "BMP280: found %s\n", bme280p ? "BME280" : "BMP280" );
    // HAL_UART_Transmit( &huart1, Data, size, 1000 );

    MPU9250_Init();

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while ( 1 ) {
        MPU9250_GetData( AccData, MagData, GyroData );  // INFO 获取数据

        Acc_x = ( float )AccData[ 0 ] * ( 16.0 / 32768.0 );
        Acc_y = ( float )AccData[ 1 ] * ( 16.0 / 32768.0 );
        Acc_z = ( float )AccData[ 2 ] * ( 16.0 / 32768.0 );


        Gyro_x = ( float )GyroData[ 0 ] * ( 2000.0 / 32768.0 );
        Gyro_y = ( float )GyroData[ 1 ] * ( 2000.0 / 32768.0 );
        Gyro_z = ( float )GyroData[ 2 ] * ( 2000.0 / 32768.0 );

        // // 14bit
        // Mag_x = MagData[ 0 ] * ( 10.0 * 4219.0 / 8190.0 );
        // Mag_y = MagData[ 1 ] * ( 10.0 * 4219.0 / 8190.0 );
        // Mag_z = MagData[ 2 ] * ( 10.0 * 4219.0 / 8190.0 );

        // 16bit
        Mag_x = ( float )MagData[ 0 ] * ( 10.0 * 4219.0 / 32760.0 );
        Mag_y = ( float )MagData[ 1 ] * ( 10.0 * 4219.0 / 32760.0 );
        Mag_z = ( float )MagData[ 2 ] * ( 10.0 * 4219.0 / 32760.0 );
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config( void ) {
    RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
    RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState       = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL     = RCC_PLL_MUL9;
    if ( HAL_RCC_OscConfig( &RCC_OscInitStruct ) != HAL_OK ) {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if ( HAL_RCC_ClockConfig( &RCC_ClkInitStruct, FLASH_LATENCY_2 ) != HAL_OK ) {
        Error_Handler();
    }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler( void ) {
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while ( 1 ) {
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
void assert_failed( uint8_t* file, uint32_t line ) {
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
