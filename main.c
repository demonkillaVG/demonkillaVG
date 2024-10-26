  /******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  * (c) EE2028 Teaching Team
  ******************************************************************************/


/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stdio.h"
#include "string.h"
#include "stdarg.h"
#include "math.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_accelero.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_tsensor.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_hsensor.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_magneto.h"

UART_HandleTypeDef huart1;

extern void initialise_monitor_handles(void);	// for semi-hosting support (printf)
static void UART1_Init(void);
int uart_print(const char* format, ...);
void u_print(void);

// Blue button press count & time
int bb_press_count = 0; // blue button press count
uint32_t bb_press_t; // blue button press time
int ghost_busting_mode = 0;

int main(void)
{
	initialise_monitor_handles(); // for semi-hosting support (printf)

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* Peripheral initializations using BSP functions */
	BSP_ACCELERO_Init();
	BSP_TSENSOR_Init();
	BSP_LED_Init(LED2);
	BSP_HSENSOR_Init();
	BSP_MAGNETO_Init();
	BSP_GYRO_Init();

	//int counter = 0;
	int delay = 1000;
	int led_t = -1;
	int h_t = -1;
	int magneto_t = -1;

	// initialising reading time interval for sensors
	uint32_t sensors_read_t = 0;
	uint32_t led_on_t;
	uint32_t accel_t;
	uint32_t temp_t;
	uint32_t humi_t;
	uint32_t gyro_t;
	uint32_t magnet_t;

	while (1)
	{
		if (!ghost_busting_mode){
			if (HAL_GetTick() - sensors_read_t >= 1000){
				sensors_read_t = HAL_GetTick();

				// Accelerometer
				float accel_data[3];
				int16_t accel_data_i16[3] = { 0 };			// array to store the x, y and z readings.
				BSP_ACCELERO_AccGetXYZ(accel_data_i16);		// read accelerometer
				float accel_val;							// Overall Magnitude
				// the function above returns 16 bit integers which are acceleration in mg (9.8/1000 m/s^2).
				// Converting to float to print the actual acceleration.
				accel_data[0] = (float)accel_data_i16[0] * (9.8/1000.0f);
				accel_data[1] = (float)accel_data_i16[1] * (9.8/1000.0f);
				accel_data[2] = (float)accel_data_i16[2] * (9.8/1000.0f);
				accel_val = sqrt(accel_data[0]*accel_data[0] + accel_data[1]*accel_data[1] + accel_data[2]*accel_data[2]);

				// Temperature Sensor
				float temp_data;
				temp_data = BSP_TSENSOR_ReadTemp();			// read temperature sensor

				// humidity sensor
				float h_data;
				h_data = BSP_HSENSOR_ReadHumidity();
				uint32_t read_h = HAL_GetTick();

				// magneto sensor
				float magneto_data[3];
				int16_t magneto_data_i16[3] = {0}; 			// declare array to store value of reading
				float magneto_val;							// Overal Magnitude
				BSP_MAGNETO_GetXYZ(magneto_data_i16);
				magneto_data[0] = (float)magneto_data_i16[0] / 1000.0; // x-axis
				magneto_data[1] = (float)magneto_data_i16[1] / 1000.0; // y-axis
				magneto_data[2] = (float)magneto_data_i16[2] / 1000.0; // z-axis
				magneto_val = sqrt(magneto_data[0]*magneto_data[0] + magneto_data[1]*magneto_data[1] + magneto_data[2]*magneto_data[2]);

				// Gyroscope
				float gyro_data[3];
				int16_t gyro_data_i16[3] = { 0 };			// array to store the x, y and z readings.
				BSP_GYRO_GetXYZ(gyro_data_i16);				// read gyroscope
				float gyro_val;							// Overall Magnitude
				// the function above returns 16 bit integers which are acceleration in mg (9.8/1000 m/s^2).
				// Converting to float to print the actual acceleration.
				gyro_data[0] = (float)gyro_data_i16[0] * (9.8/1000.0f);
				gyro_data[1] = (float)gyro_data_i16[1] * (9.8/1000.0f);
				gyro_data[2] = (float)gyro_data_i16[2] * (9.8/1000.0f);
				gyro_val = sqrt(gyro_data[0]*gyro_data[0] + gyro_data[1]*gyro_data[1] + gyro_data[2]*gyro_data[2]);

				uart_print("T: %.2f°C, P:pp.pp (unit), H: %.2f%%, A: %.2fm/s², G: %.2f (unit), M: %.2f Gauss\r\n", temp_data, h_data, accel_val, gyro_val, magneto_val);
			}
		}
//		int done_t = HAL_GetTick();
//		while (HAL_GetTick() - done_t < 10){
//
//		}
///******* Lab 3 *******/
//		float accel_data[3];
//		int16_t accel_data_i16[3] = { 0 };			// array to store the x, y and z readings.
//		BSP_ACCELERO_AccGetXYZ(accel_data_i16);		// read accelerometer
//		// the function above returns 16 bit integers which are acceleration in mg (9.8/1000 m/s^2).
//		// Converting to float to print the actual acceleration.
//		accel_data[0] = (float)accel_data_i16[0] * (9.8/1000.0f);
//		accel_data[1] = (float)accel_data_i16[1] * (9.8/1000.0f);
//		accel_data[2] = (float)accel_data_i16[2] * (9.8/1000.0f);
//
//		float temp_data;
//		temp_data = BSP_TSENSOR_ReadTemp();			// read temperature sensor
//
//		if (counter % 2 == 0){
//			printf("Accel X : %f; Accel Y : %f; Accel Z : %f;\n", accel_data[0], accel_data[1], accel_data[2]);
//		}
//		if (counter % 3 == 0){
//			printf("Temperature : %f\n", temp_data);
//		}
//		counter++;
//		HAL_Delay(500);	// read once a ~0.5 second.
//
///******* Lab 4 *******/
//		if (HAL_GetTick() - led_on_t > delay){
//			led_on_t = HAL_GetTick();
//			BSP_LED_Toggle(LED2);
//		}
//
//		// humidity sensor
//		float h_data;
//		h_data = BSP_HSENSOR_ReadHumidity();
//		uint32_t read_h = HAL_GetTick();
//		printf("Humidity: %f%%\n", h_data);
//
//		//magneto sensor
//		float magneto_data[3];
//		int16_t magneto_data_i16[3] = {0}; // declare array to store value of reading
//		BSP_MAGNETO_GetXYZ(magneto_data_i16);
//		magneto_data[0] = (float)magneto_data_i16[0] / 1000.0; // x-axis
//		magneto_data[1] = (float)magneto_data_i16[1] / 1000.0; // y-axis
//		magneto_data[2] = (float)magneto_data_i16[2] / 1000.0; // z-axis
//		printf("Magneto X : %f; Magneto Y : %f; Magneto Z : %f;\n", magneto_data[0], magneto_data[1], magneto_data[2]);
//		uart_print("Magneto X : %f; Magneto Y : %f; Magneto Z : %f;\r\n", magneto_data[0], magneto_data[1], magneto_data[2]);
//		HAL_Delay(1000);

	}

}

/***************************
*UART
***************************/

// Custom function to make UART communication like printf.
int uart_print(const char* format, ...){
	UART1_Init();
	// Size of buffer indicates number of characters UART can transmit
    char message_print[256];  // Message buffer
    va_list args;
    va_start(args, format);
    vsnprintf(message_print, sizeof(message_print), format, args);  // format the string
    va_end(args);

    // Transmit the formatted string via UART
    HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print), 0xFFFF);

    return 0;
}

// Initialisation
static void UART1_Init(void){
        /* Pin configuration for UART. BSP_COM_Init() can do this automatically */
        __HAL_RCC_GPIOB_CLK_ENABLE();
        GPIO_InitTypeDef GPIO_InitStruct = {0};
        GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
        GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_6;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        /* Configuring UART1 */
        huart1.Instance = USART1;
        huart1.Init.BaudRate = 115200;
        huart1.Init.WordLength = UART_WORDLENGTH_8B;
        huart1.Init.StopBits = UART_STOPBITS_1;
        huart1.Init.Parity = UART_PARITY_NONE;
        huart1.Init.Mode = UART_MODE_TX_RX;
        huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
        huart1.Init.OverSampling = UART_OVERSAMPLING_16;
        huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
        huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
        if (HAL_UART_Init(&huart1) != HAL_OK)
        {
          while(1);
        }
}
