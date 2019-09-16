/**
 ******************************************************************************
 * @file    main.c
 * @author  MEMS Software Solutions Team
 * @brief   Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2018 STMicroelectronics</center></h2>
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

/**
 * @mainpage Documentation for MotionFX package of X-CUBE-MEMS1 Software for X-NUCLEO-IKS01Ax expansion boards
 *
 * @image html st_logo.png
 *
 * <b>Introduction</b>
 *
 * MotionFX software is an add-on for the X-CUBE-MEMS1 software and provides
 * the real-time sensor fusion data.
 * The expansion is built on top of STM32Cube software technology that eases
 * portability across different STM32 microcontrollers.
 */

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include "main.h"
#include "com.h"
#include "DemoSerial.h"
#include "MotionFX_Manager.h"
#include "i2c.h"
/** @addtogroup MOTION_APPLICATIONS MOTION APPLICATIONS
 * @{
 */

/** @addtogroup DATALOG_FUSION DATALOG FUSION
 * @{
 */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define ALGO_FREQ    100U /* Algorithm frequency [Hz] */
#define ALGO_PERIOD  10   /* Algorithm period [ms] */
#define MOTIONFX_ENGINE_DELTATIME  0.01f

#define FROM_MG_TO_G                    0.001f
#define FROM_G_TO_MG                    1000.0f
#define FROM_MDPS_TO_DPS                0.001f
#define FROM_DPS_TO_MDPS                1000.0f
#define FROM_MGAUSS_TO_UT50             (0.1f/50.0f)
#define FROM_UT50_TO_MGAUSS             500.0f
#define ACC_GYRO_ADDR 0xD7

/* Extern variables ----------------------------------------------------------*/
volatile uint8_t DataLoggerActive = 0;
extern int UseLSI;
extern volatile uint32_t SensorsEnabled; /* This "redundant" line is here to fulfil MISRA C-2012 rule 8.4 */
volatile uint32_t SensorsEnabled = 0;
TIM_HandleTypeDef AlgoTimHandle;
extern uint8_t Enabled6X;

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static int RtcSynchPrediv;
static RTC_HandleTypeDef RtcHandle;
static IKS01A2_MOTION_SENSOR_Axes_t AccValue;
static IKS01A2_MOTION_SENSOR_Axes_t GyrValue;
static IKS01A2_MOTION_SENSOR_Axes_t MagValue;
static IKS01A2_MOTION_SENSOR_Axes_t MagOffset;
static volatile uint8_t SensorReadRequest = 0;
static volatile uint8_t MagCalRequest = 0;
HAL_StatusTypeDef status;
extern I2C_HandleTypeDef hi2c1;
#if ((defined (USE_STM32F4XX_NUCLEO)) || (defined (USE_STM32L4XX_NUCLEO)) || (defined (USE_STM32L1XX_NUCLEO)))
static uint32_t MagTimeStamp = 0;
#endif

static uint8_t MagCalStatus = 0;

/* Private function prototypes -----------------------------------------------*/
static void RTC_Config(void);
static void RTC_TimeStampConfig(void);
static void Init_Sensors(void);
static void MX_GPIO_Init(void);
static void MX_CRC_Init(void);
static void MX_TIM_ALGO_Init(void);
static void RTC_Handler(TMsg *Msg);
static void FX_Data_Handler(TMsg *Msg);
static void Accelero_Sensor_Handler(TMsg *Msg, uint32_t Instance);
static void Gyro_Sensor_Handler(TMsg *Msg, uint32_t Instance);
static void Magneto_Sensor_Handler(TMsg *Msg, uint32_t Instance);
//static void Pressure_Sensor_Handler(TMsg *Msg, uint32_t Instance);
//static void Humidity_Sensor_Handler(TMsg *Msg, uint32_t Instance);
//static void Temperature_Sensor_Handler(TMsg *Msg, uint32_t Instance);

/* Public functions ----------------------------------------------------------*/
/**
 * @brief  Main function is to show how to use X_NUCLEO_IKS01Ax
 *         expansion board to get postion data and send it from a Nucleo
 *         board to a connected PC, using UART, displaying it on Unicleo-GUI
 *         Graphical User Interface, developed by STMicroelectronics and provided
 *         with X-CUBE-MEMS1 package.
 *         After connection has been established with GUI, the user can visualize
 *         the data and save datalog for offline analysis.
 *         See User Manual for details.
 * @param  None
 * @retval None
 */
int main(void); /* This "redundant" line is here to fulfil MISRA C-2012 rule 8.4 */
int main(void)
{
  char lib_version[35];
  int lib_version_len;
  float ans_float;
  TMsg msg_dat;
  TMsg msg_cmd;

  /* STM32xxxx HAL library initialization:
  - Configure the Flash prefetch, instruction and Data caches
  - Configure the Systick to generate an interrupt each 1 msec
  - Set NVIC Group Priority to 4
  - Global MSP (MCU Support Package) initialization
  */
  (void)HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the SysTick IRQ priority - set the second lowest priority */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0x0E, 0);

  /* Initialize GPIOs */
  MX_GPIO_Init();

  /* Initialize CRC */
  MX_CRC_Init();
 MX_I2C1_Init();
  /* Initialize (disabled) Sensors */
  Init_Sensors();

  /* Sensor Fusion API initialization function */
  MotionFX_manager_init();

  /* OPTIONAL */
  /* Get library version */
  MotionFX_manager_get_version(lib_version, &lib_version_len);

  /* Initialize Communication Peripheral for data log */
  USARTConfig();

  /* RTC Initialization */
  RTC_Config();
  RTC_TimeStampConfig();

  /* Timer for algorithm synchronization initialization */
  MX_TIM_ALGO_Init();

  /* LED Blink */
  BSP_LED_On(LED2);
  HAL_Delay(500);
  BSP_LED_Off(LED2);

  /* Enable magnetometer calibration */
  MotionFX_manager_MagCal_start(ALGO_PERIOD);

  /* Test if calibration data are available */
#if ((defined (USE_STM32F4XX_NUCLEO)) || (defined (USE_STM32L4XX_NUCLEO)) || (defined (USE_STM32L1XX_NUCLEO)))
  MFX_MagCal_output_t mag_cal_test;
  MotionFX_MagCal_getParams(&mag_cal_test);

  /* If calibration data are available load HI coeficients */
  if (mag_cal_test.cal_quality == MFX_MAGCALGOOD)
#elif (defined (USE_STM32L0XX_NUCLEO))
  MFX_CM0P_MagCal_output_t mag_cal_test;
  MotionFX_CM0P_MagCal_getParams(&mag_cal_test);

  /* If calibration data are available load HI coeficients */
  if (mag_cal_test.cal_quality == MFX_CM0P_MAGCALGOOD)
#else
#error Not supported platform
#endif
  {
    ans_float = (mag_cal_test.hi_bias[0] * FROM_UT50_TO_MGAUSS);
    MagOffset.x = (int32_t)ans_float;
    ans_float = (mag_cal_test.hi_bias[1] * FROM_UT50_TO_MGAUSS);
    MagOffset.y = (int32_t)ans_float;
    ans_float = (mag_cal_test.hi_bias[2] * FROM_UT50_TO_MGAUSS);
    MagOffset.z = (int32_t)ans_float;

    MagCalStatus = 1;
  }
HAL_TIM_Base_Start_IT(&AlgoTimHandle);
	 MotionFX_manager_start_9X();
	
  for (;;)
  {
//    if (UART_ReceivedMSG((TMsg *)&msg_cmd) != 1)
//    {
//      if (msg_cmd.Data[0] == DEV_ADDR)
//      {
//        (void)HandleMSG((TMsg *)&msg_cmd);
//      }
//    }

    if (MagCalRequest == 1U)
    {
      MagCalRequest = 0;

      /* Reset magnetometer calibration value*/
      MagCalStatus = 0;
      MagOffset.x = 0;
      MagOffset.y = 0;
      MagOffset.z = 0;

#if ((defined (MOTION_FX_STORE_CALIB_FLASH)) && ((defined (USE_STM32L4XX_NUCLEO)) || (defined (USE_STM32L1XX_NUCLEO)) || (defined (USE_STM32F4XX_NUCLEO))))
      /* Reset values in memory */
      ResetCalibrationInMemory();
#endif

      /* Enable magnetometer calibration */
      MotionFX_manager_MagCal_start(ALGO_PERIOD);
    }

    if (SensorReadRequest == 1U)
    {
      SensorReadRequest = 0;

      /* Acquire data from enabled sensors and fill Msg stream */
      //RTC_Handler(&msg_dat);
      Accelero_Sensor_Handler(&msg_dat, IKS01A2_LSM6DSL_0);
      Gyro_Sensor_Handler(&msg_dat, IKS01A2_LSM6DSL_0);
      Magneto_Sensor_Handler(&msg_dat, IKS01A2_LSM303AGR_MAG_0);
//      Humidity_Sensor_Handler(&msg_dat, IKS01A2_HTS221_0);
//      Temperature_Sensor_Handler(&msg_dat, IKS01A2_HTS221_0);
//      Pressure_Sensor_Handler(&msg_dat, IKS01A2_LPS22HB_0);

      /* Sensor Fusion specific part */
      FX_Data_Handler(&msg_dat);

      /* Send data stream */
//      INIT_STREAMING_HEADER(&msg_dat);
//      msg_dat.Len = STREAMING_MSG_LENGTH;
//      UART_SendMsg(&msg_dat);
    }
  }
}

/* Private functions ---------------------------------------------------------*/
/**
 * @brief  Initialize all sensors
 * @param  None
 * @retval None
 */
static void Init_Sensors(void)
{
//  (void)IKS01A2_MOTION_SENSOR_Init(IKS01A2_LSM6DSL_0,  MOTION_GYRO);
	// MOTION_ACCELERO |
//  (void)IKS01A2_MOTION_SENSOR_Init(IKS01A2_LSM303AGR_MAG_0, MOTION_MAGNETO);
	uint8_t data[1];
	data[0] = 0x01;
	status = HAL_I2C_Mem_Write(&hi2c1, ACC_GYRO_ADDR, 0x12, I2C_MEMADD_SIZE_8BIT , data, 0x01, 100); //soft reset LSM6DSL
	HAL_Delay(10);
	data[0] = 0x40;
  status = HAL_I2C_Mem_Write(&hi2c1, ACC_GYRO_ADDR, 0x10, I2C_MEMADD_SIZE_8BIT , data, 0x01, 100); //acc on
	data[0] = 0x4C;
	status = HAL_I2C_Mem_Write(&hi2c1, ACC_GYRO_ADDR, 0x11, I2C_MEMADD_SIZE_8BIT , data, 0x01, 100); //gyro on
	 status = HAL_I2C_Mem_Read(&hi2c1, ACC_GYRO_ADDR, 0x0F, I2C_MEMADD_SIZE_8BIT , data, 0x01, 100);
	data[0] = 0x28;
	status = HAL_I2C_Mem_Write(&hi2c1, 0x3D, 0x60, I2C_MEMADD_SIZE_8BIT , data, 0x01, 100); //mag = 100Hz(high resolution + continuous mode)
	HAL_Delay(10);
		data[0] = 0x08;
	status = HAL_I2C_Mem_Write(&hi2c1, 0x3D, 0x60, I2C_MEMADD_SIZE_8BIT , data, 0x01, 100); // mag ODR = 20 Hz
		data[0] = 0x01;
	status = HAL_I2C_Mem_Write(&hi2c1, 0x3D, 0x62, I2C_MEMADD_SIZE_8BIT , data, 0x01, 100); // mag data ready interrupt enable	
  (void)IKS01A2_ENV_SENSOR_Init(IKS01A2_HTS221_0, ENV_TEMPERATURE | ENV_HUMIDITY);
  (void)IKS01A2_ENV_SENSOR_Init(IKS01A2_LPS22HB_0, ENV_PRESSURE);
}

//static void Init_Sensors(void)
//{
//	uint8_t data[1];
//	data[0] = 0x01;
//	status = HAL_I2C_Mem_Write(&hi2c1, ACC_GYRO_ADDR, 0x12, I2C_MEMADD_SIZE_8BIT , data, 0x01, 100); //soft reset LSM6DSL
//	HAL_Delay(10);
//	data[0] = 0x40;
//	status = HAL_I2C_Mem_Write(&hi2c1, ACC_GYRO_ADDR, 0x10, I2C_MEMADD_SIZE_8BIT , data, 0x01, 100); //acc on
//	data[0] = 0x4C;
//	status = HAL_I2C_Mem_Write(&hi2c1, ACC_GYRO_ADDR, 0x11, I2C_MEMADD_SIZE_8BIT , data, 0x01, 100); //gyro on
//	 status = HAL_I2C_Mem_Read(&hi2c1, ACC_GYRO_ADDR, 0x0F, I2C_MEMADD_SIZE_8BIT , data, 0x01, 100);
//	
//	data[0] = 0x28;
//	status = HAL_I2C_Mem_Write(&hi2c1, 0x3D, 0x60, I2C_MEMADD_SIZE_8BIT , data, 0x01, 100); //mag = 100Hz(high resolution + continuous mode)
//	HAL_Delay(10);
//		data[0] = 0x08;
//	status = HAL_I2C_Mem_Write(&hi2c1, 0x3D, 0x60, I2C_MEMADD_SIZE_8BIT , data, 0x01, 100); // mag ODR = 20 Hz
//		data[0] = 0x01;
//	status = HAL_I2C_Mem_Write(&hi2c1, 0x3D, 0x62, I2C_MEMADD_SIZE_8BIT , data, 0x01, 100); // mag data ready interrupt enable	
//}


/**
 * @brief  GPIO init function.
 * @param  None
 * @retval None
 * @details GPIOs initialized are User LED(PA5) and User Push Button(PC1)
 */
static void MX_GPIO_Init(void)
{
  /* Initialize LED */
  BSP_LED_Init(LED2);

  /* Initialize push button */
  BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_EXTI);
}

/**
 * @brief  CRC init function.
 * @param  None
 * @retval None
 */
static void MX_CRC_Init(void)
{
  __CRC_CLK_ENABLE();
}

/**
 * @brief  TIM_ALGO init function.
 * @param  None
 * @retval None
 * @details This function intializes the Timer used to synchronize the algorithm.
 */
static void MX_TIM_ALGO_Init(void)
{
#if (defined (USE_STM32F4XX_NUCLEO))
#define CPU_CLOCK  84000000U

#elif (defined (USE_STM32L0XX_NUCLEO))
#define CPU_CLOCK  32000000U

#elif (defined (USE_STM32L1XX_NUCLEO))
#define CPU_CLOCK  32000000U

#elif (defined (USE_STM32L4XX_NUCLEO))
#define CPU_CLOCK  80000000U

#else
#error Not supported platform
#endif

#define TIM_CLOCK  2000U

  const uint32_t prescaler = CPU_CLOCK / TIM_CLOCK - 1U;
  const uint32_t tim_period = TIM_CLOCK / ALGO_FREQ - 1U;

  TIM_ClockConfigTypeDef s_clock_source_config;
  TIM_MasterConfigTypeDef s_master_config;

  AlgoTimHandle.Instance           = TIM_ALGO;
  AlgoTimHandle.Init.Prescaler     = prescaler;
  AlgoTimHandle.Init.CounterMode   = TIM_COUNTERMODE_UP;
  AlgoTimHandle.Init.Period        = tim_period;
  AlgoTimHandle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  (void)HAL_TIM_Base_Init(&AlgoTimHandle);

  s_clock_source_config.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  (void)HAL_TIM_ConfigClockSource(&AlgoTimHandle, &s_clock_source_config);

  s_master_config.MasterOutputTrigger = TIM_TRGO_RESET;
  s_master_config.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
  (void)HAL_TIMEx_MasterConfigSynchronization(&AlgoTimHandle, &s_master_config);
}

/**
 * @brief  Handles the time+date getting/sending
 * @param  Msg the time+date part of the stream
 * @retval None
 */
static void RTC_Handler(TMsg *Msg)
{
  uint8_t sub_sec;
  uint32_t ans_uint32;
  int32_t ans_int32;
  RTC_DateTypeDef sdatestructureget;
  RTC_TimeTypeDef stimestructure;

  (void)HAL_RTC_GetTime(&RtcHandle, &stimestructure, FORMAT_BIN);
  (void)HAL_RTC_GetDate(&RtcHandle, &sdatestructureget, FORMAT_BIN);

  /* To be MISRA C-2012 compliant the original calculation:
     sub_sec = ((((((int)RtcSynchPrediv) - ((int)stimestructure.SubSeconds)) * 100) / (RtcSynchPrediv + 1)) & 0xFF);
     has been split to separate expressions */
  ans_int32 = (RtcSynchPrediv - (int32_t)stimestructure.SubSeconds) * 100;
  ans_int32 /= RtcSynchPrediv + 1;
  ans_uint32 = (uint32_t)ans_int32 & 0xFFU;
  sub_sec = (uint8_t)ans_uint32;

  Msg->Data[3] = (uint8_t)stimestructure.Hours;
  Msg->Data[4] = (uint8_t)stimestructure.Minutes;
  Msg->Data[5] = (uint8_t)stimestructure.Seconds;
  Msg->Data[6] = sub_sec;
}

/**
 * @brief  Sensor Fusion data handler
 * @param  Msg the Sensor Fusion data part of the stream
 * @retval None
 */
static void FX_Data_Handler(TMsg *Msg)
{
#if ((defined (USE_STM32F4XX_NUCLEO)) || (defined (USE_STM32L4XX_NUCLEO)) || (defined (USE_STM32L1XX_NUCLEO)))
  MFX_input_t data_in;
  MFX_input_t *pdata_in = &data_in;
  MFX_output_t data_out;
  MFX_output_t *pdata_out = &data_out;
#elif (defined (USE_STM32L0XX_NUCLEO))
  MFX_CM0P_input_t data_in;
  MFX_CM0P_input_t *pdata_in = &data_in;
  MFX_CM0P_output_t data_out;
  MFX_CM0P_output_t *pdata_out = &data_out;
#else
#error Not supported platform
#endif

//  if ((SensorsEnabled & ACCELEROMETER_SENSOR) == ACCELEROMETER_SENSOR)
//  {
//    if ((SensorsEnabled & GYROSCOPE_SENSOR) == GYROSCOPE_SENSOR)
//    {
//      if ((SensorsEnabled & MAGNETIC_SENSOR) == MAGNETIC_SENSOR)
//      {
        data_in.gyro[0] = (float)GyrValue.x * FROM_MDPS_TO_DPS;
        data_in.gyro[1] = (float)GyrValue.y * FROM_MDPS_TO_DPS;
        data_in.gyro[2] = (float)GyrValue.z * FROM_MDPS_TO_DPS;

        data_in.acc[0] = (float)AccValue.x * FROM_MG_TO_G;
        data_in.acc[1] = (float)AccValue.y * FROM_MG_TO_G;
        data_in.acc[2] = (float)AccValue.z * FROM_MG_TO_G;

        data_in.mag[0] = (float)MagValue.x * FROM_MGAUSS_TO_UT50;
        data_in.mag[1] = (float)MagValue.y * FROM_MGAUSS_TO_UT50;
        data_in.mag[2] = (float)MagValue.z * FROM_MGAUSS_TO_UT50;

        /* Run Sensor Fusion algorithm */
        BSP_LED_On(LED2);
        MotionFX_manager_run(pdata_in, pdata_out, MOTIONFX_ENGINE_DELTATIME);
        BSP_LED_Off(LED2);

        if (Enabled6X == 1U)
        {
          (void)memcpy(&Msg->Data[55], (void *)pdata_out->quaternion_6X, 4U * sizeof(float));
          (void)memcpy(&Msg->Data[71], (void *)pdata_out->rotation_6X, 3U * sizeof(float));
          (void)memcpy(&Msg->Data[83], (void *)pdata_out->gravity_6X, 3U * sizeof(float));
          (void)memcpy(&Msg->Data[95], (void *)pdata_out->linear_acceleration_6X, 3U * sizeof(float));

#if ((defined (USE_STM32F4XX_NUCLEO)) || (defined (USE_STM32L4XX_NUCLEO)) || (defined (USE_STM32L1XX_NUCLEO)))
          (void)memcpy(&Msg->Data[107], (void *) & (pdata_out->heading_6X), sizeof(float));
          (void)memcpy(&Msg->Data[111], (void *) & (pdata_out->headingErr_6X), sizeof(float));
#elif (defined (USE_STM32L0XX_NUCLEO))
          (void)memset(&Msg->Data[107], 0, sizeof(float));
          (void)memset(&Msg->Data[111], 0, sizeof(float));
#else
#error Not supported platform
#endif
        }
        else
        {
          (void)memcpy(&Msg->Data[55], (void *)pdata_out->quaternion_9X, 4U * sizeof(float));
          (void)memcpy(&Msg->Data[71], (void *)pdata_out->rotation_9X, 3U * sizeof(float));
          (void)memcpy(&Msg->Data[83], (void *)pdata_out->gravity_9X, 3U * sizeof(float));
          (void)memcpy(&Msg->Data[95], (void *)pdata_out->linear_acceleration_9X, 3U * sizeof(float));

#if ((defined (USE_STM32F4XX_NUCLEO)) || (defined (USE_STM32L4XX_NUCLEO)) || (defined (USE_STM32L1XX_NUCLEO)))
          (void)memcpy(&Msg->Data[107], (void *) & (pdata_out->heading_9X), sizeof(float));
          (void)memcpy(&Msg->Data[111], (void *) & (pdata_out->headingErr_9X), sizeof(float));
#elif (defined (USE_STM32L0XX_NUCLEO))
          (void)memset(&Msg->Data[107], 0, sizeof(float));
          (void)memset(&Msg->Data[111], 0, sizeof(float));
#else
#error Not supported platform
#endif
//        }
//      }
//    }
  }
}

/**
 * @brief  Handles the ACC axes data getting/sending
 * @param  Msg the ACC part of the stream
 * @param  Instance the device instance
 * @retval None
 */
static void Accelero_Sensor_Handler(TMsg *Msg, uint32_t Instance)
{
//  if ((SensorsEnabled & ACCELEROMETER_SENSOR) == ACCELEROMETER_SENSOR)
//  {
//    (void)IKS01A2_MOTION_SENSOR_GetAxes(Instance, MOTION_ACCELERO, &AccValue);
	uint8_t data[2] = {0};
		
   status = HAL_I2C_Mem_Read(&hi2c1, ACC_GYRO_ADDR, 0x28, I2C_MEMADD_SIZE_8BIT , data, 0x02, 100); 
	
	int16_t a;
	a=(data[1] << 8)|data[0];
	AccValue.x = a/15;					
				
		 status = HAL_I2C_Mem_Read(&hi2c1, ACC_GYRO_ADDR, 0x2A, I2C_MEMADD_SIZE_8BIT , data, 0x02, 100);
	a=(data[1] << 8)|data[0];	
		AccValue.y = a/15;
				
		status = HAL_I2C_Mem_Read(&hi2c1, ACC_GYRO_ADDR, 0x2C, I2C_MEMADD_SIZE_8BIT , data, 0x02, 100); 
		a=(data[1] << 8)|data[0];
		AccValue.z =  a/15;		
	
//    Serialize_s32(&Msg->Data[19], (int32_t)AccValue.x, 4);
//    Serialize_s32(&Msg->Data[23], (int32_t)AccValue.y, 4);
//    Serialize_s32(&Msg->Data[27], (int32_t)AccValue.z, 4);
//  }
}

/**
 * @brief  Handles the GYR axes data getting/sending
 * @param  Msg the GYR part of the stream
 * @param  Instance the device instance
 * @retval None
 */
static void Gyro_Sensor_Handler(TMsg *Msg, uint32_t Instance)
{
//  if ((SensorsEnabled & GYROSCOPE_SENSOR) == GYROSCOPE_SENSOR)
//  {
//    (void)IKS01A2_MOTION_SENSOR_GetAxes(Instance, MOTION_GYRO, &GyrValue);
	uint8_t data[2] = {0};
		 status = HAL_I2C_Mem_Read(&hi2c1, ACC_GYRO_ADDR, 0x22, I2C_MEMADD_SIZE_8BIT , data, 0x02, 100); 
	
	int16_t a;
	a=(data[1] << 8)|data[0];
		GyrValue.x = a;					
		 status = HAL_I2C_Mem_Read(&hi2c1, ACC_GYRO_ADDR, 0x24, I2C_MEMADD_SIZE_8BIT , data, 0x02, 100);
a=(data[1] << 8)|data[0];	
		GyrValue.y = a;
		status = HAL_I2C_Mem_Read(&hi2c1, ACC_GYRO_ADDR, 0x26, I2C_MEMADD_SIZE_8BIT , data, 0x02, 100); 
		a=(data[1] << 8)|data[0];
		GyrValue.z = a;	
	
//    Serialize_s32(&Msg->Data[31], GyrValue.x, 4);
//    Serialize_s32(&Msg->Data[35], GyrValue.y, 4);
//    Serialize_s32(&Msg->Data[39], GyrValue.z, 4);
//  }
}

/**
 * @brief  Handles the MAG axes data getting/sending
 * @param  Msg the MAG part of the stream
 * @param  Instance the device instance
 * @retval None
 */
static void Magneto_Sensor_Handler(TMsg *Msg, uint32_t Instance)
{
  float ans_float;
#if ((defined (USE_STM32F4XX_NUCLEO)) || (defined (USE_STM32L4XX_NUCLEO)) || (defined (USE_STM32L1XX_NUCLEO)))
  MFX_MagCal_input_t mag_data_in;
  MFX_MagCal_output_t mag_data_out;
#elif (defined (USE_STM32L0XX_NUCLEO))
  MFX_CM0P_MagCal_input_t mag_data_in;
  MFX_CM0P_MagCal_output_t mag_data_out;
#else
#error Not supported platform
#endif

//  if ((SensorsEnabled & MAGNETIC_SENSOR) == MAGNETIC_SENSOR)
//  {
//    (void)IKS01A2_MOTION_SENSOR_GetAxes(Instance, MOTION_MAGNETO, &MagValue);

	uint8_t data[2] = {0};
	status = HAL_I2C_Mem_Read(&hi2c1, 0x3D, 0x68, I2C_MEMADD_SIZE_8BIT , data, 0x02, 100); 	
	int16_t a;
	a=(data[1] << 8)|data[0];
		MagValue.x = a;					
				
		 status = HAL_I2C_Mem_Read(&hi2c1, 0x3D, 0x6A, I2C_MEMADD_SIZE_8BIT , data, 0x02, 100);
	a=(data[1] << 8)|data[0];	
		MagValue.y = a;
				
		status = HAL_I2C_Mem_Read(&hi2c1, 0x3D, 0x6C, I2C_MEMADD_SIZE_8BIT , data, 0x02, 100); 
		a=(data[1] << 8)|data[0];
		MagValue.z =  a;
	
    if (MagCalStatus == 0U)
    {
      mag_data_in.mag[0] = (float)MagValue.x * FROM_MGAUSS_TO_UT50;
      mag_data_in.mag[1] = (float)MagValue.y * FROM_MGAUSS_TO_UT50;
      mag_data_in.mag[2] = (float)MagValue.z * FROM_MGAUSS_TO_UT50;

#if ((defined (USE_STM32F4XX_NUCLEO)) || (defined (USE_STM32L4XX_NUCLEO)) || (defined (USE_STM32L1XX_NUCLEO)))
      mag_data_in.time_stamp = (int)MagTimeStamp;
      MagTimeStamp += (uint32_t)ALGO_PERIOD;
#endif

      MotionFX_manager_MagCal_run(&mag_data_in, &mag_data_out);

#if ((defined (USE_STM32F4XX_NUCLEO)) || (defined (USE_STM32L4XX_NUCLEO)) || (defined (USE_STM32L1XX_NUCLEO)))
      if (mag_data_out.cal_quality == MFX_MAGCALGOOD)
#elif (defined (USE_STM32L0XX_NUCLEO))
      if (mag_data_out.cal_quality == MFX_CM0P_MAGCALGOOD)
#else
#error Not supported platform
#endif
      {
        MagCalStatus = 1;

        ans_float = (mag_data_out.hi_bias[0] * FROM_UT50_TO_MGAUSS);
        MagOffset.x = (int32_t)ans_float;
        ans_float = (mag_data_out.hi_bias[1] * FROM_UT50_TO_MGAUSS);
        MagOffset.y = (int32_t)ans_float;
        ans_float = (mag_data_out.hi_bias[2] * FROM_UT50_TO_MGAUSS);
        MagOffset.z = (int32_t)ans_float;

        /* Disable magnetometer calibration */
        MotionFX_manager_MagCal_stop(ALGO_PERIOD);
      }
    }

    MagValue.x = (int32_t)(MagValue.x - MagOffset.x);
    MagValue.y = (int32_t)(MagValue.y - MagOffset.y);
    MagValue.z = (int32_t)(MagValue.z - MagOffset.z);

//    Serialize_s32(&Msg->Data[43], MagValue.x, 4);
//    Serialize_s32(&Msg->Data[47], MagValue.y, 4);
//    Serialize_s32(&Msg->Data[51], MagValue.z, 4);
//  }
}

/**
 * @brief  Handles the PRESS sensor data getting/sending.
 * @param  Msg the PRESS part of the stream
 * @param  Instance the device instance
 * @retval None
 */
//static void Pressure_Sensor_Handler(TMsg *Msg, uint32_t Instance)
//{
//  float press_value;

//  if ((SensorsEnabled & PRESSURE_SENSOR) == PRESSURE_SENSOR)
//  {
//    (void)IKS01A2_ENV_SENSOR_GetValue(Instance, ENV_PRESSURE, &press_value);
//    (void)memcpy(&Msg->Data[7], (void *)&press_value, sizeof(float));
//  }
//}

///**
// * @brief  Handles the TEMP axes data getting/sending
// * @param  Msg the TEMP part of the stream
// * @param  Instance the device instance
// * @retval None
// */
//static void Temperature_Sensor_Handler(TMsg *Msg, uint32_t Instance)
//{
//  float temp_value;

//  if ((SensorsEnabled & TEMPERATURE_SENSOR) == TEMPERATURE_SENSOR)
//  {
//    (void)IKS01A2_ENV_SENSOR_GetValue(Instance, ENV_TEMPERATURE, &temp_value);
//    (void)memcpy(&Msg->Data[11], (void *)&temp_value, sizeof(float));
//  }
//}

///**
// * @brief  Handles the HUM axes data getting/sending
// * @param  Msg the HUM part of the stream
// * @param  Instance the device instance
// * @retval None
// */
//static void Humidity_Sensor_Handler(TMsg *Msg, uint32_t Instance)
//{
//  float hum_value;

//  if ((SensorsEnabled & HUMIDITY_SENSOR) == HUMIDITY_SENSOR)
//  {
//    (void)IKS01A2_ENV_SENSOR_GetValue(Instance, ENV_HUMIDITY, &hum_value);
//    (void)memcpy(&Msg->Data[15], (void *)&hum_value, sizeof(float));;
//  }
//}

/**
 * @brief  Configures the RTC
 * @param  None
 * @retval None
 */
static void RTC_Config(void)
{
  /*##-1- Configure the RTC peripheral #######################################*/
  /* Check if LSE can be used */
  RCC_OscInitTypeDef rcc_osc_init_struct;

  /*##-2- Configure LSE as RTC clock soucre ###################################*/
  rcc_osc_init_struct.OscillatorType =  RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_LSE;
  rcc_osc_init_struct.PLL.PLLState = RCC_PLL_NONE;
  rcc_osc_init_struct.LSEState = RCC_LSE_ON;
  rcc_osc_init_struct.LSIState = RCC_LSI_OFF;
  if (HAL_RCC_OscConfig(&rcc_osc_init_struct) != HAL_OK)
  {
    /* LSE not available, we use LSI */
    UseLSI = 1;
    RtcHandle.Init.AsynchPrediv = RTC_ASYNCH_PREDIV_LSI;
    RtcHandle.Init.SynchPrediv = RTC_SYNCH_PREDIV_LSI;
    RtcSynchPrediv = RTC_SYNCH_PREDIV_LSI;
  }
  else
  {
    /* We use LSE */
    UseLSI = 0;
    RtcHandle.Init.AsynchPrediv = RTC_ASYNCH_PREDIV_LSE;
    RtcHandle.Init.SynchPrediv = RTC_SYNCH_PREDIV_LSE;
    RtcSynchPrediv = RTC_SYNCH_PREDIV_LSE;
  }
  RtcHandle.Instance = RTC;

  /* Configure RTC prescaler and RTC data registers */
  /* RTC configured as follow:
       - Hour Format    = Format 12
       - Asynch Prediv  = Value according to source clock
       - Synch Prediv   = Value according to source clock
       - OutPut         = Output Disable
       - OutPutPolarity = High Polarity
       - OutPutType     = Open Drain
   */
  RtcHandle.Init.HourFormat     = RTC_HOURFORMAT_12;
  RtcHandle.Init.OutPut         = RTC_OUTPUT_DISABLE;
  RtcHandle.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  RtcHandle.Init.OutPutType     = RTC_OUTPUT_TYPE_OPENDRAIN;

  if (HAL_RTC_Init(&RtcHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
}

/**
 * @brief  Configures the current time and date
 * @param  None
 * @retval None
 */
static void RTC_TimeStampConfig(void)
{
  RTC_DateTypeDef sdatestructure;
  RTC_TimeTypeDef stimestructure;

  /* Configure the Date */
  /* Set Date: Monday January 1st 2001 */
  sdatestructure.Year = 0x01;
  sdatestructure.Month = RTC_MONTH_JANUARY;
  sdatestructure.Date = 0x01;
  sdatestructure.WeekDay = RTC_WEEKDAY_MONDAY;

  if (HAL_RTC_SetDate(&RtcHandle, &sdatestructure, FORMAT_BCD) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  /* Configure the Time */
  /* Set Time: 00:00:00 */
  stimestructure.Hours = 0x00;
  stimestructure.Minutes = 0x00;
  stimestructure.Seconds = 0x00;
  stimestructure.TimeFormat = RTC_HOURFORMAT12_AM;
  stimestructure.DayLightSaving = RTC_DAYLIGHTSAVING_NONE ;
  stimestructure.StoreOperation = RTC_STOREOPERATION_RESET;

  if (HAL_RTC_SetTime(&RtcHandle, &stimestructure, FORMAT_BCD) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
}

/**
 * @brief  Configures the current date
 * @param  y the year value to be set
 * @param  m the month value to be set
 * @param  d the day value to be set
 * @param  dw the day-week value to be set
 * @retval None
 */
void RTC_DateRegulate(uint8_t y, uint8_t m, uint8_t d, uint8_t dw)
{
  RTC_DateTypeDef sdatestructure;

  sdatestructure.Year = y;
  sdatestructure.Month = m;
  sdatestructure.Date = d;
  sdatestructure.WeekDay = dw;

  if (HAL_RTC_SetDate(&RtcHandle, &sdatestructure, FORMAT_BIN) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
}

/**
 * @brief  Configures the current time
 * @param  hh the hour value to be set
 * @param  mm the minute value to be set
 * @param  ss the second value to be set
 * @retval None
 */
void RTC_TimeRegulate(uint8_t hh, uint8_t mm, uint8_t ss)
{
  RTC_TimeTypeDef stimestructure;

  stimestructure.TimeFormat = RTC_HOURFORMAT12_AM;
  stimestructure.Hours = hh;
  stimestructure.Minutes = mm;
  stimestructure.Seconds = ss;
  stimestructure.SubSeconds = 0;
  stimestructure.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  stimestructure.StoreOperation = RTC_STOREOPERATION_RESET;

  if (HAL_RTC_SetTime(&RtcHandle, &stimestructure, FORMAT_BIN) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
}

/**
 * @brief  This function is executed in case of error occurrence
 * @param  None
 * @retval None
 */
void Error_Handler(void)
{
  for (;;)
  {
    BSP_LED_On(LED2);
    HAL_Delay(100);
    BSP_LED_Off(LED2);
    HAL_Delay(100);
  }
}

/**
 * @brief  EXTI line detection callbacks
 * @param  GPIOPin the pin connected to EXTI line
 * @retval None
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIOPin)
{
  if (GPIOPin == KEY_BUTTON_PIN)
  {
    if (BSP_PB_GetState(BUTTON_KEY) == (uint32_t)GPIO_PIN_RESET)
    {
      MagCalRequest = 1;
    }
  }
}

/**
 * @brief  Period elapsed callback
 * @param  htim pointer to a TIM_HandleTypeDef structure that contains
 *              the configuration information for TIM module.
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM_ALGO)
  {
    SensorReadRequest = 1;
  }
}

#ifdef  USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number where the assert_param error has occurred
 * @param  file pointer to the source file name
 * @param  line assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  for (;;)
  {}
}
#endif

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
