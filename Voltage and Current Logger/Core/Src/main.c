/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"

char str[16];

int dly_on_time  = 3000;//ms
int dly_off_time = 3000;//ms


#define ACS712_CHANNEL 4
#define left  1
#define right 2
#define stop  3

#define left_dirction   1
#define right_dirction  2

uint8_t motor_dirction;
uint8_t motor_state;
uint8_t motor_state_old;

uint8_t left_dirction_old;
uint8_t right_dirction_old;



//----------- current sensor -------------------------------------
char buffer[290];
int sensitivity = 100;//100 for20A ,  66 for30A   , 185 for 5 A
int offsetVoltage = 1720;
float current=0.0;
float milivoltage=0.0;

//----------------------------------------------------------------

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

#include "stdio.h"

#ifdef __GNUC__
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
  #else
  #define PUTCHAR_PROTOTYPE int fputc(int ch,FILE *f)
  #endif

  PUTCHAR_PROTOTYPE
  {//                        "SALAM"       5 100
  HAL_UART_Transmit(&huart1,(uint8_t*)&ch,1,100);

    return ch;

  }

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
  int read_adc1(uint8_t ch);
  void motor_fanction(uint8_t dir);
  void left_step(uint8_t num);
  void right_step(uint8_t num);
  void get_current_and_send();



  float get_val=0;


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

int sec=0;
bool left_set_counter_flag=false;
uint8_t left_occord_interrupt_counter=0;
bool left_occord_interrupt=false;

bool right_occord_interrupt=false;
bool right_set_counter_flag=false;
uint8_t right_occord_interrupt_counter=0;


bool    motor_run_occord_interrupt=false;
bool    motor_run_set_counter_flag=false;
uint8_t motor_run_occord_interrupt_counter=0;


  void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef *htim)
  {

  	if(htim->Instance==TIM2)//interrupt evry 1000ms
  	{

  		//--------------------------------------------------------------------------------------------
  		if(left_set_counter_flag)
  		{
  			    led_toggle;

				left_occord_interrupt_counter++;

				if((left_occord_interrupt_counter % 2 ) ==0)//if sec= 2,4,6
				{
					motor_fanction(stop);
				}
				else//if sec= 1,3,5 // first run
				{
					motor_fanction(left);
				}


				if(left_occord_interrupt_counter==6)//end of motor
				{
					sec=0;
					left_occord_interrupt_counter=0;
					left_set_counter_flag=false;
				}


  		 }


  		//--------------------------------------------- right fanction -------------------------

  		if(right_set_counter_flag)
  	    {
			   led_toggle;
				right_occord_interrupt_counter++;

				if((right_occord_interrupt_counter % 2 ) ==0)//if sec= 2,4,6
				{
					motor_fanction(stop);
				}
				else//if sec= 1,3,5 // first run
				{
					motor_fanction(right);
				}


				if(right_occord_interrupt_counter==6)//end of motor
				{
					sec=0;
					right_occord_interrupt_counter=0;
					right_set_counter_flag=false;
				}


		 }

  		//------------------------------------------------------------------------------------------------------


  		if(motor_run_set_counter_flag)
  		  		{
  		  			    led_toggle;

  		  			    motor_run_occord_interrupt_counter++;

  						if((motor_run_occord_interrupt_counter % 2 ) ==0)//if sec= 2,4,6
  						{
  							motor_fanction(right);
  						}
  						else//if sec= 1,3,5 // first run
  						{
  							motor_fanction(left);
  						}


  						if(motor_run_occord_interrupt_counter==12)//end of motor 10 operation
  						{

  							led_OFF;
  							motor_run_occord_interrupt_counter=0;
  							motor_run_set_counter_flag=false;
  						}


  		  		 }

  		//get_val=read_adc1(ACS712_CHANNEL);
  		//printf("%d\n",get_val);


  	}

  }



//  void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
//  {
//    if(GPIO_Pin == GPIO_PIN_7)
//    {
//         /* Toggle LED  */
//    	led_OFF;
//    	motor_fanction(stop);
//    	sec=0;
//		left_occord_interrupt_counter=0;
//		left_set_counter_flag=false;
//		right_occord_interrupt_counter=0;
//		right_set_counter_flag=false;
//
//    }
//  }



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */



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
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

 HAL_TIM_Base_Start_IT(&htim2);



// relay1_on;
// relay2_on;
// HAL_Delay(500);
// relay1_off;
// relay2_off;

 offsetVoltage = read_adc1(ACS712_CHANNEL);;



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {


		if(!left_btn)
		{
			//------------ stop auto motor------------------------------------
			motor_fanction(stop);
			motor_run_occord_interrupt_counter=0;
			motor_run_set_counter_flag=false;
			//--------------------------------------------------

			led_ON;
			motor_fanction(left);

			while(!left_btn)
			{
				get_current_and_send();
			}
			motor_fanction(stop);
			led_OFF;
		}




		if(!right_btn)
		{
			//------------ stop auto motor------------------------------------
			motor_fanction(stop);
			motor_run_occord_interrupt_counter=0;
			motor_run_set_counter_flag=false;
			//--------------------------------------------------

			led_ON;
			motor_fanction(right);
			while(!right_btn)
			{
				get_current_and_send();
			}
			motor_fanction(stop);
			led_OFF;
		}

		if(!stop_btn){
			motor_run_set_counter_flag=true;
		}



		get_current_and_send();



//		switch(motor_state)
//		{
//
//		case left_dirction:
//			if(motor_state!=motor_state_old)
//			 {
//				left_set_counter_flag=true;
//
//			    motor_state_old=motor_state;
//			 }
//
//			break;
//
//		case right_dirction:
//			if(motor_state!=motor_state_old)
//			 {
//				right_set_counter_flag=true;
//
//			    motor_state_old=motor_state;
//			 }
//
//			break;
//		}



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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */


void get_current_and_send()
{
//    char buff[20];
//	int sensitivity = 66;
 	int adcValue= 0;
	int offsetVoltage = 2500;
	float adcVoltage = 0;
	float currentValue = 0;

//	for(int i=0 ; i<10 ; i++)
//	{
//		adcValue+=read_adc1(ACS712_CHANNEL);
//
//	}
//
//	adcValue=adcValue/10;



	adcValue=read_adc1(ACS712_CHANNEL);
	//printf("%d\n",adcValue);


	adcVoltage=(adcValue*4096.0)/3300;
	currentValue = ((adcVoltage - (float)offsetVoltage) / (float)sensitivity);


//	if(currentValue<0)currentValue*=(-1);

//	milivoltage= (float)(  (((float)adcValue) / (4096.0)) * 3300.0) ;
//	sensor1_current = (((float)milivoltage - (float)offsetVoltage) / (float)sensitivity);

//	sprintf(buff,"%4.2f",currentValue);
//	printf(buff);
//	printf("\n");
	printf("%04.2f\n",currentValue);
    HAL_Delay(5);

}




int read_adc1(uint8_t ch)
{

    int raw_val=0;

	if(ch==ACS712_CHANNEL)
	{

		hadc1.Init.NbrOfDiscConversion=2;
		HAL_ADC_Init(&hadc1);
		HAL_ADC_Start(&hadc1);
		 HAL_Delay(5);

		//while( HAL_ADC_PollForConversion(&hadc1,100)!=HAL_OK ){}


				if(HAL_ADC_PollForConversion(&hadc1,50)==HAL_OK)
				{
				raw_val=HAL_ADC_GetValue(&hadc1);
	//					sprintf(str,"%f",value);
	//					//HAL_UART_Transmit(&huart1,(uint8_t*)"value:\n",5,20000);
	//					HAL_UART_Transmit(&huart1,(uint8_t*)str,4,20000);
	//					HAL_UART_Transmit(&huart1,(uint8_t*)"\t",5,20000);
				//AL_Delay(10);



				}
		HAL_ADC_Stop(&hadc1);

	}

return raw_val;

}

void motor_fanction(uint8_t dir)
{

	//left relay 1

		if(dir==right)
		{
			relay1_off;
			relay1_off;
			relay1_off;


			relay2_on;

		}
		else if(dir==left)
		{

			relay2_off;
			relay2_off;
			relay2_off;

			relay1_on;


		}
		else if(dir==stop)
		{
			relay1_off;
			relay2_off;
		}

}


void left_step(uint8_t num)
{
	for(int i=0 ; i<num ; i++)
	{
	   motor_fanction(left);
	   HAL_Delay(dly_on_time);
	   motor_fanction(stop);
	   HAL_Delay(dly_off_time);
	}

}

void right_step(uint8_t num)
{
	for(int i=0 ; i<num ; i++)
	{
	   motor_fanction(right);
	   HAL_Delay(dly_on_time);
	   motor_fanction(stop);
	   HAL_Delay(dly_off_time);
	}

}



/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
