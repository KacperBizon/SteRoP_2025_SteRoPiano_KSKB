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
#include "dfsdm.h"
#include "dma.h"
#include "i2c.h"
#include "sai.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "stm32l476g_discovery_audio.h" // do obslugi funkcji audio
#include "stm32l476g_discovery.h"       // LED i inicjacja I2C
#include "math.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define AUDIO_FILE_ADDRESS   0x08080000 // adres pliku audio w pamieci, od niego zaczynac przy wgrywaniu
#define AUDIO_FILE_SIZE      (180*1024) // rozmiar pliku audio
#define PLAY_HEADER          0x2C       // rozmiar naglowka wav
#define PLAY_BUFF_SIZE       4096       // rozmiar bufora

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

extern SAI_HandleTypeDef hsai_BlockA1; // dostepy
extern DMA_HandleTypeDef hdma_sai1_a;

AUDIO_DrvTypeDef *audio_drv;          		// wskaznik na driver audio
uint16_t          PlayBuff[PLAY_BUFF_SIZE]; // bufor
__IO int16_t      UpdatePointer = -1;       // flaga do sprawdzania, czy transmizja DMA sie powiodla

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
/* USER CODE BEGIN PFP */

static void Playback_Init(void); // prototyp funkcji do inicjalizacji audio
void GenerateSoundToBuffer(float f);

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

  uint32_t PlaybackPosition = PLAY_BUFF_SIZE + PLAY_HEADER; // pozycja odtwarzacza = rozmiar bufora + rozmiar naglowka wav

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_SAI1_Init();
  MX_USB_DEVICE_Init();
  MX_DFSDM1_Init();
  /* USER CODE BEGIN 2 */

  BSP_LED_Init(LED5); // inicjalizacja led
  Playback_Init(); //inicjalizacja audio

  for(int i=0; i < PLAY_BUFF_SIZE; i+=2) // wypelnienie bufora danymi z pliku
  {
    PlayBuff[i/2] = *((__IO uint16_t *)(AUDIO_FILE_ADDRESS + PLAY_HEADER + i));
  }

  if(0 != audio_drv->Play(AUDIO_I2C_ADDRESS, NULL, 0)) // sprawdzenie czy AUDIO_I2C_ADDRESS zdefiniowane w BSP
  {
    Error_Handler();
  }


  if(HAL_OK != HAL_SAI_Transmit_DMA(&hsai_BlockA1, (uint8_t *)PlayBuff, PLAY_BUFF_SIZE)) // sprawdzenie czy DMA uruchomione poprawnie
  {
    Error_Handler();
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  BSP_LED_Toggle(LED5); // miganie zielona dioda - program dziala poprawnie
	  GenerateSoundToBuffer(440.0f);
//	    while(UpdatePointer == -1) //sprawdzenie czy transfer DMA w porzadku
//	    {
//	        // HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
//	    }
//
//	    int position = UpdatePointer; // sprawdzenie ktora polowa bufora wolna (0 lub PLAY_BUFF_SIZE/2)
//	    UpdatePointer = -1; // reset flagi
//
//	    for(int i = 0; i < PLAY_BUFF_SIZE/2; i++) // uzupelnienie wolnej polowy bufora danymi
//	    {
//	    	// odczyt danych z flash
//	        PlayBuff[i + position] = *((__IO uint16_t *)(AUDIO_FILE_ADDRESS + PlaybackPosition));
//	        PlaybackPosition += 2; // przesuniecie adresu o 2 bajty (bo uint16, a nie 8)
//	    }
//
//	    // sprawdzenie, czy koniec danych
//	    if((PlaybackPosition + (PLAY_BUFF_SIZE)) > AUDIO_FILE_SIZE) // jesli nastepna operacja przekroczy rozmiar pliku
//	    {
//	        PlaybackPosition = PLAY_HEADER; // wraca na poczatek pliku
//	    }

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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV4;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_SAI1|RCC_PERIPHCLK_USB;
  PeriphClkInit.Sai1ClockSelection = RCC_SAI1CLKSOURCE_PLLSAI1;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 24;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV17;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_SAI1CLK|RCC_PLLSAI1_48M2CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

static void Playback_Init(void) // funkcja odpowiedzialna za przetwarzanie audio
{
  __HAL_SAI_ENABLE(&hsai_BlockA1); // zezwolenie SAI na generowanie zegara bloku A1

  if(CS43L22_ID != cs43l22_drv.ReadID(AUDIO_I2C_ADDRESS)) //sprawdzenie inicjalizacji drivera audio
  {
    Error_Handler();
  }

  audio_drv = &cs43l22_drv;				// przypisanie wskaznika drivera
  audio_drv->Reset(AUDIO_I2C_ADDRESS);	// zresetowanie drivera audio

  // konfiguracja adresow I2C, urzadzenia wyjsciowego, glosnosci i czestotliwosci
  if(0 != audio_drv->Init(AUDIO_I2C_ADDRESS, OUTPUT_DEVICE_HEADPHONE, 80, AUDIO_FREQUENCY_22K))
  {
    Error_Handler();
  }
}

void GenerateSoundToBuffer(float f)
{
    const float sample_rate = 22050.0f;
    const float amplitude = 30000.0f;  // max 32767 for int16
    const float phase_advance = 2.0f * M_PI * f / sample_rate; // 2pi*f/fs

    static float phase = 0.0f;
    static float t = 0.0f;

    // Wait until DMA half/full complete flag is set
    while (UpdatePointer == -1) { }

    int pos = UpdatePointer;  // 0 or PLAY_BUFF_SIZE/2
    UpdatePointer = -1;

    for (int i = 0; i < PLAY_BUFF_SIZE/2; i += 2)
    {
    	float env_fundamental = expf(-t * 1.5f);
    	float env_harmonics = expf(-t * 6.0f);

    	//float env = 4.7*sqrt(t)*(1-t)*(1-t)*(1-t)*(1-t);
		//float env = expf(-15.0 * (t-0.5)*(t-0.5));
		//float env = expf(-t * 2.5f);   // exponential decay

		float s =
			  1.0f * sinf(phase) * env_fundamental     					// fundamental
			+ (0.55f * sinf(2.0f * phase)              					// 2nd harmonic
			+ 0.2f * sinf(3.0f * phase)                					// 3rd harmonic
			+ 0.18f * sinf(4.0f * phase)               					// 4th harmonic
			+ 0.15f * sinf(5.0f * phase)               					// 5th harmonic
			+ 0.05f * ((float)rand()/RAND_MAX - 0.5f)) * env_harmonics; // small random noise


          // Scale to 16-bit
          int16_t sample = (int16_t)(amplitude * s);

          // Advance phase/time
          phase += phase_advance;
          if (phase >= 2.0f * M_PI)
        	  phase -= 2.0f * M_PI;

          t += 1.0f / sample_rate;

          if (t > 1.5f)
        	  t = 0.0f;

          // Stereo output
          PlayBuff[pos + i]     = (int16_t)sample; // Left
          PlayBuff[pos + i + 1] = (int16_t)sample; // Right
    }
}


/**
  * @brief Tx Transfer completed callbacks.
  * @param  hsai : pointer to a SAI_HandleTypeDef structure that contains
  * the configuration information for SAI module.
  * @retval None
  */


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
