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

// nuty - oktawa podstawowa
#define NOTE_C4  261.63f
#define NOTE_D4  293.66f
#define NOTE_E4  329.63f
#define NOTE_F4  349.23f
#define NOTE_G4  392.00f
#define NOTE_A4  440.00f
#define NOTE_B4  493.88f
#define NOTE_C5  523.25f

// podstawowa oktawa do testow
float scale[] = { NOTE_C4, NOTE_D4, NOTE_E4, NOTE_F4, NOTE_G4, NOTE_A4, NOTE_B4, NOTE_C5 };

// zmienne syntezatora
volatile float current_t = 10.0f;     // czas trwania jednej nuty
volatile float current_phase = 0.0f;  // faza sinusa
volatile float current_phase_detune = 0.0f; // rozstrojenie dla naturalnosci
volatile float current_freq = 261.63f; // czestotliwosc

int melody_step = 0;
uint32_t last_note_time = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
/* USER CODE BEGIN PFP */

static void Playback_Init(void); // prototyp funkcji do inicjalizacji audio
void GenerateSoundToBuffer(void); // prototyp funkcji generujacej dzwieki
void PlayNote(float freq); // funkcja wywolujaca nowa nute

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

  //uint32_t PlaybackPosition = PLAY_BUFF_SIZE + PLAY_HEADER; // pozycja odtwarzacza = rozmiar bufora + rozmiar naglowka wav

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
	  GenerateSoundToBuffer();

	  // 2. Prosty sekwencer - zmiana nuty co 500ms
	    if (HAL_GetTick() - last_note_time > 500)
	    {
	        // Graj kolejną nutę z tablicy scale[]
	        PlayNote(scale[melody_step]);

	        melody_step++;
	        if (melody_step >= 8) melody_step = 0; // Wróć do C4

	        last_note_time = HAL_GetTick();
	    }

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
  if(0 != audio_drv->Init(AUDIO_I2C_ADDRESS, OUTPUT_DEVICE_HEADPHONE, 80, AUDIO_FREQUENCY_44K)) //zmieniona czestotliwosc na 44k
  {
    Error_Handler();
  }
}

void GenerateSoundToBuffer(void)
{
    const float sample_rate = 44100.0f; // czestotliwosc probkowania
    const float amplitude = 28000.0f;  // glosnosc,  max 32767 dla int16
    const float phase_advance = 2.0f * M_PI * current_freq / sample_rate; // krok zmiany fazy, 2pi*f/fs
    const float phase_advance_detune = phase_advance * 1.0015f; // analogicznie dla drugiej (minimalnie roztrojonej) struny

    float stereo = 0.3f + (current_freq / 1000.0f);
        if (stereo < 0.2f) stereo = 0.2f;
        if (stereo > 0.8f) stereo = 0.8f;
        float vol_left = 1.0f - stereo;
        float vol_right = stereo;

    while (UpdatePointer == -1) { } // obsluga buforowania
    int pos = UpdatePointer;  // 0 or PLAY_BUFF_SIZE/2
    UpdatePointer = -1;

    for (int i = 0; i < PLAY_BUFF_SIZE/2; i += 2) // skok o dwa - raz lewy raz prawy kanal
    {
    	if (current_t > 3.5f) { // wyciszenie nuty po 3,5s
    	            PlayBuff[pos + i] = 0;
    	            PlayBuff[pos + i + 1] = 0;
    	            continue;
    	        }
    	 float fm_intensity = 1.2f * expf(-current_t * 12.0f); // eksponencjalne znieksztalcenie dzwieku
    	 float s1 = sinf(current_phase + fm_intensity * sinf(current_phase)); // charakterystyka aktualnego dzwieku (modulacja fm)
    	 float s2 = sinf(current_phase_detune); // rozstrojona struna w tle, dla realizmu

    	 float hammer = 0.0f; // imitacja uderzenia mloteczka w strune - dla realizmu
    	 if(current_t < 0.02f) // dziala tylko przez pierwsze 0,02s
    		 hammer = 0.15f * sinf(current_phase * 0.5f); // dzwiek o pol oktawy nizszy niz grana nuta


    	 float fade_rate = 0.6f + (current_freq / 800.0f); // wyliczenie zanikania drgan - szybsze dla wyzszych dzwiekow
    	 float vol_fade = expf(-current_t * fade_rate);
    	 float mixed_signal = (0.45f * s1 + 0.55f * s2) * vol_fade + hammer; // finalny sygnal wyjsciowy

    	 if (mixed_signal > 1.0f) mixed_signal = 1.0f; // normowanie
    	 if (mixed_signal < -1.0f) mixed_signal = -1.0f;

         int16_t sample = (int16_t)(amplitude * mixed_signal); // rzutowanie

         PlayBuff[pos + i]     = (int16_t)(sample * vol_left); // stereo lewa
         PlayBuff[pos + i + 1] = (int16_t)(sample * vol_right); // stereo prawa

         current_phase += phase_advance; // przejscie do kolejnego kata fazy
         if (current_phase >= 2.0f * M_PI)
        	 current_phase -= 2.0f * M_PI;
         current_phase_detune += phase_advance_detune;
         if (current_phase_detune >= 2.0f * M_PI)
        	 current_phase_detune -= 2.0f * M_PI;

         current_t += 1.0f / sample_rate; // zanikanie dzwieku w czasie
    }
}

void PlayNote(float freq)
{
    current_freq = freq;
    current_t = 0.0f;     // Reset czasu -> start dźwięku (uderzenie młotka)
    current_phase = 0.0f; // Reset fazy -> czysty start fali (opcjonalne, ale zalecane dla perkusyjności)
    current_phase_detune = 0.0f;
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
