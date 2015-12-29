/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2015 STMicroelectronics
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
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */
#include "pin.h"
#include "sdram.h"
#include "ili9341.h"
//#include "graph.h"
#include "l3gd20.h"
#include "itg3200.h"
#include "hmc5883.h"
#include "adxl345.h"
#include "compas.h"
#include "clinometer.h"
#include "guage.h"
#include "math.h"
#include "ff.h"
#include "image.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

DMA2D_HandleTypeDef hdma2d;

I2C_HandleTypeDef hi2c3;

LTDC_HandleTypeDef hltdc;

SPI_HandleTypeDef hspi5;

UART_HandleTypeDef huart5;

SDRAM_HandleTypeDef hsdram1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint8_t flagButton;
char str[] = "1.0.1";
char bufStr[50] = "";
const char *directions[17]={"N","NNE","NE","ENE",
														"E","ESE","SE","SSE",
														"S","SSW","SW","WSW",
														"W","WNW","NW","NNW","N"};

__IO uint32_t TM_Time2;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_DMA2D_Init(void);
static void MX_FMC_Init(void);
static void MX_I2C3_Init(void);
static void MX_LTDC_Init(void);
static void MX_SPI5_Init(void);
static void MX_UART5_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
	uint8_t mode = 0, r, temp;
	uint8_t firsttime = 0, tosave = 0;
	uint32_t cur_tick, tmp_tick;
	//L3GD20_t lxyz;
	uint32_t i;
	XYZ_t ixyz, txyz;
	HMC_ID_StructTypeDef hres;
	int32_t tmp;
	int32_t tmpV, clV, valV;
	float angl;
	//uint16_t i;
  uint32_t adcData[5];
	
	FATFS FS;
	FRESULT fres;
	FATFS *fs;
	uint32_t Total; /*!< Total size of memory */
	uint32_t Free;  /*!< Free size of memory */

	DWORD fre_clust;

	DIR dir;
	FILINFO fno;

	FIL fil;
	UINT nWritten;
	UINT toWrite;
	UINT totalWrite;
	UINT totalWriten;
	
	BITMAPFILEHEADER bfh;
	BITMAPINFOHEADER bih;

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_DMA2D_Init();
  MX_FMC_Init();
  MX_I2C3_Init();
  MX_LTDC_Init();
  MX_SPI5_Init();
  MX_UART5_Init();

  /* USER CODE BEGIN 2 */
	GPIO_HIGH(L3GD20_CS_PORT, L3GD20_CS_PIN);
	GPIO_HIGH(GPIOF, GPIO_PIN_6);//FATFS_CS_PORT, FATFS_CS_PIN

	//SPI_Init_Pins();
	
	SDRAM_InitPins();
 	SDRAM_Init(&hsdram1);
	//if (SDRAM_Init(&hsdram1)) GPIO_TOGGLE(GPIOG, GPIO_PIN_14);

	ILI9341_InitPins();
	ILI9341_Init();
	HAL_LTDC_SetAlpha(&hltdc, 255, 0);
	HAL_LTDC_SetAlpha(&hltdc, 0, 1);
	
	GRPH_Init();
	DMA2DGRPH_Fill(); 

	GRPH_DrawRect(0, 0, 239, 319);
//	GRPH_DrawLine(0,0,239,319);
//	GRPH_DrawLine(239,0,0,319);
//	GRPH_DrawCircle(120,160,60);
	
	DrawCompas(22, 297, 20, 0, 0);
	DrawClinometer(64, 297, 20, 0, 0);
	DrawGuageAbsolut(88, 297, 20, 100, 0, GRPH_COLOR_RED);	
	
	GRPH_SetXY(200, 309);
	GRPH_Puts(str);
	
	HMC_Init(HMC_MR_CMM, HMC_DR_15, HMC_MM_Normal, HMC_GS_4_0, HMC_SS_1);
	L3GD20_Init(L3GD20_Scale_250);
	ITG_Init(ITG_DLPF_20_1, 0, ITG_CLK_GyroX);
	ADXL_Init();

	//HAL_ADC_Start(&hadc1);
	HAL_ADC_Start_DMA(&hadc1, adcData, 4);
	//ITG_SetChipAddr(0xd0);
	
//	if (f_mount(&FS, "0", 1) == FR_OK) {
//			fres = f_opendir(&dir, "/");
//			f_mount(NULL, "0", 1);
//	}
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

		
		cur_tick = HAL_GetTick();
		if (cur_tick - tmp_tick > 200){
			
			if (mode == 0 || mode == 1){
				
				HAL_ADC_Start_DMA(&hadc1, adcData, 5);
				//HAL_ADC_GetValue(&hadc1);

				r = L3GD20_INT_ReadSPI(L3GD20_REG_WHO_AM_I);
				L3GD20_Read(&ixyz);
				sprintf(bufStr, "%02x %6d; %6d; %6d;", r, ixyz.X, ixyz.Y, ixyz.Z);		
				//HAL_UART_Transmit(&huart5, (uint8_t *) bufStr, 50, 100);
				GRPH_SetXY(2, 3);
				GRPH_Puts(bufStr);

				r = ITG_GetChipAddr();
				ITG_GetXYZ(&ixyz);
				temp = ITG_GetTemperature();
				temp = ITG_ConvTemp(temp);
				sprintf(bufStr, "%02x %6d; %6d; %6d; %3d", r, ixyz.X, ixyz.Y, ixyz.Z, temp);		
				GRPH_SetXY(2, 14);
				GRPH_Puts(bufStr);
				
				//GRPH_DrawPixel(80 + ixyz.X, 100 + ixyz.Y);
				
				HMC_GetId(&hres);
				HMC_GetXYZ(&ixyz);
				sprintf(bufStr, "%c%c%c %5d; %6d; %6d", hres.VALA, hres.VALB, hres.VALC, ixyz.X, ixyz.Y, ixyz.Z);		
				GRPH_SetXY(2, 26);
				GRPH_Puts(bufStr);

				//DrawCompas(22, 297, 20, hxyz.X, hxyz.Y);
				DrawCompas(52, 100, 50, ixyz.X, ixyz.Y);

				angl = 270 + atan2(ixyz.Y, ixyz.X) * 180 /3.14;
				if (angl > 360) angl -= 360;
				tmp = (angl + 11)/22.5;
				sprintf(bufStr, "%4.1f %s      ", angl, directions[tmp]);		
				GRPH_SetXY(2, 154);
				GRPH_Puts(bufStr);

				
				DrawGuage(107, 100, 50, ixyz.X, txyz.X, GRPH_COLOR_GREEN);	
				DrawGuage(114, 100, 50, ixyz.Y, txyz.Y, GRPH_COLOR_BLUE);	
				DrawGuage(121, 100, 50, ixyz.Z, txyz.Z, GRPH_COLOR_YELLOW);	
				txyz.X = ixyz.X;
				txyz.Y = ixyz.Y;
				txyz.Z = ixyz.Z;
				
				r = ADXL_GetDeviceId();
				ADXL_GetXYZ(&ixyz);
				sprintf(bufStr, "%x %6d; %6d; %6d", r, ixyz.X, ixyz.Y, ixyz.Z);		
				GRPH_SetXY(2, 38);
				GRPH_Puts(bufStr);
				//DrawClinometer(64, 297, 20, axyz.Y, 0);
				
				DrawClinometer(176, 100, 50, ixyz.X, ixyz.Y);
				
				sprintf(bufStr, "%4x; %4x; %4x; %4x; %4x", adcData[0], adcData[1], adcData[2], adcData[3], adcData[4]);
				//sprintf(bufStr, "%2.1fV %x     ", (3.3*(adcData[0]+adcData[1]+adcData[2]))/0x0fff/3, (adcData[0]+adcData[1]+adcData[2])/3);		
				GRPH_SetXY(2, 166);
				GRPH_Puts(bufStr);
		
				//sprintf(bufStr, "%4x; %4x; %4x; %4x; %4x", adcData[0], adcData[1], adcData[2], adcData[3], adcData[4]);
				sprintf(bufStr, "%2.1fV %2.1fV     ", (3.3*(adcData[0]+adcData[1]+adcData[2]))/0x0fff/3, (3.3*(adcData[3]+adcData[4]))/0x0fff/2);		
				GRPH_SetXY(2, 178);
				GRPH_Puts(bufStr);
		
				if (adcData[0]> 0x500) clV = GRPH_COLOR_GREEN;
				else  clV = GRPH_COLOR_RED;
				valV = (adcData[0]+adcData[1]+adcData[2])/0x0f/3;			
				DrawBattery(220, 40, 10, valV, tmpV, clV);
				tmpV = valV;

				HAL_ADC_Stop_DMA(&hadc1);
					
				
				if (tosave == 1){
//					fres = img_save();
					//fres = f_mount(&FS, "SD:", 1);
					totalWrite = LCD_PIXEL_WIDTH * LCD_PIXEL_HEIGHT * LCD_PIXEL_SIZE;
					
					GPIO_TOGGLE(GPIOG, GPIO_PIN_14);
					
					//memset (&bfh, 0, sizeof(bfh));
					bfh.bfType = 0x4D42;                           
					bfh.bfOffBits = sizeof(bfh) + sizeof(bih);    
					bfh.bfSize = bfh.bfOffBits + totalWrite;    
					//memset (&bih, 0, sizeof(bih));
					bih.biSize = sizeof(bih);                      
					bih.biBitCount = 16;                           
					bih.biClrUsed = 0;                         
					bih.biCompression = 0;                    
					bih.biHeight = LCD_PIXEL_HEIGHT;
					bih.biWidth = LCD_PIXEL_WIDTH;
					bih.biPlanes = 1;
						//sprintf(bufStr, "pointer  %u", (uint32_t) &bfh);
						//GRPH_SetXY(2, 205);
						//GRPH_Puts(bufStr);
					
					
					fres = f_mount(&FS, "0", 1);
					if (fres == FR_OK) {
						//fres = img_save();

						//fres = f_open(&fil, "img.raw", FA_OPEN_ALWAYS | FA_READ | FA_WRITE);
						fres = f_open(&fil, "img.bmp", FA_OPEN_ALWAYS | FA_READ | FA_WRITE);
						if (fres == FR_OK) {

							//f_write(&fil, (uint8_t *) &bfh, sizeof(bfh), &nWritten);
							//f_write(&fil, (uint8_t *)(&bih), sizeof(bih), &nWritten);
							
							nWritten = 0;
							totalWriten = 0;
							toWrite = totalWrite;
							while (totalWrite > totalWriten){			
								
								f_write(&fil, (uint8_t *)(LCD_FRAME_BUFFER + totalWriten), toWrite, &nWritten);
								toWrite -= nWritten;
								totalWriten += nWritten; 
								nWritten = 0;
							
							}
							f_close(&fil);			
						}
						
						f_mount(NULL, "0", 1);
					}
					if (fres == FR_OK) {
						sprintf(bufStr, "Write OK   ");
						GRPH_SetXY(2, 190);
						GRPH_Puts(bufStr);
					}else{
						sprintf(bufStr, "Error:  %u ", fres);
						GRPH_SetXY(2, 190);
						GRPH_Puts(bufStr);
					}
					//fres = img_bmp_save();
					tosave = 0;
				} 
				
			}
			
			i++;
			if (i >= 10) i=0;
			
			GPIO_TOGGLE(GPIOG, GPIO_PIN_13);
			tmp_tick = cur_tick;
		}
		
		
		if (firsttime && mode == 1) {
			
			tosave = 1;
			
		}

		if (firsttime && mode == 3) {
			
			//fres = img_read();
			if (f_mount(&FS, "0", 1) == FR_OK) {
					
					fres = f_open(&fil, "img.raw", FA_OPEN_EXISTING | FA_READ );
					if (fres == FR_OK) {

							f_read(&fil, (uint8_t *)(LCD_FRAME_BUFFER), LCD_PIXEL_WIDTH*LCD_PIXEL_WIDTH*LCD_PIXEL_SIZE, &nWritten);
							
							/* Close file */
							f_close(&fil);
							
					}
					
					/* Unmount SDCARD */
					f_mount(NULL, "SD:", 1);
			}
			if (fres == FR_OK) {
				sprintf(bufStr, "Read OK  ");
				GRPH_SetXY(15, 190);
				GRPH_Puts(bufStr);
					
			}else{
				sprintf(bufStr, "Error:  %u ", fres);
				GRPH_SetXY(15, 190);
				GRPH_Puts(bufStr);
			}
		}

		if (firsttime && mode == 2) {
			
			//if (f_mount(&FS, "SD:", 1) == FR_OK) {
			if (f_mount(&FS, "0", 1) == FR_OK) {

					if (f_getfree("SD:", &fre_clust, &fs) == FR_OK) {
							Total = (fs->n_fatent - 2) * fs->csize * 0.5;
							Free = fre_clust * fs->csize * 0.5;
					}
					
					sprintf(bufStr, "Total: %u kBytes", Total);
					GRPH_SetXY(2, 3);
					GRPH_Puts(bufStr);
					sprintf(bufStr, "Free:  %u kBytes", Free);
					GRPH_SetXY(2, 14);
					GRPH_Puts(bufStr);
					
					i = 2;
					//fres = f_opendir(&dir, "SD:");
					fres = f_opendir(&dir, "/");
					if (fres == FR_OK) {
						while ((fres = f_readdir(&dir, &fno)) == FR_OK && fno.fname[0] != 0) {
							sprintf(bufStr, "%s - %u", fno.fname, fno.fsize);
							GRPH_SetXY(2, 12 * i + 3);
							GRPH_Puts(bufStr);
							i++;
						}
						f_closedir(&dir);
					}else{
						sprintf(bufStr, "Error:  %u ", fres);
						GRPH_SetXY(2, 12 * i + 3);
						GRPH_Puts(bufStr);
						i++;
					}
					
					
					/* Unmount SDCARD */
					f_mount(NULL, "SD:", 1);
			}
		}


		firsttime = 0;
		if (flagButton == 1){
			mode++;
			firsttime = 1;
      GPIO_TOGGLE(GPIOG, GPIO_PIN_14);
			flagButton = 0;

			DMA2DGRPH_Fill();

			GRPH_DrawRect(0,0,239,319);

			GRPH_SetXY(200, 309);
			GRPH_Puts(str);
		}

		if (mode == 4) mode = 0;
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

  __PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1
                              |RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_LTDC;
  PeriphClkInitStruct.PLLSAI.PLLSAIN = 192;
  PeriphClkInitStruct.PLLSAI.PLLSAIR = 5;
  PeriphClkInitStruct.PLLSAIDivR = RCC_PLLSAIDIVR_4;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

}

/* ADC1 init function */
void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION12b;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 5;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = EOC_SINGLE_CONV;
  HAL_ADC_Init(&hadc1);

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Rank = 2;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Rank = 3;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

		/**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_VBAT;
  sConfig.Rank = 4;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Rank = 5;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

}

/* DMA2D init function */
void MX_DMA2D_Init(void)
{

  hdma2d.Instance = DMA2D;
  hdma2d.Init.Mode = DMA2D_R2M;
  hdma2d.Init.ColorMode = DMA2D_RGB565;
  hdma2d.Init.OutputOffset = 0;
  HAL_DMA2D_Init(&hdma2d);

}

/* I2C3 init function */
void MX_I2C3_Init(void)
{

  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLED;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLED;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLED;
  HAL_I2C_Init(&hi2c3);

}

/* LTDC init function */
void MX_LTDC_Init(void)
{

  LTDC_LayerCfgTypeDef pLayerCfg;
  LTDC_LayerCfgTypeDef pLayerCfg1;

  hltdc.Instance = LTDC;
  hltdc.Init.HSPolarity = LTDC_HSPOLARITY_AL;
  hltdc.Init.VSPolarity = LTDC_VSPOLARITY_AL;
  hltdc.Init.DEPolarity = LTDC_DEPOLARITY_AL;
  hltdc.Init.PCPolarity = LTDC_PCPOLARITY_IPC;
  hltdc.Init.HorizontalSync = 8;
  hltdc.Init.VerticalSync = 0;
  hltdc.Init.AccumulatedHBP = 37;
  hltdc.Init.AccumulatedVBP = 3;
  hltdc.Init.AccumulatedActiveW = 277;
  hltdc.Init.AccumulatedActiveH = 323;
  hltdc.Init.TotalWidth = 279;
  hltdc.Init.TotalHeigh = 325;
  hltdc.Init.Backcolor.Blue = 0;
  hltdc.Init.Backcolor.Green = 0;
  hltdc.Init.Backcolor.Red = 0;
  HAL_LTDC_Init(&hltdc);

  pLayerCfg.WindowX0 = 0;
  pLayerCfg.WindowX1 = 240;
  pLayerCfg.WindowY0 = 0;
  pLayerCfg.WindowY1 = 320;
  pLayerCfg.PixelFormat = LTDC_PIXEL_FORMAT_RGB565;
  pLayerCfg.Alpha = 255;
  pLayerCfg.Alpha0 = 0;
  pLayerCfg.BlendingFactor1 = LTDC_BLENDING_FACTOR1_PAxCA;
  pLayerCfg.BlendingFactor2 = LTDC_BLENDING_FACTOR2_PAxCA;
  pLayerCfg.FBStartAdress = 0xD0000000;
  pLayerCfg.ImageWidth = 240;
  pLayerCfg.ImageHeight = 320;
  pLayerCfg.Backcolor.Blue = 0;
  pLayerCfg.Backcolor.Green = 0;
  pLayerCfg.Backcolor.Red = 0;
  HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg, 0);

  pLayerCfg1.WindowX0 = 0;
  pLayerCfg1.WindowX1 = 240;
  pLayerCfg1.WindowY0 = 0;
  pLayerCfg1.WindowY1 = 320;
  pLayerCfg1.PixelFormat = LTDC_PIXEL_FORMAT_RGB565;
  pLayerCfg1.Alpha = 255;
  pLayerCfg1.Alpha0 = 0;
  pLayerCfg1.BlendingFactor1 = LTDC_BLENDING_FACTOR1_PAxCA;
  pLayerCfg1.BlendingFactor2 = LTDC_BLENDING_FACTOR2_PAxCA;
  pLayerCfg1.FBStartAdress = 0xD0025800;
  pLayerCfg1.ImageWidth = 240;
  pLayerCfg1.ImageHeight = 320;
  pLayerCfg1.Backcolor.Blue = 0;
  pLayerCfg1.Backcolor.Green = 0;
  pLayerCfg1.Backcolor.Red = 0;
  HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg1, 1);

}

/* SPI5 init function */
void MX_SPI5_Init(void)
{

  hspi5.Instance = SPI5;
  hspi5.Init.Mode = SPI_MODE_MASTER;
  hspi5.Init.Direction = SPI_DIRECTION_2LINES;
  hspi5.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi5.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi5.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi5.Init.NSS = SPI_NSS_SOFT;
  hspi5.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi5.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi5.Init.TIMode = SPI_TIMODE_DISABLED;
  hspi5.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLED;
  hspi5.Init.CRCPolynomial = 10;
  HAL_SPI_Init(&hspi5);

}

/* UART5 init function */
void MX_UART5_Init(void)
{

  huart5.Instance = UART5;
  huart5.Init.BaudRate = 9600;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart5);

}

/** 
  * Enable DMA controller clock
  */
void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}
/* FMC initialization function */
void MX_FMC_Init(void)
{
  FMC_SDRAM_TimingTypeDef SdramTiming;

  /** Perform the SDRAM1 memory initialization sequence
  */
  hsdram1.Instance = FMC_SDRAM_DEVICE;
  /* hsdram1.Init */
  hsdram1.Init.SDBank = FMC_SDRAM_BANK2;
  hsdram1.Init.ColumnBitsNumber = FMC_SDRAM_COLUMN_BITS_NUM_8;
  hsdram1.Init.RowBitsNumber = FMC_SDRAM_ROW_BITS_NUM_12;
  hsdram1.Init.MemoryDataWidth = FMC_SDRAM_MEM_BUS_WIDTH_16;
  hsdram1.Init.InternalBankNumber = FMC_SDRAM_INTERN_BANKS_NUM_4;
  hsdram1.Init.CASLatency = FMC_SDRAM_CAS_LATENCY_3;
  hsdram1.Init.WriteProtection = FMC_SDRAM_WRITE_PROTECTION_DISABLE;
  hsdram1.Init.SDClockPeriod = FMC_SDRAM_CLOCK_PERIOD_2;
  hsdram1.Init.ReadBurst = FMC_SDRAM_RBURST_DISABLE;
  hsdram1.Init.ReadPipeDelay = FMC_SDRAM_RPIPE_DELAY_0;
  /* SdramTiming */
  SdramTiming.LoadToActiveDelay = 2;
  SdramTiming.ExitSelfRefreshDelay = 7;
  SdramTiming.SelfRefreshTime = 4;
  SdramTiming.RowCycleDelay = 7;
  SdramTiming.WriteRecoveryTime = 3;
  SdramTiming.RPDelay = 2;
  SdramTiming.RCDDelay = 2;

  HAL_SDRAM_Init(&hsdram1, &SdramTiming);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __GPIOF_CLK_ENABLE();
  __GPIOH_CLK_ENABLE();
  __GPIOC_CLK_ENABLE();
  __GPIOA_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();
  __GPIOG_CLK_ENABLE();
  __GPIOE_CLK_ENABLE();
  __GPIOD_CLK_ENABLE();

  /*Configure GPIO pin : PC1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PG13 PG14 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : PF6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
