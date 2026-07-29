/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 * @author		   : Jeevan Shah
 ******************************************************************************
 */

#include "main.h"
// #include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <time.h>

typedef enum {
	MUX1 = 0, MUX2 = 1, MUX3 = 2
} mux_id_t;

typedef struct {
	int16_t min_temp;
	int16_t max_temp;
	int16_t avg_temp;
	uint8_t min_channel;
	uint8_t max_channel;
	uint8_t num_enabled;
} TempStatistics_t;

#define NUM_MUXES 3U
#define MUX_CHANNELS_PER_CHIP 32U
#define MUX_DISABLE_CMD 0x80U
#define DAQ_BASE_ID 0x18FF5000U
#define DAQ_EXT_ID_MAX 0x1FFFFFFFU
// #define USE_SEMIHOSTING			true
// #define SHOW_CHANNEL_TEMPS true

/* ---------------------------------------------------------------------------
 * Biquad IIR filter — 2nd-order Butterworth lowpass at 5 kHz
 * Generated for fs = 250 kHz (one ADC sample every ~4 us).
 * Attenuates 20 kHz EMI by ~48 dB while passing the DC/thermal signal.
 * Coefficients: scipy.signal.butter(2, 5000, fs=250000, output='sos')
 * --------------------------------------------------------------------------- */
#define BIQUAD_B0  0.0036216815
#define BIQUAD_B1  0.0072433630
#define BIQUAD_B2  0.0036216815
#define BIQUAD_A1 (-1.8226949252)
#define BIQUAD_A2  0.8371816513

/* One state per mux channel — persists across samples within a channel scan */
#define NUM_CHANNELS_TOTAL (NUM_MUXES * MUX_CHANNELS_PER_CHIP)

typedef struct {
	double w1;
	double w2;
} BiquadState;

static BiquadState biquad_state[NUM_CHANNELS_TOTAL];

/* Reset filter state for a given channel (call before scanning that channel) */
static void Biquad_Reset(uint8_t mux, uint8_t ch) {
	uint16_t idx = (uint16_t) mux * MUX_CHANNELS_PER_CHIP + ch;
	biquad_state[idx].w1 = 0.0;
	biquad_state[idx].w2 = 0.0;
}

/* Process one sample through the Direct Form II biquad for this channel */
static double Biquad_Process(uint8_t mux, uint8_t ch, double x) {
	uint16_t idx = (uint16_t) mux * MUX_CHANNELS_PER_CHIP + ch;
	BiquadState *s = &biquad_state[idx];
	double w0 = x - BIQUAD_A1 * s->w1 - BIQUAD_A2 * s->w2;
	double y = BIQUAD_B0 * w0 + BIQUAD_B1 * s->w1 + BIQUAD_B2 * s->w2;
	s->w2 = s->w1;
	s->w1 = w0;
	return y;
}

ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

static GPIO_TypeDef *const mux_sync_port[NUM_MUXES] = { GPIOB, GPIOB, GPIOB };

static CAN_TxHeaderTypeDef TxHeader;
static uint8_t TxData[8];
static uint32_t TxMailbox;

static const uint16_t mux_sync_pin[NUM_MUXES] = { GPIO_PIN_0, GPIO_PIN_1,
GPIO_PIN_2 };

// static const uint16_t mux_names[NUM_MUXES][MUX_CHANNELS_PER_CHIP] = {
//     {101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111,
//      112, 113, 114, 115, 116, 117, 118, 201, 202, 203, 204,
//      205, 206, 207, 208, 209, 210, 211, 212, 213, 214},
//     {215, 216, 217, 218, 301, 302, 303, 304, 305, 306, 307,
//      308, 309, 310, 311, 312, 313, 314, 315, 316, 317, 318,
//      401, 402, 403, 404, 405, 406, 407, 408, 409, 410},
//     {411, 412, 413, 414, 415, 416, 417, 418, 501, 502, 503,
//      504, 505, 506, 507, 508, 509, 510, 511, 512, 513, 514,
//      515, 516, 517, 518, 27,  28,  29,  30,  31,  32}};

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_CAN_Init(void);
static void MX_TIM4_Init(void);

#ifdef USE_SEMIHOSTING
extern void initialise_monitor_handles(void);
#endif

static void ADC1_SetLongSampleTime(void);
static HAL_StatusTypeDef MUX_WriteRaw(mux_id_t mux, uint8_t cmd);
static HAL_StatusTypeDef MUX_Disable(mux_id_t mux);
static void MUX_DisableAll(void);
static HAL_StatusTypeDef MUX_SelectChannel(mux_id_t mux, uint8_t channel);
static uint16_t ADC1_ReadRaw(void);
static uint16_t ADC1_ReadRawSettled(void);
static float SensorVoltageToTempC(float voltage);
static void ScanAllMuxChannels(TempStatistics_t *stats, bool report);
static void CAN_Init_Filter(void);
static HAL_StatusTypeDef CAN_SendTemperatureStatistics(TempStatistics_t *stats);
static HAL_StatusTypeDef CAN_SendChannelTemp(uint32_t id, uint8_t temp[7]);
void delay_us(uint32_t us);

#ifndef USE_SEMIHOSTING
int _write(int file, char *ptr, int len) {
	(void) file;
	HAL_UART_Transmit(&huart1, (uint8_t*) ptr, (uint16_t) len, HAL_MAX_DELAY);
	return len;
}
#endif

void delay_us(uint32_t us) {
	uint32_t start = DWT->CYCCNT;
	uint32_t ticks = us * (SystemCoreClock / 1000000);
	while ((DWT->CYCCNT - start) < ticks)
		;
}

// send 7 individual channels temperatures through CAN
static HAL_StatusTypeDef CAN_SendChannelTemp(uint32_t id, uint8_t temp[7]) {
	if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) > 0) { // ensure CAN mailbox is open to receive
		TxHeader.ExtId = id;
		TxHeader.IDE = CAN_ID_EXT;
		TxHeader.RTR = CAN_RTR_DATA;
		TxHeader.DLC = 8u;
		TxHeader.TransmitGlobalTime = DISABLE;

		TxData[0] = (uint8_t) (int8_t) temp[0];
		TxData[1] = (uint8_t) (int8_t) temp[1];
		TxData[2] = (uint8_t) (int8_t) temp[2];
		TxData[3] = (uint8_t) (int8_t) temp[3];
		TxData[4] = (uint8_t) (int8_t) temp[4];
		TxData[5] = (uint8_t) (int8_t) temp[5];
		TxData[6] = (uint8_t) (int8_t) temp[6];

		// Checksum = sum of bytes 0–6, no seed
		uint8_t checksum = 0u;
		for (uint8_t i = 0u; i < 7u; i++)
			checksum += TxData[i];
		TxData[7] = checksum + 65;

		return HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox);
	}
}

// send the max/min temperatures to the BMS over CAN
static HAL_StatusTypeDef CAN_SendTemperatureStatistics(TempStatistics_t *stats) {
	if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) > 0) { // ensure CAN mailbox is open to receive
		TxHeader.ExtId = 0x1839F380;
		TxHeader.IDE = CAN_ID_EXT;
		TxHeader.RTR = CAN_RTR_DATA;
		TxHeader.DLC = 8u;
		TxHeader.TransmitGlobalTime = DISABLE;

		TxData[0] = 0;
		TxData[1] = (uint8_t) (int8_t) stats->min_temp;
		TxData[2] = (uint8_t) (int8_t) stats->max_temp;
		//	TxData[3] = (uint8_t) (int8_t) 67;
		TxData[3] = (uint8_t) (int8_t) stats->max_channel;
		TxData[4] = 1;
		TxData[5] = 1;
		TxData[6] = 0;

		// Checksum = sum of bytes 0–6, no seed
		uint8_t checksum = 0u;
		for (uint8_t i = 0u; i < 7u; i++)
			checksum += TxData[i];
		TxData[7] = checksum + 65;

		return HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox);
	}
}

static void CAN_Init_Filter(void) {
	CAN_FilterTypeDef f = { 0 };
	f.FilterActivation = CAN_FILTER_ENABLE;
	f.FilterBank = 0u;
	f.FilterFIFOAssignment = CAN_RX_FIFO0;
	f.FilterIdHigh = 0x0000u;
	f.FilterIdLow = 0x0000u;
	f.FilterMaskIdHigh = 0x0000u;
	f.FilterMaskIdLow = 0x0000u;
	f.FilterMode = CAN_FILTERMODE_IDMASK;
	f.FilterScale = CAN_FILTERSCALE_32BIT;
	f.SlaveStartFilterBank = 14u;

	if (HAL_CAN_ConfigFilter(&hcan, &f) != HAL_OK)
		Error_Handler();
}

static void ADC1_SetLongSampleTime(void) {
	ADC_ChannelConfTypeDef sConfig = { 0 };

	sConfig.Channel = ADC_CHANNEL_2;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;

	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
}

static HAL_StatusTypeDef MUX_WriteRaw(mux_id_t mux, uint8_t cmd) {
	HAL_StatusTypeDef status;

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2,
			GPIO_PIN_SET);
	HAL_GPIO_WritePin(mux_sync_port[mux], mux_sync_pin[mux], GPIO_PIN_RESET);

	status = HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY);

	HAL_GPIO_WritePin(mux_sync_port[mux], mux_sync_pin[mux], GPIO_PIN_SET);

	return status;
}

static HAL_StatusTypeDef MUX_Disable(mux_id_t mux) {
	return MUX_WriteRaw(mux, MUX_DISABLE_CMD);
}

static void MUX_DisableAll(void) {
	(void) MUX_Disable(MUX1);
	(void) MUX_Disable(MUX2);
	(void) MUX_Disable(MUX3);
}

static HAL_StatusTypeDef MUX_SelectChannel(mux_id_t mux, uint8_t channel) {
	uint8_t i;

	if (channel >= MUX_CHANNELS_PER_CHIP) {
		return HAL_ERROR;
	}

	for (i = 0; i < NUM_MUXES; i++) {
		if (i != (uint8_t) mux) {
			if (MUX_Disable((mux_id_t) i) != HAL_OK) {
				return HAL_ERROR;
			}
		}
	}

	if (MUX_WriteRaw(mux, (uint8_t) (channel & 0x1FU)) != HAL_OK) {
		return HAL_ERROR;
	}

	return HAL_OK;
}

static uint16_t ADC1_ReadRaw(void) {
	uint16_t value = 0;

	if (HAL_ADC_Start(&hadc1) != HAL_OK) {
		Error_Handler();
	}

	if (HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY) != HAL_OK) {
		Error_Handler();
	}

	value = (uint16_t) HAL_ADC_GetValue(&hadc1);

	if (HAL_ADC_Stop(&hadc1) != HAL_OK) {
		Error_Handler();
	}

	return value;
}

static uint16_t ADC1_ReadRawSettled(void) {
	return ADC1_ReadRaw();
}

// use linear interperolation to convert voltage to temperature, based on data
// from datasheet
static float SensorVoltageToTempC(float voltage) {
	static const float temp_table[] = { -40, -35, -30, -25, -20, -15, -10, -5,
			0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80,
			85, 90, 95, 100, 105, 110, 115, 120 };

	static const float volt_table[] =
			{ 2.44f, 2.42f, 2.40f, 2.38f, 2.35f, 2.32f, 2.27f, 2.23f, 2.17f,
					2.11f, 2.05f, 1.99f, 1.92f, 1.86f, 1.80f, 1.74f, 1.68f,
					1.63f, 1.59f, 1.55f, 1.51f, 1.48f, 1.45f, 1.43f, 1.40f,
					1.38f, 1.37f, 1.35f, 1.34f, 1.33f, 1.32f, 1.31f, 1.30f };

	const uint32_t n = sizeof(temp_table) / sizeof(temp_table[0]);
	uint32_t i;

	if (voltage >= volt_table[0])
		return temp_table[0];
	if (voltage <= volt_table[n - 1])
		return temp_table[n - 1];

	for (i = 0; i < n - 1; i++) {
		if ((voltage <= volt_table[i]) && (voltage >= volt_table[i + 1])) {
			float v1 = volt_table[i];
			float v2 = volt_table[i + 1];
			float t1 = temp_table[i];
			float t2 = temp_table[i + 1];

			return t1 + (voltage - v1) * (t2 - t1) / (v2 - v1);
		}
	}

	return -999.0f;
}

// finds the highest and lowest temperature of the 90 channels
// uses 3 nested loops, the outermost to iterate through the 3 different
// multiplexers, the next to iterate through each channel on the multiplexer,
// and then the innermost channel samples each channel 500 times in order for an
// average reading over a period of 2ms (longer than the duration of the noise)
static void ScanAllMuxChannels(TempStatistics_t *stats, bool report) {
	uint8_t mux;
	uint8_t ch;
	uint8_t temps[90] = { 0 };
	int8_t highTemps = 0;

	for (mux = 0; mux < NUM_MUXES; mux++) {

		for (ch = 0; ch < MUX_CHANNELS_PER_CHIP; ch++) {
			if (mux == 2 && ch > 25) {
				continue;
			}

			int32_t faults = 0u;
			uint32_t sum = 0u;
			uint16_t count = 0u;

			if (MUX_SelectChannel((mux_id_t) mux, ch) != HAL_OK) {
				continue;
			}

			HAL_Delay(5);

			/* Reset filter state — each channel scan starts fresh */
			Biquad_Reset(mux, ch);

			for (int i = 0; i < 500; i++) {
				uint16_t raw = ADC1_ReadRawSettled(); // each sample takes 4us

//					double filtered = Biquad_Process(mux, ch, (double) raw);
//					uint16_t raw_f = (uint16_t) (filtered + 0.5);
				if (raw <= 1911 || raw >= 2962) { // only accept temps between 0 and 60C
					faults++;
					continue;
				}
				sum += raw;
				count++;
			}

			float temp_c;

			// if 85% of samples are outside the acceptable range set the channel temp
			// to 120C
			if (faults >= 425) {
				temp_c = 80;
				highTemps++;
			} else {
				sum = sum / count;

				float voltage = (3.0f * (float) sum) / 4095.0f;
				temp_c = SensorVoltageToTempC(voltage);

				if (temp_c > 0 && temp_c < 85) {
					if (temp_c < stats->min_temp) {
						stats->min_temp = (int16_t) temp_c;
						//						stats->min_channel = 32
						//* mux + ch;
					}
					if (temp_c > stats->max_temp) {
						stats->max_temp = (int16_t) temp_c;
						//						stats->max_channel = 32
						//* mux + ch;
					}
				}
			}

			int16_t idx = mux * MUX_CHANNELS_PER_CHIP + ch;
			temps[idx] = temp_c;

			// conditional exists only for startup so that the BMS doesn't fault while
			// temps are still being collected
			if (report) {
				if (stats->min_temp == 300 || stats->max_temp == -300) {
					continue;
				}
				stats->max_channel = highTemps;
				CAN_SendTemperatureStatistics(stats);
			} else {
				stats->max_temp = 20;
				stats->min_temp = 20;
				CAN_SendTemperatureStatistics(stats);
				stats->max_temp = -300;
				stats->min_temp = 300;
			}
		}
	}

	if (highTemps >= 76) {
		stats->max_temp = 80;
		stats->min_temp = 80;
		CAN_SendTemperatureStatistics(stats);
		stats->max_temp = -300;
		stats->min_temp = 300;
	}

	// package all 90 channel temps into groups of 7 and send each over CAN
	for (int16_t i = 0; i < 90; i += 7) {
		uint8_t sendTemp[7] = {0};
		for (int16_t j = 0; j < 7 && (i + j) < 90; j++) {
			sendTemp[j] = temps[i + j];
		}
		CAN_SendChannelTemp(DAQ_BASE_ID + (i / 7), sendTemp);
	}
	MUX_DisableAll();
}

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	//
	//	clock_t start, end;
	//	double cpu_time_used;

#ifdef USE_SEMIHOSTING
  initialise_monitor_handles();
#endif
	HAL_Init();

	SystemClock_Config();

	MX_GPIO_Init();
	MX_ADC1_Init();
	MX_SPI1_Init();
	MX_USART1_UART_Init();
	MX_CAN_Init();
	MX_TIM4_Init();

	// set up DWT timer
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CYCCNT = 0;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

	CAN_Init_Filter();
	HAL_CAN_Start(&hcan);

	if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK) {
		Error_Handler();
	}

	TempStatistics_t stats;
	TempStatistics_t initial;

	ADC1_SetLongSampleTime();
	MUX_DisableAll();

	HAL_Delay(10);

	stats.min_temp = 300;
	stats.max_temp = -300;
	stats.min_channel = 0;
	stats.max_channel = 0;
	stats.num_enabled = 0;

	initial.min_temp = 20;
	initial.max_temp = 20;
	initial.min_channel = 0;
	initial.max_channel = 0;
	initial.num_enabled = 0;

	// report 'fake' data upon startup
	CAN_SendTemperatureStatistics(&initial);

	// collect statistics without reporting them to calibrate the system
	//	ScanAllMuxChannels(&stats, false);
	//	ScanAllMuxChannels(&stats, false);

	while (1) {
		//		uint32_t t0_ms = HAL_GetTick();
		ScanAllMuxChannels(&stats, true);
		//		uint32_t dt_ms = HAL_GetTick() - t0_ms;
		//		float dt_s = dt_ms / 1000.0f;
		//		printf("ScanAllMuxChannels time: %.3f s\r\n", dt_s);
		//		ScanAllMuxChannels(&stats, false);
		//		CAN_SendTemperatureStatistics(&initial);
		//
		//		HAL_Delay(1000);

		// reset max/min temp after each loop to ensure accurate data is being
		// reported
		stats.min_temp = 300;
		stats.max_temp = -300;
	}
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
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
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
	RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}

	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
	PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		Error_Handler();
	}

	HAL_RCC_EnableCSS();
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {
	ADC_ChannelConfTypeDef sConfig = { 0 };

	hadc1.Instance = ADC1;
	hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}

	sConfig.Channel = ADC_CHANNEL_2;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief CAN Initialization Function
 * @param None
 * @retval None
 */
static void MX_CAN_Init(void) {
	hcan.Instance = CAN1;
	hcan.Init.Prescaler = 3; /* was 2 → gave 750 kbps    */
	hcan.Init.Mode = CAN_MODE_NORMAL;
	hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
	hcan.Init.TimeSeg1 = CAN_BS1_13TQ;
	hcan.Init.TimeSeg2 = CAN_BS2_4TQ;
	hcan.Init.TimeTriggeredMode = DISABLE;
	hcan.Init.AutoBusOff = DISABLE;
	hcan.Init.AutoWakeUp = DISABLE;
	hcan.Init.AutoRetransmission = DISABLE;
	hcan.Init.ReceiveFifoLocked = DISABLE;
	hcan.Init.TransmitFifoPriority = DISABLE;
	if (HAL_CAN_Init(&hcan) != HAL_OK)
		Error_Handler();
}
/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void) {
	hspi1.Instance = SPI1;
	hspi1.Init.Mode = SPI_MODE_MASTER;
	hspi1.Init.Direction = SPI_DIRECTION_1LINE;
	hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi1.Init.NSS = SPI_NSS_SOFT;
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi1) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void) {
	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 0;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 65535;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim4) != HAL_OK) {
		Error_Handler();
	}

	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart1) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/* SET first — open-drain high = released = mux SYNC deasserted    */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2,
			GPIO_PIN_SET);

	GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	__disable_irq();
	while (1) {
	}
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line) {
  (void)file;
  (void)line;
}
#endif /* USE_FULL_ASSERT */
