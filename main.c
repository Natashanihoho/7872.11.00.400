/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define STARTBYTE 0xAA

#define LED_ON    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET)
#define LED_OFF   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET)

#define DE_ENABLE    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET)
#define DE_DISABLE   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET)

#define RE_ENABLE    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET)
#define RE_DISABLE   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint8_t count_rx;                // счетчик принятых байт
uint8_t buf_rx[9];               // буфер принятых байт
uint8_t buf_tx[9];               // буфер отправленных байт
uint8_t received_byte;           // принятый байт
uint8_t rx_checked;              // флаг = 1, если посылка принята корректно (9 байт, стартовый байт, контрольная сумма)
uint8_t collimator;              // выбранный коллиматор для опроса
uint8_t dir_auto;                // направление движения двигателя при автоматическом режиме(1 - против ЧС, 2 - по ЧС)
uint8_t dir_man;                 // направление движения двигателя при ручном режиме(1 - против ЧС, 2 - по ЧС)
uint8_t heat_L1, heat_L2;        // флаги нагрева сеток
uint8_t duty_cycle_L1, duty_cycle_L2; // процент нагрева сеток
uint8_t mode;                    // режим смещения (1 - ручной, 0  - автоматический)
uint16_t steps;
int8_t temperature;
uint8_t dt[9];
uint8_t i = 0;


uint8_t state_ds1820 = 2;

uint8_t start_mot = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM4_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
uint8_t ds18b20_Init(void);
uint8_t ds18b20_ReadScratcpad(uint8_t *Data);
void ds18b20_MeasureTemperCmd(void);
int8_t getTemperature(uint8_t *ScratchpadData);

extern uint8_t spiTxBursts[dSPIN_CMD_ARG_MAX_NB_BYTES][NUMBER_OF_SLAVES];
extern uint8_t spiRxBursts[dSPIN_CMD_ARG_MAX_NB_BYTES][NUMBER_OF_SLAVES];
extern uint8_t arrayTxBytes[NUMBER_OF_SLAVES];
extern uint32_t arrayValues[NUMBER_OF_SLAVES];

extern dSPIN_RegsStruct_TypeDef dSPIN_RegsStructArray[NUMBER_OF_SLAVES];

extern uint8_t daisy_chain;
extern uint8_t number_of_slaves;

extern uint8_t commandArray[NUMBER_OF_SLAVES];
extern uint32_t argumentArray[NUMBER_OF_SLAVES];

dSPIN_RegsStruct_TypeDef dSPIN_RegsStruct;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//-------------------------------------------------------------
//-------------------------------------------------------------
//-------------------------------------------------------------
void Delay_us(uint16_t us)
{
	htim4.Init.Period = us;
	HAL_TIM_Base_Start(&htim4);
	while(__HAL_TIM_GET_COUNTER(&htim4) != us);
	__HAL_TIM_SET_COUNTER(&htim4, 0);
	HAL_TIM_Base_Stop(&htim4);
}
//-------------------------------------------------------------
uint8_t XOR_calc(uint8_t *buf, uint8_t length)
{
	uint8_t sum_xor = 0;
	for(int i = 0; i < length; i++)
		sum_xor ^= buf[i];
	return sum_xor;	
}
void ExtractTemperature(void)
{
	uint8_t st = 1;	
	state_ds1820 = ds18b20_Init();
	while(st != 0)
	{
		ds18b20_MeasureTemperCmd();
	  HAL_Delay(150);
		st =ds18b20_ReadScratcpad(dt);
	}
	temperature = getTemperature(dt);
}
//-------------------------------------------------------------
bool CheckBit(uint8_t byte, uint8_t bit)
{
	if ((byte & (1 << bit)) == 0) return false;
	return true;
}
//-------------------------------------------------------------
void SendPacket(void)
{
	buf_tx[0] = STARTBYTE;
	buf_tx[1] = collimator;
	
	buf_tx[5] = temperature;
	buf_tx[8] = XOR_calc(buf_tx, 8);
	
	RE_ENABLE;
	DE_ENABLE;
	for(int i = 0; i < 9; i++)
		HAL_UART_Transmit(&huart1, &buf_tx[i], 1, 1000);
	DE_DISABLE;
	RE_DISABLE;
}
//-------------------------------------------------------------
void GetReceivedData(void)
{
	// 1 байт
	collimator = buf_rx[1];
	// 2 байт
	if(CheckBit(buf_rx[2], 5)) dir_auto = 2; else dir_auto = 1; 
	if(CheckBit(buf_rx[2], 3)) heat_L2 = 1;	else heat_L2 = 0;	
	if(CheckBit(buf_rx[2], 2)) heat_L1 = 1;	else heat_L1 = 0;	
	if(CheckBit(buf_rx[2], 1)) mode = 1; else mode = 0;
	if(CheckBit(buf_rx[2], 0)) dir_auto = 2; else dir_auto = 1;
	// 3-4 байты
	steps = (uint16_t)(buf_rx[3] << 8) | buf_rx[4];
	// 5-6 байты
	duty_cycle_L1 = buf_rx[5];
	duty_cycle_L2 = buf_rx[6];	
  
	ExtractTemperature();
	SendPacket();
}
//-------------------------------------------------------------
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{ 
		buf_rx[count_rx] = received_byte;
		count_rx++;
		switch(count_rx)
		{
			case 1:
				if(buf_rx[0] != STARTBYTE) count_rx = 0; break;
			case 9: 
				count_rx=0;
				if(XOR_calc(buf_rx, 9) == 0) rx_checked = 1; 
				break;
		}	  
		HAL_UART_Receive_IT(&huart1, &received_byte, 1);	
}
//-------------------------------------------------------------
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
  MX_TIM4_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
	double   MAX_SPEED[NUMBER_OF_SLAVES] = dSPIN_DC_CONF_PARAM_MAX_SPEED;			
	double   ACC[NUMBER_OF_SLAVES] = dSPIN_DC_CONF_PARAM_ACC;
  double   DEC[NUMBER_OF_SLAVES] = dSPIN_DC_CONF_PARAM_DEC;
  double   FS_SPD[NUMBER_OF_SLAVES] = dSPIN_DC_CONF_PARAM_FS_SPD;
#if defined(L6470)
  double   KVAL_HOLD[NUMBER_OF_SLAVES] = dSPIN_DC_CONF_PARAM_KVAL_HOLD;
  double   KVAL_RUN[NUMBER_OF_SLAVES] = dSPIN_DC_CONF_PARAM_KVAL_RUN;
  double   KVAL_ACC[NUMBER_OF_SLAVES] = dSPIN_DC_CONF_PARAM_KVAL_ACC;
  double   KVAL_DEC[NUMBER_OF_SLAVES] = dSPIN_DC_CONF_PARAM_KVAL_DEC;
  double   INT_SPD[NUMBER_OF_SLAVES] = dSPIN_DC_CONF_PARAM_INT_SPD;
  double   ST_SLP[NUMBER_OF_SLAVES] = dSPIN_DC_CONF_PARAM_ST_SLP;
  double   FN_SLP_ACC[NUMBER_OF_SLAVES] = dSPIN_DC_CONF_PARAM_FN_SLP_ACC;
  double   FN_SLP_DEC[NUMBER_OF_SLAVES] = dSPIN_DC_CONF_PARAM_FN_SLP_DEC;
  double   K_THERM[NUMBER_OF_SLAVES] = dSPIN_DC_CONF_PARAM_K_THERM;
  double   STALL_TH[NUMBER_OF_SLAVES] = dSPIN_DC_CONF_PARAM_STALL_TH;
  uint8_t  OCD_TH[NUMBER_OF_SLAVES] = dSPIN_DC_CONF_PARAM_OCD_TH;
  uint8_t  ALARM_EN[NUMBER_OF_SLAVES] = dSPIN_DC_CONF_PARAM_ALARM_EN;
  /* OR-ed definitions */
  double   MIN_SPEED[NUMBER_OF_SLAVES] = dSPIN_DC_CONF_PARAM_MIN_SPEED;
  uint16_t LSPD_BIT[NUMBER_OF_SLAVES] = dSPIN_DC_CONF_PARAM_LSPD_BIT;
  uint8_t  STEP_MODE[NUMBER_OF_SLAVES] = dSPIN_DC_CONF_PARAM_STEP_MODE;
  uint8_t  SYNC_MODE[NUMBER_OF_SLAVES] = dSPIN_DC_CONF_PARAM_SYNC_MODE;
  uint16_t CONFIG_CLOCK_SETTING[NUMBER_OF_SLAVES] = dSPIN_DC_CONF_PARAM_CLOCK_SETTING;
  uint16_t CONFIG_SW_MODE[NUMBER_OF_SLAVES] = dSPIN_DC_CONF_PARAM_SW_MODE;
  uint16_t CONFIG_OC_SD[NUMBER_OF_SLAVES] = dSPIN_DC_CONF_PARAM_OC_SD;
  uint16_t CONFIG_SR[NUMBER_OF_SLAVES] = dSPIN_DC_CONF_PARAM_SR;
  uint16_t CONFIG_VS_COMP[NUMBER_OF_SLAVES] = dSPIN_DC_CONF_PARAM_VS_COMP;
  uint16_t CONFIG_PWM_DIV[NUMBER_OF_SLAVES] = dSPIN_DC_CONF_PARAM_PWM_DIV;
  uint16_t CONFIG_PWM_MUL[NUMBER_OF_SLAVES] = dSPIN_DC_CONF_PARAM_PWM_MUL;

#endif /* defined(L6470) */ 
dSPIN_Regs_Struct_Reset(&dSPIN_RegsStruct);
	
	/* Acceleration rate settings to dSPIN_CONF_PARAM_ACC in steps/s2, range 14.55 to 59590 steps/s2 */
	dSPIN_RegsStruct.ACC 		= AccDec_Steps_to_Par(dSPIN_CONF_PARAM_ACC);
	/* Deceleration rate settings to dSPIN_CONF_PARAM_DEC in steps/s2, range 14.55 to 59590 steps/s2 */
	dSPIN_RegsStruct.DEC 		= AccDec_Steps_to_Par(dSPIN_CONF_PARAM_DEC); 
	/* Maximum speed settings to dSPIN_CONF_PARAM_MAX_SPEED in steps/s, range 15.25 to 15610 steps/s */
	dSPIN_RegsStruct.MAX_SPEED 	= MaxSpd_Steps_to_Par(dSPIN_CONF_PARAM_MAX_SPEED);
	/* Full step speed settings dSPIN_CONF_PARAM_FS_SPD in steps/s, range 7.63 to 15625 steps/s */
	dSPIN_RegsStruct.FS_SPD 	= FSSpd_Steps_to_Par(dSPIN_CONF_PARAM_FS_SPD);
#if defined(L6470)
	/* Minimum speed settings to dSPIN_CONF_PARAM_MIN_SPEED in steps/s, range 0 to 976.3 steps/s */
	dSPIN_RegsStruct.MIN_SPEED	= dSPIN_CONF_PARAM_LSPD_BIT|MinSpd_Steps_to_Par(dSPIN_CONF_PARAM_MIN_SPEED);
        /* Acceleration duty cycle (torque) settings to dSPIN_CONF_PARAM_KVAL_ACC in %, range 0 to 99.6% */
	dSPIN_RegsStruct.KVAL_ACC 	= Kval_Perc_to_Par(dSPIN_CONF_PARAM_KVAL_ACC);
        /* Deceleration duty cycle (torque) settings to dSPIN_CONF_PARAM_KVAL_DEC in %, range 0 to 99.6% */
	dSPIN_RegsStruct.KVAL_DEC 	= Kval_Perc_to_Par(dSPIN_CONF_PARAM_KVAL_DEC);		
        /* Run duty cycle (torque) settings to dSPIN_CONF_PARAM_KVAL_RUN in %, range 0 to 99.6% */
	dSPIN_RegsStruct.KVAL_RUN 	= Kval_Perc_to_Par(dSPIN_CONF_PARAM_KVAL_RUN);
        /* Hold duty cycle (torque) settings to dSPIN_CONF_PARAM_KVAL_HOLD in %, range 0 to 99.6% */
	dSPIN_RegsStruct.KVAL_HOLD 	= Kval_Perc_to_Par(dSPIN_CONF_PARAM_KVAL_HOLD);
	        /* Thermal compensation param settings to dSPIN_CONF_PARAM_K_THERM, range 1 to 1.46875 */
	dSPIN_RegsStruct.K_THERM 	= KTherm_to_Par(dSPIN_CONF_PARAM_K_THERM);
	/* Intersect speed settings for BEMF compensation to dSPIN_CONF_PARAM_INT_SPD in steps/s, range 0 to 3906 steps/s */
	dSPIN_RegsStruct.INT_SPD 	= IntSpd_Steps_to_Par(dSPIN_CONF_PARAM_INT_SPD);
	/* BEMF start slope settings for BEMF compensation to dSPIN_CONF_PARAM_ST_SLP in % step/s, range 0 to 0.4% s/step */
	dSPIN_RegsStruct.ST_SLP 	= BEMF_Slope_Perc_to_Par(dSPIN_CONF_PARAM_ST_SLP);
	/* BEMF final acc slope settings for BEMF compensation to dSPIN_CONF_PARAM_FN_SLP_ACC in% step/s, range 0 to 0.4% s/step */
	dSPIN_RegsStruct.FN_SLP_ACC = BEMF_Slope_Perc_to_Par(dSPIN_CONF_PARAM_FN_SLP_ACC);
	/* BEMF final dec slope settings for BEMF compensation to dSPIN_CONF_PARAM_FN_SLP_DEC in% step/s, range 0 to 0.4% s/step */
	dSPIN_RegsStruct.FN_SLP_DEC = BEMF_Slope_Perc_to_Par(dSPIN_CONF_PARAM_FN_SLP_DEC);
	/* Stall threshold settings to dSPIN_CONF_PARAM_STALL_TH in mA, range 31.25 to 4000mA */
	dSPIN_RegsStruct.STALL_TH 	= StallTh_to_Par(dSPIN_CONF_PARAM_STALL_TH);
        /* Set Config register according to config parameters */
        /* clock setting, switch hard stop interrupt mode, */
        /*  supply voltage compensation, overcurrent shutdown */
        /* slew-rate , PWM frequency */
	dSPIN_RegsStruct.CONFIG 	= (uint16_t)dSPIN_CONF_PARAM_CLOCK_SETTING |
                                          (uint16_t)dSPIN_CONF_PARAM_SW_MODE	   | 
                                          (uint16_t)dSPIN_CONF_PARAM_VS_COMP       |    
                                          (uint16_t)dSPIN_CONF_PARAM_OC_SD         | 
                                          (uint16_t)dSPIN_CONF_PARAM_SR	           | 
                                          (uint16_t)dSPIN_CONF_PARAM_PWM_DIV       |
                                          (uint16_t)dSPIN_CONF_PARAM_PWM_MUL;
  #endif /* defined(L6470) */

	/* Overcurrent threshold settings to dSPIN_CONF_PARAM_OCD_TH in mA */
	dSPIN_RegsStruct.OCD_TH 	= dSPIN_CONF_PARAM_OCD_TH;        
        /* Alarm settings to dSPIN_CONF_PARAM_ALARM_EN */
	dSPIN_RegsStruct.ALARM_EN 	= dSPIN_CONF_PARAM_ALARM_EN;
        /* Step mode and sycn mode settings via dSPIN_CONF_PARAM_SYNC_MODE and dSPIN_CONF_PARAM_STEP_MODE */
	dSPIN_RegsStruct.STEP_MODE 	= (uint8_t)dSPIN_CONF_PARAM_SYNC_MODE |
                                          (uint8_t)dSPIN_CONF_PARAM_STEP_MODE;
		
	dSPIN_Registers_Set(&dSPIN_RegsStruct);
	
	for(int i =0;i<NUMBER_OF_SLAVES;i++)
	{
    MAX_SPEED[i] = (double)dSPIN_DC_CONF_PARAM_MAX_SPEED;			
	   ACC[i] = (double)dSPIN_DC_CONF_PARAM_ACC;
     DEC[i] = (double)dSPIN_DC_CONF_PARAM_DEC;
     FS_SPD[i] = (double)dSPIN_DC_CONF_PARAM_FS_SPD;
     KVAL_HOLD[i] = (double)dSPIN_DC_CONF_PARAM_KVAL_HOLD;
     KVAL_RUN[i] = (double)dSPIN_DC_CONF_PARAM_KVAL_RUN;
     KVAL_ACC[i] = (double)dSPIN_DC_CONF_PARAM_KVAL_ACC;
     KVAL_DEC[i] = (double)dSPIN_DC_CONF_PARAM_KVAL_DEC;
     INT_SPD[i] = (double)dSPIN_DC_CONF_PARAM_INT_SPD;
     ST_SLP[i] = (double)dSPIN_DC_CONF_PARAM_ST_SLP;
     FN_SLP_ACC[i] = (double)dSPIN_DC_CONF_PARAM_FN_SLP_ACC;
     FN_SLP_DEC[i] = (double)dSPIN_DC_CONF_PARAM_FN_SLP_DEC;
     K_THERM[i] = (double)dSPIN_DC_CONF_PARAM_K_THERM;
     STALL_TH[i] = (double)dSPIN_DC_CONF_PARAM_STALL_TH;
     OCD_TH[i] = (uint8_t)dSPIN_DC_CONF_PARAM_OCD_TH;
     ALARM_EN[i] = (uint8_t)dSPIN_DC_CONF_PARAM_ALARM_EN;
     MIN_SPEED[i] = (double)dSPIN_DC_CONF_PARAM_MIN_SPEED;
     LSPD_BIT[i] = (uint16_t)dSPIN_DC_CONF_PARAM_LSPD_BIT;
     STEP_MODE[i] = (uint8_t)dSPIN_DC_CONF_PARAM_STEP_MODE;
     SYNC_MODE[i] = (uint8_t)dSPIN_DC_CONF_PARAM_SYNC_MODE;
     CONFIG_CLOCK_SETTING[i] = (uint16_t)dSPIN_DC_CONF_PARAM_CLOCK_SETTING;
     CONFIG_SW_MODE[i] = (uint16_t)dSPIN_DC_CONF_PARAM_SW_MODE;
     CONFIG_OC_SD[i] = (uint16_t)dSPIN_DC_CONF_PARAM_OC_SD;
     CONFIG_SR[i] = (uint16_t)dSPIN_DC_CONF_PARAM_SR;
     CONFIG_VS_COMP[i] = (uint16_t)dSPIN_DC_CONF_PARAM_VS_COMP;
     CONFIG_PWM_DIV[i] = (uint16_t)dSPIN_DC_CONF_PARAM_PWM_DIV;
     CONFIG_PWM_MUL[i] = (uint16_t)dSPIN_DC_CONF_PARAM_PWM_MUL;		
	}  
	
		for (i = 0; i < number_of_slaves; i++)
			{
				dSPIN_Regs_Struct_Reset(&dSPIN_RegsStructArray[i]);
			}
			for (i=0;i<number_of_slaves;i++)
        {
          dSPIN_RegsStructArray[i].ACC 		= AccDec_Steps_to_Par(ACC[i]);
          dSPIN_RegsStructArray[i].DEC 		= AccDec_Steps_to_Par(DEC[i]);
          dSPIN_RegsStructArray[i].MAX_SPEED 	= MaxSpd_Steps_to_Par(MAX_SPEED[i]);
          dSPIN_RegsStructArray[i].FS_SPD 	= FSSpd_Steps_to_Par(FS_SPD[i]);
          dSPIN_RegsStructArray[i].MIN_SPEED	= LSPD_BIT[i]|MinSpd_Steps_to_Par(MIN_SPEED[i]);
          dSPIN_RegsStructArray[i].KVAL_ACC 	= Kval_Perc_to_Par(KVAL_ACC[i]);
          dSPIN_RegsStructArray[i].KVAL_DEC 	= Kval_Perc_to_Par(KVAL_DEC[i]);		
          dSPIN_RegsStructArray[i].KVAL_RUN 	= Kval_Perc_to_Par(KVAL_RUN[i]);
          dSPIN_RegsStructArray[i].KVAL_HOLD 	= Kval_Perc_to_Par(KVAL_HOLD[i]);
          dSPIN_RegsStructArray[i].K_THERM 	= KTherm_to_Par(K_THERM[i]);
          dSPIN_RegsStructArray[i].INT_SPD 	= IntSpd_Steps_to_Par(INT_SPD[i]);
          dSPIN_RegsStructArray[i].ST_SLP 	= BEMF_Slope_Perc_to_Par(ST_SLP[i]);
          dSPIN_RegsStructArray[i].FN_SLP_ACC   = BEMF_Slope_Perc_to_Par(FN_SLP_ACC[i]);
          dSPIN_RegsStructArray[i].FN_SLP_DEC   = BEMF_Slope_Perc_to_Par(FN_SLP_DEC[i]);
          dSPIN_RegsStructArray[i].STALL_TH 	= StallTh_to_Par(STALL_TH[i]);
          dSPIN_RegsStructArray[i].CONFIG 	= (uint16_t)CONFIG_CLOCK_SETTING[i] |
                                            (uint16_t)CONFIG_SW_MODE[i]	   | 
                                            (uint16_t)CONFIG_VS_COMP[i]       |    
                                            (uint16_t)CONFIG_OC_SD[i]         | 
                                            (uint16_t)CONFIG_SR[i]	           | 
                                            (uint16_t)CONFIG_PWM_DIV[i]       |
                                            (uint16_t)CONFIG_PWM_MUL[i];
         
          dSPIN_RegsStructArray[i].OCD_TH 	= OCD_TH[i];
          dSPIN_RegsStructArray[i].ALARM_EN 	= ALARM_EN[i];
          dSPIN_RegsStructArray[i].STEP_MODE 	= (uint8_t)SYNC_MODE[i] | (uint8_t)STEP_MODE[i];
        }
				dSPIN_All_Slaves_Registers_Set(number_of_slaves, dSPIN_RegsStructArray);
		commandArray[0] = dSPIN_HARD_HIZ;
		argumentArray[0] = Speed_Steps_to_Par(250);
		commandArray[1] = (uint8_t) dSPIN_RUN |(uint8_t)REV;
				//commandArray[1] = dSPIN_HARD_HIZ;
		argumentArray[1] = Speed_Steps_to_Par(250);
		dSPIN_All_Slaves_Send_Command(number_of_slaves, commandArray, argumentArray);		
       //dSPIN_Move(REV, 528);
		HAL_Delay(5000);
    commandArray[0] = dSPIN_HARD_HIZ;
		commandArray[1] = dSPIN_HARD_HIZ;
		dSPIN_All_Slaves_Send_Command(number_of_slaves, commandArray, 0);			
		HAL_UART_Receive_IT(&huart1, &received_byte, 1);
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {		
		if(rx_checked)
		{
			rx_checked = 0;
			GetReceivedData();
		}			
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 71;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_15, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4|GPIO_PIN_8, GPIO_PIN_SET);

  /*Configure GPIO pins : PA2 PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

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
