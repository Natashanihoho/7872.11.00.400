/**
  ******************************************************************************
  * @file    dspin.c 
  * @author  IPC Rennes
  * @version V2.0
  * @date    October 4, 2013
  * @brief   dSPIN (L6470 and L6472) product related routines
  * @note    (C) COPYRIGHT 2013 STMicroelectronics
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2013 STMicroelectronics</center></h2>
  */

/* Includes ------------------------------------------------------------------*/
#include "dspin.h"


/** @addtogroup  dSPIN FW library interface
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
extern SPI_HandleTypeDef hspi1;
extern uint8_t spiTxBursts[dSPIN_CMD_ARG_MAX_NB_BYTES][NUMBER_OF_SLAVES];
extern uint8_t spiRxBursts[dSPIN_CMD_ARG_MAX_NB_BYTES][NUMBER_OF_SLAVES];
extern uint8_t arrayTxBytes[NUMBER_OF_SLAVES];
extern uint32_t arrayValues[NUMBER_OF_SLAVES];

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Inserts a delay time.
  * @param  nCount specifies the delay time length.
  * @retval None
  */
void dSPIN_Delay(__IO uint32_t nCount)
{
  for(; nCount!= 0;nCount--);
}

/**
  * @brief  Resets DSPIN and puts it into standby mode
  * @param  None
  * @retval None
  */

/**
  * @brief  Fills-in dSPIN configuration structure with default values.
  * @param  dSPIN_RegsStruct structure address (pointer to struct)
  * @retval None
  */
void dSPIN_Regs_Struct_Reset(dSPIN_RegsStruct_TypeDef* dSPIN_RegsStruct)
{
	dSPIN_RegsStruct->ABS_POS = 0;
	dSPIN_RegsStruct->EL_POS = 0;
	dSPIN_RegsStruct->MARK = 0;
	dSPIN_RegsStruct->ACC = 0x08A;
	dSPIN_RegsStruct->DEC = 0x08A;
	dSPIN_RegsStruct->MAX_SPEED = 0x041;
	dSPIN_RegsStruct->MIN_SPEED = 0;
	dSPIN_RegsStruct->FS_SPD = 0x027;
	dSPIN_RegsStruct->KVAL_HOLD = 0x29;
	dSPIN_RegsStruct->KVAL_RUN = 0x29;
	dSPIN_RegsStruct->KVAL_ACC = 0x29;
	dSPIN_RegsStruct->KVAL_DEC = 0x29;
	dSPIN_RegsStruct->INT_SPD = 0x0408;
	dSPIN_RegsStruct->ST_SLP = 0x19;
	dSPIN_RegsStruct->FN_SLP_ACC = 0x29;
	dSPIN_RegsStruct->FN_SLP_DEC = 0x29;
	dSPIN_RegsStruct->K_THERM = 0;
  dSPIN_RegsStruct->STALL_TH = 0x40;    
	dSPIN_RegsStruct->OCD_TH = 0x8;
	dSPIN_RegsStruct->STEP_MODE = 0x7;
	dSPIN_RegsStruct->ALARM_EN = 0xFF;
	dSPIN_RegsStruct->CONFIG = 0x2E88;
}
 
/**
  * @brief  Configures dSPIN internal registers with values in the config structure.
  * @param  dSPIN_RegsStruct Configuration structure address (pointer to configuration structure)
  * @retval None
  */
void dSPIN_Registers_Set(dSPIN_RegsStruct_TypeDef* dSPIN_RegsStruct)
{
	dSPIN_Set_Param(dSPIN_ABS_POS, dSPIN_RegsStruct->ABS_POS);
	dSPIN_Set_Param(dSPIN_EL_POS, dSPIN_RegsStruct->EL_POS);
	dSPIN_Set_Param(dSPIN_MARK, dSPIN_RegsStruct->MARK);
	dSPIN_Set_Param(dSPIN_ACC, dSPIN_RegsStruct->ACC);
	dSPIN_Set_Param(dSPIN_DEC, dSPIN_RegsStruct->DEC);
	dSPIN_Set_Param(dSPIN_MAX_SPEED, dSPIN_RegsStruct->MAX_SPEED);
	dSPIN_Set_Param(dSPIN_MIN_SPEED, dSPIN_RegsStruct->MIN_SPEED);
	dSPIN_Set_Param(dSPIN_FS_SPD, dSPIN_RegsStruct->FS_SPD);
	dSPIN_Set_Param(dSPIN_KVAL_HOLD, dSPIN_RegsStruct->KVAL_HOLD);
	dSPIN_Set_Param(dSPIN_KVAL_RUN, dSPIN_RegsStruct->KVAL_RUN);
	dSPIN_Set_Param(dSPIN_KVAL_ACC, dSPIN_RegsStruct->KVAL_ACC);
	dSPIN_Set_Param(dSPIN_KVAL_DEC, dSPIN_RegsStruct->KVAL_DEC);
	dSPIN_Set_Param(dSPIN_INT_SPD, dSPIN_RegsStruct->INT_SPD);
	dSPIN_Set_Param(dSPIN_ST_SLP, dSPIN_RegsStruct->ST_SLP);
	dSPIN_Set_Param(dSPIN_FN_SLP_ACC, dSPIN_RegsStruct->FN_SLP_ACC);
	dSPIN_Set_Param(dSPIN_FN_SLP_DEC, dSPIN_RegsStruct->FN_SLP_DEC);
	dSPIN_Set_Param(dSPIN_K_THERM, dSPIN_RegsStruct->K_THERM);
	dSPIN_Set_Param(dSPIN_STALL_TH, dSPIN_RegsStruct->STALL_TH);     
  dSPIN_Set_Param(dSPIN_OCD_TH, dSPIN_RegsStruct->OCD_TH);
	dSPIN_Set_Param(dSPIN_STEP_MODE, dSPIN_RegsStruct->STEP_MODE);
	dSPIN_Set_Param(dSPIN_ALARM_EN, dSPIN_RegsStruct->ALARM_EN);
	dSPIN_Set_Param(dSPIN_CONFIG, dSPIN_RegsStruct->CONFIG);
}

#if defined(DEBUG)  
/**
  * @brief Reads dSPIN internal registers and print them to terminal I/O
  * @param dSPIN_RegsStruct Configuration structure address (pointer to configuration structure)
  * @retval None
  */
void dSPIN_Registers_Get(dSPIN_RegsStruct_TypeDef* dSPIN_RegsStruct)
{
  uint32_t read_reg;
  char diff[1];
  char str[10];
  
  PRINT_REG(ABS_POS, read_reg, diff, str);
  PRINT_REG(EL_POS, read_reg, diff, str);
  PRINT_REG(MARK, read_reg, diff, str);
  PRINT_REG(SPEED, read_reg, diff, str);
  PRINT_REG(ACC, read_reg, diff, str);
  PRINT_REG(DEC, read_reg, diff, str);
  PRINT_REG(MAX_SPEED, read_reg, diff, str);
  PRINT_REG(MIN_SPEED, read_reg, diff, str);
  PRINT_REG(KVAL_HOLD, read_reg, diff, str);
  PRINT_REG(KVAL_RUN, read_reg, diff, str);
  PRINT_REG(KVAL_ACC, read_reg, diff, str);
  PRINT_REG(KVAL_DEC, read_reg, diff, str);
  PRINT_REG(INT_SPD, read_reg, diff, str);
  PRINT_REG(ST_SLP, read_reg, diff, str);
  PRINT_REG(FN_SLP_ACC, read_reg, diff, str);
  PRINT_REG(FN_SLP_DEC, read_reg, diff, str);
  PRINT_REG(K_THERM, read_reg, diff, str);
  PRINT_REG(ADC_OUT, read_reg, diff, str);
  PRINT_REG(OCD_TH, read_reg, diff, str);
  PRINT_REG(STALL_TH, read_reg, diff, str);
  PRINT_REG(FS_SPD, read_reg, diff, str);
  PRINT_REG(STEP_MODE, read_reg, diff, str);
  PRINT_REG(ALARM_EN, read_reg, diff, str);
  PRINT_REG(CONFIG, read_reg, diff, str);
  PRINT_REG(STATUS, read_reg, diff, str);
}
#endif /* defined(DEBUG) */

#if defined(DEBUG)  
/**
  * @brief  Reads dSPIN internal writable registers and compares with the values in the code
  * @param  dSPIN_RegsStruct Configuration structure address (pointer to configuration structure)
  * @retval Bitmap with the bits, corresponding to the unmatched registers, set
  */
uint32_t dSPIN_Registers_Check(dSPIN_RegsStruct_TypeDef* dSPIN_RegsStruct)
{
  uint32_t result = 0;
  
  CHECK_REG(ABS_POS, result);
  CHECK_REG(EL_POS, result);
  CHECK_REG(MARK, result);
  CHECK_REG(ACC, result);
  CHECK_REG(DEC, result);
  CHECK_REG(MAX_SPEED, result);
  CHECK_REG(MIN_SPEED, result);
  CHECK_REG(KVAL_HOLD, result);
  CHECK_REG(KVAL_RUN, result);
  CHECK_REG(KVAL_ACC, result);
  CHECK_REG(KVAL_DEC, result);
  CHECK_REG(INT_SPD, result);
  CHECK_REG(ST_SLP, result);
  CHECK_REG(FN_SLP_ACC, result);
  CHECK_REG(FN_SLP_DEC, result);
  CHECK_REG(K_THERM, result);
  CHECK_REG(OCD_TH, result);
  CHECK_REG(STALL_TH, result);
  CHECK_REG(FS_SPD, result);
  CHECK_REG(STEP_MODE, result);
  CHECK_REG(ALARM_EN, result);
  CHECK_REG(CONFIG, result);
  
  return result;
}
#endif /* defined(DEBUG) */

/**
  * @brief Issues dSPIN NOP command.
  * @param None
  * @retval None
  */
void dSPIN_Nop(void)
{
	/* Send NOP operation code to dSPIN */
	dSPIN_Write_Byte(dSPIN_NOP);
}

/**
  * @brief  Issues dSPIN Set Param command.
  * @param  param dSPIN register address
  * @param  value to be set
  * @retval None
  */
void dSPIN_Set_Param(dSPIN_Registers_TypeDef param, uint32_t value)
{
	/* Send SetParam operation code to dSPIN */
	dSPIN_Write_Byte((uint8_t)dSPIN_SET_PARAM | (uint8_t)param);
	switch (param)
	{
		case dSPIN_ABS_POS: ;
		case dSPIN_MARK: ;
			/* Send parameter - byte 2 to dSPIN */
			dSPIN_Write_Byte((uint8_t)(value >> 16));
		case dSPIN_EL_POS: ;
		case dSPIN_ACC: ;
		case dSPIN_DEC: ;
		case dSPIN_MAX_SPEED: ;
		case dSPIN_MIN_SPEED: ;
		case dSPIN_FS_SPD: ;
		case dSPIN_INT_SPD: ;
		case dSPIN_CONFIG: ;
		case dSPIN_STATUS:
			/* Send parameter - byte 1 to dSPIN */
		   	dSPIN_Write_Byte((uint8_t)(value >> 8));
		default:
			/* Send parameter - byte 0 to dSPIN */
		   	dSPIN_Write_Byte((uint8_t)(value));
	}
}

/**
  * @brief  Issues dSPIN Get Param command.
  * @param  param dSPIN register address
  * @retval Register value - 1 to 3 bytes (depends on register)
  */
//uint32_t dSPIN_Get_Param(dSPIN_Registers_TypeDef param)
//{
//	uint32_t temp = 0;
//	uint32_t rx = 0;

//	/* Send GetParam operation code to dSPIN */
//	temp = dSPIN_Write_Byte((uint8_t)dSPIN_GET_PARAM | (uint8_t)param);
//	/* MSB which should be 0 */
//	temp = temp << 24;
//	rx |= temp;
//	switch (param)
//	{
//		case dSPIN_ABS_POS: ;
//		case dSPIN_MARK: ;
//		case dSPIN_SPEED:
//		   	temp = dSPIN_Write_Byte((uint8_t)(0x00));
//			temp = temp << 16;
//			rx |= temp;
//		case dSPIN_EL_POS: ;
//		case dSPIN_ACC: ;
//		case dSPIN_DEC: ;
//		case dSPIN_MAX_SPEED: ;
//		case dSPIN_MIN_SPEED: ;
//		case dSPIN_FS_SPD: ;
//		case dSPIN_INT_SPD: ;
//		case dSPIN_CONFIG: ;
//		case dSPIN_STATUS:
//		   	temp = dSPIN_Write_Byte((uint8_t)(0x00));
//			temp = temp << 8;
//			rx |= temp;
//		default:
//		   	temp = dSPIN_Write_Byte((uint8_t)(0x00));
//			rx |= temp;
//	}
//	return rx;
//}

/**
  * @brief  Issues dSPIN Run command.
  * @param  direction Movement direction (FWD, REV)
  * @param  speed over 3 bytes
  * @retval None
  */
void dSPIN_Run(dSPIN_Direction_TypeDef direction, uint32_t speed)
{
	/* Send RUN operation code to dSPIN */
	dSPIN_Write_Byte((uint8_t)dSPIN_RUN | (uint8_t)direction);
	/* Send speed - byte 2 data dSPIN */
	dSPIN_Write_Byte((uint8_t)(speed >> 16));
	/* Send speed - byte 1 data dSPIN */
	dSPIN_Write_Byte((uint8_t)(speed >> 8));
	/* Send speed - byte 0 data dSPIN */
	dSPIN_Write_Byte((uint8_t)(speed));
}

/**
  * @brief  Issues dSPIN Step Clock command.
  * @param  direction Movement direction (FWD, REV)
  * @retval None
  */
void dSPIN_Step_Clock(dSPIN_Direction_TypeDef direction)
{
	/* Send StepClock operation code to dSPIN */
	dSPIN_Write_Byte((uint8_t)dSPIN_STEP_CLOCK | (uint8_t)direction);
}

/**
  * @brief  Issues dSPIN Move command.
  * @param  direction mMovement direction
  * @param  n_step number of steps
  * @retval None
  */
void dSPIN_Move(dSPIN_Direction_TypeDef direction, uint32_t n_step)
{
	/* Send Move operation code to dSPIN */
	dSPIN_Write_Byte((uint8_t)dSPIN_MOVE | (uint8_t)direction);
	/* Send n_step - byte 2 data dSPIN */
	dSPIN_Write_Byte((uint8_t)(n_step >> 16));
	/* Send n_step - byte 1 data dSPIN */
	dSPIN_Write_Byte((uint8_t)(n_step >> 8));
	/* Send n_step - byte 0 data dSPIN */
	dSPIN_Write_Byte((uint8_t)(n_step));
}

/**
  * @brief  Issues dSPIN Go To command.
  * @param  abs_pos absolute position where requested to move
  * @retval None
  */
void dSPIN_Go_To(uint32_t abs_pos)
{
	/* Send GoTo operation code to dSPIN */
	dSPIN_Write_Byte(dSPIN_GO_TO);
	/* Send absolute position parameter - byte 2 data to dSPIN */
	dSPIN_Write_Byte((uint8_t)(abs_pos >> 16));
	/* Send absolute position parameter - byte 1 data to dSPIN */
	dSPIN_Write_Byte((uint8_t)(abs_pos >> 8));
	/* Send absolute position parameter - byte 0 data to dSPIN */
	dSPIN_Write_Byte((uint8_t)(abs_pos));
}

/**
  * @brief  Issues dSPIN Go To Dir command.
  * @param  direction movement direction
  * @param  abs_pos absolute position where requested to move
  * @retval None
  */
void dSPIN_Go_To_Dir(dSPIN_Direction_TypeDef direction, uint32_t abs_pos)
{
	/* Send GoTo_DIR operation code to dSPIN */
	dSPIN_Write_Byte((uint8_t)dSPIN_GO_TO_DIR | (uint8_t)direction);
	/* Send absolute position parameter - byte 2 data to dSPIN */
	dSPIN_Write_Byte((uint8_t)(abs_pos >> 16));
	/* Send absolute position parameter - byte 1 data to dSPIN */
	dSPIN_Write_Byte((uint8_t)(abs_pos >> 8));
	/* Send absolute position parameter - byte 0 data to dSPIN */
	dSPIN_Write_Byte((uint8_t)(abs_pos));
}

/**
  * @brief  Issues dSPIN Go Until command.
  * @param  action
  * @param  direction movement direction
  * @param  speed
  * @retval None
  */
void dSPIN_Go_Until(dSPIN_Action_TypeDef action, dSPIN_Direction_TypeDef direction, uint32_t speed)
{
	/* Send GoUntil operation code to dSPIN */
	dSPIN_Write_Byte((uint8_t)dSPIN_GO_UNTIL | (uint8_t)action | (uint8_t)direction);
	/* Send speed parameter - byte 2 data to dSPIN */
	dSPIN_Write_Byte((uint8_t)(speed >> 16));
	/* Send speed parameter - byte 1 data to dSPIN */
	dSPIN_Write_Byte((uint8_t)(speed >> 8));
	/* Send speed parameter - byte 0 data to dSPIN */
	dSPIN_Write_Byte((uint8_t)(speed));
}

/**
  * @brief  Issues dSPIN Release SW command.
  * @param  action
  * @param  direction movement direction
  * @retval None
  */
void dSPIN_Release_SW(dSPIN_Action_TypeDef action, dSPIN_Direction_TypeDef direction)
{
	/* Send ReleaseSW operation code to dSPIN */
	dSPIN_Write_Byte((uint8_t)dSPIN_RELEASE_SW | (uint8_t)action | (uint8_t)direction);
}

/**
  * @brief  Issues dSPIN Go Home command. (Shorted path to zero position)
  * @param  None
  * @retval None
  */
void dSPIN_Go_Home(void)
{
	/* Send GoHome operation code to dSPIN */
	dSPIN_Write_Byte(dSPIN_GO_HOME);
}

/**
  * @brief  Issues dSPIN Go Mark command.
  * @param  None
  * @retval None
  */
void dSPIN_Go_Mark(void)
{
	/* Send GoMark operation code to dSPIN */
	dSPIN_Write_Byte(dSPIN_GO_MARK);
}

/**
  * @brief  Issues dSPIN Reset Pos command.
  * @param  None
  * @retval None
  */
void dSPIN_Reset_Pos(void)
{
	/* Send ResetPos operation code to dSPIN */
	dSPIN_Write_Byte(dSPIN_RESET_POS);
}

/**
  * @brief  Issues dSPIN Reset Device command.
  * @param  None
  * @retval None
  */
void dSPIN_Reset_Device(void)
{
	/* Send ResetDevice operation code to dSPIN */
	dSPIN_Write_Byte(dSPIN_RESET_DEVICE);
}

/**
  * @brief  Issues dSPIN Soft Stop command.
  * @param  None
  * @retval None
  */
void dSPIN_Soft_Stop(void)
{
	/* Send SoftStop operation code to dSPIN */
	dSPIN_Write_Byte(dSPIN_SOFT_STOP);
}

/**
  * @brief  Issues dSPIN Hard Stop command.
  * @param  None
  * @retval None
  */
void dSPIN_Hard_Stop(void)
{
	/* Send HardStop operation code to dSPIN */
	dSPIN_Write_Byte(dSPIN_HARD_STOP);
}

/**
  * @brief  Issues dSPIN Soft HiZ command.
  * @param  None
  * @retval None
  */
void dSPIN_Soft_HiZ(void)
{
	/* Send SoftHiZ operation code to dSPIN */
	dSPIN_Write_Byte(dSPIN_SOFT_HIZ);
}

/**
  * @brief  Issues dSPIN Hard HiZ command.
  * @param  None
  * @retval None
  */
void dSPIN_Hard_HiZ(void)
{
	/* Send HardHiZ operation code to dSPIN */
	dSPIN_Write_Byte(dSPIN_HARD_HIZ);
}

/**
  * @brief  Issues dSPIN Get Status command.
  * @param  None
  * @retval Status Register content
  */
//uint16_t dSPIN_Get_Status(void)
//{
//	uint16_t temp = 0;
//	uint16_t rx = 0;

//	/* Send GetStatus operation code to dSPIN */
//	dSPIN_Write_Byte(dSPIN_GET_STATUS);
//	/* Send zero byte / receive MSByte from dSPIN */
//	temp = dSPIN_Write_Byte((uint8_t)(0x00));
//	temp = temp << 8;
//	rx |= temp;
//	/* Send zero byte / receive LSByte from dSPIN */
//	temp = dSPIN_Write_Byte((uint8_t)(0x00));
//	rx |= temp;
//	return rx;
//}


/**
  * @brief  Checks if the dSPIN is Busy by SPI - Busy flag bit in Status Register.
  * @param  None
  * @retval one if chip is busy, otherwise zero
  */
//uint8_t dSPIN_Busy_SW(void)
//{
//	if(!(dSPIN_Get_Status() & dSPIN_STATUS_BUSY)) return 0x01;
//	else return 0x00;
//}

/**
  * @brief  Transmits/Receives one byte to/from dSPIN over SPI.
  * @param  byte Transmited byte
  * @retval Received byte
  */
void dSPIN_Write_Byte(uint8_t byte)
{
	uint8_t byteRX;
	/* nSS signal activation - low */
	HAL_GPIO_WritePin(dSPIN_nSS_Port, dSPIN_nSS_Pin,GPIO_PIN_RESET);
	/* SPI byte send */ 
  HAL_SPI_TransmitReceive(&hspi1,&byte,&byteRX,1,1000);
	/* Wait for SPIx Busy flag */
	while (__HAL_SPI_GET_FLAG(&hspi1, SPI_FLAG_BSY) != RESET);
	/* nSS signal deactivation - high */
	HAL_GPIO_WritePin(dSPIN_nSS_Port, dSPIN_nSS_Pin,GPIO_PIN_SET);
}

/**
  * @brief  Transmits/Receives several bytes to dSPIN over SPI
  * @param  pTxByte pTxBytePointer to TX bytes
  * @param  pRxByte Pointer to RX bytes
  * @param  nBytes Number of TX = RX bytes
  * @retval None
  */
void dSPIN_Write_Daisy_Chain_Bytes(uint8_t *pTxByte, uint8_t *pRxByte, uint8_t nBytes)
{
        uint32_t index;
	/* nSS signal activation - low */
	HAL_GPIO_WritePin(dSPIN_nSS_Port, dSPIN_nSS_Pin,GPIO_PIN_RESET);
	/* SPI byte send */
        for (index = 0; index < nBytes; index++)
        {
         HAL_SPI_TransmitReceive(&hspi1,pTxByte,pRxByte,1,1000);
          /* Wait for SPIx Busy flag */
	  while (__HAL_SPI_GET_FLAG(&hspi1, SPI_FLAG_BSY) != RESET){}
          *pRxByte = HAL_SPI_TransmitReceive(&hspi1,pTxByte,pRxByte,1,1000);
          pTxByte++;
          pRxByte++;
        }
	/* nSS signal deactivation - high */
	HAL_GPIO_WritePin(dSPIN_nSS_Port, dSPIN_nSS_Pin,GPIO_PIN_SET);
}

/**
  * @brief  Issues dSPIN Set Param command to each device (slave).
  * @param  slaves_number number of slaves
  * @param  pParam Pointer to an array of dSPIN register address
  * @param  pValue Pointer to an array of dSPIN parameter value
  * @retval None
  */
void dSPIN_All_Slaves_Set_Param(uint8_t slaves_number, uint8_t *pParam, uint32_t *pValue)
{
  uint32_t i;
  uint8_t maxArgumentNbBytes = 0;

  for (i = 0; i < slaves_number; i++)
  {
     switch (*pParam)
     {
	case dSPIN_ABS_POS: ;
	case dSPIN_MARK: ;
	case dSPIN_SPEED:
                 spiTxBursts[0][i] = *pParam;
                 spiTxBursts[1][i] = (uint8_t)(*pValue >> 16);
                 spiTxBursts[2][i] = (uint8_t)(*pValue >> 8);
	         maxArgumentNbBytes = 3;
                 break;
        case dSPIN_EL_POS: ;
	case dSPIN_ACC: ;
	case dSPIN_DEC: ;
	case dSPIN_MAX_SPEED: ;
	case dSPIN_MIN_SPEED: ;
	case dSPIN_FS_SPD: ;
	case dSPIN_INT_SPD: ;
	case dSPIN_CONFIG: ;
	case dSPIN_STATUS:
                 spiTxBursts[0][i] = dSPIN_NOP;
                 spiTxBursts[1][i] = *pParam;
                 spiTxBursts[2][i] = (uint8_t)(*pValue >> 8);           
                 if (maxArgumentNbBytes < 2)
                 {
                    maxArgumentNbBytes = 2;
                 }
                 break;
	default:
                 spiTxBursts[0][i] = dSPIN_NOP;
                 spiTxBursts[1][i] = dSPIN_NOP;
                 spiTxBursts[2][i] = *pParam;
                 if (maxArgumentNbBytes < 1)
                 {                 
                    maxArgumentNbBytes = 1;
                 }
     }
     spiTxBursts[3][i] = (uint8_t)(*pValue);
     pParam++;
     pValue++;
  }
  for (i = dSPIN_CMD_ARG_MAX_NB_BYTES-1-maxArgumentNbBytes; i < dSPIN_CMD_ARG_MAX_NB_BYTES; i++)
  {
     dSPIN_Write_Daisy_Chain_Bytes(&spiTxBursts[i][0], &spiRxBursts[i][0], slaves_number);
  }
}

/**
  * @brief  Issues dSPIN Get Param command to each device (slave).
  * @param  slaves_number number of slaves
  * @param  pParam Pointer to an array of dSPIN register address
  * @param  pValue Pointer to an array of dSPIN parameter value
  * @retval None
  */
void dSPIN_All_Slaves_Get_Param(uint8_t slaves_number, uint8_t *pParam, uint32_t *pValue)
{
  uint32_t i;
  uint8_t maxArgumentNbBytes = 0;
  
  for (i = 0; i < slaves_number; i++)
  {
     switch (*pParam)
     {
	case dSPIN_ABS_POS: ;
	case dSPIN_MARK: ;
	case dSPIN_SPEED:
                 spiTxBursts[0][i] = ((uint8_t)dSPIN_GET_PARAM )| (*pParam);
                 spiTxBursts[1][i] = dSPIN_NOP;
                 spiTxBursts[2][i] = dSPIN_NOP;
	         maxArgumentNbBytes = 3;
                 break;
        case dSPIN_EL_POS: ;
	case dSPIN_ACC: ;
	case dSPIN_DEC: ;
	case dSPIN_MAX_SPEED: ;
	case dSPIN_MIN_SPEED: ;
	case dSPIN_FS_SPD: ;
	case dSPIN_INT_SPD: ;
	case dSPIN_CONFIG: ;
	case dSPIN_STATUS:
                 spiTxBursts[0][i] = dSPIN_NOP;
                 spiTxBursts[1][i] = ((uint8_t)dSPIN_GET_PARAM )| (*pParam);
                 spiTxBursts[2][i] = dSPIN_NOP;
                 if (maxArgumentNbBytes < 2)
                 {
                    maxArgumentNbBytes = 2;
                 }
                 break;
	default:
                 spiTxBursts[0][i] = dSPIN_NOP;
                 spiTxBursts[1][i] = dSPIN_NOP;
                 spiTxBursts[2][i] = ((uint8_t)dSPIN_GET_PARAM )| (*pParam);
                 if (maxArgumentNbBytes < 1)
                 {                 
                    maxArgumentNbBytes = 1;
                 }
     }
     spiTxBursts[3][i] = dSPIN_NOP;
     spiRxBursts[1][i] = 0;
     spiRxBursts[2][i] = 0;
     spiRxBursts[3][i] = 0;
     pParam++;
  }
  for (i = dSPIN_CMD_ARG_MAX_NB_BYTES-1-maxArgumentNbBytes; i < dSPIN_CMD_ARG_MAX_NB_BYTES; i++)
  {
     dSPIN_Write_Daisy_Chain_Bytes(&spiTxBursts[i][0], &spiRxBursts[i][0], slaves_number);
  }
  for (i = 0; i < slaves_number; i++)
  {
    *pValue = (spiRxBursts[1][i] << 16) | (spiRxBursts[2][i] << 8) | (spiRxBursts[3][i]);
    pValue++;
  }
}

/**
  * @brief  Configures dSPIN slaves internal registers with values in the config structure.
  * @param  slaves_number number of slaves 
  * @param  dSPIN_RegsStructArray Configuration structure array address (pointer to configuration structure array)
  * @retval None
  */
void dSPIN_All_Slaves_Registers_Set(uint8_t slaves_number, dSPIN_RegsStruct_TypeDef *dSPIN_RegsStructArray)
{
    uint32_t i;
    
    /* ABS_POS */
    for (i=0;i<slaves_number;i++)
    {
      arrayTxBytes[i] = dSPIN_ABS_POS;
      arrayValues[i] = dSPIN_RegsStructArray[i].ABS_POS;
    }
    dSPIN_All_Slaves_Set_Param(slaves_number, arrayTxBytes, arrayValues);
    /* EL_POS */
    for (i=0;i<slaves_number;i++)
    {
      arrayTxBytes[i] = dSPIN_EL_POS;
      arrayValues[i] = dSPIN_RegsStructArray[i].EL_POS;
    }
    dSPIN_All_Slaves_Set_Param(slaves_number, arrayTxBytes, arrayValues);
    /* MARK */
    for (i=0;i<slaves_number;i++)
    {
      arrayTxBytes[i] = dSPIN_MARK;
      arrayValues[i] = dSPIN_RegsStructArray[i].MARK;
    }
    dSPIN_All_Slaves_Set_Param(slaves_number, arrayTxBytes, arrayValues);
    /* ACC */
    for (i=0;i<slaves_number;i++)
    {
      arrayTxBytes[i] = dSPIN_ACC;
      arrayValues[i] = dSPIN_RegsStructArray[i].ACC;
    }
    dSPIN_All_Slaves_Set_Param(slaves_number, arrayTxBytes, arrayValues);
    /* DEC*/
    for (i=0;i<slaves_number;i++)
    {
      arrayTxBytes[i] = dSPIN_DEC;
      arrayValues[i] = dSPIN_RegsStructArray[i].DEC;
    }
    dSPIN_All_Slaves_Set_Param(slaves_number, arrayTxBytes, arrayValues);  
    /* MAX_SPEED */
    for (i=0;i<slaves_number;i++)
    {
      arrayTxBytes[i] = dSPIN_MAX_SPEED;
      arrayValues[i] = dSPIN_RegsStructArray[i].MAX_SPEED;
    }
    dSPIN_All_Slaves_Set_Param(slaves_number, arrayTxBytes, arrayValues);
    /* MIN_SPEED */
    for (i=0;i<slaves_number;i++)
    {
      arrayTxBytes[i] = dSPIN_MIN_SPEED;
      arrayValues[i] = dSPIN_RegsStructArray[i].MIN_SPEED;
    }
    dSPIN_All_Slaves_Set_Param(slaves_number, arrayTxBytes, arrayValues);
    /* FS_SPD */
    for (i=0;i<slaves_number;i++)
    {
      arrayTxBytes[i] = dSPIN_FS_SPD;
      arrayValues[i] = dSPIN_RegsStructArray[i].FS_SPD;
    }
    dSPIN_All_Slaves_Set_Param(slaves_number, arrayTxBytes, arrayValues);
    /* KVAL_HOLD */
    for (i=0;i<slaves_number;i++)
    {
      arrayTxBytes[i] = dSPIN_KVAL_HOLD;
      arrayValues[i] = dSPIN_RegsStructArray[i].KVAL_HOLD;
    }
    dSPIN_All_Slaves_Set_Param(slaves_number, arrayTxBytes, arrayValues);
    /* KVAL_RUN */
    for (i=0;i<slaves_number;i++)
    {
      arrayTxBytes[i] = dSPIN_KVAL_RUN;
      arrayValues[i] = dSPIN_RegsStructArray[i].KVAL_RUN;
    }
    dSPIN_All_Slaves_Set_Param(slaves_number, arrayTxBytes, arrayValues);
    /* KVAL_ACC */
    for (i=0;i<slaves_number;i++)
    {
      arrayTxBytes[i] = dSPIN_KVAL_ACC;
      arrayValues[i] = dSPIN_RegsStructArray[i].KVAL_ACC;
    }
    dSPIN_All_Slaves_Set_Param(slaves_number, arrayTxBytes, arrayValues); 
    /* KVAL_DEC */
    for (i=0;i<slaves_number;i++)
    {
      arrayTxBytes[i] = dSPIN_KVAL_DEC;
      arrayValues[i] = dSPIN_RegsStructArray[i].KVAL_DEC;
    }
    dSPIN_All_Slaves_Set_Param(slaves_number, arrayTxBytes, arrayValues);
    /* INT_SPD */
    for (i=0;i<slaves_number;i++)
    {
      arrayTxBytes[i] = dSPIN_INT_SPD;
      arrayValues[i] = dSPIN_RegsStructArray[i].INT_SPD;
    }
    dSPIN_All_Slaves_Set_Param(slaves_number, arrayTxBytes, arrayValues);
    /* ST_SLP */
    for (i=0;i<slaves_number;i++)
    {
      arrayTxBytes[i] = dSPIN_ST_SLP;
      arrayValues[i] = dSPIN_RegsStructArray[i].ST_SLP;
    }
    dSPIN_All_Slaves_Set_Param(slaves_number, arrayTxBytes, arrayValues);
    /* FN_SLP_ACC */
    for (i=0;i<slaves_number;i++)
    {
      arrayTxBytes[i] = dSPIN_FN_SLP_ACC;
      arrayValues[i] = dSPIN_RegsStructArray[i].FN_SLP_ACC;
    }
    dSPIN_All_Slaves_Set_Param(slaves_number, arrayTxBytes, arrayValues);
    /* FN_SLP_DEC */
    for (i=0;i<slaves_number;i++)
    {
      arrayTxBytes[i] = dSPIN_FN_SLP_DEC;
      arrayValues[i] = dSPIN_RegsStructArray[i].FN_SLP_DEC;
    }
    dSPIN_All_Slaves_Set_Param(slaves_number, arrayTxBytes, arrayValues);
    /* K_THERM */
    for (i=0;i<slaves_number;i++)
    {
      arrayTxBytes[i] = dSPIN_K_THERM;
      arrayValues[i] = dSPIN_RegsStructArray[i].K_THERM;
    }
    dSPIN_All_Slaves_Set_Param(slaves_number, arrayTxBytes, arrayValues);
    /* STALL_TH */
    for (i=0;i<slaves_number;i++)
    {
      arrayTxBytes[i] = dSPIN_STALL_TH;
      arrayValues[i] = dSPIN_RegsStructArray[i].STALL_TH;
    }
    dSPIN_All_Slaves_Set_Param(slaves_number, arrayTxBytes, arrayValues);
    /* OCD_TH */
     for (i=0;i<slaves_number;i++)
    {
      arrayTxBytes[i] = dSPIN_OCD_TH;
      arrayValues[i] = dSPIN_RegsStructArray[i].OCD_TH;
    }
    dSPIN_All_Slaves_Set_Param(slaves_number, arrayTxBytes, arrayValues);
    /* STEP_MODE */
    for (i=0;i<slaves_number;i++)
    {
      arrayTxBytes[i] = dSPIN_STEP_MODE;
      arrayValues[i] = dSPIN_RegsStructArray[i].STEP_MODE;
    }
    dSPIN_All_Slaves_Set_Param(slaves_number, arrayTxBytes, arrayValues);
    /* ALARM_EN */
    for (i=0;i<slaves_number;i++)
    {
      arrayTxBytes[i] = dSPIN_ALARM_EN;
      arrayValues[i] = dSPIN_RegsStructArray[i].ALARM_EN;
    }
    dSPIN_All_Slaves_Set_Param(slaves_number, arrayTxBytes, arrayValues);
    /* CONFIG */
    for (i=0;i<slaves_number;i++)
    {
      arrayTxBytes[i] = dSPIN_CONFIG;
      arrayValues[i] = dSPIN_RegsStructArray[i].CONFIG;
    }
    dSPIN_All_Slaves_Set_Param(slaves_number, arrayTxBytes, arrayValues); 
}

/**
  * @brief  Issues dSPIN Move command to one slave device
  * @param  slaves_number number of slaves 
  * @param  slaveNumber slave number
  * @param  direction movement direction
  * @param  n_step number of steps
  * @retval None
  */
void dSPIN_One_Slave_Move(uint8_t slaves_number, uint8_t slaveNumber, dSPIN_Direction_TypeDef direction, uint32_t n_step)
{
  dSPIN_One_Slave_Send_Command(slaveNumber, slaves_number, (uint8_t)dSPIN_MOVE | (uint8_t)direction, n_step);
}

/**
  * @brief  Issues dSPIN Run command to one slave device
  * @param  slaveNumber slave number
  * @param  slaves_number number of slaves 
  * @param  direction movement direction
  * @param  speed
  * @retval None
  */
void dSPIN_One_Slave_Run(uint8_t slaveNumber, uint8_t slaves_number, dSPIN_Direction_TypeDef direction, uint32_t speed)
{
  dSPIN_One_Slave_Send_Command(slaveNumber, slaves_number, (uint8_t)dSPIN_RUN | (uint8_t)direction, speed);
}

/**
  * @brief  Issues a command to one slave device
  * @param  slaveNumber slave number
  * @param  slaves_number number of slaves 
  * @param  param command to issue
  * @param  value command argument 
  * @retval None
  */
void dSPIN_One_Slave_Send_Command(uint8_t slaveNumber, uint8_t slaves_number, uint8_t param, uint32_t value)
{
  uint32_t i;
  
  for (i = 0; i < slaves_number; i++)
  {
    if (i == slaveNumber)
    {
      spiTxBursts[0][i] = (param);
      spiTxBursts[1][i] = (uint8_t)(value >> 16);
      spiTxBursts[2][i] = (uint8_t)(value >> 8);
      spiTxBursts[3][i] = (uint8_t)(value);
    }
    else
    {
      spiTxBursts[0][i] = dSPIN_NOP;
      spiTxBursts[1][i] = dSPIN_NOP;
      spiTxBursts[2][i] = dSPIN_NOP;
      spiTxBursts[3][i] = dSPIN_NOP;     
    }
  }
  for (i = dSPIN_CMD_ARG_MAX_NB_BYTES-dSPIN_CMD_ARG_NB_BYTES_MOVE; i < dSPIN_CMD_ARG_MAX_NB_BYTES; i++)
  {
     dSPIN_Write_Daisy_Chain_Bytes(&spiTxBursts[i][0], &spiRxBursts[i][0], slaves_number);
  }
}

/**
  * @brief Issues commands to the slave devices for synchronous execution
  * @param slaves_number number of slaves
  * @param pParam Pointer to an array of dSPIN commands
  * @param pValue Pointer to an array of dSPIN arguments

  * @retval None
  */
void dSPIN_All_Slaves_Send_Command(uint8_t slaves_number, uint8_t *pParam, uint32_t *pValue)
{
  uint32_t i;
  uint8_t maxArgumentNbBytes = 0;

  for (i = 0; i < slaves_number; i++)
  {
     switch ((*pParam) & DAISY_CHAIN_COMMAND_MASK)
     {
        case dSPIN_RUN: ;
        case dSPIN_MOVE: ;
        case dSPIN_GO_TO: ;
        case dSPIN_GO_TO_DIR: ;
        case dSPIN_GO_UNTIL: ;
				case dSPIN_GO_UNTIL_ACT_CPY:
                 spiTxBursts[0][i] = *pParam;
                 spiTxBursts[1][i] = (uint8_t)(*pValue >> 16);
                 spiTxBursts[2][i] = (uint8_t)(*pValue >> 8);
                 spiTxBursts[3][i] = (uint8_t)(*pValue);
	         maxArgumentNbBytes = 3;
                 break;
	default:
                 spiTxBursts[0][i] = dSPIN_NOP;
                 spiTxBursts[1][i] = dSPIN_NOP;
                 spiTxBursts[2][i] = dSPIN_NOP;
                 spiTxBursts[3][i] = *pParam;
     }
     pParam++;
     pValue++;
  }
  for (i = dSPIN_CMD_ARG_MAX_NB_BYTES-1-maxArgumentNbBytes; i < dSPIN_CMD_ARG_MAX_NB_BYTES; i++)
  {
     dSPIN_Write_Daisy_Chain_Bytes(&spiTxBursts[i][0], &spiRxBursts[i][0], slaves_number);
  }
}

/**
  * @brief Issues dSPIN Get Status command to each device (slave)
  * @param slaves_number number of slaves
  * @param pValue pointer to an array of Status Register content
  * @retval None
  */
void dSPIN_All_Slaves_Get_Status(uint8_t slaves_number, uint32_t *pValue)
{
  uint32_t i;

  for (i = 0; i < slaves_number; i++)
  {
     spiTxBursts[0][i] = dSPIN_GET_STATUS;
     spiTxBursts[1][i] = dSPIN_NOP;
     spiTxBursts[2][i] = dSPIN_NOP;
     spiRxBursts[1][i] = 0;
     spiRxBursts[2][i] = 0;
  }
  for (i = 0; i < dSPIN_CMD_ARG_NB_BYTES_GET_STATUS+dSPIN_RSP_NB_BYTES_GET_STATUS; i++)
  {
     dSPIN_Write_Daisy_Chain_Bytes(&spiTxBursts[i][0], &spiRxBursts[i][0], slaves_number);
  }
  for (i = 0; i < slaves_number; i++)
  {
    *pValue = (spiRxBursts[1][i] << 8) | (spiRxBursts[2][i]);
    pValue++;
  }
}

/**
  * @brief  Checks if one of the dSPIN device (slave) is Busy by SPI - Busy flag bit in Status Register.
  * @param  slaves_number number of slaves
  * @retval one if there is a busy chip, otherwise zero
  */
uint8_t dSPIN_One_Or_More_Slaves_Busy_SW(uint8_t slaves_number)
{
  uint32_t i;
  uint16_t status;
  dSPIN_All_Slaves_Get_Status(slaves_number, arrayValues);
  for (i = 0; i < slaves_number; i++)
  {
    status |= arrayValues[i];
  }
  if(!(status & dSPIN_STATUS_BUSY)) return 0x01;
  else return 0x00;
}

/** @} */  
/******************* (C) COPYRIGHT 2013 STMicroelectronics *****END OF FILE****/
