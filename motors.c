#include "motors.h"
#include "main.h"

extern dSPIN_RegsStruct_TypeDef dSPIN_RegsStructArray[NUMBER_OF_SLAVES];

extern uint8_t daisy_chain;
extern uint8_t number_of_slaves;

extern uint8_t commandArray[NUMBER_OF_SLAVES];
extern uint32_t argumentArray[NUMBER_OF_SLAVES];

dSPIN_RegsStruct_TypeDef dSPIN_RegsStruct;

int i;

double   MAX_SPEED[NUMBER_OF_SLAVES]; //= dSPIN_DC_CONF_PARAM_MAX_SPEED;			
double   ACC[NUMBER_OF_SLAVES];// = dSPIN_DC_CONF_PARAM_ACC;
double   DEC[NUMBER_OF_SLAVES]; //= dSPIN_DC_CONF_PARAM_DEC;
double   FS_SPD[NUMBER_OF_SLAVES]; //= dSPIN_DC_CONF_PARAM_FS_SPD;

#if defined(L6470)
  double   KVAL_HOLD[NUMBER_OF_SLAVES];// = dSPIN_DC_CONF_PARAM_KVAL_HOLD;
  double   KVAL_RUN[NUMBER_OF_SLAVES]; //= dSPIN_DC_CONF_PARAM_KVAL_RUN;
  double   KVAL_ACC[NUMBER_OF_SLAVES];// = dSPIN_DC_CONF_PARAM_KVAL_ACC;
  double   KVAL_DEC[NUMBER_OF_SLAVES]; //= dSPIN_DC_CONF_PARAM_KVAL_DEC;
  double   INT_SPD[NUMBER_OF_SLAVES]; //= dSPIN_DC_CONF_PARAM_INT_SPD;
  double   ST_SLP[NUMBER_OF_SLAVES];// = dSPIN_DC_CONF_PARAM_ST_SLP;
  double   FN_SLP_ACC[NUMBER_OF_SLAVES];//= dSPIN_DC_CONF_PARAM_FN_SLP_ACC;
  double   FN_SLP_DEC[NUMBER_OF_SLAVES];// = dSPIN_DC_CONF_PARAM_FN_SLP_DEC;
  double   K_THERM[NUMBER_OF_SLAVES]; //= dSPIN_DC_CONF_PARAM_K_THERM;
  double   STALL_TH[NUMBER_OF_SLAVES]; //= dSPIN_DC_CONF_PARAM_STALL_TH;
  uint8_t  OCD_TH[NUMBER_OF_SLAVES]; //= dSPIN_DC_CONF_PARAM_OCD_TH;
  uint8_t  ALARM_EN[NUMBER_OF_SLAVES]; //= dSPIN_DC_CONF_PARAM_ALARM_EN;
  /* OR-ed definitions */
  double   MIN_SPEED[NUMBER_OF_SLAVES]; //= dSPIN_DC_CONF_PARAM_MIN_SPEED;
  uint16_t LSPD_BIT[NUMBER_OF_SLAVES]; //= dSPIN_DC_CONF_PARAM_LSPD_BIT;
  uint8_t  STEP_MODE[NUMBER_OF_SLAVES]; //= dSPIN_DC_CONF_PARAM_STEP_MODE;
  uint8_t  SYNC_MODE[NUMBER_OF_SLAVES]; //= dSPIN_DC_CONF_PARAM_SYNC_MODE;
  uint16_t CONFIG_CLOCK_SETTING[NUMBER_OF_SLAVES]; //= dSPIN_DC_CONF_PARAM_CLOCK_SETTING;
  uint16_t CONFIG_SW_MODE[NUMBER_OF_SLAVES]; //= dSPIN_DC_CONF_PARAM_SW_MODE;
  uint16_t CONFIG_OC_SD[NUMBER_OF_SLAVES]; //= dSPIN_DC_CONF_PARAM_OC_SD;
  uint16_t CONFIG_SR[NUMBER_OF_SLAVES]; //= dSPIN_DC_CONF_PARAM_SR;
  uint16_t CONFIG_VS_COMP[NUMBER_OF_SLAVES]; //= dSPIN_DC_CONF_PARAM_VS_COMP;
  uint16_t CONFIG_PWM_DIV[NUMBER_OF_SLAVES];// = dSPIN_DC_CONF_PARAM_PWM_DIV;
  uint16_t CONFIG_PWM_MUL[NUMBER_OF_SLAVES]; //= dSPIN_DC_CONF_PARAM_PWM_MUL;
#endif /* defined(L6470) */ 

void InitMot(void)
{
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
					
}

void StartMot1(void)
{
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
}

