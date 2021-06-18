#include "L6470.h"

extern SPI_HandleTypeDef hspi1;

uint8_t spiTxBursts[dSPIN_CMD_ARG_MAX_NB_BYTES][NUMBER_OF_SLAVES];
uint8_t spiRxBursts[dSPIN_CMD_ARG_MAX_NB_BYTES][NUMBER_OF_SLAVES];
uint8_t arrayTxBytes[NUMBER_OF_SLAVES];
uint32_t arrayValues[NUMBER_OF_SLAVES];

dSPIN_RegsStruct_TypeDef dSPIN_RegsStructArray[NUMBER_OF_SLAVES];

uint8_t daisy_chain = DAISY_CHAIN;
uint8_t number_of_slaves = NUMBER_OF_SLAVES;

uint8_t commandArray[NUMBER_OF_SLAVES];
uint32_t argumentArray[NUMBER_OF_SLAVES];


