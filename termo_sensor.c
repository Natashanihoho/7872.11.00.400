#include "main.h"

extern UART_HandleTypeDef huart1;


void Delay_us(uint16_t us);

int8_t getTemperature(uint8_t *ScratchpadData)
{
	uint16_t regTemp;
  int16_t temp;
	
	regTemp = (ScratchpadData[1] << 8) | ScratchpadData[0];
	regTemp = regTemp >> 4;
	
	if (regTemp & (1<<11))
	{
		regTemp = ~regTemp + 1;
    temp = 0 - regTemp;
	}
	else temp = regTemp;
		
  return temp;
}

uint8_t ds18b20_Reset(void)
{
  uint8_t status;
  GPIOA->ODR &= ~GPIO_ODR_ODR15;//низкий уровень
  Delay_us(485);//задержка как минимум на 480 микросекунд
  GPIOA->ODR |= GPIO_ODR_ODR15;//высокий уровень
  Delay_us(65);//задержка как минимум на 60 микросекунд
  status = GPIOA->IDR & GPIO_IDR_IDR15;//проверяем уровень
  Delay_us(500);//задержка как минимум на 480 микросекунд
  return (status ? 1 : 0);//вернём результат
}

uint8_t ds18b20_ReadBit(void)
{
  uint8_t bit = 0;
  GPIOA->ODR &= ~GPIO_ODR_ODR15;//низкий уровень
  Delay_us(2);
	GPIOA->ODR |= GPIO_ODR_ODR15;//высокий уровень
  Delay_us(2);
	bit = (GPIOA->IDR & GPIO_IDR_IDR15 ? 1 : 0);//проверяем уровень
	Delay_us(45);
	return bit;
}

uint8_t ds18b20_ReadByte(void)
{
  uint8_t data = 0;
  for (uint8_t i = 0; i <= 7; i++)
  data += ds18b20_ReadBit() << i;
  return data;
}

void ds18b20_WriteBit(uint8_t bit)
{
  GPIOA->ODR &= ~GPIO_ODR_ODR15;
  Delay_us(bit ? 3 : 65);
  GPIOA->ODR |= GPIO_ODR_ODR15;
  Delay_us(bit ? 65 : 3);
}

void ds18b20_WriteByte(uint8_t dt)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    ds18b20_WriteBit(dt >> i & 1);
    Delay_us(5);
  }
}

uint8_t ds18b20_Init(void)
{
  if(ds18b20_Reset()) return 1;
  ds18b20_WriteByte(0xCC);  // DS1820_SKIP_ROM 
  ds18b20_WriteByte(0x4E);  // DS1820_WRITE_SCRATCHPAD
	ds18b20_WriteByte(0x64);  // Th
  ds18b20_WriteByte(0x9E);  // Tl
	ds18b20_WriteByte(0x1F);  // RegConf (9-bit resol)
  return 0;
}

void ds18b20_MeasureTemperCmd(void)
{
  ds18b20_Reset();
  ds18b20_WriteByte(0xCC);
  ds18b20_WriteByte(0x44);
}

uint8_t ds18b20_ReadScratcpad(uint8_t *Data)
{
  uint8_t i;
  ds18b20_Reset();
  ds18b20_WriteByte(0xCC);
  ds18b20_WriteByte(0xBE);
  for(i=0;i<9;i++)
  {
    Data[i] = ds18b20_ReadByte();
	}
	if(Data[2] != 0x64 || Data[3] != 0x9E || Data[4] != 0x1F)
		return 1;
	return 0;
}

