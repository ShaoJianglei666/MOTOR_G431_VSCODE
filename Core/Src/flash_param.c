/**********************************************************
flash_param.c：用户flash参数相关操作
**********************************************************/

#include "stm32g4xx_hal.h"
#include "flash_param.h"

/**********************************************************
static uint32_t GetPage(uint32_t Addr)
Function: Gets the page of a given address
param：Address of the FLASH Memory
return：The page of a given address
**********************************************************/
static uint32_t GetPage(uint32_t Addr)
{
	return (Addr - FLASH_BASE) / FLASH_PAGE_SIZE;
}

/**********************************************************
void FlashParamRead(FlashParam_t *param)
Function: 从FLASH读取用户参数
**********************************************************/
void FlashParamRead(FlashParam_t *param)
{
	uint16_t i;
	uint16_t size;
	uint32_t *psrc;
	uint32_t *pdst;
	
	psrc = (uint32_t *)FLASH_PARAM_START_ADDR;
	pdst = (uint32_t *)param;
	size = sizeof(FlashParam_t)/4;

	for(i = 0; i < size; i++)
	{
		*pdst = *psrc;
		psrc++;
		pdst++;
	}
}

/**********************************************************
void FlashParamWrite(FlashParam_t *param)
Function: 将用户参数写入FLASH
**********************************************************/
void FlashParamWrite(FlashParam_t *param)
{
	uint16_t i;
	uint16_t size;
	uint64_t *psrc;
	uint64_t *pdst;
	uint32_t PAGEError;
	FLASH_EraseInitTypeDef EraseInitStruct;

	PAGEError = 0;
	psrc = (uint64_t *)param;
	pdst = (uint64_t *)FLASH_PARAM_START_ADDR;
	size = sizeof(FlashParam_t)/8;

  	HAL_FLASH_Unlock();
	
	EraseInitStruct.TypeErase	= FLASH_TYPEERASE_PAGES;
	EraseInitStruct.Page = GetPage(FLASH_PARAM_START_ADDR);
	EraseInitStruct.NbPages 	= 1;
	HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError);

	for(i = 0; i < size; i++)
	{
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, (uint32_t)pdst, (uint64_t)(*psrc));
		psrc++;
		pdst++;
	}
	
	HAL_FLASH_Lock();

}

