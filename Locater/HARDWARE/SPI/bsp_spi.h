/*********************************************************/
//
//
//
/*********************************************************/
#ifdef BSP_SPI
	#define BSP_SPI_EXT
#else
	#define BSP_SPI_EXT extern
#endif

#ifndef BSP_SPI_H
	#define BSP_SPI_H
	
#include "sys.h"

BSP_SPI_EXT void Init_SPI1(void);
BSP_SPI_EXT uint8_t SPI1_RW_Byte(uint8_t byte);
BSP_SPI_EXT void Init_SPI2(void);
BSP_SPI_EXT uint8_t SPI2_RW_Byte(uint8_t byte);

#endif

