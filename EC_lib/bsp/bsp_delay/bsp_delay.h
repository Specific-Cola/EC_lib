//=====================================================================================================
// bsp_delay.h
//=====================================================================================================
//
//       IRobot  EC_lib
//
//GitHub: https://github.com/Specific_Cola
// question:  specificcola@proton.me
// Date			Author			Notes
// 
//
//=====================================================================================================
#ifndef BSP_DELAY_H__
#define BSP_DELAY_H__
#include "struct_typedef.h"

extern void delayInit(void);
extern void delayUs(uint16_t nus);
extern void delayMs(uint16_t nms);
#endif

