/*
 * pwr_ctrl.h
 * 
 * Power Mode Control Library 
 * for STM32F10x Microcontroller Family
 *
 * Can be used to put CPU into Standby mode
 *
 * Version: 1.0 | Ravidu Liyanage
 */

#ifndef __PWR_CTRL_H
#define __PWR_CTRL_H

#include "stm32f10x.h"

/***********************************************************************/
/*                      Function Prototypes                            */
/***********************************************************************/

// Puts CPU into Standby Mode
void gotoStandby(void);

#endif /* __PWR_CTRL_H */

