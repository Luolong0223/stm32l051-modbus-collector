/**
 * @file    stm32l0xx_it_custom.c
 * @brief   中断处理集成说明 — 将以下代码合并到 CubeMX 生成的 stm32l0xx_it.c
 *
 *  ★ 使用方法:
 *    不要直接编译此文件! 将其中的代码复制到 CubeMX 生成的
 *    Core/Src/stm32l0xx_it.c 中对应位置 (USER CODE 区域)
 */

/* ═══════════════════════════════════════════════════════════════════════════
 *  1. 在文件头部 (USER CODE BEGIN Includes) 添加:
 * ═══════════════════════════════════════════════════════════════════════════ */
/*
#include "modbus_driver.h"
*/

/* ═══════════════════════════════════════════════════════════════════════════
 *  2. 在 USART2_IRQHandler 函数中 (USER CODE BEGIN USART2) 添加:
 * ═══════════════════════════════════════════════════════════════════════════ */
/*
void USART2_IRQHandler(void)
{
    // USER CODE BEGIN USART2_IRQn 0
    if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_IDLE)) {
        MB_UART_Idle_Callback(&huart2);
    }
    // USER CODE END USART2_IRQn 0

    HAL_UART_IRQHandler(&huart2);

    // USER CODE BEGIN USART2_IRQn 1
    // USER CODE END USART2_IRQn 1
}
*/
