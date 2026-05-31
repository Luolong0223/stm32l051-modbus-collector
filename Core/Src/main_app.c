/**
 * @file    main_app.c
 * @brief   应用主逻辑 — 系统初始化 + 主循环 (v2.1)
 *
 *  v2.1 改进:
 *    - 修复: 全局变量仅在此文件定义 (消除重复定义)
 *    - 优化: 主从切换增加超时保护 (在 modbus_driver.c 中实现)
 */
#include "sys_config.h"
#include "modbus_driver.h"
#include "eeprom_manager.h"
#include "report_manager.h"

/* ═══════════════════════════════════════════════════════════════════════════
 *  全局变量 (唯一定义点)
 * ═══════════════════════════════════════════════════════════════════════════ */
SystemCfg_t     g_sys_cfg;
SlaveData_t     g_slave_data[MAX_SLAVE_COUNT];

/* ═══════════════════════════════════════════════════════════════════════════
 *  RS485 GPIO 初始化 (PA8=DE, PA9=RE)
 * ═══════════════════════════════════════════════════════════════════════════ */
static void RS485_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitStruct.Pin   = RS485_DE_PIN;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(RS485_DE_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(RS485_DE_PORT, RS485_DE_PIN, GPIO_PIN_RESET);

    GPIO_InitStruct.Pin = RS485_RE_PIN;
    HAL_GPIO_Init(RS485_RE_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(RS485_RE_PORT, RS485_RE_PIN, GPIO_PIN_RESET);
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  UART2 空闲中断使能
 * ═══════════════════════════════════════════════════════════════════════════ */
static void UART2_Enable_Idle_IRQ(void)
{
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
    HAL_NVIC_SetPriority(USART2_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  系统初始化
 * ═══════════════════════════════════════════════════════════════════════════ */
void System_Init(void)
{
    /* 1. 初始化数据结构 */
    memset(&g_sys_cfg, 0, sizeof(g_sys_cfg));
    memset(g_slave_data, 0, sizeof(g_slave_data));

    /* 2. 从 EEPROM 加载配置 */
    EEPROM_Load_Config(&g_sys_cfg);

    /* 3. 初始化 RS485 GPIO */
    RS485_GPIO_Init();

    /* 4. 使能 UART2 空闲中断 */
    UART2_Enable_Idle_IRQ();

    /* 5. 初始化 Modbus 驱动 */
    MB_Init(&huart2);

    /* 6. 应用 UART2 配置 */
    MB_Reconfigure_UART();

    /* 7. 初始化采集时间戳 */
    uint32_t now = HAL_GetTick();
    for (uint8_t i = 0; i < MAX_SLAVE_COUNT; i++) {
        g_slave_data[i].last_poll_tick = now;
        g_slave_data[i].online = 0;
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  主循环
 *
 *  执行顺序:
 *    1. 帧超时检测
 *    2. Modbus 主站轮询采集
 *    3. Modbus 从站配置监听 (主站 IDLE 间隙，带超时保护)
 *    4. 数据上报
 * ═══════════════════════════════════════════════════════════════════════════ */
void System_MainLoop(void)
{
    /* 1. 帧超时检测 */
    MB_Check_Frame_Timeout();

    /* 2. Modbus 主站轮询 */
    MB_Master_Process();

    /* 3. 从站监听 (主站空闲时切入，超时后自动切回) */
    if (g_mb_master.state == MB_MASTER_IDLE) {
        if (g_run_mode != RUN_MODE_SLAVE) {
            MB_Switch_To_Slave();
        }
        MB_Slave_Process();
    }

    /* 4. UART1 数据上报 */
    REPORT_Process();
}
