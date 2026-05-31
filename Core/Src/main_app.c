/**
 * @file    main_app.c
 * @brief   应用主逻辑 — 系统初始化 + 主循环 (v2.2)
 *
 *  v2.2 改进:
 *    - 模式选择改为硬件 GPIO (PA15): HIGH=配置模式(从站), LOW=轮询上报模式(主站)
 *    - 去除软件主从切换的抖动问题
 *    - 延迟 UART2 重配在主站 IDLE 时安全执行
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
 *  模式选择 GPIO 初始化 (PA15 — 输入, 下拉)
 *    HIGH = 配置模式 (Modbus 从站)
 *    LOW  = 轮询上报模式 (Modbus 主站)
 * ═══════════════════════════════════════════════════════════════════════════ */
static void Mode_Select_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitStruct.Pin   = MODE_SELECT_PIN;
    GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull  = GPIO_PULLDOWN;      /* 默认下拉 → 默认主站模式 */
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(MODE_SELECT_PORT, &GPIO_InitStruct);
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  ADC 电压采集 (PA0)
 *  CubeMX 已配置 ADC，句柄为 hadc
 *  读取单次转换结果，存入 g_adc_voltage_raw
 * ═══════════════════════════════════════════════════════════════════════════ */
static void ADC_Read_Voltage(void)
{
    HAL_ADC_Start(&hadc);
    if (HAL_ADC_PollForConversion(&hadc, 10) == HAL_OK) {
        g_adc_voltage_raw = (uint16_t)HAL_ADC_GetValue(&hadc);
    }
    HAL_ADC_Stop(&hadc);
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

    /* 4. 初始化模式选择 GPIO (PA15) */
    Mode_Select_GPIO_Init();

    /* 5. 使能 UART2 空闲中断 */
    UART2_Enable_Idle_IRQ();

    /* 6. 初始化 Modbus 驱动 */
    MB_Init(&huart2);

    /* 7. 应用 UART2 配置 */
    MB_Reconfigure_UART();

    /* 8. 根据 PA15 初始状态设置模式 */
    if (MODE_IS_SLAVE()) {
        MB_Switch_To_Slave();
    } else {
        MB_Switch_To_Master();
    }

    /* 9. ADC 校准 + 首次电压采集 */
    HAL_ADCEx_Calibration_Start(&hadc);
    ADC_Read_Voltage();

    /* 10. 初始化采集时间戳 */
    uint32_t now = HAL_GetTick();
    for (uint8_t i = 0; i < MAX_SLAVE_COUNT; i++) {
        g_slave_data[i].last_poll_tick = now;
        g_slave_data[i].online = 0;
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  主循环
 *
 *  PA15=HIGH → 配置模式: 仅运行 Modbus 从站 (接受上位机配置)
 *  PA15=LOW  → 轮询上报模式: 运行 Modbus 主站 + UART1 数据上报
 *
 *  模式由硬件引脚决定，仅在切换时执行一次初始化，无抖动
 * ═══════════════════════════════════════════════════════════════════════════ */
void System_MainLoop(void)
{
    /* 1. 读取 PA15，判断目标模式 */
    RunMode_t target_mode = MODE_IS_SLAVE() ? RUN_MODE_SLAVE : RUN_MODE_MASTER;

    /* 2. 模式切换检测 (仅在变化时执行一次) */
    if (target_mode != g_run_mode) {
        if (target_mode == RUN_MODE_SLAVE) {
            MB_Switch_To_Slave();
        } else {
            MB_Switch_To_Master();
        }
    }

    /* 3. 帧超时检测 */
    MB_Check_Frame_Timeout();

    /* 3.5 周期性读取电压 (每次主循环都采样，约几十us) */
    ADC_Read_Voltage();

    /* 4. 根据当前模式执行对应任务 */
    if (g_run_mode == RUN_MODE_SLAVE) {
        /* 配置模式: 处理 Modbus 配置请求 */
        MB_Slave_Process();
        /* 从站模式下也执行延迟保存，避免配置模式中断电丢失 */
        if (g_eeprom_save_pending) {
            g_eeprom_save_pending = 0;
            EEPROM_Save_Config(&g_sys_cfg);
        }
    } else {
        /* 轮询上报模式: 主站采集 + 数据上报 */
        MB_Master_Process();
        REPORT_Process();
    }
}
