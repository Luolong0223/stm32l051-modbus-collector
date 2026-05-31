/**
 * @file    main_app.c
 * @brief   应用主逻辑 — 系统初始化 + 主循环 + 低功耗管理 (v2.3)
 *
 *  v2.2 改进:
 *    - 模式选择改为硬件 GPIO (PA15): HIGH=配置模式(从站), LOW=轮询上报模式(主站)
 *    - 去除软件主从切换的抖动问题
 *    - 延迟 UART2 重配在主站 IDLE 时安全执行
 *  v2.3 改进:
 *    - 新增设备名称、电压采集、每轮必报
 *    - 新增 Stop 模式低功耗 + RTC WakeUp 定时唤醒
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
 *  RTC WakeUp 定时器配置
 *  使用 LSE (32.768kHz) 经 ck_spre (1Hz) 输出
 *  WakeUp 计数器为 16 位，最大间隔 65535 秒
 * ═══════════════════════════════════════════════════════════════════════════ */
static RTC_HandleTypeDef hrtc;

static void LowPower_RTC_Init(void)
{
    __HAL_RCC_RTC_ENABLE();

    hrtc.Instance = RTC;
    hrtc.Init.HourFormat     = RTC_HOURFORMAT_24;
    hrtc.Init.AsynchPrediv   = 127;    /* LSE/128 = 256 */
    hrtc.Init.SynchPrediv    = 255;    /* /256 → 1Hz ck_spre */
    hrtc.Init.OutPut         = RTC_OUTPUT_DISABLE;
    hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
    hrtc.Init.OutPutType     = RTC_OUTPUT_TYPE_OPENDRAIN;
    HAL_RTC_Init(&hrtc);
}

/**
 * @brief  配置 RTC WakeUp 定时器并启动
 * @param  sec  唤醒间隔 (秒), 0=停止定时器
 */
static void LowPower_RTC_SetWakeup(uint16_t sec)
{
    HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);

    if (sec == 0) return;

    /* ck_spre = 1Hz, 直接用秒数作为计数值 */
    HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, sec, RTC_WAKEUPCLOCK_CK_SPRE_16BITS);
}

/**
 * @brief  进入 Stop 模式 (最低功耗 ~3μA)
 *  - 关闭所有时域 (MSI/HSI/HSE 停止)
 *  - SRAM 和寄存器内容保留
 *  - 由 RTC WakeUp 中断唤醒
 *  - 唤醒后需恢复时钟 (SystemClock_Config)
 */
static void LowPower_EnterStop(void)
{
    /* 1. 关闭外设时钟降低功耗 */
    __HAL_RCC_ADC1_CLK_DISABLE();

    /* 2. 配置 RTC 唤醒定时器 */
    LowPower_RTC_SetWakeup(g_sys_cfg.sleep_interval_sec);

    /* 3. 进入 Stop 模式 (Regulator=LowPower) */
    HAL_SuspendTick();   /* 停止 SysTick */
    HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);

    /* === MCU 在此休眠，被 RTC 唤醒后从这里继续 === */

    /* 4. 唤醒后恢复 */
    HAL_ResumeTick();    /* 恢复 SysTick */

    /* 5. 恢复系统时钟 (Stop 模式后时钟切换到 MSI，需恢复到原始配置) */
    SystemClock_Config();  /* CubeMX 生成的时钟配置函数 */

    /* 6. 恢复外设时钟 */
    __HAL_RCC_ADC1_CLK_ENABLE();

    /* 7. ADC 校准 (Stop 后需重新校准) */
    HAL_ADCEx_Calibration_Start(&hadc, ADC_SINGLE_ENDED);
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
    HAL_ADCEx_Calibration_Start(&hadc, ADC_SINGLE_ENDED);
    ADC_Read_Voltage();

    /* 10. 初始化 RTC (低功耗唤醒定时器) */
    LowPower_RTC_Init();

    /* 11. 初始化采集时间戳 */
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

        /* 低功耗: 一轮轮询+上报完成后进入 Stop 模式 */
        if (g_sleep_pending && g_sys_cfg.sleep_interval_sec > 0) {
            g_sleep_pending = 0;

            /* 关闭 RS485 接收 */
            RS485_RX_Enable();

            /* 读取一次电压 (Stop 前记录) */
            ADC_Read_Voltage();

            /* 进入 Stop 模式，由 RTC 定时唤醒 */
            LowPower_EnterStop();

            /* === 唤醒后从这里继续 === */

            /* 恢复 UART2 接收 */
            MB_Reconfigure_UART();

            /* 重新使能 UART2 空闲中断 */
            UART2_Enable_Idle_IRQ();

            /* 更新所有从机的轮询时间戳 (避免唤醒后立即重轮询) */
            uint32_t now = HAL_GetTick();
            for (uint8_t i = 0; i < g_sys_cfg.slave_count; i++) {
                g_slave_data[i].last_poll_tick = now;
            }
        }
    }
}
