/**
 * @file    sys_config.h
 * @brief   系统全局配置定义 — 寄存器映射、EEPROM布局、数据结构
 * @note    STM32L051K8 | UART2=Modbus RTU | UART1=数据上报 | 内部EEPROM存储
 * @version 2.4 — 低功耗改用 Standby 模式, 修复 volatile/栈溢出/帧间静默
 */
#ifndef __SYS_CONFIG_H
#define __SYS_CONFIG_H

#include "stm32l0xx_hal.h"
#include "main.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>

/* ═══════════════════════════════════════════════════════════════════════════
 *  硬件常量
 * ═══════════════════════════════════════════════════════════════════════════ */
#define EEPROM_BASE_ADDR        0x08080000  /* STM32L051 内部 EEPROM 基地址 */
#define EEPROM_SIZE             2048        /* 2KB */

/* ═══════════════════════════════════════════════════════════════════════════
 *  系统限制常量
 * ═══════════════════════════════════════════════════════════════════════════ */
#define MAX_SLAVE_COUNT         5       /* 最大采集从机数量 */
#define MAX_DATA_POINTS         8       /* 单从机最大数据点数量 */
#define NAME_MAX_LEN            20      /* 名称最大字符数(不含\0) */
#define NAME_BUF_SIZE           21      /* 名称缓冲区大小(含\0) */

#define REPORT_FORMAT_TEXT      0
#define REPORT_FORMAT_JSON      1
#define REPORT_FORMAT_HEX       2

/* ═══════════════════════════════════════════════════════════════════════════
 *  Modbus 功能码
 * ═══════════════════════════════════════════════════════════════════════════ */
#define MB_FC_READ_HOLDING_REGS     0x03
#define MB_FC_READ_INPUT_REGS       0x04
#define MB_FC_WRITE_SINGLE_REG      0x06
#define MB_FC_WRITE_MULTIPLE_REGS   0x10

#define MB_EX_NONE                  0x00
#define MB_EX_ILLEGAL_FUNCTION      0x01
#define MB_EX_ILLEGAL_DATA_ADDR     0x02
#define MB_EX_ILLEGAL_DATA_VALUE    0x03
#define MB_EX_SLAVE_DEVICE_FAILURE  0x04

/* ═══════════════════════════════════════════════════════════════════════════
 *  通信参数
 * ═══════════════════════════════════════════════════════════════════════════ */
#define MB_MASTER_TIMEOUT_MS        1000    /* 主站等待响应超时 */
#define MB_MASTER_RETRY_MAX         2       /* 主站单数据点最大重试次数 */
#define MB_FRAME_SILENT_MS          5       /* 帧间静默 (3.5字符 @9600≈4ms) */
#define MB_RX_BUF_SIZE              256
#define MB_TX_BUF_SIZE              256
#define MB_FRAME_DETECT_MS          10      /* 帧结束检测超时 (无新字节) */
#define REPORT_BUF_SIZE             640     /* 增大以容纳 HEX 格式最坏情况 */
#define HEX_TMP_BUF_SIZE            200     /* HEX 原始数据缓冲 (5×38+5=195 max) */

/* ═══════════════════════════════════════════════════════════════════════════
 *  模式选择 GPIO (PA15)
 *    PA15=HIGH → 配置模式 (Modbus 从站, 接受上位机配置)
 *    PA15=LOW  → 轮询上报模式 (Modbus 主站, 采集从机数据并上报)
 * ═══════════════════════════════════════════════════════════════════════════ */
#define MODE_SELECT_PORT    GPIOA
#define MODE_SELECT_PIN     GPIO_PIN_15
#define MODE_IS_SLAVE()     (HAL_GPIO_ReadPin(MODE_SELECT_PORT, MODE_SELECT_PIN) == GPIO_PIN_SET)

/* ═══════════════════════════════════════════════════════════════════════════
 *  数据类型 & 字节序枚举
 * ═══════════════════════════════════════════════════════════════════════════ */
typedef enum {
    DATA_TYPE_U16   = 0,
    DATA_TYPE_I16   = 1,
    DATA_TYPE_U32   = 2,
    DATA_TYPE_I32   = 3,
    DATA_TYPE_FLOAT = 4
} DataType_t;

typedef enum {
    BYTE_ORDER_ABCD = 0,    /* Big-Endian 标准网络序 */
    BYTE_ORDER_BADC = 1,    /* Big-Endian 字内字节交换 */
    BYTE_ORDER_CDAB = 2,    /* 16位字交换 */
    BYTE_ORDER_DCBA = 3     /* Full Little-Endian */
} ByteOrder_t;

/* ═══════════════════════════════════════════════════════════════════════════
 *  采集数据点配置 (含名称)
 * ═══════════════════════════════════════════════════════════════════════════ */
typedef struct {
    uint16_t    reg_addr;                   /* 从机寄存器起始地址 */
    uint8_t     data_type;                  /* DataType_t */
    uint8_t     byte_order;                 /* ByteOrder_t */
    char        name[NAME_BUF_SIZE];        /* 数据点名称 e.g. "温度" */
    uint8_t     _pad[3];                    /* 对齐至 28 字节 */
} DataPointCfg_t;   /* sizeof = 28 */

/* ═══════════════════════════════════════════════════════════════════════════
 *  从机配置 (含名称)
 * ═══════════════════════════════════════════════════════════════════════════ */
typedef struct {
    uint8_t         slave_addr;                     /* Modbus 从机地址 */
    uint8_t         enabled;                        /* 1=启用 0=停用 */
    uint8_t         data_point_count;               /* 数据点数量 0~8 */
    uint8_t         _reserved;
    uint32_t        poll_period_ms;                 /* 轮询周期 ms */
    char            name[NAME_BUF_SIZE];            /* 设备名称 e.g. "1号温湿度" */
    uint8_t         _pad[3];                        /* 对齐 */
    DataPointCfg_t  data_points[MAX_DATA_POINTS];   /* 数据点配置 */
} SlaveCfg_t;   /* sizeof = 256 (32 + 8×28, GCC -O0, ARM32 4-byte align) */

/* ═══════════════════════════════════════════════════════════════════════════
 *  系统配置 (存入 EEPROM, 总计约 1352 字节 < 2048)
 * ═══════════════════════════════════════════════════════════════════════════ */
typedef struct {
    /* --- UART2 / Modbus 物理层 --- */
    uint32_t    uart2_baudrate;
    uint8_t     uart2_parity;           /* 0=无 1=奇 2=偶 */
    uint8_t     uart2_stopbits;         /* 1 或 2 */
    uint16_t    rs485_de_delay_us;      /* DE 使能延时 us */
    uint16_t    rs485_re_delay_us;      /* RE 使能延时 us */
    uint8_t     local_mb_addr;          /* 本机 Modbus 从站地址 */
    uint8_t     slave_count;            /* 采集从机总数 1~5 */
    uint8_t     report_format;          /* 0=TEXT 1=JSON 2=HEX */
    uint8_t     _reserved0;
    uint32_t    uart1_baudrate;

    /* --- 设备本体信息 --- */
    char        device_name[NAME_BUF_SIZE];   /* 设备名称 e.g. "3号线采集器" */
    uint16_t    sleep_interval_sec;           /* 低功耗休眠间隔 (秒), 0=禁用 */
    uint8_t     _pad_device[1];               /* 对齐 */

    /* --- 从机配置 --- */
    SlaveCfg_t  slaves[MAX_SLAVE_COUNT];

    /* --- 校验 --- */
    uint32_t    config_version;
} SystemCfg_t;

/* 编译期校验结构体大小 (C11 _Static_assert) */
_Static_assert(sizeof(DataPointCfg_t) == 28, "DataPointCfg_t size mismatch");
_Static_assert(sizeof(SystemCfg_t) <= EEPROM_SIZE, "SystemCfg_t exceeds EEPROM size");
_Static_assert(MAX_SLAVE_COUNT <= 5, "MAX_SLAVE_COUNT must be <= 5");
_Static_assert(MAX_DATA_POINTS <= 8, "MAX_DATA_POINTS must be <= 8");

/* ═══════════════════════════════════════════════════════════════════════════
 *  运行时采集结果
 * ═══════════════════════════════════════════════════════════════════════════ */
typedef struct {
    float       values[MAX_DATA_POINTS];    /* 解析后的浮点值 */
    uint8_t     valid[MAX_DATA_POINTS];     /* 该点数据是否有效 */
    uint32_t    last_poll_tick;             /* 上次轮询时刻 */
    uint32_t    poll_cycle_completed;       /* 已完成的完整轮询周期数 (仅主站) */
    uint8_t     error_count;                /* 连续通信错误计数 */
    uint8_t     online;                     /* 设备在线 */
    uint16_t    _pad;
} SlaveData_t;

/* ═══════════════════════════════════════════════════════════════════════════
 *  Modbus 主站状态机
 * ═══════════════════════════════════════════════════════════════════════════ */
typedef enum {
    MB_MASTER_IDLE = 0,
    MB_MASTER_SENDING,
    MB_MASTER_WAIT_RESP,
    MB_MASTER_PROCESSING
} MBMasterState_t;

/* ═══════════════════════════════════════════════════════════════════════════
 *  Modbus 主站句柄 (改进: 超时帧检测替代单字节DMA)
 * ═══════════════════════════════════════════════════════════════════════════ */
typedef struct {
    UART_HandleTypeDef  *huart;
    MBMasterState_t     state;

    /* 接收: 逐字节中断 + 超时帧检测 */
    uint8_t             rx_buf[MB_RX_BUF_SIZE];
    volatile uint16_t   rx_pos;             /* 当前接收位置 */
    volatile uint32_t   last_rx_tick;       /* 最后一次收到字节的时间 */
    volatile uint8_t    frame_ready;        /* 帧接收完成标志 */

    /* 发送 */
    uint8_t             tx_buf[MB_TX_BUF_SIZE];

    /* 状态机 */
    uint32_t            wait_start_tick;
    uint32_t            tx_end_tick;            /* 最后一帧发送结束时刻 (帧间静默用) */
    uint16_t            polled_slave_count;     /* 本轮已轮询过的启用从机数 */
    uint8_t             current_slave;
    uint8_t             current_point;
    uint8_t             retry_cnt;
    uint8_t             _pad;
} MBMasterHandle_t;

/* ═══════════════════════════════════════════════════════════════════════════
 *  Modbus 从站句柄
 * ═══════════════════════════════════════════════════════════════════════════ */
typedef struct {
    UART_HandleTypeDef  *huart;
    uint8_t             rx_buf[MB_RX_BUF_SIZE];
    uint8_t             tx_buf[MB_TX_BUF_SIZE];
    volatile uint16_t   rx_pos;
    volatile uint32_t   last_rx_tick;
    volatile uint8_t    frame_ready;
    uint8_t             _pad[3];
} MBSlaveHandle_t;

/* ═══════════════════════════════════════════════════════════════════════════
 *  运行模式 (主从分时复用)
 * ═══════════════════════════════════════════════════════════════════════════ */
typedef enum {
    RUN_MODE_MASTER = 0,    /* 主站轮询采集模式 */
    RUN_MODE_SLAVE          /* 从站配置监听模式 */
} RunMode_t;

/* ═══════════════════════════════════════════════════════════════════════════
 *  全局变量 extern — 定义仅在 main_app.c 中
 * ═══════════════════════════════════════════════════════════════════════════ */
extern SystemCfg_t      g_sys_cfg;
extern SlaveData_t      g_slave_data[MAX_SLAVE_COUNT];
extern MBMasterHandle_t g_mb_master;
extern MBSlaveHandle_t  g_mb_slave;
extern volatile RunMode_t g_run_mode;
extern volatile uint8_t g_uart2_reconfig_pending;   /* UART2 延迟重配标志 */
extern volatile uint8_t g_uart1_reconfig_pending;   /* UART1 延迟重配标志 */
extern volatile uint8_t g_eeprom_save_pending;      /* EEPROM 延迟保存标志 */
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern ADC_HandleTypeDef hadc;                      /* ADC 句柄 (CubeMX 生成) */
extern volatile uint16_t g_adc_voltage_raw;         /* PA0 ADC 原始值 (0~4095) */
extern volatile uint8_t  g_sleep_pending;           /* 低功耗休眠请求标志 */

/* ═══════════════════════════════════════════════════════════════════════════
 *  默认配置
 * ═══════════════════════════════════════════════════════════════════════════ */
#define CFG_DEFAULT_BAUDRATE        9600
#define CFG_DEFAULT_PARITY          0
#define CFG_DEFAULT_STOPBITS        1
#define CFG_DEFAULT_DE_DELAY        50
#define CFG_DEFAULT_RE_DELAY        50
#define CFG_DEFAULT_MB_ADDR         1
#define CFG_DEFAULT_SLAVE_COUNT     1
#define CFG_DEFAULT_REPORT_FMT      REPORT_FORMAT_JSON
#define CFG_DEFAULT_POLL_MS         1000
#define CFG_VERSION                 0xA5B6C7D9  /* v2 版本标识 */

#endif /* __SYS_CONFIG_H */
