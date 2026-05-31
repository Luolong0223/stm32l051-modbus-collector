/**
 * @file    modbus_driver.h
 * @brief   Modbus RTU 驱动 — 主站轮询 + 从站配置 (UART2)
 * @version 2.4 — volatile 修复 + 帧间静默 + 名称寄存器优化
 */
#ifndef __MODBUS_DRIVER_H
#define __MODBUS_DRIVER_H

#include "sys_config.h"

/* ── 初始化 ── */
void MB_Init(UART_HandleTypeDef *huart);

/* ── 周期处理 (main while(1) 调用) ── */
void MB_Master_Process(void);
void MB_Slave_Process(void);

/* ── 帧超时检测 (main while(1) 调用, 放在循环开头) ── */
void MB_Check_Frame_Timeout(void);

/* ── UART 空闲中断回调 ── */
void MB_UART_Idle_Callback(UART_HandleTypeDef *huart);

/* ── UART 物理参数热更新 ── */
void MB_Reconfigure_UART(void);

/* ── 主从模式切换 ── */
void MB_Switch_To_Slave(void);
void MB_Switch_To_Master(void);

/* ── 从站请求处理 (供外部调试/测试调用) ── */
void MB_Slave_Handle_Request(void);

/* ── RS485 方向控制 ── */
#define RS485_DE_PORT   GPIOA
#define RS485_DE_PIN    GPIO_PIN_8
#define RS485_RE_PORT   GPIOA
#define RS485_RE_PIN    GPIO_PIN_9

void RS485_TX_Enable(void);
void RS485_RX_Enable(void);

/* ── CRC16 ── */
uint16_t MB_CRC16(const uint8_t *buf, uint16_t len);

/* ── 数据解析 ── */
float MB_Parse_Registers(const uint16_t *regs, uint8_t data_type, uint8_t byte_order);

#endif /* __MODBUS_DRIVER_H */
