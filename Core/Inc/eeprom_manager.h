/**
 * @file    eeprom_manager.h
 * @brief   STM32L051 内部 EEPROM 读写管理 — 配置存储与加载 (v2.4: 栈溢出修复)
 */
#ifndef __EEPROM_MANAGER_H
#define __EEPROM_MANAGER_H

#include "sys_config.h"

/* EEPROM 基地址 & 大小已移至 sys_config.h (供 _Static_assert 使用) */

/* 配置存储起始地址 (EEPROM 偏移) */
#define EEPROM_CFG_OFFSET       0x0000

/* ═══════════════════════════════════════════════════════════════════════════
 *  EEPROM 读写
 * ═══════════════════════════════════════════════════════════════════════════ */
HAL_StatusTypeDef EEPROM_Read(uint32_t offset, uint8_t *data, uint32_t len);
HAL_StatusTypeDef EEPROM_Write(uint32_t offset, const uint8_t *data, uint32_t len);

/* ═══════════════════════════════════════════════════════════════════════════
 *  系统配置持久化
 * ═══════════════════════════════════════════════════════════════════════════ */
void     EEPROM_Load_Config(SystemCfg_t *cfg);
void     EEPROM_Save_Config(const SystemCfg_t *cfg);
void     EEPROM_Default_Config(SystemCfg_t *cfg);
uint8_t  EEPROM_Config_Is_Valid(const SystemCfg_t *cfg);
void     EEPROM_Filter_Name(char *name);

#endif /* __EEPROM_MANAGER_H */
