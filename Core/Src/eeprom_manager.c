/**
 * @file    eeprom_manager.c
 * @brief   STM32L051 内部 EEPROM 驱动 — 配置读写、校验、默认值 (v2.1)
 *
 *  v2.1: sprintf → snprintf 防溢出
 *  v2.2: 变更检测 + 延迟保存 + 名称过滤 + 线程安全修复
 */
#include "eeprom_manager.h"

/* 延迟保存标志 (在 modbus_driver.c 中定义) */
extern volatile uint8_t g_eeprom_save_pending;

/* ═══════════════════════════════════════════════════════════════════════════
 *  EEPROM 底层读写
 * ═══════════════════════════════════════════════════════════════════════════ */
HAL_StatusTypeDef EEPROM_Read(uint32_t offset, uint8_t *data, uint32_t len)
{
    if ((offset + len) > EEPROM_SIZE) return HAL_ERROR;
    uint32_t addr = EEPROM_BASE_ADDR + offset;
    for (uint32_t i = 0; i < len; i++) {
        data[i] = *(__IO uint8_t *)(addr + i);
    }
    return HAL_OK;
}

HAL_StatusTypeDef EEPROM_Write(uint32_t offset, const uint8_t *data, uint32_t len)
{
    if ((offset + len) > EEPROM_SIZE) return HAL_ERROR;

    while ((FLASH->SR & FLASH_SR_BSY) != 0) { }

    /* 解锁 PELOCK */
    if ((FLASH->PECR & FLASH_PECR_PELOCK) != 0) {
        FLASH->PEKEYR = 0x89ABCDEF;
        FLASH->PEKEYR = 0x02030405;
    }

    uint32_t addr = EEPROM_BASE_ADDR + offset;
    for (uint32_t i = 0; i < len; i++) {
        *(__IO uint8_t *)(addr + i) = data[i];
        while ((FLASH->SR & FLASH_SR_BSY) != 0) { }
    }

    FLASH->PECR |= FLASH_PECR_PELOCK;
    return HAL_OK;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  配置管理
 * ═══════════════════════════════════════════════════════════════════════════ */
uint8_t EEPROM_Config_Is_Valid(const SystemCfg_t *cfg)
{
    return (cfg->config_version == CFG_VERSION);
}

void EEPROM_Load_Config(SystemCfg_t *cfg)
{
    EEPROM_Read(EEPROM_CFG_OFFSET, (uint8_t *)cfg, sizeof(SystemCfg_t));

    if (!EEPROM_Config_Is_Valid(cfg)) {
        /* EEPROM 中无有效配置 → 写入默认值 */
        EEPROM_Default_Config(cfg);
        /* 立即保存默认配置 (启动阶段无中断竞争) */
        EEPROM_Write(EEPROM_CFG_OFFSET, (const uint8_t *)cfg, sizeof(SystemCfg_t));
    }

    /* 名称字段过滤 (清除不可打印字符) */
    EEPROM_Filter_Name(cfg->device_name);
    for (uint8_t i = 0; i < MAX_SLAVE_COUNT; i++) {
        EEPROM_Filter_Name(cfg->slaves[i].name);
        for (uint8_t j = 0; j < MAX_DATA_POINTS; j++) {
            EEPROM_Filter_Name(cfg->slaves[i].data_points[j].name);
        }
    }

    /* 参数限幅 */
    if (cfg->slave_count < 1) cfg->slave_count = 1;
    if (cfg->slave_count > MAX_SLAVE_COUNT) cfg->slave_count = MAX_SLAVE_COUNT;
    if (cfg->report_format > REPORT_FORMAT_HEX) cfg->report_format = REPORT_FORMAT_JSON;
    if (cfg->uart2_stopbits != 1 && cfg->uart2_stopbits != 2) cfg->uart2_stopbits = 1;
    if (cfg->uart2_parity > 2) cfg->uart2_parity = 0;

    for (uint8_t i = 0; i < MAX_SLAVE_COUNT; i++) {
        SlaveCfg_t *s = &cfg->slaves[i];
        if (s->data_point_count > MAX_DATA_POINTS) s->data_point_count = MAX_DATA_POINTS;
        if (s->poll_period_ms < 100) s->poll_period_ms = 100;
        /* 确保名称以 \0 结尾 */
        s->name[NAME_MAX_LEN] = '\0';
        for (uint8_t j = 0; j < MAX_DATA_POINTS; j++) {
            if (s->data_points[j].data_type > DATA_TYPE_FLOAT)
                s->data_points[j].data_type = DATA_TYPE_U16;
            if (s->data_points[j].byte_order > BYTE_ORDER_DCBA)
                s->data_points[j].byte_order = BYTE_ORDER_ABCD;
            s->data_points[j].name[NAME_MAX_LEN] = '\0';
        }
    }
}

void EEPROM_Save_Config(const SystemCfg_t *cfg)
{
    /* 变更检测: 先读取当前 EEPROM 内容，仅在数据变化时写入 */
    uint8_t read_buf[sizeof(SystemCfg_t)];  /* 栈局部变量，避免中断/主循环竞争 */
    EEPROM_Read(EEPROM_CFG_OFFSET, read_buf, sizeof(SystemCfg_t));
    if (memcmp(read_buf, cfg, sizeof(SystemCfg_t)) == 0) {
        return;  /* 数据未变化，跳过写入以延长 EEPROM 寿命 */
    }
    EEPROM_Write(EEPROM_CFG_OFFSET, (const uint8_t *)cfg, sizeof(SystemCfg_t));
}

/**
 * @brief  过滤名称字段: 移除不可打印字符, 确保以 \0 结尾
 */
void EEPROM_Filter_Name(char *name)
{
    uint8_t dst = 0;
    for (uint8_t src = 0; src < NAME_MAX_LEN && name[src] != '\0'; src++) {
        char c = name[src];
        /* 保留可打印字符 (0x20~0x7E) 和常见高字节 (GBK/UTF-8 首字节 0x80~0xFF) */
        if ((uint8_t)c >= 0x20) {
            name[dst++] = c;
        }
        /* 跳过控制字符 (\0 已被循环条件排除) */
    }
    name[dst] = '\0';
}

void EEPROM_Default_Config(SystemCfg_t *cfg)
{
    memset(cfg, 0, sizeof(SystemCfg_t));

    cfg->uart2_baudrate     = CFG_DEFAULT_BAUDRATE;
    cfg->uart2_parity       = CFG_DEFAULT_PARITY;
    cfg->uart2_stopbits     = CFG_DEFAULT_STOPBITS;
    cfg->rs485_de_delay_us  = CFG_DEFAULT_DE_DELAY;
    cfg->rs485_re_delay_us  = CFG_DEFAULT_RE_DELAY;
    cfg->local_mb_addr      = CFG_DEFAULT_MB_ADDR;
    cfg->slave_count        = CFG_DEFAULT_SLAVE_COUNT;
    cfg->report_format      = CFG_DEFAULT_REPORT_FMT;
    cfg->uart1_baudrate     = CFG_DEFAULT_BAUDRATE;
    cfg->config_version     = CFG_VERSION;

    /* 默认设备名称 */
    strncpy(cfg->device_name, "Collector-1", NAME_MAX_LEN);

    /* 默认从机1 */
    strncpy(cfg->slaves[0].name, "Slave-1", NAME_MAX_LEN);
    cfg->slaves[0].slave_addr       = 1;
    cfg->slaves[0].enabled          = 1;
    cfg->slaves[0].data_point_count = 1;
    cfg->slaves[0].poll_period_ms   = CFG_DEFAULT_POLL_MS;

    strncpy(cfg->slaves[0].data_points[0].name, "Data-1", NAME_MAX_LEN);
    cfg->slaves[0].data_points[0].reg_addr   = 0;
    cfg->slaves[0].data_points[0].data_type  = DATA_TYPE_U16;
    cfg->slaves[0].data_points[0].byte_order = BYTE_ORDER_ABCD;

    /* 其余从机默认停用 */
    for (uint8_t i = 1; i < MAX_SLAVE_COUNT; i++) {
        snprintf(cfg->slaves[i].name, NAME_BUF_SIZE, "Slave-%u", i + 1);
        cfg->slaves[i].enabled = 0;
        cfg->slaves[i].poll_period_ms = CFG_DEFAULT_POLL_MS;
        for (uint8_t j = 0; j < MAX_DATA_POINTS; j++) {
            snprintf(cfg->slaves[i].data_points[j].name, NAME_BUF_SIZE, "P%u", j + 1);
        }
    }
}
