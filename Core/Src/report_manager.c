/**
 * @file    report_manager.c
 * @brief   UART1 数据上报管理 — 按配置格式上报, 支持设备/数据点名称
 * @version 2.1 — hex 缓冲区改 static，减少栈压力
 *
 *  TEXT:  "1号温湿度",P1("温度")=23.50,P2("湿度")=65.00;"2号压力",P1("压力")=101.32;...
 *  JSON:  {"s1":{"name":"1号温湿度","online":1,"data":{"温度":23.5,"湿度":65.0}},...}
 *  HEX:   AA 55 [slave_count] [slave_addr|online|dp_count] [float x4] ... [CRC]
 */
#include "report_manager.h"
#include "modbus_driver.h"

static char report_buf[REPORT_BUF_SIZE];

/* HEX 格式临时缓冲 (避免在栈上分配 256 字节) */
static uint8_t hex_tmp[HEX_TMP_BUF_SIZE];

/* ═══════════════════════════════════════════════════════════════════════════
 *  浮点转字符串 (轻量实现, 避免 sprintf %f 消耗 FLASH)
 * ═══════════════════════════════════════════════════════════════════════════ */
static uint16_t ftostr(float val, char *buf, uint8_t decimals)
{
    /* NaN / Inf 检测 */
    if (val != val) { buf[0]='N'; buf[1]='a'; buf[2]='N'; return 3; }
    if (val > 1e10f || val < -1e10f) return (uint16_t)sprintf(buf, "%.2e", (double)val);

    char *p = buf;
    if (val < 0) { *p++ = '-'; val = -val; }

    uint32_t int_part = (uint32_t)val;

    /* 用整数运算处理小数部分, 避免浮点累积精度误差 */
    uint32_t frac_int = 0;
    uint32_t frac_div = 1;
    for (uint8_t i = 0; i < decimals; i++) frac_div *= 10;
    float frac = val - (float)int_part;
    frac_int = (uint32_t)(frac * (float)frac_div + 0.5f);
    /* 进位处理: 如 0.9995 * 1000 + 0.5 ≈ 1000 */
    if (frac_int >= frac_div) { int_part++; frac_int -= frac_div; }

    /* 整数部分 */
    char tmp[12];
    uint8_t len = 0;
    if (int_part == 0) { tmp[len++] = '0'; }
    else { while (int_part > 0) { tmp[len++] = '0' + (int_part % 10); int_part /= 10; } }
    for (uint8_t i = 0; i < len; i++) p[i] = tmp[len - 1 - i];
    p += len;

    /* 小数部分 (纯整数运算, 无累积误差) */
    if (decimals > 0) {
        *p++ = '.';
        uint32_t divisor = frac_div / 10;
        for (uint8_t i = 0; i < decimals; i++) {
            *p++ = '0' + (uint8_t)(frac_int / divisor);
            frac_int %= divisor;
            divisor /= 10;
        }
    }
    *p = '\0';
    return (uint16_t)(p - buf);
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  安全字符串拼接 (防止溢出)
 * ═══════════════════════════════════════════════════════════════════════════ */
static char *safe_cat(char *p, const char *end, const char *src)
{
    while (p < end && *src) *p++ = *src++;
    return p;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  TEXT 格式
 * ═══════════════════════════════════════════════════════════════════════════ */
static uint16_t format_text(char *buf, uint16_t buf_size)
{
    char *p = buf;
    char *end = buf + buf_size - 4;

    /* 设备信息头: "设备名称",电压=3300mV; */
    *p++ = '"';
    p = safe_cat(p, end, g_sys_cfg.device_name[0] ? g_sys_cfg.device_name : "N/A");
    *p++ = '"';
    *p++ = ',';
    p = safe_cat(p, end, "电压=");
    /* ADC 原始值 → mV: Vref=3.3V, 12bit → raw * 3300 / 4096 */
    uint32_t mv = ((uint32_t)g_adc_voltage_raw * 3300 + 2048) / 4096;
    p += ftostr((float)mv, p, 0);
    p = safe_cat(p, end, "mV");
    *p++ = ';';

    for (uint8_t s = 0; s < g_sys_cfg.slave_count && p < end; s++) {
        SlaveCfg_t *cfg = &g_sys_cfg.slaves[s];
        SlaveData_t *data = &g_slave_data[s];

        /* 设备名 */
        *p++ = '"';
        p = safe_cat(p, end, cfg->name[0] ? cfg->name : "N/A");
        *p++ = '"';

        if (!cfg->enabled || !data->online) {
            p = safe_cat(p, end, ",OFFLINE");
        } else {
            for (uint8_t pt = 0; pt < cfg->data_point_count && p < end; pt++) {
                *p++ = ',';
                p = safe_cat(p, end, "P");
                char num[4];
                snprintf(num, sizeof(num), "%u", pt + 1);
                p = safe_cat(p, end, num);
                *p++ = '(';
                *p++ = '"';
                const char *pname = cfg->data_points[pt].name;
                p = safe_cat(p, end, pname[0] ? pname : "N/A");
                *p++ = '"';
                *p++ = ')';
                *p++ = '=';
                if (data->valid[pt]) {
                    p += ftostr(data->values[pt], p, 2);
                } else {
                    p = safe_cat(p, end, "ERR");
                }
            }
        }
        if (s < g_sys_cfg.slave_count - 1) *p++ = ';';
    }
    *p++ = '\r'; *p++ = '\n';
    return (uint16_t)(p - buf);
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  JSON 格式
 * ═══════════════════════════════════════════════════════════════════════════ */
static uint16_t format_json(char *buf, uint16_t buf_size)
{
    char *p = buf;
    char *end = buf + buf_size - 4;

    *p++ = '{';

    /* 设备信息: "device":{"name":"xxx","voltage_mv":3300} */
    p = safe_cat(p, end, "\"device\":{\"name\":\"");
    p = safe_cat(p, end, g_sys_cfg.device_name[0] ? g_sys_cfg.device_name : "");
    p = safe_cat(p, end, "\",\"voltage_mv\":");
    {
        uint32_t mv = ((uint32_t)g_adc_voltage_raw * 3300 + 2048) / 4096;
        p += snprintf(p, (uint32_t)(end - p), "%lu", (unsigned long)mv);
    }
    p = safe_cat(p, end, "},");

    for (uint8_t s = 0; s < g_sys_cfg.slave_count && p < end; s++) {
        SlaveCfg_t *cfg = &g_sys_cfg.slaves[s];
        SlaveData_t *data = &g_slave_data[s];

        /* "sN":{ */
        p += snprintf(p, (uint32_t)(end - p), "\"s%u\":{", s + 1);

        /* "name":"设备名" */
        *p++ = '"'; p = safe_cat(p, end, "name"); *p++ = '"'; *p++ = ':';
        *p++ = '"';
        p = safe_cat(p, end, cfg->name[0] ? cfg->name : "");
        *p++ = '"';

        /* ,"online":N */
        p += snprintf(p, (uint32_t)(end - p), ",\"online\":%u", data->online ? 1 : 0);

        if (cfg->enabled && data->online) {
            p = safe_cat(p, end, ",\"data\":{");
            for (uint8_t pt = 0; pt < cfg->data_point_count && p < end; pt++) {
                /* "点名":value */
                *p++ = '"';
                const char *pname = cfg->data_points[pt].name;
                if (pname[0]) {
                    p = safe_cat(p, end, pname);
                } else {
                    p += snprintf(p, (uint32_t)(end - p), "p%u", pt + 1);
                }
                *p++ = '"'; *p++ = ':';

                if (data->valid[pt]) {
                    p += ftostr(data->values[pt], p, 4);
                } else {
                    p = safe_cat(p, end, "null");
                }
                if (pt < cfg->data_point_count - 1) *p++ = ',';
            }
            *p++ = '}';
        }
        *p++ = '}';
        if (s < g_sys_cfg.slave_count - 1) *p++ = ',';
    }

    *p++ = '}';
    *p++ = '\r'; *p++ = '\n';
    return (uint16_t)(p - buf);
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  HEX 格式: AA 55 [slave_count] [per-slave data] [CRC16]
 * ═══════════════════════════════════════════════════════════════════════════ */
static uint16_t format_hex(char *buf, uint16_t buf_size)
{
    uint16_t pos = 0;

    hex_tmp[pos++] = 0xAA;
    hex_tmp[pos++] = 0x55;
    hex_tmp[pos++] = g_sys_cfg.slave_count;

    /* 设备名称 (20字节, 不足补0) */
    for (uint8_t i = 0; i < NAME_MAX_LEN; i++) {
        hex_tmp[pos++] = (i < strlen(g_sys_cfg.device_name)) ?
                          (uint8_t)g_sys_cfg.device_name[i] : 0;
    }
    /* 电压 ADC 原始值 (2字节大端) */
    hex_tmp[pos++] = (uint8_t)(g_adc_voltage_raw >> 8);
    hex_tmp[pos++] = (uint8_t)(g_adc_voltage_raw & 0xFF);

    for (uint8_t s = 0; s < g_sys_cfg.slave_count && pos + 40 < sizeof(hex_tmp); s++) {
        SlaveCfg_t *cfg = &g_sys_cfg.slaves[s];
        SlaveData_t *data = &g_slave_data[s];

        hex_tmp[pos++] = cfg->slave_addr;
        hex_tmp[pos++] = data->online ? 1 : 0;
        hex_tmp[pos++] = cfg->data_point_count;

        for (uint8_t pt = 0; pt < cfg->data_point_count && pos + 5 < sizeof(hex_tmp); pt++) {
            float fval = data->valid[pt] ? data->values[pt] : 0.0f;
            uint8_t fb[4];
            memcpy(fb, &fval, 4);  /* 安全 type-punning (本地小端) */
            /* 输出大端序 (MSB first), 与 README 示例一致 */
            hex_tmp[pos++] = fb[3]; hex_tmp[pos++] = fb[2];
            hex_tmp[pos++] = fb[1]; hex_tmp[pos++] = fb[0];
        }
    }

    uint16_t crc = MB_CRC16(hex_tmp, pos);
    hex_tmp[pos++] = (uint8_t)(crc & 0xFF);
    hex_tmp[pos++] = (uint8_t)(crc >> 8);

    /* 转 HEX 字符串 */
    static const char hx[] = "0123456789ABCDEF";
    char *p = buf;
    for (uint16_t i = 0; i < pos && p < buf + buf_size - 5; i++) {
        *p++ = hx[(hex_tmp[i] >> 4) & 0x0F];
        *p++ = hx[hex_tmp[i] & 0x0F];
        *p++ = ' ';
    }
    *p++ = '\r'; *p++ = '\n';
    return (uint16_t)(p - buf);
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  格式化单个从机 (调试用)
 * ═══════════════════════════════════════════════════════════════════════════ */
uint16_t REPORT_Format_Slave(uint8_t slave_idx, char *buf, uint16_t buf_size)
{
    if (slave_idx >= g_sys_cfg.slave_count) return 0;

    char *p = buf;
    char *end = buf + buf_size - 4;
    SlaveCfg_t *cfg = &g_sys_cfg.slaves[slave_idx];
    SlaveData_t *data = &g_slave_data[slave_idx];

    p += snprintf(p, (uint32_t)(end - p), "[%u] \"%s\" addr=%u online=%u\r\n",
                  slave_idx, cfg->name, cfg->slave_addr, data->online);

    for (uint8_t pt = 0; pt < cfg->data_point_count && p < end; pt++) {
        p += snprintf(p, (uint32_t)(end - p), "  P%u \"%s\"(reg%u)=",
                      pt + 1, cfg->data_points[pt].name, cfg->data_points[pt].reg_addr);
        if (data->valid[pt]) {
            p += ftostr(data->values[pt], p, 4);
        } else {
            p = safe_cat(p, end, "INVALID");
        }
        *p++ = '\r'; *p++ = '\n';
    }
    return (uint16_t)(p - buf);
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  UART1 发送
 * ═══════════════════════════════════════════════════════════════════════════ */
void REPORT_Send(const char *data, uint16_t len)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)data, len, 500);
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  上报主处理
 *  每轮轮询结束必定上报一次 (不管从机是否在线, 无数据则 online=0)
 *  上报间隔 = 所有已启用从机中, 最短轮询周期
 * ═══════════════════════════════════════════════════════════════════════════ */
void REPORT_Process(void)
{
    static uint32_t last_report_tick = 0;
    uint32_t now = HAL_GetTick();

    /* 上报间隔取最短轮询周期 */
    uint32_t min_period = 0xFFFFFFFF;
    uint8_t has_enabled = 0;
    for (uint8_t s = 0; s < g_sys_cfg.slave_count; s++) {
        if (g_sys_cfg.slaves[s].enabled) {
            has_enabled = 1;
            if (g_sys_cfg.slaves[s].poll_period_ms < min_period) {
                min_period = g_sys_cfg.slaves[s].poll_period_ms;
            }
        }
    }

    /* 无启用从机时跳过上报 */
    if (!has_enabled) return;

    /* 未到上报间隔时跳过 */
    if ((now - last_report_tick) < min_period) return;
    last_report_tick = now;

    uint16_t len = 0;
    switch (g_sys_cfg.report_format) {
        case REPORT_FORMAT_TEXT: len = format_text(report_buf, REPORT_BUF_SIZE); break;
        case REPORT_FORMAT_HEX:  len = format_hex(report_buf, REPORT_BUF_SIZE);  break;
        default:                 len = format_json(report_buf, REPORT_BUF_SIZE); break;
    }

    if (len > 0) REPORT_Send(report_buf, len);
}
