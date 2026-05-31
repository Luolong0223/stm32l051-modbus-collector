/**
 * @file    modbus_driver.c
 * @brief   Modbus RTU 完整驱动 — 主站轮询 + 从站配置 (UART2)
 *
 *  v2.1 改进:
 *    - 修复: 删除重复全局变量定义 (g_sys_cfg, g_slave_data)
 *    - 修复: 恢复 RS485 GPIO 方向控制
 *    - 修复: 删除 MB_UART_Idle_Callback 中的调试代码
 *    - 修复: 名称寄存器单写(0x06)支持
 *    - 优化: sprintf → snprintf 防溢出
 *  v2.2 改进:
 *    - 修复: delay_us SysTick 翻转处理
 *    - 修复: 主站超时后立即尝试下一数据点 (不再跳过)
 *    - 修复: UART1 波特率热更新改为延迟执行
 *    - 修复: EEPROM 保存延迟至主站 IDLE 时执行
 *    - 新增: 从站支持 0x04 (读输入寄存器)
 *    - 新增: 名称字段写入时自动过滤不可打印字符
 *    - 新增: 模式切换时跳过未启用从机
 */
#include "modbus_driver.h"
#include "eeprom_manager.h"

/* ═══════════════════════════════════════════════════════════════════════════
 *  全局实例 (g_sys_cfg 和 g_slave_data 仅在 main_app.c 中定义)
 * ═══════════════════════════════════════════════════════════════════════════ */
MBMasterHandle_t g_mb_master;
MBSlaveHandle_t  g_mb_slave;
volatile RunMode_t        g_run_mode = RUN_MODE_MASTER;
volatile uint8_t g_uart2_reconfig_pending = 0;  /* UART2 延迟重配标志 */
volatile uint8_t g_uart1_reconfig_pending = 0;  /* UART1 延迟重配标志 */
volatile uint8_t g_eeprom_save_pending = 0;     /* EEPROM 延迟保存标志 */
volatile uint16_t g_adc_voltage_raw = 0;        /* PA0 ADC 原始值 (0~4095) */

/* 逐字节接收临时缓冲 */
static uint8_t master_rx_byte;
static uint8_t slave_rx_byte;

/* ═══════════════════════════════════════════════════════════════════════════
 *  CRC16-Modbus 查表
 * ═══════════════════════════════════════════════════════════════════════════ */
static const uint16_t crc16_table[256] = {
    0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
    0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
    0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
    0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
    0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
    0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
    0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
    0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
    0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
    0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
    0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
    0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
    0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
    0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
    0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
    0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
    0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
    0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
    0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
    0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
    0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
    0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
    0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
    0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
    0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
    0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
    0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
    0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
    0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
    0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
    0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
    0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040
};

uint16_t MB_CRC16(const uint8_t *buf, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc = (crc >> 8) ^ crc16_table[(crc ^ buf[i]) & 0xFF];
    }
    return crc;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  RS485 收发方向控制
 * ═══════════════════════════════════════════════════════════════════════════ */
static void delay_us(uint32_t us)
{
    uint32_t ticks = (SystemCoreClock / 1000000) * us;
    if (ticks == 0) return;
    uint32_t start = SysTick->VAL;
    uint32_t load  = SysTick->LOAD + 1;  /* LOAD 是重装值，周期 = LOAD+1 */
    for (;;) {
        uint32_t now = SysTick->VAL;
        /* SysTick 是递减计数器，处理翻转: (start - now) 可能溢出，用模运算修正 */
        uint32_t elapsed = (start >= now) ? (start - now) : (load - (now - start));
        if (elapsed >= ticks) break;
    }
}

void RS485_TX_Enable(void)
{
    HAL_GPIO_WritePin(RS485_DE_PORT, RS485_DE_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(RS485_RE_PORT, RS485_RE_PIN, GPIO_PIN_SET);
    if (g_sys_cfg.rs485_de_delay_us > 0) delay_us(g_sys_cfg.rs485_de_delay_us);
}

void RS485_RX_Enable(void)
{
    if (g_sys_cfg.rs485_re_delay_us > 0) delay_us(g_sys_cfg.rs485_re_delay_us);
    HAL_GPIO_WritePin(RS485_DE_PORT, RS485_DE_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RS485_RE_PORT, RS485_RE_PIN, GPIO_PIN_RESET);
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  初始化
 * ═══════════════════════════════════════════════════════════════════════════ */
void MB_Init(UART_HandleTypeDef *huart)
{
    memset(&g_mb_master, 0, sizeof(g_mb_master));
    memset(&g_mb_slave, 0, sizeof(g_mb_slave));

    g_mb_master.huart = huart;
    g_mb_slave.huart  = huart;
    g_mb_master.state = MB_MASTER_IDLE;
    g_run_mode = RUN_MODE_MASTER;

    /* 启动逐字节接收 (主站模式) */
    HAL_UART_Receive_IT(g_mb_master.huart, &master_rx_byte, 1);
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  HAL UART 接收完成回调 — 逐字节接收, 累积到帧缓冲
 * ═══════════════════════════════════════════════════════════════════════════ */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    uint32_t now = HAL_GetTick();

    if (huart->Instance == USART2) {
        RunMode_t mode = g_run_mode;  /* 快照，防止模式切换竞争 */
        if (mode == RUN_MODE_MASTER) {
            /* 主站接收 */
            if (g_mb_master.rx_pos < MB_RX_BUF_SIZE) {
                g_mb_master.rx_buf[g_mb_master.rx_pos++] = master_rx_byte;
            }
            g_mb_master.last_rx_tick = now;
            HAL_UART_Receive_IT(huart, &master_rx_byte, 1);
        } else {
            /* 从站接收 */
            if (g_mb_slave.rx_pos < MB_RX_BUF_SIZE) {
                g_mb_slave.rx_buf[g_mb_slave.rx_pos++] = slave_rx_byte;
            }
            g_mb_slave.last_rx_tick = now;
            HAL_UART_Receive_IT(huart, &slave_rx_byte, 1);
        }
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  UART 空闲中断回调 — 标记帧完成
 * ═══════════════════════════════════════════════════════════════════════════ */
void MB_UART_Idle_Callback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART2) return;
    __HAL_UART_CLEAR_IDLEFLAG(huart);

    if (g_run_mode == RUN_MODE_MASTER) {
        if (g_mb_master.rx_pos > 0) {
            g_mb_master.frame_ready = 1;
        }
    } else {
        if (g_mb_slave.rx_pos > 0) {
            g_mb_slave.frame_ready = 1;
        }
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  帧接收完成检测 (超时法: 超过 MB_FRAME_DETECT_MS 无新字节 → 帧结束)
 *  在主循环中调用
 * ═══════════════════════════════════════════════════════════════════════════ */
void MB_Check_Frame_Timeout(void)
{
    uint32_t now = HAL_GetTick();

    /* 主站帧检测 */
    if (g_run_mode == RUN_MODE_MASTER &&
        g_mb_master.rx_pos > 0 &&
        !g_mb_master.frame_ready &&
        (now - g_mb_master.last_rx_tick) >= MB_FRAME_DETECT_MS) {
        g_mb_master.frame_ready = 1;
    }

    /* 从站帧检测 */
    if (g_run_mode == RUN_MODE_SLAVE &&
        g_mb_slave.rx_pos > 0 &&
        !g_mb_slave.frame_ready &&
        (now - g_mb_slave.last_rx_tick) >= MB_FRAME_DETECT_MS) {
        g_mb_slave.frame_ready = 1;
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  启动接收 (切换模式时调用)
 * ═══════════════════════════════════════════════════════════════════════════ */
static void MB_Start_Master_Receive(void)
{
    g_mb_master.rx_pos = 0;
    g_mb_master.frame_ready = 0;
    g_mb_master.last_rx_tick = HAL_GetTick();
    HAL_UART_Receive_IT(g_mb_master.huart, &master_rx_byte, 1);
}

static void MB_Start_Slave_Receive(void)
{
    g_mb_slave.rx_pos = 0;
    g_mb_slave.frame_ready = 0;
    g_mb_slave.last_rx_tick = HAL_GetTick();
    HAL_UART_Receive_IT(g_mb_slave.huart, &slave_rx_byte, 1);
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  UART2 物理参数热更新
 * ═══════════════════════════════════════════════════════════════════════════ */
void MB_Reconfigure_UART(void)
{
    UART_HandleTypeDef *huart = g_mb_master.huart;

    HAL_UART_AbortReceive(huart);

    huart->Init.BaudRate = g_sys_cfg.uart2_baudrate;
    switch (g_sys_cfg.uart2_parity) {
        case 1:
            huart->Init.Parity = UART_PARITY_ODD;
            huart->Init.WordLength = UART_WORDLENGTH_9B;
            break;
        case 2:
            huart->Init.Parity = UART_PARITY_EVEN;
            huart->Init.WordLength = UART_WORDLENGTH_9B;
            break;
        default:
            huart->Init.Parity = UART_PARITY_NONE;
            huart->Init.WordLength = UART_WORDLENGTH_8B;
            break;
    }
    huart->Init.StopBits = (g_sys_cfg.uart2_stopbits == 2) ?
                           UART_STOPBITS_2 : UART_STOPBITS_1;

    HAL_UART_Init(huart);
    MB_Start_Master_Receive();
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  数据解析工具
 * ═══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief  按字节序重排 4 字节数据
 * @param  regs       Modbus 寄存器数组 (每元素16bit, 内部小端)
 * @param  reg_count  寄存器数量 (1=16bit, 2=32bit)
 * @param  byte_order 字节序模式
 * @param  out        输出 4 字节 (索引0=最高字节)
 *
 *  对于 32 位数据 (reg_count=2):
 *    Modbus 返回: reg[0]={B1,B0} reg[1]={B3,B2}  (B0=最低字节)
 *    raw[0]=B0 raw[1]=B1 raw[2]=B2 raw[3]=B3
 *
 *    ABCD (Big-Endian):     out = [B3, B2, B1, B0] = [raw[3], raw[2], raw[1], raw[0]]
 *    BADC (字内交换):       out = [B2, B3, B0, B1] = [raw[2], raw[3], raw[0], raw[1]]
 *    CDAB (字交换):         out = [B1, B0, B3, B2] = [raw[1], raw[0], raw[3], raw[2]]
 *    DCBA (Full LE):        out = [B0, B1, B2, B3] = [raw[0], raw[1], raw[2], raw[3]]
 */
static void reorder_bytes(const uint16_t *regs, uint8_t reg_count,
                          uint8_t byte_order, uint8_t *out)
{
    uint8_t raw[4] = {0};
    for (uint8_t i = 0; i < reg_count && i < 2; i++) {
        raw[i * 2]     = (uint8_t)(regs[i] & 0xFF);         /* 低字节 */
        raw[i * 2 + 1] = (uint8_t)((regs[i] >> 8) & 0xFF);  /* 高字节 */
    }

    switch (byte_order) {
        case BYTE_ORDER_ABCD:  /* Big-Endian */
            out[0] = raw[3]; out[1] = raw[2]; out[2] = raw[1]; out[3] = raw[0];
            break;
        case BYTE_ORDER_BADC:  /* 字内字节交换 */
            out[0] = raw[2]; out[1] = raw[3]; out[2] = raw[0]; out[3] = raw[1];
            break;
        case BYTE_ORDER_CDAB:  /* 16位字交换 */
            out[0] = raw[1]; out[1] = raw[0]; out[2] = raw[3]; out[3] = raw[2];
            break;
        case BYTE_ORDER_DCBA:  /* Full Little-Endian */
            out[0] = raw[0]; out[1] = raw[1]; out[2] = raw[2]; out[3] = raw[3];
            break;
        default:
            out[0] = raw[3]; out[1] = raw[2]; out[2] = raw[1]; out[3] = raw[0];
            break;
    }
}

float MB_Parse_Registers(const uint16_t *regs, uint8_t data_type, uint8_t byte_order)
{
    uint8_t reordered[4];

    switch (data_type) {
        case DATA_TYPE_U16:
            return (float)regs[0];

        case DATA_TYPE_I16:
            return (float)(int16_t)regs[0];

        case DATA_TYPE_U32:
            reorder_bytes(regs, 2, byte_order, reordered);
            {
                uint32_t val = ((uint32_t)reordered[0] << 24) |
                               ((uint32_t)reordered[1] << 16) |
                               ((uint32_t)reordered[2] << 8)  |
                               ((uint32_t)reordered[3]);
                return (float)val;
            }

        case DATA_TYPE_I32:
            reorder_bytes(regs, 2, byte_order, reordered);
            {
                int32_t val = ((int32_t)reordered[0] << 24) |
                              ((int32_t)reordered[1] << 16) |
                              ((int32_t)reordered[2] << 8)  |
                              ((int32_t)reordered[3]);
                return (float)val;
            }

        case DATA_TYPE_FLOAT:
            reorder_bytes(regs, 2, byte_order, reordered);
            {
                float fval;
                /* 使用 memcpy 进行安全的 type-punning (符合 C11 标准) */
                memcpy(&fval, reordered, sizeof(float));
                return fval;
            }

        default:
            return 0.0f;
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  主站: 构建请求帧
 * ═══════════════════════════════════════════════════════════════════════════ */
static uint16_t MB_Master_Build_ReadHolding(uint8_t *buf, uint8_t slave_addr,
                                            uint16_t reg_addr, uint16_t reg_count)
{
    buf[0] = slave_addr;
    buf[1] = MB_FC_READ_HOLDING_REGS;
    buf[2] = (uint8_t)(reg_addr >> 8);
    buf[3] = (uint8_t)(reg_addr & 0xFF);
    buf[4] = (uint8_t)(reg_count >> 8);
    buf[5] = (uint8_t)(reg_count & 0xFF);
    uint16_t crc = MB_CRC16(buf, 6);
    buf[6] = (uint8_t)(crc & 0xFF);
    buf[7] = (uint8_t)(crc >> 8);
    return 8;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  主站: 发送
 * ═══════════════════════════════════════════════════════════════════════════ */
static void MB_Master_Send(uint16_t len)
{
    HAL_UART_AbortReceive(g_mb_master.huart);

    RS485_TX_Enable();
    HAL_UART_Transmit(g_mb_master.huart, g_mb_master.tx_buf, len, 200);
    RS485_RX_Enable();

    g_mb_master.tx_end_tick = HAL_GetTick();  /* 记录发送结束时刻 */

    MB_Start_Master_Receive();
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  主站: 验证响应
 * ═══════════════════════════════════════════════════════════════════════════ */
static uint8_t MB_Master_Validate_Response(uint16_t rx_len, uint8_t expected_fc)
{
    if (rx_len < 5) return 0;

    uint8_t fc = g_mb_master.rx_buf[1];

    /* 异常响应 */
    if (fc & 0x80) return 0;

    /* 功能码匹配 */
    if (fc != expected_fc) return 0;

    /* CRC 校验 */
    uint16_t rx_crc = g_mb_master.rx_buf[rx_len - 2] |
                      ((uint16_t)g_mb_master.rx_buf[rx_len - 1] << 8);
    uint16_t calc_crc = MB_CRC16(g_mb_master.rx_buf, rx_len - 2);
    return (rx_crc == calc_crc) ? 1 : 0;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  主站: 处理读保持寄存器响应
 * ═══════════════════════════════════════════════════════════════════════════ */
static void MB_Master_Process_ReadResponse(uint8_t slave_idx, uint8_t point_idx)
{
    SlaveCfg_t *cfg = &g_sys_cfg.slaves[slave_idx];
    DataPointCfg_t *pt = &cfg->data_points[point_idx];

    uint8_t byte_count = g_mb_master.rx_buf[2];
    uint8_t reg_count = byte_count / 2;
    uint8_t need_regs = (pt->data_type <= DATA_TYPE_I16) ? 1 : 2;

    if (reg_count < need_regs) {
        g_slave_data[slave_idx].valid[point_idx] = 0;
        return;
    }

    uint16_t regs[2] = {0, 0};
    for (uint8_t i = 0; i < need_regs; i++) {
        regs[i] = ((uint16_t)g_mb_master.rx_buf[3 + i * 2] << 8) |
                   (uint16_t)g_mb_master.rx_buf[4 + i * 2];
    }

    g_slave_data[slave_idx].values[point_idx] =
        MB_Parse_Registers(regs, pt->data_type, pt->byte_order);
    g_slave_data[slave_idx].valid[point_idx] = 1;
    g_slave_data[slave_idx].online = 1;
    g_slave_data[slave_idx].error_count = 0;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  主站状态机
 * ═══════════════════════════════════════════════════════════════════════════ */
void MB_Master_Process(void)
{
    uint32_t now = HAL_GetTick();

    switch (g_mb_master.state) {

    /* ─── IDLE: 检查是否需要轮询 ─── */
    case MB_MASTER_IDLE: {
        /* 帧间静默: Modbus RTU 要求帧间 ≥ 3.5 字符时间 (9600bps ≈ 4ms) */
        if ((now - g_mb_master.tx_end_tick) < MB_FRAME_SILENT_MS) break;

        /* 延迟重配: 主站空闲时安全执行 UART 重配 */
        if (g_uart2_reconfig_pending) {
            g_uart2_reconfig_pending = 0;
            MB_Reconfigure_UART();
        }
        if (g_uart1_reconfig_pending) {
            g_uart1_reconfig_pending = 0;
            HAL_UART_AbortReceive(&huart1);
            huart1.Init.BaudRate = g_sys_cfg.uart1_baudrate;
            HAL_UART_Init(&huart1);
        }
        /* 延迟保存: 主站空闲时安全写入 EEPROM */
        if (g_eeprom_save_pending) {
            g_eeprom_save_pending = 0;
            EEPROM_Save_Config(&g_sys_cfg);
        }

        for (uint8_t i = 0; i < g_sys_cfg.slave_count; i++) {
            uint8_t idx = (g_mb_master.current_slave + i) % g_sys_cfg.slave_count;
            SlaveCfg_t *s = &g_sys_cfg.slaves[idx];

            if (!s->enabled || s->data_point_count == 0) continue;
            if (g_slave_data[idx].last_poll_tick != 0 &&
                (now - g_slave_data[idx].last_poll_tick) < s->poll_period_ms) continue;

            g_mb_master.current_slave = idx;
            g_mb_master.current_point = 0;
            g_mb_master.retry_cnt = 0;
            /* last_poll_tick 延迟到整轮采集完成后再更新，避免超时导致跳过后续数据点 */

            DataPointCfg_t *pt = &s->data_points[0];
            uint8_t need_regs = (pt->data_type <= DATA_TYPE_I16) ? 1 : 2;
            uint16_t frame_len = MB_Master_Build_ReadHolding(
                g_mb_master.tx_buf, s->slave_addr, pt->reg_addr, need_regs);

            MB_Master_Send(frame_len);
            g_mb_master.state = MB_MASTER_WAIT_RESP;
            g_mb_master.wait_start_tick = now;
            return;
        }
        break;
    }

    /* ─── WAIT_RESP: 等待从机响应 ─── */
    case MB_MASTER_WAIT_RESP: {
        if ((now - g_mb_master.wait_start_tick) > MB_MASTER_TIMEOUT_MS) {
            /* 超时 */
            uint8_t sidx = g_mb_master.current_slave;
            SlaveCfg_t *s = &g_sys_cfg.slaves[sidx];

            g_mb_master.retry_cnt++;
            if (g_mb_master.retry_cnt <= MB_MASTER_RETRY_MAX) {
                /* 重试: 重新发送相同请求 */
                DataPointCfg_t *pt = &s->data_points[g_mb_master.current_point];
                uint8_t need_regs = (pt->data_type <= DATA_TYPE_I16) ? 1 : 2;
                uint16_t frame_len = MB_Master_Build_ReadHolding(
                    g_mb_master.tx_buf, s->slave_addr, pt->reg_addr, need_regs);
                MB_Master_Send(frame_len);
                g_mb_master.wait_start_tick = now;
                return;
            }

            /* 重试耗尽, 标记当前数据点错误 */
            g_slave_data[sidx].error_count++;
            g_slave_data[sidx].valid[g_mb_master.current_point] = 0;
            if (g_slave_data[sidx].error_count >= 3) {
                g_slave_data[sidx].online = 0;
            }

            /* 立即尝试下一个数据点 (不等待下一轮轮询周期) */
            g_mb_master.current_point++;
            g_mb_master.retry_cnt = 0;
            if (g_mb_master.current_point >= s->data_point_count) {
                /* 该从机所有数据点处理完毕 (超时放弃), 切换到下一从机 */
                uint8_t first_time = (g_slave_data[sidx].last_poll_tick == 0);
                g_slave_data[sidx].last_poll_tick = now;

                if (first_time) {
                    g_mb_master.polled_slave_count++;
                    uint8_t enabled_count = 0;
                    for (uint8_t i = 0; i < g_sys_cfg.slave_count; i++) {
                        if (g_sys_cfg.slaves[i].enabled &&
                            g_sys_cfg.slaves[i].data_point_count > 0)
                            enabled_count++;
                    }
                    if (g_mb_master.polled_slave_count >= enabled_count) {
                        g_slave_data[sidx].poll_cycle_completed++;
                        g_mb_master.polled_slave_count = 0;
                    }
                }

                g_mb_master.current_slave = (sidx + 1) % g_sys_cfg.slave_count;
                g_mb_master.state = MB_MASTER_IDLE;
            } else {
                /* 继续该从机的下一个数据点 */
                DataPointCfg_t *pt = &s->data_points[g_mb_master.current_point];
                uint8_t need_regs = (pt->data_type <= DATA_TYPE_I16) ? 1 : 2;
                uint16_t frame_len = MB_Master_Build_ReadHolding(
                    g_mb_master.tx_buf, s->slave_addr, pt->reg_addr, need_regs);
                MB_Master_Send(frame_len);
                g_mb_master.wait_start_tick = now;
            }
            return;
        }

        /* 检查帧完成 */
        if (g_mb_master.frame_ready) {
            g_mb_master.state = MB_MASTER_PROCESSING;
        }
        break;
    }

    /* ─── PROCESSING: 解析响应 ─── */
    case MB_MASTER_PROCESSING: {
        uint8_t sidx = g_mb_master.current_slave;
        uint8_t pidx = g_mb_master.current_point;
        SlaveCfg_t *s = &g_sys_cfg.slaves[sidx];

        if (MB_Master_Validate_Response(g_mb_master.rx_pos, MB_FC_READ_HOLDING_REGS)) {
            MB_Master_Process_ReadResponse(sidx, pidx);
        } else {
            g_slave_data[sidx].error_count++;
            g_slave_data[sidx].valid[pidx] = 0;
        }

        /* 下一个数据点 */
        g_mb_master.current_point++;
        g_mb_master.retry_cnt = 0;
        if (g_mb_master.current_point >= s->data_point_count) {
            /* 该从机整轮采集完成 */
            /* 检测是否为该从机首次被轮询 (last_poll_tick==0 表示从未轮询过) */
            uint8_t first_time = (g_slave_data[sidx].last_poll_tick == 0);
            g_slave_data[sidx].last_poll_tick = now;

            if (first_time) {
                g_mb_master.polled_slave_count++;
                /* 当所有启用从机都至少轮询过一次 → 一个完整周期完成 */
                uint8_t enabled_count = 0;
                for (uint8_t i = 0; i < g_sys_cfg.slave_count; i++) {
                    if (g_sys_cfg.slaves[i].enabled &&
                        g_sys_cfg.slaves[i].data_point_count > 0)
                        enabled_count++;
                }
                if (g_mb_master.polled_slave_count >= enabled_count) {
                    g_slave_data[sidx].poll_cycle_completed++;
                    g_mb_master.polled_slave_count = 0;
                }
            }

            g_mb_master.current_slave = (sidx + 1) % g_sys_cfg.slave_count;
        }
        g_mb_master.state = MB_MASTER_IDLE;
        g_mb_master.frame_ready = 0;
        break;
    }

    default:
        g_mb_master.state = MB_MASTER_IDLE;
        break;
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  从站: 寄存器地址映射
 * ═══════════════════════════════════════════════════════════════════════════ */

#define SLAVE_BLOCK_SIZE    0x0200
#define SLAVE_CFG_BASE(s)   (0x0100 + (s) * SLAVE_BLOCK_SIZE)
#define SLAVE_DP_BASE(s, p) (SLAVE_CFG_BASE(s) + 0x0010 + (p) * 0x0010)
#define SLAVE_NAME_BASE(s)  (SLAVE_CFG_BASE(s) + 0x0005)
#define SLAVE_DP_NAME_BASE(s, p) (SLAVE_DP_BASE(s, p) + 0x0003)

/* 将 uint16 寄存器数组转为字符串 (大端: 高字节在前) */
static void regs_to_str(const uint16_t *regs, uint8_t reg_count,
                         char *str, uint8_t max_len)
{
    uint8_t pos = 0;
    for (uint8_t i = 0; i < reg_count && pos < max_len - 1; i++) {
        str[pos++] = (char)(regs[i] >> 8);       /* 高字节 */
        if (pos < max_len - 1)
            str[pos++] = (char)(regs[i] & 0xFF);  /* 低字节 */
    }
    str[pos] = '\0';
}

/* 将字符串转为 uint16 寄存器数组 */
static void str_to_regs(const char *str, uint16_t *regs, uint8_t *reg_count)
{
    uint8_t len = (uint8_t)strlen(str);
    if (len > NAME_MAX_LEN) len = NAME_MAX_LEN;
    *reg_count = (len + 1) / 2;  /* 向上取整 */
    for (uint8_t i = 0; i < *reg_count; i++) {
        uint8_t hi = (i * 2 < len) ? (uint8_t)str[i * 2] : 0;
        uint8_t lo = (i * 2 + 1 < len) ? (uint8_t)str[i * 2 + 1] : 0;
        regs[i] = ((uint16_t)hi << 8) | lo;
    }
}

/**
 * @brief  读取单个配置寄存器
 */
static uint16_t MB_Slave_Read_Reg(uint16_t reg_addr)
{
    /* ── 系统寄存器 ── */
    switch (reg_addr) {
        case 0x0000: return (uint16_t)(g_sys_cfg.uart2_baudrate & 0xFFFF);
        case 0x0001: return (uint16_t)(g_sys_cfg.uart2_baudrate >> 16);
        case 0x0002: return g_sys_cfg.uart2_parity;
        case 0x0003: return g_sys_cfg.uart2_stopbits;
        case 0x0004: return g_sys_cfg.rs485_de_delay_us;
        case 0x0005: return g_sys_cfg.rs485_re_delay_us;
        case 0x0006: return g_sys_cfg.local_mb_addr;
        case 0x0007: return g_sys_cfg.slave_count;
        case 0x0008: return g_sys_cfg.report_format;
        case 0x0009: return (uint16_t)(g_sys_cfg.uart1_baudrate & 0xFFFF);
        case 0x000A: return (uint16_t)(g_sys_cfg.uart1_baudrate >> 16);
    }

    /* ── 设备名称寄存器 0x000B~0x0014 (10个=20字节) ── */
    if (reg_addr >= 0x000B && reg_addr < 0x000B + 10) {
        uint16_t offset = reg_addr - 0x000B;
        uint16_t regs[10];
        char name_copy[NAME_BUF_SIZE];
        strncpy(name_copy, g_sys_cfg.device_name, NAME_BUF_SIZE - 1);
        name_copy[NAME_BUF_SIZE - 1] = '\0';
        uint8_t rc;
        str_to_regs(name_copy, regs, &rc);
        if (offset < rc) return regs[offset];
        return 0;
    }

    /* ── 电压 ADC 原始值 0x0015 ── */
    if (reg_addr == 0x0015) return g_adc_voltage_raw;

    /* ── 设备运行时间 0x0016~0x0017 (只读, 单位: 秒) ── */
    if (reg_addr == 0x0016) {
        uint32_t sec = HAL_GetTick() / 1000;
        return (uint16_t)(sec & 0xFFFF);
    }
    if (reg_addr == 0x0017) {
        uint32_t sec = HAL_GetTick() / 1000;
        return (uint16_t)(sec >> 16);
    }

    /* ── 低功耗休眠间隔 0x0018 (秒, 0=禁用) ── */
    if (reg_addr == 0x0018) return g_sys_cfg.sleep_interval_sec;

    /* ── 预留寄存器 0x0019~0x001F (只读, 返回 0) ── */
    if (reg_addr >= 0x0019 && reg_addr <= 0x001F) return 0;

    /* ── 从机配置 + 数据点 ── */
    for (uint8_t s = 0; s < MAX_SLAVE_COUNT; s++) {
        uint16_t base = SLAVE_CFG_BASE(s);

        /* 从机基本配置 */
        if (reg_addr == base + 0x00) return g_sys_cfg.slaves[s].slave_addr;
        if (reg_addr == base + 0x01) return g_sys_cfg.slaves[s].enabled;
        if (reg_addr == base + 0x02) return g_sys_cfg.slaves[s].data_point_count;
        if (reg_addr == base + 0x03) return (uint16_t)(g_sys_cfg.slaves[s].poll_period_ms & 0xFFFF);
        if (reg_addr == base + 0x04) return (uint16_t)(g_sys_cfg.slaves[s].poll_period_ms >> 16);

        /* 从机名称 (10个寄存器 = 20字节) */
        if (reg_addr >= SLAVE_NAME_BASE(s) && reg_addr < SLAVE_NAME_BASE(s) + 10) {
            uint16_t offset = reg_addr - SLAVE_NAME_BASE(s);
            uint16_t regs[10];
            char name_copy[NAME_BUF_SIZE];
            strncpy(name_copy, g_sys_cfg.slaves[s].name, NAME_BUF_SIZE - 1);
            name_copy[NAME_BUF_SIZE - 1] = '\0';
            uint8_t rc;
            str_to_regs(name_copy, regs, &rc);
            if (offset < rc) return regs[offset];
            return 0;
        }

        /* 数据点配置 */
        for (uint8_t p = 0; p < MAX_DATA_POINTS; p++) {
            uint16_t dp_base = SLAVE_DP_BASE(s, p);
            if (reg_addr == dp_base + 0x00) return g_sys_cfg.slaves[s].data_points[p].reg_addr;
            if (reg_addr == dp_base + 0x01) return g_sys_cfg.slaves[s].data_points[p].data_type;
            if (reg_addr == dp_base + 0x02) return g_sys_cfg.slaves[s].data_points[p].byte_order;

            /* 数据点名称 (10个寄存器) */
            if (reg_addr >= SLAVE_DP_NAME_BASE(s, p) &&
                reg_addr < SLAVE_DP_NAME_BASE(s, p) + 10) {
                uint16_t offset = reg_addr - SLAVE_DP_NAME_BASE(s, p);
                uint16_t regs[10];
                char name_copy[NAME_BUF_SIZE];
                strncpy(name_copy, g_sys_cfg.slaves[s].data_points[p].name, NAME_BUF_SIZE - 1);
                name_copy[NAME_BUF_SIZE - 1] = '\0';
                uint8_t rc;
                str_to_regs(name_copy, regs, &rc);
                if (offset < rc) return regs[offset];
                return 0;
            }
        }
    }

    return 0;
}

/**
 * @brief  写入单个配置寄存器 (返回 1=成功, 0=非法地址/值)
 */
static uint8_t MB_Slave_Write_Reg(uint16_t reg_addr, uint16_t value)
{
    /* ── 系统寄存器 ── */
    switch (reg_addr) {
        case 0x0000:
            g_sys_cfg.uart2_baudrate = (g_sys_cfg.uart2_baudrate & 0xFFFF0000) | value;
            g_uart2_reconfig_pending = 1;  /* 延迟重配，不在从站处理中执行 */
            return 1;
        case 0x0001:
            g_sys_cfg.uart2_baudrate = (g_sys_cfg.uart2_baudrate & 0x0000FFFF) | ((uint32_t)value << 16);
            g_uart2_reconfig_pending = 1;  /* 延迟重配 */
            return 1;
        case 0x0002:
            if (value > 2) return 0;
            g_sys_cfg.uart2_parity = (uint8_t)value;
            g_uart2_reconfig_pending = 1;
            return 1;
        case 0x0003:
            if (value != 1 && value != 2) return 0;
            g_sys_cfg.uart2_stopbits = (uint8_t)value;
            g_uart2_reconfig_pending = 1;
            return 1;
        case 0x0004:
            g_sys_cfg.rs485_de_delay_us = value;
            return 1;
        case 0x0005:
            g_sys_cfg.rs485_re_delay_us = value;
            return 1;
        case 0x0006:
            if (value < 1 || value > 247) return 0;
            g_sys_cfg.local_mb_addr = (uint8_t)value;
            return 1;
        case 0x0007:
            if (value < 1 || value > MAX_SLAVE_COUNT) return 0;
            g_sys_cfg.slave_count = (uint8_t)value;
            return 1;
        case 0x0008:
            if (value > REPORT_FORMAT_HEX) return 0;
            g_sys_cfg.report_format = (uint8_t)value;
            return 1;
        case 0x0009:
            g_sys_cfg.uart1_baudrate = (g_sys_cfg.uart1_baudrate & 0xFFFF0000) | value;
            return 1;
        case 0x000A:
            g_sys_cfg.uart1_baudrate = (g_sys_cfg.uart1_baudrate & 0x0000FFFF) | ((uint32_t)value << 16);
            /* 延迟重配 UART1: 等主站 IDLE 时安全执行，避免打断当前通信 */
            g_uart1_reconfig_pending = 1;
            return 1;
    }

    /* ── 设备名称寄存器 0x000B~0x0014 (单寄存器写入) ── */
    if (reg_addr >= 0x000B && reg_addr < 0x000B + 10) {
        uint16_t offset = reg_addr - 0x000B;
        uint16_t regs[10] = {0};
        char name_copy[NAME_BUF_SIZE];
        strncpy(name_copy, g_sys_cfg.device_name, NAME_BUF_SIZE - 1);
        name_copy[NAME_BUF_SIZE - 1] = '\0';
        uint8_t rc;
        str_to_regs(name_copy, regs, &rc);
        regs[offset] = value;
        char new_name[NAME_BUF_SIZE] = {0};
        regs_to_str(regs, 10, new_name, NAME_BUF_SIZE);
        strncpy(g_sys_cfg.device_name, new_name, NAME_BUF_SIZE - 1);
        g_sys_cfg.device_name[NAME_BUF_SIZE - 1] = '\0';
        EEPROM_Filter_Name(g_sys_cfg.device_name);
        return 1;
    }

    /* ── 低功耗休眠间隔 0x0018 ── */
    if (reg_addr == 0x0018) {
        g_sys_cfg.sleep_interval_sec = value;  /* 0=禁用, 最大65535秒 */
        return 1;
    }

    /* ── 只读寄存器，写入返回非法地址 ── */
    if (reg_addr >= 0x0015 && reg_addr <= 0x001F) return 0;

    /* ── 从机配置 ── */
    for (uint8_t s = 0; s < MAX_SLAVE_COUNT; s++) {
        uint16_t base = SLAVE_CFG_BASE(s);

        if (reg_addr == base + 0x00) {
            if (value < 1 || value > 247) return 0;
            g_sys_cfg.slaves[s].slave_addr = (uint8_t)value;
            return 1;
        }
        if (reg_addr == base + 0x01) {
            g_sys_cfg.slaves[s].enabled = (value != 0) ? 1 : 0;
            return 1;
        }
        if (reg_addr == base + 0x02) {
            if (value > MAX_DATA_POINTS) return 0;
            g_sys_cfg.slaves[s].data_point_count = (uint8_t)value;
            return 1;
        }
        if (reg_addr == base + 0x03) {
            g_sys_cfg.slaves[s].poll_period_ms =
                (g_sys_cfg.slaves[s].poll_period_ms & 0xFFFF0000) | value;
            return 1;
        }
        if (reg_addr == base + 0x04) {
            g_sys_cfg.slaves[s].poll_period_ms =
                (g_sys_cfg.slaves[s].poll_period_ms & 0x0000FFFF) | ((uint32_t)value << 16);
            if (g_sys_cfg.slaves[s].poll_period_ms < 100)
                g_sys_cfg.slaves[s].poll_period_ms = 100;
            return 1;
        }

        /* 从机名称 — 单个寄存器写入时，更新对应的 2 字节 */
        if (reg_addr >= SLAVE_NAME_BASE(s) && reg_addr < SLAVE_NAME_BASE(s) + 10) {
            uint16_t offset = reg_addr - SLAVE_NAME_BASE(s);
            /* 将当前名称读出为寄存器数组，修改目标位置，再写回 */
            uint16_t regs[10] = {0};
            char name_copy[NAME_BUF_SIZE];
            strncpy(name_copy, g_sys_cfg.slaves[s].name, NAME_BUF_SIZE - 1);
            name_copy[NAME_BUF_SIZE - 1] = '\0';
            uint8_t rc;
            str_to_regs(name_copy, regs, &rc);
            regs[offset] = value;
            char new_name[NAME_BUF_SIZE] = {0};
            regs_to_str(regs, 10, new_name, NAME_BUF_SIZE);
            strncpy(g_sys_cfg.slaves[s].name, new_name, NAME_BUF_SIZE - 1);
            g_sys_cfg.slaves[s].name[NAME_BUF_SIZE - 1] = '\0';
            EEPROM_Filter_Name(g_sys_cfg.slaves[s].name);
            return 1;
        }

        /* 数据点配置 */
        for (uint8_t p = 0; p < MAX_DATA_POINTS; p++) {
            uint16_t dp_base = SLAVE_DP_BASE(s, p);
            if (reg_addr == dp_base + 0x00) {
                g_sys_cfg.slaves[s].data_points[p].reg_addr = value;
                return 1;
            }
            if (reg_addr == dp_base + 0x01) {
                if (value > DATA_TYPE_FLOAT) return 0;
                g_sys_cfg.slaves[s].data_points[p].data_type = (uint8_t)value;
                return 1;
            }
            if (reg_addr == dp_base + 0x02) {
                if (value > BYTE_ORDER_DCBA) return 0;
                g_sys_cfg.slaves[s].data_points[p].byte_order = (uint8_t)value;
                return 1;
            }

            /* 数据点名称 — 单个寄存器写入 */
            if (reg_addr >= SLAVE_DP_NAME_BASE(s, p) &&
                reg_addr < SLAVE_DP_NAME_BASE(s, p) + 10) {
                uint16_t offset = reg_addr - SLAVE_DP_NAME_BASE(s, p);
                uint16_t regs[10] = {0};
                char name_copy[NAME_BUF_SIZE];
                strncpy(name_copy, g_sys_cfg.slaves[s].data_points[p].name, NAME_BUF_SIZE - 1);
                name_copy[NAME_BUF_SIZE - 1] = '\0';
                uint8_t rc;
                str_to_regs(name_copy, regs, &rc);
                regs[offset] = value;
                char new_name[NAME_BUF_SIZE] = {0};
                regs_to_str(regs, 10, new_name, NAME_BUF_SIZE);
                strncpy(g_sys_cfg.slaves[s].data_points[p].name, new_name, NAME_BUF_SIZE - 1);
                g_sys_cfg.slaves[s].data_points[p].name[NAME_BUF_SIZE - 1] = '\0';
                EEPROM_Filter_Name(g_sys_cfg.slaves[s].data_points[p].name);
                return 1;
            }
        }
    }

    return 0;
}

/**
 * @brief  处理名称寄存器的批量写入 (0x10 专用)
 * @note   当写入范围与名称寄存器有重叠时, 将寄存器值转回字符串存入配置
 */
static void MB_Slave_Commit_NameWrites(uint16_t start_addr, uint16_t count,
                                        const uint8_t *data)
{
    uint16_t end_addr = start_addr + count;

    /* ── 设备名称 (0x000B~0x0014) ── */
    {
        uint16_t dn_base = 0x000B;
        uint16_t dn_end  = dn_base + 10;
        if (start_addr < dn_end && end_addr > dn_base) {
            uint16_t regs[10] = {0};
            char name_copy[NAME_BUF_SIZE];
            strncpy(name_copy, g_sys_cfg.device_name, NAME_BUF_SIZE - 1);
            name_copy[NAME_BUF_SIZE - 1] = '\0';
            uint8_t rc;
            str_to_regs(name_copy, regs, &rc);

            uint16_t overlap_start = (start_addr > dn_base) ? start_addr : dn_base;
            uint16_t overlap_end   = (end_addr < dn_end) ? end_addr : dn_end;
            for (uint16_t addr = overlap_start; addr < overlap_end; addr++) {
                uint16_t off = addr - dn_base;
                uint16_t data_off = addr - start_addr;
                regs[off] = ((uint16_t)data[data_off * 2] << 8) | data[data_off * 2 + 1];
            }

            char name[NAME_BUF_SIZE] = {0};
            regs_to_str(regs, 10, name, NAME_BUF_SIZE);
            strncpy(g_sys_cfg.device_name, name, NAME_BUF_SIZE - 1);
            g_sys_cfg.device_name[NAME_BUF_SIZE - 1] = '\0';
            EEPROM_Filter_Name(g_sys_cfg.device_name);
        }
    }

    for (uint8_t s = 0; s < MAX_SLAVE_COUNT; s++) {
        /* 设备名称: 检查写入范围与名称区域是否有重叠 */
        uint16_t name_base = SLAVE_NAME_BASE(s);
        uint16_t name_end  = name_base + 10;
        if (start_addr < name_end && end_addr > name_base) {
            /* 有重叠: 从写入数据中提取覆盖的部分 */
            uint16_t regs[10] = {0};
            /* 先读出当前名称作为基础 */
            char name_copy[NAME_BUF_SIZE];
            strncpy(name_copy, g_sys_cfg.slaves[s].name, NAME_BUF_SIZE - 1);
            name_copy[NAME_BUF_SIZE - 1] = '\0';
            uint8_t rc;
            str_to_regs(name_copy, regs, &rc);

            /* 用写入数据覆盖重叠部分 */
            uint16_t overlap_start = (start_addr > name_base) ? start_addr : name_base;
            uint16_t overlap_end   = (end_addr < name_end) ? end_addr : name_end;
            for (uint16_t addr = overlap_start; addr < overlap_end; addr++) {
                uint16_t off = addr - name_base;
                uint16_t data_off = addr - start_addr;
                regs[off] = ((uint16_t)data[data_off * 2] << 8) | data[data_off * 2 + 1];
            }

            char name[NAME_BUF_SIZE] = {0};
            regs_to_str(regs, 10, name, NAME_BUF_SIZE);
            strncpy(g_sys_cfg.slaves[s].name, name, NAME_BUF_SIZE - 1);
            g_sys_cfg.slaves[s].name[NAME_BUF_SIZE - 1] = '\0';
            EEPROM_Filter_Name(g_sys_cfg.slaves[s].name);
        }

        /* 数据点名称: 同样检查重叠 */
        for (uint8_t p = 0; p < MAX_DATA_POINTS; p++) {
            uint16_t dp_name_base = SLAVE_DP_NAME_BASE(s, p);
            uint16_t dp_name_end  = dp_name_base + 10;
            if (start_addr < dp_name_end && end_addr > dp_name_base) {
                uint16_t regs[10] = {0};
                char name_copy[NAME_BUF_SIZE];
                strncpy(name_copy, g_sys_cfg.slaves[s].data_points[p].name, NAME_BUF_SIZE - 1);
                name_copy[NAME_BUF_SIZE - 1] = '\0';
                uint8_t rc;
                str_to_regs(name_copy, regs, &rc);

                uint16_t overlap_start = (start_addr > dp_name_base) ? start_addr : dp_name_base;
                uint16_t overlap_end   = (end_addr < dp_name_end) ? end_addr : dp_name_end;
                for (uint16_t addr = overlap_start; addr < overlap_end; addr++) {
                    uint16_t off = addr - dp_name_base;
                    uint16_t data_off = addr - start_addr;
                    regs[off] = ((uint16_t)data[data_off * 2] << 8) | data[data_off * 2 + 1];
                }

                char name[NAME_BUF_SIZE] = {0};
                regs_to_str(regs, 10, name, NAME_BUF_SIZE);
                strncpy(g_sys_cfg.slaves[s].data_points[p].name, name, NAME_BUF_SIZE - 1);
                g_sys_cfg.slaves[s].data_points[p].name[NAME_BUF_SIZE - 1] = '\0';
                EEPROM_Filter_Name(g_sys_cfg.slaves[s].data_points[p].name);
            }
        }
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  从站: 响应构建
 * ═══════════════════════════════════════════════════════════════════════════ */
static uint16_t MB_Slave_Build_Exception(uint8_t *buf, uint8_t fc, uint8_t ex_code)
{
    buf[0] = g_sys_cfg.local_mb_addr;
    buf[1] = fc | 0x80;
    buf[2] = ex_code;
    uint16_t crc = MB_CRC16(buf, 3);
    buf[3] = (uint8_t)(crc & 0xFF);
    buf[4] = (uint8_t)(crc >> 8);
    return 5;
}

static uint16_t MB_Slave_Build_ReadResponse(uint8_t *buf, uint8_t fc,
                                            const uint16_t *values, uint16_t count)
{
    buf[0] = g_sys_cfg.local_mb_addr;
    buf[1] = fc;
    buf[2] = (uint8_t)(count * 2);
    uint16_t pos = 3;
    for (uint16_t i = 0; i < count; i++) {
        buf[pos++] = (uint8_t)(values[i] >> 8);
        buf[pos++] = (uint8_t)(values[i] & 0xFF);
    }
    uint16_t crc = MB_CRC16(buf, pos);
    buf[pos++] = (uint8_t)(crc & 0xFF);
    buf[pos++] = (uint8_t)(crc >> 8);
    return pos;
}

static uint16_t MB_Slave_Build_WriteSingle(uint8_t *buf, uint16_t reg_addr, uint16_t value)
{
    buf[0] = g_sys_cfg.local_mb_addr;
    buf[1] = MB_FC_WRITE_SINGLE_REG;
    buf[2] = (uint8_t)(reg_addr >> 8);
    buf[3] = (uint8_t)(reg_addr & 0xFF);
    buf[4] = (uint8_t)(value >> 8);
    buf[5] = (uint8_t)(value & 0xFF);
    uint16_t crc = MB_CRC16(buf, 6);
    buf[6] = (uint8_t)(crc & 0xFF);
    buf[7] = (uint8_t)(crc >> 8);
    return 8;
}

static uint16_t MB_Slave_Build_WriteMultiple(uint8_t *buf, uint16_t reg_addr, uint16_t reg_count)
{
    buf[0] = g_sys_cfg.local_mb_addr;
    buf[1] = MB_FC_WRITE_MULTIPLE_REGS;
    buf[2] = (uint8_t)(reg_addr >> 8);
    buf[3] = (uint8_t)(reg_addr & 0xFF);
    buf[4] = (uint8_t)(reg_count >> 8);
    buf[5] = (uint8_t)(reg_count & 0xFF);
    uint16_t crc = MB_CRC16(buf, 6);
    buf[6] = (uint8_t)(crc & 0xFF);
    buf[7] = (uint8_t)(crc >> 8);
    return 8;
}

static void MB_Slave_Send(uint16_t len)
{
    HAL_UART_AbortReceive(g_mb_slave.huart);
    RS485_TX_Enable();
    HAL_UART_Transmit(g_mb_slave.huart, g_mb_slave.tx_buf, len, 200);
    RS485_RX_Enable();
    MB_Start_Slave_Receive();
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  从站: 处理收到的请求帧
 * ═══════════════════════════════════════════════════════════════════════════ */
void MB_Slave_Handle_Request(void)
{
    uint8_t *rx = g_mb_slave.rx_buf;
    uint16_t rx_len = g_mb_slave.rx_pos;

    if (rx_len < 4) return;

    /* CRC 校验 */
    uint16_t rx_crc = rx[rx_len - 2] | ((uint16_t)rx[rx_len - 1] << 8);
    if (MB_CRC16(rx, rx_len - 2) != rx_crc) return;

    /* 地址匹配 */
    uint8_t is_broadcast = (rx[0] == 0);
    if (rx[0] != g_sys_cfg.local_mb_addr && !is_broadcast) return;

    uint8_t fc = rx[1];
    uint16_t resp_len = 0;

    switch (fc) {
    /* ── 0x03: 读保持寄存器 / 0x04: 读输入寄存器 (响应格式相同) ── */
    case MB_FC_READ_HOLDING_REGS:
    case MB_FC_READ_INPUT_REGS: {
        uint16_t start_addr = ((uint16_t)rx[2] << 8) | rx[3];
        uint16_t reg_count  = ((uint16_t)rx[4] << 8) | rx[5];

        if (reg_count < 1 || reg_count > 125) {
            resp_len = MB_Slave_Build_Exception(g_mb_slave.tx_buf, fc, MB_EX_ILLEGAL_DATA_VALUE);
            break;
        }

        uint16_t values[125];
        for (uint16_t i = 0; i < reg_count; i++) {
            values[i] = MB_Slave_Read_Reg(start_addr + i);
        }
        resp_len = MB_Slave_Build_ReadResponse(g_mb_slave.tx_buf, fc, values, reg_count);
        break;
    }

    /* ── 0x06: 写单个寄存器 ── */
    case MB_FC_WRITE_SINGLE_REG: {
        uint16_t reg_addr = ((uint16_t)rx[2] << 8) | rx[3];
        uint16_t value    = ((uint16_t)rx[4] << 8) | rx[5];

        if (!MB_Slave_Write_Reg(reg_addr, value)) {
            resp_len = MB_Slave_Build_Exception(g_mb_slave.tx_buf, fc, MB_EX_ILLEGAL_DATA_ADDR);
        } else {
            resp_len = MB_Slave_Build_WriteSingle(g_mb_slave.tx_buf, reg_addr, value);
            g_eeprom_save_pending = 1;  /* 延迟保存，避免中断中操作 Flash */
        }
        break;
    }

    /* ── 0x10: 写多个寄存器 (批量写入, 只存一次 EEPROM) ── */
    case MB_FC_WRITE_MULTIPLE_REGS: {
        uint16_t start_addr = ((uint16_t)rx[2] << 8) | rx[3];
        uint16_t reg_count  = ((uint16_t)rx[4] << 8) | rx[5];
        uint8_t  byte_count = rx[6];

        if (reg_count < 1 || reg_count > 123 || byte_count != reg_count * 2) {
            resp_len = MB_Slave_Build_Exception(g_mb_slave.tx_buf, fc, MB_EX_ILLEGAL_DATA_VALUE);
            break;
        }

        /* 第一遍: 写非名称寄存器 (跳过名称区域, 避免双重写入) */
        uint8_t write_ok = 1;
        for (uint16_t i = 0; i < reg_count; i++) {
            uint16_t addr = start_addr + i;
            uint16_t value = ((uint16_t)rx[7 + i * 2] << 8) | rx[8 + i * 2];

            /* 检查是否为名称寄存器, 如果是则跳过, 留给 Commit_NameWrites 处理 */
            /* 设备名称: 0x000B~0x0014, 从机名称: base+5~base+14, 数据点名称: dp_base+3~dp_base+12 */
            uint8_t is_name_reg = 0;
            if (addr >= 0x000B && addr < 0x000B + 10) {
                is_name_reg = 1;  /* 设备名称 */
            } else {
                for (uint8_t s = 0; s < MAX_SLAVE_COUNT && !is_name_reg; s++) {
                    uint16_t s_base = SLAVE_CFG_BASE(s);
                    if (addr >= s_base + 0x0005 && addr < s_base + 0x0005 + 10) {
                        is_name_reg = 1;  /* 从机名称 */
                    } else {
                        for (uint8_t p = 0; p < MAX_DATA_POINTS && !is_name_reg; p++) {
                            uint16_t dp_base = SLAVE_DP_BASE(s, p);
                            if (addr >= dp_base + 0x0003 && addr < dp_base + 0x0003 + 10)
                                is_name_reg = 1;  /* 数据点名称 */
                        }
                    }
                }
            }

            if (is_name_reg) continue;  /* 名称寄存器由 Commit_NameWrites 统一处理 */

            if (!MB_Slave_Write_Reg(addr, value)) {
                write_ok = 0;
                break;
            }
        }

        if (!write_ok) {
            resp_len = MB_Slave_Build_Exception(g_mb_slave.tx_buf, fc, MB_EX_ILLEGAL_DATA_ADDR);
        } else {
            /* 第二遍: 统一提交名称寄存器的批量写入 */
            MB_Slave_Commit_NameWrites(start_addr, reg_count, &rx[7]);

            resp_len = MB_Slave_Build_WriteMultiple(g_mb_slave.tx_buf, start_addr, reg_count);
            g_eeprom_save_pending = 1;  /* 延迟保存，避免中断中操作 Flash */
        }
        break;
    }

    default:
        resp_len = MB_Slave_Build_Exception(g_mb_slave.tx_buf, fc, MB_EX_ILLEGAL_FUNCTION);
        break;
    }

    if (resp_len > 0 && !is_broadcast) {
        MB_Slave_Send(resp_len);
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  从站周期处理 (配置模式下调用)
 *  仅处理收到的请求帧，不做模式切换 (模式由 PA15 硬件决定)
 * ═══════════════════════════════════════════════════════════════════════════ */
void MB_Slave_Process(void)
{
    if (g_run_mode != RUN_MODE_SLAVE) return;

    if (g_mb_slave.frame_ready) {
        MB_Slave_Handle_Request();
        g_mb_slave.frame_ready = 0;
        g_mb_slave.rx_pos = 0;
        /* 处理完一帧后重新启动接收，准备下一帧 */
        MB_Start_Slave_Receive();
    }
}

/**
 * @brief  主从模式切换 (由 PA15 硬件引脚触发，仅在模式变化时调用)
 */
void MB_Switch_To_Slave(void)
{
    HAL_UART_AbortReceive(g_mb_master.huart);
    g_mb_master.frame_ready = 0;
    g_mb_master.rx_pos = 0;
    g_mb_master.state = MB_MASTER_IDLE;
    g_run_mode = RUN_MODE_SLAVE;
    MB_Start_Slave_Receive();
}

void MB_Switch_To_Master(void)
{
    HAL_UART_AbortReceive(g_mb_master.huart);
    g_run_mode = RUN_MODE_MASTER;
    g_mb_master.state = MB_MASTER_IDLE;
    g_mb_master.current_slave = 0;
    g_mb_master.current_point = 0;
    g_mb_master.retry_cnt = 0;
    g_mb_master.polled_slave_count = 0;
    /* 跳过未启用的从机 */
    for (uint8_t i = 0; i < g_sys_cfg.slave_count; i++) {
        if (g_sys_cfg.slaves[i].enabled && g_sys_cfg.slaves[i].data_point_count > 0) {
            g_mb_master.current_slave = i;
            break;
        }
    }
    MB_Start_Master_Receive();
}
