# STM32L051 Modbus RTU 远程数据采集上报系统 v2.1

基于 STM32L051K8 的 Modbus RTU 多设备数据采集系统。

## 功能特性

- **UART2 (RS485)**: Modbus RTU 主站轮询 + 从站配置（主从分时复用）
- **UART1**: 数据上报（TEXT / JSON / HEX 三种格式）
- **内部 EEPROM (2KB)**: 配置断电保存
- **远程配置**: 通过 Modbus 寄存器读写所有参数
- **设备/数据点命名**: 每台设备、每个数据点可独立配置 20 字符名称

## v2.1 改进 (相对 v2.0)

| 项目 | v2.0 | v2.1 |
|------|------|------|
| 全局变量 | modbus_driver.c 和 main_app.c 重复定义 | 仅 main_app.c 定义，消除链接冲突 |
| RS485 GPIO | 被注释掉，通信不工作 | 恢复 DE/RE 方向控制 |
| 调试代码 | MB_UART_Idle_Callback 中有 UART1 调试输出 | 已移除，避免破坏数据上报 |
| 名称单写 | 0x06 写名称寄存器被忽略 | 正确解析并更新名称字符串 |
| 主从切换 | 从站模式无超时，可能阻塞主站轮询 | 增加 50ms 从站监听超时，自动切回 |
| sprintf 安全 | sprintf 无边界检查 | snprintf 防溢出 |
| 栈压力 | format_hex 局部分配 256 字节 | 改为 static 缓冲区 |
| 结构体对齐 | 依赖编译器 | 增加 _Static_assert 编译期校验 |
| 头文件 | RS485 引脚宏缺失 | 在 modbus_driver.h 中定义 |

## 文件结构

```
Core/
├── Inc/
│   ├── sys_config.h          # 全局配置、数据结构、寄存器映射
│   ├── modbus_driver.h       # Modbus 驱动接口
│   ├── eeprom_manager.h      # EEPROM 管理接口
│   └── report_manager.h      # 上报管理接口
└── Src/
    ├── modbus_driver.c       # Modbus 主站+从站完整实现
    ├── eeprom_manager.c      # EEPROM 读写 + 配置持久化
    ├── report_manager.c      # 三种格式数据上报（含名称）
    ├── main_app.c            # 应用主逻辑（初始化+主循环）
    └── stm32l0xx_it_custom.c # 中断集成说明（不编译，参考用）
```

## CubeMX 配置要求

### UART1（数据上报）
- 模式: Asynchronous | 波特率: 9600 | 数据位: 8 | 停止位: 1 | 校验: None

### UART2（Modbus RTU）
- 模式: Asynchronous | 波特率: 9600 | 数据位: 8 | 停止位: 1 | 校验: None
- **NVIC**: 使能 USART2 全局中断

### GPIO（RS485 方向控制）
- PA8: Output Push-Pull（DE — 发送使能）
- PA9: Output Push-Pull（RE — 接收使能，低有效）

## 集成步骤

### 1. 复制文件
```bash
cp Core/Inc/*.h   <你的项目>/Core/Inc/
cp Core/Src/*.c   <你的项目>/Core/Src/
# stm32l0xx_it_custom.c 不要复制，参考其中说明
```

### 2. 修改 main.c
```c
/* USER CODE BEGIN Includes */
#include "sys_config.h"
#include "modbus_driver.h"
#include "eeprom_manager.h"
#include "report_manager.h"

/* USER CODE BEGIN 2 */
System_Init();

/* USER CODE BEGIN 3 */
System_MainLoop();
```

### 3. 修改 stm32l0xx_it.c
```c
void USART2_IRQHandler(void)
{
    /* USER CODE BEGIN USART2_IRQn 0 */
    if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_IDLE)) {
        MB_UART_Idle_Callback(&huart2);
    }
    /* USER CODE END USART2_IRQn 0 */
    HAL_UART_IRQHandler(&huart2);
    /* USER CODE BEGIN USART2_IRQn 1 */
    /* USER CODE END USART2_IRQn 1 */
}
```

### 4. 确认无重复回调
modbus_driver.c 已实现 `HAL_UART_RxCpltCallback`，确保 CubeMX 生成代码中无重复定义。

## Modbus 寄存器地址映射

### 系统寄存器（0x0000 ~ 0x000A）

| 地址 | 名称 | 读写 | 说明 |
|------|------|:----:|------|
| 0x0000 | uart2_baudrate_lo | RW | UART2 波特率低 16 位 |
| 0x0001 | uart2_baudrate_hi | RW | 高 16 位，**写入立即重载** |
| 0x0002 | uart2_parity | RW | 0=无 1=奇 2=偶 |
| 0x0003 | uart2_stopbits | RW | 1 或 2 |
| 0x0004 | rs485_de_delay | RW | DE 延时 (us) |
| 0x0005 | rs485_re_delay | RW | RE 延时 (us) |
| 0x0006 | local_mb_addr | RW | 本机 Modbus 地址 (1~247) |
| 0x0007 | slave_count | RW | 采集从机数 (1~5) |
| 0x0008 | report_format | RW | 0=TEXT 1=JSON 2=HEX |
| 0x0009 | uart1_baudrate_lo | RW | UART1 波特率低 16 位 |
| 0x000A | uart1_baudrate_hi | RW | 高 16 位，**写入立即生效** |

### 从机 N 配置（基地址 = 0x0100 + N × 0x0200，N = 0~4）

| 偏移 | 名称 | 读写 | 说明 |
|------|------|:----:|------|
| +0x00 | slave_addr | RW | 从机 Modbus 地址 (1~247) |
| +0x01 | enabled | RW | 1=启用 0=停用 |
| +0x02 | data_point_count | RW | 数据点数量 (0~8) |
| +0x03 | poll_period_lo | RW | 轮询周期低 16 位 (ms) |
| +0x04 | poll_period_hi | RW | 轮询周期高 16 位 |
| +0x05~+0x0E | **name[20]** | RW | **设备名称**（10 个寄存器 = 20 字节） |

### 从机 N 数据点 M（基地址 = 从机基 + 0x0010 + M × 0x0010，M = 0~7）

| 偏移 | 名称 | 读写 | 说明 |
|------|------|:----:|------|
| +0x00 | reg_addr | RW | 从机寄存器起始地址 |
| +0x01 | data_type | RW | 0=U16 1=I16 2=U32 3=I32 4=Float |
| +0x02 | byte_order | RW | 0=ABCD 1=BADC 2=CDAB 3=DCBA |
| +0x03~+0x0C | **name[20]** | RW | **数据点名称**（10 个寄存器 = 20 字节） |

## 上报格式示例

### TEXT
```
"1号温湿度",P1("温度")=23.50,P2("湿度")=65.00;"2号压力",P1("压力")=101.32;OFFLINE
```

### JSON
```json
{"s1":{"name":"1号温湿度","online":1,"data":{"温度":23.5,"湿度":65.0}},"s2":{"name":"2号压力","online":1,"data":{"压力":101.32}}}
```

### HEX
```
AA 55 02 01 01 02 00 00 BC 41 00 00 82 42 02 01 01 00 00 CA 42 [CRC_L] [CRC_H]
```

## 注意事项

1. **EEPROM 寿命**: 10 万次写入。批量配置用 0x10 一次写入，避免反复 0x06。
2. **名称编码**: 名称字段按原始字节存储，支持 ASCII/GBK/UTF-8（由上位机决定编码）。
3. **RAM 用量**: sizeof(SystemCfg_t) ≈ 1352 字节，STM32L051 有 8KB RAM，足够。
4. **主从复用**: UART2 主从共用，主站 IDLE 间隙自动切入从站监听（50ms 超时自动切回）。
5. **编译要求**: 需要 C11 支持（`_Static_assert`），GCC/ARMCC 默认支持。

## License

MIT
