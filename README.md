# STM32L051 Modbus RTU 远程数据采集上报系统 v2.2

基于 STM32L051K8 的 Modbus RTU 多设备数据采集系统。

## 功能特性

- **PA15 模式选择**: HIGH=配置模式(从站), LOW=轮询上报模式(主站), 硬件切换无抖动
- **UART2 (RS485)**: Modbus RTU 主站轮询 + 从站配置
- **UART1**: 数据上报（TEXT / JSON / HEX 三种格式）
- **内部 EEPROM (2KB)**: 配置断电保存
- **远程配置**: 通过 Modbus 寄存器读写所有参数
- **设备/数据点命名**: 每台设备、每个数据点可独立配置 20 字符名称

## v2.2 改进 (相对 v2.1)

| 项目 | v2.1 | v2.2 |
|------|------|------|
| 主从切换 | 软件分时复用，易丢包抖动 | **PA15 硬件选择**，配置/轮询完全隔离 |
| 模式切换 | 主站 IDLE 间隙切入从站 | 仅在 PA15 电平变化时切换一次 |
| UART 热更新 | 从站处理中直接重配 UART | 延迟至主站 IDLE 时安全执行 (UART1+UART2) |
| EEPROM 磨损 | 每次写入都刷 EEPROM | 变更检测 + **延迟保存** (主站 IDLE 时写入) |
| 主站重试 | 超时后直接跳过 | 自动重试 2 再放弃, **失败后立即尝试下一数据点** |
| 广播处理 | 广播报文也发送响应 | 广播不回复 (符合 Modbus 规范) |
| HEX 缓冲区 | 512 字节，满载时溢出 | 增大至 640 字节 |
| float type-punning | union 未定义行为 | 使用 memcpy (符合 C11 标准) |
| **HEX 字节序** | 按本地小端输出 | **按大端 (MSB first) 输出，与示例一致** |
| **名称过滤** | 无 | **写入时自动过滤不可打印字符** |
| **浮点精度** | 累积误差导致尾数偏差 | **纯整数运算，无累积误差** |
| **从站功能码** | 仅 0x03 | **0x03 + 0x04 均支持** |

## v2.2.1 Bug 修复

| 问题 | 影响 | 修复 |
|------|------|------|
| `EEPROM_Filter_Name` 逻辑恒真 | 控制字符 (0x01~0x1F) 无法过滤 | 修正条件判断 |
| 从站模式不执行 EEPROM 保存 | 配置模式下写入的参数断电即丢 | 从站循环中补充延迟保存检查 |
| 模式切换主站状态残留 | 切回主站时可能处理垃圾数据 | `MB_Switch_To_Slave` 清理主站状态 |
| UART 回调模式竞争 | 模式切换瞬间数据写入错误缓冲 | 回调中快照 `g_run_mode` |
| 上报无新数据仍发送 | 重复上报旧数据浪费带宽 | 增加新数据检测，无新数据跳过 |
| `stm32l0xx_it_custom.c` 误编译 | 放在 Src 目录易被加入编译 | 移至 `docs/` 并改名 `.c.txt` |

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
└── docs/
    ├── STM32L051_Modbus_配置手册.xlsx
    └── stm32l0xx_it_custom.c.txt  # 中断集成参考（移至 docs 避免误编译）
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

### GPIO（模式选择）
- PA15: Input with Pull-Down（**HIGH=配置模式(从站), LOW=轮询上报模式(主站)**）
- 默认下拉 → 上电即为轮询上报模式

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
while (1)
{
    System_MainLoop();
}
```

### 3. 修改 stm32l0xx_it.c
参考 `docs/stm32l0xx_it_custom.c.txt` 中的说明。
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
| 0x000B~0x0014 | device_name[10] | RW | **设备名称**（10 个寄存器 = 20 字节） |
| 0x0015 | voltage_adc_raw | R | **PA0 电压 ADC 原始值** (0~4095, 只读) |

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
"3号线采集器",电压=3300mV;"1号温湿度",P1("温度")=23.50,P2("湿度")=65.00;"2号压力",P1("压力")=101.32;OFFLINE
```

### JSON
```json
{"device":{"name":"3号线采集器","voltage_mv":3300},"s1":{"name":"1号温湿度","online":1,"data":{"温度":23.5,"湿度":65.0}},"s2":{"name":"2号压力","online":1,"data":{"压力":101.32}}}
```

### HEX
```
AA 55 02 [设备名称20字节] [电压ADC_H] [电压ADC_L] 01 01 02 00 00 BC 41 00 00 82 42 02 01 01 00 00 CA 42 [CRC_L] [CRC_H]
```

## 注意事项

1. **EEPROM 寿命**: 10 万次写入。内置变更检测保护——数据未变化时自动跳过写入。批量配置用 0x10 一次写入，避免反复 0x06。写入操作延迟至主站 IDLE 时执行，避免中断上下文中操作 Flash。
2. **名称编码**: 名称字段按原始字节存储，支持 ASCII/GBK/UTF-8（由上位机决定编码）。写入时自动过滤不可打印字符（0x01~0x1F）。
3. **RAM 用量**: sizeof(SystemCfg_t) ≈ 1332 字节，STM32L051 有 8KB RAM，足够。
4. **模式选择**: PA15 硬件引脚控制。HIGH=配置模式(从站监听), LOW=轮询上报模式(主站采集+上报)。仅在电平变化时切换一次，无软件抖动。PA15 默认下拉，上电即为轮询上报模式。
5. **编译要求**: 需要 C11 支持（`_Static_assert`），GCC/ARMCC 默认支持。`docs/stm32l0xx_it_custom.c.txt` **不要参与编译**，仅作为中断集成参考。
6. **UART 热更新**: 通过 Modbus 写入 UART1/UART2 参数后，设置延迟标志——主站 IDLE 时自动执行重配，避免中断配置模式下的通信。
7. **主站重试**: 单个数据点超时后自动重试 2 次（可通过 MB_MASTER_RETRY_MAX 调整）。重试耗尽后立即尝试该从机的下一个数据点，不等待下一轮轮询周期。
8. **广播**: 收到地址 0 的广播请求时会正常处理配置变更，但不会发送响应（符合 Modbus RTU 规范）。
9. **从站功能码**: 支持 0x03 (读保持寄存器) 和 0x04 (读输入寄存器)，两者响应格式相同。

## License

MIT

## 配置上位机

`config_tool/` 目录提供了一个基于 Web UI 的配置上位机，详见 [config_tool/README.md](config_tool/README.md)。

```bash
cd config_tool
pip install pyserial
python app.py
# 浏览器打开 http://localhost:8080
```
