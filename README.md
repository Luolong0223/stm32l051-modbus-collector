# STM32L051 Modbus RTU 远程数据采集上报系统 v2.3

基于 STM32L051K8 的 Modbus RTU 多设备数据采集系统。

## 功能特性

- **PA15 模式选择**: HIGH=配置模式(从站), LOW=轮询上报模式(主站), 硬件切换无抖动
- **UART2 (RS485)**: Modbus RTU 主站轮询 + 从站配置
- **UART1**: 数据上报（TEXT / JSON / HEX 三种格式）
- **内部 EEPROM (2KB)**: 配置断电保存
- **远程配置**: 通过 Modbus 寄存器读写所有参数
- **设备/数据点命名**: 每台设备、每个数据点可独立配置 20 字符名称
- **设备名称**: 可配置的设备本体名称，随上报数据一起发送
- **电压采集**: PA0 ADC 电压检测，实时上报供电电压
- **每轮必报**: 每轮轮询结束必定上报一次，无设备响应时 online=0
- **低功耗模式**: Stop 模式 ~3μA，RTC WakeUp 定时唤醒，休眠间隔可配置 (0~65535秒)

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

## v2.3 新增功能

| 项目 | 说明 |
|------|------|
| **设备名称** | 新增 `device_name` 字段 (20字符)，通过 Modbus 寄存器 0x000B~0x0014 可读写，断电保存 |
| **电压采集** | PA0 ADC 电压检测，寄存器 0x0015 只读，上报时转换为 mV (Vref=3.3V) |
| **每轮必报** | 每轮轮询结束必定上报一次，无设备响应时 `online=0`，不再因无数据跳过上报 |
| **上报格式增强** | TEXT/JSON/HEX 三种格式均包含设备名称和电压信息 |
| **ADC 校准** | 启动时自动执行 ADC 校准 (`HAL_ADCEx_Calibration_Start`) |

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

### ADC（电压采集）
- PA0: ADC1_IN0，Single Conversion 模式
- 12 位分辨率，Vref = 3.3V
- 电压换算：`电压(mV) = ADC_RAW × 3300 / 4096`

### RTC（低功耗唤醒）
- LSE: 32.768kHz 晶振（需外部连接）
- RTC WakeUp: ck_spre (1Hz)，16 位计数器
- 唤醒间隔 = sleep_interval_sec 秒

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

### 地址空间总览

| 地址范围 | 用途 | 说明 |
|----------|------|------|
| 0x0000 ~ 0x001F | 系统寄存器 | UART 参数、本机地址、上报格式、设备名称、电压、运行时间、预留 |
| 0x0100 ~ 0x010F | 从机 0 基本配置 | 地址、使能、数据点数、轮询周期、名称 |
| 0x0110 ~ 0x018F | 从机 0 数据点 0~7 | 寄存器地址、数据类型、字节序、名称 |
| 0x0300 ~ 0x030F | 从机 1 基本配置 | 同上 |
| 0x0310 ~ 0x038F | 从机 1 数据点 0~7 | 同上 |
| 0x0500 ~ 0x050F | 从机 2 基本配置 | 同上 |
| 0x0510 ~ 0x058F | 从机 2 数据点 0~7 | 同上 |
| 0x0700 ~ 0x070F | 从机 3 基本配置 | 同上 |
| 0x0710 ~ 0x078F | 从机 3 数据点 0~7 | 同上 |
| 0x0900 ~ 0x090F | 从机 4 基本配置 | 同上 |
| 0x0910 ~ 0x098F | 从机 4 数据点 0~7 | 同上 |

### 系统寄存器（0x0000 ~ 0x001F）

| 地址 | 名称 | 读写 | 说明 |
|------|------|:----:|------|
| 0x0000 | uart2_baudrate_lo | RW | UART2 波特率低 16 位 |
| 0x0001 | uart2_baudrate_hi | RW | 高 16 位，**写入延迟重配** (主站 IDLE 时执行) |
| 0x0002 | uart2_parity | RW | 0=无 1=奇 2=偶 |
| 0x0003 | uart2_stopbits | RW | 1 或 2 |
| 0x0004 | rs485_de_delay | RW | DE 延时 (us) |
| 0x0005 | rs485_re_delay | RW | RE 延时 (us) |
| 0x0006 | local_mb_addr | RW | 本机 Modbus 地址 (1~247) |
| 0x0007 | slave_count | RW | 采集从机数 (1~5) |
| 0x0008 | report_format | RW | 0=TEXT 1=JSON 2=HEX |
| 0x0009 | uart1_baudrate_lo | RW | UART1 波特率低 16 位 |
| 0x000A | uart1_baudrate_hi | RW | 高 16 位，**写入延迟重配** |
| 0x000B | device_name[0] | RW | 设备名称 字节 0~1（大端：高字节在前） |
| 0x000C | device_name[1] | RW | 设备名称 字节 2~3 |
| 0x000D | device_name[2] | RW | 设备名称 字节 4~5 |
| 0x000E | device_name[3] | RW | 设备名称 字节 6~7 |
| 0x000F | device_name[4] | RW | 设备名称 字节 8~9 |
| 0x0010 | device_name[5] | RW | 设备名称 字节 10~11 |
| 0x0011 | device_name[6] | RW | 设备名称 字节 12~13 |
| 0x0012 | device_name[7] | RW | 设备名称 字节 14~15 |
| 0x0013 | device_name[8] | RW | 设备名称 字节 16~17 |
| 0x0014 | device_name[9] | RW | 设备名称 字节 18~19 |
| 0x0015 | voltage_adc_raw | **R** | **PA0 电压 ADC 原始值** (0~4095, 只读) |
| 0x0016 | uptime_lo | **R** | 设备运行时间低 16 位 (秒, 只读, 溢出周期 ~49710 天) |
| 0x0017 | uptime_hi | **R** | 设备运行时间高 16 位 (秒, 只读) |
| 0x0018 | sleep_interval_sec | RW | **低功耗休眠间隔** (秒, 0=禁用, 最大65535) |
| 0x0019~0x001F | reserved[7] | **R** | **预留** (只读, 返回 0, 未来扩展用) |

> **名称编码**：每个寄存器存 2 字节，大端序（高字节在前）。共 10 个寄存器 = 20 字节。支持 ASCII/GBK/UTF-8，写入时自动过滤不可打印字符。

### 从机 N 配置（基地址 = 0x0100 + N × 0x0200，N = 0~4）

| 偏移 | 地址示例(N=0) | 名称 | 读写 | 说明 |
|------|:----:|------|:----:|------|
| +0x00 | 0x0100 | slave_addr | RW | 从机 Modbus 地址 (1~247) |
| +0x01 | 0x0101 | enabled | RW | 1=启用 0=停用 |
| +0x02 | 0x0102 | data_point_count | RW | 数据点数量 (0~8) |
| +0x03 | 0x0103 | poll_period_lo | RW | 轮询周期低 16 位 (ms) |
| +0x04 | 0x0104 | poll_period_hi | RW | 轮询周期高 16 位 |
| +0x05 | 0x0105 | name[0] | RW | 设备名称 字节 0~1 |
| +0x06 | 0x0106 | name[1] | RW | 设备名称 字节 2~3 |
| +0x07 | 0x0107 | name[2] | RW | 设备名称 字节 4~5 |
| +0x08 | 0x0108 | name[3] | RW | 设备名称 字节 6~7 |
| +0x09 | 0x0109 | name[4] | RW | 设备名称 字节 8~9 |
| +0x0A | 0x010A | name[5] | RW | 设备名称 字节 10~11 |
| +0x0B | 0x010B | name[6] | RW | 设备名称 字节 12~13 |
| +0x0C | 0x010C | name[7] | RW | 设备名称 字节 14~15 |
| +0x0D | 0x010D | name[8] | RW | 设备名称 字节 16~17 |
| +0x0E | 0x010E | name[9] | RW | 设备名称 字节 18~19 |

> **从机基地址**：从机 0=0x0100，从机 1=0x0300，从机 2=0x0500，从机 3=0x0700，从机 4=0x0900

### 从机 N 数据点 M（基地址 = 从机基 + 0x0010 + M × 0x0010，M = 0~7）

| 偏移 | 地址示例(N=0,M=0) | 名称 | 读写 | 说明 |
|------|:----:|------|:----:|------|
| +0x00 | 0x0110 | reg_addr | RW | 从机寄存器起始地址 |
| +0x01 | 0x0111 | data_type | RW | 0=U16 1=I16 2=U32 3=I32 4=Float |
| +0x02 | 0x0112 | byte_order | RW | 0=ABCD 1=BADC 2=CDAB 3=DCBA |
| +0x03 | 0x0113 | name[0] | RW | 数据点名称 字节 0~1 |
| +0x04 | 0x0114 | name[1] | RW | 数据点名称 字节 2~3 |
| +0x05 | 0x0115 | name[2] | RW | 数据点名称 字节 4~5 |
| +0x06 | 0x0116 | name[3] | RW | 数据点名称 字节 6~7 |
| +0x07 | 0x0117 | name[4] | RW | 数据点名称 字节 8~9 |
| +0x08 | 0x0118 | name[5] | RW | 数据点名称 字节 10~11 |
| +0x09 | 0x0119 | name[6] | RW | 数据点名称 字节 12~13 |
| +0x0A | 0x011A | name[7] | RW | 数据点名称 字节 14~15 |
| +0x0B | 0x011B | name[8] | RW | 数据点名称 字节 16~17 |
| +0x0C | 0x011C | name[9] | RW | 数据点名称 字节 18~19 |

> **数据点基地址**：数据点 0 = 从机基+0x0010，数据点 1 = 从机基+0x0020，...，数据点 7 = 从机基+0x0080

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
3. **RAM 用量**: sizeof(SystemCfg_t) ≈ 1380 字节 (含 device_name)，STM32L051 有 8KB RAM，足够。
4. **模式选择**: PA15 硬件引脚控制。HIGH=配置模式(从站监听), LOW=轮询上报模式(主站采集+上报)。仅在电平变化时切换一次，无软件抖动。PA15 默认下拉，上电即为轮询上报模式。
5. **编译要求**: 需要 C11 支持（`_Static_assert`），GCC/ARMCC 默认支持。`docs/stm32l0xx_it_custom.c.txt` **不要参与编译**，仅作为中断集成参考。
6. **UART 热更新**: 通过 Modbus 写入 UART1/UART2 参数后，设置延迟标志——主站 IDLE 时自动执行重配，避免中断配置模式下的通信。
7. **主站重试**: 单个数据点超时后自动重试 2 次（可通过 MB_MASTER_RETRY_MAX 调整）。重试耗尽后立即尝试该从机的下一个数据点，不等待下一轮轮询周期。
8. **广播**: 收到地址 0 的广播请求时会正常处理配置变更，但不会发送响应（符合 Modbus RTU 规范）。
9. **从站功能码**: 支持 0x03 (读保持寄存器) 和 0x04 (读输入寄存器)，两者响应格式相同。
10. **电压采集**: PA0 ADC 每次主循环采样一次 (约几十 μs)，原始值存入 `g_adc_voltage_raw`，通过 Modbus 寄存器 0x0015 可读取。上报时自动转换为 mV。如 Vref 非 3.3V，需修改 `report_manager.c` 和 `modbus_driver.c` 中的换算公式。
11. **每轮必报**: 每轮轮询结束必定上报一次数据。即使所有从机离线，也会上报（`online=0`）。上报间隔取所有启用从机中最短的轮询周期。
12. **设备名称**: 通过 Modbus 寄存器 0x000B~0x0014 可读写，支持 0x06 单写和 0x10 批量写，断电自动保存至 EEPROM。默认值 `"Collector-1"`。
13. **设备运行时间**: 寄存器 0x0016~0x0017 只读，单位秒，基于 `HAL_GetTick()`。32 位无符号整数，溢出周期约 136 年。
14. **预留寄存器**: 0x0019~0x001F 共 7 个寄存器，当前只读返回 0，预留给未来功能扩展。
15. **低功耗模式**: 寄存器 0x0018 配置休眠间隔（秒），设为 0 禁用。启用后，每轮轮询+上报完成自动进入 Stop 模式（~3μA），由 RTC WakeUp 定时唤醒。唤醒后自动恢复时钟和外设，继续下一轮轮询。**注意**：配置模式（PA15=HIGH）下不进入低功耗。需要 CubeMX 配置 LSE (32.768kHz) 晶振，并实现 `SystemClock_Config()` 函数供唤醒后恢复时钟。
16. **CubeMX 集成**: 需要在 CubeMX 中配置 RTC 时钟源为 LSE，并在 `SystemClock_Config()` 中确保 LSE 和 RTC 时钟使能。唤醒后调用 `SystemClock_Config()` 恢复系统时钟。

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
