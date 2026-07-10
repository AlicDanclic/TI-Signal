# FPGA IP 核库 (Library)

## 概述

本库提供了一套完整的 FPGA IP 核集合，涵盖**通信**、**图像处理**、**测量**、**数学运算**、**存储器**、**接口协议**、**驱动**、**CPU 子系统**及**厂商原语封装**等多个领域。所有模块均使用 Verilog/SystemVerilog 编写，兼容 Vivado、Quartus 等主流 EDA 工具。

---

## 目录结构

```
library/
├── Apply/           # 应用层模块
│   ├── Comm/        #   通信领域（FDE均衡、FEC编解码、CIC/FIR滤波、MDS调制解调、曼彻斯特编码等）
│   ├── ISP/         #   图像信号处理（RAW→RGB、RGB→YCbCr、中值滤波、Sobel边缘检测、腐蚀膨胀等）
│   └── Meas/        #   测量（频率测量、有效值、峰峰值、最大最小值）
├── Basic/           # 基础构建模块
│   ├── Interface/   #   低速接口（IIC、SPI、UART/AXI-Lite UART）
│   ├── Math/        #   数学运算
│   │   ├── FixedPoint/   #     定点数（累加器、加减器、除法器、微分器、量化器、平方根）
│   │   ├── FloatPoint/   #     浮点数（加法、乘法、除法、指数，组合逻辑/时序逻辑两套实现）
│   │   └── Advance/      #     高级运算（复数加减乘、CORDIC、模值、相位旋转、FFT/IFFT、AES、均值、排序）
│   ├── Memory/      #   存储器
│   │   ├── FIFO/    #     异步FIFO、同步FIFO、ECC编解码
│   │   └── SRAM/    #     双口RAM、伪双口RAM、单口RAM、移位寄存器链
│   ├── Source/      #   信号源（任意时钟生成、整数分频器、DDS直接数字频率合成、正弦LUT）
│   └── System/      #   系统（异步复位同步释放）
├── CPU/             # CPU 子系统
│   ├── Bus/         #   总线（仲裁器、优先级编码器）
│   └── Core/        #   内核（ARM Cortex-M0、Cortex-M3 逻辑封装）
├── Driver/          # 外设驱动
│   ├── Auto/        #   自动化（按键消抖、电机驱动）
│   ├── Display/     #   显示（HDMI TMDS 编解码）
│   ├── DSP/         #   数据转换器与射频
│   │   ├── ADC/     #     AD6645、ADS412x、ADS9226、ZGAD250D14
│   │   ├── DAC/     #     DAC3162(LVDS DDR)、DAC9767、PWM-DAC
│   │   ├── PLL/     #     SI549 频率综合器
│   │   └── SDR/     #     AD9361 软件无线电（LVDS接口、SPI配置、初始化序列）
│   └── Sensor/      #   图像传感器
│       ├── capture/ #     RAW/RGB 数据捕获
│       └── config/  #     OV5640、OV7725、MT9V034 寄存器配置
├── Factory/         # 厂商原语封装
│   ├── efinix/      #   Efinix Trion/Titanium 系列原语
│   └── xilinx/      #   Xilinx 系列（时钟管理、SelectIO LVDS）
└── README.md        # 本文件
```

---

## 模块特性

### Apply — 应用层

| 子目录 | 功能 | 关键模块 |
|--------|------|----------|
| Comm/FDE | 频域均衡 | AGC自动增益控制、atan/deinter LUT |
| Comm/FEC | 前向纠错 | 卷积编码器、Viterbi译码器 |
| Comm/Filter | 数字滤波 | CIC抽取滤波器(2/3阶)、FIR滤波器(310阶)、滑动平均滤波器 |
| Comm/MDS | 调制解调 | DDC/DUC变频、AM/FM/PM调制解调、ASK/FSK/BPSK/QPSK数字调制解调 |
| Comm/Trans | 传输 | CRC32校验 |
| Comm/Wire | 线编码 | 曼彻斯特编码、m序列生成 |
| ISP | 图像处理 | RAW8→RGB888→YCbCr444、3x3矩阵、中值/灰度滤波、Sobel边缘检测、腐蚀膨胀 |
| Meas | 测量 | 频率测量、有效值、峰峰值、最大最小值 |

### Basic — 基础模块

| 子目录 | 功能 | 关键模块 |
|--------|------|----------|
| Interface | 低速接口 | IIC主机(支持SCCB)、SPI主机(可配CPOL/CPHA)、UART(AXI-Lite总线接口) |
| Math/FixedPoint | 定点运算 | 4级流水线累加器/加减器、除法器、微分器、8位量化器、开平方根 |
| Math/FloatPoint | 浮点运算 | 组合逻辑版和时序逻辑版加法/乘法/除法/指数，含unpack/pack/normal工具 |
| Math/Advance | 高级运算 | 复数加减乘、CORDIC、模值估计、相位计算、相位旋转、FFT/IFFT、AES、3输入排序 |
| Memory/FIFO | FIFO | 异步FIFO(格雷码同步、ECC、FWFT/Standard)、同步FIFO、ECC Hamming编解码 |
| Memory/SRAM | SRAM | 双口RAM、伪双口RAM、单口RAM、串并转换、可变深度移位寄存器 |
| Source | 信号源 | 任意频率发生器、整数分频器(奇偶分频)、DDS(正弦/三角/锯齿波) |
| System | 系统 | 异步复位同步释放 |

### CPU — CPU 子系统

| 子目录 | 功能 |
|--------|------|
| Bus/arbiter | 总线仲裁器（轮询/优先级仲裁） |
| Core/ARM | Cortex-M0 / Cortex-M3 逻辑封装 |

### Driver — 外设驱动

| 子目录 | 器件 | 说明 |
|--------|------|------|
| Display/HDMI | TMDS | 25MHz/250MHz时钟域，8b/10b编解码 |
| DSP/ADC | AD6645/ADS412x/ADS9226/ZGAD250D14 | 高速ADC接口与配置 |
| DSP/DAC | DAC3162(LVDS DDR)/DAC9767/DACPWM | 高速DAC接口 |
| DSP/PLL | SI549 | 频率综合器IIC配置 |
| DSP/SDR | AD9361 | LVDS接口、SPI配置ROM、初始化序列 |
| Sensor | OV5640/OV7725/MT9V034 | RAW/RGB/YUV配置序列、数据捕获 |

### Factory — 厂商原语

| 厂商 | 内容 |
|------|------|
| Efinix | 加法器、LUT、DPRAM、RAM、DSP、乘法器、SRL8、FF、全局缓冲等 |
| Xilinx | 时钟管理、SelectIO LVDS、Block Design模板(Cortex-M3/MicroBlaze/PCIe/Zynq) |

---

## 设计风格

### 接口规范

- **@Flow 标注**：标记模块间数据流方向（@Flow Input valid、@Flow Output valid等）
- **流水线握手**：统一使用 ivalid/ovalid 信号对进行数据流控制
- **复位策略**：同步或异步复位，高电平有效

### 参数化设计

所有模块通过 Verilog parameter 实现可配置：

```verilog
module divider #(
    parameter DIVIDEND = 32,  // 被除数位宽
    parameter DIVISOR  = 24   // 除数位宽
);
```

### 注释规范

所有模块注释统一使用**中文**书写，包括：
- **文件头部** — 模块功能概述（`// 模块名` + `// 功能说明`）
- **端口/参数** — 每个端口和参数的行内说明（`// 时钟输入`、`// 数据位宽`等）
- **内部逻辑** — 关键状态机、数据通路、算法步骤的行内标注

### 时序图标注

部分模块使用 `/* @wavedrom {signal: [...]} */` 格式嵌入时序图，可在 Wavedrom 在线工具中渲染查看。

---

## 使用说明

### 集成方式

直接将所需 .v/.sv 文件添加至工程：

```tcl
# Vivado
read_verilog [glob library/Basic/Math/FixedPoint/*.v]
```

### 依赖关系

```
DDS  → accuml (+ Sin LUT)
CORDIC → 自包含
FFT → ftwiddle + ftrans + BF_stage
async_fifo → DPRAM + ecc_encode/ecc_decode（可选）
Video_Image_Processor → RAW8_RGB888 + RGB888_YCbCr444 + Median_Filter + Sobel + shiftRAM
```

### EDA 工具兼容性

- **Vivado**：DPRAM/SDPRAM 可直接综合为 BRAM
- **Quartus**：Verilog-2001 语法
- **Synplify / DC**：支持 signed 类型和 generate 结构

---

## 相关资源

- Wavedrom 时序图渲染：https://wavedrom.com/
- DSP Guru 模值估计算法：http://dspguru.com/dsp/tricks/magnitude-estimator
- CORDIC 算法：https://en.wikipedia.org/wiki/CORDIC
