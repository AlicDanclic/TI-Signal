{
  "version": "1.0",
  "project": {
    "id": "918c956f-9c71-46e7-a332-71151061c9c7",
    "name": "2026江苏省电赛信号类备赛与正赛",
    "description": "2026年江苏省电赛信号类备赛需三人分工协作（硬件、算法、系统集成），重点储备信号产生、采集、调理与处理等核心模块，熟练STM32/FPGA平台，强化模拟电路基本功与报告撰写训练。正赛于7月29日至8月1日四天三夜内完成方案设计、硬件搭建、编程调试及设计报告，随后8月3日至6日进行现场实测。最终成绩依据功能实现（60%）、性能指标（30%）和设计报告（10%）综合评定。",
    "template": "default",
    "color": "#4f46e5",
    "created_at": "2026-07-09T08:22:27.258Z",
    "updated_at": "2026-07-21T14:48:42.239Z",
    "archived": false,
    "start_date": "2026-07-09",
    "end_date": "2026-08-10",
    "sync_enabled": true,
    "storage": {
      "type": "local",
      "path": "D:\\TI_Signals"
    }
  },
  "readme_file": "README.md",
  "template": {
    "dirs": [
      "Bitmap",
      "Hardware",
      "Software",
      "Reference",
      "DataSheet"
    ],
    "files": [
      "Readme.md",
      ".gitignore",
      "README.md"
    ],
    "file_contents": {
      "Readme.md": "<div align=\"center\"><h1>2026江苏省电赛信号类备赛与正赛</h1></div>",
      ".gitignore": "node_modules/\n.env\n.DS_Store\n",
      "README.md": "# 2026江苏省电赛信号类备赛与正赛\n\n2026年江苏省电赛信号类备赛需三人分工协作（硬件、算法、系统集成），重点储备信号产生、采集、调理与处理等核心模块，熟练STM32/FPGA平台，强化模拟电路基本功与报告撰写训练。正赛于7月29日至8月1日四天三夜内完成方案设计、硬件搭建、编程调试及设计报告，随后8月3日至6日进行现场实测。最终成绩依据功能实现（60%）、性能指标（30%）和设计报告（10%）综合评定。\n"
    }
  },
  "tasks": [
    {
      "id": "41865378-f2cf-4013-bd67-b9f344e8ac6e",
      "title": "Readme文档审阅",
      "description": "",
      "status": "todo",
      "priority": "low",
      "tags": [],
      "due_date": "2026-07-28",
      "due_time": null,
      "start_offset": null,
      "dependencies": [],
      "subtasks": [],
      "comments": [],
      "tracked_start": null,
      "reminder": null,
      "created_at": "2026-07-09T08:25:32.899Z",
      "updated_at": "2026-07-09T08:25:47.575Z"
    },
    {
      "id": "440cc594-21a1-42d8-a488-fd144abeb346",
      "title": "电赛正式比赛",
      "description": "",
      "status": "todo",
      "priority": "high",
      "tags": [],
      "due_date": "2026-08-01",
      "due_time": null,
      "start_offset": null,
      "dependencies": [
        {
          "taskId": "042d0ec1-ea75-4a09-801f-3eaf12658f33",
          "dayOffset": 1
        },
        {
          "taskId": "83ad93ef-7713-408b-bddf-d150682c1be5",
          "dayOffset": 1
        },
        {
          "taskId": "082069c6-cd76-4190-92d7-9f7addce7041",
          "dayOffset": 1
        }
      ],
      "subtasks": [],
      "comments": [],
      "tracked_start": null,
      "reminder": null,
      "created_at": "2026-07-09T08:26:20.313Z",
      "updated_at": "2026-07-09T08:47:37.366Z"
    },
    {
      "id": "8fac6819-3abc-460b-a937-207982701730",
      "title": "电赛现场测试",
      "description": "",
      "status": "todo",
      "priority": "high",
      "tags": [],
      "due_date": "2026-08-06",
      "due_time": null,
      "start_offset": null,
      "dependencies": [
        {
          "taskId": "440cc594-21a1-42d8-a488-fd144abeb346",
          "dayOffset": 1
        }
      ],
      "subtasks": [],
      "comments": [],
      "tracked_start": null,
      "reminder": null,
      "created_at": "2026-07-09T08:26:27.849Z",
      "updated_at": "2026-07-09T08:26:55.453Z"
    },
    {
      "id": "042d0ec1-ea75-4a09-801f-3eaf12658f33",
      "title": "软件代码",
      "description": "以下是对原始任务清单的润色与扩展，补充了每个模块的背景目标、实施要点及验收标准，保持专业且清晰。\n\n---\n\n- **SPI（串行外设接口）**  \n  实现 SPI 主从模式通信，支持标准 4 线制（SCLK、MOSI、MISO、CS），传输速率可配置（如 1~10 MHz）。需完成驱动层封装，提供寄存器级读写接口，并验证与外部设备（如 Flash、传感器）的数据交换正确性。\n\n- **I²C（IIC，内部集成电路）**  \n  支持标准模式（100 kHz）和快速模式（400 kHz），具备多主机仲裁与时钟拉伸处理。驱动需提供起始/停止条件、ACK/NACK 检测、字节读写函数，并经过与 EEPROM、温度传感器等从设备的通信测试。\n\n- **UART（通用异步收发器）上层处理**  \n  在硬件 UART 驱动基础上，构建上层数据收发模块，包括环形缓冲区管理、数据帧解析（如固定长度/分隔符模式）、以及中断或 DMA 方式的高效处理。需支持波特率 9600~115200 bps，并通过回环测试验证稳定性。\n\n- **ADC（模数转换器）**  \n  配置片内 ADC，支持单次/连续转换模式，分辨率 12 位（或更高），参考电压可调。实现多通道扫描，采样率满足应用需求（如 1 kSPS）。需提供校准机制，并验证转换误差在 ±2 LSB 以内。\n\n- **DAC（数模转换器）**  \n  实现 DAC 输出功能，支持 8 位或 12 位分辨率，输出电压范围可配置（如 0~Vref）。需提供单值写入与波形生成（如正弦波、三角波）接口，通过示波器验证输出波形正确性。\n\n- **数学运算模块**  \n  封装常用数学运算函数，包括四则运算、平方根、三角函数、指数/对数等，基于查表或 CORDIC 算法实现，兼顾速度与精度。测试时确保运算误差小于 1%，并评估在目标 MCU 上的执行时间。\n\n- **FIR 滤波器**  \n  实现线性相位 FIR 滤波器，支持滤波器阶数配置（如 16~128 阶），窗函数可选（汉宁、汉明、布莱克曼等）。提供系数生成工具及滤波调用接口，用已知信号（如叠加噪声的正弦波）验证滤波后的信噪比提升。\n\n- **FFT 频谱分析**  \n  实现基 2 时间抽取 FFT，支持 64~1024 点变换，输入数据为实序列（可扩展为复数）。需提供窗函数预处理（如汉宁窗），并计算幅度谱。通过测试信号（如 1 kHz + 3 kHz 混合）验证频率分辨率与峰值检测准确性。\n\n- **寻峰算法**  \n  在 FFT 频谱输出或原始数据中寻找显著峰值，支持阈值设定与局部最大值搜索。需处理噪声引起的误判，返回峰值频率与幅值。测试用例包括单峰、多峰、接近频率的峰分离。\n\n- **FIFO（先进先出缓冲区）**  \n  实现环形 FIFO，支持任意字节大小（如 256/512/1024 字节），提供初始化、写入、读取、满/空状态判断函数。需保证在多任务或中断环境下数据完整性，支持原子操作（临界区保护）。测试时验证写入/读取一致性及溢出处理。\n\n- **按键/编码器控制**  \n  实现按键消抖（软件去抖动，时间 20~50 ms），支持单击、长按、双击识别；编码器支持正交信号解码，识别旋转方向与步数。需提供状态回调机制，用于触发菜单导航或参数调整。\n\n- **显示器驱动**  \n  根据所选显示器类型（如 OLED、LCD、段式液晶），实现初始化、清屏、字符/图形绘制功能。支持单色或彩色显示，分辨率匹配硬件，通信接口（I²C/SPI/并行）可配置。需验证显示刷新率满足人眼舒适度（≥30 fps），并提供可移植的绘图库接口。",
      "status": "done",
      "priority": "medium",
      "tags": [],
      "due_date": "2026-07-28",
      "due_time": null,
      "start_offset": null,
      "dependencies": [],
      "subtasks": [
        {
          "id": "4cbcc2b2-b2ef-41df-9e46-25a9ff68bd9f",
          "title": "通信接口驱动（SPI与I2C）",
          "done": false
        },
        {
          "id": "86709441-ff5a-40ff-baa8-e6cf75a6f1fd",
          "title": "UART驱动及上层处理",
          "done": false
        },
        {
          "id": "3d0767e2-a9fe-4eab-993a-2efe089a9bb5",
          "title": "模拟外设驱动（ADC与DAC）",
          "done": false
        },
        {
          "id": "260a28f3-bc8a-457d-b95f-9745727337af",
          "title": "数学运算模块",
          "done": false
        },
        {
          "id": "d091ac34-4dcf-4daf-9a20-2dc6c84878f5",
          "title": "FIR/FFT/寻峰信号处理",
          "done": false
        },
        {
          "id": "2b171885-c3ca-4e15-9efd-21e690c0d244",
          "title": "基础工具模块（FIFO与按键）",
          "done": false
        },
        {
          "id": "3366171f-df76-4b83-bade-783e80f05c28",
          "title": "显示器驱动",
          "done": false
        }
      ],
      "comments": [],
      "tracked_start": null,
      "reminder": null,
      "created_at": "2026-07-09T08:40:40.568Z",
      "updated_at": "2026-07-10T12:39:49.586Z"
    },
    {
      "id": "082069c6-cd76-4190-92d7-9f7addce7041",
      "title": "硬件仿真与准备",
      "description": "",
      "status": "done",
      "priority": "medium",
      "tags": [],
      "due_date": "2026-07-28",
      "due_time": null,
      "start_offset": null,
      "dependencies": [],
      "subtasks": [],
      "comments": [],
      "tracked_start": "2026-07-11T02:45:12.619Z",
      "reminder": null,
      "created_at": "2026-07-09T08:47:02.610Z",
      "updated_at": "2026-07-21T05:27:48.012Z"
    },
    {
      "id": "83ad93ef-7713-408b-bddf-d150682c1be5",
      "title": "Simulink模拟代码生成",
      "description": "",
      "status": "todo",
      "priority": "medium",
      "tags": [],
      "due_date": "2026-07-28",
      "due_time": null,
      "start_offset": null,
      "dependencies": [],
      "subtasks": [],
      "comments": [],
      "tracked_start": null,
      "reminder": null,
      "created_at": "2026-07-09T08:47:12.986Z",
      "updated_at": "2026-07-09T08:47:19.422Z"
    },
    {
      "id": "c5e51a61-435d-4723-bcfa-213e90546951",
      "title": "文档整理上传",
      "description": "",
      "status": "todo",
      "priority": "medium",
      "tags": [],
      "due_date": "2026-08-10",
      "due_time": null,
      "start_offset": null,
      "dependencies": [
        {
          "taskId": "41865378-f2cf-4013-bd67-b9f344e8ac6e",
          "dayOffset": 1
        },
        {
          "taskId": "8fac6819-3abc-460b-a937-207982701730",
          "dayOffset": 1
        }
      ],
      "subtasks": [],
      "comments": [],
      "tracked_start": null,
      "reminder": null,
      "created_at": "2026-07-09T08:47:49.992Z",
      "updated_at": "2026-07-09T08:48:13.245Z"
    }
  ],
  "tags": [],
  "changelog": [
    {
      "version": "1.0.0",
      "date": "2026-07-09",
      "info": "初始版本"
    },
    {
      "version": "1.0.1",
      "date": "2026-07-09",
      "info": "创建任务「Readme文档审阅」"
    },
    {
      "version": "1.0.2",
      "date": "2026-07-09",
      "info": "任务「Readme文档审阅」优先级改为高"
    },
    {
      "version": "1.0.3",
      "date": "2026-07-09",
      "info": "任务「Readme文档审阅」优先级改为低"
    },
    {
      "version": "1.0.4",
      "date": "2026-07-09",
      "info": "任务「Readme文档审阅」优先级改为中"
    },
    {
      "version": "1.0.5",
      "date": "2026-07-09",
      "info": "任务「Readme文档审阅」优先级改为高"
    },
    {
      "version": "1.0.6",
      "date": "2026-07-09",
      "info": "任务「Readme文档审阅」优先级改为低"
    },
    {
      "version": "1.0.7",
      "date": "2026-07-09",
      "info": "创建任务「电赛正式比赛」"
    },
    {
      "version": "1.0.8",
      "date": "2026-07-09",
      "info": "创建任务「电赛现场测试」"
    },
    {
      "version": "1.0.9",
      "date": "2026-07-09",
      "info": "任务「电赛现场测试」优先级改为高"
    },
    {
      "version": "1.0.10",
      "date": "2026-07-09",
      "info": "任务「电赛正式比赛」优先级改为高"
    },
    {
      "version": "1.0.11",
      "date": "2026-07-09",
      "info": "创建任务「软件代码」"
    },
    {
      "version": "1.0.12",
      "date": "2026-07-09",
      "info": "创建任务「硬件仿真与准备」"
    },
    {
      "version": "1.0.13",
      "date": "2026-07-09",
      "info": "创建任务「Simulink模拟代码生成」"
    },
    {
      "version": "1.0.14",
      "date": "2026-07-09",
      "info": "创建任务「文档整理上传」"
    },
    {
      "version": "1.0.15",
      "date": "2026-07-10",
      "info": "任务「软件代码」状态变更为已完成"
    },
    {
      "version": "1.0.16",
      "date": "2026-07-21",
      "info": "任务「硬件仿真与准备」状态变更为已完成"
    }
  ],
  "milestones": []
}