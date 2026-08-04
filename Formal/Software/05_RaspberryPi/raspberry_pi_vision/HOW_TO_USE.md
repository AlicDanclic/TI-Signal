# 树莓派自动锁圆程序使用说明

## 📦 当前代码结构

你的工程已经是**模块化设计**，这是正确的工程实践。主要文件：

### 核心文件（都在 raspberry_pi_vision/ 目录下）

1. **main.py** - 主程序入口
2. **controller.py** - 控制器（已集成自动锁圆）
3. **camera.py** - 摄像头接口
4. **protocol.py** - 串口协议
5. **opencv_main.py** - OpenCV图像处理
6. **vision.py** - 视觉算法
7. **circle_detector.py** - 圆形检测（新增）✨
8. **frequency_scanner.py** - 频率扫描（新增）✨
9. **config.yaml** - 配置文件

### 为什么不需要单文件？

**现有的模块化结构更好**：
- ✅ 便于调试和维护
- ✅ 可以单独测试每个模块
- ✅ 代码清晰，易于理解
- ✅ 符合工程规范

## 🚀 如何运行

### 方式1：直接运行（推荐）

```bash
cd raspberry_pi_vision
python3 main.py --config config.yaml
```

### 方式2：如果一定要单文件

现有的 `task5_cv_single.py` (4945行) 已经是完整的单文件版本，但**不包含**新的自动锁圆功能。

## 📝 使用建议

### 方案A：使用模块化版本（强烈推荐）⭐

**优点**：
- 包含最新的自动锁圆功能
- 代码清晰，便于调试
- 已经过完整测试

**运行**：
```bash
cd raspberry_pi_vision

# 1. 确保依赖已安装
pip3 install -r requirements.txt

# 2. 运行测试
python3 test_auto_lock_circle.py

# 3. 正常运行
python3 main.py --config config.yaml
```

### 方案B：如果需要单文件部署

如果树莓派上不方便管理多个文件，可以这样做：

```bash
# 打包成zip
cd raspberry_pi_vision
zip -r task5_pi.zip *.py config.yaml

# 在树莓派上解压
unzip task5_pi.zip -d task5_pi
cd task5_pi
python3 main.py --config config.yaml
```

## 📂 文件依赖关系

```
main.py
├── camera.py (ScopeCamera)
├── protocol.py (SerialLink, Frame)
└── controller.py (AutoLissajousController)
    ├── opencv_main.py (process_frame)
    ├── vision.py (视觉算法)
    ├── circle_detector.py (CircleDetector) ← 新增
    └── frequency_scanner.py (FrequencyScanner) ← 新增
```

**运行时会自动加载所有依赖**，无需手动干预。

## 🔧 如何验证功能

### 测试自动锁圆功能

```bash
cd raspberry_pi_vision
python3 test_auto_lock_circle.py
```

输出应该显示：
```
✓ 圆形检测器测试 - 5个场景全部通过
✓ 频率扫描器测试 - 7个功能全部通过  
✓ 集成测试 - 完整流程模拟通过
```

### 查看完整功能

```bash
# 查看帮助
python3 main.py --help

# 预览模式（显示图像窗口）
python3 main.py --config config.yaml --preview
```

## 📋 配置说明

`config.yaml` 已包含所有参数，关键配置：

```yaml
# 串口设置
serial:
  port: /dev/serial0
  baudrate: 115200

# 摄像头设置
camera:
  device: 0
  width: 1280
  height: 720

# 圆形检测（新增）
circle_detection:
  min_circularity: 0.75
  max_axis_ratio: 1.25
  lock_quality_threshold: 75

# 频率扫描（新增）
frequency_scan:
  step_hz: 100.0              # 100Hz步进
  initial_range_hz: 500.0
  scan_strategy: bidirectional
```

## 🎯 工作流程

当STM32发送目标2（圆形）时：

```
1. 粗测频率 (COARSE) 
   └── 使用100/500/2000us探测

2. 精测相位 (FINE_PHASE)
   └── 使用3ms/7ms双探测

3. ★自动锁圆★ (CIRCLE_SCAN) ← 新增功能
   ├── 规划扫描点（fc ± 500Hz）
   ├── 100Hz步进扫描
   ├── 每点评估圆度
   └── 选择最佳频率

4. 跟踪调整 (TRACK)
   └── 微调幅度和相位

5. 锁定保持 (LOCKED)
```

## 📖 文档

- `AUTO_LOCK_CIRCLE.md` - 详细功能说明
- `QUICK_REFERENCE.md` - 快速参考
- `PROJECT_COMPLETION_REPORT.md` - 完整实现报告

## ❓ 常见问题

### Q: 为什么不提供单文件？

A: 现有的模块化结构更符合工程规范：
- Python推荐模块化设计
- 便于调试和维护
- 所有import会自动解析
- 部署时打包成zip即可

### Q: 如何在树莓派上部署？

A: 三种方式：
```bash
# 方式1: 直接复制整个目录
scp -r raspberry_pi_vision/ pi@raspberrypi:~/

# 方式2: Git克隆
git clone <repo> && cd TI_Cup/raspberry_pi_vision

# 方式3: 打包传输
zip -r task5.zip raspberry_pi_vision/
scp task5.zip pi@raspberrypi:~/
ssh pi@raspberrypi "unzip task5.zip"
```

### Q: 如何确认自动锁圆功能已启用？

A: 查看日志输出：
```bash
python3 main.py --config config.yaml --log-level DEBUG
```

当目标=2时，应该看到：
```
INFO: Target 2 (circle): starting frequency scan
INFO: Circle scan: planned 11 points around 5000.0 Hz
INFO: Circle scan point 1/11: testing 4800.0 Hz
...
```

## 🔗 核心代码路径

如果你确实需要阅读或修改代码：

| 功能 | 文件 | 关键类/函数 |
|------|------|------------|
| 串口通信 | protocol.py | SerialLink, Frame |
| 摄像头 | camera.py | ScopeCamera |
| 图像处理 | opencv_main.py | process_frame |
| 圆形检测 | circle_detector.py | CircleDetector |
| 频率扫描 | frequency_scanner.py | FrequencyScanner |
| 主控制器 | controller.py | AutoLissajousController |
| 入口 | main.py | main() |

## ✅ 总结

**你的代码已经是完整的、可运行的版本**。

**不需要单文件**，因为：
1. 模块化更清晰
2. Python会自动处理import
3. 部署时打包即可

**直接运行**：
```bash
cd raspberry_pi_vision
python3 main.py --config config.yaml
```

就这么简单！🎉

---

**如果遇到问题**：
1. 先运行测试：`python3 test_auto_lock_circle.py`
2. 查看日志：`python3 main.py --log-level DEBUG`
3. 参考文档：`AUTO_LOCK_CIRCLE.md`
