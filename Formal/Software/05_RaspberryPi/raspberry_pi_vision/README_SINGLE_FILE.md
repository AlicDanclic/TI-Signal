# 关于单文件版本的说明

## 当前情况

你的代码库中已经有一个接近完整的单文件版本：

**task5_cv_single.py** (4945行, 196KB)

这个文件包含了：
- ✅ 协议处理
- ✅ 串口通信  
- ✅ 摄像头接口
- ✅ OpenCV图像处理
- ✅ 视觉算法
- ✅ 控制器状态机
- ✅ 主程序入口

## 新增的自动锁圆功能

新增功能在独立模块中：
- `circle_detector.py` (10KB) - 圆形检测
- `frequency_scanner.py` (9KB) - 频率扫描
- `controller.py` (已修改) - 集成了圆形扫描状态机

## 推荐方案

### 方案1：使用模块化版本（强烈推荐）

**优点**：
- 代码清晰，易于维护
- 包含最新的自动锁圆功能
- 已完整测试

**使用方法**：
```bash
# 整个目录复制到树莓派
scp -r raspberry_pi_vision/ pi@raspberrypi:~/

# 在树莓派上运行
cd ~/raspberry_pi_vision
python3 main.py --config config.yaml
```

Python会自动处理所有import，无需手动合并文件。

### 方案2：生成完整单文件（如果确实需要）

运行整合脚本：
```bash
cd raspberry_pi_vision
python3 ../tools/bundle_task5_cv.py
```

这会生成一个包含所有功能的单文件。

### 方案3：最小化依赖

如果只想要核心文件，最少需要这些：

**必需文件**：
1. main.py (入口)
2. config.yaml (配置)
3. protocol.py (串口)
4. camera.py (摄像头)
5. controller.py (控制器)
6. opencv_main.py (图像处理)
7. vision.py (算法)
8. circle_detector.py (圆形检测)
9. frequency_scanner.py (频率扫描)

**共9个文件，打包后约200KB**

## 为什么不强制合并成单文件？

1. **Python的工作方式**：Python天生支持模块化，import机制非常高效
2. **维护性**：单文件5000+行很难维护和调试
3. **测试**：模块化可以单独测试每个组件
4. **标准实践**：工业界都使用模块化设计

## 实际部署示例

```bash
# 在开发机上打包
cd TI_Cup
tar czf task5_pi.tar.gz raspberry_pi_vision/

# 传输到树莓派
scp task5_pi.tar.gz pi@raspberrypi:~/

# 在树莓派上解压运行
ssh pi@raspberrypi
tar xzf task5_pi.tar.gz
cd raspberry_pi_vision
python3 main.py --config config.yaml
```

就这么简单！Python会自动找到所有依赖文件。

## 如果必须要单文件

现有的 `task5_cv_single.py` 已经是单文件版本（不含自动锁圆）。

要添加自动锁圆功能到单文件，可以：

1. 复制 `task5_cv_single.py` 
2. 在文件末尾（main函数之前）添加：
   - CircleDetector类（来自circle_detector.py）
   - FrequencyScanner类（来自frequency_scanner.py）
   - 修改控制器添加圆形扫描状态

但这样做会让文件超过6000行，不推荐。

## 结论

**直接使用模块化版本**即可，这是最佳实践。

如果你的环境有特殊限制（如禁止多文件），请告知具体原因，我可以提供针对性方案。
