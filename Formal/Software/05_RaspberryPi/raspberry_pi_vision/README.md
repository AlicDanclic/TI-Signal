# 第五问树莓派视觉控制

这套程序只从摄像头读取示波器屏幕，不读取信号源面板，也不与信号源产生任何电气连接。树莓派通过串口把探测波形和目标 DDS 参数交给 STM32，再由 STM32 转发给 FPGA。

## 测量流程

1. 从 100 us 开始单锯齿粗测；每档等待 180 ms 后独立采集 1 s，每帧拐点数中位数少于 5 时依次切换 500 us、2 ms。
2. 每帧使用实测固定机位标定，只寻找左右高亮拐点并计算 L→L、R→R 完整周期；跨帧使用点数中位数、有效帧率、MAD/CV，并剔除漏点形成的 2 倍及以上长间隔。
3. 粗测合格后分别输出 FPGA 定时的 3 ms 和 7 ms 双细条；每种间隔至少三个圆周平均相位块一致后联合消歧。
4. 只有精测成功才发送目标、幅度、相位和 32 位 DDS 字。FPGA 输出目标正弦（∞模式内部倍频），摄像头继续校正目标幅相并确认稳定。

主循环为非阻塞状态机，测量期间仍每 500 ms 发送心跳并实时处理 `EVENT_CANCEL`。

## 树莓派接线

所有设备必须共地，串口是 3.3 V TTL，禁止接 RS-232 电平。

| 树莓派 | STM32 Apollo |
|---|---|
| GPIO14 / TXD | PA3 / USART2_RX |
| GPIO15 / RXD | PA2 / USART2_TX |
| GND | GND |

STM32 与 FPGA 继续使用原 USART1 接线。板载 LED0/LED1 用于运行和锁定提示。代码把 `PF8` 配置成高电平有效的外接有源蜂鸣器输出；蜂鸣器另一端按模块要求接 GND，若模块电流超过 GPIO 允许值必须加三极管。

## 系统配置

在 Raspberry Pi OS 中关闭串口登录终端并启用硬件串口：

```bash
sudo raspi-config
# Interface Options -> Serial Port
# login shell: No, serial hardware: Yes
sudo reboot
```

安装依赖并启动：

```bash
cd raspberry_pi_vision
python3 -m venv .venv
. .venv/bin/activate
pip install -r requirements.txt
python3 main.py --config config.yaml --preview
```

也可以只部署已整合的 `task5_cv_single.py`。它内置当前实测配置，不依赖本目录内其他 Python 文件：

```bash
python3 task5_cv_single.py --preview
```

需要现场覆盖少量参数时仍可传入 YAML，未填写的参数继续使用内置默认值：

```bash
python3 task5_cv_single.py --config config.yaml
```

现场固定相机、关闭自动曝光后，先修改 `config.yaml` 的 `camera.roi`，使校正后的 640x480 图像恰好覆盖示波器 10x8 div 网格。绿色或黄色轨迹通常可直接使用默认 HSV；其他颜色需要修改 `hsv_low/hsv_high`。

正式测试关闭 `preview`，减少显示开销：

```bash
python3 main.py --config config.yaml
```

## STM32 按键

在 Task4 长按 UP 进入 Task5，Task5 再次长按 UP 退出并返回 Task4。Task5 中：

| 按键 | 自动目标 |
|---|---|
| LEFT | 直线 |
| DOWN | 圆/椭圆 |
| RIGHT | 水平∞ |

自动运行期间四个按键均锁定。锁定或报错后才能重新一键启动，避免违反测试过程中途不得手动调节的要求。

FPGA 本地调试键保持当前定义：K4 输出 10 kHz、±2 V DDS 正弦；K5/K6/K7 分别固定选择 0.1/0.5/2 ms 待机锯齿，并退出本地 DDS。协议新增的 5 ms 只由 `PROBE_SINGLE(P0=3)` 选择，不改变本地按键。

## 固定机位拐点调试

固定摄像头后，单文件调试程序不会逐帧搜索屏幕边框或上下参考亮线，而是直接使用 `opencv_main.py` 顶部的一次性标定常量，只处理波形左右两侧的高亮拐点：

```bash
python3 opencv_main.py --ramp-us 500
```

`--ramp-us` 允许 `100`、`500`、`2000`、`5000`，必须与 FPGA 当前锯齿档位一致。相机位置、焦距或裁切改变后，只需重新填写 `FIXED_SCREEN_CORNERS` 和固定标尺参数，不要恢复逐帧参考线搜索。当前 `104.05/469.05` 曲线标尺来自实测最佳 `TI_code_main.py`。`main.py` 是带 STM32 串口的完整入口，`opencv_main.py` 用于固定机位标定和验证。

## 离线验证

```bash
python -m pytest -q
python main.py --config config.yaml --source /path/to/scope_video.mp4 --preview
```

离线视频模式可检查 ROI、HSV 和轨迹提取，但完整自动控制仍需要 STM32/FPGA 串口和随命令变化的实时示波器画面。
