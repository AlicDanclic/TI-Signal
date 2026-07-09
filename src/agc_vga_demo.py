import numpy as np
import matplotlib.pyplot as plt

# -------------------- 仿真参数 --------------------
FS = 10e3          # 采样率 (Hz)
FC = 500           # 信号频率 (Hz)
DURATION = 0.1     # 仿真时长 (s)
T = np.arange(0, DURATION, 1/FS)

# 输入信号：幅度阶跃变化
AMP_PATTERN = [0.1, 1.0, 0.3, 0.8, 0.05]  # 相对幅度
SEG_LEN = len(T) // len(AMP_PATTERN)
amp_env = np.concatenate([np.full(SEG_LEN, a) for a in AMP_PATTERN])[:len(T)]
v_in = amp_env * np.sin(2 * np.pi * FC * T)

# -------------------- VGA 类 (开环) --------------------
class VGA:
    """线性 dB 控制的可变增益放大器 (开环)"""
    def __init__(self, vc=0.5, gain_slope_db_per_v=40, max_gain_db=20):
        self.gain_slope = gain_slope_db_per_v   # dB/V
        self.max_gain_db = max_gain_db
        self.vc = vc                            # 控制电压
        self.update_gain()

    def update_gain(self):
        """根据控制电压计算线性增益"""
        gain_db = self.max_gain_db + self.gain_slope * self.vc
        self.gain_linear = 10 ** (gain_db / 20)

    def set_control(self, vc):
        self.vc = vc
        self.update_gain()

    def process(self, vin):
        return self.gain_linear * vin

# -------------------- AGC 系统 (闭环) --------------------
class AGC:
    """包含 VGA、包络检波、积分器(环路滤波器)的闭环 AGC"""
    def __init__(self, ref_level=0.5, gain_slope=40, tau=0.005, fs=FS):
        self.ref = ref_level                  # 目标输出幅度 (线性)
        self.gain_slope = gain_slope          # dB/V
        self.tau = tau                        # 积分器时间常数 (s)
        self.fs = fs
        self.vc = 0.0                         # 积分器输出（控制电压初始值）
        self.vga = VGA(vc=self.vc, gain_slope_db_per_v=gain_slope)
        # 包络检波的低通滤波常数 (简单一阶低通)
        self.alpha = np.exp(-1 / (fs * 0.001))  # 1ms 时间常数

    def envelope_detector(self, vin):
        """平方律检波 + 一阶低通 (提取 RMS 近似)"""
        return np.abs(vin)                     # 此处简化，用瞬时绝对值

    def step(self, vin_sample):
        # 1. VGA 输出
        vout = self.vga.process(vin_sample)

        # 2. 包络检波
        v_det = self.envelope_detector(vout)
        # 可选：低通滤波 (简化直接使用)
        # self.vdet_filt = self.alpha * self.vdet_filt + (1 - self.alpha) * v_det

        # 3. 误差信号
        err = self.ref - v_det

        # 4. 积分器 (欧拉离散)
        # dv/dt = K_i * err,  其中 K_i = 1/tau (或含增益斜率折算)
        # 这里将积分增益与斜率结合，保持 dB 线性控制
        K_i = 1.0 / (self.tau * self.gain_slope)   # 折算到 V/s
        self.vc += K_i * (1/self.fs) * err
        # 电压限幅 (0~1.4V 等)
        self.vc = np.clip(self.vc, 0.0, 1.4)

        # 5. 更新 VGA 增益
        self.vga.set_control(self.vc)

        return vout, self.vga.gain_linear

# -------------------- 运行仿真 --------------------
# ----- VGA 仿真 (固定控制电压 0.5V) -----
vga_inst = VGA(vc=0.5, gain_slope_db_per_v=40, max_gain_db=10)
vga_out = vga_inst.process(v_in)

# ----- AGC 仿真 (逐样本迭代) -----
agc = AGC(ref_level=0.5, gain_slope=40, tau=0.01, fs=FS)
agc_out = np.zeros_like(v_in)
agc_gain_lin = np.zeros_like(v_in)
agc_vc = np.zeros_like(v_in)

for i, sample in enumerate(v_in):
    vout, g_lin = agc.step(sample)
    agc_out[i] = vout
    agc_gain_lin[i] = g_lin
    agc_vc[i] = agc.vc

# -------------------- 绘图 --------------------
plt.figure(figsize=(12, 9))

plt.subplot(4, 1, 1)
plt.plot(T, v_in, label='Input signal (amplitude steps)')
plt.ylabel('Amplitude')
plt.legend()
plt.grid(True)

plt.subplot(4, 1, 2)
plt.plot(T, vga_out, label='VGA output (fixed control $V_c$=0.5V)', alpha=0.7)
plt.plot(T, agc_out, label='AGC output (closed‑loop, ref=0.5)')
plt.ylabel('Amplitude')
plt.legend()
plt.grid(True)

plt.subplot(4, 1, 3)
gain_db_vga = 20 * np.log10(np.abs(vga_out) / (np.abs(v_in) + 1e-12))
gain_db_agc = 20 * np.log10(np.abs(agc_out) / (np.abs(v_in) + 1e-12))
plt.plot(T, gain_db_vga, label='VGA instantaneous gain (dB)')
plt.plot(T, gain_db_agc, label='AGC instantaneous gain (dB)')
plt.ylabel('Gain (dB)')
plt.legend()
plt.grid(True)

plt.subplot(4, 1, 4)
plt.plot(T, agc_vc, label='AGC control voltage $V_c$')
plt.axhline(0.5, color='gray', linestyle='--', label='VGA fixed $V_c$=0.5V')
plt.ylabel('Control Voltage (V)')
plt.xlabel('Time (s)')
plt.legend()
plt.grid(True)

plt.tight_layout()
plt.show()