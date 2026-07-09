import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import hilbert
import os

# ==================== 解决中文显示 ====================
plt.rcParams['font.sans-serif'] = ['SimHei']      # 或 'Microsoft YaHei'
plt.rcParams['axes.unicode_minus'] = False

# ==================== 参数设置 ====================
fs = 200_000
fc = 20_000
fm = 1000
duration = 0.005
sps = 50
num_symbols = int(fs * duration / sps)

t = np.linspace(0, duration, int(fs * duration), endpoint=False)

# ==================== 基带信号 ====================
m_tone = np.sin(2 * np.pi * fm * t)

bits_symbols = np.random.randint(0, 2, num_symbols)
digital_base = np.repeat(bits_symbols, sps)
digital_base = digital_base[:len(t)]

# ==================== 调制函数（保持不变） ====================
def am_mod(m, fc, fs, ma=0.8):
    t = np.arange(len(m)) / fs
    return (1 + ma * m) * np.cos(2 * np.pi * fc * t)

def dsb_mod(m, fc, fs):
    t = np.arange(len(m)) / fs
    return m * np.cos(2 * np.pi * fc * t)

def ssb_mod(m, fc, fs):
    t = np.arange(len(m)) / fs
    m_hat = hilbert(m)
    return m * np.cos(2 * np.pi * fc * t) - np.imag(m_hat) * np.sin(2 * np.pi * fc * t)

def fm_mod(m, fc, fs, kf=5000):
    t = np.arange(len(m)) / fs
    integral = np.cumsum(m) / fs
    return np.cos(2 * np.pi * fc * t + 2 * np.pi * kf * integral)

def pm_mod(m, fc, fs, kp=np.pi/2):
    t = np.arange(len(m)) / fs
    return np.cos(2 * np.pi * fc * t + kp * m)

def ask_mod(digital, fc, fs):
    t = np.arange(len(digital)) / fs
    return digital * np.cos(2 * np.pi * fc * t)

def fsk_mod(digital, fc, fs, delta_f=5000):
    t = np.arange(len(digital)) / fs
    freq_dev = (2 * digital - 1) * delta_f
    phase = 2 * np.pi * np.cumsum(freq_dev) / fs
    return np.cos(2 * np.pi * fc * t + phase)

def bpsk_mod(digital, fc, fs):
    t = np.arange(len(digital)) / fs
    return np.cos(2 * np.pi * fc * t + np.pi * digital)

def qpsk_mod(fc, fs, sps, num_symbols):
    t = np.arange(num_symbols * sps) / fs
    bits = np.random.randint(0, 2, size=num_symbols * 2)
    I = np.zeros(num_symbols)
    Q = np.zeros(num_symbols)
    for i in range(num_symbols):
        b1, b0 = bits[2*i], bits[2*i+1]
        I[i] =  1 if b1 == 0 else -1
        Q[i] =  1 if b0 == 0 else -1
    I_up = np.repeat(I, sps)[:len(t)]
    Q_up = np.repeat(Q, sps)[:len(t)]
    return I_up * np.cos(2 * np.pi * fc * t) - Q_up * np.sin(2 * np.pi * fc * t)

def qam16_mod(fc, fs, sps, num_symbols):
    t = np.arange(num_symbols * sps) / fs
    bits = np.random.randint(0, 2, size=num_symbols * 4)
    I = np.zeros(num_symbols)
    Q = np.zeros(num_symbols)
    for i in range(num_symbols):
        b3, b2, b1, b0 = bits[4*i:4*i+4]
        I[i] = (2 * b3 - 1) * (1 + 0.5 * b2)
        Q[i] = (2 * b1 - 1) * (1 + 0.5 * b0)
    I_up = np.repeat(I, sps)[:len(t)]
    Q_up = np.repeat(Q, sps)[:len(t)]
    return I_up * np.cos(2 * np.pi * fc * t) - Q_up * np.sin(2 * np.pi * fc * t)

# ==================== 生成所有调制信号 ====================
modulations = {
    "AM": am_mod(m_tone, fc, fs),
    "DSB-SC": dsb_mod(m_tone, fc, fs),
    "SSB": ssb_mod(m_tone, fc, fs),
    "FM": fm_mod(m_tone, fc, fs),
    "PM": pm_mod(m_tone, fc, fs),
    "ASK": ask_mod(digital_base, fc, fs),
    "FSK": fsk_mod(digital_base, fc, fs),
    "BPSK": bpsk_mod(digital_base, fc, fs),
    "QPSK": qpsk_mod(fc, fs, sps, num_symbols),
    "16-QAM": qam16_mod(fc, fs, sps, num_symbols),
}

# ==================== 逐个绘制并保存 ====================
# 获取当前脚本所在目录（也可用 os.getcwd()）
save_dir = os.path.dirname(os.path.abspath(__file__)) or '.'

for name, signal in modulations.items():
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 4))
    fig.suptitle(f"{name} 调制信号", fontsize=14)

    # 时域（前 200 个采样）
    n_show = min(200, len(signal))
    ax1.plot(t[:n_show] * 1000, signal[:n_show], linewidth=0.8)
    ax1.set_title("时域波形")
    ax1.set_xlabel("时间 (ms)")
    ax1.set_ylabel("幅度")
    ax1.grid(True, alpha=0.3)

    # 频谱（0~50 kHz）
    N = len(signal)
    f_axis = np.fft.fftfreq(N, 1 / fs)
    spectrum = np.fft.fft(signal)
    spectrum_mag = np.abs(np.fft.fftshift(spectrum))
    f_shift = np.fft.fftshift(f_axis)
    pos_mask = f_shift >= 0
    ax2.plot(f_shift[pos_mask] / 1000,
             20 * np.log10(spectrum_mag[pos_mask] + 1e-12),
             linewidth=0.8)
    ax2.set_title("频谱")
    ax2.set_xlabel("频率 (kHz)")
    ax2.set_ylabel("幅度 (dB)")
    ax2.grid(True, alpha=0.3)
    ax2.set_xlim(0, 50)

    plt.tight_layout()
    # 保存为 PNG，文件名去除可能的不合法字符（此处均合法）
    filename = os.path.join(save_dir, f"{name}.png")
    plt.savefig(filename, dpi=150, bbox_inches='tight')
    plt.close(fig)   # 关闭当前图形，释放内存

print("所有调制图像已保存到当前目录。")