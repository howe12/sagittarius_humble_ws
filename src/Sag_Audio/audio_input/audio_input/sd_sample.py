import sounddevice as sd
import numpy as np
from scipy.io.wavfile import write
import time
import signal

# 设置录音参数
fs = 44100   # 采样率
channels = 1 # 声道数，1为单声道
block_size = 1024  # 每次读取的块大小
duration = 5  # 默认录音时长（秒）

# 初始化录音数据缓冲区
audio_buffer = []

# 定义一个回调函数
def callback(indata, frames, time, status):
    if status:
        print(status)
    # 将音频数据追加到缓冲区
    audio_buffer.append(indata.copy())

# 处理 Ctrl+C 信号
def signal_handler(sig, frame):
    print('检测到 Ctrl+C，正在保存录音...')
    save_recording()
    exit(0)

# 保存录音到文件
def save_recording():
    # 将列表转换为 NumPy 数组
    data = np.concatenate(audio_buffer, axis=0)
    # 保存录音为 .wav 文件
    write('output.wav', fs, data)

# 注册信号处理器
signal.signal(signal.SIGINT, signal_handler)

# 开始录音
try:
    with sd.InputStream(samplerate=fs, channels=channels,
                        blocksize=block_size, callback=callback):
        print("开始录音，按 Ctrl+C 可以停止...")
        start_time = time.time()
        while (time.time() - start_time) < duration:  # 根据 duration 控制录音时间
            time.sleep(0.1)  # 防止 CPU 占用过高
except KeyboardInterrupt:
    pass  # 如果用户按下 Ctrl+C，信号处理器会捕获并处理

# 如果录音未因 Ctrl+C 而中断，则在计时结束后保存录音
save_recording()
print("录音已保存为 output.wav")