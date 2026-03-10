import os
import signal
import numpy as np
import pyaudio
import collections
import time
import sys
from array import array
from vad import VoiceActivityDetection
from scipy.io.wavfile import write
from modelscope.outputs import OutputKeys
from modelscope.pipelines import pipeline
from modelscope.utils.constant import Tasks
def listen(vad_model, threshold=0.7, duration=60):
    """
    Use pyaudio to capture audio and perform VAD using ONNX model.
    """
    FORMAT = pyaudio.paInt16
    CHANNELS = 1
    RATE = 16000
    CHUNK_SIZE = 100  # ms
    PADDING_DURATION_MS = 400  # 400 ms for judgement
    RAW_CHUNK = int(RATE * CHUNK_SIZE / 1000)  # Chunk size to read
    NUM_WINDOW_CHUNKS = int(PADDING_DURATION_MS / CHUNK_SIZE)
    NUM_WINDOW_CHUNKS_END = NUM_WINDOW_CHUNKS * 2
    pa = pyaudio.PyAudio()
    stream = pa.open(format=FORMAT, channels=CHANNELS, rate=RATE, input=True, start=False, frames_per_buffer=RAW_CHUNK)
    got_a_sentence = False
    leave = False

    def handle_int(sig, chunk):
        global leave, got_a_sentence
        leave = True
        got_a_sentence = True

    signal.signal(signal.SIGINT, handle_int)

    while not leave:
        ring_buffer = collections.deque(maxlen=NUM_WINDOW_CHUNKS)
        triggered = False
        voiced_frames = []
        ring_buffer_flags = [0] * NUM_WINDOW_CHUNKS
        ring_buffer_index = 0

        ring_buffer_flags_end = [0] * NUM_WINDOW_CHUNKS_END
        ring_buffer_index_end = 0

        raw_data = array('h')
        index = 0
        StartTime = time.time()
        os.system("mpv /home/aubo/aubo_dev/src/llmrobotics/LLM/audio_input/resource/ding-126626.mp3")
        print("* recording: ")
        stream.start_stream()

        while not got_a_sentence and not leave:
            chunk = stream.read(RAW_CHUNK)
            raw_data.extend(array('h', chunk))
            index += RAW_CHUNK
            TimeUse = time.time() - StartTime

            frame_np = np.frombuffer(chunk, dtype=np.int16)
            # frame_float = int2float(frame_np)
            frame_float = frame_np / np.iinfo(np.int16).max  # Normalize
            # Use your VAD module to perform VAD and detect voice activity
            res = vad_model(frame_float, sr=RATE).item()

            active = res > threshold
            sys.stdout.write('1' if active else '_')
            ring_buffer_flags[ring_buffer_index] = 1 if active else 0
            ring_buffer_index += 1
            ring_buffer_index %= NUM_WINDOW_CHUNKS

            ring_buffer_flags_end[ring_buffer_index_end] = 1 if active else 0
            ring_buffer_index_end += 1
            ring_buffer_index_end %= NUM_WINDOW_CHUNKS_END

            # Start point detection
            if not triggered:
                if TimeUse > 5:
                    print('end of listening')
                    return False
                ring_buffer.append(chunk)
                num_voiced = sum(ring_buffer_flags)
                if num_voiced > 0.8 * NUM_WINDOW_CHUNKS:
                    sys.stdout.write(' Open ')
                    triggered = True
                    voiced_frames.extend(ring_buffer)
                    ring_buffer.clear()
            # End point detection
            else:
                voiced_frames.append(chunk)
                ring_buffer.append(chunk)
                num_unvoiced = NUM_WINDOW_CHUNKS_END - sum(ring_buffer_flags_end)
                if num_unvoiced > 0.90 * NUM_WINDOW_CHUNKS_END or TimeUse > duration:
                    sys.stdout.write(' Close ')
                    triggered = False
                    got_a_sentence = True

            sys.stdout.flush()

        sys.stdout.write('\n')
        if not voiced_frames:
            return False
        data = b''.join(voiced_frames)

        stream.stop_stream()
        print("* done recording")
        got_a_sentence = False
        leave = True

    stream.close()
    return data

def save_bytes_wavfile(file_name,sample_rate,audio_bytes):
    # 假设 audio_bytes 是一个字节对象
    # sample_rate 是音频的采样率，例如 44100 Hz

    # 解码字节数据到 numpy 数组
    # 这里假设音频是 16-bit PCM 编码的单声道音频
    # 如果是立体声，channels 将会是 2 或更多
    # 如果是 8-bit PCM，dtype 应该改为 'int8'
    if len(audio_bytes) % 2 == 0:
        dtype = 'int16'  # 对于 16-bit PCM 数据
        n_channels = 1   # 单声道
        byte_depth = 2   # 每个样本的字节数
    else:
        raise ValueError("字节长度不符合预期的位深度")

    audio_array = np.frombuffer(audio_bytes, dtype=dtype)

    # 如果是立体声，需要调整形状
    if n_channels > 1:
        audio_array = audio_array.reshape((-1, n_channels))

    # 写入 .wav 文件
    write(file_name, sample_rate, audio_array)

def main():
    vad = VoiceActivityDetection(model_path = '/home/aubo/aubo_dev/src/llmrobotics/LLM/audio_input/resource/silero_vad.onnx')
    paraformer = pipeline(task=Tasks.auto_speech_recognition,
                      model='iic/speech_paraformer-large-contextual_asr_nat-zh-cn-16k-common-vocab8404',
                      model_revision="v2.0.4",
                      punc_model='iic/punc_ct-transformer_zh-cn-common-vocab272727-pytorch',
                      punc_model_revision="v2.0.4")
    audio_data = listen(vad,threshold=0.3,duration=15)
    if audio_data:
        print("Audio data captured.")
        raw_prompt = paraformer(audio_data, hotword="遨博")
        print(raw_prompt[0]['text'])
        # Save as WAV file
        # import wave
        # with wave.open("output.wav", "wb") as wf:
        #     wf.setnchannels(1)
        #     wf.setsampwidth(2)  # 16-bit audio
        #     wf.setframerate(16000)
        #     wf.writeframes(audio_data)
        # print("Audio saved to output.wav")
        # 保存录音为 .wav 文件
        save_bytes_wavfile('output.wav', 16000, audio_data)
        print("录音已保存为 output.wav")
    else:
        print("No audio detected.")

if __name__ == "__main__":
    main()