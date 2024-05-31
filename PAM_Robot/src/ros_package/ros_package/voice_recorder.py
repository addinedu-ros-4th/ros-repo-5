import pyaudio
import struct
import numpy as np
import wave
import os
import time

class VoiceRecorder:
    def __init__(self, threshold_energy=100):
        self.FORMAT = pyaudio.paInt16
        self.CHANNELS = 1
        self.RATE = 16000
        self.CHUNK = 1024
        self.THRESHOLD_ENERGY = threshold_energy
        self.audio_path = ""
        self.is_recording = False
        self.audio = pyaudio.PyAudio()
        self.stream = None
        self.frames = []
        self.stop_callback = None  # 콜백 함수

    def set_stop_callback(self, stop_callback):
        """녹음이 중지될 때 호출되는 콜백 함수 설정"""
        self.stop_callback = stop_callback

    def start_recording(self, duration=4):
        """시작 녹음 함수"""
        self.frames = []
        self.stream = self.audio.open(format=self.FORMAT, channels=self.CHANNELS,
                                     rate=self.RATE, input=True,
                                     frames_per_buffer=self.CHUNK)
        print("Listening...")
        self.is_recording = True

        # Record for `duration` seconds initially
        start_time = time.time()
        while time.time() - start_time < duration and self.is_recording:
            data = self.stream.read(self.CHUNK)
            self.frames.append(data)

        # Then continue recording and check for voice activity
        try:
            while self.is_recording:
                data = self.stream.read(self.CHUNK)
                self.frames.append(data)
                samples = struct.unpack(f'{self.CHUNK}h', data)
                energy = np.sum(np.square(samples)) / self.CHUNK
                print(f'Energy: {energy}')
                
                if energy < self.THRESHOLD_ENERGY:
                    print("Voice activity detected! Saving recording...")
                    self.save_recording()
                    break  # Exit the loop after saving the recording
        except Exception as e:
            print(f"Error during recording: {e}")
        finally:
            self.stop_recording()

    def save_recording(self):
        """녹음 파일을 저장하는 함수"""
        path = "/home/jongchanjang/my_mobile/src/ros_package/resource/mic_data"
        
        # Create directory if it does not exist
        if not os.path.exists(path):
            os.makedirs(path)
        
        if not os.listdir(path):
            new_file_name = "0.wav"
        else:
            files = os.listdir(path)
            last_file = sorted(files)[-1]
            last_number = int(last_file.split('.')[0])
            new_file_name = f"{last_number + 1}.wav"

        filename = os.path.join(path, new_file_name)
        
        with wave.open(filename, 'wb') as wf:
            wf.setnchannels(self.CHANNELS)
            wf.setsampwidth(self.audio.get_sample_size(self.FORMAT))
            wf.setframerate(self.RATE)
            wf.writeframes(b''.join(self.frames))
        print(f"Recorded audio saved at: {filename}")
        self.audio_path = filename
        self.is_recording = False

    def stop_recording(self):
        """녹음을 중지하는 함수"""
        self.is_recording = False
        if self.stream:
            self.stream.stop_stream()
            self.stream.close()
        self.audio.terminate()

        if self.stop_callback:
            self.stop_callback()

    def handle_interrupt(self, sig, frame):
        """시그널 인터럽트 처리 함수"""
        self.is_recording = False
        self.save_recording()
        self.stop_recording()
        exit(0)

    def get_audio_path(self):
        """저장된 오디오 파일 경로를 반환하는 함수"""
        return self.audio_path