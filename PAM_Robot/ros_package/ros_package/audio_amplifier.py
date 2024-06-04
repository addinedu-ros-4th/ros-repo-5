import os
import threading
from pydub import AudioSegment

class AudioAmplifier:
    def __init__(self, gain_dB):
        self.gain_dB = gain_dB

    def amplify_audio(self, input_file, output_file):
        audio = AudioSegment.from_file(input_file)
        amplified_audio = audio + self.gain_dB
        amplified_audio.export(output_file, format=input_file.split('.')[-1])
        print(f"Amplified {input_file} and saved as {output_file}")
