import os
import sys
import speech_recognition as sr

def audio_to_text(audio_path):
    recognizer = sr.Recognizer()

    # Load audio file
    with sr.AudioFile(audio_path) as source:
        audio_data = recognizer.record(source)

    try:
        # Perform speech recognition
        text = recognizer.recognize_google(audio_data, language='ko-KR')  # 한국어로 설정
        return text
    except sr.UnknownValueError:
        print(f"Speech recognition could not understand audio: {audio_path}")
        return None
    except sr.RequestError as e:
        print(f"Could not request results from Google Speech Recognition service for {audio_path}: {e}")
        return None

def main():
    print(sys.path)
    directory = "/home/jongchanjang/my_mobile/src/ros_package/resource/mic_data"
    
    # Process all WAV files in the input directory
    for filename in os.listdir(directory):
        if filename.endswith(".wav"):
            audio_path = os.path.join(directory, filename)
            text = audio_to_text(audio_path)
            if text:
                print(f"Text recognized from {filename}: {text}")

if __name__ == "__main__":
    main()
