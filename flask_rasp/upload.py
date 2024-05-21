import os
import time
import requests
import logging
import threading

UPLOAD_FOLDER = '/home/pink/pinkbot/src/flask_rasp/uploads'
SERVER_URL = 'http://192.168.0.47:5000/upload'
UPLOAD_INTERVAL = 1  # 업로드 확인 주기 (초)

logging.basicConfig(level=logging.INFO)

uploaded_files = set()

def upload_files():
    while True:
        files = os.listdir(UPLOAD_FOLDER)
        for file in files:
            if file not in uploaded_files:
                try:
                    with open(os.path.join(UPLOAD_FOLDER, file), 'rb') as f:
                        logging.info(f'Trying to upload: {file}')
                        r = requests.post(SERVER_URL, files={'file': f})
                        if r.status_code == 200:
                            uploaded_files.add(file)
                            logging.info(f'Uploaded: {file}')
                        else:
                            logging.error(f'Failed to upload: {file} - Status code: {r.status_code}')
                except requests.exceptions.RequestException as e:
                    logging.error(f'Connection error: {e}')
        time.sleep(UPLOAD_INTERVAL)

# 업로드 스레드를 생성하고 시작
upload_thread = threading.Thread(target=upload_files, daemon=True)
upload_thread.start()


try:
    while True:
        # 여기에 메인 프로그램의 다른 작업을 추가할 수 있습니다.
        time.sleep(UPLOAD_INTERVAL)  # 메인 루프의 실행 주기 (초)
except KeyboardInterrupt:
    logging.info('프로그램이 종료되었습니다.')
