# upload_files.py
import os
import time
import requests

UPLOAD_FOLDER = '/home/ito/ros_final/src/flask/uploads'
SERVER_URL = 'http://127.0.0.1:5001/upload'  # Flask 서버의 IP 주소와 포트를 입력

uploaded_files = set()

while True:
    files = os.listdir(UPLOAD_FOLDER)
    for file in files:
        if file not in uploaded_files:
            with open(os.path.join(UPLOAD_FOLDER, file), 'rb') as f:
                r = requests.post(SERVER_URL, files={'file': f})
                if r.status_code == 200:
                    uploaded_files.add(file)
                    print(f'Uploaded: {file}')
    time.sleep(1)

    # download_files.py
import os
import time
import requests

DOWNLOAD_FOLDER = 'downloads'
SERVER_URL = 'http://0.0.0.0:5000/upload'
LIST_FILES_URL = f'{SERVER_URL}/list_files'
DOWNLOAD_FILE_URL = f'{SERVER_URL}/download'

os.makedirs(DOWNLOAD_FOLDER, exist_ok=True)

downloaded_files = set()

while True:
    r = requests.get(LIST_FILES_URL)
    if r.status_code == 200:
        files = r.json()
        for file in files:
            if file not in downloaded_files:
                r = requests.get(f'{DOWNLOAD_FILE_URL}/{file}')
                if r.status_code == 200:
                    with open(os.path.join(DOWNLOAD_FOLDER, file), 'wb') as f:
                        f.write(r.content)
                    downloaded_files.add(file)
                    print(f'Downloaded: {file}')
    time.sleep(1)