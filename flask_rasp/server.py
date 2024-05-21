from flask import Flask, render_template, request, send_file
import os

app = Flask(__name__)

# 다운로드할 파일이 저장될 디렉토리 경로 설정
DOWNLOADS_DIRECTORY = '/home/pink/pinkbot/src/flask_rasp/downloads'

# 파일 목록을 가져와서 템플릿에 전달하는 라우트
@app.route('/')
def show_files():
    files = os.listdir(DOWNLOADS_DIRECTORY)
    return render_template('files.html', files=files)

# 각 파일을 다운로드하는 라우트
@app.route('/download/<filename>')
def download_file(filename):
    file_path = os.path.join(DOWNLOADS_DIRECTORY, filename)
    if os.path.exists(file_path):
        return send_file(file_path, as_attachment=True, attachment_filename=filename)
    else:
        return f'파일 "{filename}"을(를) 찾을 수 없습니다.'

# 파일 업로드를 위한 라우트
@app.route('/upload', methods=['POST'])
def upload_file():
    if 'file' not in request.files:
        return '파일이 없습니다.'
    
    file = request.files['file']

    if file.filename == '':
        return '파일 이름이 없습니다.'

    # 파일을 다운로드 디렉토리에 저장합니다.
    file_path = os.path.join(DOWNLOADS_DIRECTORY, file.filename)
    file.save(file_path)
    return f'파일 "{file.filename}"이 업로드되었습니다.'

if __name__ == '__main__':
    app.run(debug=True)
