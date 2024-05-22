import os
import cv2
import base64
import time
import mysql.connector
from datetime import datetime
from pyzbar.pyzbar import decode
from cryptography.hazmat.primitives import serialization, hashes
from cryptography.hazmat.primitives.asymmetric import padding
from cryptography.hazmat.backends import default_backend

# 개인 키 로드
def load_private_key(private_key_path):
    with open(private_key_path, "rb") as key_file:
        private_key = serialization.load_pem_private_key(
            key_file.read(),
            password=None,  # 개인 키 파일에 비밀번호가 없는 경우 None으로 설정
            backend=default_backend()
        )
    return private_key

# MySQL 데이터베이스 연결
def connect_to_database():
    connection = mysql.connector.connect(
        host='localhost',
        port=3306,
        user='root',      # MySQL 사용자 이름
        password='1234',  # MySQL 사용자 비밀번호
        database='test_qr_sms'     # 사용할 데이터베이스 이름
    )
    return connection

# 날짜 형식 변환
def convert_date(date_str):
    # "5월 16일" 형식에서 "YYYY-MM-DD" 형식으로 변환
    months = {
        "1월": "01", "2월": "02", "3월": "03", "4월": "04",
        "5월": "05", "6월": "06", "7월": "07", "8월": "08",
        "9월": "09", "10월": "10", "11월": "11", "12월": "12"
    }
    month_day = date_str.split(' ')
    month = months[month_day[0]]
    day = month_day[1].replace('일', '')
    year = datetime.now().year
    return f"{year}-{month}-{day.zfill(2)}"

# 시간 형식 변환
def convert_time(time_str):
    # "2시 30분" 형식에서 "HH:MM:SS" 형식으로 변환
    hour_min = time_str.split(' ')
    hour = hour_min[0].replace('시', '')
    minute = hour_min[1].replace('분', '')
    return f"{hour.zfill(2)}:{minute.zfill(2)}:00"

# 로봇 안내 여부 변환
def convert_robot_guidance(guidance_str):
    return 1 if guidance_str == '네' else 0

# 복호화된 데이터 삽입
def insert_decrypted_data(connection, decrypted_data):
    cursor = connection.cursor()
    insert_query = '''
    INSERT INTO VisitRecords (
        visit_place, purpose, name, phone_number, affiliation, visit_date, visit_time, robot_guidance
    ) VALUES (%s, %s, %s, %s, %s, %s, %s, %s)
    '''
    
    data = decrypted_data.split(', ')
    values = (
        data[0].split(': ')[1],
        data[1].split(': ')[1],
        data[2].split(': ')[1],
        data[3].split(': ')[1],
        data[4].split(': ')[1],
        convert_date(data[5].split(': ')[1]),  # "5월 16일" 형식 변환
        convert_time(data[6].split(': ')[1]),  # "2시 30분" 형식 변환
        convert_robot_guidance(data[7].split(': ')[1])  # "네" 또는 "아니오" 변환
    )

    cursor.execute(insert_query, values)
    connection.commit()
    cursor.close()

# QR 코드 읽기 및 복호화
def read_qr_code(private_key, connection):
    cap = cv2.VideoCapture(0)
    last_message_time = time.time()
    message_interval = 3  # 메시지 출력 간격 (초)
    
    while True:
        ret, frame = cap.read()
        
        if not ret:
            print("Failed to grab frame")
            break
        
        qr_codes = decode(frame)
        if qr_codes:
            for qr_code in qr_codes:
                qr_data = qr_code.data.decode('utf-8')
                print("QR Code detected")

                # QR 코드 데이터 디코딩
                encrypted_data = base64.b64decode(qr_data)

                # 데이터 복호화
                decrypted_data = private_key.decrypt(
                    encrypted_data,
                    padding.OAEP(
                        mgf=padding.MGF1(algorithm=hashes.SHA256()),
                        algorithm=hashes.SHA256(),
                        label=None
                    )
                )

                decrypted_data_str = decrypted_data.decode('utf-8')
                print("Decrypted Data:", decrypted_data_str)

                # 복호화된 데이터 삽입
                insert_decrypted_data(connection, decrypted_data_str)

                cap.release()
                cv2.destroyAllWindows()
                return
        else:
            if time.time() - last_message_time >= message_interval:
                print("QR 코드를 찾을 수 없습니다. 다시 시도 중...")
                last_message_time = time.time()

        cv2.imshow('QR Code Scanner', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    private_key_path = os.path.join('data', 'private_key.pem')
    private_key = load_private_key(private_key_path)

    connection = connect_to_database()
    read_qr_code(private_key, connection)
    connection.close()
