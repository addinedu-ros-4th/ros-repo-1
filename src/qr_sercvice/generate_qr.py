import os
import base64
import qrcode
from cryptography.hazmat.primitives.asymmetric import padding
from cryptography.hazmat.primitives import serialization, hashes
from cryptography.hazmat.backends import default_backend

# 예약 정보 암호화
def encrypt_visit_info(visit_info, public_key):
    encrypted_visit_info = public_key.encrypt(
        visit_info.encode(),
        padding.OAEP(
            mgf=padding.MGF1(algorithm=hashes.SHA256()),
            algorithm=hashes.SHA256(),
            label=None
        )
    )
    return encrypted_visit_info

# QR 코드 생성 및 저장
def generate_qr_code(encrypted_visit_info, name):
    encoded_data = base64.b64encode(encrypted_visit_info).decode('utf-8')

    qr = qrcode.QRCode(
        version=1,
        error_correction=qrcode.constants.ERROR_CORRECT_M,
        box_size=10,
        border=4,
    )
    qr.add_data(encoded_data)

    # QR 코드 이미지 생성 및 저장
    qr_img = qr.make_image(fill_color="black", back_color="white")
    qr_path = os.path.join('data', f"{name}_qr_code.png")
    qr_img.save(qr_path)

    return qr_path

if __name__ == "__main__":
    # RSA 공개 키 불러오기
    public_key_path = os.path.join('data', 'public_key.pem')
    with open(public_key_path, "rb") as key_file:
        public_key = serialization.load_pem_public_key(
            key_file.read(),
            backend=default_backend()
        )

    # 사용자로부터 예약 정보 입력 받기
    place = input("방문 장소 : ")
    purpose = input("방문 목적 (ex. 회의 목적): ")
    name = input("이름 : ")
    phone_number = input("전화 번호 (-빼고 입력해주세요): ")
    affiliation = input("소속 : ")
    visit_date = input("방문 날짜 (ex. *월 *일) : ")
    visit_time = input("방문 시간 (ex. 15시 30분): ")
    robot_guide = input("로봇 안내 여부 (ex. 네 or 아니오): ")

    # 예약 정보 생성
    visit_info = f"방문 장소: {place}, 방문 목적: {purpose}, 이름: {name}, 전화 번호: {phone_number}, 소속: {affiliation}, 방문 날짜: {visit_date}, 방문 시간: {visit_time}, 로봇 안내 여부: {robot_guide}"

    # 예약 정보 암호화
    encrypted_visit_info = encrypt_visit_info(visit_info, public_key)

    # QR 코드 생성 및 저장
    qr_path = generate_qr_code(encrypted_visit_info, name)

    print(f"QR 코드 생성이 완료되었습니다. 경로 : {qr_path}")
