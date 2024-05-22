import os
from cryptography.hazmat.primitives import serialization
from cryptography.hazmat.primitives.asymmetric import rsa
from cryptography.hazmat.backends import default_backend

# RSA 키 쌍 생성 및 저장
def generate_keys():
    # RSA 키 쌍 생성
    private_key = rsa.generate_private_key(
        public_exponent=65537,
        key_size=4096,
        backend=default_backend()
    )
    
    if not os.path.exists('data'):
        os.makedirs('data')

    # 개인 키 저장
    private_key_path = os.path.join('data', 'private_key.pem')
    with open(private_key_path, "wb") as key_file:
        key_file.write(private_key.private_bytes(
            encoding=serialization.Encoding.PEM,
            format=serialization.PrivateFormat.TraditionalOpenSSL,
            encryption_algorithm=serialization.NoEncryption()
        ))

    # 공개 키 저장
    public_key = private_key.public_key()
    public_key_path = os.path.join('data', 'public_key.pem')
    with open(public_key_path, "wb") as key_file:
        key_file.write(public_key.public_bytes(
            encoding=serialization.Encoding.PEM,
            format=serialization.PublicFormat.SubjectPublicKeyInfo
        ))

# RSA 키 쌍 생성 및 저장 함수 호출
generate_keys()
