import os
from cryptography.hazmat.primitives.asymmetric import rsa
from cryptography.hazmat.primitives import serialization
from cryptography.hazmat.backends import default_backend

class KeySaveLoad():
    def __init__(self):
        keys_dir = 'keys'
        if not os.path.exists(keys_dir):
            os.makedirs(keys_dir)

        self.private_key_path = os.path.join(keys_dir, 'private_key.pem')
        self.public_key_path = os.path.join(keys_dir, 'public_key.pem')

    def generate_keys(self):

        private_key = rsa.generate_private_key(
            public_exponent=65537,
            key_size=2048,
            backend=default_backend()
        )
        
        public_key = private_key.public_key()
        
        # 개인 키 저장
        with open(self.private_key_path, 'wb') as priv_file:
            priv_file.write(
                private_key.private_bytes(
                    encoding=serialization.Encoding.PEM,
                    format=serialization.PrivateFormat.TraditionalOpenSSL,
                    encryption_algorithm=serialization.NoEncryption()
                )
            )
    
        # 공개 키 저장
        with open(self.public_key_path, 'wb') as pub_file:
            pub_file.write(
                public_key.public_bytes(
                    encoding=serialization.Encoding.PEM,
                    format=serialization.PublicFormat.SubjectPublicKeyInfo
                )
            )

    def load_private_key(self):
        try:
            with open(self.private_key_path, 'rb') as key_file:
                private_key = serialization.load_pem_private_key(
                    key_file.read(),
                    password=None,
                    backend=default_backend()
                )
            return private_key
        except FileNotFoundError:
            print(f"Private key file not found: {self.private_key_path}")
        except Exception as e:
            print(f"An error occurred while loading the private key: {str(e)}")

    def load_public_key(self):
        try:
            with open(self.public_key_path, 'rb') as key_file:
                public_key = serialization.load_pem_public_key(
                    key_file.read(),
                    backend=default_backend()
                )
            return public_key
        except FileNotFoundError:
            print(f"Public key file not found: {self.public_key_path}")
        except Exception as e:
            print(f"An error occurred while loading the public key: {str(e)}")
