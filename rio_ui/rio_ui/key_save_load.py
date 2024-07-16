import os
from cryptography.hazmat.primitives.asymmetric import rsa
from cryptography.hazmat.primitives import serialization
from cryptography.hazmat.backends import default_backend

from ament_index_python.packages import get_package_share_directory

class KeySaveLoad():
    def __init__(self):
        # keys_dir = os.path.join(get_package_share_directory("rio_ui"), "data", "keys")
        keys_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../../../../src/rio_ui/rio_ui/data/keys/'))
        os.makedirs(keys_dir, exist_ok=True)

        self.private_key_path = os.path.join(keys_dir, 'private_key.pem')
        self.public_key_path = os.path.join(keys_dir, 'public_key.pem')

    def generate_keys(self):
        private_key = rsa.generate_private_key(
            public_exponent=65537,
            key_size=4096,
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

def main():
    keysaveload = KeySaveLoad()

    if not os.path.exists(keysaveload.private_key_path) or not os.path.exists(keysaveload.public_key_path):
        keysaveload.generate_keys()
        print(f"Keys saved at :{keysaveload.public_key_path}") 
    else:
        print("Keys already exist. No need to generate new keys.")

if __name__ == "__main__":
    main()
