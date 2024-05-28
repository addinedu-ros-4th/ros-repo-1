import base64
import json
from pyzbar.pyzbar import decode

from cryptography.hazmat.primitives.asymmetric import padding
from cryptography.hazmat.primitives import hashes

from key_save_load import KeySaveLoad


class DataDecryptor():
    def __init__(self):
        key_save_load = KeySaveLoad()
        self.private_key = key_save_load.load_private_key()

    def decrypt_data(self, encrypted_data):
        encrypted_data = base64.b64decode(encrypted_data.encode('utf-8'))
        decrypted_data = self.private_key.decrypt(
            encrypted_data,
            padding.OAEP(
                mgf=padding.MGF1(algorithm=hashes.SHA256()),
                algorithm=hashes.SHA256(),
                label=None
            )
        )
        decrypted_data = json.loads(decrypted_data.decode('utf-8'))
        return decrypted_data

    def decrypt_config(self, encrypted_file):
        with open(encrypted_file, 'r') as file:
            encrypted_data = file.read()
        decrypted_data = self.decrypt_data(encrypted_data)
        
        return decrypted_data
