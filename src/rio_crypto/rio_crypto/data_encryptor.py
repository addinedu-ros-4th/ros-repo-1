import base64
import json
from pyzbar.pyzbar import decode

from cryptography.hazmat.primitives.asymmetric import padding
from cryptography.hazmat.primitives import hashes

from rio_crypto.key_save_load import KeySaveLoad

class DataEncryptor():
    def __init__(self):
        key_save_load = KeySaveLoad()
        self.public_key = key_save_load.load_public_key()

    def encrypt_data(self, data):
        if isinstance(data, dict):
            data = json.dumps(data)

        encrypted_data = self.public_key.encrypt(
            data.encode('utf-8'),
            padding.OAEP(
                mgf=padding.MGF1(algorithm=hashes.SHA256()),
                algorithm=hashes.SHA256(),
                label=None
            )
        )

        encrypted_data = base64.b64encode(encrypted_data).decode('utf-8')
        return encrypted_data

    def encrypt_config(self, config_file, output_file):
        with open(config_file, 'r') as file:
            config_data = json.load(file)
        encrypted_data = self.encrypt_data(config_data, self.public_key)
        with open(output_file, 'w') as file:
            file.write(encrypted_data)


# def generate_qr_code(encrypted_data, output_dir, name):
#     if not os.path.exists(output_dir):
#         os.makedirs(output_dir)

#     qr = qrcode.QRCode(
#         version=1,
#         error_correction=qrcode.constants.ERROR_CORRECT_M,
#         box_size=10,
#         border=4,
#     )
#     qr.add_data(encrypted_data)

#     qr_img = qr.make_image(fill_color="black", back_color="white")
#     qr_path = os.path.join(output_dir, f"{name}_qr_code.png")
#     qr_img.save(qr_path)

# def read_qr_data(frame, private_key):
#     qr_codes = decode(frame)
#     if qr_codes:
#         for qr_code in qr_codes:
#             qr_data = qr_code.data.decode('utf-8')
#             print("QR Code detected")

#             # QR 코드 데이터 디코딩
#             encrypted_data = base64.b64decode(qr_data)

#             # 데이터 복호화
#             decrypted_data = private_key.decrypt(
#                 encrypted_data,
#                 padding.OAEP(
#                     mgf=padding.MGF1(algorithm=hashes.SHA256()),
#                     algorithm=hashes.SHA256(),
#                     label=None
#                 )
#             )

#             decrypted_data_str = decrypted_data.decode('utf-8')
#             print("Decrypted Data:", decrypted_data_str)



#     return decrypt_data(frame)
