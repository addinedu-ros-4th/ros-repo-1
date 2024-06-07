import sys
from PyQt5.QtWidgets import QApplication, QLabel
from PyQt5.QtGui import QPixmap
import resource_rc  # 생성된 resource_rc.py 파일을 import

app = QApplication(sys.argv)

label = QLabel()

# 문제의 이미지 경로
image_path = "./src/delivery_ui/delivery_ui/data/check.png"

pixmap = QPixmap(image_path)
if pixmap.isNull():
    print(f"이미지를 불러올 수 없습니다: {image_path}")
else:
    print(f"이미지를 성공적으로 불러왔습니다: {image_path}")
    label.setPixmap(pixmap)

label.show()

sys.exit(app.exec_())