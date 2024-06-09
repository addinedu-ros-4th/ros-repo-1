from navertts import NaverTTS
import playsound
import os

class TTSService():
    # 메시지 내용을 입력받아 파일명으로 저장
    def create_tts_file(self, file_name, text): 
        tts = NaverTTS(text=text)
        tts.save(file_name)
    # 파일 경로를 받아 파일 출력 (라즈베리파이는 컴퓨터 성능 문제로 2초정도 딜레이)
    def speak(self, filename):
        playsound.playsound(filename)

