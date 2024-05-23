from navertts import NaverTTS
import playsound

# 메시지 내용을 입력받아 파일명으로 저장
def save(file_name, text): 
    tts = NaverTTS(text=text)
    tts.save(file_name)

# 파일 경로를 받아 파일 출력 (라즈베리파이는 컴퓨터 성능 문제로 2초정도 딜레이)
def play_audio(filename):
    playsound.playsound(filename)

# 음성 실행만 할 시 save 함수는 주석 처리하기
# save('see_you_next_time.mp3', '이용해 주셔서 감사합니다. 다음에 다시 방문해 주시기를 바랍니다!')
play_audio('../../data/tts_files/see_you_next_time.mp3')