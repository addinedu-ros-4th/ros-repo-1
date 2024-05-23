import speech_recognition as sr

def my_stt() :
    r = sr.Recognizer()
    with sr.Microphone() as source:
        print("말씀 하세요 : ")
        audio = r.listen(source)
    
    mySpeech = r.recognize_google(audio, language='ko', show_all=True)
    try :
        return mySpeech
        
    except sr.UnknownValueError:
        print("Google 음성 인식이 오디오를 이해할 수 없습니다.")
    except sr.RequestError as e:
        print("Google 음성 인식 서비스에서 결과를 요청할 수 없습니다.; {0}".format(e))

while True :
    my_speech = my_stt()
    if my_speech == "종료" :
        break
    else :
        print(my_speech)