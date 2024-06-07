import RPi.GPIO as GPIO
import time

class ServoController:
    def __init__(self, servo_pin=17, pwm_freq=50):
        self.servo_pin = servo_pin
        self.PWM_FREQ = pwm_freq

        # GPIO 모드 설정
        #   # 또는 GPIO.setmode(GPIO.BOARD)

        # GPIO 핀 설정
        try:
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.servo_pin, GPIO.OUT)
            self.pwm = GPIO.PWM(self.servo_pin, self.PWM_FREQ)
            self.pwm.start(0)
        except ValueError as e:
            print(f"Error in setting up GPIO pin: {e}")
            GPIO.cleanup()

    def set_angle(self, angle):
        try:
            duty = angle / 18.0 + 2.5
            self.pwm.ChangeDutyCycle(duty)
            time.sleep(1)  # 서보가 목표 각도로 이동할 시간을 줍니다.
            self.pwm.ChangeDutyCycle(0)  # 서보를 비활성화하여 과열을 방지합니다.
        except Exception as e:
            print(f"Error: {e}")
        # finally:
        #     self.cleanup()

    def cleanup(self):
        self.pwm.stop()

    # 추가로 GPIO 리소스를 정리하기 위해 cleanup() 메소드를 사용합니다.

if __name__ == "__main__":
    # 테스트 코드 (직접 실행 시 사용할 수 있음)
    
    servo_controller = ServoController()
    servo_controller.set_angle(90)  # 90도 각도로 서보 회전
    # time.sleep(2)
    # servo_controller.set_angle(0)   # 0도 각도로 서보 회전