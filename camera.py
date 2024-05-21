# https://uigwonblog.tistory.com/25
import cv2
import numpy as np
import RPi.GPIO as GPIO

# 모터 상태
STOP  = 0
FORWARD  = 1
BACKWORD = 2

# 모터 채널
CH1 = 0
CH2 = 1

# PIN 입출력 설정
OUTPUT = 1
INPUT = 0

# PIN 설정
HIGH = 1
LOW = 0

# 실제 핀 정의
#PWM PIN
ENA = 26  #37 pin
ENB = 0   #27 pin

# GPIO PIN
IN1 = 19  #37 pin
IN2 = 13  #35 pin
IN3 = 6   #31 pin
IN4 = 5   #29 pin

def set_pin(EN, INA, INB) -> GPIO.PWM:        
    GPIO.setup(EN, GPIO.OUT)
    GPIO.setup(INA, GPIO.OUT)
    GPIO.setup(INB, GPIO.OUT)
    
    # 100khz 로 PWM 동작 시킴 
    pwm = GPIO.PWM(EN, 100) 
    
    # 우선 PWM 멈춤.   
    pwm.start(0) 
    return pwm

def motor_control(pwm, INA, INB, speed, stat) -> None:
    pwm.ChangeDutyCycle(speed)  
    
    if stat == FORWARD:
        GPIO.output(INA, HIGH)
        GPIO.output(INB, LOW)
    elif stat == BACKWORD:
        GPIO.output(INA, LOW)
        GPIO.output(INB, HIGH)
    elif stat == STOP:
        GPIO.output(INA, LOW)
        GPIO.output(INB, LOW)

pwmA = set_pin(ENA, IN1, IN2)
pwmB = set_pin(ENB, IN3, IN4)

def set_motor(ch, speed, stat):
    if ch == CH1:
        # pwmA는 핀 설정 후 pwm 핸들을 리턴 받은 값이다.
        motor_control(pwmA, IN1, IN2, speed, stat)
    else:
        # pwmB는 핀 설정 후 pwm 핸들을 리턴 받은 값이다.
        motor_control(pwmB, IN3, IN4, speed, stat)

face_cascade = cv2.CascadeClassifier('cascades/haarcascade_frontalface_default.xml')
cap = cv2.VideoCapture(0)

frame_width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
frame_height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)

while True:
    ret, frame = cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    faces: np.ndarray = face_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))
    
    if len(faces) > 0:
        areas = faces[:, 2] * faces[:, 3]
        sorted_faces = faces[np.argsort(-areas)]
    else:
        sorted_faces = np.zeros((0, 4))
    # print(sorted_faces)

    for (x, y, w, h) in sorted_faces:
        cv2.putText(frame, f"{x=}, {y=}, {w=}, {h=}", (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
        cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)

    if len(sorted_faces) > 0:
        x, _, w, _ = sorted_faces[0]
        center_x = x + w // 2
        
        if center_x < frame_width / 3:
            set_motor(CH1, 80, FORWARD)
            print('left')
        elif center_x > 2 * frame_width / 3:
            set_motor(CH1, 80, BACKWORD)
            print('right')

    cv2.imshow('Faces', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
