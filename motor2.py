import RPi.GPIO as GPIO
from time import sleep

#  
STOP  = 0
FORWARD  = 1
BACKWORD = 2

#  
CH1 = 0
CH2 = 1

# PIN  
OUTPUT = 1
INPUT = 0

# PIN 
HIGH = 1
LOW = 0

#   
#PWM PIN
ENA = 26  #37 pin
ENB = 0   #27 pin

#GPIO PIN
IN1 = 19  #37 pin
IN2 = 13  #35 pin
IN3 = 6   #31 pin
IN4 = 5   #29 pin

#   
def set_pin(EN, INA, INB) -> GPIO.PWM:        
    GPIO.setup(EN, GPIO.OUT)
    GPIO.setup(INA, GPIO.OUT)
    GPIO.setup(INB, GPIO.OUT)
    
    # 100khz  PWM   
    pwm = GPIO.PWM(EN, 100) 
    
    #  PWM .   
    pwm.start(0) 
    return pwm

#   
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
        
#         
def set_motor(ch, speed, stat):
    if ch == CH1:
        # pwmA    pwm    .
        motor_control(pwmA, IN1, IN2, speed, stat)
    else:
        # pwmB    pwm    .
        motor_control(pwmB, IN3, IN4, speed, stat)
  
if __name__ == "__main__":
    GPIO.setmode(GPIO.BCM)
    pwmA = set_pin(ENA, IN1, IN2)
    pwmB = set_pin(ENB, IN3, IN4)
    
    try:
        while True:
            n = int(input('angle: '))
            
            if n > 0:
                set_motor(CH1, 10, FORWARD)
                set_motor(CH2, 10, FORWARD)
            else:
                set_motor(CH1, 10, BACKWORD)
                set_motor(CH2, 10, BACKWORD)
            sleep(2)
            pwmA.ChangeDutyCycle(0)
    except:
        GPIO.cleanup()
