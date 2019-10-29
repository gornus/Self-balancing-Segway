import pyb, time
from pyb import Pin, Timer, ADC, UART
from oled_938 import OLED_938
from mpu6050 import MPU6050
from motor import MOTOR

A1 = Pin('X3', Pin.OUT_PP)		# Control direction of motor A
A2 = Pin('X4', Pin.OUT_PP)
PWMA = Pin('X1')				# Control speed of motor A

B1 = Pin('X7', Pin.OUT_PP)		# Control direction of motor B
B2 = Pin('X8', Pin.OUT_PP)
PWMB = Pin('X2')				# Control speed of motor B

tim = Timer(2, freq = 1000)
motorA = tim.channel (1, Timer.PWM, pin = PWMA)
motorB = tim.channel (2, Timer.PWM, pin = PWMB)

pot = pyb.ADC(Pin('X11'))
uart = UART(6)
uart.init(9600, bits=8, parity = None, stop = 2)

imu = MPU6050(1, False)
pitch = 0

robot = MOTOR()

oled = OLED_938(pinout={'sda': 'Y10', 'scl': 'Y9', 'res': 'Y8'}, height=64,
                   external_vcc=False, i2c_devid=61)
oled.poweron()
oled.init_display()


'''Pitch angle calculation using complementary filter'''
def pitch_estimate(pitch, dt, alpha):
  theta = imu.pitch()
  pitch_dot = imu.get_gy()
  pitch = alpha*(pitch + pitch_dot*dt) + (1- alpha)*theta
  return (pitch, pitch_dot)

def motordrive(w1, w2):
    robot.Aspeed = w1
    robot.Bspeed = w2
    robot.drive()

#Tuning the PID controller - using potentiometer and USR switch to adjust the gain values live
trigger = pyb.Switch()
scale = 20.0
while not trigger():
    time.sleep(0.001)
    offset = pot.read()/4095-0.5
    oled.draw_text(0, 30, 'offset = {:5.2f}'.format(offset))
    oled.display()


oled.draw_text(0, 20, 'Button pressed. Running.')
oled.display()

def PID(pitch, pitch_dot, setpoint, pError, dt, total):
    pError = setpoint - pitch
    total = total + pError*dt
    w = Kp*pError - Kd*pitch_dot + Ki*total
    return (w, total)

total = 0
pError = 0
speed = 0.2
tic1 = pyb.micros()
Kp = 6
Ki = 80
Kd = 0.6
target = offset
ratio1 = 1
ratio2 = 1
while True:
    dt_micro = pyb.micros() - tic1
    if (dt_micro >= 5000):
        dt = dt_micro*0.000001
        pitch, pitch_dot = pitch_estimate(pitch, dt, 0.95)
        w, total = PID(pitch, pitch_dot, target, pError, dt, total)
        motordrive(w*ratio1, w*ratio2)
        tic1 = pyb.micros()
    elif (uart.any()==5):    # wait for 10 chars
        command = uart.read(5)
        if command[2]==ord('5'):
            ratio1 = 1
            ratio2 = 1
            target = offset + speed
        elif command[2]==ord('6'):
            ratio1 = 1
            ratio2 = 1
            target = offset - speed
        elif command[2]==ord('7'):
            ratio1 = 0.5
            ratio2 = 1
            target = offset + speed
        elif command[2]==ord('8'):
            ratio1 = 1
            ratio2 = 0.5
            target = offset + speed
        elif command[2]==ord('1'):
            ratio1 = 1
            ratio2 = 1
            target = offset
        else:
            ratio1 = 1
            ratio2 = 1
            target = offset
