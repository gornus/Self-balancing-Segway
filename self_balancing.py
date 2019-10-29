import pyb
import time
from oled_938 import OLED_938
from mpu6050 import MPU6050
from motor import MOTOR

imu = MPU6050(1, False)
pitch = 0

robot = MOTOR()
pot = pyb.ADC(Pin('X11'))

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

def motordrive(w):
    robot.Aspeed = w
    robot.Bspeed = w
    robot.drive()

#Tuning the PID controller - using potentiometer and USR switch to adjust the gain values live
trigger = pyb.Switch()
while not trigger():
    time.sleep(0.001)
    offset = pot.read()*2/4095 - 1
    oled.draw_text(0, 0, 'offset = {:5.2f}'.format(offset))
    oled.display()
while trigger(): pass
'''while not trigger():
    time.sleep(0.001)
    Kp = pot.read()*8/4095
    oled.draw_text(0, 10, 'Kp = {:5.2f}'.format(Kp))
    oled.display()
while trigger(): pass
while not trigger():
    time.sleep(0.001)
    Kd = pot.read()*5/4095
    oled.draw_text(0, 20, 'Kd = {:5.2f}'.format(Kd))
    oled.display()
while trigger(): pass
while not trigger():
    time.sleep(0.001)
    Ki = pot.read()*100/4095
    oled.draw_text(0,30, 'Ki = {:5.2f}'.format(Ki))
    oled.display()
while trigger(): pass'''
Kp = 6.01
Ki = 79.95
Kd = 0.6

oled.draw_text(0, 40, 'Button pressed. Running.')
oled.display()

def PID(pitch, pitch_dot, setpoint, dt, total):
    pError = setpoint - pitch
    total = total + pError*dt
    #print("pitch: {:2.f}".format(pitch))
    #print("error: {:2.f}".format(pError))
    #print("p dot: {:2.f}".format(pitch_dot))
    #print("integral: {:2.f}".format(total))
    w = Kp*pError - Kd*pitch_dot + Ki*total
    #print("w: {:2.f}".format(w))
    return (w, total)

total = 0
pError = 0
fall = False
tic1 = pyb.micros()
down = int(3)
pyb.delay(500)
'''while down>0:
    oled.draw_text(0,40, 'zeroing in: {:}'.format(down))
    oled.display()
    down = down - 1
    pyb.delay(1000)
zero, dummy = pitch_estimate(pitch, 0.2, 0.95)
oled.draw_text(0,40, 'zero: {:.2f}'.format(zero))
oled.display()
'''

while True:
    dt_micro = pyb.micros() - tic1
    if (dt_micro >= 5000):
        dt = dt_micro*0.000001
        pitch, pitch_dot = pitch_estimate(pitch, dt, 0.95)
        w, total = PID(pitch, pitch_dot, offset, dt, total)
        motordrive(w)
        tic1 = pyb.micros()
