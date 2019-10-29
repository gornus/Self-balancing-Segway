import pyb
from pyb import Pin, Timer, ADC, DAC, LED
import time
from oled_938 import OLED_938
from mpu6050 import MPU6050
from motor import MOTOR
from array import array
#from mic import MICROPHONE

'''initialisation of pins and class'''
imu = MPU6050(1, False)
robot = MOTOR()
pot = ADC(Pin('X11'))
trigger = pyb.Switch()
mic = ADC(Pin('Y11'))
MIC_OFFSET = 1523  # ADC reading of microphone for silence
b_LED = LED(4)  # flash for beats on blue LED

'''initialisation of oled'''
oled = OLED_938(pinout={'sda': 'Y10', 'scl': 'Y9', 'res': 'Y8'}, height=64,
                   external_vcc=False, i2c_devid=61)
oled.poweron()
oled.init_display()

import micropython
micropython.alloc_emergency_exception_buf(100)

'''initialisation of variables used for self balancing'''
last = 0
pitch = 0
total = 0
pError = 0
scale = 16.0
target = 0
ratio1 = 1
ratio2 = 1

'''initialisation of variables used for beat detection'''
N = 160  # size of sample buffer s_buf[]
s_buf = array('H', 0
for i in range(N))  # reserve buffer memory
ptr = 0  # sample buffer index pointer
buffer_full = False  # semaphore - ISR communicate with main program
# Define constants for main program loop - shown in UPPERCASE
M = 20  # number of instantaneous energy epochs to sum
BEAT_THRESHOLD = 2.0  # threshold for c to indicate a beat
SILENCE_THRESHOLD = 1.3  # threshold for c to indicate silence
# initialise variables for main program loop
e_ptr = 0  # pointer to energy buffer
e_buf = array('L', 0
for i in range(M))  # reserve storage for energy buffer
sum_energy = 0  # total energy in last 50 epochs

'''initialisation of motor'''
A1 = Pin('X3', Pin.OUT_PP)		# Control direction of motor A
A2 = Pin('X4', Pin.OUT_PP)
PWMA = Pin('X1')				# Control speed of motor A
B1 = Pin('X7', Pin.OUT_PP)		# Control direction of motor B
B2 = Pin('X8', Pin.OUT_PP)
PWMB = Pin('X2')				# Control speed of motor B
# Configure timer 2 to produce 1KHz clock for PWM control
tim = Timer(2, freq = 1000)
motorA = tim.channel (1, Timer.PWM, pin = PWMA)
motorB = tim.channel (2, Timer.PWM, pin = PWMB)
speed = 20

'''following functions are for beat detection'''
def flash():  # routine to flash blue LED when beat detected
    b_LED.on()
    pyb.delay(10)
    b_LED.off()


def energy(buf):  # Compute energy of signal in buffer
    sum = 0
    for i in range(len(buf)):
        s = buf[i] - MIC_OFFSET  # adjust sample to remove dc offset
        sum = sum + s * s  # accumulate sum of energy
    return sum

def isr_sampling(dummy):  # timer interrupt at 8kHz
    global ptr  # need to make ptr visible inside ISR
    global buffer_full  # need to make buffer_full inside ISR

    s_buf[ptr] = mic.read()  # take a sample every timer interrupt
    ptr += 1  # increment buffer pointer (index)
    if (ptr == N):  # wraparound ptr - goes 0 to N-1
        ptr = 0
        buffer_full = True  # set the flag (semaphore) for buffer full

pyb.disable_irq()			# disable interrupt while configuring timer
sample_timer = pyb.Timer(7, freq=8000)	# set timer 7 for 8kHz
sample_timer.callback(isr_sampling)		# specify interrupt service routine
pyb.enable_irq()			# enable interrupt again

Kp = 6.01
Ki = 79.95
Kd = 0.6

def PID(pitch, pitch_dot, setpoint, dt, total):
    pError = setpoint - pitch
    total = total + pError*dt
    w = Kp*pError - Kd*pitch_dot + Ki*total
    return (w, total)

total = 0
pError = 0
speed = 0.3
fall = False
tic1 = pyb.micros()
down = int(3)
pyb.delay(500)
w = 0

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

oled.draw_text(0, 30, 'Button pressed. Running.')
oled.display()

#reading our text file into an array:
f = open('texti.txt', 'r')
line = f.read().split()
counter = 0

pyb.delay(100)
tic2 = pyb.millis()  # mark time now in msec

trigger = pyb.Switch()
while not trigger():
    time.sleep(0.001)
    offset = pot.read()*1/4095 - 0.5
    oled.draw_text(0, 0, 'offset = {:5.2f}'.format(offset))
    oled.display()
while trigger(): pass

try:
    tic1 = pyb.micros()
    while True:
        dt_micro = pyb.micros() - tic1
        if (dt_micro >= 5000):
            dt = dt_micro * 0.000001
            pitch, pitch_dot = pitch_estimate(pitch, dt, 0.95)
            w, total = PID(pitch, pitch_dot, offset, dt, total)
            motordrive(ratio1*w, ratio2*w)
            tic1 = pyb.micros()
        if buffer_full:  # semaphore signal from ISR - set if buffer is full
            # Calculate instantaneous energy
            E = energy(s_buf)
            #print("buffer full")
            # compute moving sum of last 50 energy epochs
            sum_energy = sum_energy - e_buf[e_ptr] + E
            e_buf[e_ptr] = E  # over-write earliest energy with most recent
            e_ptr = (e_ptr + 1) % M  # increment e_ptr with wraparound - 0 to M-1
            # Compute ratio of instantaneous energy/average energy
            c = E * M / sum_energy
            '''print("c: {:2.f}".format(E*M/sum_energy))
            print("threshold: {:2.f}".format(BEAT_THRESHOLD))
            print("current: {:2.f}".format(E))
            print("last: {:2.f}".format(last*2))
            print("dt: {:2.f}".format(pyb.millis() - tic2))'''
            if (pyb.millis() - tic2 > 100):  # if more than 500ms since last beat
                if (c > BEAT_THRESHOLD) and E > last * 1.5 and E > 100000:  # look for a beat
                    print("beat detected")
                    print("beat {:d}".format(counter))
                    flash()  # beat found, flash blue LED
                    if line[counter] == 'F':
                        target = offset+speed
                        ratio1 = 1
                        ratio2 = 1
                        print("dancing F")
                    if line[counter] == 'B':
                        target = offset-speed
                        ratio1 = 1
                        ratio2 = 1
                        print("dancing B")
                    if line[counter] == 'C':
                        target = offset+speed
                        ratio1 = 1
                        ratio2 = 0.3
                        print("dancing C")
                    if line[counter] == 'A':
                        target = offset+speed
                        ratio1 = 0.3
                        ratio2 = 1
                        print("dancing A")
                    if line[counter] == 'P':
                        target = offset
                        ratio1 = 1
                        ratio2 = 1
                        print("dancing P")
                    counter = counter + 1
                    if counter == len(line):
                        counter = 0
                    #print("length of file {:d}".format(len(line)))
                    tic2 = pyb.millis()  # reset tic
            oled.display()
            buffer_full = False  # reset status flag
            last = E
        if w > 60 or w < -60:
            break

finally:
    print("terminated")
    robot.A_stop()
    robot.B_stop()