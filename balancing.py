from mpu6050 import MPU6050
imu = MPU6050(1, False)    	# Use I2C port 1 on Pyboard

def pitch_estimation(pitch, dt, alpha):
    theta = imu.pitch()
    pitch_dot = imu.get_gy()
    pitch = alpha * (pitch + pitch_dot * dt * 0.001) + (1 - alpha) * theta
    return (pitch, pitch_dot)

def PID(target, pitch, dt, total, Kp, Ki, Kd, alpha):
    current, change = pitch_estimation(pitch, dt, alpha)
    error = target - current
    total = total + error
    P = error*Kp
    I = total*Ki
    D = change*Kd
    w = P+I+D
    return w, current, total