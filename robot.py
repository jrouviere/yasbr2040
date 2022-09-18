import math
import time

from pimoroni import PID
from imu import IMU
from ibus_rx import IBus

class Robot:
    SUB_SAMPLING = 24
    PRINT_DIVIDER = 1000
    MAX_TT = 0.12 # maximum angle correction by the position loop
    THETA_OFF = -0.26 # stable angle offset


    def __init__(self, imu, ibus, motors, encoders):
        assert len(motors) == 2
        assert len(encoders) == 2

        self.imu = imu
        self.ibus = ibus
        self.motors = motors
        self.encoders = encoders

        self.UPDATE_PER = 1 / imu.RATE
        # PID settings
        self.alpha_pid  = PID(1.50, 0.0, 0.10, self.UPDATE_PER*self.SUB_SAMPLING)
        self.pos_pid    = PID(0.20, 0.0, 0.25, self.UPDATE_PER*self.SUB_SAMPLING)
        self.theta_pid  = PID(5.50, 0.1, 0.05, self.UPDATE_PER)


    def run(self):
        alpha = 0
        theta = self.THETA_OFF
        ticks = 0
        pos = 0
        start = time.ticks_us()

        while True:
            # update odometry            

            # downsample position pid loop
            if ticks % self.SUB_SAMPLING == 0:
                # measured linear and rotational movements
                c1 = self.encoders[0].capture()
                c2 = self.encoders[1].capture()
                alpha += (c1.delta - c2.delta) / 2000.0
                pos += (c1.delta + c2.delta) / 2000.0
                self.pos_pid.setpoint = 0.0
                
                target_theta = clamp(-self.pos_pid.calculate(pos), self.MAX_TT)

                self.alpha_pid.setpoint = 0.0
                alpha_cor = self.alpha_pid.calculate(alpha)

            
            # update angle            
            gyro, acc = self.imu.read()       
            dt = gyro[0]
            ang = -math.atan2(acc[2], acc[1])
            theta = 0.99*(theta+dt*self.UPDATE_PER) + 0.01*ang
            

            # inclination angle we are aiming for
            self.theta_pid.setpoint = target_theta+self.THETA_OFF
            speed_out = self.theta_pid.calculate(theta)

            # Set the new motor driving speed
            self.motors[0].speed(speed_out + alpha_cor / 2.0)
            self.motors[1].speed(speed_out - alpha_cor / 2.0)

            # failsafe
            if abs(pos)>1.0 or abs(theta)>0.8:
                raise BaseException("failsafe")

            
            # timer management
            ticks += 1
                
            now = time.ticks_us()
            delay = (ticks * 1e6 // self.imu.RATE) - time.ticks_diff(now, start)
            time.sleep_us(int(delay))

            if ticks == 10000:
                ticks = 0
                start = time.ticks_us()

            # DBG
            if ticks % self.PRINT_DIVIDER == 0:
                print("- ALPHA:", alpha, "POS:", pos, "TT:", target_theta, "T:", theta, "delay:", delay)


def clamp(val, vmax):
    if val < -vmax:
        return -vmax
    if val > vmax:
        return vmax
    return val
