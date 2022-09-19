import math
import time

from machine import Timer
from pimoroni import PID
from imu import to_radsec
from ibus_rx import normalise

class Robot:
    ODO_PERIOD = 60 # in milliseconds
    MAX_TT = 0.25 # maximum angle correction by the position loop
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
        self.alpha_pid  = PID(4.00, 0.0, 0.10, self.ODO_PERIOD/1000)
        self.speed_pid  = PID(5.00, 0.3, 0.02, self.ODO_PERIOD/1000)
        self.theta_pid  = PID(5.50, 0.1, 0.05, self.UPDATE_PER)
        
        # speed and angles
        self.alpha = 0.0
        self.speed = 0.0
        self.theta = self.THETA_OFF

        # controls
        self.target_alpha = 0.0
        self.target_speed = 0.0
        self.target_theta = 0.0
        self.alpha_cor = 0.0


    def update_imu(self):
        gyro, acc = self.imu.read()
        dt = to_radsec(gyro[0])
        ang = -math.atan2(acc[2], acc[1])
        # fuse accelerometer and gyroscope mesurements
        self.theta = 0.99*(self.theta+dt*self.UPDATE_PER) + 0.01*ang

    def update_cmd(self):
        input = self.ibus.read_raw()
        if input:
            # assuming channels are in AETR order
            alpha_cmd = normalise(input[0])
            self.target_alpha -= deadband(alpha_cmd, 0.05) / 1000

            sp_cmd = normalise(input[2])
            self.target_speed = -deadband(sp_cmd, 0.05) / 20

    def update_odometry(self):
        # measured linear and rotational movements
        c1 = self.encoders[0].capture()
        c2 = self.encoders[1].capture()
        self.alpha += (c1.delta - c2.delta) / 2000.0 
        self.speed = (c1.delta + c2.delta) / 2000.0

        self.speed_pid.setpoint = self.target_speed
        self.target_theta = clamp(-self.speed_pid.calculate(self.speed), self.MAX_TT)

        self.alpha_pid.setpoint = self.target_alpha
        self.alpha_cor = self.alpha_pid.calculate(self.alpha)

    def update_speed(self):
        # inclination angle we are aiming for
        self.theta_pid.setpoint = self.target_theta+self.THETA_OFF
        speed_out = self.theta_pid.calculate(self.theta)

        # Set the new motor driving speed
        self.motors[0].speed(speed_out + self.alpha_cor / 2.0)
        self.motors[1].speed(speed_out - self.alpha_cor / 2.0)
        

    def log_debug(self):
        print("- TSP:", self.target_speed, "SP:", self.speed, "TT:", self.target_theta, "delay:", self.delay)

    def set_odo(self, t):
        self.run_odo = True
    def set_debug(self, t):
        self.run_debug = True
    def set_cmd(self, t):
        self.run_cmd = True

    def run(self):
        self.run_odo = True
        self.run_debug = False
        self.run_cmd = True
        

        t0 = Timer(period=self.ODO_PERIOD, mode=Timer.PERIODIC, callback=self.set_odo)
        t1 = Timer(period=1000, mode=Timer.PERIODIC, callback=self.set_debug)
        t2 = Timer(period=7, mode=Timer.PERIODIC, callback=self.set_cmd)


        ticks = 0
        start = time.ticks_us()

        while True:
            if self.run_cmd:
                self.update_cmd()
                self.run_cmd=False
            
            if self.run_odo:
                self.update_odometry()
                self.run_odo=False
            
            self.update_imu()
            self.update_speed()

            # failsafe
            if  abs(self.speed)>0.2 or abs(self.theta)>0.8:
                raise BaseException("failsafe")

            if self.run_debug:
                self.log_debug()
                self.run_debug=False

            # timer management
            ticks += 1
            now = time.ticks_us()
            self.delay = (ticks * 1e6 // self.imu.RATE) - time.ticks_diff(now, start)
            time.sleep_us(int(self.delay))


def clamp(val, vmax):
    if val < -vmax:
        return -vmax
    if val > vmax:
        return vmax
    return val

def deadband(val, band):
    if abs(val) < band:
        return 0
    return val
