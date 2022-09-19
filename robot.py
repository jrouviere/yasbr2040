import math
import time

from machine import Timer
from pimoroni import PID
from imu import to_radsec
from ibus_rx import normalise

class Robot:
    ODO_PERIOD = 60 # in milliseconds
    PRINT_DIVIDER = 1000
    MAX_TT = 0.20 # maximum angle correction by the position loop
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
        self.alpha_pid  = PID(1.50, 0.0, 0.10, self.ODO_PERIOD/1000)
        self.pos_pid    = PID(0.20, 0.0, 0.30, self.ODO_PERIOD/1000)
        self.theta_pid  = PID(5.50, 0.1, 0.05, self.UPDATE_PER)
        
        # pos and angles
        self.alpha = 0.0
        self.pos = 0.0
        self.theta = self.THETA_OFF

        # controls
        self.target_alpha = 0.0
        self.target_pos = 0.0
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
            self.target_alpha -= normalise(input[0]) / 500
            self.target_pos -= normalise(input[2]) / 500

    def update_odometry(self):
        # measured linear and rotational movements
        c1 = self.encoders[0].capture()
        c2 = self.encoders[1].capture()
        self.alpha += (c1.delta - c2.delta) / 2000.0 
        self.pos += (c1.delta + c2.delta) / 2000.0

        self.pos_pid.setpoint = self.target_pos
        self.target_theta = clamp(-self.pos_pid.calculate(self.pos), self.MAX_TT)

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
        print("- TP:", self.target_pos, "POS:", self.pos, "T:", self.theta, "delay:", self.delay)

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
            if abs(self.pos-self.target_pos)>0.5 or abs(self.theta)>0.8:
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
