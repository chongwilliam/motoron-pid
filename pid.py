# Defines the PID class for control

import time
import motoron 
from encoder import LS7366R
from filter import ButterworthFilter
import redis 

class PID:
    def __init__(self, kp, ki, kd, loop_time, host_name, port_id, pos_str):
        # pid setup
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.loop_time = loop_time * 1e9  # ns
        self.integral_error = 0
        self.curr_time = 0
        self.prev_time = 0
        self.curr_pos = 0
        self.prev_pos = 0
        self.pos_des = 0
        self.dt = 0
        self.remaining_dt = 0

        # filter and velocity setup
        self.vel = 0
        self.raw_vel = 0
        self.filter = ButterworthFilter(100, 0.1)  # 100 Hz at 0.1 Hz cutoff frequency 

        # motor and encoder setup
        self.encoder = LS7366R(0, 1000000, 4)
        self.sf = (1. / 7239) * 2.54  # encoder scale factor [cm]: (in / pulse) * (cm / in) 
        self.speed = 0
        self.mc = motoron.MotoronI2C()
        self.initMotor()

        # redis setup
        self.redis_client = redis.Redis(host = host_name, port = port_id, db = 0, decode_responses = True)
        self.pos_str = pos_str

    def initMotor(self):
        self.mc.reInitialize()
        self.mc.disable_crc()
        self.mc.clear_reset_flag()

    def startClock(self):
        # self.curr_time = time.time_ns()
        self.curr_time = time.clock_gettime_ns(time.CLOCK_MONOTONIC)
        self.prev_time = self.curr_time

    def computeSpeed(self):
        # self.curr_time = time.time_ns()
        self.curr_time = time.clock_gettime_ns(time.CLOCK_MONOTONIC)
        self.next_time = self.curr_time + self.loop_time
        self.dt = (self.curr_time - self.prev_time) * 1e-9  # ns to s for velocity computation 

        # get current position from encoders
        self.getPosition()

        # update velocity
        self.raw_vel = (self.curr_pos - self.prev_pos) / self.dt
        self.getVelocity(self.raw_vel)

        # Get desired position from redis 
        self.getDesiredPosition()

        # Update integral error
        self.updateIntegralError(self.curr_pos - self.prev_pos)

        # Compute PID loop
        self.speed = self.kp * (self.curr_pos - self.pos_des) + self.kd * self.vel + self.ki * (self.integral_error)

        # Set speed 
        self.mc.set_speed_now(self.speed)

        # Wait for next loop
        self.prev_pos = self.curr_pos  # push forward 
        self.prev_time = self.curr_time  # push forward (called right after setting motor speed for accurate ZOH timing)
        self.waitUntilNextLoop()

    def updateIntegralError(self, error):
        self.integral_error += error * self.dt;  
    
    def waitUntilNextLoop(self):
        self.remaining_dt = self.next_time - time.clock_gettime_ns(time.CLOCK_MONOTONIC)
        if self.remaining_dt > 0:
            time.clock_nanosleep(self.remaining_dt)  # unix only 

    def getDesiredPosition(self):
        self.pos_des = float(self.redis_client.get(self.pos_str))

    def getPosition(self):
        self.curr_pos = self.encoder.readCounter() * self.sf 
    
    def getVelocity(self, vel):
        return self.filter.update(vel)

