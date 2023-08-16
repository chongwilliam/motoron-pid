# Defines the PID class for control

import time
import motoron
from rotary_encoder import decoder
from filter import ButterworthFilter
import redis
import pigpio
import atomics

class PID:
    def __init__(self, kp, ki, kd, loop_freq, host_name, port_id, pos_str, enc_a, enc_b, motor_id=1, start_pos=0, min_pos=100, max_pos=-100):
        # pid setup
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_error = 0
        self.integral_speed = 0
        self.curr_pos = start_pos
        self.prev_pos = self.curr_pos
        self.pos_des = self.curr_pos
        self.vel_des = 0

        # timing setup
        self.loop_time = (1. / loop_freq) * 1e9  # ns
        self.curr_time = 0
        self.prev_time = 0
        self.dt = 0
        self.remaining_dt = 0
        self.motor_id = motor_id

        # filter and velocity setup
        self.vel = 0
        self.raw_vel = 0
        self.filter = ButterworthFilter(loop_freq, 0.1)  # 0.1 Hz cutoff frequency 

        # motor and encoder setup
        self.pi = pigpio.pi()
        self.enc_cnt = atomics.atomic(width=4, atype=atomics.INT)
        self.encoder = decoder(self.pi, enc_a, enc_b, self.callback)
        self.sf = (1. / 7239) * 2.54  # encoder scale factor [cm]: (in / pulse) * (cm / in) 
        self.speed = 0
        self.mc = motoron.MotoronI2C()
        self.initMotor()

        # redis setup
        self.redis_client = redis.Redis(host=host_name, port=port_id, db=0, decode_responses=True)
        self.pos_str = pos_str
        self.redis_client.set(self.pos_str, '0')

        # limits
        self.max_vel = 1e1
        self.vel_sat_mass = 10  
        self.integral_sat = 50
        self.nu = 0
        self.max_pos = max_pos
        self.min_pos = min_pos
        self.pos_saturation_flag = False
        self.vel_saturation_flag = False

    def callback(self, way):
        if way == 1:
            self.enc_cnt.inc()
        elif way == -1:
            self.enc_cnt.dec()

    def initMotor(self):
        self.mc.reinitialize()
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
        self.vel = self.filter.update(self.raw_vel)

        # Get desired position from redis 
        self.getDesiredPosition()

        # Saturate desired position within limits 
        if self.pos_saturation_flag:
            if self.pos_des < self.min_pos:
                self.pos_des = self.min_pos
            elif self.pos_des > self.max_pos:
                self.pos_des = self.max_pos

        # Update integral error
        self.updateIntegralError(self.curr_pos - self.pos_des)

        # Compute PID loop
        if self.vel_saturation_flag:
            self.vel_des = (self.kp / self.kd) * (self.pos_des - self.curr_pos)
            if self.vel_des == 0:
                self.speed = 0
            else:
                self.nu = self.sat(self.max_vel / abs(self.vel_des))
                self.speed = - self.kd * (self.vel - self.nu * self.vel_des)
                self.speed *= self.vel_sat_mass
        else:
            self.speed = -(self.kp * (self.curr_pos - self.pos_des) + self.kd * self.vel)
            self.integral_speed = self.ki * self.integral_error
            if abs(self.integral_speed) > self.integral_sat:
                self.integral_speed = self.sign(self.integral_speed) * self.integral_sat
            self.speed -= self.integral_speed

        # Set speed 
        self.mc.set_speed_now(self.motor_id, int(self.speed))

        # Wait for next loop
        self.prev_pos = self.curr_pos  # push forward 
        self.prev_time = self.curr_time  # push forward (called right after setting motor speed for accurate ZOH timing)
        self.waitUntilNextLoop()

    def updateIntegralError(self, error):
        self.integral_error += error * self.dt;

    def waitUntilNextLoop(self):
        self.remaining_dt = self.next_time - time.clock_gettime_ns(time.CLOCK_MONOTONIC)
        if self.remaining_dt > 0:
            # time.clock_nanosleep(self.remaining_dt)  # unix only 
            time.sleep(self.remaining_dt * 1e-9)

    def getDesiredPosition(self):
        self.pos_des = float(self.redis_client.get(self.pos_str))

    def getPosition(self):
        # self.curr_pos = self.encoder.readCounter() * self.sf 
        self.curr_pos = self.enc_cnt.load() * self.sf

    def getVelocity(self, vel):
        return self.filter.update(vel)

    def sign(self, x):
        return (x > 0) - (x < 0)

    def sat(self, x):
        if abs(x) < 1:
            return x
        else:
            return self.sign(x)

    def cancel(self):
        self.mc.set_speed_now(self.motor_id, 0)
