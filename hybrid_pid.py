# Defines the PID class for control with position and force switching

import time
import motoron
from rotary_encoder import decoder
from filter import ButterworthFilter
import redis
import pigpio
import atomics
import yaml

motor_name = ['x', 'y', 'z']

class PID:
    def __init__(self, param_fname):
        # read yaml file
        with open(param_fname, 'r') as yaml_file:
            params = yaml.safe_load(yaml_file)

        # read last saved position offset
        with open(motor_name[params['axis']] + '.txt', 'r') as f:
            self.start_pos = float(f.readline().strip())
            print('Starting motor position: ' + str(self.start_pos))
            f.close()

        self.motor_id = params['axis']

        # position pid setup
        self.kp_pos = params['kp_pos']
        self.ki_pos = params['ki_pos']
        self.kd_pos = params['kd_pos']
        self.integral_position_error = 0
        self.integral_speed = 0
        self.curr_pos = self.start_pos
        self.prev_pos = self.curr_pos
        self.pos_des = self.curr_pos
        self.vel_des = 0
        self.error = 0
        self.prev_error = 0

        # force pi setup
        self.kp_force = params['kp_force']
        self.ki_force = params['ki_force']
        self.kd_force = params['kd_force']
        self.force_des = 0
        self.integral_force_error = 0

        # timing setup
        loop_freq = 100  # fixed parameter
        self.loop_time = (1. / loop_freq) * 1e9  # ns
        self.curr_time = 0
        self.prev_time = 0
        self.dt = 0
        self.remaining_dt = 0
        # self.motor_id = motor_id

        # filter and velocity setup
        self.vel = 0
        self.raw_vel = 0
        self.filter = ButterworthFilter(loop_freq, 0.1)  # 0.1 Hz cutoff frequency 

        # motor and encoder setup
        self.pi = pigpio.pi()
        self.enc_cnt = atomics.atomic(width=4, atype=atomics.INT)
        self.encoder = decoder(self.pi, params['enc_a'], params['enc_b'], self.callback)
        # self.sf = (1. / 7239) * 2.54  # encoder scale factor [cm]: (in / pulse) * (cm / in) 
        self.sf = params['enc_sf']
        self.speed = 0
        self.mc = motoron.MotoronI2C()
        self.initMotor()

        # redis setup
        self.redis_client = redis.Redis(host=params['host_name'], port=params['port_id'], db=0, decode_responses=True)
        self.pos_key = params['pos_key']
        self.force_key = params['force_key']
        self.pos_sense_key = params['pos_sense_key']
        self.force_sensor_key = params['force_sensor_key']
        self.redis_client.set(self.pos_key, str(self.start_pos))
        self.redis_client.set(self.pos_sense_key, str(self.start_pos))
        self.redis_client.set(self.force_key, '0')
        self.redis_client.set(self.force_sensor_key, '0')

        # limits
        self.max_vel = params['max_vel']
        self.vel_sat_mass = 10
        self.integral_sat = params['integral_sat']
        self.integral_thresh = params['integral_thresh']  # if error is within this percentage, then start integral action
        self.nu = 0
        self.max_pos = params['max_pos']
        self.min_pos = params['min_pos']
        self.max_force = params['max_force']
        self.min_force = params['min_force']
        self.pos_saturation_flag = False
        self.vel_saturation_flag = False

        # control flags
        self.state = 0  # controller starts at no control 
        self.state_request = 0
        self.state_request_key = params['state_request_key']
        self.state_key = params['state_key']
        self.redis_client.set(self.state_key, '0')
        self.redis_client.set(self.state_request_key, '0')
        self.pos_control_flag = False
        self.force_control_flag = False

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

        # get requested state
        self.getState()

        # get current position and/or force from sensors
        self.getPosition()
        self.getForce()

        # update velocity
        self.raw_vel = (self.curr_pos - self.prev_pos) / self.dt
        self.vel = self.getVelocity(self.raw_vel)
        # self.vel = self.filter.update(self.raw_vel)

        # Get desired position and/or force from redis 
        self.getDesiredPosition()
        self.getDesiredForce()

        # Position control
        if self.pos_control_flag:
            # Saturate desired position within limits 
            if self.pos_saturation_flag:
                if self.pos_des < self.min_pos:
                    self.pos_des = self.min_pos
                elif self.pos_des > self.max_pos:
                    self.pos_des = self.max_pos

            # Update integral error
            self.error = self.curr_pos - self.pos_des
            if (abs(self.error) < self.integral_thresh):
                self.updatePositionIntegralError(self.error)
            else:
                self.integral_position_error = 0

            # Integral zero crossing detection
            if (self.sign(self.prev_error) is not self.sign(self.error)):
                self.integral_position_error = 0

            self.prev_error = self.error

            # Compute PID loop
            if self.vel_saturation_flag:
                self.vel_des = (self.kp_pos / self.kd_pos) * (self.pos_des - self.curr_pos)
                if self.vel_des == 0:
                    self.speed = 0
                else:
                    self.nu = self.sat(self.max_vel / abs(self.vel_des))
                    self.speed = - self.kd_pos * (self.vel - self.nu * self.vel_des)
                    self.speed *= self.vel_sat_mass
            else:
                self.speed = -(self.kp_pos * (self.curr_pos - self.pos_des) + self.kd_pos * self.vel)
                self.integral_speed = self.ki_pos * self.integral_position_error
                if abs(self.integral_speed) > self.integral_sat:
                    self.integral_speed = self.sign(self.integral_speed) * self.integral_sat
                self.speed -= self.integral_speed
        elif self.force_control_flag:
            # Saturate desired force within limits 
            if self.force_des < self.min_force:
                self.force_des = self.min_force
            elif self.force_des > self.max_force:
                self.force_des = self.max_force

            # Update integral error
            self.updateForceIntegralError(self.curr_force - self.force_des)

            # Compute PI control
            self.speed = -(self.kp_force * (self.curr_force - self.force_des) + self.kd_force * self.vel)
            self.integral_speed = self.ki_force * self.integral_force_error
            if abs(self.integral_speed) > self.integral_sat:
                    self.integral_speed = self.sign(self.integral_speed) * self.integral_sat
            self.speed -= self.integral_speed
        else:
            self.speed = 0

        # Set speed 
        if (self.pos_control_flag or self.force_control_flag):
            if (abs(self.speed) > 400):
                self.speed = self.sign(self.speed) * 400
            elif (abs(self.speed) < 200):
                self.speed = self.sign(self.speed) * 200
            if self.motor_id == 0:
                self.mc.set_speed_now(1, int(self.speed))
            else:
                self.mc.set_speed_now(self.motor_id, int(self.speed))

        # Publish position information
        self.redis_client.set(self.pos_sense_key, str(self.curr_pos))

        # Wait for next loop
        self.prev_pos = self.curr_pos  # push forward 
        self.prev_time = self.curr_time  # push forward (called right after setting motor speed for accurate ZOH timing)
        self.waitUntilNextLoop()

    def updatePositionIntegralError(self, error):
        self.integral_position_error += error * self.dt

    def updateForceIntegralError(self, error):
        self.integral_force_error += error * self.dt

    def waitUntilNextLoop(self):
        self.remaining_dt = self.next_time - time.clock_gettime_ns(time.CLOCK_MONOTONIC)
        if self.remaining_dt > 0:
            # time.clock_nanosleep(self.remaining_dt)  # unix only 
            time.sleep(self.remaining_dt * 1e-9)

    def getDesiredPosition(self):
        self.pos_des = float(self.redis_client.get(self.pos_key))

    def getDesiredForce(self):
        self.force_des = float(self.redis_client.get(self.force_key))

    def getPosition(self):
        # self.curr_pos = self.encoder.readCounter() * self.sf 
        self.curr_pos = self.start_pos + self.enc_cnt.load() * self.sf

    def getForce(self):
        self.curr_force = float(self.redis_client.get(self.force_sensor_key))

    def getVelocity(self, vel):
        return self.filter.update(vel)

    def sign(self, x):
        if (x > 0):
            return 1
        elif (x < 0):
            return -1
        else:
            return 0

    def sat(self, x):
        if abs(x) < 1:
            return x
        else:
            return self.sign(x)

    def cancel(self):
        self.mc.set_speed_now(self.motor_id, 0)

    def enableForceControl(self):
        self.force_control_flag = True
        self.pos_control_flag = False
        self.pos_des = self.curr_pos
        self.force_des = 0
        self.integral_position_error = 0
        self.integral_force_error = 0
        self.redis_client.set(self.pos_key, str(self.pos_des))
        self.redis_client.set(self.force_key, str(self.force_des))

    def enablePositionControl(self):
        self.force_control_flag = False
        self.pos_control_flag = True
        self.pos_des = self.curr_pos
        self.force_des = 0
        self.integral_position_error = 0
        self.integral_force_error = 0
        self.redis_client.set(self.pos_key, str(self.pos_des))
        self.redis_client.set(self.force_key, str(self.force_des))

    def getState(self):
        self.state_request = self.redis_client.get(self.state_request_key)
        self.redis_client.set(self.state_request_key, '0')
        if self.state_request == '0':
            pass
        elif self.state_request == '1':
            self.state = 1
            self.redis_client.set(self.state_key, '1')
            print('Position Control')
            self.enablePositionControl()
        elif self.state_request == '2':
            self.state = 2
            self.redis_client.set(self.state_key, '2')
            print('Force Control')
            self.enableForceControl()
        else:
            print('Invalid state')
