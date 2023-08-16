import sys
sys.path.append('../')
from pid import PID
from threading import Thread

import signal

def signal_handler(sig, frame):
    global motor
    print('Exit')
    print(motor.curr_pos)
    print(motor.cancel())
    sys.exit(0)

if __name__ == '__main__':
   kp = 100
   ki = 10
   kd = 20
   loop_freq = 100
   host_name = 'localhost'
   port_id = 6379
   pos_str = 'right_z_des'
   enc_a = 20
   enc_b = 21
   motor_id = 1
   max_pos = 100
   min_pos = -100

   motor = PID(kp, ki, kd, loop_freq, host_name, port_id, pos_str, enc_a, enc_b, motor_id, max_pos, min_pos)

   while True:
       signal.signal(signal.SIGINT, signal_handler)
       motor.computeSpeed()
       print('Position: ' + str(motor.curr_pos))
       print('Velocity: ' + str(motor.vel))
       # print('loop time: ')
      #  print(motor.dt)

