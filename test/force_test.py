import sys
sys.path.append('../')
from hybrid_pid import PID
from threading import Thread

import signal

motor_name = ['y', 'z']

def signal_handler(sig, frame):
    global motor
    print('\nExit')
    print('Motor Position: ' + str(motor.curr_pos))
    print(motor.cancel())
    with open(motor_name[motor.motor_id] + '.txt', 'w') as f:
        f.write(str(motor.curr_pos))
        f.close()
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

   # motor = PID(kp, ki, kd, loop_freq, host_name, port_id, pos_str, enc_a, enc_b, motor_id, max_pos, min_pos)
   param_fname = 'y_motor.yaml'
   motor = PID(param_fname)

   cnt = 0
   while True:
       signal.signal(signal.SIGINT, signal_handler)
       motor.computeSpeed()
       if cnt % 10 == 0:
           print('Position: ' + str(motor.curr_pos))
           print('Force: ' + str(motor.curr_force))
           print('Speed: ' + str(motor.speed))
           # print('Velocity: ' + str(motor.curr_vel))
           print('Loop dt: ' + str(motor.dt))   
       cnt += 1
