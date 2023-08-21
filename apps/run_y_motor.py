import sys
sys.path.append('../')
from hybrid_pid import PID
from threading import Thread

import signal

motor_name = ['x', 'y', 'z']

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
   
   param_fname = 'y_motor.yaml'
   motor = PID(param_fname)

   cnt = 0
   motor.startClock()
   while True:
       signal.signal(signal.SIGINT, signal_handler)
       motor.computeSpeed()
       if cnt % 100 == 0:
           print('Position: ' + str(motor.curr_pos))
           print('Force: ' + str(motor.curr_force))
           print('Speed: ' + str(motor.speed))
           # print('Velocity: ' + str(motor.curr_vel))
           # print('Loop dt: ' + str(motor.dt))   
           # print('Integral position error: ' + str(motor.integral_position_error))
       cnt += 1
