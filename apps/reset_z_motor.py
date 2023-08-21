import motoron
import time
import sys
import signal

def signal_handler(sig, frame):
    global mc
    mc.set_speed_now(2, 0)
    sys.exit(0)

if __name__ == '__main__':
    mc = motoron.MotoronI2C()
    mc.reinitialize()
    mc.disable_crc()
    mc.clear_reset_flag()

    signal.signal(signal.SIGINT, signal_handler)

    t_wait = 1  # duration for motion direction  
    direction = int(sys.argv[1])  # 1 or -1

    f = open('z.txt', 'w')
    f.write('0')
    f.close()

    last_time = time.time()

    while(1):
        time_duration = time.time() - last_time
        if (time_duration > t_wait):
            # direction *= -1
            last_time = time.time()
            # break

        mc.set_speed_now(2, direction * 400)
