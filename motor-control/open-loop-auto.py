import motoron
import time
import sys

if __name__ == '__main__':
	mc = motoron.MotoronI2C()
	mc.reinitialize()
	mc.disable_crc()
	mc.clear_reset_flag()

	t_wait = 1  # duration for motion direction  
	direction = int(sys.argv[1])  # 1 or -1   

	last_time = time.time()

	while(1):
		time_duration = time.time() - last_time
		if (time_duration > t_wait):
			# direction = 0
			last_time = time.time()

		mc.set_speed_now(1, direction * 400)
