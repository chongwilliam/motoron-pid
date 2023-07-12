import redis
import motoron
import time
from HX711 import *

if __name__ == '__main__':
	mc = motoron.MotoronI2C()
	mc.reinitialize()
	mc.disable_crc()
	mc.clear_reset_flag()

	r = redis.Redis(decode_responses=True)
	r.set('OPEN_LOOP_DIR_0', 0)
	r.set('OPEN_LOOP_ENABLE_0', 0)
	r.set('OPEN_LOOP_DIR_1', 0)
	r.set('OPEN_LOOP_ENABLE_1', 0)

	pipe = r.pipeline()

	# Load cell setup
	hx = SimpleHX711(20, 21, -101, -2194, Rate.HZ_80)
	hx.setUnit(Mass.Unit.G)
	hx.zero()
	force_tol = 5  # grams of force tolerance to stop

	while(1):
		pipe.get('OPEN_LOOP_DIR_0')
		pipe.get('OPEN_LOOP_ENABLE_0')
		pipe.get('OPEN_LOOP_DIR_1')
		pipe.get('OPEN_LOOP_ENABLE_1')
		flags = pipe.execute()

		# Get load cell reading
		force = float(hx.weight(1))

		if (force > force_tol and int(flags[0]) == 1):
			mc.set_speed_now(1, 0)
		elif (force < -force_tol and int(flags[0]) == -1):
			mc.set_speed_now(1, 0)
		else:
			if (flags[1] == '1'):
				mc.set_speed_now(1, int(flags[0]) * 400)
			else:
				mc.set_speed_now(1, 0)
			if (flags[3] == '1'):
				mc.set_speed_now(2, int(flags[2]) * 400)
			else:
				mc.set_speed_now(2, 0)
