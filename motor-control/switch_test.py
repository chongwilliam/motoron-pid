import pigpio
import time

pi = pigpio.pi()
pin_id = 17

# set pin to read
pi.set_pull_up_down(pin_id, pigpio.PUD_DOWN)
pi.set_mode(pin_id, pigpio.INPUT)

# while loop to read
while(1):
	if (pi.read(pin_id) == 1):
		print("High")
	else:
		print("Low")
	time.sleep(0.1)





