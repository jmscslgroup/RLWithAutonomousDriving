from start_sim import start_sim
import time
for i in range(2):
	sim = start_sim()
	sim.spawn()
	time.sleep(10)
	sim.signal_handler(2)
	time.sleep(5)

print("We are done now!!!!!!!!~~~~!!!! \n")


