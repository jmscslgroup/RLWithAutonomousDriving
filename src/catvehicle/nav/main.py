from car_connection2 import car_connection2
import time
import rospy

control = car_connection2()

raw_input('Press enter to continue: ')

control.to_destination()

raw_input('Press enter to continue: ')

time.sleep(10)