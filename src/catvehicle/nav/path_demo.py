from car_connection import car_connection
import time
import rospy

control = car_connection()

raw_input('Press enter to continue: ')

#control.print_slam_odom()

control.to_destination()

raw_input('Press enter to continue: ')

time.sleep(10)