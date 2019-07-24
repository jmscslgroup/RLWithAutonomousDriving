import rospy
from geometry_msgs.msg import Twist
import time

rospy.init_node('moving_car') 
cmd_vel_pub = rospy.Publisher('/catvehicle/cmd_vel', Twist, queue_size=10)

print(cmd_vel_pub.get_num_connections())

spd = Twist()

spd.linear.x = 1.0
spd.angular.z = 0.1

cmd_vel_pub.publish(spd)

time.sleep(10)