from __future__ import division
import rospy
import sys, math, time
import numpy as np
from std_msgs.msg import Float64, Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped
import yaml

class car_connection:

    def __init__(self):
        rospy.init_node('car_controls', anonymous=True) 
        self.distsb_sub = rospy.Subscriber("/distanceEstimatorSteeringBased/dist", Float64, self._distsb_callback)
        self.anglesb_sub = rospy.Subscriber("/distanceEstimatorSteeringBased/angle", Float64, self._anglesb_callback)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self._odom_callback)
        #self.slamodom_sub = rospy.Subscriber("/slam_out_pose", PoseStamped, self._slamodom_callback)

        self._cmd_vel_pub = rospy.Publisher('/catvehicle/cmd_vel/in', Twist, queue_size=10)

        self._check_all_systems_ready()

        '''
        self.goal = [50, 0]
        self.path = [[self.goal]]
        '''

        self.goal = [self.odom.pose.pose.position.x + 20, self.odom.pose.pose.position.y]
        self.path = [[self.goal]]
        self.last_pt = [0, 0]

        self.linear_speed = 0.5
        self.turning_speed = 0.5
        self.angular_speed = 0.3

        self.yaw = 0
        self.cumm_yaw = 0

        self.epsilon = 3

        self.old_dist = 0

        self.history = []

        self.x1 = self.odom.pose.pose.position.x
        self.y1 = self.odom.pose.pose.position.y
        self.xn = self.goal[0]
        self.yn = self.goal[1]

        print('Making car_connection')


    def move_car(self, linear_speed, angular_speed):
        """
        linear speed: speed car drives when going forward
        angular_speed: speed car goes when turning
        """
        cmd_vel_value = Twist() # Describes linear motion and angular motion of robot
        cmd_vel_value.linear.x = linear_speed
        cmd_vel_value.angular.z = angular_speed
        self._check_cmd_vel_pub()
        self._cmd_vel_pub.publish(cmd_vel_value)
        self.rate.sleep()

    def to_destination(self):
        '''
        Breaks up moving the car into 2 sec blocks; assumes target in front of car
        '''

        self.update_vals()

        while (self.euclid_distance(self.x1, self.y1, self.xn, self.yn) > self.epsilon):
            if (self.distsb.data < 10) and (abs(self.distsb.data - self.old_dist) > 0.01):
                print('Dists: ' + str(self.old_dist)+' '+str(self.distsb.data) + "Differece: " + str(abs(self.distsb.data - self.old_dist)))
                self.old_dist = self.distsb.data
                if (abs(self.anglesb.data) < 0.7):
                    if self.anglesb.data < 0: # Obj on left, turn right
                        print('1')
                        self.avoid_obstacle_turn_right()
                    else:
                        self.avoid_obstacle_turn_left()
                        print('2')
                print('3')
                self.move_car(self.linear_speed, 0)
            else:
                print('5')
                self.point_towards_wpt()
                self.move_car(self.linear_speed, 0)
                print("Current position: " + str([self.x1, self.y1])) 
                print("Current waypoint: " + str(self.next_waypoint))

    def avoid_obstacle_turn_right(self):
        #while(abs(self.anglesb.data) < 0.79):
        #    self.move_car(self.linear_speed, -1 * self.angular_speed)
        self.last_pt = [self.odom.pose.pose.position.x, self.odom.pose.pose.position.y]

        i = 0
        j = 0
        init_dist = self.distsb.data
        init_x = self.x1

        dx = math.cos(self.anglesb.data) * init_dist

        # Turn until object is out of sight
        while ((abs(self.anglesb.data) < 7) or ((self.anglesb.data + 1.57)) <= 0.01):
            if ((abs(abs(self.anglesb.data) - 1.57)) <= 0.01):
                pass
            else:
                print('pivoting car')
                i += 1
                self.move_car(self.turning_speed, -1 * self.angular_speed)

        # Go straight until adjacent to obstacle
        while(self.x1 < init_x + dx):
            print('going past obstacle')
            j += 1
            self.move_car(self.linear_speed, 0)
            self.update_vals()

        if (self.distsb.data < 10) and (abs(self.anglesb.data) < 1):
            # Readjust yaw
            for it in range(2 * i):
                self.move_car(self.turning_speed, self.angular_speed)
            
            # Readjust y value
            for jt in range(j):
                self.move_car(self.linear_speed, 0)

            i = int(math.ceil(i/2))            
            
            # Straighten out yaw
            for it in range(i):
                self.move_car(self.turning_speed, self.angular_speed)

        self.update_vals()
        

        return

    def avoid_obstacle_turn_left(self):
        self.last_pt = [self.odom.pose.pose.position.x, self.odom.pose.pose.position.y]
        i = 0
        j = 0
        init_dist = self.distsb.data
        init_x = self.x1

        dx = math.cos(self.anglesb.data) * init_dist

        while ((abs(self.anglesb.data) < 0.7) or ((self.anglesb.data + 1.57) <= 0.01)):
            if (abs(abs(self.anglesb.data - 1.57) <= 0.01)):
                pass
            else:
                print('pivoting car')
                i += 1
                self.move_car(self.turning_speed, self.angular_speed)

        while(self.x1 < init_x + dx):
            print('going past obstacle')
            j += 1
            self.move_car(self.linear_speed, 0)
            self.update_vals()

        
        if (self.distsb.data < 10) and (abs(self.anglesb.data) < 0.7):
            for it in range(2 * i):
                self.move_car(self.turning_speed, -1 * self.angular_speed)
            
            for jt in range(j):
                self.move_car(self.linear_speed, 0)

            i = int(math.ceil(i/2))            
            
            for it in range(i):
                self.move_car(self.turning_speed, -1 * self.angular_speed)

        self.update_vals()
        


        '''
        x3 = self.x1 + init_dist
        y3 = self.y1 + init_dist

        new_waypoint = [[x3, y3]]

        # Make new waypoint
        self.path.insert(0, new_waypoint)
        '''

        return

    def straight_to_wpt(self):
        # Only run when car is facing towards waypoint
        print('going straight to waypoint: ' + str([self.x2, self.y2]))
        while (self.euclid_distance(self.x1, self.y1, self.x2, self.y2) > self.epsilon):
            self.move_car(self.linear_speed, 0)
        
    

    def update_vals(self):
        #self.quaternion2euler()
        
        self.prev_x = self.last_pt[0]
        self.prev_y = self.last_pt[1]

        self.x1 = self.odom.pose.pose.position.x
        self.y1 = self.odom.pose.pose.position.y
        
        '''
        self.x1 = self.slamodom.pose.position.x
        self.y1 = self.slamodom.pose.position.y
        '''

        # Gets most immediate waypoint

        self.next_waypoint = self.path[0][0]
        self.x2 = self.next_waypoint[0]
        self.y2 = self.next_waypoint[1]

        self.xn = self.goal[0]
        self.yn = self.goal[1]

        self.dx = self.x2 - self.x1
        self.dy = self.y2 - self.y1

        self.hyp = self.euclid_distance(self.x1, self.y1, self.x2, self.y2)

        if (self.y2 < self.y1):
            self.desired_yaw = -1 *  abs(np.arcsin((self.y2 - self.y1)/self.hyp))
        
        if (self.y2 >= self.y1):
            self.desired_yaw = np.arcsin((self.y2 - self.y1)/self.hyp)

        self.hyp = self.euclid_distance(self.prev_x, self.prev_y, self.x1, self.y1)

        if (self.y1 < self.prev_y):
            self.yaw = -1 * abs((np.arcsin((self.y1 - self.prev_y)/self.hyp)))

        if (self.y1 >= self.prev_y):
            self.yaw = np.arcsin((self.y1 - self.prev_y)/self.hyp)

        # Edge cases for yaws

        if (abs(abs(self.desired_yaw) - 1.57) < 0.1):
            self.desired_yaw = 0
        

        if (abs(abs(self.yaw) - 1.57) < 0.1):
            self.yaw = 0

        if ((self.y1 - self.y2) < 0.1): # Car gets past goal
            self.yaw = 0
            self.desired_yaw = 0

        return


    def euclid_distance(self, x1, y1, x2, y2):
        return math.sqrt((x1 - x2)**2 + (y1 - y2)**2) 

    def quaternion2euler(self):
        '''
        x = self.odom.pose.pose.orientation.x
        y = self.odom.pose.pose.orientation.y
        z = self.odom.pose.pose.orientation.z
        w = self.odom.pose.pose.orientation.w
        '''
        '''
        x = self.slamodom.pose.orientation.x
        y = self.slamodom.pose.orientation.y
        z = abs(self.slamodom.pose.orientation.z)
        w = self.slamodom.pose.orientation.w
        '''
        '''
        self.angles = []

        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)
        
        self.yaw = yaw
        '''

        self.yaw = 0



        return yaw
        

    def point_towards_wpt(self):
        if (self.desired_yaw> self.yaw): # Want to turn left
            print('adjusting from: '+str(self.yaw)+'\t to: '+str(self.desired_yaw))
            while (self.desired_yaw - self.yaw > 0.01):
                self.move_car(self.turning_speed, self.angular_speed)
                self.last_pt = [self.odom.pose.pose.position.x, self.odom.pose.pose.position.y]
                self.move_car(self.turning_speed, 0)
                self.move_car(self.turning_speed, 0)
                self.update_vals()
                print('yaw: '+str(self.yaw) + '\t desired yaw: ' + str(self.desired_yaw))
        elif (self.desired_yaw < self.yaw): # Want to turn right
            print('adjusting from: '+str(self.yaw)+'\t to: '+str(self.desired_yaw))
            while (self.desired_yaw - self.yaw < -0.01):
                self.move_car(self.turning_speed, -1 * self.angular_speed)
                self.last_pt = [self.odom.pose.pose.position.x, self.odom.pose.pose.position.y]
                self.move_car(self.turning_speed, 0)
                self.move_car(self.turning_speed, 0)
                self.update_vals()
                print('yaw: '+str(self.yaw) + '\t desired yaw: ' + str(self.desired_yaw))
        else:
            return

    def print_slam_odom(self):
        self._check_slamodom_ready()
        #print(type(self.slamodom))
        print(str(self.slamodom.pose.position.x))
        print(str(self.slamodom.pose.position.y))
        print(str(self.slamodom.pose.position.z))

    """
    These are the callback functions of subscribers
    """
    
    def _distsb_callback(self, data):
        self.distsb = data
    
    def _anglesb_callback(self, data):
        self.anglesb = data
    

    def _odom_callback(self, data):
            self.odom = data
    '''
    def _slamodom_callback(self, data):
        self.slamodom = data
    '''

    """ 
    These functions check if subscribers/publishers are ready  
    """
    def _check_all_systems_ready(self):
        """
        Checks that all the sensors, publishers are operational.
        """
            
        self._check_all_sensors_ready()
        self._check_cmd_vel_pub()
            
        return True

    def _check_all_sensors_ready(self):
        self._check_distsb_ready()
        self._check_anglesb_ready()
        self._check_odom_ready()
        #self._check_slamodom_ready()
            
        return True

    
    def _check_distsb_ready(self):
        self.distsb = None
        rospy.logdebug("Waiting for /distanceEstimatorSteeringBased/dist to be READY...")
        while self.distsb is None and not rospy.is_shutdown():
            try:
                self.distsb = rospy.wait_for_message("/distanceEstimatorSteeringBased/dist", Float64, timeout=5.0)
                rospy.logdebug("Current /distanceEstimatorSteeringBased/dist READY=>")

            except:
                print("distsb: " +  str(self.distsb) + ", rospy.is_shutdown(): " + str(rospy.is_shutdown))
                rospy.logerr("Current /distanceEstimatorSteeringBased/dist not ready yet, retrying for getting dist")

        return self.distsb

    def _check_anglesb_ready(self):
        self.anglesb = None
        rospy.logdebug("Waiting for /distanceEstimatorSteeringBased/angle to be READY...")
        while self.anglesb is None and not rospy.is_shutdown():
            try:
                self.anglesb = rospy.wait_for_message("/distanceEstimatorSteeringBased/angle", Float64, timeout=5.0)
                rospy.logdebug("Current /distanceEstimatorSteeringBased/angle READY=>")

            except:
                print("anglesb: " +  str(self.anglesb) + ", rospy.is_shutdown(): " + str(rospy.is_shutdown))
                rospy.logerr("Current /distanceEstimatorSteeringBased/angle not ready yet, retrying for getting angle")

        return self.anglesb
    

    def _check_odom_ready(self):
        self.odom = None
        rospy.logdebug("Waiting for /odom to be READY...")
        while self.odom is None and not rospy.is_shutdown():
            try:
                self.odom = rospy.wait_for_message("/odom", Odometry, timeout=5.0)
                rospy.logdebug("Current /odom READY=>")

            except:
                print("odom: " +  str(self.odom) + ", rospy.is_shutdown(): " + str(rospy.is_shutdown))
                rospy.logerr("Current /odom not ready yet, retrying for getting odom")
    '''
    def _check_slamodom_ready(self):
        self.slamodom = None
        rospy.logdebug("Waiting for /slam_out_pose to be READY...")
        while self.slamodom is None and not rospy.is_shutdown():
            try:
                self.slamodom = rospy.wait_for_message("/slam_out_pose", PoseStamped, timeout=5.0)
                rospy.logdebug("Current /slam_out_pose READY=>")

            except:
                print("slamodom: " +  str(self.slamodom) + ", rospy.is_shutdown(): " + str(rospy.is_shutdown))
                rospy.logerr("Current /slam_out_pose not ready yet, retrying for getting slamodom")

        return self.slamodom
    '''
    def _check_cmd_vel_pub(self):
        self.rate = rospy.Rate(10)  # 10hz
        while self._cmd_vel_pub.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.logdebug("No susbribers to _cmd_vel_pub yet so we wait and try again")
            try:
                self.rate.sleep()
            except rospy.ROSInterruptException:
                # This is to avoid error when world is rested, time when backwards.
                pass
        rospy.logdebug("_cmd_vel_pub Publisher Connected")

