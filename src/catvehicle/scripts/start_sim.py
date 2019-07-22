#!/usr/bin/env python
#
# Author: Rahul Bhadani
# Copyright (c) 2019 Rahul Bhadani, Arizona Board of Regents
# All rights reserved.

""" This script helps launch a fleet of n cars along x-axis. """

import roslaunch
import rospy
import sys, math, time
import subprocess
from subprocess import call
import signal
import psutil
import numpy as np
from std_msgs.msg import Float64, Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import yaml

# Creates a class about closing and launching gazebo worlds; meant to alleviate poor gazebo optimization
class start_sim:
    def __init__(self):
        call(["pkill", "ros"])      # 'process kill' ros
        call(["pkill", "gzserver"]) # 'process kill' gzserver
        call(["pkill", "gzclient"]) # 'process kill' gzclient
        time.sleep(2)               # Give 2 sec for all processes to be killed

    def spawn(self):
        """Start roscore"""
        self.roscore = subprocess.Popen('roscore', stdout=subprocess.PIPE, shell=True)
        self.roscore_pid = self.roscore.pid
        time.sleep(10)   # Give roscore time to start
        rospy.init_node('rl_sim', anonymous=True)    # Requirement before we continue /w ROS


        """ Set up uuid; unique code for a computer process """
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)

        #Object to launch empty world; parent roslaunch type
        self.launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/nguy2539/catvehicle_ws/src/catvehicle/launch/catvehicle_collision.launch"])
        

        #Object to spawn catvehicle in the empty world
        
        """ Input Eric's actor here """
        """
        #cli_args = []           # Arguments you put in the terminal before you run the launch file
        spawn_file = []         # Array for spawn.launch files
        self.launchspawn = []   # Array for spawn.launch files executables
        launchfile = ['/home/reu-cat/catvehicle_ws/src/catvehicle/launch/catvehicle_brickwall.launch']
        humanlaunchfile = ['/home/reu-cat/catvehicle_ws/src/catvehicle/launch/catvehicle_brickwall.launch']
        spawn_file.append([(roslaunch.rlutil.resolve_launch_arguments(launchfile)[0])])
        self.launchspawn.append(roslaunch.parent.ROSLaunchParent(uuid, spawn_file[0]))
        """
        
        '''Launch the brickwall world'''
        self.launch.start()

        time.sleep(15)
        
        print('Brickwall world launched.')
        '''
        """ Spawn the world, car and actor """
        for i in range(len(self.launchspawn)):
            self.launchspawn[i].start()
            time.sleep(2)
        '''

        self.io_setup()
        self.load_params()

        # If the car gets close to here, it gets reward
        self.path_array = [[5, 0, 0], [10, 0, 0], [12.5, 6.25, 0], [12.5, -6.25, 0], [20, 12.5, 0], [20, -12.5, 0], [25, 6.25, 0], [25, -6.25, 0], [27.5, 0, 0], [30, 0, 0], [35, 0, 0], [40, 0, 0], [45, 0, 0]]

        self.cumulated_reward = 0


    def signal_handler(self, sig):
        self.distsb_sub.unregister()
        self.anglesb_sub.unregister()
        self.dist_sub.unregister()
        self.angle_sub.unregister()
        self._cmd_vel_pub.unregister()
        self.odom_sub.unregister()

        time.sleep(10)


        print('CLOSING SIMULATION')
        """
        print('Terminating spawn launches')
        for n in range(len(self.launchspawn)):
            self.launchspawn[n].shutdown()
        """
        
        self.launch.shutdown()

        print('Now killing roscore')
        #kill the child process of roscore
        try:
            parent = psutil.Process(self.roscore_pid)
            print(parent)
        except psutil.NoSuchProcess:
            print("Parent process doesn't exist.")
            return
        children = parent.children(recursive=True)
        print(children)
        for process in children:
            print("Attempted to kill child: " + str(process))
            process.send_signal(signal.SIGTERM)

        #kill the roscore
        self.roscore.terminate()
        #Wait to prevent the creation of zombie processes.
        self.roscore.wait()

        call(["pkill", "ros"])
        call(["pkill", "gzserver"])
        call(["pkill", "gzclient"])

    def io_setup(self):
        print("SUBSCRIBER/PUBLISHER SETUP NOW")
        self.distsb_sub = rospy.Subscriber("/catvehicle/distanceEstimatorSteeringBased/dist", Float64, self._distsb_callback)
        self.anglesb_sub = rospy.Subscriber("/catvehicle/distanceEstimatorSteeringBased/angle", Float64, self._anglesb_callback)
        self.dist_sub = rospy.Subscriber("/catvehicle/distanceEstimator/dist", Float32, self._dist_callback)
        self.angle_sub = rospy.Subscriber("/catvehicle/distanceEstimator/angle", Float32, self._angle_callback)
        self.odom_sub = rospy.Subscriber("/catvehicle/odom", Odometry, self._odom_callback)

        self._cmd_vel_pub = rospy.Publisher('/catvehicle/cmd_vel', Twist, queue_size=10)

        self._check_all_systems_ready()

    def _check_all_systems_ready(self):
        """
        Checks that all the sensors, publishers are operational.
        """
        
        self._check_all_sensors_ready()
        self._check_cmd_vel_pub()
        
        return True

    def _check_all_sensors_ready(self):
        """
        Checks that all the sensors, publishers and other simulation systems are
        operational.
        """
        
        self._check_dist_ready()
        self._check_angle_ready()
        self._check_odom_ready()
        self._check_distsb_ready()
        self._check_anglesb_ready()
        
        return True

        
    def _check_dist_ready(self):
        self.dist = None
        rospy.logdebug("Waiting for /catvehicle/distanceEstimator/dist to be READY...")
        while self.dist is None and not rospy.is_shutdown():
            try:
                print(str(rospy.is_shutdown()))
                self.dist = rospy.wait_for_message("/catvehicle/distanceEstimator/dist", Float32, timeout=5.0)
                rospy.logdebug("Current /catvehicle/distanceEstimator/dist READY=>")

            except:
                print("dist: "+ str(self.dist) + ", rospy.is_shutdown(): " + str(rospy.is_shutdown))
                rospy.logerr("Current /catvehicle/distanceEstimator/dist not ready yet, retrying for getting dist")
        return self.dist
        
    
    def _check_angle_ready(self):
        self.angle = None
        rospy.logdebug("Waiting for /catvehicle/distanceEstimator/angle to be READY...")
        while self.angle is None and not rospy.is_shutdown():
            try:
                self.angle = rospy.wait_for_message("/catvehicle/distanceEstimator/angle", Float32, timeout=5.0)
                rospy.logdebug("Current /catvehicle/distanceEstimator/angle READY=>")

            except:
                print("angle: " +  str(self.angle) + ", rospy.is_shutdown(): " + str(rospy.is_shutdown))
                rospy.logerr("Current /catvehicle/distanceEstimator/angle not ready yet, retrying for getting angle")
        return self.angle

    def _check_odom_ready(self):
        self.odom = None
        rospy.logdebug("Waiting for /catvehicle/odom to be READY...")
        while self.odom is None and not rospy.is_shutdown():
            try:
                self.odom = rospy.wait_for_message("/catvehicle/odom", Odometry, timeout=5.0)
                rospy.logdebug("Current /catvehicle/odom READY=>")

            except:
                print("odom: " +  str(self.odom) + ", rospy.is_shutdown(): " + str(rospy.is_shutdown))
                rospy.logerr("Current /catvehicle/odom not ready yet, retrying for getting odom")

        return self.odom
    
    def _check_distsb_ready(self):
        self.distsb = None
        rospy.logdebug("Waiting for /catvehicle/distanceEstimatorSteeringBased/dist to be READY...")
        while self.distsb is None and not rospy.is_shutdown():
            try:
                self.distsb = rospy.wait_for_message("/catvehicle/distanceEstimatorSteeringBased/dist", Float64, timeout=5.0)
                rospy.logdebug("Current /catvehicle/distanceEstimatorSteeringBased/dist READY=>")

            except:
                print("distsb: " +  str(self.distsb) + ", rospy.is_shutdown(): " + str(rospy.is_shutdown))
                rospy.logerr("Current /catvehicle/distanceEstimatorSteeringBased/dist not ready yet, retrying for getting dist")

        return self.distsb
        
    def _check_anglesb_ready(self):
        self.anglesb = None
        rospy.logdebug("Waiting for /catvehicle/distanceEstimatorSteeringBased/angle to be READY...")
        while self.anglesb is None and not rospy.is_shutdown():
            try:
                self.anglesb = rospy.wait_for_message("/catvehicle/distanceEstimatorSteeringBased/angle", Float64, timeout=5.0)
                rospy.logdebug("Current /catvehicle/distanceEstimatorSteeringBased/angle READY=>")

            except:
                print("anglesb: " +  str(self.anglesb) + ", rospy.is_shutdown(): " + str(rospy.is_shutdown))
                rospy.logerr("Current /catvehicle/distanceEstimatorSteeringBased/angle not ready yet, retrying for getting angle")

        return self.anglesb

    def _check_cmd_vel_pub(self):
        rate = rospy.Rate(10)  # 10hz
        while self._cmd_vel_pub.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.logdebug("No susbribers to _cmd_vel_pub yet so we wait and try again")
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                # This is to avoid error when world is rested, time when backwards.
                pass
        rospy.logdebug("_cmd_vel_pub Publisher Connected")

    def move_car(self, linear_speed, angular_speed, epsilon=0.05, update_rate=10, min_laser_distance=-1):
        """
        linear speed: speed car drives when going forward
        angular_speed: speed car goes when turning
        """
        cmd_vel_value = Twist() # Describes linear motion and angular motion of robot
        cmd_vel_value.linear.x = linear_speed
        cmd_vel_value.angular.z = angular_speed
        self._check_cmd_vel_pub()
        self._cmd_vel_pub.publish(cmd_vel_value)
        
    def load_params(self):
        with open('rl_params.yaml', 'r') as f:
            params = yaml.load(f)

        # Don't think I need this
        
        self.dec_obs = 1
        self.linear_forward_speed = params['catvehicle']['linear_forward_speed']
        self.linear_turn_speed = params['catvehicle']['linear_turn_speed']
        self.angular_speed = params['catvehicle']['angular_speed']
        
        
        # Sensors and input data
        self.min_range = params['catvehicle']['min_range']
        self.max_sensor_value = params['catvehicle']['max_sensor_value']
        self.min_sensor_value = params['catvehicle']['min_sensor_value']
        self.min_angle_value = params['catvehicle']['min_angle_value']
        self.max_angle_value = params['catvehicle']['max_angle_value']

        # Rewards
        self.forwards_reward = params['catvehicle']['forwards_reward']
        self.turn_reward = params['catvehicle']['turn_reward']
        self.end_episode_points = params['catvehicle']['end_episode_points']
        self.path_epsilon = params['catvehicle']['path_epislon']
        self.path_reward = params['catvehicle']['path_reward']

        self.complete_reward = params['catvehicle']['complete_reward']

    def do_action(self, action):
        # action is 0 (straight), 1 (left) or 2 (right)
        """
        This set action will Set the linear and angular speed of the turtlebot2
        based on the action number given.
        :param action: The action integer that set s what movement to do next.
        """

        # We convert the actions to speed movements to send to the parent class CATVehicleEnv
        if action == 0: #FORWARD
            linear_speed = self.linear_forward_speed
            angular_speed = 0.0
            self.last_action = "FORWARDS"
        elif action == 1: #LEFT
            linear_speed = self.linear_turn_speed
            angular_speed = self.angular_speed
            self.last_action = "TURN_LEFT"
        else: #RIGHT
            linear_speed = self.linear_turn_speed
            angular_speed = -1*self.angular_speed
            self.last_action = "TURN_RIGHT"

        print("EXECUTING ACTION: ", self.last_action)
        
        # We tell TurtleBot2 the linear and angular speed to set to execute
        self.move_car( linear_speed,
                        angular_speed,
                        epsilon=0.05,
                        update_rate=10,
                        min_laser_distance=self.min_range)
        
        rospy.logdebug("END Set Action ==>"+str(action)+", NAME="+str(self.last_action))

    """ Functions to check if car has crashed """

    def has_crashed(self, min_distance):
        """
        It states based on the laser scan if the robot has crashed or not.
        Crashed means that the minimum laser reading is lower than the
        min_laser_distance value given.
        If min_laser_distance == -1, it returns always false, because its the way
        to deactivate this check.
        """
        dist = self.distsb.data

        if (dist <= min_distance):
            rospy.logwarn("CATVehicle HAS CRASHED >>> item = " + str(dist)+" < "+str(min_distance))
            return True
        return False

    def is_done(self):
        return self.has_crashed(self.min_distance)

    """ Functions associated with returning sensor data to the algo """
    def _get_obs(self):
        self.dist = self.get_dist()
        self.angle = self.get_angle()
        
        if (self.angle.data < self.min_angle_value) or (self.angle.data > self.max_angle_value):
            return [self.dist.data, 0]
        else:
            return [self.dist.data, self.angle.data]


    """ Functions for computing reward """
    def _compute_reward(self, observations, done):
        dists_to_reward = []


        if not done:
            if self.last_action == "FORWARDS":
                reward = self.forwards_reward
            else:
                reward = self.turn_reward
        else:
            reward = -1*self.end_episode_points
            
        x = self.odom.pose.pose.position.x
        y = self.odom.pose.pose.position.y
        
        # If the car gets epsilon close to any points on the path, car gets reward
        for i in range(len(self.path_array)):
            reward_x = self.path_array[i][0]
            reward_y = self.path_array[i][1]
            dists_to_reward.append(self.euclid_distance(reward_x, reward_y, x, y))
            
            if (self.euclid_distance(reward_x, reward_y, x, y) < self.path_epsilon) and self.path_array[i][2] != 1:
                self.path_array[i][2] = 1
                reward += self.path_reward

        if self.is_complete() == (len(self.path_array) - 3):
            reward += self.complete_reward
        

        self.cumulated_reward += reward

        return reward

    def euclid_distance(self, x1, y1, x2, y2):
        return math.sqrt((x1 - x2)**2 + (y1 - y2)**2) 

    def step(self, action):
        self.do_action(action)
        time.sleep(0.1) # Current timestep length
        obs = self._get_obs()
        done = self.has_crashed(self.min_range)
        reward = self._compute_reward(obs, done)
        complete = self.is_complete()

        if complete == (len(self.path_array) - 3):
            done = True
        

        return obs, reward, done, complete

    def is_complete(self):
        num_complete = 0
        for i in range(len(self.path_array)):
            if self.path_array[i][2] == 1:
                num_complete += 1
        return num_complete



    """ Subscriber callback functions """
    def get_dist(self):
        return self.dist
        
    def get_angle(self):
        return self.angle
        
    def _dist_callback(self, data):
        self.dist = data
    
    def _angle_callback(self, data):
        self.angle = data
        
    def _distsb_callback(self, data):
        self.distsb = data
        
    def _anglesb_callback(self, data):
        self.anglesb = data
        
    def _odom_callback(self, data):
        self.odom = data
    
