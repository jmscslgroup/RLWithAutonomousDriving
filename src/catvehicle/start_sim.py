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
        time.sleep(5)   # Give roscore time to start
        rospy.init_node('rl_sim', anonymous=True)    # Requirement before we continue /w ROS

        """ Set up uuid; unique code for a computer process """
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)

        #Object to launch empty world; parent roslaunch type
        launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/nguy2539/catvehicle_ws/src/catvehicle/launch/catvehicle_brickwall.launch"])
        
        # TODO: make .py script that trains for 1 episode.

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
        launch.start()
        
        print('Brickwall world launched.')
        time.sleep(3)
        '''
        """ Spawn the world, car and actor """
        for i in range(len(self.launchspawn)):
            self.launchspawn[i].start()
            time.sleep(2)
        '''

    def signal_handler(self, sig):
        print('You pressed Ctrl+C!')
        print('############################################')
        """
        print('Terminating spawn launches')
        for n in range(len(self.launchspawn)):
            self.launchspawn[n].shutdown()
        """
        
        print(str(sig))
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

'''
def main(argv):
    cl = start_sim()

    cl.spawn()
    
    """ When SIGINT (signal interrupt) recieved, run cl.signal_handler """
    signal.signal(signal.SIGINT, cl.signal_handler) 
    print('Press Ctrl+C')
    signal.pause()

if __name__ == '__main__':
    main(sys.argv[1:])
'''
