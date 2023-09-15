#!/usr/bin/env python2.7

import rospy
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg
import subprocess
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Twist, PoseStamped
from visualization_msgs.msg import Marker
import math
from time import sleep, time
import serial

# Defines #
WATER_CANNON_MISSION_TYPE = 1
X_DIST_TO_SHOOT = 1.5 # How far the boat should be in X axis from the target when shooting - X is "in front of me"
Y_DIST_TO_SHOOT = 0.2 # How far the boat should be in Y axis from the target when shooting - Y is "left (positive) and right (negativ)"
MAX_TIME_FOR_SHOOTING = 20 # How much time in secs we let the cannon shoot water to complete the mission
MIN_TIMES_THAT_TARGET_FOUND_IN_A_ROW = 5

ARDUINO_PORT = '/dev/ttyUSB0' 
ARDUINO_BAUDRATE = 115200
ARDUINO_COMMAND_STOP_WATER_CANNON = 7000
ARDUINO_COMMAND_START_WATER_CANNON = 17000

# We want to use class Point for target managment #
class Point():
    def __init__(self):
        self.x = 0
        self.y = 0
        self.z = 0
        self.angle = 0
        self.time = 0
    
    def update(self, x, y, z, angle):
        self.x = x
        self.y = y
        self.z = z
        self.angle = angle
        self.time = time()

    def reset(self):
        self.x = 0
        self.y = 0
        self.z = 0
        self.angle = 0
        self.time = 0

# This class will hold all the functionality of the water cannon, from listening, publishing and sending command to arduino #
class WaterCannon:
    def __init__(self):
        self.mission_type = WATER_CANNON_MISSION_TYPE
        self.start_shooting_time = 0
        self.target_location = Point()
        self.how_much_time_target_found_in_a_row = 0

        self.arduino = serial.Serial(port= ARDUINO_PORT , baudrate = ARDUINO_BAUDRATE, timeout = 1)

        rospy.init_node('Water_Cannon')
        self.boat_motor_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.targets_subscriber = rospy.Subscriber("/ball_marker", MarkerArray, callback= self.handling_target)
        while not rospy.is_shutdown():  
            rospy.Rate(1).sleep()
    
    ######### End of init #########

    def handling_target(self, marker_array):
        closest_target = Marker()
        found = 0
        for i in range(len(marker_array.markers)):
            if ((marker_array.markers[i].type == WATER_CANNON_MISSION_TYPE) and (marker_array.markers[i].pose.position.x != None)):
                #found = 1
                if (marker_array.markers[i].pose.position.x != self.target_location.x):
                    found = 1
                    closest_target = marker_array.markers[i]

        if (found == 1):
            self.target_location.update(closest_target.pose.position.x, closest_target.pose.position.y, closest_target.pose.position.z, 0)

            rospy.loginfo("Hi, I found target in: {},{},{}".format(str(self.target_location.x), str(self.target_location.y), str(self.target_location.z)))
            self.publish_to_cmd_vel(self.target_location.x, self.target_location.y, self.target_location.y)
            self.shooting_control()
        if (found == 0):
            self.how_much_time_target_found_in_a_row = 0
        return

    def publish_to_cmd_vel(self, pos_x, pos_y, pos_z):
        cmd_vel_msg= Twist()
        cmd_vel_msg.linear.x= pos_x
        cmd_vel_msg.linear.y=pos_y
        cmd_vel_msg.linear.z = pos_z
        cmd_vel_msg.angular.z=0
        if(pos_x < X_DIST_TO_SHOOT):
            cmd_vel_msg.linear.x = 0
        if(abs(pos_y) < Y_DIST_TO_SHOOT):
            cmd_vel_msg.linear.y = 0
        self.boat_motor_publisher.publish(cmd_vel_msg)
        return


    def shooting_control(self):
        if(self.target_location.x < X_DIST_TO_SHOOT and abs(self.target_location.y) < Y_DIST_TO_SHOOT):
           self.how_much_time_target_found_in_a_row += 1
        
        if(self.how_much_time_target_found_in_a_row >= MIN_TIMES_THAT_TARGET_FOUND_IN_A_ROW):
            if(self.start_shooting_time == 0):
                self.start_shooting_time = time()
            if(time() - self.start_shooting_time <= MAX_TIME_FOR_SHOOTING):
                rospy.loginfo("Start shooting - Let's blast this target with some water!")
                self.send_command_to_arduino(ARDUINO_COMMAND_START_WATER_CANNON)
            else:
                rospy.loginfo("Stop shooting - I feel like the target had enough")
                self.send_command_to_arduino(ARDUINO_COMMAND_STOP_WATER_CANNON)      
        return           


    def send_command_to_arduino(self, command):
        self.arduino.write(str(command).encode("utf-8"))
        return


if __name__ == '__main__':
    my_water_cannon = WaterCannon() # All the magic happens inside the class
