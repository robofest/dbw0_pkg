#!/usr/bin/env python3

import rospy
#import cv2 as cv
#import numpy as np

from std_msgs.msg import Empty
from geometry_msgs.msg import Twist

def drive4(n):

    # Define enable message
    msg_empty_cmd = Empty()
        
    # Define twist message
    msg_twist_cmd = Twist()
    msg_twist_cmd.linear.x = 0
    msg_twist_cmd.angular.z = 0

    # Define publishers
    pub_enable_cmd =  rospy.Publisher('/vehicle/enable', Empty, queue_size=1)       
    pub_twist_cmd = rospy.Publisher('/vehicle/cmd_vel', Twist, queue_size=1)

    # Define ROS rate for loop
    rate = rospy.Rate(20) # Default rate

    # Get the input from the user
    rospy.loginfo('###')
    rospy.loginfo('### PUT VEHICLE IN FORWARD GEAR')
    rospy.loginfo('###')

    if n == 1 or n == 2 or n == 3:   
        veh_speed = 1.5 # float(input("Enter the vehicle speed (m/s): "))
    elif n == 4:
        veh_speed = -1 # float(input("Enter the vehicle speed (m/s): "))
    msg_twist_cmd.linear.x = veh_speed
    
    
    time_dur = 7  # float(input("Enter the duration of command (s): "))
                
    # Loop and publish commands to vehicle based on the choice
    time_start = rospy.Time.now()
    dt = 0.0

    rospy.loginfo('Starting Timer...')
    while not rospy.is_shutdown() and (dt < time_dur): # study rospy.is_shutdown()
        dt = (rospy.Time.now() - time_start).to_sec() # compute elapsed time since start          
        
        pub_enable_cmd.publish(msg_empty_cmd) # Publish enable message
        pub_twist_cmd.publish(msg_twist_cmd)
        rate.sleep()
    rospy.loginfo('Time elasped...')

    if n == 2 or n == 3: # Turn left or right
        time_start2 = rospy.Time.now()
        dt = 0.0
        while not rospy.is_shutdown() and (dt < time_dur): # study rospy.is_shutdown()
            dt = (rospy.Time.now() - time_start2).to_sec() # compute elapsed time since start 2           
            
            pub_enable_cmd.publish(msg_empty_cmd) # Publish enable message
            if n == 2:
                msg_twist_cmd.angular.z = 0.1
            else: # must be 3
                msg_twist_cmd.angular.z = -0.1
            pub_twist_cmd.publish(msg_twist_cmd)
            rate.sleep()
        rospy.loginfo('Time elasped 2...')
        
    # Stop the vehicle
    msg_twist_cmd.linear.x = 0 
    #while not rospy.is_shutdown():
    pub_enable_cmd.publish(msg_empty_cmd)
    pub_twist_cmd.publish(msg_twist_cmd)
    #rate.sleep()           
    return 

#################    
# Main function
#################
if __name__ == '__main__':
    
    # Initialize the node and name it.
    rospy.init_node('drive4_node')
    print("drive 4 node initialized")
    
    while True:
        try:
            print("0: quit")
            print("1: forward")
            print("2: turn left")
            print("3: turn right")
            print("4: backward")
            choice = int(input("Enter a number: "))
            if choice >= 1 and choice <= 4: # valid input
                drive4(choice)
            elif choice == 0:
                break
            else:
                print("Enter 0 ~ 4")
        except Exception as e:
            print(f"Oops! {e.__class__} occured")
