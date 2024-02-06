#!/usr/bin/env python3

# Using UlcCmd and SteeringCmd instead of Twist
# CJ Nov 2023

import rospy
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from dataspeed_ulc_msgs.msg import UlcCmd
from dbw_polaris_msgs.msg import SteeringCmd

# DurationDrive class definition
class DurationDrive():
    def __init__(self):
        """Node to drive forward for a give time and speed"""

        # Define enable message
        self.msg_empty_cmd = Empty()

        self.msg_ulc = UlcCmd()
        self.msg_ulc.clear = False
        self.msg_ulc.enable_pedals = True
        self.msg_ulc.enable_shifting = True
        self.msg_ulc.enable_steering = False
        self.msg_ulc.shift_from_park = False

        self.msg_steer = SteeringCmd()
        self.msg_steer.steering_wheel_angle_cmd = 0.0
        self.msg_steer.steering_wheel_angle_velocity = 0.0
        self.msg_steer.steering_wheel_torque_cmd = 0.0
        self.msg_steer.cmd_type = SteeringCmd.CMD_ANGLE
        self.msg_steer.enable = True
        self.msg_steer.clear = False
        self.msg_steer.ignore = False
        self.msg_steer.calibrate = False
        self.msg_steer.quiet = False
        self.msg_steer.count = 0
        
        # Define twist message
        self.msg_twist_cmd = Twist()
        self.msg_twist_cmd.linear.x = 0
        self.msg_twist_cmd.angular.z = 0

        # Define publishers
        self.pub_enable_cmd =  rospy.Publisher('/vehicle/enable', Empty, queue_size=1)
        
        self.pub_twist_cmd = rospy.Publisher('/vehicle/cmd_vel', Twist, queue_size=1)
        
        self.pub_ulc_cmd = rospy.Publisher('/vehicle/ulc_cmd', UlcCmd, queue_size=1)

        self.pub_steer_cmd = rospy.Publisher('/vehicle/steering_cmd', SteeringCmd, queue_size=1)

        # Define ROS rate
        self.rate = rospy.Rate(20) # Default rate

        # Get input from user
        print("********************** First check if the wheel is straight **********************")
        veh_speed = float(input("Enter the vehicle speed (m/s): "))
        #veh_yaw = float(input("Enter the vehicle yaw rate (rad/s): "))
        veh_swa = float(input("Enter the vehicle SWA (rad, negative turn right): "))
        time_dur = float(input("Enter the duration of command (s): "))
        
        
        # Loop and publish commands to vehicle
        time_start = rospy.Time.now()
        dt = 0.0
        time_stop = 2 # estimated time to setup DBW
        
        rospy.loginfo('Starting Timer...')
        while( not rospy.is_shutdown()):

            # Compute dt
            dt = (rospy.Time.now() - time_start).to_sec()
            
            # Publish enable message
            self.pub_enable_cmd.publish(self.msg_empty_cmd)
            if( dt <  time_stop ):
                rospy.loginfo('Initializing...')
                self.msg_twist_cmd.linear.x = 0
                self.msg_twist_cmd.angular.z = 0
                self.pub_twist_cmd.publish(self.msg_twist_cmd)
            elif( dt < time_dur + time_stop ):
                rospy.loginfo('Turning...')
                #self.msg_twist_cmd.linear.x = veh_speed
                self.msg_ulc.linear_velocity = veh_speed
                self.pub_ulc_cmd.publish(self.msg_ulc)
                #if dt>time_dur+0.25:
                #   self.msg_twist_cmd.angular.z = veh_yaw
                self.msg_steer.steering_wheel_angle_cmd = veh_swa
                self.pub_steer_cmd.publish(self.msg_steer)
            else:
                rospy.loginfo('Time elasped...')
                #self.msg_twist_cmd.linear.x = 0
                #self.msg_twist_cmd.angular.z = 0
                #self.pub_twist_cmd.publish(self.msg_twist_cmd)
                self.msg_ulc.linear_velocity = 0.0
                self.pub_ulc_cmd.publish(self.msg_ulc)
                self.msg_steer.steering_wheel_angle_cmd = 0.0
                self.pub_steer_cmd.publish(self.msg_steer)
            
            # Q: Can we mix Ulc and Twist? it seems yes         

            # Sleep for time step
            self.rate.sleep()

        return

    

#################    
# Main function
#################
if __name__ == '__main__':
    
    # Initialize the node and name it.
    rospy.init_node('duration_drive_node')
    print("Duration Drive node initialized")
    
    # Start tester
    try:
        DurationDrive()
    except rospy.ROSInterruptException:
        pass



    
