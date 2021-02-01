#!/usr/bin/env python
# license removed for brevity

import rospy
import math
from geometry_msgs.msg import PoseStamped, TransformStamped, PoseArray, Twist, Vector3
from std_msgs.msg import String, Empty, UInt8
from nav_msgs.msg import Odometry
import numpy as np
from tf.transformations import euler_from_quaternion

class globalPlanner:
    def __init__(self):
        self.agent = 0    #Defines ID number of this agent (replace for hardware definition)
        print("linha14")
        self.cmd_pub  = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=1)
        print("linha17")
        self.cmd_sub = rospy.Subscriber("cmd_global", PoseArray, self.callback_cmd)
        print("linha19")
        self.odom_sub = rospy.Subscriber("odom_imu", Odometry, self.callback_imu)      #APENAS DEBUG
        print("linha21")
        self.takeoff = rospy.Publisher("/bebop/takeoff", Empty, queue_size=10)
        self.land = rospy.Publisher("/bebop/land", Empty, queue_size=10)
        rospy.init_node('Rasp_link', anonymous=True)

        print("comecou")

    def callback_imu(self, odom_imu):
        odomimu = Odometry()

        # odomimu.pose.pose.position.x = odom_imu.pose.pose.position.x
        # odomimu.pose.pose.position.y = odom_imu.pose.pose.position.y
        # odomimu.pose.pose.position.z = odom_imu.pose.pose.position.z
        odomimu = odom_imu

        #TENTAR SUBSTITUIR POR UMA UNICA CHAMADA!
        odomimu.header.stamp.secs = rospy.get_rostime().secs
        odomimu.header.stamp.nsecs = rospy.get_rostime().nsecs

        odomimu.header.frame_id = "3"
        #EXECUTA TODA VEZ QUE CHEGA UPDATE NA IMU
        self.odom_pub.publish(odomimu)

    def callback_cmd(self, cmdvelglb):
        cmdvel = Twist()

        if(cmdvelglb.poses[self.agent].orientation.y > 0):
            print("Taking Off")            
            self.takeoff.publish(Empty())
        elif(cmdvelglb.poses[self.agent].orientation.z > 0):
            print("Landing")            
            self.land.publish(Empty())
        else:
            cmdvel.linear.x = cmdvelglb.poses[self.agent].position.x
            cmdvel.linear.y = cmdvelglb.poses[self.agent].position.y
            cmdvel.linear.z = cmdvelglb.poses[self.agent].position.z
            cmdvel.angular.x = 0
            cmdvel.angular.y = 0
            cmdvel.angular.z = cmdvelglb.poses[self.agent].orientation.x

            #EXECUTA SEMPRE QUE CHEGA MENSAGEM NOVA NO TOPICO GLOBAL (VINDO DO DRONE_DEV)
            self.cmd_pub.publish(cmdvel)

def main():
    print("linha45")
    globalPlanner()
    print("linha47")
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main()