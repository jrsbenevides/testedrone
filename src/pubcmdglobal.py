#!/usr/bin/env python
# license removed for brevity

import rospy
from geometry_msgs.msg import PoseStamped, TransformStamped, PoseArray, Twist, Vector3

def ControlDecision():
    pubCmdGlb = rospy.Publisher("cmd_vel_global", Twist, queue_size=1)
    rospy.init_node('global_controller', anonymous=True)
    rate = rospy.Rate(20) # 20Hz
    while not rospy.is_shutdown():
        
        cmdglb = Twist()

        cmdglb.linear.x = 1.0
        cmdglb.linear.y = 1.0
        cmdglb.linear.z = 0.0

        #EXECUTA TODA VEZ QUE CHEGA UPDATE NA IMU
        pubCmdGlb.publish(cmdglb)
        rate.sleep()

if __name__ == '__main__':
    try:
        ControlDecision()
    except rospy.ROSInterruptException:
        pass