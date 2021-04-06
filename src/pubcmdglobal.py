#!/usr/bin/env python
# license removed for brevity

import rospy
from geometry_msgs.msg import PoseStamped, TransformStamped, PoseArray, Twist, Vector3,Pose

def ControlDecision():
    pubCmdGlb = rospy.Publisher("cmd_global", PoseArray, queue_size=1)
    rospy.init_node('global_controller', anonymous=True)
    rate = rospy.Rate(20) # 20Hz
    while not rospy.is_shutdown():
        

        ps = PoseArray()
   
        ps.header.frame_id = "/base_link"
        ps.header.stamp = rospy.Time.now()
   
        cmdglb = Pose()

        cmdglb.position.x = 0.1
        cmdglb.position.y = 0.1
        cmdglb.position.z = 0.1

        cmdglb.orientation.w = 0
        cmdglb.orientation.x = 0
        cmdglb.orientation.y = 0 #takeoff instruction
        cmdglb.orientation.z = 0 #land instruction

        ps.poses.append(cmdglb)

        #EXECUTA TODA VEZ QUE CHEGA UPDATE NA IMU
        pubCmdGlb.publish(ps)
        rate.sleep()

if __name__ == '__main__':
    try:
        ControlDecision()
    except rospy.ROSInterruptException:
        pass