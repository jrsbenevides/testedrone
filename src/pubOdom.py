#!/usr/bin/env python
# license removed for brevity

import rospy
from nav_msgs.msg import Odometry

def sampleIMU():
    agent = 0
    nOfAgents = 1
    dt = 0.0
    pubIMU = rospy.Publisher("/odom_global", Odometry, queue_size=1)
    rospy.init_node('pubOdom', anonymous=True)
    rate = rospy.Rate(25) # 25hz
    while not rospy.is_shutdown():
        
        odomimu = Odometry()


        odomimu.header.frame_id = str(agent)
        odomimu.header.stamp.secs = rospy.get_rostime().secs
        odomimu.header.stamp.nsecs = rospy.get_rostime().nsecs

        odomimu.pose.pose.position.x = 1.0 + dt + agent
        odomimu.pose.pose.position.y = 2.0 + dt + agent
        odomimu.pose.pose.position.z = 3.0 + dt + agent

        odomimu.pose.pose.orientation.x = 0.0
        odomimu.pose.pose.orientation.y = 0.0
        odomimu.pose.pose.orientation.z = 0.0
        odomimu.pose.pose.orientation.w = 1.0

        #EXECUTA TODA VEZ QUE CHEGA UPDATE NA IMU
        pubIMU.publish(odomimu)
        #INCREMENTA O AGENTE    
        agent = agent + 1
        dt = dt + 0.1

        #debug
        print("Enviando Msg do Ag " + odomimu.header.frame_id)
        print("\nConteudo:")
        print("\nx:" + str(odomimu.pose.pose.position.x))
        print("\ny:" + str(odomimu.pose.pose.position.y))
        print("\nz:" + str(odomimu.pose.pose.position.z))

        if(agent >= nOfAgents):
            agent = 0
        rate.sleep()

if __name__ == '__main__':
    try:
        sampleIMU()
    except rospy.ROSInterruptException:
        pass



# agent = odomRaw->header.frame_id;
# nAgent =  atoi(agent.c_str());   
# incomingMsg.index = nAgent;
# incomingMsg.tsArrival = ros::Time::now().toSec();
# incomingMsg.tsSensor  = odomRaw->header.stamp.toSec();
# incomingMsg.tGSendCont = 0;