#!/usr/bin/env python
import rospy
import os
from std_msgs.msg import String



def callback(data):
    #rospy.loginfo(rospy.get_caller_id()+"I heard %s",data.data)
    if data.data=='cutting':
        rospy.loginfo("stoping telecontrol")
        os.system('rosservice call /multiple_joystick_mux/select "/joy_other"')
        

def listener():
    kill_flag=True
    # in ROS, nodes are unique named. If two nodes with the same
    # node are launched, the previous one is kicked off. The 
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaenously.
    rospy.init_node('self_action_killnode', anonymous=True)

    rospy.Subscriber("self_action_rec", String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
        
if __name__ == '__main__':
    listener()
