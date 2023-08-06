#!/usr/bin/python
#-*- coding:utf-8 -*-

import rospy
from rospy import Time
from gazebo_msgs.srv import GetModelState, GetModelStateRequest

def timestamp2sec(timestamp):
    return timestamp.secs + 1e-9 * timestamp.nsecs

if __name__ == "__main__":
    rospy.init_node("model_state_subscriber")
    get_state_service = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    model = GetModelStateRequest()
    model.model_name = "typhoon_h480_0"
    loop_rate = rospy.Rate(1)
    pose_file = open("../data/poseB_quaterniond.txt", "w")
    is_initialization = False
    initial_time = None
    while (not rospy.is_shutdown()):
        objstate = get_state_service(model)
        if not is_initialization:
            initial_time = timestamp2sec(objstate.header.stamp)
            is_initialization = True
        pose_file.write(str(timestamp2sec(objstate.header.stamp) - initial_time) + " ")
        pose_file.write(str(objstate.pose.position.x) + " " +
                                        str(objstate.pose.position.y) + " " +
                                        str(objstate.pose.position.z) + " " +
                                        str(objstate.pose.orientation.x) + " " +
                                        str(objstate.pose.orientation.y) + " " +
                                        str(objstate.pose.orientation.z) + " " +
                                        str(objstate.pose.orientation.w) + "\n")
        loop_rate.sleep()
    print("Over.")
    pose_file.close()