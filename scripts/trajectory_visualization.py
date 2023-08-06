import numpy as np
from math import pi
from scipy.spatial.transform import Rotation as R

import rospy
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

class TrajectoryVisualizer:
    def __init__(self, visualization_topic = "visualization"):
        self.frame_id = 0
        self.orig_framework_points = np.array([[0, 0, 0], [-0.3, -0.6, 0.3], 
                                                [-0.3, 0.6, 0.3], [-0.3, 0.6, -0.3], [-0.3, -0.6, -0.3]])
        self.framework_points = np.array([])
        self.de2ra = pi/180

        self.pose_pub = rospy.Publisher(visualization_topic + "_pose",
                                                                            MarkerArray,
                                                                            queue_size = 1000)
        self.traj_pub = rospy.Publisher(visualization_topic + "_traj",
                                                                            MarkerArray,
                                                                            queue_size = 1000)
        
        self.robot_poses = MarkerArray()
        self.trajectory = MarkerArray()
    
    def pose_transformation(self, pose):
        traj_point = Point()
        translation = np.array([float(pose[1]), float(pose[2]), float(pose[3])])
        traj_point.x = translation[0]
        traj_point.y = translation[1]
        traj_point.z = translation[2]
        rotation = R.from_quat([float(pose[4]), float(pose[5]),
                                                        float(pose[6]), float(pose[7])]).as_matrix()

        self.framework_points = - np.matmul(rotation, 
                                    self.orig_framework_points.transpose()).transpose()
        for i in range(5):
            self.framework_points[i,:] += translation 
        return traj_point

    def add_frame(self, pose_dir, color_r = 1.0, color_g = 0.0, color_b = 0.0,):
        pose_marker = Marker()
        pose_marker.header.frame_id = "map"
        pose_marker.id = self.frame_id
        pose_marker.type = Marker.LINE_LIST
        pose_marker.scale.x = 0.05
        pose_marker.scale.y = 0.05
        pose_marker.scale.z = 0.05
        pose_marker.color.r = color_r
        pose_marker.color.g = color_g
        pose_marker.color.b = color_b
        pose_marker.color.a = 1.0

        traj_marker = Marker()
        traj_marker.header.frame_id = "map"
        traj_marker.id = self.frame_id
        traj_marker.type = Marker.LINE_STRIP
        traj_marker.scale.x = 0.1
        traj_marker.scale.y = 0.1
        traj_marker.scale.z = 0.1
        traj_marker.color.r = color_r
        traj_marker.color.g = color_g
        traj_marker.color.b = color_b
        traj_marker.color.a = 1.0
        poses = open(pose_dir, "r")
        for pose in poses:
            traj = self.pose_transformation(pose.split(" "))

            for i in range(len(self.framework_points)):
                for j in range(len(self.framework_points)):
                    if i != j:
                        first_point = Point()
                        first_point.x = self.framework_points[i][0]
                        first_point.y = self.framework_points[i][1]
                        first_point.z = self.framework_points[i][2]
                        pose_marker.points.append(first_point)
                        second_point = Point()
                        second_point.x = self.framework_points[j][0]
                        second_point.y = self.framework_points[j][1]
                        second_point.z = self.framework_points[j][2]
                        pose_marker.points.append(second_point)
            
            self.frame_id += 1
            traj_marker.points.append(traj)
            pose_marker.header.stamp = rospy.Time.now()
            traj_marker.header.stamp = rospy.Time.now()
            self.robot_poses.markers.append(pose_marker)
            self.trajectory.markers.append(traj_marker)

    def publish_msgs(self):
        self.pose_pub.publish(self.robot_poses)
        self.traj_pub.publish(self.trajectory)

if __name__ == '__main__':
    rospy.init_node("visualization_node")
    visualizerA = TrajectoryVisualizer("visualizationA")
    visualizerA.add_frame("../data/poseA_quaterniond.txt")

    visualizerB = TrajectoryVisualizer("visualizationB")
    visualizerB.add_frame("../data/poseB_quaterniond.txt", 0, 0, 1)

    visualizerA_noise = TrajectoryVisualizer("visualizationA_noise")
    visualizerA_noise.add_frame("../data/poseA_noise.txt", 1, 1, 0)

    visualizerB_noise = TrajectoryVisualizer("visualizationB_noise")
    visualizerB_noise.add_frame("../data/poseB_noise.txt", 0, 1, 1)

    visualizerA_opti = TrajectoryVisualizer("visualizationA_opti")
    visualizerA_opti.add_frame("../data/poseA_opti.txt", 1, 0, 1)

    visualizerB_opti = TrajectoryVisualizer("visualizationB_opti")
    visualizerB_opti.add_frame("../data/poseB_opti.txt", 1, 1, 1)
    loop_rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        visualizerA.publish_msgs()
        visualizerB.publish_msgs()
        visualizerA_noise.publish_msgs()
        visualizerB_noise.publish_msgs()
        visualizerA_opti.publish_msgs()
        visualizerB_opti.publish_msgs()
        loop_rate.sleep()