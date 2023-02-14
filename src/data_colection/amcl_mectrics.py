#! /usr/bin/env python
"""
    File:
        amcl_metrics.py
    Description:
        Taking mectrics to evaluate efficiency of AMCL algorythm
    Author:
        Pedro Croso <pedrocroso@usp.br>
    Data:
        02/2023
"""

#Save several pose arrays from AMCL and check for:
#   1 - How variance of distribution behaves
#   2 - How wrong is the final result



import rospy
import csv
import time

import tf
from geometry_msgs.msg import PoseArray

last_pose = []
last_time = 0.0
readings = 0


def handle_get_pose_array(my_array):
    """copies pose array into globalpython variable to be treated somewhere else

    Args:
        my_array: pose array from ROS
    """
    global last_pose, readings
    last_pose = my_array.poses
    print(last_pose[0])
    readings += 1
    return

def main():
    """Setup enviroment, reads one pose array every rospy.Rate(Hz) and the base frame used for navigation and put this data in a .csv file
    """
    #ROS enviroment setup
    rospy.init_node('get_pose_array')
    s =rospy.Subscriber('/particlecloud', PoseArray, handle_get_pose_array) #Setup subscriber to read the pose_array through a callback function
    listener = tf.TransformListener()   #Setup to listener of the tf transformations

    # CSV configuration
    header = ['time', 'p_position_x', 'p_position_y', 'p_position_z', 'p_orientation_x', 'p_orientation_y', 'p_orientation_z', 'p_orientation_w','base_trans_x', 'base_trans_y', 'base_rot_x', 'base_rot_y', 'base_rot_z', 'base_rot_w']
    f = open('amcl_mectrics.csv', 'w')
    writer = csv.writer(f)
    writer.writerow(header)


    rate = rospy.Rate(20.0)
    start = time.time()
    now = 0
    while (not rospy.is_shutdown()) and (now < 5):
        try:
            # Reads tf position of base
            (trans_base,rot_base) = listener.lookupTransform('map', 'base', rospy.Time(0))
            now = time.time() - start
            for pose in last_pose:
                #Creates and writes .csv row
                data = [now, pose.position.x, pose.position.y, pose.position.z, pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w, trans_base[0], trans_base[1], rot_base[0], rot_base[1], rot_base[2], rot_base[3]]
                writer.writerow(data)
            print(readings)

        except:
            print("Error readind pose or tf")

        rate.sleep()

    f.close()
    print("Closed file")
    return


if __name__ == '__main__':
    main()
