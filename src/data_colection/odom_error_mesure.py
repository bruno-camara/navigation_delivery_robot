#! /usr/bin/env python
"""
    File:
        odom_error_measure.py
    Description:
        Reads position of base frame and odom frame and saves in a csv file. Usefull for odometry performance evaluation.
    Author:
        Pedro Croso <pedrocroso@usp.br>
    Data:
        02/2023
"""

import rospy
import csv

import tf


def main():
    rospy.init_node('Odometry_measurments')
    listener = tf.TransformListener()
    header = ['odom_trans_x', 'odom_trans_y', 'odom_rot_x', 'odom_rot_y', 'odom_rot_z', 'odom_rot_w', 'base_x_trans', 'base_y_trans', 'base_rot_x', 'base_rot_y', 'base_rot_z', 'base_rot_w']
    header_simple = ['odom_trans_x', 'odom_trans_y', 'base_x_trans', 'base_y_trans']


    f = open('odom_error.csv', 'w')
    writer = csv.writer(f)
    writer.writerow(header_simple)

    rate = rospy.Rate(20.0)
    i = 0
    while i < 500 and (not rospy.is_shutdown()):
        try:
            (trans_odom,rot_odom) = listener.lookupTransform('map', 'odom', rospy.Time(0))
            (trans_base,rot_base) = listener.lookupTransform('map', 'base', rospy.Time(0))
            print("Odom:")
            print(rot_odom)
            print("Base:")
            print(rot_base)
            print("\n \n")

            data = [trans_odom[0], trans_odom[1], trans_base[0], trans_base[1]]
            writer.writerow(data)
            i += 1
        except:
            print("Error readind tf")

        rate.sleep()

    f.close()


if __name__ == '__main__':
    main()
