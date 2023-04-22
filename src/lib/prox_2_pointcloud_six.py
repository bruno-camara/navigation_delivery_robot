#! /usr/bin/env python
# import ros stuff

import rospy
import numpy as np
import std_msgs.msg
from sensor_msgs.msg import PointCloud2
from distance_sensors import DistanceSensor
import sensor_msgs.point_cloud2 as pcl2


#posicao dos sensores no sistema
back_right_position = [-0.275, -0.085, -0.44]
back_left_position = [-0.275, 0.085, -0.44]

right_position = [0.0, -0.256, -0.44]
left_position = [0.0, 0.256, -0.44]

#Angulo dos sensores (graus)

back_right_angle = [0.0, 180.0]
back_left_angle = [0.0, 180.0]

right_angle = [0.0, -90.0]
left_angle = [0.0, 90.0]

#pontos obtidos no mapa
back_right_point = [0.0, 0.0, 0.0]
back_left_point = [0.0, 0.0, 0.0]

right_point = [0.0, 0.0, 0.0]
left_point = [0.0, 0.0, 0.0]

# System thresholds

lateral_th = 1.8
front_th = 1.3
back_th = 1.5




def conect_points(point_1, point_2, total_points):
    #vetor_diretor = np.array(point_2) - np.array(point_1)
    del_x = (point_2[0]-point_1[0])
    del_y = (point_2[1]-point_1[1])
    del_z = (point_2[2]-point_1[2])
    points = []
    for i in range(total_points):
        #new_point = np.array(point_1) + vetor_diretor/float(i)
        #points.append(list(new_point))
        points.append([point_1[0] + (del_x * float(i))/total_points, point_1[1] + (del_y * float(i))/total_points, point_1[2] + (del_z * float(i))/total_points])
    return points

def add_height(height, points):
    new_points = []
    items = len(points)
    for i in range(20):
        for j in range(items):
            new_point = list(points[j])
            new_point[2] += height*(float(i)/20)
            new_points.append(new_point)
    return new_points



def main():
    rospy.init_node('range_2_point_cloud')

    back_l = DistanceSensor('distance_sensor_back_left',back_left_angle, back_left_position)
    back_r = DistanceSensor('distance_sensor_back_right',back_right_angle, back_right_position)
    front_l = DistanceSensor('distance_sensor_left',left_angle, left_position)
    front_r = DistanceSensor('distance_sensor_right',right_angle, right_position)

    back_l.initialise()
    back_r.initialise()
    front_l.initialise()
    front_r.initialise()

    pcl_pub = rospy.Publisher("/d_hospital/point_cloud", PointCloud2, queue_size=10)

    rospy.loginfo("Initializing sample pcl2 publisher node...")


    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        back_right_point = back_r.get_point_detected()
        back_left_point = back_l.get_point_detected()

        right_point = front_r.get_point_detected()
        left_point = front_l.get_point_detected()
        cloud_points = []

        # Verify OUT OF RANGE

        in_range_back_left = True
        in_range_back_right = True
        in_range_left = True
        in_range_right = True

        if(back_l.get_distance() < 0):
            in_range_back_left = False

        if(back_r.get_distance() < 0):
            in_range_back_right = False


        if(front_l.get_distance() < 0):
            in_range_left = False

        if(front_r.get_distance() < 0):
            in_range_right = False

        try:
            # Connect left points

            if (back_l.get_distance() < lateral_th) and (front_l.get_distance() < lateral_th) and (in_range_back_left and in_range_left):
                cloud_points += conect_points(back_left_point, left_point, 20)

            #  Connect right points
            if (front_r.get_distance() < lateral_th) and (back_r.get_distance() < lateral_th) and (in_range_back_right and in_range_right):
                cloud_points += conect_points(right_point, back_right_point, 20)


            # Connect back points
            if (back_l.get_distance() < 1.8 and back_r.get_distance() < 1.8) and (in_range_back_left and in_range_back_right):
                cloud_points += conect_points(back_left_point, back_right_point, 10)


            # Add single points
            if (back_l.get_distance() < lateral_th)  and (front_l.get_distance() >= lateral_th) and (in_range_back_left):
                cloud_points.append(back_left_point)

            if (front_l.get_distance() < lateral_th) and (back_l.get_distance() >= lateral_th) and (in_range_left):
                cloud_points.append(left_point)

            if (front_r.get_distance() < lateral_th) and (back_r.get_distance >= lateral_th) and (in_range_right):
                cloud_points.append(right_point)

            if (back_r.get_distance < lateral_th) and (front_r.get_distance() >= lateral_th) and (in_range_back_right):
                cloud_points.append(back_left_point)


            cloud_points = add_height(1.5, cloud_points)


            header = std_msgs.msg.Header()
            header.stamp = rospy.Time.now()
            header.frame_id = 'base'

            #create pcl from points
            scaled_polygon_pcl = pcl2.create_cloud_xyz32(header, cloud_points)
            #publish
            rospy.loginfo("happily publishing sample pointcloud.. !")

            pcl_pub.publish(scaled_polygon_pcl)
        except ZeroDivisionError:
            print (ZeroDivisionError)

        rate.sleep()


if __name__ == '__main__':
    main()
