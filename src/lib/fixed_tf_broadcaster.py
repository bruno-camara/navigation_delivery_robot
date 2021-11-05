#!/usr/bin/env python
import roslib
#roslib.load_manifest('learning_tf')

import rospy
import tf
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3




velocity_msg= 0.0 #Twist()
rot_msg = 0.0 # Twist()




def main():
    rospy.init_node('fixed_tf_broadcaster')
    br = tf.TransformBroadcaster()
    br1 = tf.TransformBroadcaster()
    br2 = tf.TransformBroadcaster()

    odom_pub = rospy.Publisher("odom", Odometry, queue_size=1)


    ## Como n tem dado de odometria, sera usado o comando no lugar
    rospy.Subscriber("/cmd_vel", Twist, _callback)

    x = 0.0
    y = 0.0
    th = 0.0

    vx = 0.0  ### Implementar metodo para coletar vx
    vth = 0.0 ### Implementar metodo para colatar vth

    current_time = rospy.Time.now()
    last_time = rospy.Time.now()

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():

        #rospy.spin()

        #Criar parametrizacao para posicao inicial no mapa
        br.sendTransform((0.0, 0.0, 1.0),
                        (0.0, 0.0, 0.0, 1.0),
                        rospy.Time.now(),
                        "odom",
                        "map")

        br1.sendTransform((0.0, 0.0, 0.7),
                        (0.0, 0.0, 0.0, 1.0),
                        rospy.Time.now(),
                        "laser",
                        "base")



        vx = velocity_msg#.linear.x
        vth = rot_msg#.angular.z

        current_time = rospy.Time.now()

        dt = (current_time - last_time).to_sec()
        delta_x = (vx * math.cos(th)) * dt
        delta_y = (vx * math.sin(th)) * dt
        delta_th = vth * dt

        x += delta_x
        y += delta_y
        th += delta_th

        odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)

        br2.sendTransform((x, y, 0.0),
                        odom_quat,
                        current_time,
                        "base",
                        "odom")

        ## This will publish an Odom message.
        ## Intresting to have control ESP32 publishing this with driver info

        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"

        # set the position
        odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))

        # set the velocity
        odom.child_frame_id = "base"
        odom.twist.twist = Twist(Vector3(vx, 0, 0), Vector3(0, 0, vth))

        # publish the message
        odom_pub.publish(odom)

        last_time = current_time


        rate.sleep()

def _callback(data):
    global velocity_msg, rot_msg
    velocity_msg = data.linear.x
    rot_msg = data.angular.z
    return


if __name__ == '__main__':
    main()
