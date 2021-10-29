#! /usr/bin/env python
# import ros stuff

import rospy
import numpy as np

from sensor_msgs.msg import LaserScan, Range
from geometry_msgs.msg import Twist


from nav_msgs.msg import Odometry
from tf import transformations
from std_srvs.srv import *

from lib.lidar_sensor import LidarSensor

import math
pub = None

state = 0
state_dict_ = {
    0: 'find the wall',
    1: 'turn left',
    2: 'turn right',
    3: 'adjust to left',
    4: 'adjust to left soft',
    5: 'adjust to right',
    6: 'adjust to right soft',
    7: 'all good',
}

#LIDAR eh origem do sistema
front_vector_lidar = np.array([0.0, 0.0])
back_vector_lidar = np.array([0.0, 0.0])
# LIDAR Object. Comment first and uncomment second for simulation
lidar = LidarSensor('d_hospital/laser/scan', 180, 90)
#lidar = LidarSensor('d_hospital/laser/scan')


#vetor da origem a parede na frente pelo sensor de proximidade
front_vector_prox = np.array([0.0, 0.0])

#vetor da origem a parede atras pelo sensor de proximidade
bottom_vector_prox = np.array([0.0, 0.0])

#posicao dos sensores no sistema
bottom_position = np.array([-0.22, -0.25])
front_position = np.array([-0.22, 0.25])
#todos os sensores sao da esquerda

wall_direction = np.array([0.0, 0.0])
wall_direction_prox = np.array([0.0, 0.0])

#distance robot will keep from wall
best_distance = 0.8
closest_point = 0.0

#Uncomment 720 for simulation
regions = [0.0] * 180
#regions = [0.0] * 720


next_turn_direction = 0


#def clbk_lidar(msg):
#    global regions, front_vector_lidar, back_vector_lidar, closest_point
#    for i in range(0, 720, 1):
#        regions[i] = min(msg.ranges[i], 10)
#    print regions
#    distance_to_wall = min(min(regions[494 : 585]), 20.0)
#    closest_point = distance_to_wall*np.array([np.cos(math.radians((regions.index(distance_to_wall, 494, 585)-180)/2)),
#                    np.sin(math.radians((regions.index(distance_to_wall, 494, 585)-180)/2))])
#
#    front_vector_lidar = np.array([min(regions[520:540]), 0.0]) #[539 : 541]
#    ang_back = 30.0
#    back_vector_lidar = np.array([regions[int(2*ang_back+540)]*np.cos((ang_back*np.pi)/180), regions[int(2*ang_back+540)]*np.sin((ang_back*np.pi)/180)])





def clbk_prox_bottom(msg):
    global bottom_vector_prox, bottom_position
    bottom_vector_prox = np.array([-msg.range, 0.0]) + bottom_position



def clbk_prox_front(msg):
    global front_vector_prox, front_position
    front_vector_prox = np.array([-msg.range, 0.0]) + front_position



def find_wall_direction():
    global bottom_vector_prox, front_vector_prox, wall_direction, wall_direction_prox, front_vector_lidar, back_vector_lidar, regions, closest_point


    #wall_direction = front_vector_prox - bottom_vector_prox
    distancia_normal = lidar.get_normal_distance_left() #sum(regions[538 : 542])/5
    wall_direction = lidar.get_closest_point(22.5, 67.5) - lidar.get_normal_point_left() #closest_point - np.array([distancia_normal, 0.0])
    wall_direction_prox = front_vector_prox - bottom_vector_prox
    magnitude = np.linalg.norm(wall_direction)
    magnitude_prox = np.linalg.norm(wall_direction_prox)
    if magnitude != 0:
        wall_direction = wall_direction / magnitude
        #print "Wall direction and angle"
        #print wall_direction
        #print math.degrees(math.asin(wall_direction[1]))
    else:
        print ("magnitude == 0")

    if magnitude_prox != 0:
        wall_direction_prox = wall_direction_prox / magnitude_prox
        #print "Wall direction prox and angle"
        #print wall_direction_prox
        #print math.degrees(math.asin(wall_direction_prox[0]))
    else:
        print ("magnitude_prox == 0")
    #rospy.loginfo(wall_direction)

def find_next_corner():
    global wall_direction, wall_direction_prox, regions, front_vector_prox, bottom_vector_prox, next_turn_direction, best_distance, front_vector_lidar, back_vector_lidar

    angle = math.asin(wall_direction[1])
    angle_prox = math.asin(wall_direction_prox[0])
    #wall_direction_prox = front_vector_prox - bottom_vector_prox

    #magnitude = np.linalg.norm(wall_direction_prox)
    #if magnitude != 0:
    #    wall_direction_prox = wall_direction_prox / magnitude

    #angle_prox = math.asin(-wall_direction_prox[0])

    turn_dict = {
        "not found" : 0,
        "left" : 1,
        "right" : 2,
    }
    next_turn_direction = 0

    #try:
    distance_threshold = best_distance / 8.0
    medium_distance = np.linalg.norm(lidar.get_closest_point(-180, 180))
    distance_error = medium_distance - best_distance
    print (lidar.get_closest_distance(-22.5, 22.5))


    if lidar.get_closest_distance(90, 93) > best_distance * 1.5 and lidar.get_closest_point_angle(22.5, 112.5) >= 90:
        print ("need to turn left")
        next_turn_direction = 1
    elif lidar.get_closest_distance(-22.5, 22.5) < 0.9 or lidar.get_closest_distance(-112.5, -23) <0.3:
        print ("need to turn right")

        next_turn_direction = 2
    else:
        next_turn_direction = 0
    #except ValueError as error:
    #    print ("A value error ocoured")
    #    print (error)




def need_to_turn(): # Usar metodo vetorial: com o lidar estimar o angulo e distancia da parede supondo infinita, e achar a quina/fim
    global wall_direction, regions, best_distance, next_turn_direction
    turns = {
        "none"  : 0,
        "left"  : 1,
        "right" : 2,
    }
    find_next_corner()
    if (next_turn_direction == 0):# | (next_corner[1] > best_distance * 1.0):
        print ("no turns detected")
        return 0
    elif next_turn_direction == 1:
        print ("need to turn left")
        return 1
    elif next_turn_direction == 2:
        print ("need to turn right")
        return 2
    else:
        print ("Unknown turn state: ")
        print (next_turn_direction)


def change_state(new_state):
    global state, state_dict_
    if new_state is not state:
        print ("Wall follower - [%s] - [%s]" % (new_state, state_dict_[new_state]))
        state = new_state


def is_in_window():
    global wall_direction, wall_direction_prox, front_vector_prox, bottom_vector_prox

    lidar_angle = math.degrees(math.asin(wall_direction[1]))
    lidar_distance = lidar.get_closest_distance(72.5, 107) - front_position[0] #min(regions[505:574]) - front_position[0]
    #print lidar_distance

    prox_angle = math.degrees(math.asin(wall_direction_prox[0]))
    #print -front_vector_prox[0], -bottom_vector_prox[0]
    prox_medium_distance = ((-front_vector_prox[0] - bottom_vector_prox[0]) / 2) - front_position[0]

    if min((-front_vector_prox[0]), (-bottom_vector_prox[0])) - front_position[0] < ((lidar_distance) / 1.2) and min((-front_vector_prox[0], -bottom_vector_prox[0])) < (best_distance * 1.25): #and prox_medium_distance > best_distance / 1.5) or (False)
        #print prox_medium_distance
        return True
    return False

def take_action(): #works only if wall is already found
    global pub, wall_direction, wall_direction_prox, bottom_vector_prox, front_vector_prox, state, best_distance, closest_point
    #crating action parameters
    min_angle  = 0.05
    distance_threshold = best_distance / 15.0

    if 1: #not(is_in_window()): #follow using lidar
        angle  = -math.asin(wall_direction[1])
        print ("Using lidar")
        print (angle)
        medium_distance = lidar.get_closest_distance(67, 112.5)#np.linalg.norm(closest_point)
        print (medium_distance)
    else:                   #follow using proximity
        angle  = -math.asin(wall_direction_prox[0])
        print ("Using prox")
        print (angle)
        medium_distance = (-front_vector_prox[0] - bottom_vector_prox[0]) / 2
        print (medium_distance)

    distance_error = medium_distance - best_distance
    #decision making
    state_description = ""


    if ((distance_error < -distance_threshold) and (angle < -min_angle)):
        state_description = "Way too close to wall"
        change_state(2)
    elif ((distance_error < -distance_threshold) and ((angle < 0) & (angle > -min_angle))):
        state_description = "getting too close to wall"
        change_state(5)
    elif (distance_error < distance_threshold) and (angle > 0):
        state_description = "close but getting far"
        change_state(7)
    elif ((distance_error < 0) and (distance_error > -distance_threshold)) & (angle < 0):
        state_description = "kinda close and gettin closer"
        change_state(6)
    elif distance_error < 0 :
        state_description = "kinda close but getting far"
        change_state(7)
    elif (distance_error > distance_threshold) and (angle > 0) :
        state_description = "getting too far"
        change_state(3)
    elif distance_error > distance_threshold:
        state_description = "Far but getting close"
        change_state(4)
    elif ((distance_error > 0) and (distance_error < distance_threshold)) & (angle > 0):
        state_description = "kinda far and getting far"
        change_state(4)
    elif distance_error > 0:
        state_description = "kinda far but getting close"
        change_state(7)
    else:
        state_description = "unknown state: \n\tmedium distance = %s\n\tangle = %s" % (medium_distance, angle)



    print (state_description)


def find_wall():
    msg = Twist()
    msg.linear.x = 0.2 * 1
    msg.angular.z = 0.3 * 1
    return msg


def turn_right():
    msg = Twist()
    msg.linear.x = 0.02 * 1
    msg.angular.z = -0.2 * 1
    return msg


def turn_left():
    msg = Twist()
    msg.linear.x = 0.04 * 1
    msg.angular.z = 0.25 * 1
    return msg


def adjust_left():
    msg = Twist()
    msg.linear.x = 0.15 * 1
    msg.angular.z = 0.2 * 1
    return msg


def adjust_left_soft():
    msg = Twist()
    msg.linear.x = 0.15 * 1
    msg.angular.z = 0.2 * 1
    return msg


def adjust_right():
    msg = Twist()
    msg.linear.x = 0.2 * 1
    msg.angular.z = -0.1 * 1
    return msg


def adjust_right_soft():
    msg = Twist()
    msg.linear.x = 0.2 * 1
    msg.angular.z = -0.05 * 1
    return msg


def go_straight():
    msg = Twist()
    msg.linear.x = 0.2 * 1
    msg.angular.x = 0.0 * 1
    return msg


def main():
    global pub, bottom_vector_prox, front_vector_prox, wall_direction, state, front_vector_lidar, back_vector_lidar, lidar

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    rospy.init_node('reading_laser')
    #sub_lidar = rospy.Subscriber('d_hospital/laser/scan', LaserScan, clbk_lidar)

    lidar.initialise()
    print(lidar)

    sub_prox_bottom = rospy.Subscriber('distance_sensor_back_left', Range, clbk_prox_bottom)
    sub_prox_front = rospy.Subscriber('distance_sensor_front_left', Range, clbk_prox_front)

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():


        find_wall_direction()
        direction = need_to_turn()
        if direction == 1:
            change_state(1)
        elif direction == 2:
            change_state(2)
        else:
            take_action()

        msg = Twist()

        if state == 0:
            msg =find_wall()
        elif state == 1:
            msg = turn_left()
        elif state == 2:
            msg = turn_right()
        elif state == 3:
            msg = adjust_left()
        elif state == 4:
            msg = adjust_left_soft()
        elif state == 5:
            msg = adjust_right()
        elif state == 6:
            msg = adjust_right_soft()
        elif state == 7:
            msg = go_straight()
        else:
            print ("unknown state: ")
            print (state)


        pub.publish(msg)

        rate.sleep()

    #rospy.spin()


if __name__ == '__main__':
    main()
