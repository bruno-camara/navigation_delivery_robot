#! /usr/bin/env python
# import ros stuff
import rospy
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseGoal


publisher = rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size=1,latch=True)
move_base_goal = MoveBaseGoal()

roboState = [
  'in-point-a',
  'in-point-b',
  'going-to-point-a',
  'going-to-point-b',
  'between-a-and-b',
  'loading',
]

def read_instruction():
    file = open("src/d_hospital_navigation/src/server-test/from-server.txt", 'r')
    state = file.read()
    file.close()
    rospy.loginfo(state)
    return state



def publish_goal(position, orientation):

    move_base_goal.target_pose.pose.position.x = position[0]
    move_base_goal.target_pose.pose.position.y = position[1]
    move_base_goal.target_pose.pose.position.z = 0.0

    move_base_goal.target_pose.pose.orientation.x = orientation[0]
    move_base_goal.target_pose.pose.orientation.y = orientation[1]
    move_base_goal.target_pose.pose.orientation.z = orientation[2]
    move_base_goal.target_pose.pose.orientation.w = orientation[3]

    move_base_goal.target_pose.header.frame_id = 'map'

    goal = MoveBaseActionGoal()

    goal.goal = move_base_goal

    publisher.publish(goal)
    #rospy.spin()



def main():
    rospy.init_node('goal_publisher')
    rospy.loginfo("Initializing reading from server")
    state = read_instruction()
    rate = rospy.Rate(2)
    while not rospy.is_shutdown():
        rospy.loginfo("Rospy not shutdown")
        next_state = read_instruction()
        if state != next_state:
            state = next_state
            if state == roboState[2]:
                pos_x = -1.76558256149
                pos_y = -0.156761646271
                pos = [pos_x, pos_y]

                orientation_x = 0.0
                orientation_y = 0.0
                orientation_z = 0.0
                orientation_w = 1.0
                orientation = [orientation_x, orientation_y, orientation_z, orientation_w]
                publish_goal(pos, orientation)
                #rospy.loginfo("going-to-point-a")

            elif state == roboState[3]:
                pos_x = 29.5025749207
                pos_y = 0.71737909317
                pos = [pos_x, pos_y]

                orientation_x = 0.0
                orientation_y = 0.0
                orientation_z = 1.0
                orientation_w = 0.0
                orientation = [orientation_x, orientation_y, orientation_z, orientation_w]
                publish_goal(pos, orientation)
                #rospy.loginfo("going-to-point-a")
        rate.sleep()


if __name__ == '__main__':
    main()
