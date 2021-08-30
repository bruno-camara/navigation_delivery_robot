import rospy
from lib.motor import MotorControl
from lib.serial_communication import SerialCommunication

CONTROL_RATE = 60  # Hz

MIN = -1
MAX = 1

NEW_MIN = 1
NEW_MAX = 1023

PORT = '/dev/ttyUSB0'

def map(value,min,max,new_min,new_max):
    new_value = (value-min)*(new_max-new_min)
    new_value /= (max-min)
    new_value += min
    return new_value

constrain = lambda val, min_val, max_val: min(max_val, max(min_val, val))
communication = SerialCommunication(PORT)

def robot_manual_controller(debug = False):
    #motor delcaration
    rospy.init_node('test_robot_manual_controller', anonymous=True)
    rate = rospy.Rate(CONTROL_RATE)
    motor_back=MotorControl("/cmd_vel")
    motor_back.initialise()

    #serial comunication
    communication.initialise()

    #parado
    info_serial_vel = 511
    info_serial_rot = 511

    isvel = True

    while not rospy.is_shutdown():
        if (motor_back.get_velocity()._type == 'geometry_msgs/Vector3'):
            info_serial_vel = int(map(motor_back.get_velocity().x,MIN,MAX,NEW_MIN,NEW_MAX))
            info_serial_rot = int(map(motor_back.get_rotation().z,MIN,MAX,NEW_MIN,NEW_MAX))

            info_serial_vel = constrain(info_serial_vel,NEW_MIN,NEW_MAX)
            info_serial_rot = constrain(info_serial_rot,NEW_MIN,NEW_MAX)
            
        else:
            info_serial_vel = 511 #parado
            info_serial_rot = 511 #parado

        if(debug == True):
                print("velocity",str(info_serial_vel)+'0')
                print("rotation",str(info_serial_rot)+'1')

        if (isvel == True):
            communication.send_data(str(info_serial_vel)+'0')
            isvel = not isvel
        else:
            #communication.send_data(str(info_serial_rot)+'0')
            communication.send_data(str(info_serial_rot)+'1')
            isvel = not isvel
    

if __name__ == "__main__":
    try:
        robot_manual_controller(debug = True)
        pass
    except rospy.ROSInterruptException:
        communication.finalize()
        pass

    finally:
        pass