import rospy
from dynamixel_hr_ros.msg import *
import time
from std_msgs.msg import String, Bool


class patricc_controller():

    def __init__(self):
        self.mode = {
            'face_tracking' : True,
            'motion_control': True,
            'face_coordinates': []
        }
        self.changed = True

        self.command = CommandPosition()
        self.robot_angle_range = [[0.0, 5.0],  # [1.1, 3.9],
                                  [2.8, 1.6],
                                  [2, 3.3], [2.2, 2.5],
                                  [4.1, 0.9], [1.3, 3],
                                  [1, 4.1], [2.5, 3.75]]
        self.command.id = [i for i in range(1, 9)]
        self.command.angle = [(self.robot_angle_range[i][0] + self.robot_angle_range[i][1]) / 2.0 for i in range(8)]
        self.command.angle[1] = 2.6
        self.command.speed = [1.5, 1.5, 2, 7, 5, 5, 5, 5]


        rospy.init_node('patricc_controller', anonymous=True)
        rospy.Subscriber('/patricc_face_tracking',CommandPosition , self.face_tracking_callback)
        rospy.Subscriber('/patricc_motion_control', CommandPosition, self.motion_control_callback)
        rospy.Subscriber('/patricc_activation_mode', String, self.activation_mode_callback)
        self.publisher = rospy.Publisher("/dxl/command_position", CommandPosition, queue_size=10)
        enabler = rospy.Publisher("/dxl/enable", Bool, queue_size=10)
        time.sleep(1)
        enabler.publish(True)
        time.sleep(1)

        self.move_motors()

        rospy.spin()

    def face_tracking_callback(self, data):
        if self.mode['face_tracking']:
            self.mode['face_coordinates'] = []
            for i, angle in enumerate(data.angle):
                if angle >= 0.0:
                    self.mode['face_coordinates'].append(i)
                    self.command.angle[i] = angle
                    self.changed = True
            self.move_motors()

    def motion_control_callback(self, data):
        if self.mode['motion_control']:
            for i, angle in enumerate(data.angle):
                if angle >= 0.0:
                    if self.mode['face_tracking'] and i in self.mode['face_coordinates']:
                        pass
                    else:
                        self.command.angle[i] = angle
                        self.changed = True
            self.move_motors()

    def activation_mode_callback(self, data):
        msg = data.data.split('|')
        self.mode['face_tracking'] = 'face_tracking' in msg
        self.mode['motion_control'] = 'motion_control' in msg

    def move_motors(self):
        if self.changed:
            self.publisher.publish(self.command)
            self.changed = False


patricc = patricc_controller()
