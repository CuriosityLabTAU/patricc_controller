import rospy
from dynamixel_hr_ros.msg import *
import time
from std_msgs.msg import String, Bool


class patricc_controller():

    def __init__(self):
        self.mode = {
            'gaze_face' : True,
            'gaze_motion': True,
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
        self.command.angle[4] = 3.5
        self.command.angle[6] = 1.7
        self.command.angle[3] = 1.95
        #print "command angle: ", self.command.angle
        self.command.angle[1] = 2.6
        self.command.speed = [1.5, 1.5, 2.5, 7, 5, 5, 5, 5]
        self.speed_default = [1.5, 1.5, 2, 7, 5, 5, 5, 5]
        self.wake_up_speed = [0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5]
        self.old_motion_command = self.command.angle
        self.old_face_command = self.command.angle


        rospy.init_node('patricc_controller', anonymous=True)
        rospy.Subscriber('/patricc_face_tracking',CommandPosition , self.face_tracking_callback)
        rospy.Subscriber('/patricc_motion_control', CommandPosition, self.motion_control_callback)
        rospy.Subscriber('/patricc_activation_mode', String, self.activation_mode_callback)
        self.publisher = rospy.Publisher("/dxl/command_position", CommandPosition, queue_size=10)
        enabler = rospy.Publisher("/dxl/enable", Bool, queue_size=10)
        time.sleep(1)
        enabler.publish(True)
        time.sleep(1)
        self.command.speed = self.wake_up_speed
        self.move_motors()
        self.command.speed = self.speed_default

        rospy.spin()

    def face_tracking_callback(self, data):
        if self.mode['gaze_face']:
            self.mode['face_coordinates'] = []
            for i, angle in enumerate(data.angle):
                if angle >= 0.0:
                    self.mode['face_coordinates'].append(i)
                    if i == 2:
                        self.command.angle[i] = angle
                    else:
                        self.command.angle[i] = self.old_motion_command[i]
                self.changed = True
                self.command.speed[2] = 1.6#data.speed[2]
            self.move_motors()
            #self.command.speed[2] = self.speed_default[2]
            self.old_face_command[2] = self.command.angle[2]



    def motion_control_callback(self, data):
        for i, angle in enumerate(data.angle):
            if angle >= 0.0:
                #if self.mode['face_tracking'] and i in self.mode['face_coordinates']:
                if self.mode['gaze_face'] and i==2:
                    self.command.angle[i] = self.old_face_command[2]
                else:
                    self.command.angle[i] = angle
                self.changed = True
        self.move_motors()
        self.old_motion_command = self.command.angle
        #print 'old motion control: ', self.old_motion_command



    def activation_mode_callback(self, data):
        msg = data.data.split('|')
        #print 'msg: ', msg
        #print 'mode before: ', self.mode
        self.mode['gaze_face'] = 'gaze_face' in msg
        self.mode['gaze_motion'] = 'gaze_motion' in msg
        #print 'mode after: ', self.mode

    def move_motors(self):
        if self.changed:
            self.publisher.publish(self.command)
            self.changed = False


patricc = patricc_controller()
