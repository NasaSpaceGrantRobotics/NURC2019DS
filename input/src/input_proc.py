#!/usr/bin/env python3
"""
Xbox 360 Wired Controller Indices

Buttons
Index | Button | Function
  0   |   A    | Unassigned
  1   |   B    | Unassigned
  2   |   X    | Unassigned
  3   |   Y    | Unassigned
  4   |   LB   | Rotate claw CCW
  5   |   RB   | Rotate claw CW
  6   |  Back  | Cycle through pitch, yaw, and roll on  LS-X
  7   |  Start | Toggle light
  8   |  Power | E-Stop
  9   |   LS   | Unassigned
  10  |   RS   | Unassigned

Axis
Index | Button | Function
  0   |  LS-X  | Rotate (yaw by default; see Back function)
  1   |  LS-Y  | Ascend/Descend
  2   |   LT   | Close claw
  3   |  RS-X  | Translate local x-axis
  4   |  RS-Y  | Translate local y-axis
  5   |   RT   | Open claw
  6   | DPAD-X | Roll (right is clockwise)
  7   | DPAD-Y | Pitch (up is counterclockwise/up)
"""
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from enum import Enum


class InputProcessor:
    class Button(Enum):
        a = 0
        b = 1
        x = 2
        y = 3
        lb = 4
        rb = 5
        select = 6
        start = 7
        power = 8
        ls = 9
        rs = 10
    
    class Axis(Enum):
        ls_x = 0
        ls_y = 1
        lt = 2
        rs_x = 3
        rs_y = 4
        rt = 5
        dpad_x = 6
        dpad_y = 7

    class RotAxis(Enum):
        pitch = 0
        roll = 1
        yaw = 2

    def __init__(self):
        self.drive_pub = rospy.Publisher('target_drive', Twist, queue_size=10)
        self.target_drive_msg = Twist()
        self.target_drive_msg.linear.x = self.target_drive.linear.y = self.target_drive.linear.z = 0.0
        self.target_drive_msg.angular.x = self.target_drive.angular.y = self.target_drive.angular.z = 0.0
        self.target_drive_linear = [0.0, 0.0, 0.0]
        self.target_drive_rot = [0.0, 0.0, 0.0]

        self.__rot_select = self.RotAxis.yaw
        self.__rot_select_button_prev = False

        self.claw_pub = rospy.Publisher('target_claw', Float32, queue_size=10)
        self.target_claw_msg = Twist()
        self.target_claw_msg.linear.x = self.target_drive.linear.y = self.target_drive.linear.z = 0.0
        self.target_claw_msg.angular.x = self.target_drive.angular.y = self.target_drive.angular.z = 0.0
        self.target_claw_linear = [0.0, 0.0, 0.0]
        self.target_claw_rot = [0.0, 0.0, 0.0]

        self.e_stop_pub = rospy.Publisher('e_stop', Bool, queue_size=10)
        self.e_stop_msg = Bool()
        self.e_stop_msg.data = False
        self.__e_stop = False

        self.light_pub = rospy.Publisher('light', Bool, queue_size=10)
        self.light_msg = Bool()
        self.light_msg.data = False
        self.__light_button_prev = False
        self.__target_light_state = False

    @staticmethod
    def __bound_number(number, bounds):
        if number < bounds[0]:
            number = bounds[0]
        elif number > bounds[1]:
            number = bounds[1]
    
        return number

    def joy_callback(self, data):
        buttons = data.buttons
        axes = data.axes

        # begin by processing the emergency stop, which can only be cleared through manual software reset
        self.e_stop = buttons[self.Button.power]
        if self.e_stop:
            self.target_drive_linear = self.target_drive_rot = [0.0, 0.0, 0.0]
            self.target_drive_msg.linear.x = self.target_drive.linear.y = self.target_drive.linear.z = 0.0
            self.target_drive_msg.angular.x = self.target_drive.angular.y = self.target_drive.angular.z = 0.0

            self.target_claw_linear = [0.0, 0.0, 0.0]
            self.target_claw_rot = [0.0, 0.0, 0.0]
            self.target_claw_msg.linear.x = self.target_drive.linear.y = self.target_drive.linear.z = 0.0
            self.target_claw_msg.angular.x = self.target_drive.angular.y = self.target_drive.angular.z = 0.0

            self.e_stop_msg.data = True

            self.light_msg.data = \
                self.__target_light_state = self.__light_button_prev = False
        else:  # Not in emergency stop state
            self.e_stop_msg.data = False

            self.__update_light_state(buttons[self.Button.start])
            self.light_msg.data = self.__target_light_state

            self.target_drive_linear[0] = self.__bound_number(axes[self.Axis.rs_x], [-1.0, 1.0])
            self.target_drive_linear[1] = self.__bound_number(-axes[self.Axis.rs_y], [-1.0, 1.0])
            self.target_drive_linear[2] = self.__bound_number(-axes[self.Axis.ls_y], [-1.0, 1.0])
            self.target_drive_msg.linear.x = self.target_drive_linear[0]
            self.target_drive_msg.linear.y = self.target_drive_linear[1]
            self.target_drive_msg.linear.z = self.target_drive_linear[2]

            self.target_drive_rot[0] = self.__bound_number(axes[self.Axis.dpad_y], [-1.0, 1.0])
            self.target_drive_rot[1] = self.__bound_number(-axes[self.Axis.dpad_x], [-1.0, 1.0])
            self.target_drive_rot[2] = 0.0
            self.__update_rot_select(buttons[self.Button.select])
            selected_axis_input = self.__bound_number(-axes[self.Axis.ls_y], [-1.0, 1.0])
            if abs(selected_axis_input) > 0.05:
                if self.__rot_select == self.RotAxis.pitch:
                    self.target_drive_rot[0] = selected_axis_input
                elif self.__rot_select == self.RotAxis.roll:
                    self.target_drive_rot[1] = selected_axis_input
                elif self.__rot_select == self.RotAxis.yaw:
                    self.target_drive_rot[2] = selected_axis_input
            self.target_drive_msg.angular.x = self.target_drive_rot[0]
            self.target_drive_msg.angular.y = self.target_drive_rot[1]
            self.target_drive_msg.angular.z = self.target_drive_rot[2]

            self.target_claw_linear[0] = self.__bound_number(-axes[self.Axis.lt] + axes[self.Axis.rt], [-1.0, 1.0])
            self.target_claw_msg.linear.x = self.target_claw_linear

            if buttons[self.Button.lb]:
                self.target_claw_rot[1] = 1.0
            elif buttons[self.Button.rb]:
                self.target_claw_rot[1] = -1.0
            else:
                self.target_claw_rot[1] = 0.0
            self.target_claw_msg.angular.y = self.target_claw_rot[1]

        # publish constructed ROS messages to their respective topics
        self.drive_pub.publish(self.target_drive_msg)
        self.claw_pub.publish(self.target_claw_msg)
        self.e_stop_pub.publish(self.e_stop_msg)
        self.light_pub.publish(self.light_msg)
    
    def start(self):
        rospy.Subscriber('joystick', Joy, self.joy_callback)

        rospy.init_node('input_proc')
        rospy.spin()

    # these decorators prohibit e_stop state from being cleared without manual system software reset
    @property
    def e_stop(self):
        return self.__e_stop

    @e_stop.setter
    def e_stop(self, value):
        if self.__e_stop:
            pass
        else:
            self.__e_stop = value

    @property
    def target_light_state(self):
        return self.__target_light_state

    @target_light_state.setter
    def target_light_state(self, value):
        pass

    def __update_light_state(self, current_light_input):
        if (self.__light_button_prev == False) and (current_light_input == True):
            self.__target_light_state = not self.__target_light_state
        self.__light_button_prev = current_light_input

    def __update_rot_select(self, rot_select_button_current):
        if (not self.__rot_select_button_prev) and rot_select_button_current:
            if self.__rot_select == self.RotAxis.pitch:
                self.__rot_select = self.RotAxis.roll
            if self.__rot_select == self.RotAxis.roll:
                self.__rot_select = self.RotAxis.yaw
            elif self.__rot_select == self.RotAxis.yaw:
                self.__rot_select = self.RotAxis.pitch
        self.__rot_select_button_prev = rot_select_button_current


if __name__ == "__main__":
    try:
        input_proc = InputProcessor()
        input_proc.start()
    except rospy.ROSInterruptException:
        pass
