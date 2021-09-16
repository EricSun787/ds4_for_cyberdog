#!/usr/bin/env python3
import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from ds4_for_cyberdog.msg import Status
from motion_msgs.msg import SE3Velocity,SE3VelocityCMD
from motion_msgs.msg import ActionRequest


class StatusToTwist(object):
    def __init__(self,node):
        self._node = node
        self._node.declare_parameter('stamped', False)
        self._node.declare_parameter('frame_id', 'base_link')

        self._cyberCMD = SE3VelocityCMD
        self._cyberAction = ActionRequest

        self._action_show_pubed = False
        self._action_run_pubed = False
        self._action_stand_pubed = False
        self._action_sit_pubed = False


        self.param_dict = dict()
        param_types = ['inputs', 'scales']
        param_categories = ['velocity']
        param_axis = ['linear_x', 'linear_y', 'angular_y','angular_z']

        # We want to be able to declare each parameter type.
        # In the past this was not necc. but for rclpy you must
        # declare the parameter before you use it.
        for t in param_types:
            self.param_dict[t] = dict()
            for c in param_categories:
                self.param_dict[t][c] = dict()
                for a in param_axis:
                    self.param_dict[t][c][a] = dict()
                    self._node.declare_parameter("{}.{}.{}".format(t,c,a))
                    self.param_dict[t][c][a] = self._node.get_parameter("{}.{}.{}".format(t,c,a)).value

        self._inputs = self.param_dict['inputs']
        self._scales = self.param_dict['scales']

        self._attrs = []
        for attr in Status.__slots__:
            # add an underscore since ROS2 slots have an prepended underscore
            if attr.startswith('_axis_') or attr.startswith('_button_'):
                self._attrs.append(attr[1:])  # get rid of the prepended underscore

        
        self.cyber_cmd_pub = self._node.create_publisher(self._cyberCMD,'mi1036351/body_cmd',0)
        self.cyber_action_pub = self._node.create_publisher(self._cyberAction,'mi1036351/cyberdog_action',0)

        self._sub = self._node.create_subscription(Status, 'status', self.cb_status, 0)

    def cb_status(self, msg):
        """
        :param msg:
        :type msg: Status
        :return:
        """
        input_vals = {}
        for attr in self._attrs:
            input_vals[attr] = getattr(msg, attr)


        #print(input_vals)
        cmd_to_pub = self._cyberCMD()
        cmd_to_pub.velocity.timestamp = self._node.get_clock().now().to_msg()
        cmd_to_pub.sourceid = 2
        cmd_to_pub.velocity.frameid.id = 1

        cybercmd = cmd_to_pub
        for vel_type in self._inputs.keys():
            vel_vec = getattr(cybercmd, vel_type)
            for k, expr in self._inputs[vel_type].items():
                scale = self._scales[vel_type].get(k,1.0)
                if scale is None:
                    scale = 1.0
                try:
                    val = eval(expr, {}, input_vals)
                    setattr(vel_vec, k, scale * val)
                except NameError:
                    pass
        
        #print(input_vals['button_cross'])

        action_to_pub = self._cyberAction()

        if input_vals['button_square']:   #姿态展示
            action_to_pub.type = 2
            action_to_pub.request_id = 5
            action_to_pub.gait.gait = 4
            action_to_pub.timeout = 35
            if not self._action_show_pubed :
                self.cyber_action_pub.publish(action_to_pub)
                self._action_show_pubed = True
        else:
            self._action_show_pubed = False

        if input_vals['button_circle']:  #小跑
            action_to_pub.type = 2
            action_to_pub.request_id = 6
            action_to_pub.gait.gait = 7
            action_to_pub.timeout = 35
            if not self._action_run_pubed :
                self.cyber_action_pub.publish(action_to_pub)
                self._action_run_pubed = True
        else:
            self._action_run_pubed = False
        
        if input_vals['button_triangle']:  #站起
            action_to_pub.type = 2
            action_to_pub.request_id = 10
            action_to_pub.gait.gait = 3
            action_to_pub.timeout = 35
            if not self._action_stand_pubed :
                self.cyber_action_pub.publish(action_to_pub)
                self._action_stand_pubed = True
        else:
            self._action_stand_pubed = False
        
        if input_vals['button_cross']:  #趴下
            action_to_pub.type = 2
            action_to_pub.request_id = 9
            action_to_pub.gait.gait = 2
            action_to_pub.timeout = 35
            if not self._action_sit_pubed :
                self.cyber_action_pub.publish(action_to_pub)
                self._action_sit_pubed = True
        else:
            self._action_sit_pubed = False



        self.cyber_cmd_pub.publish(cmd_to_pub)
        
        
        
def main():
    rclpy.init()
    node = rclpy.create_node('ds4_twist')

    StatusToTwist(node)

    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
