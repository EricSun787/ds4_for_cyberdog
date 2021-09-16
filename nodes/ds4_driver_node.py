#!/usr/bin/env python3
from ds4_for_cyberdog.logger import Logger
from ds4_for_cyberdog.controller_ros import ControllerRos

from ds4drv.backends import BluetoothBackend, HidrawBackend
from ds4drv.exceptions import BackendError

import rclpy
from threading import Thread

import signal
import sys


class SignalHandler(object):
    def __init__(self, controller):
        self.controller = controller

    def __call__(self, signum, frame):
        rclpy.logging.get_logger('SignalHandler').info('Shutting down...')
        self.controller.exit()
        rclpy.shutdown()
        sys.exit(0)


def main():
    rclpy.init()
    node = rclpy.create_node('ds4_driver_node')
    node.declare_parameter('device_addr', None)
    node.declare_parameter('backend', 'hidraw')
    device_addr = node.get_parameter('device_addr').value
    backend_type = node.get_parameter('backend').value

    controller = ControllerRos(node)

    sigint_handler = SignalHandler(controller)
    # Since backend.devices is a non-ROS iterator that doesn't consider
    # rclpy.is_shutdown(), the program freezes upon receiving SIGINT when
    # using rclpy.on_shutdown. Thus, we need to define our shutdown sequence
    # using signal.signal as is done in the original ds4drv script.
    signal.signal(signal.SIGINT, sigint_handler)

    if backend_type == 'bluetooth':
        backend = BluetoothBackend(Logger('backend'))
    else:
        backend = HidrawBackend(Logger('backend'))

    try:
        backend.setup()
    except BackendError as err:
        node.get_logger().error(err)
        rclpy.signal_shutdown(str(err))
        sys.exit(1)

    spin_thread = Thread(target=rclpy.spin, args=(node,))
    spin_thread.start()

    for device in backend.devices:
        node.get_logger().info('Connected to {0}'.format(device.name))
        if device_addr in (None, '', device.device_addr):
            controller.setup_device(device)
            if not controller.is_alive():
                controller.start()
            controller.loop.register_event('device-report', controller.cb_report)
        else:
            node.get_logger().error("...but it's not the one we're looking for :(")


if __name__ == '__main__':
    main()
