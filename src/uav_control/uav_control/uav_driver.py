import math
import rclpy
from rclpy.qos import qos_profile_sensor_data
from pprint import pprint, pformat
from webots_interfaces.msg import Propellars
from sensor_msgs.msg import Image


class UAVDriver:

    def init(self, webots_node, properties):

        self.__robot = webots_node.robot

        # Propellers
        self.__propellers = [
            self.__robot.getDevice('front right propeller'),
            self.__robot.getDevice('front left propeller'),
            self.__robot.getDevice('rear right propeller'),
            self.__robot.getDevice('rear left propeller')
        ]
        for propeller in self.__propellers:
            propeller.setPosition(float('inf'))
            propeller.setVelocity(0)

        # display
        self.__display = self.__robot.getDevice("grid")

        # ROS interface
        rclpy.init(args=None)
        self.__node = rclpy.create_node('mavic_driver')
        self.__node.create_subscription(Propellars, "Mavic_2_PRO/propellers",
                                        self.__on_propellars,
                                        qos_profile_sensor_data)
        self.__node.create_subscription(Image, "Mavic_2_PRO/grid",
                                        self.__on_grid,
                                        qos_profile_sensor_data)

    def __on_propellars(self, msg):
        self.__propellers[0].setVelocity(msg.fr)
        self.__propellers[1].setVelocity(msg.fl)
        self.__propellers[2].setVelocity(msg.rr)
        self.__propellers[3].setVelocity(msg.rl)

    def __on_grid(self, msg):
        ir = self.__display.imageNew(msg.data.tobytes(), self.__display.BGRA,
                                     msg.width, msg.height)
        self.__display.imagePaste(ir, 0, 0, False)
        self.__display.imageDelete(ir)

    def step(self):
        for s in range(10):
            rclpy.spin_once(self.__node, timeout_sec=0)


# eof
