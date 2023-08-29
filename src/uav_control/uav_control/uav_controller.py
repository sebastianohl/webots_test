import numpy as np
import math
from pprint import pprint, pformat

import rclpy
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PointStamped, Vector3
from webots_interfaces.msg import Propellars

DEG_TO_RAD = 0.0174533


def getAnglesFromQuadrion(o):
    # https://github.com/cyberbotics/webots/blob/1b8fdd6243a34c8a3d08a7c1bd84528e9e321609/src/controller/c/inertial_unit.c
    # ENU: extrensic rotation matrix e = Z(yaw) Y(pitch) X(roll)
    t0 = 2.0 * (o.w * o.x + o.y * o.z)
    t1 = 1.0 - 2.0 * (o.x * o.x + o.y * o.y)
    roll = math.atan2(t0, t1)
    t2 = 2.0 * (o.w * o.y - o.z * o.x)
    t2 = 1.0 if t2 > 1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)
    t3 = 2.0 * (o.w * o.z + o.x * o.y)
    t4 = 1.0 - 2.0 * (o.y * o.y + o.z * o.z)
    yaw = math.atan2(t3, t4)
    return roll, yaw, pitch


def clamp(value, value_min, value_max):
    return min(max(value, value_min), value_max)


def distance(target, current):
    return math.sqrt(
        pow(target.lat - current.lat, 2) + pow(target.lon - current.lon, 2) +
        pow(target.height - current.height, 2))


def rotateZ(x, y, yaw):
    return (x * math.cos(yaw) - y * math.sin(yaw),
            x * math.sin(yaw) + y * math.cos(yaw))


def radDiff(a, b):
    if (math.fabs(a) < math.pi / 2 and math.fabs(b) < math.pi / 2):
        return math.fabs(a - b)
    a = a + 2 * math.pi if a < 0 else a
    b = b + 2 * math.pi if b < 0 else b
    e = math.fabs(a - b)
    return e if (e < math.pi) else 2 * math.pi - e


def normRadianPIPI(r):
    while r > math.pi:
        r = r - 2 * math.pi
    while r < -math.pi:
        r = r + 2 * math.pi
    return r


class WayPoint:

    def __init__(self, lat_, lon_, height_, yaw_):
        self.lat = lat_
        self.lon = lon_
        self.height = height_
        self.yaw = normRadianPIPI(yaw_)


class UAVController(Node):

    def __init__(self):
        super().__init__('uav_controller')

        self.__gps = {
            "lat": None,
            "lon": None,
            "height": None,
            "set": False,
        }
        self.__gps_speed = {
            "x": None,
            "y": None,
            "z": None,
            "set": False,
        }
        self.__imu = {"roll": None, "yaw": None, "pitch": None, "set": False}

        self.__waypoints = []
        self.__waypoints.append(WayPoint(0, 0, 1.5, 0))
        self.__waypoints.append(WayPoint(0, 3, 1.5, math.pi / 2))

        for i in range(9):
            x, y = rotateZ(0, 3, i * 2 * math.pi / 8)
            self.__waypoints.append(
                WayPoint(x, y, 1.5, i * 2 * math.pi / 8 + math.pi))

        self.__waypoints.append(WayPoint(0, 0, 1.5, 0))
        self.__waypoints.append(WayPoint(0, 0, 0.4, 0))

        for w in self.__waypoints:
            self.get_logger().info("waypoint %f, %f, %f, %f" %
                                   (w.lat, w.lon, w.height, w.yaw))

        self.__last_waypoint_reached = None

        # ROS interface
        self.create_subscription(Vector3, 'Mavic_2_PRO/gps/speed_vector',
                                 self.__on_gps_speed_vector,
                                 qos_profile_sensor_data)
        self.create_subscription(PointStamped, 'Mavic_2_PRO/gps',
                                 self.__on_gps, qos_profile_sensor_data)
        self.create_subscription(Imu, 'imu', self.__on_imu,
                                 qos_profile_sensor_data)
        self._propellers = self.create_publisher(Propellars,
                                                 "Mavic_2_PRO/propellers", 1)
        self.timer = self.create_timer(0.008,
                                       self.__timer_callback,
                                       clock=self.get_clock())

    def __on_gps_speed_vector(self, message):
        # self.get_logger().info("got gps speed %f,%f,%f" %
        #                       (message.x, message.y, message.z))
        self.__gps_speed["x"] = message.x
        self.__gps_speed["y"] = message.y
        self.__gps_speed["z"] = message.z
        self.__gps_speed["set"] = True

    def __on_gps(self, message):
        # self.get_logger().info("got gps %f,%f,%f" %
        #                       (message.point.x, message.point.y, message.point.z))
        self.__gps["lat"] = message.point.x
        self.__gps["lon"] = message.point.y
        self.__gps["height"] = message.point.z
        self.__gps["set"] = True

    def __on_imu(self, message):
        roll, yaw, pitch = getAnglesFromQuadrion(message.orientation)
        # self.get_logger().info("got imu r %f y %f p %f" % (roll, yaw, pitch))

        self.__imu["roll"] = roll
        self.__imu["yaw"] = yaw
        self.__imu["pitch"] = pitch
        self.__imu["set"] = True

    def __timer_callback(self):

        if not self.__gps["set"] or \
           not self.__imu["set"] or \
           not self.__gps_speed["set"]:
            self.get_logger().info("no data yet, stay put")
            return

        # Read sensors
        roll, pitch, yaw = (self.__imu["roll"], self.__imu["pitch"],
                            self.__imu["yaw"])
        yaw = normRadianPIPI(yaw)

        lat = self.__gps["lat"]
        lon = self.__gps["lon"]
        vertical = self.__gps["height"]

        if len(self.__waypoints) == 0:
            self._propellers.publish(Propellars())
            return

        # self.get_logger().info("roll %f pitch %f yaw %f height %f lat %f lon %f tlat %f tlon %f theight %f, tyaw %f, yawdiff %f" %
        #                       (roll, pitch, yaw, vertical, lat, lon, self.__waypoints[0].lat, self.__waypoints[0].lon, self.__waypoints[0].height, self.__waypoints[0].yaw, radDiff(yaw, self.__waypoints[0].yaw)))

        # way point handling
        if distance(self.__waypoints[0], WayPoint(
                lat, lon, vertical, yaw)) < 0.3 and radDiff(
                    yaw, self.__waypoints[0].yaw) < 0.1:
            self.__last_waypoint_reached = self.get_clock().now(
            ) if self.__last_waypoint_reached is None else self.__last_waypoint_reached
            if self.get_clock().now(
            ) - self.__last_waypoint_reached > Duration(seconds=3):
                self.__waypoints.pop(0)
                self.__last_waypoint_reached = None
            if len(self.__waypoints) == 0:
                self.get_logger().info("reached final waypoint")
                return

        # target
        target_height = self.__waypoints[0].height
        target_lat = self.__waypoints[0].lat
        target_lon = self.__waypoints[0].lon
        target_yaw = self.__waypoints[0].yaw

        # height control
        base = 68.5 + clamp(target_height - vertical, -.3, .5) * 5

        # direction control
        target_pitch = clamp(target_lat - lat, -0.2, 0.2) * 0.3
        target_roll = -clamp(target_lon - lon, -0.2, 0.2) * 0.3
        target_pitch, target_roll = rotateZ(target_pitch, target_roll, yaw)

        # turn into the shortes direction
        turn_direction = 1
        if normRadianPIPI(target_yaw - yaw) > 0:
            turn_direction = -1

        # roll/pitch/yaw control
        m1 = base - 4 * (target_pitch-pitch) - 4 * (target_roll-roll) - \
            radDiff(target_yaw, yaw) * .2 * turn_direction
        m2 = base - 4 * (target_pitch-pitch) + 4 * (target_roll-roll) + \
            radDiff(target_yaw, yaw) * .2 * turn_direction
        m3 = base + 4 * (target_pitch-pitch) - 4 * (target_roll-roll) + \
            radDiff(target_yaw, yaw) * .2 * turn_direction
        m4 = base + 4 * (target_pitch-pitch) + 4 * (target_roll-roll) - \
            radDiff(target_yaw, yaw) * .2 * turn_direction

        p = Propellars()
        p.fr = -m1
        p.fl = m2
        p.rr = m3
        p.rl = -m4
        self._propellers.publish(p)


def main(args=None):
    rclpy.init(args=args)
    agent = UAVController()
    rclpy.spin(agent)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
