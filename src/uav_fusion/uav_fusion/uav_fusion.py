import cv2
import numpy as np
import math
from pprint import pprint, pformat

import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Imu, Image
from geometry_msgs.msg import PointStamped, Vector3
from cv_bridge import CvBridge


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


def rotate(a, b, angle):
    return (a * math.cos(angle) - b * math.sin(angle),
            a * math.sin(angle) + b * math.cos(angle))


class HeightGrid:

    def __init__(self, size, resolution, anchor):
        self._size = size
        self._resolution = resolution
        self._anchor = self.world2DiscreteWorld(anchor)

        self.grid = np.zeros(self._size)

    def world2DiscreteWorld(self, world):
        return [
            int(world[0] / self._resolution[0]),
            int(world[1] / self._resolution[1])
        ]

    def world2Grid(self, world, anchor=None):
        anchor = self._anchor if anchor is None else anchor
        w = self.world2DiscreteWorld(world)
        return [
            w[0] - anchor[0],
            w[1] - anchor[1],
        ]

    def grid2World(self, grid, anchor=None):
        anchor = self._anchor if anchor is None else anchor
        return [
            (grid[0] + anchor[0]) * self._resolution[0],
            (grid[1] + anchor[1]) * self._resolution[1],
        ]

    def move(self, new_anchor):
        # define new grid
        new_grid = np.zeros(self._size)
        # create new anchor
        new_anchor = self.world2DiscreteWorld(new_anchor)
        for x in range(self._size[0]):
            for y in range(self._size[1]):
                # get cell coordinate in world coordinates
                w = self.grid2World([x, y], new_anchor)
                # convert to old grid
                g = self.world2Grid(w)
                if self.isInGrid(g):  # if its in the old grid, copy data
                    new_grid[x, y] = self.grid[g[0], g[1]]
        # make moved grid active
        self.grid = new_grid
        self._anchor = new_anchor

    def size(self):
        return self._size

    def anchor(self):
        return self._anchor

    def resolution(self):
        return self._resolution

    def show(self):
        # scale max height = 1.5
        grayscale = np.array(self.grid * (255.0 / 1.5), dtype=np.uint8)
        bgra = cv2.cvtColor(grayscale, cv2.COLOR_GRAY2BGRA)
        bridge = CvBridge()

        return bridge.cv2_to_imgmsg(bgra, encoding="passthrough")

    def isInGrid(self, grid):
        return True if grid[0] >= 0 and grid[0] < self._size[0] and grid[
            1] >= 0 and grid[1] < self._size[1] else False

    def updateCell(self, coordinate, measurement):
        if self.isInGrid(coordinate):
            self.grid[coordinate[0], coordinate[1]] = max(
                self.grid[coordinate[0], coordinate[1]], measurement)


class UAVFusion(Node):

    def __init__(self):
        super().__init__('uav_fusion')

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

        # ROS interface
        self.create_subscription(PointCloud2, 'Mavic_2_PRO/lidar/point_cloud',
                                 self.__on_lidar, qos_profile_sensor_data)
        self.create_subscription(Vector3, 'Mavic_2_PRO/gps/speed_vector',
                                 self.__on_gps_speed_vector,
                                 qos_profile_sensor_data)
        self.create_subscription(PointStamped, 'Mavic_2_PRO/gps',
                                 self.__on_gps, qos_profile_sensor_data)
        self.create_subscription(Imu, 'imu', self.__on_imu,
                                 qos_profile_sensor_data)
        self._grid = self.create_publisher(Image, "Mavic_2_PRO/grid", 1)
        self.timer = self.create_timer(.2,
                                       self.__timer_callback,
                                       clock=self.get_clock())

        self.grid = HeightGrid([50, 50], [0.1, 0.1], [-2.5, -2.5])

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
        #self.get_logger().info("got imu r %f y %f p %f" % (roll, yaw, pitch))

        self.__imu["roll"] = roll
        self.__imu["yaw"] = yaw
        self.__imu["pitch"] = pitch
        self.__imu["set"] = True

    def __on_lidar(self, message):

        if not self.__gps["set"] or \
           not self.__imu["set"] or \
           not self.__gps_speed["set"]:
            return

        # magic conversion to numpy array
        np_dtype_list = []
        np_dtype_list.append(("x", np.dtype('float32')))
        np_dtype_list.append(("y", np.dtype('float32')))
        np_dtype_list.append(("z", np.dtype('float32')))
        np_dtype_list.append(("layer", np.dtype('int32')))
        np_dtype_list.append(("time", np.dtype('float32')))
        data = np.fromstring(message.data.tobytes(), np_dtype_list)

        #self.get_logger().info("got cloud with %d points" % (len(data)))

        # skip 200 point at the sides as they are reflections from the drone itselt
        for p in data[200:880]:
            if not math.isinf(p[0]) and not math.isnan(
                    p[0]):  # got reflection for ray
                # measurement
                z = p[0]
                x = 0
                y = p[1]
                # rotate roll
                z, y = rotate(z, y, self.__imu["roll"])
                # rotate pitch
                z, x = rotate(z, x, self.__imu["pitch"])
                # rotate yaw
                x, y = rotate(x, y, self.__imu["yaw"])

                # absolute pos
                z = self.__gps["height"] - z
                world = [
                    self.__gps["lat"] + x,
                    self.__gps["lon"] + y,
                ]

                # convert to grid
                grid = self.grid.world2Grid(world)

                # self.get_logger().info("llh %f,%f,%f p %f,%f world %f,%f grid %d,%d" %
                #                       (self.__gps["lat"], self.__gps["lon"], self.__gps["height"], p[0],p[1], world[0], world[1], grid[0], grid[1]))
                if self.grid.isInGrid(grid):
                    # self.get_logger().info("update cell %d %d to %f" %
                    #                       (grid[0], grid[1], z))
                    self.grid.updateCell(grid, z)

    def __timer_callback(self):
        # move grid
        g = self.grid.world2Grid([self.__gps["lat"], self.__gps["lon"]])
        # is UAV in inner square
        if (g[0] < self.grid.size()[0]*1/4 or g[0] > self.grid.size()[0]*3/4) or \
           (g[1] < self.grid.size()[1]*1/4 or g[1] > self.grid.size()[1]*3/4):
            new_anchor = [
                self.__gps["lat"] -
                self.grid.size()[0] * self.grid.resolution()[0] * 0.5,
                self.__gps["lon"] -
                self.grid.size()[1] * self.grid.resolution()[1] * 0.5
            ]
            self.get_logger().info("move to %f %f" %
                                   (new_anchor[0], new_anchor[1]))
            self.grid.move(new_anchor)

        self._grid.publish(self.grid.show())


def main(args=None):
    rclpy.init(args=args)
    agent = UAVFusion()
    rclpy.spin(agent)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
