#include "uav_fusion.hpp"

#include "cv_bridge/cv_bridge.h"
#include <chrono>
#include <sstream>
#include <string.h>

using std::placeholders::_1;
using namespace std::chrono_literals;

#include <sstream>

#pragma pack(push, 1)
struct LIDARPoint
{
    float x, y, z;
    uint32_t layer;
    float time;
};
#pragma pack(pop)

template <typename T> void rotate(T &a, T &b, const T &yaw)
{
    T a_{a * cos(yaw) - b * sin(yaw)};
    b = a * sin(yaw) + b * cos(yaw);
    a = a_;
}

geometry_msgs::msg::PointStamped
getAnglesFromQuadrion(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    geometry_msgs::msg::PointStamped result;
    result.header = msg->header;

    const geometry_msgs::msg::Quaternion &q(msg->orientation);

    // ENU: extrensic rotation matrix e = Z(yaw) Y(pitch) X(roll)
    const double t0 = 2.0 * (q.w * q.x + q.y * q.z);
    const double t1 = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
    const double roll = atan2(t0, t1);
    double t2 = 2.0 * (q.w * q.y - q.z * q.x);
    t2 = (t2 > 1.0) ? 1.0 : t2;
    t2 = (t2 < -1.0) ? -1.0 : t2;
    const double pitch = asin(t2);
    const double t3 = 2.0 * (q.w * q.z + q.x * q.y);
    const double t4 = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    const double yaw = atan2(t3, t4);
    result.point.x = roll;
    result.point.y = pitch;
    result.point.z = yaw;

    return result;
}

UAVFusion::HeightGrid::HeightGrid(
    const UAVFusion::HeightGrid::Point2D<uint32_t> &size,
    const UAVFusion::HeightGrid::Point2D<double> &resolution,
    const UAVFusion::HeightGrid::Point2D<double> &anchor)
    : m_size(size), m_resolution(resolution),
      m_grid(size.x, size.y, CV_64FC1, cv::Scalar(0.0))
{
    m_anchor = world2DiscreteWorld(anchor);
}

UAVFusion::HeightGrid::Point2D<int32_t>
UAVFusion::HeightGrid::world2DiscreteWorld(
    const UAVFusion::HeightGrid::Point2D<double> &world) const
{
    return Point2D<int32_t>{int(world.x / m_resolution.x),
                            int(world.y / m_resolution.y)};
}
UAVFusion::HeightGrid::Point2D<int32_t> UAVFusion::HeightGrid::world2Grid(
    const UAVFusion::HeightGrid::Point2D<double> &world) const
{
    return world2Grid(world, m_anchor);
}
UAVFusion::HeightGrid::Point2D<int32_t> UAVFusion::HeightGrid::world2Grid(
    const UAVFusion::HeightGrid::Point2D<double> &world,
    const UAVFusion::HeightGrid::Point2D<int32_t> &anchor) const
{
    auto discrete_world = world2DiscreteWorld(world);
    return Point2D<int32_t>{discrete_world.x - anchor.x,
                            discrete_world.y - anchor.y};
}
UAVFusion::HeightGrid::Point2D<double> UAVFusion::HeightGrid::grid2World(
    const UAVFusion::HeightGrid::Point2D<int32_t> &grid) const
{
    return grid2World(grid, m_anchor);
}
UAVFusion::HeightGrid::Point2D<double> UAVFusion::HeightGrid::grid2World(
    const UAVFusion::HeightGrid::Point2D<int32_t> &grid,
    const UAVFusion::HeightGrid::Point2D<int32_t> &anchor) const
{
    return Point2D<double>{(grid.x + anchor.x) * m_resolution.x,
                           (grid.y + anchor.y) * m_resolution.y};
}

void UAVFusion::HeightGrid::move(
    const UAVFusion::HeightGrid::Point2D<double> &new_anchor)
{
    // define new grid
    auto new_grid = cv::Mat(m_size.x, m_size.y, CV_64FC1, cv::Scalar(0.0));
    // define new anchor
    auto new_anchor_discrete = world2DiscreteWorld(new_anchor);
    for (int32_t x = 0; x < m_size.x; ++x)
    {
        for (int32_t y = 0; y < m_size.y; ++y)
        {
            // get cell coordinate in world coordinates
            auto w = grid2World(Point2D<int32_t>{x, y}, new_anchor_discrete);
            // convert to old grid
            auto g = world2Grid(w);
            if (isInGrid(g)) // if in old grid, copy data
            {
                new_grid.at<double>(x, y) = m_grid.at<double>(g.x, g.y);
            }
        }
    }
    // make new grid active
    m_grid = new_grid;
    m_anchor = new_anchor_discrete;
}

std::shared_ptr<sensor_msgs::msg::Image> UAVFusion::HeightGrid::show() const
{
    // scale max height 1.5m
    cv::Mat scaled = m_grid * (255.0 / 1.5);
    cv::Mat grayscale;
    scaled.convertTo(grayscale, CV_8U);
    cv::Mat bgra;
    cv::cvtColor(grayscale, bgra, cv::COLOR_GRAY2BGRA);

    std_msgs::msg::Header header;
    cv_bridge::CvImage cvimage(header, "bgra", bgra);
    return cvimage.toImageMsg();
}

bool UAVFusion::HeightGrid::isInGrid(
    const UAVFusion::HeightGrid::Point2D<int32_t> &grid) const
{
    return (grid.x >= 0 && grid.x < m_size.x && grid.y >= 0 &&
            grid.y < m_size.y);
}
void UAVFusion::HeightGrid::updateCell(
    const UAVFusion::HeightGrid::Point2D<int32_t> &grid,
    const double &measurement)
{
    if (isInGrid(grid))
    {
        m_grid.at<double>(grid.x, grid.y) =
            fmax(m_grid.at<double>(grid.x, grid.y), measurement);
    }
}

UAVFusion::UAVFusion()
    : Node("uav_fusion_cpp"),
      m_grid(UAVFusion::HeightGrid::Point2D<uint32_t>{50, 50},
             UAVFusion::HeightGrid::Point2D<double>{0.1, 0.1},
             UAVFusion::HeightGrid::Point2D<double>{-2.5, -2.5})
{
    // ROS interface
    m_lidarSubscription = create_subscription<sensor_msgs::msg::PointCloud2>(
        "Mavic_2_PRO/lidar/point_cloud",
        rclcpp::QoS(
            rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data),
            rmw_qos_profile_sensor_data),
        std::bind(&UAVFusion::onLIDAR, this, _1));
    m_imuSubscription = create_subscription<sensor_msgs::msg::Imu>(
        "imu",
        rclcpp::QoS(
            rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data),
            rmw_qos_profile_sensor_data),
        std::bind(&UAVFusion::onIMU, this, _1));
    m_gpsSubscription = create_subscription<geometry_msgs::msg::PointStamped>(
        "Mavic_2_PRO/gps",
        rclcpp::QoS(
            rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data),
            rmw_qos_profile_sensor_data),
        std::bind(&UAVFusion::onGPS, this, _1));
    m_gpsSpeedSubscription = create_subscription<geometry_msgs::msg::Vector3>(
        "Mavic_2_PRO/gps/speed_vector",
        rclcpp::QoS(
            rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data),
            rmw_qos_profile_sensor_data),
        std::bind(&UAVFusion::onGPSSpeedVector, this, _1));

    m_gridPublisher =
        create_publisher<sensor_msgs::msg::Image>("Mavic_2_PRO/grid", 1);

    m_timer = rclcpp::create_timer(this, get_clock(), 200ms,
                                   std::bind(&UAVFusion::onTimer, this));
}

void UAVFusion::onLIDAR(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    // RCLCPP_INFO(get_logger(), "got point cloud");
    // skip 200 point at the sides as they are reflections from the drone itselt
    for (uint32_t i = 200; i < (msg->width * msg->height) - 200; ++i)
    {
        // accessing point data really sucks...
        const LIDARPoint &p(
            *(reinterpret_cast<LIDARPoint *>(msg->data.data()) + i));
        if (!isnan(p.x) && !isinf(p.x)) // do we got a reflection of this ray?
        {
            // measurement
            double z = p.x;
            double x = 0.0;
            double y = p.y;

            // rotate roll
            rotate(z, y, m_imu.point.y);
            // rotate pitch
            rotate(z, x, m_imu.point.x);
            // rotate yaw
            rotate(x, y, m_imu.point.z);

            // absolute position
            z = m_gps.point.z - z;
            auto world = HeightGrid::Point2D<double>{m_gps.point.x + x,
                                                     m_gps.point.y + y};
            auto grid = m_grid.world2Grid(world);
            // RCLCPP_INFO(get_logger(),
            //            "llh %f,%f,%f p %f,%f world %f,%f grid %d,%d",
            //            m_gps.point.x, m_gps.point.y, m_gps.point.z, p.x, p.y,
            //            world.x, world.y, grid.x, grid.y);
            if (m_grid.isInGrid(grid))
            {
                // RCLCPP_INFO(get_logger(), "update cell %d %d to %f", grid.x,
                //            grid.y, z);
                m_grid.updateCell(grid, z);
            }
        }
    }
}

void UAVFusion::onGPSSpeedVector(
    const geometry_msgs::msg::Vector3::SharedPtr msg)
{
    m_gpsSpeed = *msg;
}
void UAVFusion::onGPS(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
    m_gps = *msg;
}
void UAVFusion::onIMU(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    m_imu = getAnglesFromQuadrion(msg);
}

void UAVFusion::onTimer()
{
    // move grid
    auto g = m_grid.world2Grid(
        HeightGrid::Point2D<double>{m_gps.point.x, m_gps.point.y});
    // is UAV in inner square
    if (g.x < m_grid.getSize().x * 0.25 || g.x > m_grid.getSize().x * 0.75 ||
        g.y < m_grid.getSize().y * 0.25 || g.y > m_grid.getSize().y * 0.75)
    {
        auto new_anchor = HeightGrid::Point2D<double>{
            m_gps.point.x - m_grid.getSize().x * m_grid.getResolution().x * 0.5,
            m_gps.point.y -
                m_grid.getSize().y * m_grid.getResolution().y * 0.5};
        RCLCPP_INFO(get_logger(), "move to %f %f", new_anchor.x, new_anchor.y);
        m_grid.move(new_anchor);
    }
    m_gridPublisher->publish(*m_grid.show());
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UAVFusion>());
    rclcpp::shutdown();
    return 0;
}