#pragma once

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "webots_interfaces/msg/propellars.hpp"
#include <deque>

class UAVController : public rclcpp::Node
{
  public:
    UAVController();

  private:
    class WayPoint
    {
      public:
        WayPoint(const double &lat_, const double &lon_, const double &height_,
                 const double &yaw_)
            : lat{lat_}, lon{lon_}, height{height_}, yaw{yaw_}
        {
        }

        double lat;
        double lon;
        double height;
        double yaw;
        double operator-(const WayPoint &other)
        {
            return sqrt((lat - other.lat) * (lat - other.lat) +
                        (lon - other.lon) * (lon - other.lon) +
                        (height - other.height) * (height - other.height));
        }
    };

    geometry_msgs::msg::PointStamped m_gps;
    geometry_msgs::msg::PointStamped m_imu;
    geometry_msgs::msg::Vector3 m_gpsSpeed;

    std::deque<WayPoint> m_waypoints;
    rclcpp::Time m_last_waypoint_reached;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr m_imuSubscription;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr
        m_gpsSubscription;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr
        m_gpsSpeedSubscription;
    rclcpp::Publisher<webots_interfaces::msg::Propellars>::SharedPtr
        m_propellersPublisher;
    rclcpp::TimerBase::SharedPtr m_timer;

    void onTimer();
    void onGPSSpeedVector(const geometry_msgs::msg::Vector3::SharedPtr msg);
    void onGPS(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void onIMU(const sensor_msgs::msg::Imu::SharedPtr msg);

    friend std::ostream &operator<<(std::ostream &stream,
                                    const UAVController::WayPoint &wp);
};
std::ostream &operator<<(std::ostream &stream,
                         const UAVController::WayPoint &wp);
