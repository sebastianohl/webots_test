#pragma once

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <opencv2/opencv.hpp>

class UAVFusion : public rclcpp::Node
{
  public:
    UAVFusion();

  private:
    class HeightGrid
    {
      public:
        template <typename T> struct Point2D
        {
            T x;
            T y;
        };
        HeightGrid(const Point2D<uint32_t> &size,
                   const Point2D<double> &resolution,
                   const Point2D<double> &anchor);

        HeightGrid(const HeightGrid &) = delete;
        void operator=(const HeightGrid &) = delete;

        Point2D<int32_t>
        world2DiscreteWorld(const Point2D<double> &world) const;
        Point2D<int32_t> world2Grid(const Point2D<double> &world) const;
        Point2D<int32_t> world2Grid(const Point2D<double> &world,
                                    const Point2D<int32_t> &anchor) const;
        Point2D<double> grid2World(const Point2D<int32_t> &grid) const;
        Point2D<double> grid2World(const Point2D<int32_t> &grid,
                                   const Point2D<int32_t> &anchor) const;

        void move(const Point2D<double> &new_anchor);

        const Point2D<uint32_t> &getSize() const { return m_size; }
        const Point2D<int32_t> &getAnchor() const { return m_anchor; }
        const Point2D<double> &getResolution() const { return m_resolution; }

        std::shared_ptr<sensor_msgs::msg::Image> show() const;

        bool isInGrid(const Point2D<int32_t> &grid) const;
        void updateCell(const Point2D<int32_t> &grid,
                        const double &measurement);

      private:
        Point2D<uint32_t> m_size;
        Point2D<int32_t> m_anchor;
        Point2D<double> m_resolution;
        cv::Mat m_grid;
    };

    geometry_msgs::msg::PointStamped m_gps;
    geometry_msgs::msg::PointStamped m_imu;
    geometry_msgs::msg::Vector3 m_gpsSpeed;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr m_imuSubscription;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
        m_lidarSubscription;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr
        m_gpsSubscription;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr
        m_gpsSpeedSubscription;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_gridPublisher;
    rclcpp::TimerBase::SharedPtr m_timer;

    HeightGrid m_grid;

    void onTimer();
    void onGPSSpeedVector(const geometry_msgs::msg::Vector3::SharedPtr msg);
    void onGPS(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void onIMU(const sensor_msgs::msg::Imu::SharedPtr msg);
    void onLIDAR(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
};
