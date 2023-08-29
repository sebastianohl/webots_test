#include "uav_controller.hpp"

#include <chrono>
#include <sstream>

using std::placeholders::_1;
using namespace std::chrono_literals;

#include <sstream>

template <typename T> T clamp(const T &value, const T &min, const T &max)
{
    return (value < min) ? min : (value > max) ? max : value;
}

template <typename T> void rotateZ(T &x, T &y, const T &yaw)
{
    T x_{x * cos(yaw) - y * sin(yaw)};
    y = x * sin(yaw) + y * cos(yaw);
    x = x_;
}

template <typename T> T radDiff(T a, T b)
{
    if (fabs(a) < M_PI_2 && fabs(b) < M_PI_2)
        return fabs(a - b);
    a = (a < 0) ? a + 2 * M_PI : a;
    b = (b < 0) ? b + 2 * M_PI : b;
    auto e = fabs(a - b);
    return (e < M_PI) ? e : 2 * M_PI - e;
}

template <typename T> T normRadianPIPI(T r)
{
    while (r > M_PI)
        r -= 2 * M_PI;
    while (r < -M_PI)
        r += 2 * M_PI;
    return r;
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

UAVController::UAVController()
    : Node("uav_controller_cpp"),
      m_last_waypoint_reached(
          rclcpp::Time(0, 0, get_clock().get()->get_clock_type()))
{
    m_waypoints.push_back(WayPoint{0, 0, 1.5, 0});
    m_waypoints.push_back(WayPoint{0, 3, 1.5, M_PI_2});
    for (auto i = 0; i < 9; ++i)
    {
        double x{0}, y{3};
        rotateZ(x, y, i * M_PI * 2 / 8);
        m_waypoints.push_back(WayPoint{x, y, 1.5, i * M_PI * 2 / 8 + M_PI});
    }

    m_waypoints.push_back(WayPoint{0, 0, 1.5, 0});
    m_waypoints.push_back(WayPoint{0, 0, 0.2, 0});

    for (const auto &w : m_waypoints)
    {
        std::ostringstream os;
        os << w;
        RCLCPP_INFO(get_logger(), "waypoint %s", os.str().c_str());
    }

    // ROS interface
    m_imuSubscription = create_subscription<sensor_msgs::msg::Imu>(
        "imu",
        rclcpp::QoS(
            rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data),
            rmw_qos_profile_sensor_data),
        std::bind(&UAVController::onIMU, this, _1));
    m_gpsSubscription = create_subscription<geometry_msgs::msg::PointStamped>(
        "Mavic_2_PRO/gps",
        rclcpp::QoS(
            rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data),
            rmw_qos_profile_sensor_data),
        std::bind(&UAVController::onGPS, this, _1));
    m_gpsSpeedSubscription = create_subscription<geometry_msgs::msg::Vector3>(
        "Mavic_2_PRO/gps/speed_vector",
        rclcpp::QoS(
            rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data),
            rmw_qos_profile_sensor_data),
        std::bind(&UAVController::onGPSSpeedVector, this, _1));

    m_propellersPublisher =
        create_publisher<webots_interfaces::msg::Propellars>(
            "Mavic_2_PRO/propellers", 1);

    m_timer = rclcpp::create_timer(this, get_clock(), 8ms,
                                   std::bind(&UAVController::onTimer, this));
}

void UAVController::onGPSSpeedVector(
    const geometry_msgs::msg::Vector3::SharedPtr msg)
{
    m_gpsSpeed = *msg;
}
void UAVController::onGPS(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
    m_gps = *msg;
}
void UAVController::onIMU(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    m_imu = getAnglesFromQuadrion(msg);
}

void UAVController::onTimer()
{
    if (m_imu.header.frame_id == "" || m_gps.header.frame_id == "")
    {
        RCLCPP_INFO(get_logger(), "no data yet, stay put");
        return;
    }

    // read sensors
    auto roll{m_imu.point.x};
    auto pitch{m_imu.point.y};
    auto yaw{normRadianPIPI(m_imu.point.z)};

    auto lat{m_gps.point.x};
    auto lon{m_gps.point.y};
    auto vertical{m_gps.point.z};

    auto velocity{m_gpsSpeed};

    if (m_waypoints.size() == 0)
    {
        m_propellersPublisher->publish(webots_interfaces::msg::Propellars());
        return;
    }

    /*RCLCPP_INFO(get_logger(),
                "roll %f pitch %f yaw %f height %f lat %f lon %f tlat %f tlon "
                "%f theight %f, tyaw %f, yawdiff %f",
                roll, pitch, yaw, vertical, lat, lon, m_waypoints[0].lat,
                m_waypoints[0].lon, m_waypoints[0].height, m_waypoints[0].yaw,
                radDiff(yaw, m_waypoints[0].yaw));*/

    // way point handling
    if (m_waypoints[0] - WayPoint(lat, lon, vertical, yaw) < 0.3 &&
        radDiff(yaw, m_waypoints[0].yaw) < 0.1)
    {
        m_last_waypoint_reached =
            (m_last_waypoint_reached ==
             rclcpp::Time(0, 0, get_clock().get()->get_clock_type()))
                ? get_clock().get()->now()
                : m_last_waypoint_reached;
        if (get_clock().get()->now() - m_last_waypoint_reached >
            rclcpp::Duration(0, 3 * 1e9))
        {
            m_waypoints.pop_front();
            m_last_waypoint_reached =
                rclcpp::Time(0, 0, get_clock().get()->get_clock_type());
        }
        if (m_waypoints.size() == 0)
        {
            RCLCPP_INFO(get_logger(), "reached final waypoint");
            return;
        }
    }

    // target
    auto &target_height = m_waypoints[0].height;
    auto &target_lat = m_waypoints[0].lat;
    auto &target_lon = m_waypoints[0].lon;
    auto &target_yaw = m_waypoints[0].yaw;

    // height control
    auto base = 68.5 + clamp(target_height - vertical, -.3, .5) * 5;

    // direction control
    auto target_pitch = clamp(target_lat - lat, -0.2, 0.2) * 0.3;
    auto target_roll = -clamp(target_lon - lon, -0.2, 0.2) * 0.3;
    rotateZ(target_pitch, target_roll, yaw);

    // turn into the shortes direction
    auto turn_direction = 1;
    if (normRadianPIPI(target_yaw - yaw) > 0)
        turn_direction = -1;

    // roll/pitch/yaw control
    auto m1 = base - 4 * (target_pitch - pitch) - 4 * (target_roll - roll) -
              radDiff(target_yaw, yaw) * .2 * turn_direction;
    auto m2 = base - 4 * (target_pitch - pitch) + 4 * (target_roll - roll) +
              radDiff(target_yaw, yaw) * .2 * turn_direction;
    auto m3 = base + 4 * (target_pitch - pitch) - 4 * (target_roll - roll) +
              radDiff(target_yaw, yaw) * .2 * turn_direction;
    auto m4 = base + 4 * (target_pitch - pitch) + 4 * (target_roll - roll) -
              radDiff(target_yaw, yaw) * .2 * turn_direction;

    webots_interfaces::msg::Propellars p;
    p.fr = -m1;
    p.fl = m2;
    p.rr = m3;
    p.rl = -m4;
    m_propellersPublisher->publish(p);
}

std::ostream &operator<<(std::ostream &stream,
                         const UAVController::WayPoint &wp)
{
    stream << "lat=" << wp.lat << " lon=" << wp.lon << " height=" << wp.height
           << " yaw=" << wp.yaw;
    return stream;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UAVController>());
    rclcpp::shutdown();
    return 0;
}