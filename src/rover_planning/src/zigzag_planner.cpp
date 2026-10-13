#include <cmath>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

class ZigzagPlanner : public rclcpp::Node
{
public:
  ZigzagPlanner() : Node("zigzag_planner")
  {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    declare_parameter("base_frame", "base_footprint_ekf");
    declare_parameter("margin", 2.0);          // meters kept free from every map border
    declare_parameter("lane_spacing", 1.0);    // distance between lanes (<= detector width)
    declare_parameter("point_step", 0.1);      // distance between path points
    declare_parameter("lanes_along_x", true);  // true: horizontal lanes, false: vertical
    declare_parameter("area_min_x", nan);      // optional manual override (map frame)
    declare_parameter("area_max_x", nan);
    declare_parameter("area_min_y", nan);
    declare_parameter("area_max_y", nan);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);


    rclcpp::QoS map_qos(10);
    map_qos.transient_local();
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", map_qos,
      [this](const nav_msgs::msg::OccupancyGrid::SharedPtr m) {map_ = m;});

    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", 10,
      std::bind(&ZigzagPlanner::trigger, this, std::placeholders::_1));

    rclcpp::QoS path_qos(1);
    path_qos.transient_local();  // late RViz subscribers still get the path
    path_pub_ = create_publisher<nav_msgs::msg::Path>("/zigzag/path", path_qos);

    RCLCPP_INFO(get_logger(), "Zigzag planner ready. Send any /goal_pose to plan.");
  }

private:
  using Pt = std::pair<double, double>;

  void trigger(const geometry_msgs::msg::PoseStamped::SharedPtr)
  {
    if (!map_) {
      RCLCPP_ERROR(get_logger(), "No map received!");
      return;
    }

    const double margin = get_parameter("margin").as_double();
    const double spacing = get_parameter("lane_spacing").as_double();
    const double step = get_parameter("point_step").as_double();
    const bool along_x = get_parameter("lanes_along_x").as_bool();
    if (spacing <= 0.0 || step <= 0.0) {

      RCLCPP_ERROR(get_logger(), "lane_spacing and point_step must be > 0");
      return;
    }

    // ---- coverage area (map frame) ----
    const double res = map_->info.resolution;
    double x_min = map_->info.origin.position.x + margin;
    double x_max = map_->info.origin.position.x + map_->info.width * res - margin;
    double y_min = map_->info.origin.position.y + margin;
    double y_max = map_->info.origin.position.y + map_->info.height * res - margin;
    override_if_set("area_min_x", x_min);
    override_if_set("area_max_x", x_max);
    override_if_set("area_min_y", y_min);
    override_if_set("area_max_y", y_max);
    if (x_max <= x_min || y_max <= y_min) {
      RCLCPP_ERROR(get_logger(), "Coverage area is empty. Check margin / area_* parameters.");
      return;
    }

    // ---- robot position (start) ----
    bool have_robot = false;
    double rx = 0.0, ry = 0.0;
    try {
      auto tf = tf_buffer_->lookupTransform(
        map_->header.frame_id, get_parameter("base_frame").as_string(),
        tf2::TimePointZero);
      rx = tf.transform.translation.x;
      ry = tf.transform.translation.y;
      have_robot = true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(get_logger(), "No TF for robot, starting from area corner: %s", ex.what());
      rx = x_min;

      ry = y_min;
    }

    // Work in (a, b): a = along the lane, b = across lanes.
    const double a0 = along_x ? x_min : y_min;
    const double a1 = along_x ? x_max : y_max;
    const double b0 = along_x ? y_min : x_min;
    const double b1 = along_x ? y_max : x_max;
    const double ra = along_x ? rx : ry;
    const double rb = along_x ? ry : rx;

    // Lane positions across: b0, b0+spacing, ..., and always finish on b1.
    std::vector<double> lanes;
    for (double b = b0; b < b1 - 1e-6; b += spacing) {
      lanes.push_back(b);
    }
    if (lanes.empty() || (b1 - lanes.back()) > spacing * 0.5) {
      lanes.push_back(b1);
    } else {
      lanes.back() = b1;
    }
    // Start from the side nearest to the robot.
    if (std::abs(rb - b1) < std::abs(rb - b0)) {
      std::reverse(lanes.begin(), lanes.end());
    }
    bool forward = std::abs(ra - a0) <= std::abs(ra - a1);  // first lane direction

    // ---- waypoints ----
    auto toXY = [&](double a, double b) -> Pt {
      return along_x ? Pt{a, b} : Pt{b, a};
    };
    std::vector<Pt> wps;

    if (have_robot) {
      wps.push_back({rx, ry});
    }
    for (double b : lanes) {
      const double a_start = forward ? a0 : a1;
      const double a_end = forward ? a1 : a0;
      wps.push_back(toXY(a_start, b));
      wps.push_back(toXY(a_end, b));
      forward = !forward;
    }

    // ---- densify and build the path ----
    std::vector<Pt> pts;
    pts.push_back(wps.front());
    for (size_t i = 1; i < wps.size(); ++i) {
      const double dx = wps[i].first - wps[i - 1].first;
      const double dy = wps[i].second - wps[i - 1].second;
      const double dist = std::hypot(dx, dy);
      const int n = std::max(1, static_cast<int>(std::ceil(dist / step)));
      for (int k = 1; k <= n; ++k) {
        pts.push_back({wps[i - 1].first + dx * k / n, wps[i - 1].second + dy * k / n});
      }
    }

    nav_msgs::msg::Path path;
    path.header.frame_id = map_->header.frame_id;
    path.header.stamp = now();
    int occupied = 0;
    double length = 0.0;
    for (size_t i = 0; i < pts.size(); ++i) {
      const size_t j = (i + 1 < pts.size()) ? i + 1 : i;
      double yaw = 0.0;

      if (j != i) {
        yaw = std::atan2(pts[j].second - pts[i].second, pts[j].first - pts[i].first);
        length += std::hypot(pts[j].first - pts[i].first, pts[j].second - pts[i].second);
      } else if (!path.poses.empty()) {
        yaw = 2.0 * std::atan2(path.poses.back().pose.orientation.z,
                               path.poses.back().pose.orientation.w);
      }
      geometry_msgs::msg::PoseStamped ps;
      ps.header = path.header;
      ps.pose.position.x = pts[i].first;
      ps.pose.position.y = pts[i].second;
      ps.pose.orientation.z = std::sin(yaw / 2.0);
      ps.pose.orientation.w = std::cos(yaw / 2.0);
      path.poses.push_back(ps);
      if (isOccupied(pts[i].first, pts[i].second)) {
        ++occupied;
      }
    }

    path_pub_->publish(path);
    RCLCPP_INFO(
      get_logger(), "Zigzag path: %zu lanes, %zu points, %.1f m, area x[%.2f, %.2f] y[%.2f, %.2f]",
      lanes.size(), path.poses.size(), length, x_min, x_max, y_min, y_max);
    if (occupied > 0) {
      RCLCPP_WARN(get_logger(), "%d path points are on occupied map cells.", occupied);
    }
  }

  void override_if_set(const std::string & name, double & value)
  {
    const double v = get_parameter(name).as_double();
    if (!std::isnan(v)) {

      value = v;
    }
  }

  bool isOccupied(double x, double y) const
  {
    const double res = map_->info.resolution;
    const int ix = static_cast<int>(std::floor((x - map_->info.origin.position.x) / res));
    const int iy = static_cast<int>(std::floor((y - map_->info.origin.position.y) / res));
    if (ix < 0 || iy < 0 ||
      ix >= static_cast<int>(map_->info.width) || iy >= static_cast<int>(map_->info.height))
    {
      return false;
    }
    return map_->data.at(iy * map_->info.width + ix) > 50;
  }

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  nav_msgs::msg::OccupancyGrid::SharedPtr map_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ZigzagPlanner>());
  rclcpp::shutdown();
  return 0;
}