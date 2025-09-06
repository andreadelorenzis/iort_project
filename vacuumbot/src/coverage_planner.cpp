#include "pluginlib/class_list_macros.hpp"
#include "coverage_planner.hpp"
#include "fields2cover.h"
#include "std_msgs/msg/string.hpp"
#include <iostream>

namespace coverage_planner
{

void CoveragePlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  const std::shared_ptr<tf2_ros::Buffer> tf,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  // Store node handle (lock weak pointer)
  node_ = parent;
  costmap_ros_ = costmap_ros;

  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node in CoveragePlanner::configure"};
  }

  RCLCPP_INFO(node->get_logger(), "CoveragePlanner configured");
}

void CoveragePlanner::cleanup()
{
  auto node = node_.lock();
  if (node) {
    RCLCPP_INFO(node->get_logger(), "Cleaning up CoveragePlanner");
  }
  costmap_ros_.reset();
}

void CoveragePlanner::activate()
{
  auto node = node_.lock();
  if (node) {
    RCLCPP_INFO(node->get_logger(), "Activating CoveragePlanner");

    // Publisher for planner_selector
    rclcpp::QoS qos(rclcpp::KeepLast(1));
    qos.transient_local().reliable();
    planner_selector_pub_ = node->create_publisher<std_msgs::msg::String>("planner_selector", qos);

    // Publish the planner ID so the BT picks it up
    auto msg = std_msgs::msg::String();
    msg.data = "CoveragePlanner"; 
    planner_selector_pub_->publish(msg);
    RCLCPP_INFO(node->get_logger(), "Published CoveragePlanner on /planner_selector");
  }
}

void CoveragePlanner::deactivate()
{
  auto node = node_.lock();
  if (node) {
    RCLCPP_INFO(node->get_logger(), "Deactivating CoveragePlanner");
  }
}

nav_msgs::msg::Path CoveragePlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
  {
    auto node = node_.lock();
    RCLCPP_INFO(node->get_logger(), "Creating plan in CoveragePlanner");
    nav_msgs::msg::Path path;
    // path.header.frame_id = costmap_ros_->getGlobalFrameID();
    // auto node = node_.lock();
    // path.header.stamp = node->now();

    // try {
    //   // Points
    //   F2CPoint p1 (1.2, 3.4);
    //   F2CPoint p2 (9.8, 7.6, 5.4);
    //   F2CPoint p3 (OGRPoint(11, 22));
    //   F2CPoint p4;
    //   p4.setX(3.0);
    //   p4.setZ(-1.0);
    //   F2CPoint p5;
    //   p5.importFromWkt("POINT (0 4 4)");
    //   F2CMultiPoint points {F2CPoint(1, 2), F2CPoint(3, 4)};
    //   points.addPoint(5, 6);
    //   points.addPoint(p5);
  
    //   // Lines
    //   F2CLineString line1;
    //   line1.addPoint(3, 0);
    //   line1.addPoint(p5);  // Point(0, 4)
    //   F2CLineString line2({F2CPoint(1, 0), F2CPoint(1, 1), F2CPoint(0, 1)});
    //   F2CLinearRing ring{F2CPoint(1,1), F2CPoint(1,2), F2CPoint(2,2), F2CPoint(1,1)};
    //   F2CMultiLineString lines;
    //   lines.addGeometry(line1);
    //   lines.addGeometry(line2);
  
    //   // Cells
    //   F2CLinearRing outter_ring{
    //     F2CPoint(0, 0), F2CPoint(2, 0),F2CPoint(2, 2), F2CPoint(0, 2), F2CPoint(0, 0)};
    //   F2CLinearRing inner_ring{
    //     F2CPoint(0.5, 0.5), F2CPoint(1.5, 0.5), F2CPoint(1.5, 1.5),
    //     F2CPoint(0.5, 1.5), F2CPoint(0.5, 0.5)};
    //   F2CCell cell;
    //   cell.addRing(outter_ring);
    //   cell.addRing(inner_ring);
    //   F2CCells cells;
    //   cells.addGeometry(cell);
  
    //   F2CRobot robot(3.0, 3.0);
  
    //   RCLCPP_INFO(node->get_logger(), "Created lines using f2c in CoveragePlanner");
    // }  catch (const std::exception & e) {
    //     RCLCPP_ERROR(node->get_logger(), "Exception in F2C objects: %s", e.what());
    // }
    // f2c::Visualizer::figure();
    // f2c::Visualizer::plot(lines);
    // // f2c::Visualizer::show();
    // f2c::Visualizer::save("Tutorial_image.png");

    RCLCPP_INFO(node->get_logger(), "END createPlan in CoveragePlanner");

    return path;
  }

}  // namespace coverage_planner

PLUGINLIB_EXPORT_CLASS(coverage_planner::CoveragePlanner, nav2_core::GlobalPlanner)
