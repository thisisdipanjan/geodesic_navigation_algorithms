#pragma once

#include <set>
#include <map>
#include <functional>
#include <tf2_ros/buffer.hpp>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav2_core/global_planner.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>

namespace uninformed_search_nav2_plugins{
    class BreadthFirstSearch : public nav2_core::GlobalPlanner{
        public:
            BreadthFirstSearch();
            void configure(
                const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
                std::string name,
                std::shared_ptr<tf2_ros::Buffer> tf,
                std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros
            ) override;
            void cleanup() override;
            void activate() override;
            void deactivate() override;

            nav_msgs::msg::Path createPlan(
                const geometry_msgs::msg::PoseStamped &start,
                const geometry_msgs::msg::PoseStamped &goal,
                std::function<bool()> cancel_checker
            ) override;

        private:
            std::vector<std::pair<int, int>> directions;
            nav2_costmap_2d::Costmap2D *costmap_;
            std::string global_frame_;
            bool isvalid(unsigned int x, unsigned int y);
    };
}