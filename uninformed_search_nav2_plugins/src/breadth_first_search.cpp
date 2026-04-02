#include "uninformed_search_nav2_plugins/breadth_first_search.hpp"
#include <pluginlib/class_list_macros.hpp>


using namespace uninformed_search_nav2_plugins;

void BreadthFirstSearch::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr &,
    std::string,
    std::shared_ptr<tf2_ros::Buffer>,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS>costmap_ros
){
    costmap_ = costmap_ros->getCostmap();
    global_frame_ = costmap_ros->getGlobalFrameID();
}

void BreadthFirstSearch::activate(){
}

void BreadthFirstSearch::deactivate(){
}

void BreadthFirstSearch::cleanup(){
}

bool BreadthFirstSearch::isvalid(unsigned int x, unsigned int y){
    if (x >=costmap_->getSizeInCellsX() || y >= costmap_->getSizeInCellsY()){
        return false;
    }
    unsigned char cost = costmap_->getCost(x, y);
    return cost < nav2_costmap_2d::LETHAL_OBSTACLE;
}

nav_msgs::msg::Path BreadthFirstSearch::createPlan(
    const geometry_msgs::msg::PoseStamped &start,
    const geometry_msgs::msg::PoseStamped &goal,
    std::function<bool()> cancel_checker
){
    nav_msgs::msg::Path path;
    path.header.frame_id = global_frame_;

    unsigned int sx, sy, gx, gy; //start and goal x,y coordinates

    if(!costmap_->worldToMap(start.pose.position.x, start.pose.position.y, sx, sy) || 
    !costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, gx, gy)){
        RCLCPP_ERROR(rclcpp::get_logger("BFS"), "start or goal outside map");
        return path;
    }

    std::queue<std::pair<unsigned int, unsigned int>> queue; //FIFO First in first
    std::set<std::pair<unsigned int, unsigned int>> visited; //To avoid visting same node
    std::map<std::pair<unsigned int, unsigned int>, std::pair<unsigned int, unsigned int>> parent; //To keep track of parent node 

    queue.push({sx, sy});
    visited.insert({sx, sy});
    directions = {{1,0}, {-1,0}, {0,1}, {0,-1}};
    bool found = false;

    while(!queue.empty()){ //While queue have nodes
        if (cancel_checker()) {
            RCLCPP_WARN(rclcpp::get_logger("BFS"), "Planning cancelled");
            return path;
        }

        auto current = queue.front(); //Current stores the nodes to be visited
        queue.pop(); //remove that element from the container
        unsigned int x = current.first;
        unsigned int y = current.second;

        if (x == gx && y == gy){ //current::x is equal to goal::x and current::y is equal to goal::y 
            found = true;
            break;
        }

        for(auto &d : directions){ //exploring neighbours
            int nx = x + d.first;
            int ny = y + d.second;

            if (nx < 0 || ny <0){
                continue;;
            }

            std::pair<unsigned int, unsigned int> next = {nx, ny};
            if (visited.count(next)==0 && isvalid(nx, ny)){
                queue.push(next);
                visited.insert(next);
                parent[next] = current;
            }
        }

    }

    if (!found){
        RCLCPP_ERROR(rclcpp::get_logger("BFS"), "No path found");
        return path;
    }

    std::vector<std::pair<unsigned int, unsigned int>> backtrack_path;
    std::pair<unsigned int, unsigned int> current_path = {gx, gy};

    while (current_path != std::make_pair(sx, sy))
    {
        backtrack_path.push_back(current_path);
        current_path = parent[current_path];
    }
    backtrack_path.push_back({sx, sy});
    std::reverse(backtrack_path.begin(), backtrack_path.end());

    for(auto &nodes : backtrack_path){
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = global_frame_;

        double wx, wy;
        costmap_->mapToWorld(nodes.first, nodes.second, wx, wy);
        pose.pose.position.x = wx;
        pose.pose.position.y = wy;
        path.poses.push_back(pose);
    }
    return path;
}

PLUGINLIB_EXPORT_CLASS(
    uninformed_search_nav2_plugins::BreadthFirstSearch, nav2_core::GlobalPlanner
)