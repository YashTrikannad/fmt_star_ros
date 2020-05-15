#ifndef SRC_PLANNER_IMPL_H
#define SRC_PLANNER_IMPL_H

//#include "planner.h"

#include <unordered_set>
#include <queue>
#include <utility>
#include <algorithm>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace fmt_star
{

/// Creates a Planner instance with the input occupancy grid as the map
/// @param occupancy_grid - current occupancy grid (map)
/// @param no_of_nodes - No. of nodes to sample over a map
/// @param ball_radius - radius to be considered for a sample to be near neighbor
/// @param obstacle_inflation_radius - safety boundary around obstacles
/// @param sampling_rectangle - Rectangle defining the boundary for sampling nodes
Planner::Planner(nav_msgs::OccupancyGridConstPtr occupancy_grid,
                 size_t no_of_nodes,
                 double ball_radius,
                 size_t n_collision_checks,
                 int obstacle_inflation_radius,
                 double goal_tolerance_,
                 double hg_ratio,
                 bool online,
                 const std::array<double, 4> &sampling_rectangle,
                 bool minimal_sampling,
                 double sampling_tolerance,
                 bool visualization,
                 ros::Publisher* samples_visualizer,
                 ros::Publisher* tree_visualizer,
                 ros::Publisher* path_visualizer) :
        occupancy_grid_(*occupancy_grid),
        no_of_nodes_(no_of_nodes),
        generator(rd_engine()),
        ball_radius_(ball_radius),
        n_collision_checks_(n_collision_checks),
        obstacle_inflation_radius_(obstacle_inflation_radius),
        goal_tolerance_(goal_tolerance_),
        hg_ratio_(hg_ratio),
        online_(online),
        minimal_sampling_(minimal_sampling),
        sampling_tolerance_(sampling_tolerance),
        visualization_(visualization),
        start_node_ptr_(nullptr),
        goal_node_ptr_(nullptr),
        samples_pub_(samples_visualizer),
        tree_pub_(tree_visualizer),
        path_pub_(path_visualizer)
{
    ROS_INFO("Initializing Planner");

    // Get Map Setting
    occupancy_grid_cols_ = occupancy_grid_.info.width;
    occupancy_grid_resolution_ = occupancy_grid_.info.resolution;
    occupancy_grid_origin_x_ = occupancy_grid_.info.origin.position.x;
    occupancy_grid_origin_y_ = occupancy_grid_.info.origin.position.y;

    // Bounding Box in Map for Sampling if
    if(!minimal_sampling_)
    {
        x_min_ = sampling_rectangle[0];
        x_max_ = sampling_rectangle[1];
        y_min_ = sampling_rectangle[2];
        y_max_ = sampling_rectangle[3];

        ROS_INFO("x_min: %f", x_min_);
        ROS_INFO("x_max: %f", x_max_);
        ROS_INFO("y_min: %f", y_min_);
        ROS_INFO("y_max: %f", y_max_);

        std::uniform_real_distribution<>::param_type x_param(x_min_, x_max_);
        std::uniform_real_distribution<>::param_type y_param(y_min_, y_max_);
        dis_x.param(x_param);
        dis_y.param(y_param);

        ROS_INFO("Constructing Graph");
        if(!online)
        {
            construct_nodes_and_add_near_neighbors();
        }
        else
        {
            construct_nodes();
        }
    }
    else
    {
        ROS_INFO("Minimal Sampling is On. Waiting for Goal to construct the sampling graph.");
    }

    ROS_INFO("Planner Initialized");
}

/// This function runs the FMT star search and returns the path between the start and the goal
std::vector<std::array<double, 2>> Planner::get_plan(
        const std::array<double, 2> &start, const std::array<double, 2> &goal)
{
    // Initialize Data Structures used in FMT*
    ROS_DEBUG("Planning Started");
    std::unordered_set<Node*> unvisited_set{};
    for(auto &node: sampled_nodes_)
    {
        unvisited_set.insert(&node);
    }

    std::unordered_set<Node*> open_set{};

    // Priority Queue for Storing Least Cost Nodes
    auto less = [&](const Node *left, const Node *right) {
        return (left->g_cost+ hg_ratio_*left->h_cost) > (right->g_cost+hg_ratio_*right->h_cost);
    };
    std::priority_queue<Node*, std::vector<Node*>, decltype(less)> open_queue(less);
    // Construct start node, find neighbours and add to open set and queue
    Node start_node = Node(start[0],start[1]);
    start_node_ptr_ = &start_node;
    add_near_nodes(start_node_ptr_);

    if(!online_)
    {
        for(const auto& start_node_neighbor_ptr: start_node_ptr_->near_nodes)
        {
            start_node_neighbor_ptr->near_nodes.emplace_back(start_node_ptr_);
        }
    }

    open_set.insert(start_node_ptr_);
    open_queue.push(start_node_ptr_);
    // Add goal node to unvisited
    Node goal_node = Node(goal[0],goal[1]);
    goal_node_ptr_ = &goal_node;
    add_near_nodes(goal_node_ptr_);
    unvisited_set.insert(goal_node_ptr_);

    if(!online_)
    {
        for(const auto& goal_node_neighbor_ptr: goal_node_ptr_->near_nodes)
        {
            goal_node_neighbor_ptr->near_nodes.emplace_back(&goal_node);
        }
    }

    start_node_ptr_->h_cost = start_node_ptr_->traversal_cost(goal_node_ptr_);

    Node* z_node_ptr = start_node_ptr_;

    // Until z is goal node
    while(z_node_ptr->traversal_cost(goal_node) > goal_tolerance_)
    {
        if(visualization_)
        {
            visualize_tree();
            ros::Duration(0.01).sleep();
        }

        std::unordered_set<Node *> open_new_set{};
        open_new_set.clear();

        for(const auto& x_node_ptr: z_node_ptr->near_nodes)
        {
            if(x_node_ptr->near_nodes.empty())
            {
                add_near_nodes(x_node_ptr);
            }
            //for all unvisited nodes x_node_ptr in neighbourhood of z_node_ptr
            if (unvisited_set.find(x_node_ptr) != unvisited_set.end())
            {
                // minimum cost node for x_node_ptr
                Node* y_min_node_ptr = nullptr;

                for (auto& y_node_ptr: x_node_ptr->near_nodes)
                {
                    //for all open y_nodes in neighbourhood of unvisited x_node_ptr
                    if (open_set.find(y_node_ptr) != open_set.end())
                    {
                        //find least cost path to x_node_ptr from open nodes y_node_ptr
                        if(y_min_node_ptr == nullptr)
                        {
                            y_min_node_ptr = y_node_ptr;
                        }
                        else if(y_min_node_ptr->g_cost + y_min_node_ptr->traversal_cost(x_node_ptr)
                                > y_node_ptr->g_cost + y_node_ptr->traversal_cost(x_node_ptr))
                        {
                            y_min_node_ptr = y_node_ptr;
                        }
                    }
                }
                // if no open nodes in neighbourhood of x_node_ptr
                if(y_min_node_ptr == nullptr) continue;

                if(is_collision_free(x_node_ptr, y_min_node_ptr))
                {
                    //make_y_node parent of x_node_ptr and set cost
                    x_node_ptr->parent_node = y_min_node_ptr;
                    x_node_ptr->g_cost = y_min_node_ptr->g_cost + y_min_node_ptr->traversal_cost(x_node_ptr);
                    x_node_ptr->h_cost = x_node_ptr->traversal_cost(goal_node_ptr_);

                    open_new_set.insert(x_node_ptr);
                    unvisited_set.erase(x_node_ptr);
                }
            }
        }

        // Remove z_node_ptr from open
        open_set.erase(z_node_ptr);
        open_queue.pop();

        if(!open_new_set.empty())
        {
            ROS_DEBUG("Adding new open set");
            open_set.insert(open_new_set.begin(), open_new_set.end());
            for (auto &node_ptr: open_new_set) {
                open_queue.push(node_ptr);
            }
        }

        // No path found
        if(open_set.empty())
        {
            ROS_ERROR("Open Set is Empty. Planner Failed to find a Path");
            return {};
        }

        // Least cost node in open
        z_node_ptr = open_queue.top();
    }

    return generate_path(z_node_ptr);
}

/// Updates the occupancy grid with the latest one
/// @param occupancy_grid
void Planner::update_occupancy_grid(nav_msgs::OccupancyGridConstPtr occupancy_grid,
                                    const std::array<double, 2>& start,
                                    const std::array<double, 2>& goal)
{
    occupancy_grid_ = *occupancy_grid;
    sampled_nodes_.clear();
    start_node_x_ = start[0];
    start_node_y_ = start[1];
    goal_node_x_ = goal[0];
    goal_node_y_ = goal[1];
    refresh_sampling();
    if(!online_)
    {
        construct_nodes_and_add_near_neighbors();
    }
    else
    {
        construct_nodes();
    }
    visualize_samples();
    ROS_DEBUG("Updated Occupancy Grid");
}

// TODO: Improve over this naive method for collision checking as well as obstacle inflation
/// Check if there was a collision between two nodes (Internally does Obstacle Inflation)
/// @param node1
/// @param node2
/// @return return true if there is no collision between two nodes
bool Planner::is_collision_free(Node* node1, Node* node2) const
{
    double current_x = node1->x;
    double current_y = node1->y;
    double diff_x = (node2->x-node1->x)/10;
    double diff_y = (node2->y-node1->y)/10;
    for(int ii=0; ii<10; ii++)
    {
        const auto x_index = static_cast<int>((current_x - occupancy_grid_origin_x_)/occupancy_grid_resolution_);
        const auto y_index = static_cast<int>((current_y - occupancy_grid_origin_y_)/occupancy_grid_resolution_);
        for(int i=-obstacle_inflation_radius_+x_index; i<obstacle_inflation_radius_+1+x_index; i++)
        {
            for(int j=-obstacle_inflation_radius_+y_index; j<obstacle_inflation_radius_+1+y_index; j++)
            {
                if(occupancy_grid_.data[j*occupancy_grid_cols_ + i]==100)
                {
                    return false;
                }
            }
        }
        current_x = current_x + ii*diff_x;
        current_y = current_y + ii*diff_y;
    }
    return true;
}

/// Generates path from goal node to start node
/// \param goal_node
/// \return vector of (x,y) in map frame denoting path
std::vector<std::array<double,2>> Planner::generate_path(fmt_star::Node *node)
{
    std::vector<std::array<double,2>> path;
    ROS_DEBUG("Goal Reached. Backtracking ... ");
    while(node->parent_node!= nullptr)
    {
        path.emplace_back(std::array<double,2>({node->x,node->y}));
        node = node->parent_node;
    }
    ROS_INFO("Plan Ready");
    std::reverse(path.begin(), path.end());
    if(visualization_)
    {
        std::vector<std::array<double,2>> visualization_path;
        for(const auto& path_node: path)
        {
            visualization_path.emplace_back(path_node);
            visualize_path(visualization_path);
            ros::Duration(0.01).sleep();
        }
    }
    visualize_path(path);
    return path;
}

/// Sets up graph nodes - Samples N points and then constructs each node
/// @param no_of_nodes
void Planner::construct_nodes_and_add_near_neighbors()
{
    construct_nodes();
    for(auto& node: sampled_nodes_)
    {
        add_near_nodes(&node);
    }
}

/// Constructs all the nodes
void Planner::construct_nodes()
{
    for (size_t iter = 0; iter < no_of_nodes_; iter++)
    {
        auto x_map = dis_x(generator);
        auto y_map = dis_y(generator);
        if(x_map < occupancy_grid_origin_x_ || y_map < occupancy_grid_origin_y_ ||
        x_map >= occupancy_grid_origin_x_ + occupancy_grid_.info.width*occupancy_grid_resolution_ ||
        y_map >= occupancy_grid_origin_y_ + occupancy_grid_.info.height*occupancy_grid_resolution_)
        {
            iter--;
            continue;
        }
        const auto rmi = row_major_index(x_map, y_map);
        if(rmi < occupancy_grid_.data.size() || occupancy_grid_.data[rmi] == 100)
        {
            sampled_nodes_.emplace_back(Node{x_map, y_map});
        }
    }
}

/// Fills the near node vector of the input node
/// @param node - current node
void Planner::add_near_nodes(Node* node)
{
    for(auto& sample_node: sampled_nodes_)
    {
        if(node->traversal_cost(sample_node) < ball_radius_)
        {
            node->near_nodes.push_back(&sample_node);
        }
    }
    if(start_node_ptr_ && node != start_node_ptr_)
    {
        if(node->traversal_cost(start_node_ptr_) < ball_radius_)
        {
            node->near_nodes.push_back(start_node_ptr_);
        }
    }
    if(goal_node_ptr_ && node != goal_node_ptr_)
    {
        if(node->traversal_cost(goal_node_ptr_) < ball_radius_)
        {
            node->near_nodes.push_back(goal_node_ptr_);
        }
    }
}

/// Refreshes the samples around the current start node
void Planner::refresh_sampling()
{
    ROS_DEBUG("start node x %f", start_node_x_);
    ROS_DEBUG("start node y %f", start_node_y_);
    ROS_DEBUG("goal node x %f", goal_node_x_);
    ROS_DEBUG("goal node y %f", goal_node_y_);
    ROS_DEBUG("x min %f", x_min_);
    ROS_DEBUG("x max %f", x_max_);
    ROS_DEBUG("y min %f", y_min_);
    ROS_DEBUG("y max %f", y_max_);

    if(minimal_sampling_)
    {
        double less_x = start_node_x_ > goal_node_x_ ? goal_node_x_ : start_node_x_;
        double less_y = start_node_y_ > goal_node_y_ ? goal_node_y_ : start_node_y_;
        double more_x =  start_node_x_ > goal_node_x_? start_node_x_ : goal_node_x_;
        double more_y = start_node_y_ > goal_node_y_ ? start_node_y_ : goal_node_y_;

        std::uniform_real_distribution<>::param_type x_param(less_x - sampling_tolerance_, more_x + sampling_tolerance_);
        std::uniform_real_distribution<>::param_type y_param(less_y - sampling_tolerance_, more_y + sampling_tolerance_);
        dis_x.param(x_param);
        dis_y.param(y_param);
        return;
    }

    std::uniform_real_distribution<>::param_type x_param(start_node_x_ + x_min_, start_node_x_ + x_max_);
    std::uniform_real_distribution<>::param_type y_param(start_node_y_ + y_min_, start_node_y_ + y_max_);
    dis_x.param(x_param);
    dis_y.param(y_param);
}

/// Get Row Major Index corresponding to the occupancy grid initialized in the planner class
size_t Planner::row_major_index(double x, double y)
{
    const auto x_index = static_cast<size_t>((x - occupancy_grid_origin_x_)/occupancy_grid_resolution_);
    const auto y_index = static_cast<size_t>((y - occupancy_grid_origin_y_)/occupancy_grid_resolution_);
    return y_index*occupancy_grid_cols_ + x_index;
}

/// Visualize Input Vector of 2 element array
/// @param input
void Planner::visualize_path(const std::vector<std::array<double,2>>& input)
{
    visualization_msgs::Marker line;
    line.header.frame_id = "/map";
    line.header.stamp = ros::Time::now();
    line.ns = "path";
    line.action = visualization_msgs::Marker::ADD;
    line.pose.orientation.w = 1.0;
    line.id = 1;
    line.type = visualization_msgs::Marker::LINE_STRIP;
    line.scale.x = 0.1;
    line.color.r = 1.0f;
    line.color.a = 1.0;
    line.lifetime = ros::Duration(5);
    geometry_msgs::Point start_point;
    start_point.x = start_node_ptr_->x;
    start_point.y = start_node_ptr_->y;
    line.points.push_back(start_point);
    for(const auto& node: input)
    {
        geometry_msgs::Point point;
        point.x = node[0];
        point.y = node[1];
        line.points.push_back(point);
    }
    path_pub_->publish(line);
}

/// Visualize Tree
void Planner::visualize_tree()
{
    visualization_msgs::MarkerArray tree;
    int i = 1;
    for(const auto& sampled_node: sampled_nodes_)
    {
        if(sampled_node.parent_node)
        {
            visualization_msgs::Marker line;
            line.header.frame_id = "/map";
            line.header.stamp = ros::Time::now();
            line.ns = "tree";
            line.action = visualization_msgs::Marker::ADD;
            line.pose.orientation.w = 1.0;
            line.id = i; i++;
            line.type = visualization_msgs::Marker::LINE_STRIP;
            line.scale.x = 0.02;
            line.color.b = 1.0f;
            line.color.a = 1.0;
            geometry_msgs::Point parent;
            parent.x = sampled_node.parent_node->x;
            parent.y = sampled_node.parent_node->y;
            line.points.push_back(parent);
            geometry_msgs::Point child;
            child.x = sampled_node.x;
            child.y = sampled_node.y;
            line.points.push_back(child);
            line.lifetime = ros::Duration(10);
            tree.markers.push_back(std::move(line));
        }
    }
    tree_pub_->publish(tree);
}

/// Visualize all the Samples
void Planner::visualize_samples()
{
    visualization_msgs::MarkerArray samples;
    int i = 1;
    for(const auto& sampled_node: sampled_nodes_)
    {
        visualization_msgs::Marker point;
        point.header.frame_id = "/map";
        point.header.stamp = ros::Time::now();
        point.ns = "samples";
        point.action = visualization_msgs::Marker::ADD;
        point.id = i; i++;
        point.type = visualization_msgs::Marker::SPHERE;
        point.pose.position.x = sampled_node.x;
        point.pose.position.y = sampled_node.y;
        point.pose.position.z = 0;
        point.pose.orientation.x = 0.0;
        point.pose.orientation.y = 0.0;
        point.pose.orientation.z = 0.0;
        point.pose.orientation.w = 1.0;
        point.scale.x = 0.1;
        point.scale.y = 0.1;
        point.scale.z = 0.1;
        point.color.a = 1.0;
        point.color.r = 0.0;
        point.color.g = 1.0;
        point.color.b = 0.0;
        point.lifetime = ros::Duration(10);
        samples.markers.emplace_back(std::move(point));
    }
    ROS_DEBUG("Publishing %i sample points", static_cast<int>(sampled_nodes_.size()));
    samples_pub_->publish(samples);
}

/// Helper function to convert sequence of user coordinates from non ros map to ros coordinates
/// \note This function is only useful for scaling and flipping of coordinates based on the parameters passed to the
/// function
/// \param nonros_coords - sequence of coords in nonros coord system
/// \param nonros_map_width - width of map in nonros coord system
/// \param nonros_map_height - height of map in nonros coord system
/// \param resolution - resolution of ros map (m/cell)
/// \param xy_switched - is x and y switched from nonros to ros coords
/// \param ros_map_width - width of map in ros coords system
/// \param ros_map_height - height of map in ros coords system
/// \param ros_map_origin_x - origin of map x ros coord
/// \param ros_map_origin_y - origin of map y ros coord
/// \return sequence of coords in ros coords system
std::vector<std::array<double, 2>> translate_sequence_to_ros_coords(const std::vector<std::array<int, 2>>& nonros_coords,
                                                                    const double nonros_map_width,
                                                                    const double nonros_map_height,
                                                                    const bool xy_switched,
                                                                    const double ros_map_width,
                                                                    const double ros_map_height,
                                                                    const double ros_map_origin_x,
                                                                    const double ros_map_origin_y)
{
    std::vector<std::array<double, 2>> ros_coords;
    int x_index = xy_switched ? 1 : 0;
    int y_index = xy_switched ? 0 : 1;
    for(const auto& nonros_coord: nonros_coords)
    {
        const double x = ros_map_origin_x + (nonros_coord[x_index] * ros_map_width ) / (nonros_map_width);
        const double y =
                ros_map_origin_y + ros_map_height * (1 - (nonros_coord[y_index] / nonros_map_height));
        ros_coords.emplace_back(std::array<double, 2>{x, y});
        ROS_DEBUG("ROS COORDS: (%f, %f)", x, y);
    }
    return ros_coords;
}

} // namespace fmt_star

#endif //SRC_PLANNER_IMPL_H
