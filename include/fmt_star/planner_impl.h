#ifndef SRC_PLANNER_IMPL_H
#define SRC_PLANNER_IMPL_H

//#include "planner.h"

#include <unordered_set>
#include <queue>

namespace fmt_star
{

/// Creates a Planner instance with the input occupancy grid as the map
/// @param occupancy_grid
Planner::Planner(const nav_msgs::OccupancyGrid &occupancy_grid,
                 size_t no_of_nodes,
                 double ball_radius,
                 const std::array<double, 4> &sampling_rectangle) :
        generator(rd_engine()),
        ball_radius_(ball_radius)
{
    occupancy_grid_ = occupancy_grid;
    occupancy_grid_cols_ = occupancy_grid.info.width;
    occupancy_grid_resolution_ = occupancy_grid.info.resolution;
    occupancy_grid_origin_x_ = occupancy_grid.info.origin.position.x;
    occupancy_grid_origin_y_ = occupancy_grid.info.origin.position.y;

    std::uniform_real_distribution<>::param_type x_param(sampling_rectangle[0], sampling_rectangle[1]);
    std::uniform_real_distribution<>::param_type y_param(sampling_rectangle[2], sampling_rectangle[3]);
    dis_x.param(x_param);
    dis_y.param(y_param);

    setup_graph_nodes(no_of_nodes);
}

/// This function runs the FMT star search and returns the path between the start and the goal
std::vector<std::array<double, 2>> Planner::get_plan(
        const std::array<double, 2> &start, const std::array<double, 2> &goal) const
{
    std::unordered_set<Node *> visited_set{};
    std::unordered_set<Node *> open_set{};

    auto less = [&](const Node *left, const Node *right) {
        return left->cost > right->cost;
    };
    std::priority_queue<Node*, std::vector<Node*>, decltype(less)> open_queue(less);

    // TODO: FMT* Path Planning

    return {};
}

/// Inflate the obstacle with the obstacle radius
void Planner::do_obstacle_inflation()
{
    // TODO: Obstacle Inflation
}

/// Updates the occupancy grid with the latest one
/// @param occupancy_grid
void Planner::update_occupancy_grid(const nav_msgs::OccupancyGrid& occupancy_grid)
{
    occupancy_grid_ = occupancy_grid;
}

/// Check if there was a collision between two nodes
/// @param node1
/// @param node2
/// @return return true if there was a collision between two nodes
bool Planner::is_collision_free(const Node& node1, const Node& node2)
{
    double current_x = node1.x;
    double current_y = node1.y;
    double diff_x = (node2.x-node1.x)/10;
    double diff_y = (node2.y-node1.y)/10;
    for(int i=0; i<10; i++)
    {
        if(occupancy_grid_.data[row_major_index(current_x, current_y)]==100)
        {
            return false;
        }
        current_x = current_x + i*diff_x;
        current_y = current_y + i*diff_y;
    }
    return true;
}

/// Sets up graph nodes - Samples N points and then constructs each node
/// @param no_of_nodes
void Planner::setup_graph_nodes(size_t no_of_nodes)
{
    for(size_t iter=0; iter<no_of_nodes; iter++)
    {
        construct_node();
    }
    for(auto& node: sampled_nodes_)
    {
        add_near_nodes(&node);
    }
}

/// Constructs a Node by Sampling a x, y point from the Occupancy Grid with limits and adds it to
/// the vector of sampled nodes
/// @param x_min - x lower limit for sampling
/// @param x_max - x higher limit for sampling
/// @param y_min - y lower limit for sampling
/// @param y_max - y higher limit for sampling
void Planner::construct_node()
{
    auto x_map = dis_x(generator);
    auto y_map = dis_y(generator);
    while(occupancy_grid_.data[row_major_index(x_map, y_map)] == 100)
    {
        x_map = dis_x(generator);
        y_map = dis_y(generator);
    };
    sampled_nodes_.emplace_back(Node{x_map, y_map});
}

/// Fills the near node vector of the input node
/// @param node - current node
void Planner::add_near_nodes(Node* node)
{
    for(auto& sample_node: sampled_nodes_)
    {
        if(sqrt(pow(node->x - sample_node.x, 2) + pow(node->y - sample_node.y, 2)) < ball_radius_)
        {
            node->near_nodes.emplace_back(&sample_node);
        }
    }
}

/// Get Row Major Index corresponding to the occupancy grid initialized in the planner class
size_t Planner::row_major_index(double x, double y)
{
    const auto x_index = static_cast<size_t>((x - occupancy_grid_origin_x_)/occupancy_grid_resolution_);
    const auto y_index = static_cast<size_t>((y - occupancy_grid_origin_y_)/occupancy_grid_resolution_);
    return y_index*occupancy_grid_cols_ + x_index;
}

/// Get the (x, y) position in map frame given the row major index
/// @param row_major_index of position in the map
/// @return (x, y) position in map
std::array<double, 2> Planner::get_xy(size_t row_major_index)
{
    std::array<double, 2> xy_coordinates{};
    xy_coordinates[1] = static_cast<int>(row_major_index/occupancy_grid_cols_);
    xy_coordinates[0] = row_major_index - (xy_coordinates[1] * occupancy_grid_cols_);
    return xy_coordinates;
}

} // namespace fmt_star

#endif //SRC_PLANNER_IMPL_H
