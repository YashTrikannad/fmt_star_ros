#ifndef SRC_PLANNER_IMPL_H
#define SRC_PLANNER_IMPL_H

#include "planner.h"

namespace fmt_star
{

/// Creates a Planner instance with the input occupancy grid as the map
/// @param occupancy_grid
Planner::Planner(const nav_msgs::OccupancyGrid& occupancy_grid,
                 size_t no_of_nodes,
                 double ball_radius) :
        generator(rd_engine()),
        ball_radius_(ball_radius)
{
    occupancy_grid_ = occupancy_grid;
    occupancy_grid_cols_ = occupancy_grid.info.width;
    occupancy_grid_resolution_ = occupancy_grid.info.resolution;
    occupancy_grid_origin_x_ = occupancy_grid.info.origin.position.x;
    occupancy_grid_origin_y_ = occupancy_grid.info.origin.position.y;
    setup_graph_nodes(no_of_nodes);
}

/// This function runs the FMT star search and returns the path between the start and the goal
/// @param start
/// @param goal
/// @return
std::vector<int> Planner::get_plan(const int start, const int goal) const
{
    // FMT Star Algorithm
    return {};
}

/// Updates the occupancy grid with the latest one
/// @param occupancy_grid
void Planner::update_occupancy_grid(const nav_msgs::OccupancyGrid& occupancy_grid)
{
    occupancy_grid_ = occupancy_grid;
}

/// Sets up graph nodes - Samples N points and then constructs each node
/// @param no_of_nodes
void Planner::setup_graph_nodes(size_t no_of_nodes)
{
    for(size_t iter=0; iter<no_of_nodes; iter++)
    {
        construct_node(-15, 15, -10, 10);
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
void Planner::construct_node(double x_min, double x_max, double y_min, double y_max)
{
    std::uniform_real_distribution<double> dis_x(x_min, x_max);
    std::uniform_real_distribution<double> dis_y(y_min, y_max);
    auto x_map = dis_x(generator);
    auto y_map = dis_y(generator);
    while(occupancy_grid_.data[row_major_index(x_map, y_map)] == 100)
    {
        x_map = dis_x(generator);
        y_map = dis_y(generator);
    };
    sampled_nodes_.emplace_back(Node(x_map, y_map));
}

/// Fills the near node vector of the input node
/// @param node - current node
void Planner::add_near_nodes(Node* node)
{
    for(auto& sample_node: sampled_nodes_)
    {
        if(sqrt(pow(node->x - sample_node.x, 2) - pow(node->y - sample_node.y, 2)) < ball_radius_)
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
    std::array<double, 2> xy_coordinates;
    xy_coordinates[1] = static_cast<int>(row_major_index/occupancy_grid_cols_);
    xy_coordinates[0] = row_major_index - (xy_coordinates[1] * occupancy_grid_cols_);
    return xy_coordinates;
}

} // namespace fmt_star

#endif //SRC_PLANNER_IMPL_H
