#ifndef SRC_PLANNER_H
#define SRC_PLANNER_H

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <random>

namespace fmt_star
{

struct Node
{
    Node(double x, double y) : x(x), y(y), cost(0.0), parent_node(nullptr), near_nodes({})
    {}
    double x;
    double y;
    double cost;
    Node* parent_node;
    std::vector<Node*> near_nodes;
};

class Planner
{
public:
    /// Creates a Planner instance with the input occupancy grid as the map
    /// @param occupancy_grid
    explicit Planner(const nav_msgs::OccupancyGrid& occupancy_grid,
                    size_t no_of_nodes,
                    double ball_radius);

    /// Updates the occupancy grid with the latest one
    /// @param occupancy_grid
    void update_occupancy_grid(const nav_msgs::OccupancyGrid& occupancy_grid);

    /// This function runs the FMT star search and returns the path between the start and the goal
    /// @param start - row major index of the start in the map
    /// @param goal - row major index of the goal in the map
    /// @return vector of row major indices of the obstacle free path from the start to goal
    std::vector<int> get_plan(int start, int goal) const;

    std::vector<std::array<double, 2>> get_sampled_nodes()
    {
        std::vector<std::array<double, 2>> sampled_nodes_xy;
        for(const auto& node:sampled_nodes_)
        {
            sampled_nodes_xy.push_back({node.x, node.y});
        }
        return sampled_nodes_xy;
    }

private:
    nav_msgs::OccupancyGrid occupancy_grid_;
    size_t occupancy_grid_cols_;
    double occupancy_grid_resolution_;
    double occupancy_grid_origin_x_;
    double occupancy_grid_origin_y_;

    double ball_radius_;

    std::random_device rd_engine;
    std::mt19937 generator;

    std::vector<Node> sampled_nodes_;

    /// Sets up graph nodes - Samples N points and then constructs each node
    /// @param no_of_nodes
    void setup_graph_nodes(size_t no_of_nodes);

    /// Constructs a Node by Sampling a x, y point from the Occupancy Grid with limits and adds it to
    /// the vector of sampled nodes
    /// @param x_min - x lower limit for sampling
    /// @param x_max - x higher limit for sampling
    /// @param y_min - y lower limit for sampling
    /// @param y_max - y higher limit for sampling
    void construct_node(double x_min, double x_max, double y_min, double y_max);

    /// Fills the near node vector of the input node
    /// @param node - current node
    void add_near_nodes(Node* node);

    /// Get Row Major Index corresponding to the occupancy grid initialized in the planner class
    ///
    /// @param x - x position in map frame
    /// @param y - y position in map frame
    /// @return row major index
    size_t row_major_index(double x, double y);

    /// Get the (x, y) position in map frame given the row major index
    /// @param row_major_index of position in the map
    /// @return (x, y) position in map
    std::array<double, 2> get_xy(size_t row_major_index);
};

} // namespace fmt_star

#endif //SRC_PLANNER_H

#include "fmt_star/planner_impl.h"