#ifndef SRC_PLANNER_H
#define SRC_PLANNER_H

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <random>

namespace fmt_star
{

struct Node
{
    Node() = default;
    Node(double x, double y) : x(x), y(y), g_cost(0.0), h_cost(0.0), parent_node(nullptr), near_nodes({})
    {}
    double x;
    double y;
    double g_cost;
    double h_cost;
    Node* parent_node;
    std::vector<Node*> near_nodes;

    double traversal_cost(const Node& other_node) const
    {
        return sqrt(pow(x-other_node.x,2)+pow(y-other_node.y,2));
    }
    double traversal_cost(Node* other_node) const
    {
        return sqrt(pow(x-other_node->x,2)+pow(y-other_node->y,2));
    }
};

class Planner
{
public:
    /// Creates a Planner instance with the input occupancy grid as the map
    /// @param occupancy_grid - current occupancy grid (map)
    /// @param no_of_nodes - No. of nodes to sample over a map
    /// @param ball_radius - radius to be considered for a sample to be near neighbor
    /// @param obstacle_inflation_radius - safety boundary around obstacles
    /// @param sampling_rectangle - Rectangle defining the boundary for sampling nodes
    explicit Planner(nav_msgs::OccupancyGridConstPtr occupancy_grid,
                    size_t no_of_nodes,
                    double ball_radius,
                    size_t n_collision_checks,
                    int obstacle_inflation_radius,
                    double goal_tolerance,
                    double hg_ratio,
                    bool online,
                    const std::array<double, 4>& sampling_rectangle,
                    bool visualization,
                    ros::Publisher* samples_pub,
                    ros::Publisher* tree_visualizer,
                    ros::Publisher* path_visualizer);

    /// Updates the occupancy grid with the latest one
    /// @param occupancy_grid
    void update_occupancy_grid(nav_msgs::OccupancyGridConstPtr occupancy_grid,
                               const std::array<double, 2>& start,
                               const std::array<double, 2>& goal);

    /// This function runs the FMT star search and returns the path between the start and the goal
    /// @param start - (x, y) position of start in map frame
    /// @param goal - (x, y) position of goal in map frame
    /// @return vector of (x, y) positions along the path from start to goal in map frame
    std::vector<std::array<double, 2>> get_plan(
            const std::array<double, 2>& start, const std::array<double, 2>&goal);

private:
    ros::Publisher* samples_pub_;
    ros::Publisher* tree_pub_;
    ros::Publisher* path_pub_;

    nav_msgs::OccupancyGrid occupancy_grid_;
    size_t occupancy_grid_cols_;
    double occupancy_grid_resolution_;
    double occupancy_grid_origin_x_;
    double occupancy_grid_origin_y_;

    size_t no_of_nodes_;
    double ball_radius_;
    size_t n_collision_checks_;
    int obstacle_inflation_radius_;
    double goal_tolerance_;
    double hg_ratio_;
    bool online_;
    bool visualization_;

    Node* start_node_ptr_;
    Node* goal_node_ptr_;

    std::random_device rd_engine;
    std::mt19937 generator;

    std::vector<Node> sampled_nodes_;

    double x_min_;
    double x_max_;
    double y_min_;
    double y_max_;
    double start_node_x_;
    double start_node_y_;
    double goal_node_x_;
    double goal_node_y_;
    std::uniform_real_distribution<double> dis_x;
    std::uniform_real_distribution<double> dis_y;

    /// Refreshes the samples around the current start node
    void refresh_sampling();

    /// Sets up graph nodes - Samples N points and then constructs each node
    void construct_nodes_and_add_near_neighbors();

    /// Constructs all the nodes by Sampling a x, y point from the Occupancy Grid with limits and adds it to
    /// the vector of sampled nodes
    void construct_nodes();

    /// Fills the near node vector of the input node
    /// @param node - current node
    void add_near_nodes(Node* node);

    /// Generates path from goal node to start node
    /// \param goal_node
    /// \return vector of (x,y) denoting path
    std::vector<std::array<double,2>> generate_path(Node* goal_node);

    /// Check if there was a collision between two nodes
    /// @param node1
    /// @param node2
    /// @return return true if there was a collision between two nodes
    bool is_collision_free(Node* node1, Node* node2) const;

    /// Get Row Major Index corresponding to the occupancy grid initialized in the planner class
    ///
    /// @param x - x position in map frame
    /// @param y - y position in map frame
    /// @return row major index
    size_t row_major_index(double x, double y);

    /// Visualize Input Vector of 2 element array
    /// @param input
    void visualize_path(const std::vector<std::array<double,2>>& input);

    /// Visualize Tree
    void visualize_tree();

    /// Visualize all the Samples
    void visualize_samples();
};

} // namespace fmt_star

#endif //SRC_PLANNER_H

#include "fmt_star/planner_impl.h"