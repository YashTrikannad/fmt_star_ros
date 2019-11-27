#ifndef SRC_PLANNER_IMPL_H
#define SRC_PLANNER_IMPL_H

//#include "planner.h"

#include <unordered_set>
#include <queue>
#include <utility>

namespace fmt_star
{

/// Creates a Planner instance with the input occupancy grid as the map
/// @param occupancy_grid - current occupancy grid (map)
/// @param no_of_nodes - No. of nodes to sample over a map
/// @param ball_radius - radius to be considered for a sample to be near neighbor
/// @param obstacle_inflation_radius - safety boundary around obstacles
/// @param sampling_rectangle - Rectangle defining the boundary for sampling nodes
Planner::Planner(nav_msgs::OccupancyGrid occupancy_grid,
                 size_t no_of_nodes,
                 double ball_radius,
                 size_t n_collision_checks,
                 int obstacle_inflation_radius,
                 double goal_tolerance_,
                 const std::array<double, 4> &sampling_rectangle) :
        occupancy_grid_(std::move(occupancy_grid)),
        generator(rd_engine()),
        ball_radius_(ball_radius),
        n_collision_checks_(n_collision_checks),
        obstacle_inflation_radius_(obstacle_inflation_radius),
        goal_tolerance_(goal_tolerance_)
{
    occupancy_grid_cols_ = occupancy_grid_.info.width;
    occupancy_grid_resolution_ = occupancy_grid_.info.resolution;
    occupancy_grid_origin_x_ = occupancy_grid_.info.origin.position.x;
    occupancy_grid_origin_y_ = occupancy_grid_.info.origin.position.y;

    std::uniform_real_distribution<>::param_type x_param(sampling_rectangle[0], sampling_rectangle[1]);
    std::uniform_real_distribution<>::param_type y_param(sampling_rectangle[2], sampling_rectangle[3]);
    dis_x.param(x_param);
    dis_y.param(y_param);

    setup_graph_nodes(no_of_nodes);
}

/// This function runs the FMT star search and returns the path between the start and the goal
std::vector<std::array<double, 2>> Planner::get_plan(
        const std::array<double, 2> &start, const std::array<double, 2> &goal)
{
    // Initialize Data Structures used in FMT*
    std::unordered_set<Node*> unvisited_set{};
    for(auto &node: sampled_nodes_)
    {
        unvisited_set.insert(&node);
    }

    std::unordered_set<Node*> open_set{};

    // Priority Queue for Storing Least Cost Nodes
    auto less = [&](const Node *left, const Node *right) {
        return left->cost > right->cost;
    };
    std::priority_queue<Node*, std::vector<Node*>, decltype(less)> open_queue(less);

    // Construct start node,find neighbours and add to open set and queue
    Node start_node = Node(start[0],start[1]);
    Node* start_node_ptr = &start_node;
    add_near_nodes(start_node_ptr);

    for(const auto& start_node_neighbor_ptr: start_node_ptr->near_nodes)
    {
        start_node_neighbor_ptr->near_nodes.emplace_back(start_node_ptr);
    }

    open_set.insert(start_node_ptr);
    open_queue.push(start_node_ptr);

    // Add goal node to unvisited
    Node goal_node = Node(goal[0],goal[1]);
    Node* goal_node_ptr = &goal_node;
    add_near_nodes(goal_node_ptr);
    unvisited_set.insert(goal_node_ptr);

    for(const auto& goal_node_neighbor_ptr: goal_node_ptr->near_nodes)
    {
        goal_node_neighbor_ptr->near_nodes.emplace_back(&goal_node);
    }

    Node* z_node_ptr = start_node_ptr;

    // Until z is goal node
    while(z_node_ptr->traversal_cost(goal_node) > goal_tolerance_)
    {
        std::unordered_set<Node *> open_new_set{};
        open_new_set.clear();

        for(const auto& x_node_ptr: z_node_ptr->near_nodes)
        {
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
                        else if(y_min_node_ptr->cost + y_min_node_ptr->traversal_cost(x_node_ptr)
                                > y_node_ptr->cost + y_node_ptr->traversal_cost(x_node_ptr))
                        {
                            y_min_node_ptr = y_node_ptr;
                        }
                    }
                }
                // if no open nodes in neighbourhood of x_node_ptr
                if(y_min_node_ptr == nullptr)
                {
                    continue;
                }

                if(is_collision_free(x_node_ptr, y_min_node_ptr))
                {
                    //make_y_node parent of x_node_ptr and set cost
                    x_node_ptr->parent_node = y_min_node_ptr;
                    x_node_ptr->cost = y_min_node_ptr->cost + y_min_node_ptr->traversal_cost(x_node_ptr);

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
            ROS_ERROR("No Path Found");
            return {};
        }

        // Least cost node in open
        z_node_ptr = open_queue.top();
    }

    return generate_path(z_node_ptr);
}

/// Updates the occupancy grid with the latest one
/// @param occupancy_grid
void Planner::update_occupancy_grid(const nav_msgs::OccupancyGrid& occupancy_grid)
{
    occupancy_grid_ = occupancy_grid;
}

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

std::vector<std::array<double,2>> Planner::generate_path(fmt_star::Node *node)
{
    std::vector<std::array<double,2>> path;
    ROS_INFO("Path Found. Backtracking ... ");
    while(node->parent_node!= nullptr)
    {
        std::array<double,2> coords = {node->x,node->y};
        std::cout << "" << coords[0] << " " << coords[1] << std::endl;
        path.push_back(coords);
        node = node->parent_node;
    }
    ROS_INFO("Backtracking successful");
    return path;
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
            node->near_nodes.push_back(&sample_node);
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
