#ifndef SRC_PLANNER_IMPL_H
#define SRC_PLANNER_IMPL_H

#include "planner.h"

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
        const std::array<double, 2> &start, const std::array<double, 2> &goal)///const
{
    //declare data structures
    std::unordered_set<Node *> unvisited_set(sampled_nodes_.begin(),sampled_nodes_.end());
    std::unordered_set<Node *> closed_set{};
    std::unordered_set<Node *> open_set{};
    std::unordered_set<Node *> open_new_set{};

    auto less = [&](const Node *left, const Node *right) {
        return left->cost > right->cost;
    };
    std::priority_queue<Node*, std::vector<Node*>, decltype(less)> open_queue(less);

    //construct start node,find neighbours and add to open set and queue
    Node* start_node = new Node(start[0],start[1]);
    add_near_nodes(start_node);
    open_set.insert(start_node);
    open_queue.push(start_node);

    //add goal node to unvisited - no need to find neighbours for goal node(?)
    unvisited_set.insert(new Node(goal[0],goal[1]));

    Node* z_node = start_node;

    //till z is goal node
    while(z_node->x != goal[0] || z_node->y != goal[1])
    {
        open_new_set.clear();

        for(auto& x_node: z_node->near_nodes)
        {   //for all unvisited nodes x_node in neighbourhood of z_node
            if (unvisited_set.count(x_node))
            {
                // minimum cost node for x_node
                Node* y_min_node = nullptr;

                for (auto& y_node: x_node->near_nodes)
                {   //for all open y_node in neighbourhood of unvisited x_node
                    if (open_set.count(y_node))
                    {
                        //find least cost path to x_node from open nodes y_node
                        if(y_min_node == nullptr)
                            y_min_node = y_node;
                        else if(y_min_node->cost + get_node_to_node_cost(y_min_node,x_node)
                                    > y_node->cost + get_node_to_node_cost(y_node,x_node))
                            y_min_node = y_node;
                    }
                }
                // if no open nodes in neighbourhood of x_node
                if(y_min_node == nullptr)
                    continue;
                if(is_collision_free(x_node,y_min_node))///?????
                {
                    //make_y_node parent of x_node and set cost
                    x_node->parent_node = y_min_node;
                    x_node->cost = y_min_node->cost + get_node_to_node_cost(x_node, y_min_node);

                    open_new_set.insert(x_node);
                    unvisited_set.erase(x_node);

                }

            }
        }
        //remove z_node from open
        open_set.erase(z_node);
        open_queue.pop();

        //add z_node to closed
        closed_set.insert(z_node);

        if(!open_new_set.empty())
        {   //add open_new to open
            open_set.insert(open_new_set.begin(), open_new_set.end());
            for (auto &node: open_new_set) {
                open_queue.push(node);
            }
        }

        // no path found
        if(open_set.empty())
            return {};

        //least cost node in open
        z_node = open_queue.top();

    }

    return generate_path(z_node);
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
/// @return return true if there is no collision between two nodes
bool Planner::is_collision_free(Node* node1, Node* node2)
{
    double current_x = node1->x;
    double current_y = node1->y;
    double diff_x = (node2->x-node1->x)/10;
    double diff_y = (node2->y-node1->y)/10;
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

std::vector<std::array<double,2>> Planner::generate_path(fmt_star::Node *node)
{
    std::vector<std::array<double,2>> path;

    while(node->parent_node!= nullptr)
    {
        std::array<double,2> coods = {node->x,node->y};
        //std::cout<<coods<<std::endl;

        path.push_back(coods);
        node = node->parent_node;

    }
    return path;
}

double Planner::get_node_to_node_cost(fmt_star::Node* node1, fmt_star::Node* node2)///make static?
{
    return sqrt(pow(node1->x - node2->x, 2) + pow(node1->y - node2->y, 2));
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
