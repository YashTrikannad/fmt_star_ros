#ifndef SRC_PLANNER_H
#define SRC_PLANNER_H

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

namespace fmt_star
{

struct Node
{
    double x;
    double y;
    double cost;
    int index;
    int parent_index;
};

class Planner
{
public:
    /// Creates a Planner instance with the input occupancy grid as the map
    /// @param occupancy_grid
    explicit Planner(const nav_msgs::OccupancyGrid& occupancy_grid)
    {
        occupancy_grid_ = occupancy_grid;
    }

    /// Updates the occupancy grid with the latest one
    /// @param occupancy_grid
    void update_occupancy_grid(const nav_msgs::OccupancyGrid& occupancy_grid);

    /// This function runs the FMT star search and returns the path between the start and the goal
    /// @param start
    /// @param goal
    /// @return
    std::vector<int> get_plan(const int start, const int goal) const;


private:
    nav_msgs::OccupancyGrid occupancy_grid_;
};

} // namespace fmt_star

#endif //SRC_PLANNER_H

#include "fmt_star/planner_impl.h"