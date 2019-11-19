#ifndef SRC_PLANNER_IMPL_H
#define SRC_PLANNER_IMPL_H


namespace fmt_star
{

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

} // namespace fmt_star

#endif //SRC_PLANNER_IMPL_H
