#include <orunav_constraint_extract/grid_map.h>

using namespace std;
using namespace constraint_extract;

void GridMap::assignMsg(const nav_msgs::OccupancyGridConstPtr &msg)
{
    map = *msg;
}

void
GridMap::printDebug() const
{
}
