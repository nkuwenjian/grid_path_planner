#ifndef _ASTAR_PLANNER_UTILS_H_
#define _ASTAR_PLANNER_UTILS_H_

#include <stdexcept>
#include <string>

#ifndef WIN32
#define __max(x, y) (x > y ? x : y)
#define __min(x, y) (x > y ? y : x)
#endif

#define CONTXY2DISC(X, CELLSIZE) (((X) >= 0) ? ((int)((X) / (CELLSIZE))) : ((int)((X) / (CELLSIZE)) - 1))
#define DISCXY2CONT(X, CELLSIZE) ((X) * (CELLSIZE) + (CELLSIZE) / 2.0)
#define INFINITECOST 1000000000

#endif  // _ASTAR_PLANNER_UTILS_H_