#ifndef _ASTAR_PLANNER_ASTAR_PLANNER_H_
#define _ASTAR_PLANNER_ASTAR_PLANNER_H_

#include <cstdio>
#include <vector>
#include <cmath>
#include <string.h>
#include "astar_planner/heap.h"
#include "astar_planner/utils.h"

#define SBPL_2DGRIDSEARCH_HEUR2D(x, y) ((int)(1000 * cellSize_m_ * __max(abs(x - goalX_), abs(y - goalY_))))
#define SBPL_2DGRIDSEARCH_NUMOF2DDIRS 8

namespace astar_planner
{
/**
 * \brief search state corresponding to each 2D cell
 */
class SBPL_2DGridSearchState : public AbstractSearchState
{
public:
  /**
   * \brief coordinates
   */
  int x, y;

  /**
   * \brief search relevant data
   */
  int g;
  /**
   * \brief iteration at which the state was accessed (generated) last
   */
  int iterationaccessed;

  SBPL_2DGridSearchState* predecessor;

public:
  SBPL_2DGridSearchState()
  {
    iterationaccessed = 0;
  }
  ~SBPL_2DGridSearchState()
  {
  }
};

/**
 * \brief 2D search itself
 */
class SBPL2DGridSearch
{
public:
  /**
   * @brief SBPL2DGridSearch Create a search space for 2D grids
   * @param width_x grid width
   * @param height_y grid height
   * @param cellsize_m resolution
   * @param downsample edge length of block in main grid that corresponds to a single cell in this grid
   * @param initial_dynamic_bucket_size Initial dynamic bucket size, set to 0 for fixed size
   */
  SBPL2DGridSearch(int width_x, int height_y, float cellsize_m);
  ~SBPL2DGridSearch()
  {
    destroy();
  }

  /**
   * \brief destroys the search and clears all memory
   */
  void destroy();

  /** \brief performs search itself. All costs are given as cost(cell1,
   *         cell2) = Euclidean distance*1000*(max(cell1,cell2)+1) for adjacent
   *         cells.
   * \note It is infinite if max(cell1,cell2) >= obsthresh For cells that are
   *       not adjacent (which may happen if 16 connected grid is ON), then max is
   *       taken over all the cells center line goes through Search is done from
   *       start to goal. termination_condition specifies when to stop the search.
   *       In particular, one may specify to run it until OPEN is empty. In this
   *       case, the values of all states will be computed.
   */
  bool search(unsigned char* Grid2D, unsigned char obsthresh, int startx_c, int starty_c, int goalx_c, int goaly_c,
              std::vector<std::pair<int, int> >& path);

  /**
   * \brief returns largest optimal g-value computed by search - a lower
   *        bound on the state values of unexpanded states
   */
  int getlargestcomputedoptimalf_inmm()
  {
    return largestcomputedoptf_;
  }

private:
  bool withinMap(int x, int y)
  {
    return (x >= 0 && y >= 0 && x < width_ && y < height_);
  }

  unsigned char getCost(unsigned char* Grid2D, int x, int y)
  {
    return Grid2D[x + y * width_];
  }

  void computedxy();
  void initializeSearchState2D(SBPL_2DGridSearchState* state2D);
  bool createSearchStates2D(void);

  bool search_withheap(unsigned char* Grid2D, unsigned char obsthresh, int startx_c, int starty_c, int goalx_c,
                       int goaly_c, std::vector<std::pair<int, int> >& path);

  // 2D search data
  CIntHeap* OPEN2D_;
  SBPL_2DGridSearchState** searchStates2D_;
  int dx_[SBPL_2DGRIDSEARCH_NUMOF2DDIRS];
  int dy_[SBPL_2DGRIDSEARCH_NUMOF2DDIRS];
  // distances of transitions
  int dxy_distance_mm_[SBPL_2DGRIDSEARCH_NUMOF2DDIRS];

  // start and goal configurations
  int startX_, startY_;
  int goalX_, goalY_;

  // map parameters
  int width_, height_;
  float cellSize_m_;

  // search iteration
  int iteration_;

  // largest optimal g-value computed by search
  int largestcomputedoptf_;
};

}  // namespace astar_planner

#endif  // _ASTAR_PLANNER_ASTAR_PLANNER_