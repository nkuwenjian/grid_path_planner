#include <algorithm>
#include <chrono>
#include "astar_planner/astar_planner.h"

namespace astar_planner
{
//---------------------initialization and destruction routines--------------------------------------------------------
SBPL2DGridSearch::SBPL2DGridSearch(int width_x, int height_y, float cellsize_m)
{
  iteration_ = 0;
  searchStates2D_ = NULL;

  width_ = width_x;
  height_ = height_y;
  cellSize_m_ = cellsize_m;

  startX_ = -1;
  startY_ = -1;
  goalX_ = -1;
  goalY_ = -1;

  largestcomputedoptf_ = 0;

  // compute dx, dy, dxintersects and dyintersects arrays
  computedxy();

  // allocate memory
  OPEN2D_ = new CIntHeap(width_x * height_y);
  if (!createSearchStates2D())
  {
    throw SBPL_Exception("ERROR: failed to create searchstatespace2D");
  }
}

bool SBPL2DGridSearch::createSearchStates2D(void)
{
  int x, y;

  if (searchStates2D_ != NULL)
  {
    printf("ERROR: We already have a non-NULL search states array\n");
    return false;
  }

  searchStates2D_ = new SBPL_2DGridSearchState*[width_];
  for (x = 0; x < width_; x++)
  {
    searchStates2D_[x] = new SBPL_2DGridSearchState[height_];
    for (y = 0; y < height_; y++)
    {
      searchStates2D_[x][y].iterationaccessed = iteration_;
      searchStates2D_[x][y].x = x;
      searchStates2D_[x][y].y = y;
      initializeSearchState2D(&searchStates2D_[x][y]);
    }
  }
  return true;
}

inline void SBPL2DGridSearch::initializeSearchState2D(SBPL_2DGridSearchState* state2D)
{
  state2D->g = INFINITECOST;
  state2D->heapindex = 0;
  state2D->iterationaccessed = iteration_;
  state2D->predecessor = NULL;
}

void SBPL2DGridSearch::destroy()
{
  // destroy the OPEN list:
  if (OPEN2D_ != NULL)
  {
    OPEN2D_->makeemptyheap();
    delete OPEN2D_;
    OPEN2D_ = NULL;
  }

  // destroy the 2D states:
  if (searchStates2D_ != NULL)
  {
    for (int x = 0; x < width_; x++)
    {
      delete[] searchStates2D_[x];
    }
    delete[] searchStates2D_;
    searchStates2D_ = NULL;
  }
}

void SBPL2DGridSearch::computedxy()
{
  // initialize some constants for 2D search
  dx_[0] = 1;
  dy_[0] = 1;
  dx_[1] = 1;
  dy_[1] = 0;
  dx_[2] = 1;
  dy_[2] = -1;
  dx_[3] = 0;
  dy_[3] = 1;
  dx_[4] = 0;
  dy_[4] = -1;
  dx_[5] = -1;
  dy_[5] = 1;
  dx_[6] = -1;
  dy_[6] = 0;
  dx_[7] = -1;
  dy_[7] = -1;

  // compute distances
  for (int dind = 0; dind < SBPL_2DGRIDSEARCH_NUMOF2DDIRS; dind++)
  {
    if (dx_[dind] != 0 && dy_[dind] != 0)
      dxy_distance_mm_[dind] = (int)(cellSize_m_ * 1414);
    else
      dxy_distance_mm_[dind] = (int)(cellSize_m_ * 1000);  // the cost of a horizontal move in millimeters
  }
}

//--------------------------------------------------------------------------------------------------------------------

//-----------------------------------------main functions--------------------------------------------------------------

bool SBPL2DGridSearch::search(unsigned char* Grid2D, unsigned char obsthresh, int startx_c, int starty_c, int goalx_c,
                              int goaly_c, std::vector<std::pair<int, int> >& path)
{
  return SBPL2DGridSearch::search_withheap(Grid2D, obsthresh, startx_c, starty_c, goalx_c, goaly_c, path);
}

bool SBPL2DGridSearch::search_withheap(unsigned char* Grid2D, unsigned char obsthresh, int startx_c, int starty_c,
                                       int goalx_c, int goaly_c, std::vector<std::pair<int, int> >& path)
{
  SBPL_2DGridSearchState* searchExpState = NULL;
  SBPL_2DGridSearchState* searchPredState = NULL;
  int numofExpands = 0;
  int key;

  // get the current time
  const auto start_t = std::chrono::system_clock::now();

  // closed = 0
  iteration_++;

  // init start and goal coordinates
  startX_ = startx_c;
  startY_ = starty_c;
  goalX_ = goalx_c;
  goalY_ = goaly_c;

  // clear the heap
  OPEN2D_->makeemptyheap();

  // check the validity of start/goal
  if (!withinMap(startx_c, starty_c) || !withinMap(goalx_c, goaly_c))
  {
    printf("ERROR: grid2Dsearch is called on invalid start (%d %d) or goal(%d %d)\n", startx_c, starty_c, goalx_c,
           goaly_c);
    return false;
  }

  // initialize the start and goal states
  searchExpState = &searchStates2D_[startX_][startY_];
  initializeSearchState2D(searchExpState);
  initializeSearchState2D(&searchStates2D_[goalx_c][goaly_c]);
  SBPL_2DGridSearchState* search2DGoalState = &searchStates2D_[goalx_c][goaly_c];

  // seed the search
  searchExpState->g = 0;
  key = searchExpState->g + SBPL_2DGRIDSEARCH_HEUR2D(startX_, startY_);

  OPEN2D_->insertheap(searchExpState, key);

  char* pbClosed = (char*)calloc(1, width_ * height_);

  // the main repetition of expansions
  while (!OPEN2D_->emptyheap() && __min(INFINITECOST, search2DGoalState->g) > OPEN2D_->getminkeyheap())
  {
    // get the next state for expansion
    searchExpState = dynamic_cast<SBPL_2DGridSearchState*>(OPEN2D_->deleteminheap());
    numofExpands++;

    int exp_x = searchExpState->x;
    int exp_y = searchExpState->y;

    // close the state
    pbClosed[exp_x + width_ * exp_y] = 1;

    // iterate over successors
    int expcost = getCost(Grid2D, exp_x, exp_y);
    for (int dir = 0; dir < SBPL_2DGRIDSEARCH_NUMOF2DDIRS; dir++)
    {
      int newx = exp_x + dx_[dir];
      int newy = exp_y + dy_[dir];

      // make sure it is inside the map and has no obstacle
      if (!withinMap(newx, newy))
        continue;

      if (pbClosed[newx + width_ * newy] == 1)
        continue;

      // compute the cost
      int mapcost = __max(getCost(Grid2D, newx, newy), expcost);

      if (mapcost >= obsthresh)  // obstacle encountered
        continue;
      // int cost = (mapcost + 1) * dxy_distance_mm_[dir];
      int cost = dxy_distance_mm_[dir];

      // get the predecessor
      searchPredState = &searchStates2D_[newx][newy];

      // update predecessor if necessary
      if (searchPredState->iterationaccessed != iteration_ || searchPredState->g > cost + searchExpState->g)
      {
        searchPredState->iterationaccessed = iteration_;
        searchPredState->g = __min(INFINITECOST, cost + searchExpState->g);
        key = searchPredState->g + SBPL_2DGRIDSEARCH_HEUR2D(searchPredState->x, searchPredState->y);

        if (searchPredState->heapindex == 0)
          OPEN2D_->insertheap(searchPredState, key);
        else
          OPEN2D_->updateheap(searchPredState, key);

        searchPredState->predecessor = searchExpState;
      }
    }  // over successors
  }    // while

  // set lower bounds for the remaining states
  if (!OPEN2D_->emptyheap())
    largestcomputedoptf_ = OPEN2D_->getminkeyheap();
  else
    largestcomputedoptf_ = INFINITECOST;

  free(pbClosed);

  const auto end_t = std::chrono::system_clock::now();
  const std::chrono::duration<double> timediff = end_t - start_t;

  printf(
      "# of expands during 2dgridsearch=%d time=%.2f msecs 2Dsolcost_inmm=%d "
      "largestoptfval=%d (start=%d %d goal=%d %d)\n",
      numofExpands, timediff.count() * 1000, searchStates2D_[goalx_c][goaly_c].g, largestcomputedoptf_, startx_c,
      starty_c, goalx_c, goaly_c);

  path.clear();
  searchPredState = search2DGoalState;
  path.push_back(std::make_pair(searchPredState->x, searchPredState->y));
  while (searchPredState->predecessor != NULL)
  {
    searchPredState = searchPredState->predecessor;
    path.push_back(std::make_pair(searchPredState->x, searchPredState->y));
  }

  std::reverse(path.begin(), path.end());

  return true;
}

}  // namespace astar_planner