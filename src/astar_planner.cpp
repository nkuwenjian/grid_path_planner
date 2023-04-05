/******************************************************************************
 * Copyright (c) 2023, NKU Mobile & Flying Robotics Lab
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 'AS IS'
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Jian Wen (nkuwenjian@gmail.com)
 *****************************************************************************/

#include "astar_planner_ros/astar_planner.h"

#include <algorithm>
#include <chrono>  // NOLINT

#include "glog/logging.h"

namespace astar_planner_ros {

GridSearch::GridSearch(const int max_grid_x, const int max_grid_y,
                       const double xy_grid_resolution)
    : max_grid_x_(max_grid_x),
      max_grid_y_(max_grid_y),
      xy_grid_resolution_(xy_grid_resolution) {
  open_list_ = std::make_unique<Heap>();
  closed_list_.resize(max_grid_x * max_grid_y, NodeStatus::OPEN);

  dp_lookup_table_.resize(max_grid_x_);
  for (int grid_x = 0; grid_x < max_grid_x; ++grid_x) {
    for (int grid_y = 0; grid_y < max_grid_y; ++grid_y) {
      dp_lookup_table_[grid_x].emplace_back(grid_x, grid_y);
    }
  }

  ComputeGridSearchActions();
}

GridSearch::~GridSearch() { open_list_->Clear(); }

bool GridSearch::GenerateGridPath(const int sx, const int sy, const int ex,
                                  const int ey, const uint8_t* grid_map,
                                  const uint8_t obsthresh,
                                  SearchType search_type,
                                  GridAStarResult* result) {
  if (grid_map == nullptr) {
    LOG(ERROR) << "Grid map is null pointer";
    return false;
  }
  const auto start_timestamp = std::chrono::system_clock::now();

  obsthresh_ = obsthresh;
  grid_map_ = grid_map;

  // check the validity of start/goal
  if (!IsValidCell(sx, sy)) {
    LOG(ERROR) << "GridSearch is called on invalid start (" << sx << "," << sy
               << ")";
    return false;
  }
  if (!IsValidCell(ex, ey)) {
    LOG(ERROR) << "GridSearch is called on invalid end (" << ex << "," << ey
               << ")";
    return false;
  }

  // clean up heap elements
  open_list_->Clear();
  // create start node
  Node2d* start_node = &dp_lookup_table_[sx][sy];
  start_node->set_g(0);
  if (search_type == SearchType::A_STAR) {
    start_node->set_h(CalcHeuCost(sx, sy, ex, ey));
  }
  // insert start node into heap
  open_list_->Insert(start_node, start_node->f());
  final_node_ = &dp_lookup_table_[ex][ey];

  int term_factor;
  if (search_type == SearchType::A_STAR) {
    term_factor = 1;
  } else if (search_type == SearchType::DP) {
    term_factor = 0;
  } else {
    LOG(WARNING) << "Unknown search type";
    term_factor = 0;
  }

  // grid search begins
  closed_list_.clear();
  closed_list_.resize(max_grid_x_ * max_grid_y_, NodeStatus::OPEN);
  size_t explored_node_num = 0;
  while (!open_list_->Empty() &&
         final_node_->g() > term_factor * open_list_->GetMinKey()) {
    auto* curr_node = dynamic_cast<Node2d*>(open_list_->Pop());
    CHECK_NOTNULL(curr_node);
    const int curr_x = curr_node->grid_x();
    const int curr_y = curr_node->grid_y();
    closed_list_[GetIndex(curr_x, curr_y)] = NodeStatus::CLOSED;
    ++explored_node_num;
    for (int action_id = 0; action_id < kNumOfGridSearchActions; ++action_id) {
      const int succ_x = curr_x + dx_[action_id];
      const int succ_y = curr_y + dy_[action_id];
      if (!IsValidCell(succ_x, succ_y)) {
        continue;
      }
      if (closed_list_[GetIndex(succ_x, succ_y)] == NodeStatus::CLOSED) {
        continue;
      }
      int action_cost = GetActionCost(curr_x, curr_y, action_id);
      if (action_cost == kInfiniteCost) {
        continue;
      }

      Node2d* succ_node = &dp_lookup_table_[succ_x][succ_y];
      if (succ_node->g() > curr_node->g() + action_cost) {
        succ_node->set_g(curr_node->g() + action_cost);
        if (search_type == SearchType::A_STAR && succ_node->h() == 0) {
          succ_node->set_h(CalcHeuCost(succ_x, succ_y, ex, ey));
        }
        succ_node->set_pre_node(curr_node);
        if (succ_node->heap_index() == 0) {
          open_list_->Insert(succ_node, succ_node->f());
        } else {
          open_list_->Update(succ_node, succ_node->f());
        }
      }
    }
  }

  const auto end_timestamp = std::chrono::system_clock::now();
  const std::chrono::duration<double> diff = end_timestamp - start_timestamp;
  VLOG(4) << "time used is " << diff.count() * 1e3 << " ms";
  VLOG(4) << "explored node num is " << explored_node_num;
  VLOG(4) << "2Dsolcost_inms = " << final_node_->g();
  VLOG(4) << "largestoptfval = " << open_list_->GetMinKey();
  VLOG(4) << "heap size = " << open_list_->Size();

  if (final_node_->g() == kInfiniteCost) {
    LOG(ERROR) << "Grid searching return infinite cost (open_list ran out)";
    return false;
  }
  if (search_type == SearchType::A_STAR) {
    LoadGridAStarResult(result);
  }
  return true;
}

bool GridSearch::IsWithinMap(const int x, const int y) const {
  return x >= 0 && x < max_grid_x_ && y >= 0 && y < max_grid_y_;
}

bool GridSearch::IsValidCell(const int x, const int y) const {
  if (!IsWithinMap(x, y)) {
    return false;
  }
  if (grid_map_[GetIndex(x, y)] >= obsthresh_) {
    return false;
  }
  return true;
}

int GridSearch::GetIndex(const int x, const int y) const {
  DCHECK(IsWithinMap(x, y));
  return x + y * max_grid_x_;
}

int GridSearch::GetActionCost(const int curr_x, const int curr_y,
                              const int action_id) const {
  CHECK(IsValidCell(curr_x, curr_y));

  const int succ_x = curr_x + dx_[action_id];
  const int succ_y = curr_y + dy_[action_id];
  CHECK(IsValidCell(succ_x, succ_y));

  uint8_t cost = std::max(grid_map_[GetIndex(curr_x, curr_y)],
                          grid_map_[GetIndex(succ_x, succ_y)]);
  if (kNumOfGridSearchActions > 8) {
    if (action_id > 7) {
      int x = curr_x + dx0intersects_[action_id];
      int y = curr_y + dy0intersects_[action_id];
      CHECK(IsWithinMap(x, y));
      cost = std::max(cost, grid_map_[GetIndex(x, y)]);
      x = curr_x + dx1intersects_[action_id];
      y = curr_y + dy1intersects_[action_id];
      CHECK(IsWithinMap(x, y));
      cost = std::max(cost, grid_map_[GetIndex(x, y)]);
    }
  }
  if (cost >= obsthresh_) {
    return kInfiniteCost;
  }
  return static_cast<int>(cost + 1) * dxy_distance_mm_[action_id];
}

void GridSearch::ComputeGridSearchActions() {
  // initialize some constants for 2D search
  // up
  dx_[0] = 0;
  dy_[0] = 1;
  dx0intersects_[0] = -1;
  dy0intersects_[0] = -1;
  // up right
  dx_[1] = 1;
  dy_[1] = 1;
  dx0intersects_[1] = -1;
  dy0intersects_[1] = -1;
  // right
  dx_[2] = 1;
  dy_[2] = 0;
  dx0intersects_[2] = -1;
  dy0intersects_[2] = -1;
  // down right
  dx_[3] = 1;
  dy_[3] = -1;
  dx0intersects_[3] = -1;
  dy0intersects_[3] = -1;
  // down
  dx_[4] = 0;
  dy_[4] = -1;
  dx0intersects_[4] = -1;
  dy0intersects_[4] = -1;
  // down left
  dx_[5] = -1;
  dy_[5] = -1;
  dx0intersects_[5] = -1;
  dy0intersects_[5] = -1;
  // left
  dx_[6] = -1;
  dy_[6] = 0;
  dx0intersects_[6] = -1;
  dy0intersects_[6] = -1;
  // up left
  dx_[7] = -1;
  dy_[7] = 1;
  dx0intersects_[7] = -1;
  dy0intersects_[7] = -1;

  // Note: these actions have to be starting at 8 and through 15, since they
  // get multiplied correspondingly in Dijkstra's search based on index
  if (kNumOfGridSearchActions == 16) {
    dx_[8] = 1;
    dy_[8] = 2;
    dx0intersects_[8] = 0;
    dy0intersects_[8] = 1;
    dx1intersects_[8] = 1;
    dy1intersects_[8] = 1;
    dx_[9] = 2;
    dy_[9] = 1;
    dx0intersects_[9] = 1;
    dy0intersects_[9] = 0;
    dx1intersects_[9] = 1;
    dy1intersects_[9] = 1;
    dx_[10] = 2;
    dy_[10] = -1;
    dx0intersects_[10] = 1;
    dy0intersects_[10] = 0;
    dx1intersects_[10] = 1;
    dy1intersects_[10] = -1;
    dx_[11] = 1;
    dy_[11] = -2;
    dx0intersects_[11] = 0;
    dy0intersects_[11] = -1;
    dx1intersects_[11] = 1;
    dy1intersects_[11] = -1;
    dx_[12] = -1;
    dy_[12] = -2;
    dx0intersects_[12] = 0;
    dy0intersects_[12] = -1;
    dx1intersects_[12] = -1;
    dy1intersects_[12] = -1;
    dx_[13] = -2;
    dy_[13] = -1;
    dx0intersects_[13] = -1;
    dy0intersects_[13] = 0;
    dx1intersects_[13] = -1;
    dy1intersects_[13] = -1;
    dx_[14] = -2;
    dy_[14] = 1;
    dx0intersects_[14] = -1;
    dy0intersects_[14] = 0;
    dx1intersects_[14] = -1;
    dy1intersects_[14] = 1;
    dx_[15] = -1;
    dy_[15] = 2;
    dx0intersects_[15] = 0;
    dy0intersects_[15] = 1;
    dx1intersects_[15] = -1;
    dy1intersects_[15] = 1;
  }

  // compute distances
  for (int dind = 0; dind < kNumOfGridSearchActions; dind++) {
    if (dx_[dind] != 0 && dy_[dind] != 0) {
      if (dind <= 7) {
        // the cost of a diagonal move in millimeters
        dxy_distance_mm_[dind] = static_cast<int>(xy_grid_resolution_ * 1414);
      } else {
        // the cost of a move to 1,2 or 2,1 or so on in millimeters
        dxy_distance_mm_[dind] = static_cast<int>(xy_grid_resolution_ * 2236);
      }
    } else {
      // the cost of a horizontal move in millimeters
      dxy_distance_mm_[dind] = static_cast<int>(xy_grid_resolution_ * 1000);
    }
  }
}

int GridSearch::CalcHeuCost(const int x1, const int y1, const int x2,
                            const int y2) const {
  return static_cast<int>(1000 * xy_grid_resolution_ *
                          std::max(abs(x1 - x2), abs(y1 - y2)));
}

void GridSearch::LoadGridAStarResult(GridAStarResult* result) const {
  if (result == nullptr) {
    return;
  }
  (*result).path_cost = final_node_->g();
  const Node2d* curr_node = final_node_;
  std::vector<int> grid_a_x;
  std::vector<int> grid_a_y;
  while (curr_node != nullptr) {
    grid_a_x.push_back(curr_node->grid_x());
    grid_a_y.push_back(curr_node->grid_y());
    curr_node = curr_node->pre_node();
  }
  std::reverse(grid_a_x.begin(), grid_a_x.end());
  std::reverse(grid_a_y.begin(), grid_a_y.end());
  (*result).x = std::move(grid_a_x);
  (*result).y = std::move(grid_a_y);
}

int GridSearch::CheckDpMap(const int grid_x, const int grid_y) {
  return dp_lookup_table_[grid_x][grid_y].g();
}

}  // namespace astar_planner_ros
