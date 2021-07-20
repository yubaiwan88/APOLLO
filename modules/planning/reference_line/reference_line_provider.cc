/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/**
 * @file
 * @brief Implementation of the class ReferenceLineProvider.
 */

#include "modules/planning/reference_line/reference_line_provider.h"

#include <algorithm>
#include <chrono>
#include <limits>
#include <utility>

#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/time/time.h"
#include "modules/common/util/file.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/map/pnc_map/path.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/planning_util.h"
#include "modules/routing/common/routing_gflags.h"

/**
 * ReferenceLineProvider类是规划模块中参考线（即候选路径）的提供和管理者，
 * 它以Routing模块输出的高精度路径（从起点到终点的Lane片段集）为输入数据，
 * 经优化处理后生成平滑的参考线集给后续的动作规划子模块使用。
 * ReferenceLineProvider类是采用C++ 11标准实现的多线程类，
 * 每次调用ReferenceLineProvider::Start()函数后，该类就会在内部开启一个新线程，执行参考线的优化。
 * ReferenceLineProvider类是使用宏DECLARE_SINGLETON(ReferenceLineProvider)定义的单实例类，
 * 获取该类对象请使用ReferenceLineProvider::instance()。
 * @namespace apollo::planning
 * @brief apollo::planning
 */
namespace apollo {
namespace planning {

using apollo::common::VehicleConfigHelper;
using apollo::common::VehicleState;
using apollo::common::adapter::AdapterManager;
using apollo::common::math::Vec2d;
using apollo::common::time::Clock;
using apollo::hdmap::HDMapUtil;
using apollo::hdmap::LaneWaypoint;
using apollo::hdmap::MapPathPoint;
using apollo::hdmap::RouteSegments;

ReferenceLineProvider::~ReferenceLineProvider() {
  if (thread_ && thread_->joinable()) {
    thread_->join();
  }
}
/*
创建hdmap::PncMap类对象，清空segment_history_，
若启用螺旋参考线（FLAGS_enable_spiral_reference_line），
则将参考线平滑类对象（smoother_）创建为SpiralReferenceLineSmoother类对象，
否则创建为QpSplineReferenceLineSmoother类对象。
将初始化标志is_initialized_置为true。
*/ 
ReferenceLineProvider::ReferenceLineProvider(const hdmap::HDMap *base_map) {
  if (!FLAGS_use_navigation_mode) {
    pnc_map_.reset(new hdmap::PncMap(base_map));
  }
  CHECK(common::util::GetProtoFromFile(FLAGS_smoother_config_filename,
                                       &smoother_config_))
      << "Failed to load smoother config file "
      << FLAGS_smoother_config_filename;
  if (smoother_config_.has_qp_spline()) {
    smoother_.reset(new QpSplineReferenceLineSmoother(smoother_config_));
  } else if (smoother_config_.has_spiral()) {
    smoother_.reset(new SpiralReferenceLineSmoother(smoother_config_));
  } else {
    CHECK(false) << "unknown smoother config "
                 << smoother_config_.DebugString();
  }
  is_initialized_ = true;
}  // namespace planning

bool ReferenceLineProvider::UpdateRoutingResponse(const routing::RoutingResponse &routing) {
  std::lock_guard<std::mutex> routing_lock(routing_mutex_);
  routing_ = routing;
  has_routing_ = true;
  return true;
}

std::vector<routing::LaneWaypoint>
ReferenceLineProvider::FutureRouteWaypoints() {
  std::lock_guard<std::mutex> lock(pnc_map_mutex_);
  return pnc_map_->FutureRouteWaypoints();
}

void ReferenceLineProvider::UpdateVehicleState(
    const VehicleState &vehicle_state) {
  std::lock_guard<std::mutex> lock(vehicle_state_mutex_);
  vehicle_state_ = vehicle_state;
}
/*
如果初始化标志is_initialized_置为false，直接返回false。若启用参考线提供者线程（FLAGS_enable_reference_line_provider_thread），
则以ReferenceLineProvider:: GenerateThread为入口函数创建一个新线程并返回true，否则直接返回true，不做任何实际工作。
*/ 
bool ReferenceLineProvider::Start() {
  if (FLAGS_use_navigation_mode) {
    return true;
  }
  if (!is_initialized_) {
    AERROR << "ReferenceLineProvider has NOT been initiated.";
    return false;
  }
  if (FLAGS_enable_reference_line_provider_thread) {
    thread_.reset(new std::thread(&ReferenceLineProvider::GenerateThread, this));
  }
  return true;
}

void ReferenceLineProvider::Stop() {
  is_stop_ = true;
  if (FLAGS_enable_reference_line_provider_thread && thread_ &&
      thread_->joinable()) {
    thread_->join();
  }
}
// 判断新参考线各线段是否与原参考线对应线段相同，若相同则忽略，否则更新
void ReferenceLineProvider::UpdateReferenceLine(
    const std::list<ReferenceLine> &reference_lines,
    const std::list<hdmap::RouteSegments> &route_segments) {
  if (reference_lines.size() != route_segments.size()) {
    AERROR << "The calculated reference line size(" << reference_lines.size()
           << ") and route_segments size(" << route_segments.size()
           << ") are different";
    return;
  }
  std::lock_guard<std::mutex> lock(reference_lines_mutex_);
  if (reference_lines_.size() != reference_lines.size()) {
    reference_lines_ = reference_lines;
    route_segments_ = route_segments;

  } else {
    auto segment_iter = route_segments.begin();
    auto internal_iter = reference_lines_.begin();
    auto internal_segment_iter = route_segments_.begin();
    for (auto iter = reference_lines.begin(); iter != reference_lines.end();
         ++iter, ++segment_iter, ++internal_iter, ++internal_segment_iter) {
      if (iter->reference_points().empty()) {
        *internal_iter = *iter;
        *internal_segment_iter = *segment_iter;
        continue;
      }
      if (common::util::SamePointXY(
              iter->reference_points().front(),
              internal_iter->reference_points().front()) &&
          common::util::SamePointXY(iter->reference_points().back(),
                                    internal_iter->reference_points().back()) &&
          std::fabs(iter->Length() - internal_iter->Length()) <
              common::math::kMathEpsilon) {
        continue;
      }
      *internal_iter = *iter;
      *internal_segment_iter = *segment_iter;
    }
  }
  // update history
  reference_line_history_.push(reference_lines_);
  route_segments_history_.push(route_segments_);
  constexpr int kMaxHistoryNum = 3;
  if (reference_line_history_.size() > kMaxHistoryNum) {
    reference_line_history_.pop();
    route_segments_history_.pop();
  }
}
/*

*/ 
void ReferenceLineProvider::GenerateThread() {
  constexpr int32_t kSleepTime = 50;  // milliseconds
  while (!is_stop_) {
    std::this_thread::yield();
    std::this_thread::sleep_for(
        std::chrono::duration<double, std::milli>(kSleepTime));
    double start_time = Clock::NowInSeconds();
    if (!has_routing_) {
      AERROR << "Routing is not ready.";
      continue;
    }
    std::list<ReferenceLine> reference_lines;
    std::list<hdmap::RouteSegments> segments;
    if (!CreateReferenceLine(&reference_lines, &segments)) {
      AERROR << "Fail to get reference line";
      continue;
    }
    UpdateReferenceLine(reference_lines, segments);
    double end_time = Clock::NowInSeconds();
    std::lock_guard<std::mutex> lock(reference_lines_mutex_);
    last_calculation_time_ = end_time - start_time;
  }
}

double ReferenceLineProvider::LastTimeDelay() {
  if (FLAGS_enable_reference_line_provider_thread &&
      !FLAGS_use_navigation_mode) {
    std::lock_guard<std::mutex> lock(reference_lines_mutex_);
    return last_calculation_time_;
  } else {
    return last_calculation_time_;
  }
}

bool ReferenceLineProvider::GetReferenceLines(
    std::list<ReferenceLine> *reference_lines,
    std::list<hdmap::RouteSegments> *segments) {
  CHECK_NOTNULL(reference_lines);
  CHECK_NOTNULL(segments);

  if (FLAGS_use_navigation_mode) {
    double start_time = Clock::NowInSeconds();
    bool result = GetReferenceLinesFromRelativeMap(
        AdapterManager::GetRelativeMap()->GetLatestObserved(), reference_lines,
        segments);
    if (!result) {
      AERROR << "Failed to get reference line from relative map";
    }
    double end_time = Clock::NowInSeconds();
    last_calculation_time_ = end_time - start_time;
    return result;
  }

  if (FLAGS_enable_reference_line_provider_thread) {
    std::lock_guard<std::mutex> lock(reference_lines_mutex_);
    if (!reference_lines_.empty()) {
      reference_lines->assign(reference_lines_.begin(), reference_lines_.end());
      segments->assign(route_segments_.begin(), route_segments_.end());
      return true;
    } else {
      AWARN << "Reference line is NOT ready.";
      if (reference_line_history_.empty()) {
        return false;
      }
      reference_lines->assign(reference_line_history_.back().begin(),
                              reference_line_history_.back().end());
      segments->assign(route_segments_history_.back().begin(),
                       route_segments_history_.back().end());
    }
  } else {
    double start_time = Clock::NowInSeconds();
    if (!CreateReferenceLine(reference_lines, segments)) {
      AERROR << "Failed to create reference line";
      return false;
    }
    UpdateReferenceLine(*reference_lines, *segments);
    double end_time = Clock::NowInSeconds();
    last_calculation_time_ = end_time - start_time;
  }
  return true;
}

void ReferenceLineProvider::PrioritzeChangeLane(
    std::list<hdmap::RouteSegments> *route_segments) {
  CHECK_NOTNULL(route_segments);
  auto iter = route_segments->begin();
  while (iter != route_segments->end()) {
    if (!iter->IsOnSegment()) {
      route_segments->splice(route_segments->begin(), *route_segments, iter);
      break;
    }
    ++iter;
  }
}

bool ReferenceLineProvider::GetReferenceLinesFromRelativeMap(
    const relative_map::MapMsg &relative_map,
    std::list<ReferenceLine> *reference_line,
    std::list<hdmap::RouteSegments> *segments) {
  DCHECK_GE(relative_map.navigation_path_size(), 0);
  DCHECK_NOTNULL(reference_line);
  DCHECK_NOTNULL(segments);

  if (relative_map.navigation_path().empty()) {
    AERROR << "There isn't any navigation path in current relative map.";
    return false;
  }

  auto *hdmap = HDMapUtil::BaseMapPtr();
  if (!hdmap) {
    AERROR << "hdmap is null";
    return false;
  }

  // 1.get adc current lane info ,such as lane_id,lane_priority,neighbor lanes
  std::unordered_set<std::string> navigation_lane_ids;
  for (const auto &path_pair : relative_map.navigation_path()) {
    const auto lane_id = path_pair.first;
    navigation_lane_ids.insert(lane_id);
  }
  if (navigation_lane_ids.empty()) {
    AERROR << "navigation path ids is empty";
    return false;
  }
  // get curent adc lane info by vehicle state
  common::VehicleState vehicle_state =
      common::VehicleStateProvider::instance()->vehicle_state();
  hdmap::LaneWaypoint adc_lane_way_point;
  if (!GetNearestWayPointFromNavigationPath(vehicle_state, navigation_lane_ids,
                                            &adc_lane_way_point)) {
    return false;
  }
  const std::string adc_lane_id = adc_lane_way_point.lane->id().id();
  auto adc_navigation_path = relative_map.navigation_path().find(adc_lane_id);
  if (adc_navigation_path == relative_map.navigation_path().end()) {
    AERROR << "adc lane cannot be found in relative_map.navigation_path";
    return false;
  }
  const uint32_t adc_lane_priority =
      adc_navigation_path->second.path_priority();
  // get adc left neighbor lanes
  std::vector<std::string> left_neighbor_lane_ids;
  auto left_lane_ptr = adc_lane_way_point.lane;
  while (left_lane_ptr != nullptr &&
         left_lane_ptr->lane().left_neighbor_forward_lane_id_size() > 0) {
    auto neighbor_lane_id =
        left_lane_ptr->lane().left_neighbor_forward_lane_id(0);
    left_neighbor_lane_ids.emplace_back(neighbor_lane_id.id());
    left_lane_ptr = hdmap->GetLaneById(neighbor_lane_id);
  }
  ADEBUG << adc_lane_id
         << " left neighbor size : " << left_neighbor_lane_ids.size();
  for (const auto &neighbor : left_neighbor_lane_ids) {
    ADEBUG << adc_lane_id << " left neighbor : " << neighbor;
  }
  // get adc right neighbor lanes
  std::vector<std::string> right_neighbor_lane_ids;
  auto right_lane_ptr = adc_lane_way_point.lane;
  while (right_lane_ptr != nullptr &&
         right_lane_ptr->lane().right_neighbor_forward_lane_id_size() > 0) {
    auto neighbor_lane_id =
        right_lane_ptr->lane().right_neighbor_forward_lane_id(0);
    right_neighbor_lane_ids.emplace_back(neighbor_lane_id.id());
    right_lane_ptr = hdmap->GetLaneById(neighbor_lane_id);
  }
  ADEBUG << adc_lane_id
         << " right neighbor size : " << right_neighbor_lane_ids.size();
  for (const auto &neighbor : right_neighbor_lane_ids) {
    ADEBUG << adc_lane_id << " right neighbor : " << neighbor;
  }
  // 2.get the higher priority lane info list which priority higher
  // than current lane and get the highest one as the target lane
  using LaneIdPair = std::pair<std::string, uint32_t>;
  std::vector<LaneIdPair> high_priority_lane_pairs;
  ADEBUG << "relative_map.navigation_path_size = "
         << relative_map.navigation_path_size();
  for (const auto &path_pair : relative_map.navigation_path()) {
    const auto lane_id = path_pair.first;
    const uint32_t priority = path_pair.second.path_priority();
    ADEBUG << "lane_id = " << lane_id << " priority = " << priority
           << " adc_lane_id = " << adc_lane_id
           << " adc_lane_priority = " << adc_lane_priority;
    // the smaller the number, the higher the priority
    if (adc_lane_id != lane_id && priority < adc_lane_priority) {
      high_priority_lane_pairs.emplace_back(lane_id, priority);
    }
  }
  // get the target lane
  bool is_lane_change_needed = false;
  LaneIdPair target_lane_pair;
  if (!high_priority_lane_pairs.empty()) {
    std::sort(high_priority_lane_pairs.begin(), high_priority_lane_pairs.end(),
              [](const LaneIdPair &left, const LaneIdPair &right) {
                return left.second < right.second;
              });
    ADEBUG << "need to change lane";
    // the higheast priority lane as the target naviagion lane
    target_lane_pair = high_priority_lane_pairs.front();
    is_lane_change_needed = true;
  }
  // 3.get current lane's the neareast neighbor lane to the target lane
  // and make sure it position is left or right on the current lane
  routing::ChangeLaneType lane_change_type = routing::FORWARD;
  std::string neareast_neighbor_lane_id;
  if (is_lane_change_needed) {
    // target on the left of adc
    if (left_neighbor_lane_ids.end() !=
        std::find(left_neighbor_lane_ids.begin(), left_neighbor_lane_ids.end(),
                  target_lane_pair.first)) {
      // take the id of the first adjacent lane on the left of adc as
      // the neareast_neighbor_lane_id
      neareast_neighbor_lane_id =
          adc_lane_way_point.lane->lane().left_neighbor_forward_lane_id(0).id();
    } else if (right_neighbor_lane_ids.end() !=
               std::find(right_neighbor_lane_ids.begin(),
                         right_neighbor_lane_ids.end(),
                         target_lane_pair.first)) {
      // target lane on the right of adc
      // take the id  of the first adjacent lane on the right of adc as
      // the neareast_neighbor_lane_id
      neareast_neighbor_lane_id = adc_lane_way_point.lane->lane()
                                      .right_neighbor_forward_lane_id(0)
                                      .id();
    }
  }

  for (const auto &path_pair : relative_map.navigation_path()) {
    const auto &lane_id = path_pair.first;
    const auto &path_points = path_pair.second.path().path_point();
    auto lane_ptr = hdmap->GetLaneById(hdmap::MakeMapId(lane_id));
    RouteSegments segment;
    segment.emplace_back(lane_ptr, 0.0, lane_ptr->total_length());
    segment.SetCanExit(true);
    segment.SetId(lane_id);
    segment.SetNextAction(routing::FORWARD);
    segment.SetStopForDestination(false);
    segment.SetPreviousAction(routing::FORWARD);

    if (is_lane_change_needed) {
      if (lane_id == neareast_neighbor_lane_id) {
        ADEBUG << "adc lane_id = " << adc_lane_id
               << " neareast_neighbor_lane_id = " << lane_id;
        segment.SetIsNeighborSegment(true);
        segment.SetPreviousAction(lane_change_type);
      } else if (lane_id == adc_lane_id) {
        segment.SetIsOnSegment(true);
        segment.SetNextAction(lane_change_type);
      }
    }

    segments->emplace_back(segment);
    std::vector<ReferencePoint> ref_points;
    for (const auto &path_point : path_points) {
      ref_points.emplace_back(
          MapPathPoint{Vec2d{path_point.x(), path_point.y()},
                       path_point.theta(),
                       LaneWaypoint(lane_ptr, path_point.s())},
          path_point.kappa(), path_point.dkappa());
    }
    reference_line->emplace_back(ref_points.begin(), ref_points.end());
  }
  return !segments->empty();
}
Startcle_state,
    const double look_backward_distance, const double look_forward_distance,
    std::list<hdmap::RouteSegments> *segments) {
  {
    std::lock_guard<std::mutex> lock(pnc_map_mutex_);
    if (!pnc_map_->GetRouteSegments(vehicle_state, look_backward_distance,
                                    look_forward_distance, segments)) {
      AERROR << "Failed to extract segments from routing";
      return false;
    }
  }

  if (FLAGS_prioritize_change_lane) {
    PrioritzeChangeLane(segments);
  }
  return !segments->empty();
}

double ReferenceLineProvider::LookForwardDistance(const VehicleState &state) {
  auto forward_distance = state.linear_velocity() * FLAGS_look_forward_time_sec;

  if (forward_distance > FLAGS_look_forward_short_distance) {
    return FLAGS_look_forward_long_distance;
  }

  return FLAGS_look_forward_short_distance;
}
/*
首先使用互斥体锁定，分别获取当前车辆状态和路由，接着调用pnc_map_->IsNewRouting(routing)判断当前路由是否为新路由，
若是则调用pnc_map_->UpdateRoutingResponse(routing)更新路由。之后，调用CreateRouteSegments函数来创建路由片段。

若不需粘合参考线（!FLAGS_enable_reference_line_stitching），则调用SmoothRouteSegment函数来平滑各路由片段列表segments，
并将平滑后的路由片段存储到参考线列表reference_lines中，同时将不能平滑的路由片段从segments中删除；

若需粘合参考线（FLAGS_enable_reference_line_stitching），则调用ExtendReferenceLine函数来合并不同参考线片段的重合部分，
并将粘合后的路由片段保存到参考线列表reference_lines中，同时将不能粘合的路由片段从列表segments中删除。
*/ 
// 将平滑后的参考线路由片段保存在参考线列表reference_lines
// 平滑参考线的拼接

/*
平滑参考线拼接是针对不同时刻的RawReference，如果两条原始的RawReference是相连并且有覆盖的，那么可以不需要重新去进行平滑，
只要直接使用上时刻的平滑参考线，或者仅仅平滑部分anchor point即可。

例如上时刻得到的平滑参考线reference_prev，这时刻由RouteSegments得到的原始费平滑参考线reference_current。
由于RouteSegments生成有一个look_forward_distance的前向查询距离，
所以这时候车辆的位置很可能还在前一时刻的平滑参考线reference_prev，这时候就可以复用上时刻的参考线信息，
*/ 
bool ReferenceLineProvider::CreateReferenceLine(
    std::list<ReferenceLine> *reference_lines,
    std::list<hdmap::RouteSegments> *segments) {
  CHECK_NOTNULL(reference_lines);
  CHECK_NOTNULL(segments);

  common::VehicleState vehicle_state;
  {
    std::lock_guard<std::mutex> lock(vehicle_state_mutex_);
    vehicle_state = vehicle_state_;
  }

  routing::RoutingResponse routing;
  {
    std::lock_guard<std::mutex> lock(routing_mutex_);
    routing = routing_;
  }
  bool is_new_routing = false;
  {
    // Update routing in pnc_map 判断当前路由是否为新路由
    if (pnc_map_->IsNewRouting(routing)) {
      is_new_routing = true;
      // 更新路由
      if (!pnc_map_->UpdateRoutingResponse(routing)) {
        AERROR << "Failed to update routing in pnc map";
        return false;
      }
    }
  }
  // 前向查询距离
  double look_forward_distance = LookForwardDistance(vehicle_state);
  double look_backward_distance = FLAGS_look_backward_distance;
  // 创建路由片段
  if (!CreateRouteSegments(vehicle_state, look_backward_distance, look_forward_distance, segments)) {
    AERROR << "Failed to create reference line from routing";
    return false;
  }
   // A. 参考线平滑，条件enable_reference_line_stitching设置为False，也就是不允许参考线拼接操作
  if (is_new_routing || !FLAGS_enable_reference_line_stitching) {
    for (auto iter = segments->begin(); iter != segments->end();) {
      reference_lines->emplace_back();
      if (!SmoothRouteSegment(*iter, &reference_lines->back())) {
        AERROR << "Failed to create reference line from route segments";
        reference_lines->pop_back();
        iter = segments->erase(iter);
      } else {
        ++iter;
      }
    }
    return true;
  } else {  // stitching reference line  允许参考线拼接
    for (auto iter = segments->begin(); iter != segments->end();) {
      reference_lines->emplace_back();
      if (!ExtendReferenceLine(vehicle_state, &(*iter),&reference_lines->back())) {
        AERROR << "Failed to extend reference line";
        reference_lines->pop_back();
        iter = segments->erase(iter);
      } else {
        ++iter;
      }
    }
  }
  return true;
}
// 参考线扩展/拼接函数
bool ReferenceLineProvider::ExtendReferenceLine(const VehicleState &state,
                                                RouteSegments *segments,
                                                ReferenceLine *reference_line) {
  // case A:根据历史缓存信息，查询当前RouteSegments是否在某条(Smoothed)ReferenceLine上，如果不是就直接进行平滑参考线操作
  RouteSegments segment_properties;
  segment_properties.SetProperties(*segments);
  auto prev_segment = route_segments_.begin();
  auto prev_ref = reference_lines_.begin();
  while (prev_segment != route_segments_.end()) {
    // 查询路径段prev_segment是否连接到segments整个路径上的函数IsConnectedSegment
    if (prev_segment->IsConnectedSegment(*segments)) {
      break;
    }
    ++prev_segment;
    ++prev_ref;
  }
  if (prev_segment == route_segments_.end()) {
    if (!route_segments_.empty() && segments->IsOnSegment()) {
      AWARN << "Current route segment is not connected with previous route "
               "segment";
    }
    return SmoothRouteSegment(*segments, reference_line);
  }
  
  // case B:如果在同一个平滑参考线(历史平滑参考线)上，计算车辆当前位置和历史平滑参考线终点的距离，
  // 如果距离超过了阈值，则可以复用这条历史参考线；否则长度不够需要拼接。
  common::SLPoint sl_point;
  Vec2d vec2d(state.x(), state.y());
  LaneWaypoint waypoint;
  if (!prev_segment->GetProjection(vec2d, &sl_point, &waypoint)) { //计算车辆当前位置在历史平滑参考线上的位置
    AWARN << "Vehicle current point: " << vec2d.DebugString()
          << " not on previous reference line";
    return SmoothRouteSegment(*segments, reference_line);
  }
  const double prev_segment_length = RouteSegments::Length(*prev_segment); //历史平滑参考线的总长度
  const double remain_s = prev_segment_length - sl_point.s();                                        // 历史平滑参考线前方剩余距离
  const double look_forward_required_distance = LookForwardDistance(state);     // 前向查询距离
  if (remain_s > look_forward_required_distance) {                                                               // 如果剩余的距离足够长，直接复用这条历史平滑参考线
    *segments = *prev_segment;
    segments->SetProperties(segment_properties);
    *reference_line = *prev_ref;
    ADEBUG << "Reference line remain " << remain_s
           << ", which is more than required " << look_forward_required_distance
           << " and no need to extend";
    return true;
  }
  // case C:如果2种情况历史参考线遗留长度不够，那么就需要先对RouteSegments进行扩展，这部分在Pnc Map后接车道处理中有相关介绍。
  // 如果扩展失败直接进行平滑操作；如果扩展以后长度仍然不够，说明死路没有后继车道，只能复用历史平滑参考线。
  double future_start_s =
      std::max(sl_point.s(), prev_segment_length -
                                 FLAGS_reference_line_stitch_overlap_distance);
  double future_end_s =
      prev_segment_length + FLAGS_look_forward_extend_distance; // 向后额外扩展look_forward_extend_distance的距离，默认50m
  RouteSegments shifted_segments;
  std::unique_lock<std::mutex> lock(pnc_map_mutex_);
  if (!pnc_map_->ExtendSegments(*prev_segment, future_start_s, future_end_s,
                                &shifted_segments)) {
    lock.unlock();
    AERROR << "Failed to shift route segments forward";
    return SmoothRouteSegment(*segments, reference_line);  // C.1 扩展操作失败，直接对新的RouteSegments进行平滑得到平滑参考线
  }
  lock.unlock();
  if (prev_segment->IsWaypointOnSegment(shifted_segments.LastWaypoint())) {
    *segments = *prev_segment;             // C.2 扩展操作成功，但是扩展以后长度没有太大变化，死路，直接使用历史平滑参考线
    segments->SetProperties(segment_properties);
    *reference_line = *prev_ref;
    ADEBUG << "Could not further extend reference line";
    return true;
  }
// case D:如果3情况下扩展成功，并且额外增加了一定长度，得到了新的Path(也即新的RouteSegments)，
// 接下来对新的路径进行平滑然后与历史平滑参考线进行拼接，就可以得到一条更长的平滑参考线。
  hdmap::Path path;
  hdmap::PncMap::CreatePathFromLaneSegments(shifted_segments, &path);
  ReferenceLine new_ref(path);
  if (!SmoothPrefixedReferenceLine(*prev_ref, new_ref, reference_line)) { // SmoothPrefixedReferenceLine过程和普通的拼接其实没多大差异
    AWARN << "Failed to smooth forward shifted reference line";
    return SmoothRouteSegment(*segments, reference_line);
  }
  if (!reference_line->Stitch(*prev_ref)) {                                 // 两条平滑车道线拼接
    AWARN << "Failed to stitch reference line";
    return SmoothRouteSegment(*segments, reference_line);
  }
  if (!shifted_segments.Stitch(*prev_segment)) {                 // 两条平滑车道线对应的RouteSegments拼接
    AWARN << "Failed to stitch route segments";
    return SmoothRouteSegment(*segments, reference_line);
  }
  // Case E:当在4完成参考线的拼接以后，就可以得到一条更长的参考线，前向查询距离经过限制不会超出要求，
  // 但是随着车辆的前进，车后参考线的长度会变得很大，所以最后一步就是对车后的参考线进行收缩，保证车辆前后都有合理长度的参考线
  *segments = shifted_segments;
  segments->SetProperties(segment_properties);
  common::SLPoint sl;
  if (!reference_line->XYToSL(vec2d, &sl)) {
    AWARN << "Failed to project point: " << vec2d.DebugString()
          << " to stitched reference line";
  }
  if (sl.s() > FLAGS_look_backward_distance * 1.5) { //如果车后参考线的距离大于后向查询距离1.5倍，就需要收缩
    ADEBUG << "reference line back side is " << sl.s()
           << ", shrink reference line: origin lenght: "
           << reference_line->Length();
    if (!reference_line->Shrink(vec2d, FLAGS_look_backward_distance, // E.1 拼接参考线收缩
                                std::numeric_limits<double>::infinity())) {
      AWARN << "Failed to shrink reference line";
    }
    if (!segments->Shrink(vec2d, FLAGS_look_backward_distance,        // E.2 对应的RouteSegments收缩
                          std::numeric_limits<double>::infinity())) {
      AWARN << "Failed to shrink route segment";
    }
  }
  return true;
}

bool ReferenceLineProvider::IsReferenceLineSmoothValid(
    const ReferenceLine &raw, const ReferenceLine &smoothed) const {
  constexpr double kReferenceLineDiffCheckStep = 10.0;
  for (double s = 0.0; s < smoothed.Length();
       s += kReferenceLineDiffCheckStep) {
    auto xy_new = smoothed.GetReferencePoint(s);

    common::SLPoint sl_new;
    if (!raw.XYToSL(xy_new, &sl_new)) {
      AERROR << "Fail to change xy point on smoothed reference line to sl "
                "point respect to raw reference line.";
      return false;
    }

    const double diff = std::fabs(sl_new.l());
    if (diff > FLAGS_smoothed_reference_line_max_diff) {
      AERROR << "Fail to provide reference line because too large diff "
                "between smoothed and raw reference lines. diff: "
             << diff;
      return false;
    }
  }
  return true;
}

// 函数是如何完成采样点的坐标计算与轨迹点坐标纠正的
AnchorPoint ReferenceLineProvider::GetAnchorPoint(
    const ReferenceLine &reference_line, double s) const {
  AnchorPoint anchor;
  // longitudinal_bound是预测的y值需要在ref_point的F轴的longitudinal_bound前后领域内
  anchor.longitudinal_bound = smoother_config_.longitudinal_boundary_bound();
// 采样点坐标计算
  auto ref_point = reference_line.GetReferencePoint(s);
  if (ref_point.lane_waypoints().empty()) {
    anchor.path_point = ref_point.ToPathPoint(s);
    anchor.lateral_bound = smoother_config_.lateral_boundary_bound();
    return anchor;
  }
  // 计算车辆宽度和半宽
  const double adc_width = VehicleConfigHelper::GetConfig().vehicle_param().width();
  const double adc_half_width = adc_width / 2.0;
  const Vec2d left_vec = Vec2d::CreateUnitVec2d(ref_point.heading() + M_PI / 2.0);
  auto waypoint = ref_point.lane_waypoints().front();
  // shift to center 计算车道距左边界距离left_width和距右边界距离right_width
  double left_width = 0.0;
  double right_width = 0.0;
  waypoint.lane->GetWidth(waypoint.s, &left_width, &right_width);
  double total_width = left_width + right_width; // 当前位置，车道总宽度
  // only need to track left side width shift
  double shifted_left_width = total_width / 2.0;

  // 计算车辆应该右移或者左移（纠正）的距离
  // shift to left (or right) on wide lanes
  if (total_width > adc_width * smoother_config_.wide_lane_threshold_factor()) {
    // 靠右行驶模式
    if (smoother_config_.driving_side() == ReferenceLineSmootherConfig::RIGHT) {
      shifted_left_width =adc_half_width +
          adc_width * smoother_config_.wide_lane_shift_remain_factor();
    } else {
      // 靠左行驶模式
      shifted_left_width = std::fmax(adc_half_width,total_width -(adc_half_width +
               adc_width * smoother_config_.wide_lane_shift_remain_factor()));
    }
  }
// 第二部分纠正：根据左右边界线是否是道路边界
  // shift away from curb boundary
  auto left_type = hdmap::LeftBoundaryType(waypoint);
  if (left_type == hdmap::LaneBoundaryType::CURB) {
    shifted_left_width += smoother_config_.curb_shift();
  }
  auto right_type = hdmap::RightBoundaryType(waypoint);
  if (right_type == hdmap::LaneBoundaryType::CURB) {
    shifted_left_width -= smoother_config_.curb_shift();
  }

  ref_point += left_vec * (left_width - shifted_left_width);
  auto shifted_right_width = total_width - shifted_left_width;
  anchor.path_point = ref_point.ToPathPoint(s);
  double effective_width = std::min(shifted_left_width, shifted_right_width) -
                           adc_half_width - FLAGS_reference_line_lateral_buffer;
  anchor.lateral_bound =
      std::max(smoother_config_.lateral_boundary_bound(), effective_width);
  return anchor;
}

// 路径点采样与轨迹纠正阶段工作就是对路径做一个离散点采样，重新采样
// 首先，已知路径的长度length_，只需要给出采样的间距interval，就能完成采样。
void ReferenceLineProvider::GetAnchorPoints(
    const ReferenceLine &reference_line,
    std::vector<AnchorPoint> *anchor_points) const {
  CHECK_NOTNULL(anchor_points);
  // interval为采样间隔，默认max_constraint_interval = 5.0,即路径累计距离每5米采样一个点
  const double interval = smoother_config_.max_constraint_interval();
  // 路径采样点数量的计算
  int num_of_anchors = std::max(2, static_cast<int>(reference_line.Length() / interval + 0.5));
  std::vector<double> anchor_s;
   // uniform_slice函数就是对[0.0, reference_line.Length()]区间等间隔采样，每两个点之间距离为(length_-0.0)/(num_of_anchors - 1)
  common::util::uniform_slice(0.0, reference_line.Length(), num_of_anchors - 1, &anchor_s);
  // 根据每个采样点的累积距离s，以及Path的lane_segments_to_next_point_进行平滑插值，
  // 得到累积距离为s的采样点的坐标(x,y)，并进行轨迹点矫正
  for (const double s : anchor_s) {
    // GetAnchorPoint函数
    anchor_points->emplace_back(GetAnchorPoint(reference_line, s));
  }
  anchor_points->front().longitudinal_bound = 1e-6;
  anchor_points->front().lateral_bound = 1e-6;
  anchor_points->front().enforced = true;
  anchor_points->back().longitudinal_bound = 1e-6;
  anchor_points->back().lateral_bound = 1e-6;
  anchor_points->back().enforced = true;
}

bool ReferenceLineProvider::SmoothRouteSegment(const RouteSegments &segments,
                                               ReferenceLine *reference_line) {
  hdmap::Path path;
  hdmap::PncMap::CreatePathFromLaneSegments(segments, &path);
  return SmoothReferenceLine(ReferenceLine(path), reference_line);
}

bool ReferenceLineProvider::SmoothPrefixedReferenceLine(
    const ReferenceLine &prefix_ref, const ReferenceLine &raw_ref,
    ReferenceLine *reference_line) {
  if (!FLAGS_enable_smooth_reference_line) {
    *reference_line = raw_ref;
    return true;
  }
  // generate anchor points:
  std::vector<AnchorPoint> anchor_points;
  GetAnchorPoints(raw_ref, &anchor_points);
  // modify anchor points based on prefix_ref
  for (auto &point : anchor_points) {
    common::SLPoint sl_point;
    Vec2d xy{point.path_point.x(), point.path_point.y()};
    if (!prefix_ref.XYToSL(xy, &sl_point)) {
      AERROR << "Failed to get projection for point: " << xy.DebugString();
      return false;
    }
    if (sl_point.s() < 0 || sl_point.s() > prefix_ref.Length()) {
      continue;
    }
    auto prefix_ref_point = prefix_ref.GetNearestReferencePoint(sl_point.s());
    point.path_point.set_x(prefix_ref_point.x());
    point.path_point.set_y(prefix_ref_point.y());
    point.path_point.set_z(0.0);
    point.path_point.set_theta(prefix_ref_point.heading());
    point.longitudinal_bound = 1e-6;
    point.lateral_bound = 1e-6;
    point.enforced = true;
    break;
  }

  smoother_->SetAnchorPoints(anchor_points);
  if (!smoother_->Smooth(raw_ref, reference_line)) {
    AERROR << "Failed to smooth prefixed reference line with anchor points";
    return false;
  }
  if (!IsReferenceLineSmoothValid(raw_ref, *reference_line)) {
    AERROR << "The smoothed reference line error is too large";
    return false;
  }
  return true;
}

bool ReferenceLineProvider::SmoothReferenceLine(
    const ReferenceLine &raw_reference_line, ReferenceLine *reference_line) {
  if (!FLAGS_enable_smooth_reference_line) {
    *reference_line = raw_reference_line;
    return true;
  }
  // generate anchor points:
  std::vector<AnchorPoint> anchor_points;
  GetAnchorPoints(raw_reference_line, &anchor_points);
  smoother_->SetAnchorPoints(anchor_points);
  if (!smoother_->Smooth(raw_reference_line, reference_line)) {
    AERROR << "Failed to smooth reference line with anchor points";
    return false;
  }
  if (!IsReferenceLineSmoothValid(raw_reference_line, *reference_line)) {
    AERROR << "The smoothed reference line error is too large";
    return false;
  }
  return true;
}
}  // namespace planning
}  // namespace apollo
