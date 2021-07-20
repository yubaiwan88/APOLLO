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
 * @file dp_road_graph.h
 **/

#include "modules/planning/tasks/dp_poly_path/dp_road_graph.h"

#include <algorithm>
#include <utility>

#include "modules/common/proto/error_code.pb.h"
#include "modules/common/proto/pnc_point.pb.h"
#include "modules/planning/proto/planning_internal.pb.h"
#include "modules/planning/proto/planning_status.pb.h"

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/log.h"
#include "modules/common/math/cartesian_frenet_conversion.h"
#include "modules/common/util/util.h"
#include "modules/map/hdmap/hdmap_util.h"

#include "modules/planning/common/path/frenet_frame_path.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/planning_thread_pool.h"
#include "modules/planning/common/planning_util.h"
#include "modules/planning/math/curve1d/quintic_polynomial_curve1d.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::SLPoint;
using apollo::common::math::CartesianFrenetConverter;
using apollo::common::util::MakeSLPoint;

DPRoadGraph::DPRoadGraph(const DpPolyPathConfig &config,
                         const ReferenceLineInfo &reference_line_info,
                         const SpeedData &speed_data)
    : config_(config),
      reference_line_info_(reference_line_info),
      reference_line_(reference_line_info.reference_line()),
      speed_data_(speed_data) {}

//FindPathTunnel()的结果是依据若干level之间分段5次多项式的采样点，
//保存在path_data.frenet_path_（SL系）和path_data.discretized_path_（XY系）中

// FindPathTunnel()主要分为3部分：1、先设置相关前提条件，2、然后查找代价最小路径，
//3、最后对每段代价最小路径采样以构造FrenetFramePath类的实例，并存入path_data中。

/*
Step A. 计算起始点的累计距离s，侧方相对偏移l，侧向速度dl和侧向速度ddl

Step B. 获取当前参考线下最优的前进路线

Step C. 将最优前进路线封装成path_data
*/ 
bool DPRoadGraph::FindPathTunnel(const common::TrajectoryPoint &init_point,
    const std::vector<const PathObstacle *> &obstacles,
    PathData *const path_data) {

  CHECK_NOTNULL(path_data);

  init_point_ = init_point;
  // 将自车当前位置转换为sl上坐标
  if (!reference_line_.XYToSL({init_point_.path_point().x(), init_point_.path_point().y()}, &init_sl_point_)) {
    AERROR << "Fail to create init_sl_point from : "<< init_point.DebugString();
    return false;
  }
// 起始点所对应的参考线上点
// step A:计算起始点的累计距离s，侧方相对偏移l，侧向速度dl和侧向加速度ddl
  if (!CalculateFrenetPoint(init_point_, &init_frenet_frame_point_)) {
    AERROR << "Fail to create init_frenet_frame_point_ from : "<< init_point_.DebugString();
    return false;
  }

// 代价最小的路径的节点
// step B:获取当前参考线下最优的前进路线
  std::vector<DPRoadGraphNode> min_cost_path;
  if (!GenerateMinCostPath(obstacles, &min_cost_path)) {
    AERROR << "Fail to generate graph!";
    return false;
  }
  // frenet路径
  // step C:将最优前进路线封装成path_data
  std::vector<common::FrenetFramePoint> frenet_path;
  float accumulated_s = init_sl_point_.s();
  const float path_resolution = config_.path_resolution();

  for (std::size_t i = 1; i < min_cost_path.size(); ++i) {
    const auto &prev_node = min_cost_path[i - 1]; //
    const auto &cur_node = min_cost_path[i];

    const float path_length = cur_node.sl_point.s() - prev_node.sl_point.s();
    float  current_s = 0.0;
    
    // 一维五次多项式曲线
    // Evaluate(const std::uint32_t order, const double p)，五次多项式，l=a0+a1*p+a2*p^2+a3+p^3+a4*p^4+a5*p^5,order代表执行几次求导
    const auto &curve = cur_node.min_cost_curve;
    // 对每一段curve采样(path_resolution为1米)
    while (current_s + path_resolution / 2.0 < path_length) {
      const float l = curve.Evaluate(0, current_s);
      const float dl = curve.Evaluate(1, current_s);
      const float ddl = curve.Evaluate(2, current_s);
      common::FrenetFramePoint frenet_frame_point;
      frenet_frame_point.set_s(accumulated_s + current_s);
      frenet_frame_point.set_l(l);
      frenet_frame_point.set_dl(dl);
      frenet_frame_point.set_ddl(ddl);
      frenet_path.push_back(std::move(frenet_frame_point));
      current_s += path_resolution;
    }
    if (i == min_cost_path.size() - 1) {
      accumulated_s += current_s;
    } else {
      accumulated_s += path_length;
    }
  }
  FrenetFramePath tunnel(frenet_path);
  path_data->SetReferenceLine(&reference_line_);
  path_data->SetFrenetPath(tunnel);
  return true;
}

// 查找代价最小路径的核心在于GenerateMinCostPath()，也是分为3部分：先采样，然后构造graph，
// 最后查找从起点（自车当前位置）到终点（尽可能远的某个采样点）的代价最小路径
// 将代价最小的点保存在min_cost_path 上
bool DPRoadGraph::GenerateMinCostPath(const std::vector<const PathObstacle *> &obstacles,
                                                                                          std::vector<DPRoadGraphNode> *min_cost_path) {
  CHECK(min_cost_path != nullptr);
  // 基于当前参考线及初始点，生成候选路径采样点数组
  // 路径航点（path_waypoints）里面的每个vecotr存储相同s值（轨迹累计弧长）下的多个采样点
  std::vector<std::vector<common::SLPoint>> path_waypoints;
  if (!SamplePathWaypoints(init_point_, &path_waypoints) ||path_waypoints.size() < 1) {
    AERROR << "Fail to sample path waypoints! reference_line_length = "
           << reference_line_.Length();
    return false;
  }

  // 将初始点加入到路径航点数组的最前面
  path_waypoints.insert(path_waypoints.begin(),std::vector<common::SLPoint>{init_sl_point_});
  // 读取车辆配置信息
  const auto &vehicle_config =common::VehicleConfigHelper::instance()->GetConfig();
// 轨迹代价
/*
1、障碍物处理
在规划中，需要考虑到障碍物的轨迹信息，也就是需要考虑未来每个时刻，障碍物出现的位置，
只要将障碍物每个时间点出现的位置，作为开销计算项即可。
*/ 
  TrajectoryCost trajectory_cost(
      config_, reference_line_, reference_line_info_.IsChangeLanePath(),
      obstacles, vehicle_config.vehicle_param(), speed_data_, init_sl_point_);
// 最小代价 路图节表点 链表
// 模板类list是一个容器，list是由双向链表来实现的，每个节点存储1个元素。list支持前后两种移动方向
 // ------这是最后的前向遍历图，类似于神经网络结构，N个level，每个level若干横向采样点，两层level之间的采样点互相连接。
  std::list<std::list<DPRoadGraphNode>> graph_nodes; //链表矩阵，链表中包含链表
   // ------  而且网络中的每个node，都保存了，从规划起始点到该节点的最小cost，以及反向连接链(cost最小对应的parent)
  graph_nodes.emplace_back();
  graph_nodes.back().emplace_back(init_sl_point_, nullptr, ComparableCost());
  auto &front = graph_nodes.front().front();  // 规划起始点
  // 总共的level数-----总共采样了几个横向 ----------- 网络层数，level数量
  size_t total_level = path_waypoints.size();

  // 采用自下而上的动态规划算法，迭代更新最小代价值
  // graph_nodes存储的就是各级（level）路径航点（path_waypoints）所包含的最小代价航点
  // graph_nodes.back()（即最后一条航点链表）就是我们所需的最小代价航点链表
  for (std::size_t level = 1; level < total_level; ++level) { // 网络两两level之间计算连接cost
    // 前链表航点数组------------前一层level
    const auto &prev_dp_nodes = graph_nodes.back();
    // 第level中的所有采样点 ------------ 当前层level中的所有横向采样点
    const auto &level_points = path_waypoints[level];

    graph_nodes.emplace_back();

    for (size_t i = 0; i < level_points.size(); ++i) { // 计算当前层level中与前一层所有计算的连接权值，也就是cost
      const auto &cur_point = level_points[i];

      graph_nodes.back().emplace_back(cur_point, nullptr);
      auto &cur_node = graph_nodes.back().back();
      // 采用多线程并行计算最小代价航点
      if (FLAGS_enable_multi_thread_in_dp_poly_path) {
        PlanningThreadPool::instance()->Push(std::bind(&DPRoadGraph::UpdateNode, this, std::ref(prev_dp_nodes), level,
            total_level, &trajectory_cost, &(front), &(cur_node)));

      } else {
        // 采用单线程计算最小代价航点
        // 计算前一层prev_dp_nodes和当前层的节点cur_node的开销cost，取prev_dp_nodes中与cur_node开销cost最小的节点，设置为最优路径
        UpdateNode(prev_dp_nodes, level, total_level, &trajectory_cost, &front,&cur_node);
      }
    }
    // 多线程模式下的同步
    if (FLAGS_enable_multi_thread_in_dp_poly_path) {
      PlanningThreadPool::instance()->Synchronize();
    }
  }

  // find best path  发现代价最小的路径
  // graph_nodes.back()（即最后一条航点链表）就是我们所需的最小代价航点链表
  DPRoadGraphNode fake_head;
  for (const auto &cur_dp_node : graph_nodes.back()) {
    fake_head.UpdateCost(&cur_dp_node, cur_dp_node.min_cost_curve,cur_dp_node.min_cost);
  }
// 从终点顺藤摸瓜向起点逐个找出最小代价值航点，并将其加入min_cost_path
  const auto *min_cost_node = &fake_head;
  // 判断条件是前一个最小代价航点是否为空
  while (min_cost_node->min_cost_prev_node) {
    min_cost_node = min_cost_node->min_cost_prev_node;
    min_cost_path->push_back(*min_cost_node);
  }
  if (min_cost_node != &graph_nodes.front().front()) {
    return false;
  }
 // 将航点顺序调整为起点到终点
  std::reverse(min_cost_path->begin(), min_cost_path->end());

  for (const auto &node : *min_cost_path) {
    ADEBUG << "min_cost_path: " << node.sl_point.ShortDebugString();
    planning_debug_->mutable_planning_data()
        ->mutable_dp_poly_graph()
        ->add_min_cost_point()
        ->CopyFrom(node.sl_point);
  }
  return true;
}

// 在当前level下，获得一条代价值最小的航点链表
/*
1、在当前current node 与任一prev node间、以及current node与first node间构造五次多项式曲线；
2、计算这2个node间的cost；
3、更新current node 的cost；
*/ 
void DPRoadGraph::UpdateNode(const std::list<DPRoadGraphNode> &prev_nodes,
                                                                      const uint32_t level, const uint32_t total_level,
                                                                      TrajectoryCost *trajectory_cost,
                                                                      DPRoadGraphNode *front,
                                                                      DPRoadGraphNode *cur_node) {
  DCHECK_NOTNULL(trajectory_cost);
  DCHECK_NOTNULL(front);
  DCHECK_NOTNULL(cur_node);

  for (const auto &prev_dp_node : prev_nodes) {
    const auto &prev_sl_point = prev_dp_node.sl_point;
    const auto &cur_point = cur_node->sl_point;
    float init_dl = 0.0;
    float init_ddl = 0.0;
    if (level == 1) {
      // 仅自车当前姿态有dl（角度朝向）,ddl,其余点的dl,ddl都为0
      init_dl = init_frenet_frame_point_.dl();
      init_ddl = init_frenet_frame_point_.ddl();
    }
        // 生成当前点到前一level所有航点的的曲线
    QuinticPolynomialCurve1d curve(prev_sl_point.l(), init_dl, init_ddl,
                                   cur_point.l(), 0.0, 0.0,
                                   cur_point.s() - prev_sl_point.s());

    if (!IsValidCurve(curve)) {
      continue;
    }
    // node间的cost主要分为3部分：路径平滑，避开静态障碍物，避开动态障碍物，
    // 使用comparableCost类描述，该类除包含常规的safety_cost（无碰）和smoothness_cost（平滑）外，
    // 还含有3个bool型的成员变量，以表示是否碰撞、是否超出边界、是否超出车道线。
    // ------------- cur_code与pre_dp_node相连得到cost
    const auto cost =trajectory_cost->Calculate(curve, prev_sl_point.s(), cur_point.s(),level, total_level) +prev_dp_node.min_cost;
    // 根据代价最小的原则，在前一level所有航点中找到与当前点连接代价最小的航点，
    // 将结果存储于prev_dp_node中
    // -----------刷新最小cost，得到与cur_node相连最小的cost,以及对应的prev_dp_node
    cur_node->UpdateCost(&prev_dp_node, curve, cost);
  }
    // 尝试将当前点直接连接到初始点，看其代价是否比当前点到前一level航点的最小代价还小，
    // 若小于则将最小代价航点更新。这种情况一般只会存在于改变车道的情形。
  // try to connect the current point with the first point directly
  if (level >= 2) {
    const float init_dl = init_frenet_frame_point_.dl();
    const float init_ddl = init_frenet_frame_point_.ddl();
    QuinticPolynomialCurve1d curve(init_sl_point_.l(), init_dl, init_ddl,
                                   cur_node->sl_point.l(), 0.0, 0.0,
                                   cur_node->sl_point.s() - init_sl_point_.s());
    if (!IsValidCurve(curve)) {
      return;
    }
    const auto cost = trajectory_cost->Calculate(curve, init_sl_point_.s(), cur_node->sl_point.s(), level, total_level);
    cur_node->UpdateCost(front, curve, cost);
  }
}

// 采样点（采样航点），航点数组生成
bool DPRoadGraph::SamplePathWaypoints(
    const common::TrajectoryPoint &init_point,
    std::vector<std::vector<common::SLPoint>> *const points) {
  CHECK_NOTNULL(points);
// 最小的采样距离（应该是在这个距离内采样）---------  未来40m的距离
  const float kMinSampleDistance = 40.0;
  // sl坐标下总的路径长度，不能超过参考线的长度
  const float total_length = std::fmin(init_sl_point_.s() + std::fmax(init_point.v() * 8.0, kMinSampleDistance),reference_line_.Length());
  const auto &vehicle_config =common::VehicleConfigHelper::instance()->GetConfig();
  // 车宽的一半
  const float half_adc_width = vehicle_config.vehicle_param().width() / 2.0;
  // --------------大约10-20米进行一次纵向采样，横向采样（对车道不同宽度进行采样）
  // ------------- 每个level的采样个数
  const size_t num_sample_per_level =
                    FLAGS_use_navigation_mode ? config_.navigator_sample_num_each_level(): config_.sample_points_num_each_level();
// 是否绕行
  const bool has_sidepass = HasSidepass();
// 样本点向前采样时间
  constexpr float kSamplePointLookForwardTime = 4.0;
  // 纵向步长[8,15],该函数是返回[8,15]之间的值
  const float step_length =common::math::Clamp(init_point.v() * kSamplePointLookForwardTime,
                                                                                                       config_.step_length_min(), config_.step_length_max());
//采样步长，两个level之间的距离
  const float level_distance =(init_point.v() > FLAGS_max_stop_speed) ? step_length : step_length / 2.0;
  // 累计轨迹弧长
  float accumulated_s = init_sl_point_.s();
  // 上次轨迹弧长
  float prev_s = accumulated_s;

  auto *status = util::GetPlanningStatus();
  if (status == nullptr) {
    AERROR << "Fail to  get planning status.";
    return false;
  }
// 无人车当前状态正在寻找停车点，也就是PULL_OVER状态，采样点其实只要设置pull_over的计算得到起始点即可
// 当前前提条件是需要进入到他的可操作区域
  if (status->planning_state().has_pull_over() &&status->planning_state().pull_over().in_pull_over()) {
    status->mutable_planning_state()->mutable_pull_over()->set_status(PullOverStatus::IN_OPERATION);
    const auto &start_point = status->planning_state().pull_over().start_point();
    SLPoint start_point_sl;
    if (!reference_line_.XYToSL(start_point, &start_point_sl)) {
      AERROR << "Fail to change xy to sl.";
      return false;
    }
// pull over靠边停车    ------ 表示无人车已进入PULL_OVER可操作区域
    if (init_sl_point_.s() > start_point_sl.s()) {
      const auto &stop_point =status->planning_state().pull_over().stop_point();
      SLPoint stop_point_sl;
      if (!reference_line_.XYToSL(stop_point, &stop_point_sl)) {
        AERROR << "Fail to change xy to sl.";
        return false;
      }
      // 这时候只要设置停车点为下一时刻(纵向)的采样点即可，那么横向采样点就只有一个，就是停车位置
      std::vector<common::SLPoint> level_points(1, stop_point_sl);
      points->emplace_back(level_points);
      return true;
    }
  }
   // 累计弧长小于总长度时，将累计轨迹弧长每次加上采样步长，进行循环采样
  for (std::size_t i = 0; accumulated_s < total_length; ++i) {
    accumulated_s += level_distance;
    if (accumulated_s + level_distance / 2.0 > total_length) {
      accumulated_s = total_length;
    }
    // 本次轨迹弧长，取累计弧长和总长度的最小值
    const float s = std::fmin(accumulated_s, total_length);
    // 最小允许采样步长
    constexpr float kMinAllowedSampleStep = 1.0;
    // 若本次轨迹弧长于上次轨迹弧长间的差值小于最小允许的采样步长，跳过本次采样
    if (std::fabs(s - prev_s) < kMinAllowedSampleStep) {
      continue;
    }
    prev_s = s;
  // 左右车道宽度
    double left_width = 0.0;
    double right_width = 0.0;
    // 得到高精度地图中左右车道线宽度 
    // ------------- 计算纵向每个位置，有效的左右边界，kBoundaryBuff是边界缓冲。需要保持无人车在车道内行驶。
    reference_line_.GetLaneWidth(s, &left_width, &right_width);
  // 边界缓冲
    constexpr float kBoundaryBuff = 0.20;
    // 右车道允许宽度 = 右车道宽-半车宽-边界缓冲
    const float eff_right_width = right_width - half_adc_width - kBoundaryBuff;
    const float eff_left_width = left_width - half_adc_width - kBoundaryBuff;
   
  //  车变道情形下L的启发式移位
    // the heuristic shift of L for lane change scenarios
    const double delta_dl = 1.2 / 20.0;
    const double kChangeLaneDeltaL = common::math::Clamp(level_distance * (std::fabs(init_frenet_frame_point_.dl()) + delta_dl),1.2, 3.5);
    // 横向采样间隔,点与点之间的间隔，差不多0.2m
    float kDefaultUnitL = kChangeLaneDeltaL / (num_sample_per_level - 1);
    // 如果当前参考线是变道，且变道不安全（无人车前后一定距离内有障碍物）
    // 那么增加采样点间隔，这样可以下一时刻减少变道的时间
    if (reference_line_info_.IsChangeLanePath() &&!reference_line_info_.IsSafeToChangeLane()) {
      kDefaultUnitL = 1.0;
    }
    // 横向采样的宽度
    const float sample_l_range = kDefaultUnitL * (num_sample_per_level - 1);
    // 右采样边界（车辆右侧为负）------右边界
    float sample_right_boundary = -eff_right_width;
    float sample_left_boundary = eff_left_width; // 左边界
  //  L最大的偏差
    const float kLargeDeviationL = 1.75;
    // 参考线正在改变车道时，------------选取左右采样边界
    // 修正左右边界，如果参考线需要变道，并且已经便宜车道了，那么就将横向采样区间向变道方向平移。
    if (reference_line_info_.IsChangeLanePath() ||std::fabs(init_sl_point_.l()) > kLargeDeviationL)
     {
      // 右采样边界取  右采样边界与初始点横向偏移之间的最小值 ---------- 左边界，修正左边界
      sample_right_boundary = std::fmin(-eff_right_width, init_sl_point_.l());
      // 左采样边界 ----------------  右变道，修正右边界
      sample_left_boundary = std::fmax(eff_left_width, init_sl_point_.l());  

      // 若初始点横向偏移 > 左侧允许宽度，则将右侧采样边界设置为右侧采样边界与（初始点横向偏移- 横向采样距离）之间的最大值
      //  ----------------- 左变道，修正右边界，向左平移，因为左变道车道线右边的区域不用考虑
      if (init_sl_point_.l() > eff_left_width) {
        sample_right_boundary = std::fmax(sample_right_boundary,init_sl_point_.l() - sample_l_range);
      }
      // 若初始点横向偏移 < 右侧允许宽度，则将左侧采样边界设置为左侧采样边界与（初始点横向偏移+ 横向采样距离）之间的最小值
      // ------------------  右变道，修正左边界，向右平移，因为右变道车道线左边的区域不用考虑
      if (init_sl_point_.l() < eff_right_width) {
        sample_left_boundary = std::fmin(sample_left_boundary,
                                         init_sl_point_.l() + sample_l_range);
      }
    }
    // 横向采样距离数组（横向采样距离应该是横向的自由空间）
    std::vector<float> sample_l;
    // 每个纵向位置的横向区间采样，采样分为多种情况
    // 情况1：如果当前参考线需要变道，并且变道不安全，那么横向采样点就设置为第二条参考线的位置，直接走第二条参考线。
// 参考线正在改变车道且改变车道不安全时，将当前参考线到其他参考线的偏移值存储到横向采样距离数组
    if (reference_line_info_.IsChangeLanePath() &&!reference_line_info_.IsSafeToChangeLane()) {
      sample_l.push_back(reference_line_info_.OffsetToOtherReferenceLine());
    }
    // 情况2：如果当前参考线需要侧方绕行(SIDEPASS)，即从旁边超车。
    // 若是可以进行左边超车，那么横向采样点设置为左边界+超车距离；右边超车，横向采样点设置为右边界+超车距离
     else if (has_sidepass) {
      // currently only left nudge is supported. Need road hard boundary for
      // both sides
      switch (sidepass_.type()) {
        case ObjectSidePass::LEFT: {
          // 左侧绕行：将（左侧允许宽度 + 左侧绕行距离）存储到横向采样距离数组
          sample_l.push_back(eff_left_width + config_.sidepass_distance());
          break;
        }
        case ObjectSidePass::RIGHT: {
          // 右侧绕行：将-（右侧允许宽度 + 右侧绕行距离）存储到横向采样距离数组
          sample_l.push_back(-eff_right_width - config_.sidepass_distance());
          break;
        }
        default:
          break;
      }
    } 
    // 正常行驶情况下，在横向区间中进行均匀采样
    else {
      // 其他情形，从右采样边界到左采样边界，按照每步采样点数进行均匀采样，并将结果存储到横向采样距离数组
      common::util::uniform_slice(sample_right_boundary, sample_left_boundary,num_sample_per_level - 1, &sample_l);
    }
    // 本次采样点数组
    std::vector<common::SLPoint> level_points;
    planning_internal::SampleLayerDebug sample_layer_debug;
    for (size_t j = 0; j < sample_l.size(); ++j) { //每个纵向位置采样得到的所有横向采样点，封装成一个level
      // SL采样点坐标
      common::SLPoint sl = common::util::MakeSLPoint(s, sample_l[j]);
      sample_layer_debug.add_sl_point()->CopyFrom(sl);
      // 将当前采样点坐标存储到本次采样点数组
      level_points.push_back(std::move(sl));
    }

    // 若参考线未改变车道且不绕行，则将横向偏移值为0的采样点（即沿参考线方向的采样点）也加入本次采样点数组
    // 对于不变道但是要超车的情况，额外增加一个车道线中心采样点
    if (!reference_line_info_.IsChangeLanePath() && has_sidepass) {
      auto sl_zero = common::util::MakeSLPoint(s, 0.0);//MakeSLPoint(s,l)
      sample_layer_debug.add_sl_point()->CopyFrom(sl_zero);
      level_points.push_back(std::move(sl_zero));
    }

    if (!level_points.empty()) {
      planning_debug_->mutable_planning_data()->mutable_dp_poly_graph()->add_sample_layer() ->CopyFrom(sample_layer_debug);
      // 将本次的所有采样点存储到总采样点数组
      points->emplace_back(level_points);
    }
  }
  return true;
}

// 计算frenet点,frenet坐标系中点
bool DPRoadGraph::CalculateFrenetPoint(
    const common::TrajectoryPoint &traj_point,
    common::FrenetFramePoint *const frenet_frame_point) {
  common::SLPoint sl_point;
  // 将笛卡尔坐标点转化为sl坐标点
  if (!reference_line_.XYToSL({traj_point.path_point().x(), traj_point.path_point().y()},&sl_point)) {
    return false;
  }
  frenet_frame_point->set_s(sl_point.s());
  frenet_frame_point->set_l(sl_point.l());

  const float theta = traj_point.path_point().theta();
  const float kappa = traj_point.path_point().kappa();
  const float l = frenet_frame_point->l();

  ReferencePoint ref_point;
  ref_point = reference_line_.GetReferencePoint(frenet_frame_point->s());

  const float theta_ref = ref_point.heading();
  const float kappa_ref = ref_point.kappa();
  const float dkappa_ref = ref_point.dkappa();

  const float dl = CartesianFrenetConverter::CalculateLateralDerivative(theta_ref, theta, l, kappa_ref);
  const float ddl =CartesianFrenetConverter::CalculateSecondOrderLateralDerivative(theta_ref, theta, kappa_ref, kappa, dkappa_ref, l);
  frenet_frame_point->set_dl(dl);
  frenet_frame_point->set_ddl(ddl);
  return true;
}

bool DPRoadGraph::IsValidCurve(const QuinticPolynomialCurve1d &curve) const {
  constexpr float kMaxLateralDistance = 20.0;
  for (float s = 0.0; s < curve.ParamLength(); s += 2.0) {
    const float l = curve.Evaluate(0, s); //ls五次多项式曲线，Evaluate(order,p) 
    if (std::fabs(l) > kMaxLateralDistance) {
      return false;
    }
  }
  return true;
}

void DPRoadGraph::GetCurveCost(TrajectoryCost trajectory_cost,
                               const QuinticPolynomialCurve1d &curve,
                               const float start_s, const float end_s,
                               const uint32_t curr_level,
                               const uint32_t total_level,
                               ComparableCost *cost) {
  *cost =trajectory_cost.Calculate(curve, start_s, end_s, curr_level, total_level);
}

bool DPRoadGraph::HasSidepass() {
  const auto &path_decision = reference_line_info_.path_decision();
  for (const auto &obstacle : path_decision.path_obstacles().Items()) {
    if (obstacle->LateralDecision().has_sidepass()) {
      sidepass_ = obstacle->LateralDecision().sidepass();
      return true;
    }
  }
  return false;
}

}  // namespace planning
}  // namespace apollo
