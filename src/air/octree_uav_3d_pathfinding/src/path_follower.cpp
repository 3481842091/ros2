// =============================================================================
// path_follower —— 把规划出的路径逐点喂给控制器，实现闭环飞行
//
// 分工：
//   · 状态机（前瞻切换 / 终点判定 / 超时）在 include/.../path_follow_core.h 里，
//     是纯函数，可离线单测。本文件只做消息转换与发布。
//   · "飞到哪里"由桥接节点执行；它默认订阅 /uav/goal，
//     本节点发的是 /uav/goal_cmd —— 两者分开才不会让 astar_planner
//     把每个航点都当成"用户下发了新目标"而重新规划（见 design §1）。
//
// 话题：
//   订阅  path_topic      nav_msgs/Path       要跟踪的路径（默认 /uav/path_smooth）
//   订阅  odom_topic      nav_msgs/Odometry   位姿（默认 /ground_truth/odom，20 Hz）
//   发布  goal_cmd_topic  geometry_msgs/Point 当前要飞的航点（默认 /uav/goal_cmd）
//   发布  status_topic    std_msgs/String     状态文本，供日志与 RViz
//   发布  flown_topic     nav_msgs/Path       实际飞过的轨迹，便于与规划路径对比
//
// 状态机的时钟来自 odom 回调：位姿每 20 Hz 来一次，正好是"该不该切下一个点"的判断时机。
// =============================================================================

#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include "octree_uav_3d_pathfinding/path_follow_core.h"

using octree_uav_3d_pathfinding::FollowAction;
using octree_uav_3d_pathfinding::FollowOptions;
using octree_uav_3d_pathfinding::FollowPhase;
using octree_uav_3d_pathfinding::FollowState;
using octree_uav_3d_pathfinding::FollowStep;
using octree_uav_3d_pathfinding::FollowVec3;
using octree_uav_3d_pathfinding::followPolylineLength;
using octree_uav_3d_pathfinding::followReset;
using octree_uav_3d_pathfinding::followStep;
using octree_uav_3d_pathfinding::followVec;

namespace {

const char* phaseName(FollowPhase p) {
  switch (p) {
    case FollowPhase::Idle:      return "IDLE";
    case FollowPhase::Following: return "FOLLOWING";
    case FollowPhase::Done:      return "DONE";
  }
  return "?";
}

class PathFollower {
 public:
  PathFollower() : pnh_("~") {
    // ---------------- 话题 ----------------
    path_topic_ = pnh_.param<std::string>("path_topic", "/uav/path_smooth");
    odom_topic_ = pnh_.param<std::string>("odom_topic", "/ground_truth/odom");
    goal_cmd_topic_ = pnh_.param<std::string>("goal_cmd_topic", "/uav/goal_cmd");
    status_topic_ = pnh_.param<std::string>("status_topic", "/uav/follow_status");
    flown_topic_ = pnh_.param<std::string>("flown_topic", "/uav/flown_path");
    world_frame_ = pnh_.param<std::string>("world_frame", "world");
    body_frame_ = pnh_.param<std::string>("body_frame", "base_link");

    // ---------------- 参数 ----------------
    opt_.lookahead = pnh_.param("lookahead", 1.5);
    opt_.finish_tolerance = pnh_.param("finish_tolerance", 0.5);
    opt_.per_point_timeout = pnh_.param("per_point_timeout", 15.0);
    auto_start_ = pnh_.param("auto_start", true);
    publish_flown_ = pnh_.param("publish_flown", true);

    ROS_INFO("path_follower started");
    ROS_INFO("  path topic    : %s", path_topic_.c_str());
    ROS_INFO("  odom topic    : %s", odom_topic_.c_str());
    ROS_INFO("  goal cmd topic: %s", goal_cmd_topic_.c_str());
    ROS_INFO("  frames        : %s <- %s", world_frame_.c_str(), body_frame_.c_str());
    ROS_INFO("  lookahead     : %.2f m (switch waypoint within this distance)",
             opt_.lookahead);
    ROS_INFO("  finish tol.   : %.2f m", opt_.finish_tolerance);
    ROS_INFO("  per-point t/o : %.1f s", opt_.per_point_timeout);
    ROS_INFO("  auto start    : %s", auto_start_ ? "true" : "false");

    if (opt_.lookahead <= opt_.finish_tolerance) {
      ROS_WARN("lookahead (%.2f) <= finish_tolerance (%.2f): the vehicle will stop at "
               "every waypoint; continuous flight needs lookahead > finish_tolerance",
               opt_.lookahead, opt_.finish_tolerance);
    }

    // ---------------- 收发 ----------------
    goal_pub_ = nh_.advertise<geometry_msgs::Point>(goal_cmd_topic_, 1, true);
    status_pub_ = nh_.advertise<std_msgs::String>(status_topic_, 10, true);
    flown_pub_ = nh_.advertise<nav_msgs::Path>(flown_topic_, 1, true);
    path_sub_ = nh_.subscribe(path_topic_, 1, &PathFollower::pathCallback, this);
    odom_sub_ = nh_.subscribe(odom_topic_, 1, &PathFollower::odomCallback, this);

    publishStatus("IDLE waiting for a path");
  }

 private:
  // ------------------------------------------------------------------
  // 路径：转成核心结构 → 复位状态机
  // ------------------------------------------------------------------
  void pathCallback(const nav_msgs::Path::ConstPtr& msg) {
    std::vector<FollowVec3> path;
    path.reserve(msg->poses.size());
    for (std::size_t i = 0; i < msg->poses.size(); ++i) {
      const geometry_msgs::Point& p = msg->poses[i].pose.position;
      path.push_back(followVec(p.x, p.y, p.z));
    }

    if (path.size() < 2) {
      ROS_WARN("received a path with %zu point(s); need at least 2, ignoring",
               path.size());
      return;
    }

    // 收到新路径一律复位：旧下标可能已经因为地图更新而失效
    path_.swap(path);
    followReset(state_);
    length_ = followPolylineLength(path_);
    started_s_ = -1.0;
    flown_.clear();

    ROS_INFO("path received: %zu points, length %.2f m (tracking %s)",
             path_.size(), length_, path_topic_.c_str());

    if (auto_start_) {
      publishStatus("IDLE path received, waiting for the first pose");
    }
  }

  // ------------------------------------------------------------------
  // 位姿：驱动状态机
  // ------------------------------------------------------------------
  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    if (path_.size() < 2) return;

    const geometry_msgs::Point& p = msg->pose.pose.position;
    const FollowVec3 pos = followVec(p.x, p.y, p.z);
    const double now = ros::Time::now().toSec();

    if (!auto_start_ && state_.phase == FollowPhase::Idle) return;

    const FollowStep step = followStep(state_, path_, pos, now, opt_);

    // 记录实际轨迹（与规划路径对比用）
    if (publish_flown_ && state_.phase != FollowPhase::Idle) {
      geometry_msgs::PoseStamped ps;
      ps.header.stamp = msg->header.stamp;
      ps.header.frame_id = msg->header.frame_id.empty() ? world_frame_
                                                        : msg->header.frame_id;
      ps.pose = msg->pose.pose;
      flown_.push_back(ps);

      // 飞行中每秒发一次，让 RViz 里能看到轨迹**边飞边生长**；
      // 只在终点发一次的话，动图录不到过程，只有最后一条完整的线
      if (now - last_flown_pub_s_ > 1.0) {
        last_flown_pub_s_ = now;
        publishFlown();
      }
    }

    if (step.timed_out) {
      ROS_WARN("waypoint %zu timed out after %.1f s (distance %.2f m); skipping",
               step.index, opt_.per_point_timeout, step.distance_to_index);
    }

    switch (step.action) {
      case FollowAction::Send:
        sendGoal(path_[step.index]);
        if (started_s_ < 0.0) started_s_ = now;
        publishStatus(std::string(phaseName(state_.phase)) + " " +
                      std::to_string(step.index + 1) + "/" +
                      std::to_string(path_.size()));
        break;

      case FollowAction::Finished:
        finish(now);
        break;

      case FollowAction::None:
        break;
    }
  }

  void sendGoal(const FollowVec3& p) {
    geometry_msgs::Point msg;
    msg.x = p.x;
    msg.y = p.y;
    msg.z = p.z;
    goal_pub_.publish(msg);
    ROS_DEBUG("sent goal cmd (%.2f, %.2f, %.2f)", msg.x, msg.y, msg.z);
  }

  void finish(double now) {
    const double duration = (started_s_ >= 0.0) ? (now - started_s_) : 0.0;

    // 平均速度必须用**实际飞过的距离**，不能用规划路径长度。
    // 前瞻跳点会切内角，实际路程比规划路径短；用路径长度当分子算出来的
    // 是"路径推进速率"，会虚高 —— 实测 37.22 m / 10.90 s = 3.41 m/s，
    // 而载具的实际飞行速度上限是 goal_velocity = 3.0 m/s，物理上不可能超过。
    const double flown = flownLength();
    const double mean_speed = (duration > 0.0) ? (flown / duration) : 0.0;
    const std::size_t skipped = state_.skipped;
    const std::size_t reached = path_.size() - skipped;

    ROS_INFO("follow: DONE | waypoints=%zu reached=%zu skipped=%zu | "
             "path=%.2f m flown=%.2f m | duration=%.2fs | mean_speed=%.2f m/s "
             "| max_dev=%.2f m",
             path_.size(), reached, skipped, length_, flown, duration,
             mean_speed, state_.max_deviation);

    // flown < path 就是"切内角"的另一个证据：少走的那部分，正是斜穿拐角省下的路
    if (length_ > 0.0) {
      ROS_INFO("  corner cutting: flown is %.1f%% shorter than the path",
               100.0 * (length_ - flown) / length_);
    }

    publishStatus("DONE " + std::to_string(reached) + "/" +
                  std::to_string(path_.size()));

    if (publish_flown_ && !flown_.empty()) {
      publishFlown();
      ROS_INFO("published %zu flown poses on %s", flown_.size(),
               flown_topic_.c_str());
    }
  }

  /// 实际飞过的总路程（m）：相邻位姿的欧氏距离累加。
  double flownLength() const {
    double total = 0.0;
    for (std::size_t i = 0; i + 1 < flown_.size(); ++i) {
      const geometry_msgs::Point& a = flown_[i].pose.position;
      const geometry_msgs::Point& b = flown_[i + 1].pose.position;
      const double dx = a.x - b.x;
      const double dy = a.y - b.y;
      const double dz = a.z - b.z;
      total += std::sqrt(dx * dx + dy * dy + dz * dz);
    }
    return total;
  }

  /// 把到目前为止的轨迹发出去。飞行中每秒一次，到达时再发一次（补齐最后一段）。
  void publishFlown() {
    if (flown_.empty()) return;
    nav_msgs::Path flown;
    flown.header.stamp = ros::Time::now();
    flown.header.frame_id = world_frame_;
    flown.poses = flown_;
    flown_pub_.publish(flown);
  }

  void publishStatus(const std::string& text) {
    std_msgs::String msg;
    msg.data = text;
    status_pub_.publish(msg);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  std::string path_topic_;
  std::string odom_topic_;
  std::string goal_cmd_topic_;
  std::string status_topic_;
  std::string flown_topic_;
  std::string world_frame_;
  std::string body_frame_;

  FollowOptions opt_;
  bool auto_start_ = true;
  bool publish_flown_ = true;

  FollowState state_;
  std::vector<FollowVec3> path_;
  std::vector<geometry_msgs::PoseStamped> flown_;
  double length_ = 0.0;
  double started_s_ = -1.0;
  double last_flown_pub_s_ = 0.0;   ///< 上次发布实际轨迹的时刻（飞行中限流到 1 Hz）

  ros::Publisher goal_pub_;
  ros::Publisher status_pub_;
  ros::Publisher flown_pub_;
  ros::Subscriber path_sub_;
  ros::Subscriber odom_sub_;
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "path_follower");
  PathFollower node;
  ros::spin();
  return 0;
}
