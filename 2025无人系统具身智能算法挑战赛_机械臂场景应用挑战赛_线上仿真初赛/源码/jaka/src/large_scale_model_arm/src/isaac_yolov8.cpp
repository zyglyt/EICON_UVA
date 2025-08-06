/*

* 2025无人系统具身智能算法挑战赛 专用代码
* 版权所有 (c) 2025 无人系统具身智能算法挑战赛组委会
* 
* 本源码仅限本赛事参赛团队在比赛过程中使用，
* 禁止任何形式的商业用途、非授权传播或用于其他非比赛场景。
* 
* 依照 GNU 通用公共许可证（GPL）条款授权：
* 参赛者可基于赛事目的对源码进行修改和扩展，
* 但修改后的代码仍受限于本声明的约束条款。
* 
* 本源码按"现状"提供，组委会不承担任何明示或暗示的担保责任，
* 包括但不限于适销性或特定用途适用性的保证。

*/
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/CameraInfo.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <regex>
#include <cmath>
#include <vector>

class ModelToArmNode
{
public:
  ModelToArmNode(ros::NodeHandle &nh);

private:
  void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &info);
  void modelCallback(const std_msgs::String::ConstPtr &msg);
  void pickResultCallback(const std_msgs::Bool::ConstPtr &msg);
  void publishDetection(size_t idx);

  ros::NodeHandle &nh_;
  ros::Subscriber cam_info_sub_, model_sub_, result_sub_;
  ros::Publisher pose_pub_, side_pub_;

  Eigen::Matrix3d R_;
  Eigen::Vector3d T_;
  double fx_, fy_, cx_, cy_;
  bool got_cam_info_;

  std::vector<std::array<double, 7>> detections_;
  size_t current_idx_;
};

ModelToArmNode::ModelToArmNode(ros::NodeHandle &nh)
    : nh_(nh), got_cam_info_(false), current_idx_(0)
{
  cam_info_sub_ = nh_.subscribe("/Jaka/camera/camera_info", 1,
                                &ModelToArmNode::cameraInfoCallback, this);
  model_sub_ = nh_.subscribe("/model_output", 10,
                             &ModelToArmNode::modelCallback, this);
  result_sub_ = nh_.subscribe("/pick_place_result", 10,
                              &ModelToArmNode::pickResultCallback, this);

  pose_pub_ = nh_.advertise<std_msgs::Float64MultiArray>(
      "/arm_end_pose_quaternion", 10);
  side_pub_ = nh_.advertise<std_msgs::String>(
      "/conveyor_side", 10);

  R_ << 0.99887577, -0.04588472, -0.01190774,
      0.04628524, 0.99828373, 0.03587854,
      0.01024103, -0.03638936, 0.99928521;
  T_ << 0.0470183, 0.03276388, 0.35385047;

  ROS_INFO("ModelToArmNode 启动，等待 /Jaka/camera_info 和 /model_output 消息...");
}

void ModelToArmNode::cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &info)
{
  fx_ = info->K[0];
  fy_ = info->K[4];
  cx_ = info->K[2];
  cy_ = info->K[5];
  got_cam_info_ = true;
  ROS_INFO("Got camera intrinsics: fx=%.2f, fy=%.2f, cx=%.2f, cy=%.2f",
           fx_, fy_, cx_, cy_);
  cam_info_sub_.shutdown();
}

void ModelToArmNode::modelCallback(const std_msgs::String::ConstPtr &msg)
{
  if (!got_cam_info_)
  {
    ROS_WARN_THROTTLE(5.0, "还没收到 camera_info，无法计算 3D 坐标");
    return;
  }
  const auto &s = msg->data;
  ROS_INFO("收到 /model_output: %s", s.c_str());

  std_msgs::String side_msg;
  if (s.find(u8"左侧") != std::string::npos)
    side_msg.data = "left";
  else if (s.find(u8"右侧") != std::string::npos)
    side_msg.data = "right";
  else
    side_msg.data = "unknown";
  side_pub_.publish(side_msg);

  static const std::regex re(R"(\(\s*(\d+)\s*,\s*(\d+)\s*,\s*([\d\.]+)m\s*,\s*([\d\.]+)\s*\))");
  std::sregex_iterator it(s.begin(), s.end(), re), end;
  detections_.clear();
  for (; it != end; ++it)
  {
    double u = std::stod((*it)[1].str());
    double v = std::stod((*it)[2].str());
    double Z = std::stod((*it)[3].str());
    double angle_rad = std::stod((*it)[4].str()) * M_PI / 180.0;

    double Xc = (u - cx_) * Z / fx_;
    double Yc = (v - cy_) * Z / fy_;
    Eigen::Vector3d p_cam(Xc, Yc, Z), p_base = R_ * p_cam + T_;

    Eigen::Quaterniond q(0.0, 0.0, angle_rad, 1.57);
    q.normalize();

    detections_.push_back({-p_base.x(), p_base.y(), p_base.z(),
                           q.x(), q.y(), q.z(), q.w()});
    ROS_INFO("检测到瓶子 %zu: pos(%.3f,%.3f,%.3f)", detections_.size(),
             p_base.x(), p_base.y(), p_base.z());
  }
  if (detections_.empty())
  {
    ROS_WARN("无法提取任何检测结果");
    return;
  }
  current_idx_ = 0;
  publishDetection(current_idx_);
}

void ModelToArmNode::publishDetection(size_t idx)
{
  if (idx >= detections_.size())
    return;
  std_msgs::Float64MultiArray msg;
  msg.data.assign(detections_[idx].begin(), detections_[idx].end());
  pose_pub_.publish(msg);
  ROS_INFO("发布第 %zu 个目标", idx + 1);
}

void ModelToArmNode::pickResultCallback(const std_msgs::BoolConstPtr &msg)
{
  if (msg->data)
    ROS_INFO("第 %zu 个目标抓取成功", current_idx_ + 1);
  else
    ROS_WARN("第 %zu 个目标抓取失败", current_idx_ + 1);

  ++current_idx_;
  if (current_idx_ < detections_.size())
    publishDetection(current_idx_);
  else
    ROS_INFO("所有 %zu 个目标处理完毕", detections_.size());
}

int main(int argc, char **argv)
{
  setlocale(LC_ALL, "");
  ros::init(argc, argv, "model_to_arm_node");
  ros::NodeHandle nh;
  ModelToArmNode node(nh);
  ros::spin();
  return 0;
}