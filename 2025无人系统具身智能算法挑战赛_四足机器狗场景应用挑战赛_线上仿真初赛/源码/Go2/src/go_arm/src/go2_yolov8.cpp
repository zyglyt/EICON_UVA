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
  void pickResultCallback(const std_msgs::BoolConstPtr &msg);
  void publishDetection(size_t idx);

  ros::Subscriber cam_info_sub_, model_sub_, result_sub_;
  ros::Publisher pose_pub_;

  double fx_, fy_, cx_, cy_;
  bool got_cam_info_;

  Eigen::Quaterniond fixed_q_;
  Eigen::Matrix3d R_;
  Eigen::Vector3d T_;

  std::vector<std::array<double, 8>> detections_;
  size_t current_idx_;
};

ModelToArmNode::ModelToArmNode(ros::NodeHandle &nh)
    : got_cam_info_(false), current_idx_(0),
      fixed_q_(-0.70741957, -0, 0, -0.70744854)
{
  cam_info_sub_ = nh.subscribe("/camera/wrist/info", 1,
                               &ModelToArmNode::cameraInfoCallback, this);

  model_sub_ = nh.subscribe("/model_output", 10,
                            &ModelToArmNode::modelCallback, this);

  result_sub_ = nh.subscribe("/pick_place_result", 10,
                             &ModelToArmNode::pickResultCallback, this);

  pose_pub_ = nh.advertise<std_msgs::Float64MultiArray>(
      "/arm_end_pose_quaternion", 10);

  R_ << 0.0078832, -0.96441021, -0.26429302,
        -0.99996868, -0.00741594, -0.00276565,
         0.00070724,  0.26430655, -0.96443846;
  T_ << 0.10009959, 0.3903029, 0.39465327;

  ROS_INFO("ModelToArmNode 启动，等待 camera_info 和 model_output...");
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
  const std::string &s = msg->data;
  ROS_INFO("收到 /model_output: %s", s.c_str());

  static const std::regex re(R"(\(\s*(\d+)\s*,\s*(\d+)\s*,\s*([\d\.]+)m\s*,\s*([\d\.\-]+)\s*\))");
  std::sregex_iterator it(s.begin(), s.end(), re), end;
  detections_.clear();

  for (; it != end; ++it)
  {
    double u = std::stod((*it)[1].str());
    double v = std::stod((*it)[2].str());
    double Z = std::stod((*it)[3].str());
    double angle_deg = std::stod((*it)[4].str());

    double Xc = (u - cx_) * Z / fx_;
    double Yc = (v - cy_) * Z / fy_;
    Eigen::Vector3d p_cam(Xc, Yc, Z);

    Eigen::Vector3d p_base = R_ * p_cam + T_;

    Eigen::Quaterniond q_fixed = fixed_q_;

    double angle_rad = angle_deg * M_PI / 180.0;
    Eigen::AngleAxisd rot_z(angle_rad, Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond q_rot(rot_z);

    Eigen::Quaterniond q_final = q_fixed * q_rot;

    detections_.push_back({
        p_base.x(), p_base.y(), p_base.z(),
        q_final.x(), q_final.y(), q_final.z(), q_final.w(),
        angle_deg
    });

    ROS_INFO("检测到目标 %zu: [%.3f, %.3f, %.3f], angle=%.1f°",
             detections_.size(), p_base.x(), p_base.y(), p_base.z(), angle_deg);
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
  ROS_INFO("发布第 %zu 个目标到 /arm_end_pose_quaternion (含角度字段)", idx + 1);
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
    ROS_INFO("所有 %zu 个目标已处理完毕", detections_.size());
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
