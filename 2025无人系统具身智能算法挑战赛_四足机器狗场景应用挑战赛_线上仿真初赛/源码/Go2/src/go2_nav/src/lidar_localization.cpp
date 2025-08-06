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
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/RegionOfInterest.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_srvs/Empty.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <angles/angles.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <deque>
#include <tuple>
#include <cmath>

class AMCLCorrector
{
public:
  AMCLCorrector()
    : nh_(),
      tf_buffer_(std::make_shared<tf2_ros::Buffer>()),
      tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_)),
      tf_broadcaster_(std::make_shared<tf2_ros::TransformBroadcaster>()),
      lidar_x_(0.0f), lidar_y_(0.0f), lidar_yaw_(0.0f),
      filtered_x_(0.0), filtered_y_(0.0), filtered_yaw_(0.0),
      first_map_received_(false),
      clear_countdown_(-1), scan_count_(0),
      max_size_(10), base_alpha_(0.5f),
      publish_threshold_dist_(0.1),        
      publish_threshold_yaw_(5.0 * M_PI/180.0),  
      last_pub_x_(0.0), last_pub_y_(0.0), last_pub_yaw_(0.0)
  {
    nh_.param<std::string>("base_frame", base_frame_, "base_link");
    nh_.param<std::string>("odom_frame", odom_frame_, "odom");
    nh_.param<std::string>("laser_frame", laser_frame_, "lidar");
    nh_.param<std::string>("laser_topic", laser_topic_, "scan");
    nh_.param<std::string>("amcl_topic", amcl_topic_, "amcl_pose");

    map_sub_  = nh_.subscribe("map", 1, &AMCLCorrector::mapCallback, this);
    scan_sub_ = nh_.subscribe(laser_topic_, 1, &AMCLCorrector::scanCallback, this);
    amcl_sub_ = nh_.subscribe(amcl_topic_, 1, &AMCLCorrector::amclCallback, this);

    clear_costmaps_client_ = nh_.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");

    timer_ = nh_.createTimer(ros::Duration(0.1), &AMCLCorrector::poseTF, this);

    ROS_INFO("[AMCLCorrector] Node initialized (optimized version)");
  }

private:
  ros::NodeHandle nh_;

  ros::Subscriber map_sub_;
  ros::Subscriber scan_sub_;
  ros::Subscriber amcl_sub_;
  ros::ServiceClient clear_costmaps_client_;

  ros::Timer timer_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  std::string base_frame_, odom_frame_, laser_frame_;
  std::string laser_topic_, amcl_topic_;

  nav_msgs::OccupancyGrid map_msg_;
  cv::Mat map_cropped_;
  cv::Mat map_temp_;
  sensor_msgs::RegionOfInterest map_roi_info_;
  cv::Mat gradient_mask_;

  std::vector<cv::Point2f> scan_points_;

  float lidar_x_, lidar_y_, lidar_yaw_;
  const float deg_to_rad_ = M_PI / 180.0f;
  int clear_countdown_, scan_count_;

  std::deque<std::tuple<float, float, float>> data_queue_;
  const size_t max_size_;
  const float base_alpha_;

  std::deque<std::tuple<float, float, float>> pose_history_;
  static const int HISTORY_SIZE = 8;

  double filtered_x_, filtered_y_, filtered_yaw_;
  const double publish_threshold_dist_;
  const double publish_threshold_yaw_;
  double last_pub_x_, last_pub_y_, last_pub_yaw_;

  bool first_map_received_;

  void amclCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
  {
    if (!tf_buffer_->canTransform(base_frame_, laser_frame_, ros::Time(0), ros::Duration(0.1))) {
      ROS_WARN_THROTTLE(5, "[AMCLCorrector] Missing TF %s->%s", base_frame_.c_str(), laser_frame_.c_str());
      return;
    }
    geometry_msgs::TransformStamped tf_b2l;
    try {
      tf_b2l = tf_buffer_->lookupTransform(base_frame_, laser_frame_, ros::Time(0));
    } catch (tf2::TransformException &ex) {
      ROS_WARN_THROTTLE(5, "[AMCLCorrector] TF lookup failed: %s", ex.what());
      return;
    }

    geometry_msgs::PoseStamped base_pose, laser_pose;
    base_pose.header = msg->header;
    base_pose.pose   = msg->pose.pose;
    tf2::doTransform(base_pose, laser_pose, tf_b2l);

    double x = laser_pose.pose.position.x;
    double y = laser_pose.pose.position.y;

    tf2::Quaternion q(
      laser_pose.pose.orientation.x,
      laser_pose.pose.orientation.y,
      laser_pose.pose.orientation.z,
      laser_pose.pose.orientation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    lidar_x_ = static_cast<float>((x - map_msg_.info.origin.position.x) / map_msg_.info.resolution
                                   - map_roi_info_.x_offset);
    lidar_y_ = static_cast<float>((y - map_msg_.info.origin.position.y) / map_msg_.info.resolution
                                   - map_roi_info_.y_offset);
    lidar_yaw_ = -static_cast<float>(yaw);

    pose_history_.clear();
    filtered_x_   = lidar_x_;
    filtered_y_   = lidar_y_;
    filtered_yaw_ = lidar_yaw_;
    last_pub_x_   = lidar_x_;
    last_pub_y_   = lidar_y_;
    last_pub_yaw_ = lidar_yaw_;

    clear_countdown_ = 30;
  }

  void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
  {
    map_msg_ = *msg;
    cropMap();
    processMap();
  }

  void scanCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
  {
    scan_points_.clear();
    const size_t decimation = 4;
    for (size_t i = 0; i < msg->ranges.size(); i += decimation) {
      double ang = msg->angle_min + i * msg->angle_increment;
      double r   = msg->ranges[i];
      if (r >= msg->range_min && r <= msg->range_max) {
        float x = static_cast<float>(r * cos(ang) / map_msg_.info.resolution);
        float y = static_cast<float>(-r * sin(ang) / map_msg_.info.resolution);
        scan_points_.emplace_back(x, y);
      }
    }
    if (scan_count_ == 0) {
      scan_count_++;
    }
    updateLidarPose(3);

    if (clear_countdown_ > -1) {
      clear_countdown_--;
    }
    if (clear_countdown_ == 0) {
      std_srvs::Empty srv;
      if (!clear_costmaps_client_.waitForExistence(ros::Duration(0.5))) {
        ROS_WARN("[AMCLCorrector] clear_costmap unavailable");
      } else {
        clear_costmaps_client_.call(srv);
      }
    }
  }

  void updateLidarPose(int max_iterations)
  {
    int iter = 0;
    while (ros::ok() && iter < max_iterations) {
      if (map_cropped_.empty() || map_temp_.empty()) break;

      std::vector<float> yaw_offsets;
      for (float d = -2.0f * deg_to_rad_; d <= 2.0f * deg_to_rad_; d += 0.5f * deg_to_rad_) {
        yaw_offsets.push_back(d);
      }
      std::vector<cv::Point2f> offsets;
      for (int dx = -2; dx <= 2; ++dx) {
        for (int dy = -2; dy <= 2; ++dy) {
          offsets.emplace_back(static_cast<float>(dx), static_cast<float>(dy));
        }
      }

      std::vector<std::vector<cv::Point2f>> all_pts;
      all_pts.reserve(yaw_offsets.size());
      for (float dyaw : yaw_offsets) {
        std::vector<cv::Point2f> pts;
        float ang = lidar_yaw_ + dyaw;
        float c = cos(ang), s = sin(ang);
        for (auto &p : scan_points_) {
          float rx = p.x * c - p.y * s;
          float ry = p.x * s + p.y * c;
          pts.emplace_back(rx + lidar_x_, lidar_y_ - ry);
        }
        all_pts.push_back(std::move(pts));
      }

      int best_score = -1;
      float best_dx = 0, best_dy = 0, best_dyaw = 0;
      for (size_t i = 0; i < offsets.size(); ++i) {
        for (size_t j = 0; j < all_pts.size(); ++j) {
          int sum = 0;
          for (auto &pt : all_pts[j]) {
            int cx = static_cast<int>(pt.x + offsets[i].x);
            int cy = static_cast<int>(pt.y + offsets[i].y);
            if (cx >= 0 && cx < map_temp_.cols && cy >= 0 && cy < map_temp_.rows) {
              sum += map_temp_.at<uchar>(cy, cx);
            }
          }
          if (sum > best_score) {
            best_score = sum;
            best_dx    = offsets[i].x;
            best_dy    = offsets[i].y;
            best_dyaw  = yaw_offsets[j];
          }
        }
      }

      float new_x   = lidar_x_ + best_dx;
      float new_y   = lidar_y_ + best_dy;
      float new_yaw = lidar_yaw_ + best_dyaw;

      if (fabs(best_dx) < 0.3f && fabs(best_dy) < 0.3f && fabs(best_dyaw) < 0.5f * deg_to_rad_) {
        lidar_x_   = new_x;
        lidar_y_   = new_y;
        lidar_yaw_ = new_yaw;
        break;
      }

      float alpha = base_alpha_;
      if (fabs(best_dyaw) > 2.0f * deg_to_rad_ ||
          fabs(best_dx) > 1.5f ||
          fabs(best_dy) > 1.5f) {
        alpha = 0.2f;
      }
      if (fabs(best_dx) > 3.0f || fabs(best_dyaw) > 5.0f * deg_to_rad_) {
        lidar_x_   = new_x;
        lidar_y_   = new_y;
        lidar_yaw_ = new_yaw;
      } else {
        lidar_x_   = alpha * new_x   + (1 - alpha) * lidar_x_;
        lidar_y_   = alpha * new_y   + (1 - alpha) * lidar_y_;
        lidar_yaw_ = alpha * new_yaw + (1 - alpha) * lidar_yaw_;
      }

      if (checkConverged(lidar_x_, lidar_y_, lidar_yaw_)) {
        break;
      }
      ++iter;
    }

    if (checkConverged(lidar_x_, lidar_y_, lidar_yaw_)) {
      if (pose_history_.size() >= HISTORY_SIZE) {
        pose_history_.pop_front();
      }
      pose_history_.emplace_back(lidar_x_, lidar_y_, lidar_yaw_);
    }
  }

  void cropMap()
  {
    const auto &info = map_msg_.info;
    int xMin = info.width / 2, xMax = info.width / 2;
    int yMin = info.height / 2, yMax = info.height / 2;
    bool firstObs = true;

    cv::Mat raw(info.height, info.width, CV_8UC1, cv::Scalar(128));
    for (int y = 0; y < (int)info.height; ++y) {
      for (int x = 0; x < (int)info.width; ++x) {
        int8_t occ = map_msg_.data[y * info.width + x];
        raw.at<uchar>(y, x) = static_cast<uchar>(occ);
        if (occ == 100) {
          if (firstObs) {
            xMin = xMax = x;
            yMin = yMax = y;
            firstObs = false;
          } else {
            xMin = std::min(xMin, x);
            xMax = std::max(xMax, x);
            yMin = std::min(yMin, y);
            yMax = std::max(yMax, y);
          }
        }
      }
    }

    int cx = (xMin + xMax) / 2;
    int cy = (yMin + yMax) / 2;
    int halfW = abs(xMax - xMin) / 2 + 50;
    int halfH = abs(yMax - yMin) / 2 + 50;

    int ox = std::max(0, cx - halfW);
    int oy = std::max(0, cy - halfH);
    int w  = std::min((int)info.width  - ox, halfW * 2);
    int h  = std::min((int)info.height - oy, halfH * 2);

    cv::Rect roi(ox, oy, w, h);
    map_cropped_ = raw(roi).clone();

    map_roi_info_.x_offset = ox;
    map_roi_info_.y_offset = oy;
    map_roi_info_.width    = w;
    map_roi_info_.height   = h;

    if (!first_map_received_) {
      first_map_received_ = true;
      ROS_INFO("[AMCLCorrector] First map received, ready for AMCL updates");
    }
  }

  cv::Mat createGradientMask(int size)
  {
    cv::Mat mask(size, size, CV_8UC1);
    int center = size / 2;
    for (int y = 0; y < size; ++y) {
      for (int x = 0; x < size; ++x) {
        double d = sqrt((x-center)*(x-center) + (y-center)*(y-center));
        mask.at<uchar>(y, x) = cv::saturate_cast<uchar>(255 * std::max(0.0, 1.0 - d / center));
      }
    }
    return mask;
  }

  void processMap()
  {
    if (map_cropped_.empty()) return;
    if (gradient_mask_.empty()) {
      gradient_mask_ = createGradientMask(51);
    }
    map_temp_ = cv::Mat::zeros(map_cropped_.size(), CV_8UC1);
    int rows = map_cropped_.rows;
    int cols = map_cropped_.cols;
    int gm   = gradient_mask_.rows / 2;

    for (int y = 0; y < rows; ++y) {
      for (int x = 0; x < cols; ++x) {
        if (map_cropped_.at<uchar>(y, x) == 100) {
          int l = std::max(0, x - gm);
          int t = std::max(0, y - gm);
          int r = std::min(cols - 1, x + gm);
          int b = std::min(rows - 1, y + gm);
          cv::Rect region(l, t, r - l + 1, b - t + 1);
          cv::Mat dst = map_temp_(region);
          int ml = gm - (x - l);
          int mt = gm - (y - t);
          cv::Rect mask_roi(ml, mt, region.width, region.height);
          cv::Mat src = gradient_mask_(mask_roi);
          cv::max(dst, src, dst);
        }
      }
    }
  }

  bool checkConverged(float x, float y, float yaw)
  {
    if (x == 0 && y == 0 && yaw == 0) {
      data_queue_.clear();
      return true;
    }
    data_queue_.emplace_back(x, y, yaw);
    if (data_queue_.size() > max_size_) {
      data_queue_.pop_front();
    }
    if (data_queue_.size() == max_size_) {
      auto first = data_queue_.front();
      auto last  = data_queue_.back();
      float dx   = fabs(std::get<0>(last) - std::get<0>(first));
      float dy   = fabs(std::get<1>(last) - std::get<1>(first));
      float dyaw = fabs(std::get<2>(last) - std::get<2>(first));
      if (dx < 2.0f && dy < 2.0f && dyaw < 2.0f * deg_to_rad_) {
        data_queue_.clear();
        return true;
      }
    }
    return false;
  }

  void poseTF(const ros::TimerEvent &)
  {
    if (scan_count_ == 0 || map_cropped_.empty() || map_msg_.data.empty()) return;
    if (pose_history_.empty()) return;

    double sum_x = 0, sum_y = 0, sum_yaw = 0;
    for (auto &t : pose_history_) {
      sum_x   += std::get<0>(t);
      sum_y   += std::get<1>(t);
      sum_yaw += std::get<2>(t);
    }
    filtered_x_   = sum_x   / pose_history_.size();
    filtered_y_   = sum_y   / pose_history_.size();
    filtered_yaw_ = sum_yaw / pose_history_.size();

    double dx    = filtered_x_ - last_pub_x_;
    double dy    = filtered_y_ - last_pub_y_;
    double dyaw  = angles::shortest_angular_distance(filtered_yaw_, last_pub_yaw_);

    if (std::sqrt(dx*dx + dy*dy) < publish_threshold_dist_ &&
        fabs(dyaw) < publish_threshold_yaw_) {
      return;
    }

    double xm    = (filtered_x_ + map_roi_info_.x_offset) * map_msg_.info.resolution
                   + map_msg_.info.origin.position.x;
    double ym    = (filtered_y_ + map_roi_info_.y_offset) * map_msg_.info.resolution
                   + map_msg_.info.origin.position.y;
    double yaw_ros = -filtered_yaw_;

    tf2::Quaternion q;
    q.setRPY(0, 0, yaw_ros);

    geometry_msgs::TransformStamped odom2laser;
    try {
      odom2laser = tf_buffer_->lookupTransform(odom_frame_, laser_frame_, ros::Time(0), ros::Duration(0.1));
    } catch (tf2::TransformException &ex) {
      ROS_WARN_THROTTLE(5, "[AMCLCorrector] TF lookup in poseTF failed: %s", ex.what());
      return;
    }

    tf2::Transform map2base, odom2base;
    map2base.setOrigin(tf2::Vector3(xm, ym, 0.0));
    map2base.setRotation(q);
    tf2::fromMsg(odom2laser.transform, odom2base);

    tf2::Vector3 origin = odom2base.getOrigin(); origin.setZ(0.0);
    odom2base.setOrigin(origin);
    double roll, pitch, yaw;
    tf2::Matrix3x3(odom2base.getRotation()).getRPY(roll, pitch, yaw);
    tf2::Quaternion zrot; zrot.setRPY(0, 0, yaw);
    odom2base.setRotation(zrot);

    tf2::Transform map2odom = map2base * odom2base.inverse();

    geometry_msgs::TransformStamped out;
    out.header.stamp    = ros::Time::now();
    out.header.frame_id = "map";
    out.child_frame_id  = odom_frame_;
    out.transform       = tf2::toMsg(map2odom);
    tf_broadcaster_->sendTransform(out);

    last_pub_x_   = filtered_x_;
    last_pub_y_   = filtered_y_;
    last_pub_yaw_ = filtered_yaw_;
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "amcl_corrector");
  AMCLCorrector node;
  ros::spin();
  return 0;
}
