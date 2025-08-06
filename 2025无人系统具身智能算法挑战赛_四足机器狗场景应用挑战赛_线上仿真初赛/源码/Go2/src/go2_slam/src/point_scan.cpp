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
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <limits>
#include <vector>
#include <deque>
#include <algorithm>
#include <cmath>

class PointCloudToLaserScan
{
public:
    PointCloudToLaserScan() : nh_("~")
    {
        nh_.param("use_height_filter", use_height_filter_, true);
        nh_.param("min_height", min_height_, -0.2);
        nh_.param("max_height", max_height_, 2.0);
        nh_.param("angle_min", angle_min_, -M_PI);
        nh_.param("angle_max", angle_max_, M_PI);
        nh_.param("angle_inc", angle_inc_, M_PI / 180.0);
        nh_.param("range_min", range_min_, 0.1);
        nh_.param("range_max", range_max_, 30.0);
        nh_.param("accumulate_frames", acc_frames_, 5);
        nh_.param("interp_radius", interp_radius_, 1);

        bucket_size_ = std::ceil((angle_max_ - angle_min_) / angle_inc_);
        current_ranges_.assign(bucket_size_, std::numeric_limits<float>::infinity());
        combined_ranges_.assign(bucket_size_, std::numeric_limits<float>::infinity());

        sub_ = nh_.subscribe("/lidar/points", 10, &PointCloudToLaserScan::cloudCallback, this);
        pub_ = nh_.advertise<sensor_msgs::LaserScan>("/scan", 10);
    }

private:
    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
    {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(*cloud_msg, cloud);

        std::fill(current_ranges_.begin(), current_ranges_.end(),
                  std::numeric_limits<float>::infinity());

        for (const auto &pt : cloud.points)
        {
            if (use_height_filter_)
            {
                if (pt.z < min_height_ || pt.z > max_height_)
                    continue;
            }
            float r = std::hypot(pt.x, pt.y);
            if (r < range_min_ || r > range_max_)
                continue;
            float ang = std::atan2(pt.y, pt.x);
            if (ang < angle_min_ || ang > angle_max_)
                continue;

            int idx = static_cast<int>((ang - angle_min_) / angle_inc_);
            if (idx < 0 || idx >= bucket_size_)
                continue;

            current_ranges_[idx] = std::min(current_ranges_[idx], r);
        }

        history_.push_back(current_ranges_);
        if ((int)history_.size() > acc_frames_)
            history_.pop_front();

        for (int i = 0; i < bucket_size_; ++i)
        {
            float m = std::numeric_limits<float>::infinity();
            for (auto &frame : history_)
                m = std::min(m, frame[i]);
            combined_ranges_[i] = m;
        }

        for (int i = 0; i < bucket_size_; ++i)
        {
            if (std::isinf(combined_ranges_[i]))
            {
                float best = std::numeric_limits<float>::infinity();
                for (int d = 1; d <= interp_radius_; ++d)
                {
                    int l = (i - d + bucket_size_) % bucket_size_;
                    int r = (i + d) % bucket_size_;
                    best = std::min(best, combined_ranges_[l]);
                    best = std::min(best, combined_ranges_[r]);
                }
                if (!std::isinf(best))
                    combined_ranges_[i] = best;
            }

        }

        sensor_msgs::LaserScan scan;
        scan.header.frame_id = cloud_msg->header.frame_id;
        scan.header.stamp = ros::Time::now();
        scan.angle_min = angle_min_;
        scan.angle_max = angle_max_;
        scan.angle_increment = angle_inc_;
        scan.time_increment = 0.0;
        scan.scan_time = 0.1;
        scan.range_min = range_min_;
        scan.range_max = range_max_;
        scan.ranges = combined_ranges_;

        pub_.publish(scan);
    }

    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;

    bool use_height_filter_;
    double min_height_, max_height_;
    double angle_min_, angle_max_, angle_inc_;
    double range_min_, range_max_;
    int acc_frames_, interp_radius_;
    int bucket_size_;

    std::vector<float> current_ranges_;
    std::vector<float> combined_ranges_;
    std::deque<std::vector<float>> history_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pointcloud_to_laserscan");
    PointCloudToLaserScan node;
    ros::spin();
    return 0;
}