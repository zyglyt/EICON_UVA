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
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>
#include <mutex>
#include <thread>

class PickAndPlaceNode
{
public:
    PickAndPlaceNode(ros::NodeHandle &nh);

private:
    void sideCallback(const std_msgs::String::ConstPtr &msg);
    void endEffectorCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void arrayCallback(const std_msgs::Float64MultiArray::ConstPtr &msg);
    void gripperCapturedCallback(const std_msgs::Bool::ConstPtr &msg);

    void publishPose(const geometry_msgs::PoseStamped &ps, const std::string &tag);
    void controlGripperIncremental(bool closed, const std::string &tag);
    void openGripperDirectly(const std::string &tag);
    void waitForPosition(const geometry_msgs::PoseStamped &target);
    void waitForGripper();
    void doPickPlace(geometry_msgs::PoseStamped target);
    double calcDistance(const geometry_msgs::PoseStamped &a, const geometry_msgs::PoseStamped &b);

    ros::NodeHandle nh_;
    ros::Publisher ee_pose_pub_, gripper_pub_, result_pub_;
    ros::Subscriber side_sub_, end_pose_sub_, object_sub_, gripper_sub_;

    geometry_msgs::PoseStamped place_pose_, init_pose_, current_pose_;
    bool gripper_closed_, busy_, got_end_pose_, pick_success_;
    std::string frame_id_, conveyor_side_;
    double approach_height_, left_place_x_, right_place_x_;
    double pos_tolerance_, ori_tolerance_, max_step_distance_;
    float gripper_value_, gripper_step_;
    std::mutex mutex_, pose_mutex_;
};

PickAndPlaceNode::PickAndPlaceNode(ros::NodeHandle &nh)
    : nh_(nh), busy_(false), got_end_pose_(false),
      gripper_closed_(false), pick_success_(false),
      gripper_value_(0.0f), gripper_step_(0.001f),
      conveyor_side_("left")
{
    ros::NodeHandle pnh("~");
    pnh.param<std::string>("frame_id", frame_id_, "base_link");
    pnh.param("approach_height", approach_height_, 0.1);
    pnh.param("pos_tolerance", pos_tolerance_, 0.06);
    pnh.param("ori_tolerance", ori_tolerance_, 0.2);
    pnh.param("max_step_distance", max_step_distance_, 0.5);
    pnh.param("place_pose_x", left_place_x_, 0.60);
    pnh.param("place_pose_y", place_pose_.pose.position.y, 0.5);
    pnh.param("place_pose_z", place_pose_.pose.position.z, 2.67);
    pnh.param("place_pose_qx", place_pose_.pose.orientation.x, 0.00);
    pnh.param("place_pose_qy", place_pose_.pose.orientation.y, 1.2);
    pnh.param("place_pose_qz", place_pose_.pose.orientation.z, 1.2);
    pnh.param("place_pose_qw", place_pose_.pose.orientation.w, 0.0);
    place_pose_.header.frame_id = frame_id_;
    right_place_x_ = -left_place_x_;

    init_pose_.header.frame_id = frame_id_;
    init_pose_.pose.position.x = 0.0662943895;
    init_pose_.pose.position.y = 0.0862721270;
    init_pose_.pose.position.z = 2.7503792615;
    init_pose_.pose.orientation.x = 0.0146265891;
    init_pose_.pose.orientation.y = -0.0000924757;
    init_pose_.pose.orientation.z = 0.9998736759;
    init_pose_.pose.orientation.w = -0.0062198575;

    ee_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/Jaka/set_end_effector_pose", 10);
    gripper_pub_ = nh_.advertise<std_msgs::Float32>("/Jaka/set_gripper_value", 10);
    result_pub_ = nh_.advertise<std_msgs::Bool>(
        "/pick_place_result", 1,
        /*latched=*/true);

    side_sub_ = nh_.subscribe("/conveyor_side", 10, &PickAndPlaceNode::sideCallback, this);
    object_sub_ = nh_.subscribe("/arm_end_pose_quaternion", 10, &PickAndPlaceNode::arrayCallback, this);
    end_pose_sub_ = nh_.subscribe("/Jaka/get_end_effector_pose", 10, &PickAndPlaceNode::endEffectorCallback, this);
    gripper_sub_ = nh_.subscribe("/Jaka/gripper_is_captured", 10, &PickAndPlaceNode::gripperCapturedCallback, this);

    ros::Duration(1.0).sleep();
    openGripperDirectly("夹爪初始化");
    ROS_INFO("[PickAndPlaceNode] 夹爪初始化完成，已张开");
    ROS_INFO("[PickAndPlaceNode] 初始化完成");
}

void PickAndPlaceNode::sideCallback(const std_msgs::String::ConstPtr &msg)
{
    std::lock_guard<std::mutex> lock(mutex_);
    conveyor_side_ = msg->data;
    ROS_INFO("收到 /conveyor_side: %s", conveyor_side_.c_str());
}

void PickAndPlaceNode::endEffectorCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    std::lock_guard<std::mutex> lock(pose_mutex_);
    current_pose_ = *msg;
    got_end_pose_ = true;
}

void PickAndPlaceNode::gripperCapturedCallback(const std_msgs::Bool::ConstPtr &msg)
{
    gripper_closed_ = msg->data;
}

void PickAndPlaceNode::arrayCallback(const std_msgs::Float64MultiArray::ConstPtr &msg)
{
    if (msg->data.size() < 7)
    {
        ROS_WARN_THROTTLE(5.0, "数据长度 %lu < 7，跳过", msg->data.size());
        return;
    }
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (busy_)
            return;
        busy_ = true;
        pick_success_ = false;
    }

    geometry_msgs::PoseStamped target;
    target.header.stamp = ros::Time::now();
    target.header.frame_id = frame_id_;
    target.pose.position.x = msg->data[0];
    target.pose.position.y = msg->data[1];
    target.pose.position.z = msg->data[2];
    target.pose.orientation.x = msg->data[3];
    target.pose.orientation.y = msg->data[4];
    target.pose.orientation.z = msg->data[5];
    target.pose.orientation.w = msg->data[6];

    std::thread(&PickAndPlaceNode::doPickPlace, this, target).detach();
}

void PickAndPlaceNode::publishPose(const geometry_msgs::PoseStamped &ps, const std::string &tag)
{
    ee_pose_pub_.publish(ps);
    ROS_INFO("%s → [x=%.3f, y=%.3f, z=%.3f]", tag.c_str(),
             ps.pose.position.x, ps.pose.position.y, ps.pose.position.z);
}

void PickAndPlaceNode::controlGripperIncremental(bool closed, const std::string &tag)
{
    gripper_value_ = closed
                         ? std::min(1.0f, gripper_value_ + gripper_step_)
                         : std::max(0.0f, gripper_value_ - gripper_step_);
    std_msgs::Float32 cmd;
    cmd.data = gripper_value_;
    gripper_pub_.publish(cmd);
    ROS_INFO("%s → 夹爪值: %.4f", tag.c_str(), gripper_value_);
}

void PickAndPlaceNode::openGripperDirectly(const std::string &tag)
{
    gripper_value_ = 0.001f;
    std_msgs::Float32 cmd;
    cmd.data = gripper_value_;
    gripper_pub_.publish(cmd);
    ROS_INFO("%s → 夹爪直接设置为: %.4f", tag.c_str(), gripper_value_);
}

double PickAndPlaceNode::calcDistance(const geometry_msgs::PoseStamped &a,
                                      const geometry_msgs::PoseStamped &b)
{
    double dx = a.pose.position.x - b.pose.position.x;
    double dy = a.pose.position.y - b.pose.position.y;
    double dz = a.pose.position.z - b.pose.position.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

void PickAndPlaceNode::waitForPosition(const geometry_msgs::PoseStamped &target)
{
    ros::Rate rate(20);
    while (ros::ok())
    {
        geometry_msgs::PoseStamped cmd;
        {
            std::lock_guard<std::mutex> lock(pose_mutex_);
            if (!got_end_pose_)
            {
                ros::spinOnce();
                rate.sleep();
                continue;
            }

            double dist = calcDistance(current_pose_, target);
            tf2::Quaternion qc(current_pose_.pose.orientation.x,
                               current_pose_.pose.orientation.y,
                               current_pose_.pose.orientation.z,
                               current_pose_.pose.orientation.w);
            tf2::Quaternion qt(target.pose.orientation.x,
                               target.pose.orientation.y,
                               target.pose.orientation.z,
                               target.pose.orientation.w);
            double angle = qc.angleShortestPath(qt);
            if (dist <= pos_tolerance_ && angle <= ori_tolerance_)
            {
                ROS_INFO("目标到达 (误差: %.3fm, %.2f°)", dist, angle * 180.0 / M_PI);
                break;
            }

            cmd = target;
            if (dist > max_step_distance_)
            {
                double ratio = max_step_distance_ / dist;
                cmd.pose.position.x = current_pose_.pose.position.x + (target.pose.position.x - current_pose_.pose.position.x) * ratio;
                cmd.pose.position.y = current_pose_.pose.position.y + (target.pose.position.y - current_pose_.pose.position.y) * ratio;
                cmd.pose.position.z = current_pose_.pose.position.z + (target.pose.position.z - current_pose_.pose.position.z) * ratio;
            }
        }
        cmd.header.stamp = ros::Time::now();
        cmd.header.frame_id = frame_id_;
        ee_pose_pub_.publish(cmd);
        ros::spinOnce();
        rate.sleep();
    }
}

void PickAndPlaceNode::waitForGripper()
{
    ros::Rate rate(10);
    while (ros::ok())
    {
        if (gripper_closed_)
        {
            ROS_INFO("夹爪确认闭合");
            break;
        }
        controlGripperIncremental(true, "夹爪闭合");
        ros::spinOnce();
        rate.sleep();
    }
}

void PickAndPlaceNode::doPickPlace(geometry_msgs::PoseStamped target)
{
    geometry_msgs::PoseStamped approach = target;
    approach.pose.position.x += 0.03;
    approach.pose.position.y += 0.0075;
    approach.pose.position.z += approach_height_;
    approach.header = target.header;
    publishPose(approach, "抬升至目标上方");
    waitForPosition(approach);

    target.header.stamp = ros::Time::now();
    target.pose.position.x += 0.01;
    target.pose.position.y += 0.0075;
    publishPose(target, "下移至抓取点");
    waitForPosition(target);
    ros::Duration(5.0).sleep();

    {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        double dist = calcDistance(current_pose_, target);
        tf2::Quaternion qc(current_pose_.pose.orientation.x,
                           current_pose_.pose.orientation.y,
                           current_pose_.pose.orientation.z,
                           current_pose_.pose.orientation.w);
        tf2::Quaternion qt(target.pose.orientation.x,
                           target.pose.orientation.y,
                           target.pose.orientation.z,
                           target.pose.orientation.w);
        double angle = qc.angleShortestPath(qt);
        if (dist <= pos_tolerance_ && angle <= ori_tolerance_)
        {
            controlGripperIncremental(true, "夹爪闭合");
            waitForGripper();
        }
        else
        {
            ROS_WARN("未到位 (%.3fm, %.2f°)，跳过闭合", dist, angle * 180.0 / M_PI);
        }
    }

    init_pose_.header.stamp = ros::Time::now();
    publishPose(init_pose_, "回到初始位姿");
    waitForPosition(init_pose_);

    place_pose_.pose.position.x = (conveyor_side_ == "right" ? right_place_x_ : left_place_x_);
    place_pose_.header.stamp = ros::Time::now();
    publishPose(place_pose_, "移动至放置位姿1");
    waitForPosition(place_pose_);

    geometry_msgs::PoseStamped set_pose = place_pose_;
    set_pose.header.stamp = ros::Time::now();
    set_pose.pose.position.z -= 0.2;
    publishPose(set_pose, "移动至放置位姿2");
    waitForPosition(set_pose);
    ros::Duration(3.0).sleep();

    openGripperDirectly("夹爪打开");
    ros::Duration(2.0).sleep();

    geometry_msgs::PoseStamped get_pose = set_pose;
    get_pose.header.stamp = ros::Time::now();
    get_pose.pose.position.z += 0.2;
    publishPose(get_pose, "移动至放置位姿3");
    waitForPosition(get_pose);
    ros::Duration(3.0).sleep();

    init_pose_.header.stamp = ros::Time::now();
    publishPose(init_pose_, "回到初始位姿");
    waitForPosition(init_pose_);

    std_msgs::Bool res_msg;
    {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        double dist = calcDistance(current_pose_, init_pose_);
        tf2::Quaternion qc(current_pose_.pose.orientation.x,
                           current_pose_.pose.orientation.y,
                           current_pose_.pose.orientation.z,
                           current_pose_.pose.orientation.w);
        tf2::Quaternion qt(init_pose_.pose.orientation.x,
                           init_pose_.pose.orientation.y,
                           init_pose_.pose.orientation.z,
                           init_pose_.pose.orientation.w);
        double angle = qc.angleShortestPath(qt);
        pick_success_ = (dist <= pos_tolerance_ && angle <= ori_tolerance_);
    }
    res_msg.data = pick_success_;
    result_pub_.publish(res_msg);
    ROS_INFO("本次抓放流程 %s", pick_success_ ? "成功" : "失败");

    {
        std::lock_guard<std::mutex> lock(mutex_);
        busy_ = false;
    }
    ROS_INFO("抓放任务完成");
}

int main(int argc, char **argv)
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "pick_and_place_node");
    ros::NodeHandle nh;
    PickAndPlaceNode node(nh);
    ros::spin();
    return 0;
}
