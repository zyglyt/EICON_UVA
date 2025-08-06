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
#include <geometry_msgs/PoseStamped.h>
#include <cmath>
#include <mutex>
#include <thread>
#include <atomic>

class PickAndPlaceNode
{
public:
    explicit PickAndPlaceNode(ros::NodeHandle &nh);

private:
    void endEffectorCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void arrayCallback(const std_msgs::Float64MultiArray::ConstPtr &msg);

    void publishPose(const geometry_msgs::PoseStamped &ps, const std::string &tag);
    void setGripper(float width);                    
    bool waitForPosition(const geometry_msgs::PoseStamped &target, double tol = 0.02);
    void doPick(geometry_msgs::PoseStamped target);
    static double distance(const geometry_msgs::PoseStamped &a,
                           const geometry_msgs::PoseStamped &b);

    ros::NodeHandle nh_;
    ros::Publisher  ee_pub_, grip_pub_, result_pub_;
    ros::Subscriber ee_sub_,  obj_sub_;

    geometry_msgs::PoseStamped home_, cur_;
    std::atomic<bool> busy_{false}, got_pose_{false};
    std::atomic<float> grip_width_{0.06f};           
    const float kOpen = 0.06f;                        
    const float kClose = 0.005f;                    
    std::mutex mtx_;
};

static const auto MAKE_HOME = []{
    geometry_msgs::PoseStamped p;
    p.header.frame_id = "base_link";
    p.pose.position.x = -0.02943485;
    p.pose.position.y =  0.24787514;
    p.pose.position.z =  0.27064234;
    p.pose.orientation.x = -0.1222970;
    p.pose.orientation.y =  0.1224853;
    p.pose.orientation.z = -0.6964485;
    p.pose.orientation.w = -0.6964196;
    return p;
}();

PickAndPlaceNode::PickAndPlaceNode(ros::NodeHandle &nh): nh_(nh)
{
    ee_pub_   = nh_.advertise<geometry_msgs::PoseStamped>("/end_effector/target_pose", 10);
    grip_pub_ = nh_.advertise<std_msgs::Float32>("/gripper/target", 10);
    result_pub_=nh_.advertise<std_msgs::Bool>("/pick_place_result", 1, true);

    ee_sub_  = nh_.subscribe("/end_effector/pose", 10,
                             &PickAndPlaceNode::endEffectorCallback, this);
    obj_sub_ = nh_.subscribe("/arm_end_pose_quaternion", 10,
                             &PickAndPlaceNode::arrayCallback, this);

    ros::Duration(1.0).sleep();
    setGripper(kOpen);
    ROS_INFO("初始化完成，夹爪张开 %.2f cm", kOpen * 100);

    home_ = MAKE_HOME;
    home_.header.stamp = ros::Time::now();
    publishPose(home_, "启动 → 回到 Home");
    waitForPosition(home_);
}

void PickAndPlaceNode::setGripper(float width)
{
    width = std::clamp(width, 0.001f, 0.06f);
    grip_width_ = width;
    std_msgs::Float32 msg;
    msg.data = width;
    grip_pub_.publish(msg);
}

void PickAndPlaceNode::endEffectorCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    std::lock_guard<std::mutex> lock(mtx_);
    cur_ = *msg;
    got_pose_ = true;
}

void PickAndPlaceNode::arrayCallback(const std_msgs::Float64MultiArray::ConstPtr &msg)
{
    if (msg->data.size() < 7 || busy_) return;

    busy_ = true;
    geometry_msgs::PoseStamped target;
    target.header.frame_id = "base_link";
    target.header.stamp = ros::Time::now();
    target.pose.position.x = msg->data[0];
    target.pose.position.y = msg->data[1];
    target.pose.position.z = msg->data[2];
    target.pose.orientation.x = msg->data[3];
    target.pose.orientation.y = msg->data[4];
    target.pose.orientation.z = msg->data[5];
    target.pose.orientation.w = msg->data[6];

    std::thread(&PickAndPlaceNode::doPick, this, target).detach();
}

void PickAndPlaceNode::publishPose(const geometry_msgs::PoseStamped &ps, const std::string &tag)
{
    ee_pub_.publish(ps);
    ROS_INFO("%s → [x=%.3f, y=%.3f, z=%.3f]", tag.c_str(),
             ps.pose.position.x, ps.pose.position.y, ps.pose.position.z);
}

double PickAndPlaceNode::distance(const geometry_msgs::PoseStamped &a,
                                  const geometry_msgs::PoseStamped &b)
{
    double dx = a.pose.position.x - b.pose.position.x;
    double dy = a.pose.position.y - b.pose.position.y;
    double dz = a.pose.position.z - b.pose.position.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

bool PickAndPlaceNode::waitForPosition(const geometry_msgs::PoseStamped &target,
                                       double tol)
{
    ros::Rate rate(50);
    while (ros::ok()) {
        {
            std::lock_guard<std::mutex> lock(mtx_);
            if (got_pose_ && distance(cur_, target) <= tol) return true;
        }
        auto retry = target;
        retry.header.stamp = ros::Time::now();
        ee_pub_.publish(retry);
        rate.sleep();
    }
    return false;
}

void PickAndPlaceNode::doPick(geometry_msgs::PoseStamped target)
{
    home_.header.stamp = ros::Time::now();
    publishPose(home_, "抓取前 → Home");
    waitForPosition(home_);

    geometry_msgs::PoseStamped approach = target;
    approach.pose.position.z += 0.10;
    approach.header.stamp = ros::Time::now();
    publishPose(approach, "Approach");
    waitForPosition(approach);

    target.header.stamp = ros::Time::now();
    publishPose(target, "Descent");
    waitForPosition(target);

    setGripper(kClose);          
    ros::Duration(0.8).sleep();

    home_.header.stamp = ros::Time::now();
    publishPose(home_, "抓取后 → Home");
    waitForPosition(home_);

    std_msgs::Bool res; res.data = true;
    result_pub_.publish(res);
    busy_ = false;
    ROS_INFO("抓取流程完成");
}

int main(int argc, char **argv)
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "pick_and_place_node");
    ros::AsyncSpinner spinner(2); spinner.start();

    ros::NodeHandle nh;
    PickAndPlaceNode node(nh);

    ros::waitForShutdown();
    return 0;
}
