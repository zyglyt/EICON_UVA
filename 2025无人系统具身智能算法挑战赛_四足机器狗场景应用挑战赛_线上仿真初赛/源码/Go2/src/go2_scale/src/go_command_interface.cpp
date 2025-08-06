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
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <vector>
#include <utility>
#include <tf/transform_datatypes.h>
#include <string> 

class ModelOutputListener
{
public:
    ModelOutputListener()
        : ac_("move_base", true), current_goal_idx_(0), is_navigating_(false)
    {
        ROS_INFO("正在等待 move_base action server 启动...");
        if (!ac_.waitForServer(ros::Duration(5.0)))
        {
            ROS_ERROR("move_base action server 启动失败！");
        }
        else
        {
            ROS_INFO("move_base action server 已连接");
        }

        line_control_pub_ = nh_.advertise<std_msgs::String>("/line_follow_control", 10);

        llm_nav_goal_sub_ = nh_.subscribe("/llm_nav_goal", 10, &ModelOutputListener::llmNavGoalCallback, this);
        line_control_sub_ = nh_.subscribe("/line_follow_control", 10, &ModelOutputListener::lineControlCallback, this);

        ROS_INFO("ModelOutputListener 初始化完成，等待 /llm_nav_goal 和 /line_follow_control 指令...");
    }

private:
    void llmNavGoalCallback(const std_msgs::String::ConstPtr& msg)
    {
        std::string json_str = msg->data;
        ROS_INFO("收到来自大模型的导航目标: %s", json_str.c_str());

        try {
            auto getValue = [&](const std::string& key) {
                size_t key_pos = json_str.find(key);
                if (key_pos == std::string::npos) throw std::runtime_error("Key not found: " + key);
                size_t colon_pos = json_str.find(':', key_pos);
                size_t value_start = colon_pos + 1;
                while (isspace(json_str[value_start])) value_start++;
                size_t value_end = json_str.find_first_of(",}", value_start);
                return std::stod(json_str.substr(value_start, value_end - value_start));
            };
            
            double x = getValue("\"x\"");
            double y = getValue("\"y\"");
            double yaw = getValue("\"yaw\"");

            ROS_INFO("解析得到坐标: x=%.2f, y=%.2f, yaw=%.2f", x, y, yaw);

            std_msgs::String cmd;
            cmd.data = "stop";
            line_control_pub_.publish(cmd);
            ROS_INFO("发布巡线停止指令");

            waypoints_.clear();
            waypoints_.push_back(std::make_tuple(x, y, yaw));
            startWaypointNavigation();

        } catch (const std::exception& e) {
            ROS_ERROR("解析导航目标失败: %s. 无效的JSON: %s", e.what(), json_str.c_str());
        }
    }

    void lineControlCallback(const std_msgs::String::ConstPtr& msg)
    {
        std::string action = msg->data;
        if (action == "start")
        {
            ROS_INFO("启动巡线");
            cancelNavigationGoal();
        }
        else if (action == "stop")
        {
            ROS_INFO("停止巡线");
        }
    }

    void startWaypointNavigation()
    {
        if (!ac_.isServerConnected())
        {
            ROS_WARN("move_base action server 未连接，无法开始导航");
            return;
        }

        if (waypoints_.empty())
        {
            ROS_WARN("导航点列表为空，无可执行点");
            return;
        }

        current_goal_idx_ = 0;
        is_navigating_ = true;

        ROS_INFO("多点导航开始，总共 %lu 个目标点", waypoints_.size());
        sendNextGoal();
    }

    void sendNextGoal()
    {
        if (!is_navigating_)
        {
            ROS_INFO("当前不处于导航状态，不发送新目标");
            return;
        }

        if (current_goal_idx_ >= waypoints_.size())
        {
            ROS_INFO("所有导航点已执行完毕，导航序列结束");
            is_navigating_ = false;
            return;
        }

        double x = std::get<0>(waypoints_[current_goal_idx_]);
        double y = std::get<1>(waypoints_[current_goal_idx_]);
        double yaw = std::get<2>(waypoints_[current_goal_idx_]);

        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose.position.x = x;
        goal.target_pose.pose.position.y = y;
        goal.target_pose.pose.position.z = 0.0;

        tf::Quaternion q;
        q.setRPY(0.0, 0.0, yaw);
        goal.target_pose.pose.orientation.x = q.x();
        goal.target_pose.pose.orientation.y = q.y();
        goal.target_pose.pose.orientation.z = q.z();
        goal.target_pose.pose.orientation.w = q.w();

        ROS_INFO("发送第 %zu/%lu 个导航目标： (%.2f, %.2f)，朝向角度：%.2f", current_goal_idx_ + 1, waypoints_.size(), x, y, yaw);

        ac_.sendGoal(goal,
                     boost::bind(&ModelOutputListener::doneCb, this, _1, _2),
                     boost::bind(&ModelOutputListener::activeCb, this),
                     boost::bind(&ModelOutputListener::feedbackCb, this, _1));
    }

    void doneCb(const actionlib::SimpleClientGoalState &state,
                const move_base_msgs::MoveBaseResultConstPtr &result)
    {
        std::string status = state.toString();
        ROS_INFO("目标完成，状态：%s", status.c_str());

        if (is_navigating_ && state == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            current_goal_idx_++;
            sendNextGoal();
        }
        else
        {
            if (is_navigating_)
            {
                ROS_WARN("目标未正常到达，导航中断");
            }
            is_navigating_ = false;
            current_goal_idx_ = 0;
        }
    }

    void activeCb()
    {
        ROS_INFO("目标正在执行...");
    }

    void feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr &feedback)
    {
        double fx = feedback->base_position.pose.position.x;
        double fy = feedback->base_position.pose.position.y;
        ROS_DEBUG("导航反馈 (%.2f, %.2f)", fx, fy);
    }

    void cancelNavigationGoal()
    {
        if (ac_.getState() == actionlib::SimpleClientGoalState::ACTIVE ||
            ac_.getState() == actionlib::SimpleClientGoalState::PENDING)
        {
            ac_.cancelGoal();
        }
        is_navigating_ = false;
        current_goal_idx_ = 0;
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher line_control_pub_;
    ros::Subscriber llm_nav_goal_sub_;  
    ros::Subscriber line_control_sub_;  
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac_;
    std::vector<std::tuple<double, double, double>> waypoints_;
    size_t current_goal_idx_;
    bool is_navigating_;
};

int main(int argc, char **argv)
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "model_output_listener_cpp");
    ModelOutputListener listener;
    ros::spin();
    return 0;
}
