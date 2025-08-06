"""

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

"""
#!/usr/bin/env python3
# coding=utf-8

import rospy
from geometry_msgs.msg import PoseStamped

def arm_pose_callback(msg):
    rospy.loginfo("Received arm_end_pose_quaternion:")
    rospy.loginfo("  Position:   [%.3f, %.3f, %.3f]",
                  msg.pose.position.x,
                  msg.pose.position.y,
                  msg.pose.position.z)
    rospy.loginfo("  Orientation:[%.3f, %.3f, %.3f, %.3f]",
                  msg.pose.orientation.x,
                  msg.pose.orientation.y,
                  msg.pose.orientation.z,
                  msg.pose.orientation.w)

    desired = PoseStamped()
    desired.header.stamp = rospy.Time.now()
    desired.header.frame_id = msg.header.frame_id 

    desired.pose = msg.pose  

    set_pose_pub.publish(desired)
    rospy.loginfo("Published to /Jaka/set_end_effector_pose: Position[%.3f, %.3f, %.3f]",
                  desired.pose.position.x,
                  desired.pose.position.y,
                  desired.pose.position.z)

def main():
    rospy.init_node("arm_end_pose_relay", anonymous=True)

    rospy.Subscriber("arm_end_pose_quaternion", PoseStamped, arm_pose_callback)

    global set_pose_pub
    set_pose_pub = rospy.Publisher("/Jaka/set_end_effector_pose", PoseStamped, queue_size=10)

    rospy.loginfo("Node started: relaying from 'arm_end_pose_quaternion' to '/Jaka/set_end_effector_pose'")
    rospy.spin()

if __name__ == "__main__":
    main()
