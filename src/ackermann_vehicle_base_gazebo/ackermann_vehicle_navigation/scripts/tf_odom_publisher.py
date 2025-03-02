#!/usr/bin/env python3

import rospy
import tf2_ros
import geometry_msgs.msg
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry

# 全局变量
last_time = None

def handle_vehicle_pose(msg, vehicle_name):
    global last_time
    
    vehicle_index = msg.name.index(vehicle_name)

    # 获取当前时间
    current_time = rospy.Time.now()

    # 频率控制：确保以每秒 30 次（30 Hz）更新
    if last_time and (current_time - last_time).to_sec() < (1.0 / 30.0):
        return
    last_time = current_time

    br = tf2_ros.TransformBroadcaster()
    tf_broadcaster = tf2_ros.TransformBroadcaster()
    transform2 = geometry_msgs.msg.TransformStamped()
    t = geometry_msgs.msg.TransformStamped()
    
    
    transform2.header.stamp = rospy.Time.now()
    transform2.header.frame_id = "world"
    transform2.child_frame_id = "odom"
    transform2.transform.translation.x = 0
    transform2.transform.translation.y = 0
    transform2.transform.translation.z = 0
    transform2.transform.rotation.x = 0
    transform2.transform.rotation.y = 0
    transform2.transform.rotation.z = 0
    transform2.transform.rotation.w = 1


    tf_broadcaster.sendTransform(transform2)

    t.header.stamp = current_time
    t.header.frame_id = global_frame_id
    t.child_frame_id = vehicle_name
    t.transform.translation.x = msg.pose[vehicle_index].position.x + 1
    t.transform.translation.y = msg.pose[vehicle_index].position.y
    t.transform.translation.z = 0.0
    t.transform.rotation = msg.pose[vehicle_index].orientation

    br.sendTransform(t)
    
    odom_msg = Odometry()
    odom_msg.header.stamp = current_time
    odom_msg.header.frame_id = global_frame_id
    odom_msg.child_frame_id = vehicle_name
    odom_msg.pose.pose.position.x = msg.pose[vehicle_index].position.x + 1
    odom_msg.pose.pose.position.y = msg.pose[vehicle_index].position.y
    odom_msg.pose.pose.position.z = 0
    odom_msg.pose.pose.orientation = msg.pose[vehicle_index].orientation
    odom_msg.twist.twist = msg.twist[vehicle_index]
    odom_publisher.publish(odom_msg)

if __name__ == '__main__':
    rospy.init_node('tf_odom_publisher')
    vehicle_name = rospy.get_param('~vehicle_name', 'ackermann_vehicle')
    global_frame_id = rospy.get_param('~global_frame_id', 'odom')
    odom_publisher = rospy.Publisher('/legOdom', Odometry, queue_size=1)
    
    # 订阅模型状态
    rospy.Subscriber('/gazebo/model_states',
                     ModelStates,
                     handle_vehicle_pose,
                     vehicle_name)

    # 控制频率
    rate = rospy.Rate(30)  # 设置频率为 30 Hz
    while not rospy.is_shutdown():
        rate.sleep()  # 控制循环频率

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
# #!/usr/bin/env python3

# import rospy
# import tf_conversions
# import tf2_ros
# import geometry_msgs.msg
# from gazebo_msgs.msg import ModelStates
# from nav_msgs.msg import Odometry

# def handle_vehicle_pose(msg, vehicle_name):
#     vehicle_index = msg.name.index(vehicle_name)
#     br = tf2_ros.TransformBroadcaster()
#     t = geometry_msgs.msg.TransformStamped()

#     current = rospy.Time.now()
#     t.header.stamp = current
#     t.header.frame_id = global_frame_id
#     t.child_frame_id = vehicle_name
#     t.transform.translation.x = msg.pose[vehicle_index].position.x + 1
#     t.transform.translation.y = msg.pose[vehicle_index].position.y
#     t.transform.translation.z = 0.0
#     t.transform.rotation = msg.pose[vehicle_index].orientation

#     br.sendTransform(t)
    
#     odom_msg = Odometry()
#     odom_msg.header.stamp = rospy.Time.now()
#     odom_msg.header.frame_id = global_frame_id
#     odom_msg.child_frame_id = vehicle_name
#     odom_msg.pose.pose.position.x = msg.pose[vehicle_index].position.x + 1
#     odom_msg.pose.pose.position.y = msg.pose[vehicle_index].position.y
#     odom_msg.pose.pose.position.z = 0
#     odom_msg.pose.pose.orientation = msg.pose[vehicle_index].orientation
#     odom_msg.twist.twist = msg.twist[vehicle_index]
#     odom_publisher.publish(odom_msg)

# if __name__ == '__main__':
#     rospy.init_node('tf_odom_publisher')
#     vehicle_name = rospy.get_param('~vehicle_name', 'ackermann_vehicle')
#     global_frame_id = rospy.get_param('~global_frame_id', 'odom')
#     odom_publisher = rospy.Publisher('/odom', Odometry, queue_size=1)
#     rospy.Subscriber('/gazebo/model_states',
#                      ModelStates,
#                      handle_vehicle_pose,
#                      vehicle_name)

#     try:
#         rospy.spin()
#     except rospy.ROSInterruptException:
#         pass

    