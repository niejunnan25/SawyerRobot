#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

# 导入 Python 的 datetime 库，并额外导入 timezone 和 timedelta
import datetime

def talker():
    pub = rospy.Publisher('my_test_topic', String, queue_size=10)
    rospy.init_node('my_test_publisher', anonymous=True)
    rate = rospy.Rate(1) 

    # --- 新增：定义北京时间时区 (UTC+8) ---
    # 这是一个固定的偏移量，不需要外部库
    beijing_timezone = datetime.timezone(datetime.timedelta(hours=8))
    # ------------------------------------

    rospy.loginfo("节点已启动，开始向 /my_test_topic 发布消息...")
    
    count = 0
    while not rospy.is_shutdown():
        # --- 修改：获取带时区的北京时间 ---
        # 我们使用 datetime.datetime.now() 并传入定义好的时区
        now_beijing = datetime.datetime.now(beijing_timezone)
        # --------------------------------

        # 将时间格式化为字符串，并加上时区
        time_str = now_beijing.strftime("%Y-%m-%d %H:%M:%S (北京时间)")
        
        # 准备要发送的消息 (加入了北京时间字符串)
        hello_str = "时间: %s | 消息计数: %s" % (time_str, count)
        
        pub.publish(hello_str)
        rospy.loginfo("已发送: [%s]" % hello_str)
        
        count += 1
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        rospy.loginfo("节点已关闭。")
        pass