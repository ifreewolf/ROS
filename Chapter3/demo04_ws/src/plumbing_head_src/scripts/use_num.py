import rospy
import os

import sys

# 设置临时环境变量
# 路径写死，影响了代码的可移植性
# sys.path.insert(0, "/home/fgs/Workstations/ROS/Chapter3/demo04_ws/src/plumbing_head_src/scripts")
# 优化，可以动态获取路径
path = os.path.abspath(".")
sys.path.insert(0, path + "/src/plumbing_head_src/scripts")

import tools

if __name__ == "__main__":
    rospy.init_node("use_num")

    # 异常，AttributeError: module 'tools' has no attribute 'num'
    """
        原因：rosrun 执行时，参考路径是工作空间的路径，在工作空间下无法查找依赖的模块
        解决：可以声明python 的环境变量，当依赖某个模块时，先去指定的环境变量中查找依赖
    """
    path = os.path.abspath(".") # 执行时参考的路径：/home/fgs/Workstations/ROS/Chapter3/demo04_ws
    rospy.loginfo("执行时参考的路径：%s", path)
    rospy.loginfo("num = %d", tools.num)