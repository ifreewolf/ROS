#! /usr/bin/env python
"""
    参数服务器操作之 新增与修改(二者API一样)——Python实现：
"""

import rospy

if __name__ == "__main__":
    rospy.init_node("set_update_param_p")

    # 设置各种类型
    rospy.set_param("p_int", 10)
    rospy.set_param("p_double", 3.14)
    rospy.set_param("p_bool", True)
    rospy.set_param("p_string", "hello python")
    rospy.set_param("p_list", ["hello", "haha", "xixi"])
    rospy.set_param("p_dict", {"name":"hulu", "age":18})

    # 修改
    rospy.set_param("p_int", 100)