import rospy
from turtlesim.srv import Spawn, SpawnRequest

"""
    需求：向服务器发送请求生成一只乌龟
        话题：/spawn
        消息：turtlesim/Spawn
"""

if __name__ == "__main__":
    rospy.init_node("new_turtle_p")
    client = rospy.ServiceProxy("/spawn", Spawn)

    request = SpawnRequest()
    request.x = 4.5
    request.y = 2.0
    request.theta = -3
    request.name = "turtle2"

    client.wait_for_service()
    try:
        response = client.call(request)
        rospy.loginfo("生成乌龟的名字叫：%s", response.name)
    except Exception as e:
        rospy.logerr("请求异常")