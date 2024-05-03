import rospy
from human_position.srv import SetDistanceOpenpose

_DISTANCE = 5.0 

rospy.init_node('set_distance')

print("Wait for the /set_distance service")
rospy.wait_for_service('set_distance')

set_dist = rospy.ServiceProxy('set_distance',SetDistanceOpenpose)
set_dist(_DISTANCE)
print("current distance = ", _DISTANCE)

rospy.spin()
