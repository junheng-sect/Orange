import json
import sys
import rospy
from std_srvs.srv import Trigger
assert __import__('os').environ['ROS_MASTER_URI']=='http://127.0.0.1:11311'
rospy.init_node('offline_goal_request',anonymous=True)
goal=[float(v) for v in sys.argv[1:]]
assert len(goal)==2
rospy.wait_for_service('/ego_demo/plan_example',timeout=60)
rospy.set_param('/ego_demo/example_goal',goal)
r=rospy.ServiceProxy('/ego_demo/plan_example',Trigger)()
print(json.dumps({'success':r.success,'message':r.message}))
sys.exit(0 if r.success else 1)
