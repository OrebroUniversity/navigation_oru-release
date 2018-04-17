#!/usr/bin/env python
from gazebo_msgs.srv import DeleteModel, GetWorldProperties
from std_srvs.srv import Empty
import rospy, time

# Initialize node
rospy.init_node("delete_humans_from_gazebo")

# Create service clients
rospy.wait_for_service('/gazebo/delete_model')
rospy.wait_for_service('/gazebo/get_world_properties')
rospy.wait_for_service('/gazebo/pause_physics')
rospy.wait_for_service('/gazebo/unpause_physics')

deleteModel = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
getWorldProperties = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
pausePhysics = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
unpausePhysics = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)

rospy.logwarn("Deleting human actors from the scene!")
#rospy.loginfo("Pausing physics")
#pausePhysics()

# Get available models
worldProperties = getWorldProperties()
for name in worldProperties.model_names:
    if name.startswith("actor") or name.startswith("person_standing") or name.startswith("person_walking"):
        res = deleteModel(name)
        if res.success:
            rospy.loginfo("Deleted model " + name)
        else:
            rospy.logerr("Failed to delete model " + name)

#time.sleep(2.0)
#rospy.loginfo("Unpausing physics")
#unpausePhysics()
