#!/usr/bin/env python
import rospy
import os

# Import the spawn urdf model service from Gazebo.
from gazebo_msgs.srv import SpawnModel, SpawnModelRequest, GetModelProperties, GetModelPropertiesRequest
from geometry_msgs.msg import Pose
from rospkg import RosPack
from yaml import load


def spawn_unknown_obstacle(obstacle_name, obstacle_model_xml, obstacle_pose):
  # Service proxy to spawn urdf object in Gazebo.
  spawn_obstacle_service = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
  # Create service request.
  spawn_obstacle_request = SpawnModelRequest()
  spawn_obstacle_request.model_name = obstacle_name
  spawn_obstacle_request.model_xml = obstacle_model_xml
  spawn_obstacle_request.robot_namespace = ''
  spawn_obstacle_request.initial_pose = obstacle_pose
  spawn_obstacle_request.reference_frame = 'map'
  # Call spawn model service.
  spawn_obstacle_response = spawn_obstacle_service(spawn_obstacle_request)
  if(not spawn_obstacle_response.success):
    rospy.logerr('Could not spawn unknown obstacle')
    rospy.logerr(spawn_obstacle_response.status_message)
  else:
    rospy.loginfo(spawn_obstacle_response.status_message)

def check_obstacle_existence(obstacle_name):
  # Service proxy to spawn urdf object in Gazebo.
  check_obstacle_service = rospy.ServiceProxy('/gazebo/get_model_properties', GetModelProperties)

  check_obstacle_response = check_obstacle_service(obstacle_name)
  return check_obstacle_response.success


if __name__ == '__main__':

  # Initialize a ROS Node
  rospy.init_node('spawn_unknown_obstacles')
  rospack = RosPack()
  
  # Read parameters
  goal_nr = rospy.get_param("~goal_nr", 1)
  n_osbtacles = rospy.get_param("~obstacles", 1)

  if n_osbtacles == 0:
    rospy.signal_shutdown(0)

  try:
    dict = load(file(rospack.get_path('metacontrol_experiments')+'/yaml/obstacle_positions_G'+ str(goal_nr) + '.yaml', 'r'))
    obstacle_pose_array = []
    for index_obstr in range(1, n_osbtacles+1):

      obst_pose = Pose()
      obst_pose.position.x = dict['O'+str(index_obstr)]['pose']['position']['x']
      obst_pose.position.y = dict['O'+str(index_obstr)]['pose']['position']['y']
      obst_pose.position.z = dict['O'+str(index_obstr)]['pose']['position']['z']
      obst_pose.orientation.x = dict['O'+str(index_obstr)]['pose']['orientation']['x']
      obst_pose.orientation.y = dict['O'+str(index_obstr)]['pose']['orientation']['y']
      obst_pose.orientation.z = dict['O'+str(index_obstr)]['pose']['orientation']['z']
      obst_pose.orientation.w = dict['O'+str(index_obstr)]['pose']['orientation']['w']
      obstacle_pose_array.append(obst_pose)
  except rospy.ROSInterruptException:
    rospy.loginfo("Error loading yaml file")
 
  rospy.loginfo("Adding obstacles.")

  try:

    #Get file name of the object to be spawned
    with open(os.path.join(rospack.get_path('metacontrol_sim'), "urdf/unknown_obstacle/unknown_obstacle.urdf"), "r") as box_file:
      box_xml = box_file.read()
    obs_nr = 1
    for obstacle_pose in obstacle_pose_array:
      if not check_obstacle_existence('unknown_obstacle' + str(obs_nr)):
        # add obstacle
        spawn_unknown_obstacle('unknown_obstacle' + str(obs_nr), box_xml, obstacle_pose)
        obs_nr = obs_nr + 1

  except rospy.ROSInterruptException:
    rospy.loginfo("Interrupt received to stop ROS node.")
