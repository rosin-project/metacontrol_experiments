#!/usr/bin/env python
import rospy
import os

# Import the spawn urdf model service from Gazebo.
from gazebo_msgs.srv import SpawnModel, SpawnModelRequest
from gazebo_msgs.srv import GetModelProperties, GetModelPropertiesRequest
from geometry_msgs.msg import Pose
from rospkg import RosPack
from yaml import load


def spawn_unknown_obstacle(obstacle_name, obstacle_model_xml, obstacle_pose):
    # Service proxy to spawn urdf object in Gazebo.
    spawn_obstacle_service =  \
        rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
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
    check_obstacle_service = rospy.ServiceProxy('/gazebo/get_model_properties',
                                                GetModelProperties)
    try:
        check_obstacle_response = check_obstacle_service(obstacle_name)
    except rospy.ServiceException as e:
        rospy.loginfo("Interrupt received when calling service: %s", str(e))
        return False
    else:
        return check_obstacle_response.success


def add_obstacles_main():
    # Initialize a ROS Node
    rospy.init_node('spawn_unknown_obstacles')
    ros_pack = RosPack()
    rospy.sleep(1.0)

    # Read parameters
    clutterness = rospy.get_param("~n_obstacles", 0)

    rospy.loginfo("\n\n\n\t Clutterness: {}\n\n".format(clutterness))

    if (clutterness == 1):
        n_obstacles = 6
    elif (clutterness == 2):
        n_obstacles = 14
    elif (clutterness == 3):
        n_obstacles = 20
    else:
        n_obstacles = 0

    rospy.loginfo("Adding obstacles: " + str(n_obstacles))

    try:
        dict = load(file(ros_pack.get_path('metacontrol_experiments')
                         + '/yaml/obstacle_positions.yaml', 'r'))
    except rospy.ROSInterruptException:
        rospy.loginfo("Error loading yaml file")
        return
    else:
        obstacle_pose_array = []
        for index_obstacle in range(1, n_obstacles+1):
            obstacle_pose = Pose()
            obstacle_pose.position.x = \
                dict['O'+str(index_obstacle)]['pose']['position']['x']
            obstacle_pose.position.y = \
                dict['O'+str(index_obstacle)]['pose']['position']['y']
            obstacle_pose.position.z = \
                dict['O'+str(index_obstacle)]['pose']['position']['z']
            obstacle_pose.orientation.x = \
                dict['O'+str(index_obstacle)]['pose']['orientation']['x']
            obstacle_pose.orientation.y = \
                dict['O'+str(index_obstacle)]['pose']['orientation']['y']
            obstacle_pose.orientation.z = \
                dict['O'+str(index_obstacle)]['pose']['orientation']['z']
            obstacle_pose.orientation.w = \
                dict['O'+str(index_obstacle)]['pose']['orientation']['w']
            obstacle_pose_array.append(obstacle_pose)

    rospy.loginfo("Adding obstacles.")

    try:
        # Get file name of the object to be spawned
        with open(os.path.join(ros_pack.get_path('metacontrol_sim'),
                  "urdf/unknown_obstacle/unknown_obstacle.urdf"),
                  "r") as box_file:
            box_xml = box_file.read()
    except rospy.ROSInterruptException:
        rospy.loginfo("Interrupt received to stop ROS node.")
    else:
        obs_nr = 1
        for obstacle_pose in obstacle_pose_array:
            if not check_obstacle_existence('unknown_obstacle' + str(obs_nr)):
                # add obstacle
                spawn_unknown_obstacle('unknown_obstacle'
                                       + str(obs_nr),
                                       box_xml,
                                       obstacle_pose)
                obs_nr = obs_nr + 1


if __name__ == '__main__':
    add_obstacles_main()
