#!/usr/bin/env python
import roslib
roslib.load_manifest('hello_ros')
import sys
import copy
import rospy
import tf_conversions
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import shape_msgs.msg as shape_msgs
import math
import tf
import re
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.msg import ModelStates


currentJointState = JointState()

def gripper(tmp):   #function to operate the gripper;

    pub = rospy.Publisher("/jaco/joint_control", JointState, queue_size=1)

    currentJointState = rospy.wait_for_message("/joint_states",JointState)
    print ('Received!')
    currentJointState.header.stamp = rospy.get_rostime()
    #tmp_tuple=tuple([tmp] + list(currentJointState.position[1:]))
    currentJointState.position = tuple(list(currentJointState.position[:6]) + [tmp] + [tmp]+ [tmp])
    currentJointState.velocity = tuple([0,0,0,0,0,0,0,0,0])

    rate = rospy.Rate(10) # 10hz
    for i in range(3):
        pub.publish(currentJointState)
        print ('Published!')
        rate.sleep()

    print ('end!')


def callback(data): #callback function; this ask the model states to get the number of cubes and their positions
    global cubePositions
    cubePositions = []
    numCubes = len(data.name)-3 #number of cubes
    for idx, item in enumerate(data.name):
      if  re.match('cube',item):
        cubePositions.append(data.pose[idx])


def move_group_python_interface_tutorial(): #the main function
    moveit_commander.roscpp_initialize(sys.argv)
    sub = rospy.Subscriber('/gazebo/model_states', ModelStates, callback, queue_size=1000 )
    rospy.init_node('move_group_python_interface_tutorial',
                    anonymous=True)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander("Arm")

    ## trajectories for RVIZ to visualize.
    display_trajectory_publisher = rospy.Publisher(
                                        '/move_group/display_planned_path',
                                        moveit_msgs.msg.DisplayTrajectory)

    gripper(0.005) #open the gripper

                    ## Let's setup the planner
    #group.set_planning_time(0.0)
    group.set_goal_orientation_tolerance(0.01)
    group.set_goal_tolerance(0.01)
    group.set_goal_joint_tolerance(0.01)
    group.set_num_planning_attempts(100)
    group.set_max_velocity_scaling_factor(1.0)
    group.set_max_acceleration_scaling_factor(1.0)

    ## Planning to a Pose goal
    print ("============ Generating plan 1")

    ##moving the arm into the initial position
    pose_goal = group.get_current_pose().pose
    pose_goal.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0., -math.pi/2, 0.)) #end facing down
    #position of the bucket
    pose_goal.position.x =0.54373
    pose_goal.position.y =-0.24439
    pose_goal.position.z =0.73763+0.5
    print (pose_goal)
    group.set_pose_target(pose_goal)


    ## Now, we call the planner to compute the plan
    plan1 = group.plan()

    print ("============ Waiting while RVIZ displays plan1...")
    rospy.sleep(0.5)


    ## You can ask RVIZ to visualize a plan (aka trajectory) for you.
    print ("============ Visualizing plan1")
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan1)
    display_trajectory_publisher.publish(display_trajectory)
    print ("============ Waiting while plan1 is visualized (again)...")
    rospy.sleep(1.)
    ## Moving to a pose goal
    group.go(wait=True)


    try:        
        #loop to get all the cubes
        for cubes in cubePositions:
            waypoints = [] #first set of movement. goes until grabbing the cube

            ## Planning to a Pose goal
            print ("============ Generating plan 1")          
            pose_goal1 = group.get_current_pose().pose
            pose_goal1.position.x =cubes.position.x
            pose_goal1.position.y =cubes.position.y
            pose_goal1.position.z =0.73763+0.5
            print (pose_goal1)
          
            #Create waypoints
            waypoints.append(copy.deepcopy(pose_goal1))

            pose_goal2 = group.get_current_pose().pose
            pose_goal2.position.x =cubes.position.x
            pose_goal2.position.y =cubes.position.y
            pose_goal2.position.z =cubes.position.z+0.05
            print (pose_goal2)
            waypoints.append(copy.deepcopy(pose_goal2))

            waypoints2 = [] # second set of movements. from grabing the cube to dropping it
            pose_goal3 = group.get_current_pose().pose
            pose_goal3.position.x =cubes.position.x
            pose_goal3.position.y =cubes.position.y
            pose_goal3.position.z =0.73763+0.5
            print (pose_goal3)
            waypoints2.append(copy.deepcopy(pose_goal3))

            
            pose_goal4 = group.get_current_pose().pose
            pose_goal4.position.x =0.54373
            pose_goal4.position.y =-0.24439
            pose_goal4.position.z =0.73763+0.5
            waypoints2.append(copy.deepcopy(pose_goal4))


            #createcartesian  plan2 for the firs set of movements
            (plan2, fraction) = group.compute_cartesian_path(
                                                waypoints,   # waypoints to follow
                                                0.01,        # eef_step
                                                0.0)         # jump_threshold
            #plan1 = group.retime_trajectory(robot.get_current_state(), plan1, 1.0)
            rospy.sleep(2.)
          
          ## You can ask RVIZ to visualize a plan (aka trajectory) for you.
            display_trajectory = moveit_msgs.msg.DisplayTrajectory()
            display_trajectory.trajectory_start = robot.get_current_state()
            display_trajectory.trajectory.append(plan2)
            display_trajectory_publisher.publish(display_trajectory)
          
            rospy.sleep(1)
          
            group.execute(plan2,wait=True) #do the movements
            rospy.sleep(1.)
            gripper(0.7) #grab the cube

            #createcartesian  plan3 for the second batch of movements
            (plan3, fraction) = group.compute_cartesian_path(
                                                waypoints2,   # waypoints to follow
                                                0.01,        # eef_step
                                                0.0)         # jump_threshold
            #plan1 = group.retime_trajectory(robot.get_current_state(), plan1, 1.0)
            rospy.sleep(2.)
          
          ## You can ask RVIZ to visualize a plan (aka trajectory) for you.
            display_trajectory = moveit_msgs.msg.DisplayTrajectory()
            display_trajectory.trajectory_start = robot.get_current_state()
            display_trajectory.trajectory.append(plan3)
            display_trajectory_publisher.publish(display_trajectory)
          
            rospy.sleep(1)
          
            group.execute(plan3,wait=True) #do it
            rospy.sleep(1.)
            gripper(0.005) #release the cube into the bucket
      ## When finished shut down moveit_commander.
        
        moveit_commander.roscpp_shutdown()
      
        ## END_TUTORIAL
        print ("============ STOPPING")
        R = rospy.Rate(10)
        while not rospy.is_shutdown():
          R.sleep()
    except rospy.ServiceException as e:
        rospy.loginfo("Get Model State service call failed:  {0}".format(e))


 
if __name__=='__main__':
  try:
    move_group_python_interface_tutorial()

  except rospy.ROSInterruptException:
    pass