#! /usr/bin/env python

import roslib;
import rospy

import actionlib

import ompl_param_tune.msg
import exotica.msg
import moveit_msgs.msg
import shape_msgs.msg
import geometry_msgs.msg

def ompl_tune_client():
    client = actionlib.SimpleActionClient('/OMPL_PARAM_TUNE', ompl_param_tune.msg.OMPLParamTuneAction)
    client.wait_for_server()
    goal = ompl_param_tune.msg.OMPLParamTuneGoal()
    goal.q_start.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    goal.q_goal.data = [0.0, 1.9, 0.0, 0.5, 0.0, 0.0, 0.0]
    
    obs = moveit_msgs.msg.CollisionObject()
    obs.header.frame_id = "/base"
    obs.primitives.append(shape_msgs.msg.SolidPrimitive())
    obs.primitives[0].type = 1;
    obs.primitives[0].dimensions = [0.1, 1, 0.1]
    
    obs.primitive_poses.append(geometry_msgs.msg.Pose())
    obs.primitive_poses[0].position.x = -0.3
    obs.primitive_poses[0].position.y = 0.0
    obs.primitive_poses[0].position.z = 0.6
    
    goal.environment.collision_objects.append(obs)
    
    goal.max_it = 10
    
    
    #----------------------------------------------------------------------------------------#
    goal.param_names.append("range")
    goal.param_values.append("1")
    
    goal.param_names.append("border_fraction")
    goal.param_values.append("0.5")
    
    goal.param_names.append("failed_expansion_score_factor")
    goal.param_values.append("0.5")
    
    goal.param_names.append("min_valid_path_fraction")
    goal.param_values.append("0.5")  
    
    #----------------------------------------------------------------------------------------#
    
    client.send_goal(goal)
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()  # A FibonacciResult

if __name__ == '__main__':
    try:
        rospy.init_node('ompl_tune_client_py')
        result = ompl_tune_client()
        print "Succeeded:", result.succeed, " average time ", result.planning_time
    except rospy.ROSInterruptException:
        print "program interrupted before completion"