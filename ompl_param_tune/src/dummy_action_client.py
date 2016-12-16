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
    obs.id = "obs1"
    obs.primitives.append(shape_msgs.msg.SolidPrimitive())
    obs.primitives[0].type = 1;
    obs.primitives[0].dimensions = [0.1, 1, 0.1]
    
    obs.primitive_poses.append(geometry_msgs.msg.Pose())
    obs.primitive_poses[0].position.x = -0.3
    obs.primitive_poses[0].position.y = 0.0
    obs.primitive_poses[0].position.z = 0.9
    goal.environment.collision_objects.append(obs)
    
    obs2 = moveit_msgs.msg.CollisionObject()
    obs2.header.frame_id = "/base"
    obs2.id = "obs2"
    obs2.primitives.append(shape_msgs.msg.SolidPrimitive())
    obs2.primitives[0].type = 1;
    obs2.primitives[0].dimensions = [0.8, 0.1, 0.8]
    
    obs2.primitive_poses.append(geometry_msgs.msg.Pose())
    obs2.primitive_poses[0].position.x = -0.5
    obs2.primitive_poses[0].position.y = -0.2
    obs2.primitive_poses[0].position.z = 0.6
    #goal.environment.collision_objects.append(obs2)
    
    obs3 = moveit_msgs.msg.CollisionObject()
    obs3.header.frame_id = "/base"
    obs3.id = "obs3"
    obs3.primitives.append(shape_msgs.msg.SolidPrimitive())
    obs3.primitives[0].type = 1;
    obs3.primitives[0].dimensions = [0.8, 0.1, 0.8]
    
    obs3.primitive_poses.append(geometry_msgs.msg.Pose())
    obs3.primitive_poses[0].position.x = -0.5
    obs3.primitive_poses[0].position.y = 0.2
    obs3.primitive_poses[0].position.z = 0.6
    goal.environment.collision_objects.append(obs3)
    
    
    goal.max_it = 10
    goal.max_time = 10;
    
    
    #----------------------------------------------------------------------------------------#
    goal.param_names.append("range")
    goal.param_values.append(".1")
    
    goal.param_names.append("border_fraction")
    goal.param_values.append("0.5")
    
    goal.param_names.append("failed_expansion_score_factor")
    goal.param_values.append("0.5")
    
    goal.param_names.append("min_valid_path_fraction")
    goal.param_values.append("0.5")  
    
    goal.param_names.append("simplification_trails")
    goal.param_values.append("10")  
    
    #----------------------------------------------------------------------------------------#
    
    client.send_goal(goal)
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()

if __name__ == '__main__':
    try:
        rospy.init_node('ompl_tune_client_py')
        result = ompl_tune_client()
        print "Success rate       : %.4f"% result.success_rate
        print "Planning time      : %.4f"% result.planning_time_mean, u"\u00B1", "%.4f"%result.planning_time_std
        print "Simplification time: %.4f"% result.simplification_time_mean, u"\u00B1", "%.4f"%result.simplification_time_std
        print "Trajectory cost    : %.4f"% result.cost_mean, u"\u00B1", "%.4f"%result.cost_std
    except rospy.ROSInterruptException:
        print "program interrupted before completion"