#! /usr/bin/env python

#import roslib
import rospy

import actionlib

import ompl_param_tune.msg
#import exotica.msg
import moveit_msgs.msg
import shape_msgs.msg
import geometry_msgs.msg

import sys


##########################################################################################

def ompl_tune_client(p1, p2, p3, p4, p5):
    
    client = actionlib.SimpleActionClient('/OMPL_PARAM_TUNE', ompl_param_tune.msg.OMPLParamTuneAction)
    client.wait_for_server()
    
    goal = ompl_param_tune.msg.OMPLParamTuneGoal()
    goal.q_start.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    goal.q_goal.data = [0.0, 1.9, 0.0, 0.5, 0.0, 0.0, 0.0]   
    
    #----------------------------------------------------------------------------------------#	objects           
           
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
    goal.environment.collision_objects.append(obs2)
    
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
    
    #----------------------------------------------------------------------------------------#
    
    goal.max_it = 10    
    goal.max_time = 30    
        
    #----------------------------------------------------------------------------------------#
    
    goal.param_names.append("range")	# default: 1
    goal.param_values.append(p1)	# values interval: [0.1, 2.0] ; step: 0.1	--> 20 possible values					
										
    goal.param_names.append("border_fraction")	# default: 0.5
    goal.param_values.append(p2)	# values interval: ]0, 1] ; step: 0.05		--> 20 possible values					
    
    goal.param_names.append("failed_expansion_score_factor")	# default: 0.5
    goal.param_values.append(p3)	# values interval: ]0, 1] ; step: 0.05		--> 20 possible values					
    
    goal.param_names.append("min_valid_path_fraction")	# default: 0.5
    goal.param_values.append(p4)	# values interval: ]0, 1] ; step: 0.05		--> 20 possible values					
		
    goal.param_names.append("simplification_trails")	# default: 10
    goal.param_values.append(p5) 	# values: [0, 5, 10, 20, 50, 100, 200] 		--> 7 possible values
    
    #----------------------------------------------------------------------------------------#
        
    # Search space = 20 x 20 x 20 x 20 x 7 = 1,120,000        
    
    #----------------------------------------------------------------------------------------#
    
    client.send_goal(goal)
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()


##########################################################################################################################

if __name__ == '__main__':
    
    if len(sys.argv) < 6:
        print "Please provide 5 parameters"
        sys.exit(-1)
        
    try:
        rospy.init_node('ompl_tune_client_py')                                                   
        
        result = ompl_tune_client(sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4], sys.argv[5])
        
        t_time = result.planning_time_mean + result.simplification_time_mean
	t_time_std = result.planning_time_std + result.simplification_time_std		
	t_time_error = t_time_std/t_time * 100;
	cost = result.cost_mean
	cost_error = result.cost_std/cost * 100
                            
        print sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4], sys.argv[5], format(t_time, '.6f'), format(result.total_time_min, '.6f'), format(result.total_time_max, '.6f'), format(t_time_std, '.2f'), format(t_time_error, '.2f'), format(cost, '.2f'), format(result.cost_min, '.2f'), format(result.cost_max, '.2f'), format(result.cost_std, '.2f'), format(cost_error, '.2f'), format(result.success_rate, '.2f')
                
                        
    except rospy.ROSInterruptException:
	print "program interrupted before completion"
        
       
      
        
        

