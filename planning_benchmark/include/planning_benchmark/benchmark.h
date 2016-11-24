/*
 * benchmark.h
 *
 *  Created on: 24 Nov 2016
 *      Author: yiming
 */

#ifndef PARAM_TUNING_PLANNING_BENCHMARK_INCLUDE_BENCHMARK_H_
#define PARAM_TUNING_PLANNING_BENCHMARK_INCLUDE_BENCHMARK_H_

#include <ros/ros.h>
#include <ros/package.h>
#include <exotica/Initialiser.h>
#include <sensor_msgs/JointState.h>
#include "exotica/Definitions/TaskTerminationCriterion.h"

using namespace exotica;
class OMPLBenchmarkNode
{
  public:
    OMPLBenchmarkNode();
    bool benchmarking();
    bool benchmarkingConfig(const std::string &name, const std::string &config);
  private:
    MotionSolver_ptr sol;
    Server_ptr ser;
    PlanningProblem_ptr prob;
    ros::NodeHandle nh_;

    std::map<std::string, std::string> configs;
    std::map<std::string, double> scores;
    int trials;

    moveit_msgs::PlanningSceneWorld environment;
    Eigen::VectorXd q_start;
    Eigen::VectorXd q_goal;
};



#endif /* PARAM_TUNING_PLANNING_BENCHMARK_INCLUDE_BENCHMARK_H_ */
