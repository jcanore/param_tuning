/*
 * benchmark.cpp
 *
 *  Created on: 24 Nov 2016
 *      Author: yiming
 */

#include "planning_benchmark/benchmark.h"
#include <ros/package.h>
using namespace exotica;

OMPLBenchmarkNode::OMPLBenchmarkNode()
    : nh_("~")
{

  std::string package_path = ros::package::getPath("planning_benchmark");
//  configs["RRT"] = package_path + "/resources/RRT.xml";
  configs["RRTConnect"] = package_path + "/resources/RRTConnect.xml";
  configs["PRM"] = package_path + "/resources/PRM.xml";

  trials = 100;

  moveit_msgs::CollisionObject box;
  box.header.frame_id = "/base";
  box.id = "box0";
  box.primitives.resize(1);
  box.primitives[0].type = box.primitives[0].BOX;
  box.primitives[0].dimensions=
  { 0.1, 1, 0.1};
  box.primitive_poses.resize(1);
  box.primitive_poses[0].orientation.w = 1;
  box.primitive_poses[0].position.x = 0.3;
  box.primitive_poses[0].position.y = 0.0;
  box.primitive_poses[0].position.z = 0.7;
  environment.collision_objects.push_back(box);

  q_start.setZero(7);
  q_goal.resize(7);
  q_goal << -0.134914, -1, -0.124971, 2, 0, 0, 0;
}

bool OMPLBenchmarkNode::benchmarking()
{
  scores.clear();
  for (auto &it : configs)
    if (!benchmarkingConfig(it.first, it.second)) return false;

  std::cout<<std::endl;
  HIGHLIGHT(
      "======================= Benchmarking Result =======================");
  std::cout<<std::endl;

  for (auto &it : scores)
    HIGHLIGHT_NAMED(it.first, "Average planning time "<<it.second);

  std::cout<<std::endl;
  HIGHLIGHT(
      "===================================================================");
  return true;
}

bool OMPLBenchmarkNode::benchmarkingConfig(const std::string &name,
    const std::string &config)
{
  std::string problem_name, solver_name;
  nh_.getParam("problem", problem_name);
  nh_.getParam("solver", solver_name);
  HIGHLIGHT("Benchmarking for configuration "<<name);
  Initialiser::Instance()->initialise(config, ser, sol, prob, problem_name,
      solver_name);
  sol->specifyProblem(prob);

  prob->scene_->getPlanningScene()->processPlanningSceneWorldMsg(environment);
  prob->scene_->publishScene();

  Eigen::VectorXd q = q_start;
  Eigen::MatrixXd solution;

  sol->setGoalState(q_goal);

  bool problem_ok = false;
  double time = 0.0;
  try
  {
    ros::Time start_time = ros::Time::now();
    sol->Solve(q, solution);
    time += ros::Duration(ros::Time::now() - start_time).toSec();
    problem_ok = true;
  } catch (SolveException e)
  {
    ERROR(e.what());
  }

  if (!problem_ok)
  {
    ERROR("Problem can not be solved");
    return false;
  }

  for (int i = 0; i < trials - 1; i++)
  {
    HIGHLIGHT_NAMED(name, "Benchmarking iteration "<<i+1<<"/"<<trials);
    ros::Time start_time = ros::Time::now();
    sol->Solve(q, solution);
    time += ros::Duration(ros::Time::now() - start_time).toSec();
  }
  scores[name] = time / (double) trials;
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "OMPLSolverDemoNode");
  ROS_INFO_STREAM("Started");
  OMPLBenchmarkNode ex;
  ex.benchmarking();
  ros::spin();
}

