/*
 * ompl_param_tune.cpp
 *
 *  Created on: 9 Nov 2016
 *      Author: yiming
 */

#include <ompl_param_tune.h>

namespace exotica
{
  OMPLParamTune::OMPLParamTune()
      : nh_("~"), as_(nh_, "/OMPL_PARAM_TUNE",
          boost::bind(&OMPLParamTune::evaluate, this, _1), false)
  {
    std::string config_name;
    std::string problem_name, solver_name;
    nh_.getParam("config", config_name);
    nh_.getParam("problem", problem_name);
    nh_.getParam("solver", solver_name);
    ROS_INFO_STREAM(
        "Config: "<<config_name<<"\nSolver: "<<solver_name[0]<<"\nProblem: "<<problem_name[0]);

    Initialiser::Instance()->initialise(config_name, ser_, sol_, prob_,
        problem_name, solver_name);
    sol_->specifyProblem(prob_);

    as_.start();
    HIGHLIGHT_NAMED("OMPL Paramter Tunning Action",
        "Ready. Action topic /OMPL_PARAM_TUNE");
  }

  OMPLParamTune::~OMPLParamTune()
  {

  }

  bool OMPLParamTune::evaluate(
      const ompl_param_tune::OMPLParamTuneGoalConstPtr &goal)
  {

    int dimension = prob_->scene_->getNumJoints();
    Eigen::VectorXd q_start, q_goal;
    exotica::vectorExoticaToEigen(goal->q_start, q_start);
    exotica::vectorExoticaToEigen(goal->q_goal, q_goal);
    Eigen::MatrixXd solution;

    prob_->scene_->getPlanningScene()->processPlanningSceneWorldMsg(
        goal->environment);
    prob_->scene_->publishScene();
    sol_->setGoalState(q_goal);

    ompl_param_tune::OMPLParamTuneResult result;
    if (goal->param_names.size() != goal->param_values.size())
    {
      ERROR(
          "Param name size "<<goal->param_names.size()<<", param value size "<<goal->param_values.size());
      result.succeed = false;
      as_.setAborted(result);
      return false;
    }
    else
    {
      OMPLsolver_ptr ompl_solver = boost::static_pointer_cast<OMPLsolver>(sol_);
      for (int i = 0; i < goal->param_names.size(); i++)
      {
        ompl_solver->getOMPLSolver()->getOMPLSimpleSetup()->getPlanner()->params().setParam(
            goal->param_names[i], goal->param_values[i]);
      }
    }

    //  Everything is set, now lets solve
    int max_it = goal->max_it;
    //  First check if the problem is solvable.
    bool solvable = false;
    try
    {
      sol_->Solve(q_start, solution);
      solvable = true;
    } catch (SolveException ex)
    {
      ERROR("OMPL Solver failed, "<<ex.what());
    }
    if (!solvable)
    {
      result.succeed = false;
      as_.setAborted(result);
    }
    else
    {
      HIGHLIGHT(
          "The problem is solvable, now start the evaluation over "<<max_it<<" trials");
      std::map<int, double> time_maps;
      for (int i = 0; i < max_it; i++)
      {
        try
        {
          ros::Time start = ros::Time::now();
          sol_->Solve(q_start, solution);
          time_maps[i] = ros::Duration(ros::Time::now() - start).toSec();
        } catch (SolveException ex)
        {
          ERROR("OMPL Solver failed at iteration "<<i<<", "<<ex.what());
        }
      }

      result.succeed = true;
      double overall = 0.0;
      for (auto &it : time_maps)
        overall += it.second;
      result.planning_time = overall / (double) time_maps.size();
      as_.setSucceeded(result);
    }
    return result.succeed;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "OMPL_PARAM_TUNE_ACTION_NODE");
  exotica::OMPLParamTune ompl_tune;
  ros::spin();
}
