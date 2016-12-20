/*
 * ompl_param_tune.cpp
 *
 *  Created on: 9 Nov 2016
 *      Author: yiming
 */

#include <ompl_param_tune.h>

#include <iostream>
#include <fstream>
#include <ctime>
using namespace std;

int ITER=1;
time_t now = time(0);
tm *ltm = localtime(&now);

ofstream myfile;
//file_name = "/home/jcanore/anyscale/param/tuning/src/param_tuning/ompl_param_tune/src/output_" << ITER; 
std::string file_name="/home/jcanore/anyscale/param/tuning/src/param_tuning/ompl_param_tune/src/output_" + 
    std::to_string(1900 + ltm->tm_year) + "-" + std::to_string(ltm->tm_mon)+ "-" + std::to_string(ltm->tm_mday) + 
    "_" + std::to_string(ltm->tm_hour) + "-" + std::to_string(ltm->tm_min);    

namespace exotica
{
  OMPLParamTune::OMPLParamTune(): nh_("~"), as_(nh_, "/OMPL_PARAM_TUNE", boost::bind(&OMPLParamTune::evaluate, this, _1), false)
  {
    std::string config_name;
    std::string problem_name, solver_name;
    nh_.getParam("config", config_name);
    nh_.getParam("problem", problem_name);
    nh_.getParam("solver", solver_name);
    ROS_INFO_STREAM("Config: "<<config_name<<"\nSolver: "<<solver_name[0]<<"\nProblem: "<<problem_name[0]);

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
  
  /////////////////////////////////////////////////////////////////////////////////////////////////////

  bool OMPLParamTune::evaluate(
      const ompl_param_tune::OMPLParamTuneGoalConstPtr &goal)
  {
    OMPLsolver_ptr ompl_solver = boost::static_pointer_cast<OMPLsolver>(sol_);
    ompl_solver->getOMPLSolver()->timeout_ = goal->max_time;

    int dimension = prob_->scene_->getNumJoints();
    Eigen::VectorXd q_start, q_goal;
    exotica::vectorExoticaToEigen(goal->q_start, q_start);
    exotica::vectorExoticaToEigen(goal->q_goal, q_goal);
    Eigen::MatrixXd solution;
    prob_->scene_->getPlanningScene()->getWorldNonConst()->clearObjects();
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
      for (int i = 0; i < goal->param_names.size(); i++)
      {
        if (ompl_solver->getOMPLSolver()->getOMPLSimpleSetup()->getPlanner()->params().hasParam(
            goal->param_names[i]))
          ompl_solver->getOMPLSolver()->getOMPLSimpleSetup()->getPlanner()->params().setParam(
              goal->param_names[i], goal->param_values[i]);
        else if (goal->param_names[i] == "simplification_trails")
        {
          ompl_solver->getOMPLSolver()->simplify_trails_ = std::stoi(
              goal->param_values[i]);
          ROS_INFO_STREAM("Set simplification trails to "<<ompl_solver->getOMPLSolver()->simplify_trails_);
        }
        else
          ROS_ERROR_STREAM("Unknown parameter "<<goal->param_names[i]);
      }
    }

    //  Everything is set, now lets solve
    int max_it = goal->max_it;
    int succeed_cnt = 0;          
    
    struct ResultData
    {
        double planning_time;
        double simplification_time;
        double cost;
    };

    std::map<int, ResultData> result_maps;
    {
      for (int i = 0; i < max_it; i++)
      {
        try
        {
          sol_->Solve(q_start, solution);
          ResultData new_result;
          new_result.planning_time =
              ompl_solver->getOMPLSolver()->planning_time_.toSec();
          new_result.simplification_time =
              ompl_solver->getOMPLSolver()->simplification_time_.toSec();
          new_result.cost = 0.0;
          for (int n = 0; n < solution.rows() - 1; n++)
            new_result.cost += (solution.row(n + 1) - solution.row(n)).norm();
          result_maps[i] = new_result;
        } catch (SolveException ex)
        {
          ERROR("OMPL Solver failed at iteration "<<i<<", "<<ex.what());
        }
      }

      result.succeed = true;
      double success_cnt = result_maps.size();
      result.success_rate = (double) success_cnt / max_it;
      ROS_INFO_STREAM(
          "Sucessfully solved "<<success_cnt<<" out of "<<max_it<<" trails");

      Eigen::VectorXd planning_times(success_cnt), simplification_times(
          success_cnt), costs(success_cnt);
	  
      int i = 0;      
      double plan_time_min=100;
      double simp_time_min=100;
      double tot_time_min=100;
      double ct_min=100;
      
      double plan_time_max=0;
      double simp_time_max=0;
      double tot_time_max=0;
      double ct_max=0;
      
      
      myfile.open (file_name, ios::app);
      myfile << "\nConf_" << ITER << "\n\n";
      myfile.close();
      
      
      for (auto &it : result_maps)
      {
        planning_times[i] = it.second.planning_time;
        simplification_times[i] = it.second.simplification_time;
        costs[i] = it.second.cost;		
	
	myfile.open (file_name, ios::app);
	myfile << it.second.planning_time << "   " << it.second.simplification_time << "   " << it.second.cost << "\n";
	myfile.close();
	
	//////////////////////////////////////////////////////////////////////////////
	
	if(it.second.planning_time < plan_time_min)
	    plan_time_min=it.second.planning_time;	
	
	if(it.second.planning_time > plan_time_max)
	    plan_time_max=it.second.planning_time;
	
	if(it.second.simplification_time < simp_time_min)
	    simp_time_min=it.second.simplification_time;
	
	if(it.second.simplification_time > simp_time_max)
	    simp_time_max=it.second.simplification_time;
	
	if(it.second.planning_time + it.second.simplification_time < tot_time_min)
	    tot_time_min=it.second.planning_time + it.second.simplification_time;
	
	if(it.second.planning_time + it.second.simplification_time > tot_time_max)
	    tot_time_max=it.second.planning_time + it.second.simplification_time;
	    
	if(it.second.cost < ct_min)
	    ct_min=it.second.cost;
	
	if(it.second.cost > ct_max)
	    ct_max=it.second.cost;
	
	//////////////////////////////////////////////////////////////////////////////
	
        i++;
      }                  
      
      ////////////////////////////////////////////////
      
      result.planning_time_min=plan_time_min;
      result.planning_time_max=plan_time_max;      
      result.simplification_time_min=simp_time_min;
      result.simplification_time_max=simp_time_max;
      result.total_time_min=tot_time_min;
      result.total_time_max=tot_time_max;
      result.cost_min=ct_min;
      result.cost_max=ct_max;
      
      ////////////////////////////////////////////////
      
      computeMeanStd(planning_times, result.planning_time_mean, result.planning_time_std);
      computeMeanStd(simplification_times, result.simplification_time_mean, result.simplification_time_std);
      computeMeanStd(costs, result.cost_mean, result.cost_std);
      as_.setSucceeded(result);
    }   
    
    myfile.open (file_name, ios::app);
    myfile << "-----------------------------------" << "\n";
    myfile.close();
    
    ITER++;
    
    return result.succeed;
  }

  void OMPLParamTune::computeMeanStd(const Eigen::VectorXd &data, double &mean, double &std)
  {
    double sum = data.sum();
    mean = data.mean();
    std = 0.0;

    for (int i = 0; i < data.rows(); ++i)
      std += pow(data(i) - mean, 2.0);
    std = sqrt(std/data.rows());
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "OMPL_PARAM_TUNE_ACTION_NODE");
  exotica::OMPLParamTune ompl_tune;
  ros::spin();
}
