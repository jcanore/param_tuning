/*
 * ompl_param_tune.h
 *
 *  Created on: 8 Nov 2016
 *      Author: yiming
 */

#ifndef OMPL_PARAM_TUNE_INCLUDE_OMPL_PARAM_TUNE_H_
#define OMPL_PARAM_TUNE_INCLUDE_OMPL_PARAM_TUNE_H_

#include <exotica/EXOTica.hpp>
#include <ompl_param_tune/OMPLParamTuneAction.h>
#include <actionlib/server/simple_action_server.h>
#include <ompl_imp_solver/OMPLImpSolver.h>
#include <ompl_solver/OMPLsolver.h>

namespace exotica
{
  class OMPLParamTune
  {
    public:
      OMPLParamTune();
      ~OMPLParamTune();

    private:
      bool evaluate(const ompl_param_tune::OMPLParamTuneGoalConstPtr &goal);
      void computeMeanStd(const Eigen::VectorXd &data, double &mean, double &std);
      ros::NodeHandle nh_;
      actionlib::SimpleActionServer<ompl_param_tune::OMPLParamTuneAction> as_;
      MotionSolver_ptr sol_;
      Server_ptr ser_;
      PlanningProblem_ptr prob_;
  };
}

#endif /* OMPL_PARAM_TUNE_INCLUDE_OMPL_PARAM_TUNE_H_ */
