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

int ITER = 1;
time_t now = time(0);
tm *ltm = localtime(&now);

ofstream myfile;
//file_name = "/home/jcanore/anyscale/param/tuning/src/param_tuning/ompl_param_tune/src/output_" << ITER; 
std::string file_name = "/home/jcanore/anyscale/param/tuning/src/param_tuning/ompl_param_tune/src/output_"
        + std::to_string(1900 + ltm->tm_year) + "-" + std::to_string(ltm->tm_mon) + "-" + std::to_string(ltm->tm_mday)
        + "_" + std::to_string(ltm->tm_hour) + "-" + std::to_string(ltm->tm_min);

namespace exotica
{
    OMPLParamTune::OMPLParamTune() :
            nh_("~"), as_(nh_, "/OMPL_PARAM_TUNE", boost::bind(&OMPLParamTune::evaluate, this, _1), false)
    {
        std::string config_name;
        std::string problem_name, solver_name;
        nh_.getParam("config", config_name);
        nh_.getParam("problem", problem_name);
        nh_.getParam("solver", solver_name);
        if (!nh_.getParam("thread", thread_cnt_))
            thread_cnt_ = 8;
        ROS_INFO_STREAM(
                "Config: "<<config_name<<"\nSolver: "<<solver_name<<"\nProblem: "<<problem_name<<", using "<<thread_cnt_<<" threads");

        sols_.resize(thread_cnt_);
        probs_.resize(thread_cnt_);
        ompl_solvers_.resize(thread_cnt_);
        for (int i = 0; i < thread_cnt_; i++)
        {
            Initialiser::Instance()->initialise(config_name, ser_, sols_[i], probs_[i], problem_name, solver_name);
            sols_[i]->specifyProblem(probs_[i]);
            ompl_solvers_[i] = boost::static_pointer_cast<OMPLsolver>(sols_[i]);
        }

        as_.start();
        HIGHLIGHT_NAMED("OMPL Paramter Tunning Action", "Ready. Action topic /OMPL_PARAM_TUNE");
    }

    OMPLParamTune::~OMPLParamTune()
    {

    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////

    bool OMPLParamTune::evaluate(const ompl_param_tune::OMPLParamTuneGoalConstPtr &goal)
    {

        ompl_param_tune::OMPLParamTuneResult result;

        int succeed_cnt = 0;

        result_maps.clear();

        std::vector<boost::thread*> th(thread_cnt_);
        for (int i = 0; i < thread_cnt_; i++)
            th[i] = new boost::thread(boost::bind(&OMPLParamTune::evaluateThread, this, goal, i));
        for (unsigned int i = 0; i < thread_cnt_; ++i)
        {
            th[i]->join();
            delete th[i];
        }

        int max_it = goal->max_it;

        result.succeed = true;
        double success_cnt = result_maps.size();
        result.success_rate = (double)success_cnt / max_it;
        ROS_INFO_STREAM("Sucessfully solved "<<success_cnt<<" out of "<<max_it<<" trails");

        Eigen::VectorXd planning_times(success_cnt), simplification_times(success_cnt), costs(success_cnt);

        int i = 0;
        double plan_time_min = 100;
        double simp_time_min = 100;
        double tot_time_min = 100;
        double ct_min = 100;

        double plan_time_max = 0;
        double simp_time_max = 0;
        double tot_time_max = 0;
        double ct_max = 0;

        myfile.open(file_name, ios::app);
        myfile << "\nConf_" << ITER << "\n\n";
        myfile.close();

        for (auto &it : result_maps)
        {
            planning_times[i] = it.second.planning_time;
            simplification_times[i] = it.second.simplification_time;
            costs[i] = it.second.cost;

            myfile.open(file_name, ios::app);
            myfile << it.second.planning_time << "   " << it.second.simplification_time << "   " << it.second.cost
                    << "\n";
            myfile.close();

            //////////////////////////////////////////////////////////////////////////////

            if (it.second.planning_time < plan_time_min)
                plan_time_min = it.second.planning_time;

            if (it.second.planning_time > plan_time_max)
                plan_time_max = it.second.planning_time;

            if (it.second.simplification_time < simp_time_min)
                simp_time_min = it.second.simplification_time;

            if (it.second.simplification_time > simp_time_max)
                simp_time_max = it.second.simplification_time;

            if (it.second.planning_time + it.second.simplification_time < tot_time_min)
                tot_time_min = it.second.planning_time + it.second.simplification_time;

            if (it.second.planning_time + it.second.simplification_time > tot_time_max)
                tot_time_max = it.second.planning_time + it.second.simplification_time;

            if (it.second.cost < ct_min)
                ct_min = it.second.cost;

            if (it.second.cost > ct_max)
                ct_max = it.second.cost;

            //////////////////////////////////////////////////////////////////////////////

            i++;
        }

        ////////////////////////////////////////////////

        result.planning_time_min = plan_time_min;
        result.planning_time_max = plan_time_max;
        result.simplification_time_min = simp_time_min;
        result.simplification_time_max = simp_time_max;
        result.total_time_min = tot_time_min;
        result.total_time_max = tot_time_max;
        result.cost_min = ct_min;
        result.cost_max = ct_max;

        ////////////////////////////////////////////////

        computeMeanStd(planning_times, result.planning_time_mean, result.planning_time_std);
        computeMeanStd(simplification_times, result.simplification_time_mean, result.simplification_time_std);
        computeMeanStd(costs, result.cost_mean, result.cost_std);
        as_.setSucceeded(result);

        myfile.open(file_name, ios::app);
        myfile << "-----------------------------------" << "\n";
        myfile.close();

        ITER++;

        return result.succeed;
    }

    bool OMPLParamTune::evaluateThread(const ompl_param_tune::OMPLParamTuneGoalConstPtr &goal, int thread_id)
    {
        ompl_solvers_[thread_id]->getOMPLSolver()->timeout_ = goal->max_time;
        Eigen::VectorXd q_start, q_goal;
        exotica::vectorExoticaToEigen(goal->q_start, q_start);
        exotica::vectorExoticaToEigen(goal->q_goal, q_goal);
        Eigen::MatrixXd solution;
        int dimension = probs_[thread_id]->scene_->getNumJoints();
        probs_[thread_id]->scene_->getPlanningScene()->getWorldNonConst()->clearObjects();
        probs_[thread_id]->scene_->getPlanningScene()->processPlanningSceneWorldMsg(goal->environment);
        probs_[thread_id]->scene_->publishScene();
        sols_[thread_id]->setGoalState(q_goal);

        for (int i = 0; i < goal->param_names.size(); i++)
        {
            if (ompl_solvers_[thread_id]->getOMPLSolver()->getOMPLSimpleSetup()->getPlanner()->params().hasParam(
                    goal->param_names[i]))
                ompl_solvers_[thread_id]->getOMPLSolver()->getOMPLSimpleSetup()->getPlanner()->params().setParam(
                        goal->param_names[i], goal->param_values[i]);
            else if (goal->param_names[i] == "simplification_trails")
            {
                ompl_solvers_[thread_id]->getOMPLSolver()->simplify_trails_ = std::stoi(goal->param_values[i]);
                ROS_INFO_STREAM(
                        "Set simplification trails to "<<ompl_solvers_[thread_id]->getOMPLSolver()->simplify_trails_);
            }
            else
                ROS_ERROR_STREAM("Unknown parameter "<<goal->param_names[i]);
        }
        //  Everything is set, now lets solve
        int max_it = goal->max_it;
        for (int i = thread_id; i < max_it; i += thread_cnt_)
        {
            try
            {
                sols_[thread_id]->Solve(q_start, solution);
                ResultData new_result;
                ros::Time start_time = ros::Time::now();
                new_result.planning_time = ompl_solvers_[thread_id]->getOMPLSolver()->planning_time_.toSec();
                new_result.simplification_time = ros::Duration(ros::Time::now()-start_time).toSec();
//                new_result.simplification_time =
//                        ompl_solvers_[thread_id]->getOMPLSolver()->simplification_time_.toSec();
                new_result.cost = 0.0;
                for (int n = 0; n < solution.rows() - 1; n++)
                    new_result.cost += (solution.row(n + 1) - solution.row(n)).norm();
                result_maps[i] = new_result;
            }
            catch (SolveException ex)
            {
                ERROR("OMPL Solver failed at iteration "<<i<<", "<<ex.what());
            }
        }

        return true;
    }

    void OMPLParamTune::computeMeanStd(const Eigen::VectorXd &data, double &mean, double &std)
    {
        double sum = data.sum();
        mean = data.mean();
        std = 0.0;

        for (int i = 0; i < data.rows(); ++i)
            std += pow(data(i) - mean, 2.0);
        std = sqrt(std / data.rows());
    }
}
