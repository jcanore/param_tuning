/*
 * random_env_tune.cpp
 *
 *  Created on: 27 Jun 2017
 *      Author: yiming
 */

#include <random_env_tune.h>

#include <iostream>
#include <fstream>
#include <ctime>
using namespace std;

double fRand(double fMin, double fMax)
{
  double f = (double) rand() / RAND_MAX;
  return fMin + f * (fMax - fMin);
}

namespace exotica
{

  RandomEnvTune::RandomEnvTune()
  {
    ros::Time start = ros::Time::now();
    int cnt, obs;
    nh_.getParam("ProblemCnt", cnt);
    nh_.getParam("ObstacleCnt", obs);
    HIGHLIGHT("Creating "<<cnt<<" problems, each with "<<obs<<" obstacles");
    createRandomProblems(cnt, obs);
    HIGHLIGHT(
        "Created "<<cnt<<" problems in "<<ros::Duration(ros::Time::now()-start).toSec()<<" seconds");
  }
  RandomEnvTune::~RandomEnvTune()
  {

  }

  bool RandomEnvTune::evaluateThread(
      const ompl_param_tune::OMPLParamTuneGoalConstPtr &goal, int thread_id)
  {

    ompl_solvers_[thread_id]->getOMPLSolver()->timeout_ = goal->max_time;

    Eigen::MatrixXd solution;
    int dimension = probs_[thread_id]->scene_->getNumJoints();

    for (int i = 0; i < goal->param_names.size(); i++)
    {
      if (ompl_solvers_[thread_id]->getOMPLSolver()->getOMPLSimpleSetup()->getPlanner()->params().hasParam(
          goal->param_names[i]))
        ompl_solvers_[thread_id]->getOMPLSolver()->getOMPLSimpleSetup()->getPlanner()->params().setParam(
            goal->param_names[i], goal->param_values[i]);
      else if (goal->param_names[i] == "simplification_trails")
      {
        ompl_solvers_[thread_id]->getOMPLSolver()->simplify_trails_ = std::stoi(
            goal->param_values[i]);
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
      probs_[thread_id]->scene_->getPlanningScene()->getWorldNonConst()->clearObjects();
      probs_[thread_id]->scene_->getPlanningScene()->processPlanningSceneWorldMsg(
          problems_[i].env);
      sols_[thread_id]->setGoalState(problems_[i].qT);
      try
      {
        sols_[thread_id]->Solve(problems_[i].q0, solution);
        ResultData new_result;
        ros::Time start_time = ros::Time::now();
        new_result.planning_time =
            ompl_solvers_[thread_id]->getOMPLSolver()->planning_time_.toSec();
        new_result.simplification_time = ros::Duration(
            ros::Time::now() - start_time).toSec();
        new_result.cost = 0.0;
        for (int n = 0; n < solution.rows() - 1; n++)
          new_result.cost += (solution.row(n + 1) - solution.row(n)).norm();
        result_maps[i] = new_result;
      } catch (SolveException ex)
      {
        ERROR("OMPL Solver failed at iteration "<<i<<", "<<ex.what());
      }
    }

    return true;
  }

  void RandomEnvTune::createRandomProblems(int prob_cnt, int obs_cnt)
  {
    //  Create prob_cnt problems, each with obs_cnt obstacles
    problems_.resize(prob_cnt);
    planning_scene::PlanningScenePtr existing_scene =
        probs_[0]->scene_->getPlanningScene();
    planning_scene::PlanningScenePtr test_scene =
        probs_[1]->scene_->getPlanningScene();
    for (int p = 0; p < prob_cnt; p++)
    {
      existing_scene->getWorldNonConst()->clearObjects();
      test_scene->getWorldNonConst()->clearObjects();
      int dim = probs_[0]->scene_->getNumJoints();
      Eigen::VectorXd q0(dim), qT(dim), q1(dim), q2(dim);

      do
      {
        test_scene->getCurrentStateNonConst().setToRandomPositions();
        test_scene->getCurrentStateNonConst().update(true);
      } while (!test_scene->isStateValid(test_scene->getCurrentState()));
      for (int n = 0; n < dim; n++)
        q0(n) = test_scene->getCurrentStateNonConst().getVariablePosition(n);

      do
      {
        test_scene->getCurrentStateNonConst().setToRandomPositions();
        test_scene->getCurrentStateNonConst().update(true);
        for (int n = 0; n < dim; n++)
          qT(n) = test_scene->getCurrentStateNonConst().getVariablePosition(n);
      } while (!test_scene->isStateValid(test_scene->getCurrentState())
          && (qT - q0).norm() > 0.5 * M_PI * dim);

      do
      {
        test_scene->getCurrentStateNonConst().setToRandomPositions();
        test_scene->getCurrentStateNonConst().update(true);
      } while (!test_scene->isStateValid(test_scene->getCurrentState()));

      for (int n = 0; n < dim; n++)
        q1(n) = test_scene->getCurrentStateNonConst().getVariablePosition(n);

      do
      {
        test_scene->getCurrentStateNonConst().setToRandomPositions();
        test_scene->getCurrentStateNonConst().update(true);
      } while (!test_scene->isStateValid(test_scene->getCurrentState()));

      for (int n = 0; n < dim; n++)
        q2(n) = test_scene->getCurrentStateNonConst().getVariablePosition(n);

      std::vector<Eigen::VectorXd> solution(0);

      double step = 10.0;
      Eigen::VectorXd diff = (q1 - q0) / (step - 1.0);
      for (int i = 0; i < step; i++)
      {
        Eigen::VectorXd delta = q0 + i * diff;
        solution.push_back(delta);
      }

      diff = (q2 - q1) / (step - 1.0);
      for (int i = 1; i < step; i++)
      {
        Eigen::VectorXd delta = q1 + i * diff;
        solution.push_back(delta);
      }

      diff = (qT - q2) / (step - 1.0);
      for (int i = 1; i < step; i++)
      {
        Eigen::VectorXd delta = q2 + i * diff;
        solution.push_back(delta);
      }

      bool valid = true;
      for (int t = 0; t < solution.size(); t++)
      {
        for (int n = 0; n < dim; n++)
          test_scene->getCurrentStateNonConst().setVariablePosition(n,
              solution[t](n));
        test_scene->getCurrentStateNonConst().update(true);
        if (!test_scene->isStateValid(test_scene->getCurrentState()))
        {
          valid = false;
          break;
        }
      }
      if (!valid)
      {
        p--;
        continue;
      }

//  Make sure we have at least two threads

      moveit_msgs::PlanningSceneWorld existing_world;
      existing_world.collision_objects.clear();
//      existing_world.collision_objects[0].header.frame_id = "base";

      for (int o = 0; o < obs_cnt; o++)
      {
        collision_detection::CollisionRequest req;
        collision_detection::CollisionResult res;
        moveit_msgs::PlanningSceneWorld test_world;
        test_world.collision_objects.resize(1);
        test_world.collision_objects[0].id = o;
        test_world.collision_objects[0].header.frame_id = "base";
        test_world.collision_objects[0].primitives.resize(1);
        test_world.collision_objects[0].primitives[0].type =
            test_world.collision_objects[0].primitives[0].SPHERE;
        test_world.collision_objects[0].primitives[0].dimensions=
        { 0.1,0.1,0.1};
        test_world.collision_objects[0].primitive_poses.resize(1);
        test_world.collision_objects[0].primitive_poses[0].orientation.w = 1;
        test_world.collision_objects[0].primitive_poses[0].position.x = fRand(
            -1, 1);
        test_world.collision_objects[0].primitive_poses[0].position.y = fRand(
            -1, 1);
        test_world.collision_objects[0].primitive_poses[0].position.z = fRand(
            -0.0, 1);

        test_scene->getWorldNonConst()->clearObjects();
        test_scene->processPlanningSceneWorldMsg(test_world);
        //  First, make sure the new obs does not collide with the existing world
        existing_scene->getCollisionWorld()->checkWorldCollision(req, res,
            *test_scene->getCollisionWorld());

        if (res.collision)
        {
          o--;
          continue;
        }

        bool is_ok = true;
        for (int t = 0; t < solution.size(); t++)
        {
          for (int n = 0; n < dim; n++)
            test_scene->getCurrentStateNonConst().setVariablePosition(n,
                solution[t](n));
          test_scene->getCurrentStateNonConst().update(true);
          if (!test_scene->isStateValid(test_scene->getCurrentState()))
          {
            is_ok = false;
            break;
          }
        }
        if (!is_ok)
        {
          o--;
          continue;
        }

        existing_world.collision_objects.push_back(
            test_world.collision_objects[0]);
//        existing_world.collision_objects[0].primitives.push_back(
//            test_world.collision_objects[0].primitives[0]);
//        existing_world.collision_objects[0].primitive_poses.push_back(
//            test_world.collision_objects[0].primitive_poses[0]);
        existing_scene->getWorldNonConst()->clearObjects();
        existing_scene->processPlanningSceneWorldMsg(existing_world);
      }

      //  A random problem is ready
      problems_[p].q0.resize(dim);
      problems_[p].q0 = q0;
      problems_[p].qT.resize(dim);
      problems_[p].qT = qT;
      problems_[p].env = existing_world;
      probs_[0]->scene_->publishScene();
//      getchar();
    }

  }
}
