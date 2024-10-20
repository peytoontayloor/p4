///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 4
// Authors: Peyton Elebash, Swaha Roy and Audrey Lu
//////////////////////////////////////

#include <iostream>

#include <ompl/base/ProjectionEvaluator.h>

#include <ompl/control/SimpleSetup.h>
#include <ompl/control/ODESolver.h>

// Your implementation of RG-RRT
#include "RG-RRT.h"

//includes from the demo documentation that I think are necessary:
#include <ompl/control/SpaceInformation.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/control/ODESolver.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/config.h>

//need includes for the planners too!
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>

namespace ob = ompl::base;
namespace oc = ompl::control;

// Your projection for the pendulum
class PendulumProjection : public ob::ProjectionEvaluator
{
public:
    PendulumProjection(const ob::StateSpace *space) : ProjectionEvaluator(space)
    {
    }

    unsigned int getDimension() const override
    {
        // TODO: The dimension of your projection for the pendulum
        return 0;
    }

    void project(const ob::State */* state */, Eigen::Ref<Eigen::VectorXd> /* projection */) const override
    {
        // TODO: Your projection for the pendulum
    }
};

/*
        state of system q = (theta, w), theta is orientation of pendulum and w is rotational velocity

        torque is t (from main) -->might be a control input

        gravity is roughly 9.81 per the project spec

        qdot = (w, -gcos(theta)+t)
*/
void pendulumODE(const oc::ODESolver::StateType & q, const oc::Control * control,
                 oc::ODESolver::StateType & qdot)
{
    
    // Retrieve control values: torque.
    const double *u = controls->as<oc::RealVectorControlSpace::ControlType>()->values;
    const double torque = u[0];

    // Retrieve current orientation: q[0] = theta, q[1] = w
    theta = q[0]
    w = q[1]

    // Ensure qdot same size as q. Zero out all values. 
    // q = (theta, w), qdot = (thetadot, wdot)
    qdot.resize(q.size(), 0);

    // System dynamics: qdot = (thetadot, wdot) = (w, -gcos(theta) + torque)
    //
    qdot[0] = w
    qdot[1] = ((-9.81)*(cos(theta)) + torque)
}

oc::SimpleSetupPtr createPendulum(double torque)
{
    // Create pendulum's state space: SO(2)
    auto space(std::make_shared<ob::SO2StateSpace>());
    // TODO: check, I think we don't need to manually set bounds, since we never set theta bounds for SE(2)
    // hold up maybe ang velocity is also part of state space??
    
    //control space setup:
    //TODO: verify we only need "1" for 1 control input, torque
    auto cspace(std::make_shared<oc::RealVectorControlSpace>(space, 1));

    // Set control space bounds.
    ob::RealVectorBounds cbounds(1);
    cbounds.setLow(-1 * torque);
    cbounds.setHigh(torque);
    cspace->setBounds(cbounds);
    
    // Create SimpleSetupPtr.
    oc::SimpleSetupPtr ss = std::make_shared<oc::SimpleSetup>(cspace);

    //TODO: figure out how to set the validity checker correctly
    //(following create car method as of now, pretty sure wrong)
    ob::SpaceInformationPtr si = ss->getSpaceInformation();

    si->setStateValidityChecker(std::bind(isValidStatePoint, std::placeholders::_1));
    si->setup();

    //propogate with the ODE function
    //TODO: need to figure out how to pass torque into controls?
    auto odeSolver(std::make_shared<oc::ODEBasicSolver<>>(ss->getSpaceInformation(), &pendulumODE));

    //TODO: figure out how to set bounds here (more info in createCar comments)
    ss->setStatePropagator(oc::ODESolver::getStatePropagator(odeSolver));

    //TODO: confirm start and goal states okay for environment:
    ob::ScopedState<ob::RealVectorStateSpace> start(space);
    start->setX(-5.0);
    start->setY(0.0);
    start->setYaw(0.0);
  
    ob::ScopedState<ob::RealVectorStateSpace> goal(space);
    goal->setX(2.0);
    goal->setY(0.0);
    goal->setYaw(0.0);

    //TODO: check goal if goal region/radius okay
    ss->setStartAndGoalStates(start, goal, 0.05);

    ss->setup();

    return ss;
}

void planPendulum(oc::SimpleSetupPtr & ss, int choice)
{
    /*
        kept this one the same as the car plan function
    */

    //set the planner based on choice:
    if (choice == 1)
    {
        ss->setPlanner(std::make_shared<oc::RRT>(ss->getSpaceInformation()));
    }
    if (choice == 2)
    {
        ss->setPlanner(std::make_shared<oc::KPIECE1>(ss->getSpaceInformation()));
    }

    //comment out RGRRT for now- wait until we implement bc it causes comiler errors rn
    /*if (choice == 3)
    {
        ss->setPlanner(std::make_shared<oc::RGRRT>(ss->getSpaceInformation()));
    }*/

    //solve the problem:
    base::PlannerStatus solved = ss->solve(10.0);

    if (solved)
    {
        std::cout << "Found Solution:" << std::endl;
        //convert to geometric path so we can nicely print path as matrix
        ss->getSolutionPath().asGeometric().printAsMatrix(std::cout);
    }
    else
    {
        std::cout << "No Solution Found" << std::endl;
    }
}

void benchmarkPendulum(oc::SimpleSetupPtr &/* ss */)
{
    // TODO: Do some benchmarking for the pendulum
}

int main(int /* argc */, char ** /* argv */)
{
    int choice;
    do
    {
        std::cout << "Plan or Benchmark? " << std::endl;
        std::cout << " (1) Plan" << std::endl;
        std::cout << " (2) Benchmark" << std::endl;

        std::cin >> choice;
    } while (choice < 1 || choice > 2);

    int which;
    do
    {
        std::cout << "Torque? " << std::endl;
        std::cout << " (1)  3" << std::endl;
        std::cout << " (2)  5" << std::endl;
        std::cout << " (3) 10" << std::endl;

        std::cin >> which;
    } while (which < 1 || which > 3);

    double torques[] = {3., 5., 10.};
    double torque = torques[which - 1];

    oc::SimpleSetupPtr ss = createPendulum(torque);

    // Planning
    if (choice == 1)
    {
        int planner;
        do
        {
            std::cout << "What Planner? " << std::endl;
            std::cout << " (1) RRT" << std::endl;
            std::cout << " (2) KPIECE1" << std::endl;
            std::cout << " (3) RG-RRT" << std::endl;

            std::cin >> planner;
        } while (planner < 1 || planner > 3);

        planPendulum(ss, planner);
    }
    // Benchmarking
    else if (choice == 2)
        benchmarkPendulum(ss);

    else
        std::cerr << "How did you get here? Invalid choice." << std::endl;

    return 0;
}
