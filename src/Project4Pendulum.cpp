///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 4
// Authors: Peyton Elebash and Swaha Roy
//////////////////////////////////////

#include <iostream>
#include <fstream>


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

// need for benchmarking
#include "ompl/tools/benchmark/Benchmark.h"

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
        return 1;
    }

    void project(const ob::State * state, Eigen::Ref<Eigen::VectorXd> projection) const override
    {
        const auto *cmpd = state->as<ob::CompoundStateSpace::StateType>();
        const auto *so2state = cmpd->as<ob::SO2StateSpace::StateType>(0);
        
        //grabbing theta from so2->value
        projection(0) = so2state->value;
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
    const double *u = control->as<oc::RealVectorControlSpace::ControlType>()->values;
    const double torque = u[0];

    // Retrieve current orientation: q[0] = theta, q[1] = w
    const double theta = q[0];
    const double w = q[1];

    // Ensure qdot same size as q. Zero out all values. 
    // q = (theta, w), qdot = (thetadot, wdot)
    qdot.resize(q.size(), 0);

    // System dynamics: qdot = (thetadot, wdot) = (w, -gcos(theta) + torque)
    //
    qdot[0] = w;
    qdot[1] = ((-9.81)*(cos(theta)) + torque);
}

// ADDED FUNCTION
// Just like car setup except we don't need to check for obstacle collision since no obstacles
// Still should check bounds though
bool isValidStatePendulum(const ob::State *state, const oc::SpaceInformation *si)
{
    // No obstacles, just need to check the bounds we set when setting up the space
    return si->satisfiesBounds(state);
}

void PostIntegration (const ob::State* /*state*/, const oc::Control* /*control*/, const double /*duration*/, ob::State *result)
{
    // Normalize orientation between 0 and 2*pi
    ob::SO2StateSpace SO2;
    SO2.enforceBounds(result->as<ob::CompoundStateSpace::StateType>()->as<ob::SO2StateSpace::StateType>(1));
}

oc::SimpleSetupPtr createPendulum(double torque)
{
    // Create pendulum's state space: SO(2) X R
    //auto space(std::make_shared<ob::SO2StateSpace>());

    const auto so2 = std::make_shared<ob::SO2StateSpace>();
    const auto r = std::make_shared<ob::RealVectorStateSpace>(1);
    const auto space = so2 + r;

    // setting bounds for r state space:
    ob::RealVectorBounds rbounds(1);
    // rotational velocity
    rbounds.setLow(-10);
    rbounds.setHigh(10);
    r->setBounds(rbounds);
    
    //control space setup:
    auto cspace(std::make_shared<oc::RealVectorControlSpace>(space, 1));

    // Set control space bounds.
    ob::RealVectorBounds cbounds(1);
    cbounds.setLow(-1 * torque);
    cbounds.setHigh(torque);
    cspace->setBounds(cbounds);
    
    // Create SimpleSetupPtr.
    oc::SimpleSetupPtr ss = std::make_shared<oc::SimpleSetup>(cspace);

    oc::SpaceInformationPtr si = ss->getSpaceInformation();

    // Fixing pendulum alidity checker to work similar the car one
    ss->setStateValidityChecker([si](const ob::State *state) {return isValidStatePendulum(state, si.get()); });

    //propogate with the ODE function
    auto odeSolver(std::make_shared<oc::ODEBasicSolver<>>(ss->getSpaceInformation(), &pendulumODE));

    // Set propgator with post integration to enforce that theta is in [0, 2pi].
    ss->setStatePropagator(oc::ODESolver::getStatePropagator(odeSolver,  &PostIntegration));
  

    // Set the start and goal states (-pi/2, 0) --> (pi/2, 0)
    ob::ScopedState<ob::CompoundStateSpace> start(space);
    start[0] = -3.14159/2;
    start[1] = 0.0;
  
    ob::ScopedState<ob::CompoundStateSpace> goal(space);
    goal[0] =  3.14159/2;
    goal[1] = 0.0;

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
        // If KPIECE, must set the projection (details in planCar)
        ss->getStateSpace()->registerDefaultProjection(ob::ProjectionEvaluatorPtr(new PendulumProjection(ss->getStateSpace().get())));
        ss->setPlanner(std::make_shared<oc::KPIECE1>(ss->getSpaceInformation()));
    }

    //RGRRT:
    if (choice == 3)
    {
        ss->setPlanner(std::make_shared<oc::RGRRT>(ss->getSpaceInformation()));
    }

    //solve the problem:
    ob::PlannerStatus solved = ss->solve(10.0);

    if (solved)
    {
        std::cout << "Found Solution:" << std::endl;

        auto path = ss->getSolutionPath().asGeometric();
        path.printAsMatrix(std::cout);

        // Capture output in path.txt file
        std::ofstream pathOutput("path.txt");
        path.printAsMatrix(pathOutput);
        pathOutput.close();
    }
    else
    {
        std::cout << "No Solution Found" << std::endl;
    }
}

void benchmarkPendulum(oc::SimpleSetupPtr & ss)
{
    // TODO: Do some benchmarking for the car
    // Create benchmark class
    // Had to dereference to work
    ompl::tools::Benchmark b(*ss, "my experiment");

    // For KPIECE:
    ss->getStateSpace()->registerDefaultProjection(ob::ProjectionEvaluatorPtr(new PendulumProjection(ss->getStateSpace().get())));

    // Add planners
    b.addPlanner(ob::PlannerPtr(new oc::RRT(ss->getSpaceInformation())));
    b.addPlanner(ob::PlannerPtr(new oc::KPIECE1(ss->getSpaceInformation())));
    b.addPlanner(ob::PlannerPtr(new oc::RGRRT(ss->getSpaceInformation())));

    // Create benchmark request
    ompl::tools::Benchmark::Request req;
    req.maxTime = 30.0;
    req.maxMem = 100.0;
    req.runCount = 20;
    req.displayProgress = true;
    b.benchmark(req);

    // Generate file
    b.saveResultsToFile();
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
