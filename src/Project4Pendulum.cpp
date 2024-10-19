///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 4
// Authors: FILL ME OUT!!
//////////////////////////////////////

#include <iostream>

#include <ompl/base/ProjectionEvaluator.h>

#include <ompl/control/SimpleSetup.h>
#include <ompl/control/ODESolver.h>

// Your implementation of RG-RRT
#include "RG-RRT.h"

//includes from the demo documentation that I think are necessary:
 #include <ompl/control/SpaceInformation.h>
 #include <ompl/base/spaces/RealVectorStateSpace.h>
 #include <ompl/control/ODESolver.h>
 #include <ompl/control/spaces/RealVectorControlSpace.h>
 #include <ompl/control/SimpleSetup.h>
 #include <ompl/config.h>

// Your projection for the pendulum
class PendulumProjection : public ompl::base::ProjectionEvaluator
{
public:
    PendulumProjection(const ompl::base::StateSpace *space) : ProjectionEvaluator(space)
    {
    }

    unsigned int getDimension() const override
    {
        // TODO: The dimension of your projection for the pendulum
        return 0;
    }

    void project(const ompl::base::State */* state */, Eigen::Ref<Eigen::VectorXd> /* projection */) const override
    {
        // TODO: Your projection for the pendulum
    }
};

void pendulumODE(const ompl::control::ODESolver::StateType & q, const ompl::control::Control * control,
                 ompl::control::ODESolver::StateType & qdot)
{
    /*
        state of system q = (theta, w), theta is orientation of pendulum and w is rotational velocity

        torque is t (from main) -->might be a control input

        gravity is roughly 9.81 per the project spec

        qdot = (w, -gcos(theta)+t)
    */

    const double *controls = u->as<ompl::control::RealVectorControlSpace::ControlType>()->values;
    const double torque = controls[0];

    qdot.resize(q.size(), 0);

    //q[0] = theta, q[1] = w
    qdot[0] = q[1]; //w
    qdot[1] = ((-9.81)*(cos(q[0])) + torque)
}

ompl::control::SimpleSetupPtr createPendulum(double torque)
{
    /*
       As of now, kept same as car create function, will need to adjust when
       we figure out how to correctly set up.
    */

    //TODO: make sure state space correct! --> if pendulum is dot, R^2, if line, SE(2)?
    //create pendulum's state space: R^2
    auto space(std::make_shared<ompl::base::RealVectorStateSpace>(2));

    //TODO: investigate if these are correct for the environment
    ompl::base::RealVectorBounds bounds(2);
    bounds.setLow(-10);
    bounds.setHigh(10);
    space->setBounds(bounds);

    //anything below this is based off current implementation of createCar, if we make changes to that, will need to update here one too
    
    //control space setup:
    //TODO: verify we only need "1" for 1 control input, torque
    auto cspace(std::make_shared<ompl::control::RealVectorControlSpace>(space, 1));

    //set control space bounds
    //TODO: double check that this is where we set the control input bounds (bounds are [-torque, torque])

    ompl::base::RealVectorBounds cbounds(1);
    cbounds.setLow(-torque);
    cbounds.setHigh(torque);
    cspace->setBounds(cbounds);
    
    //initialize our simple setup class:
    //TODO: needs to be of type SimpleSetupPtr when returning, investigate how to do this
    ompl::control::SimpleSetup ss(cspace);

    //TODO: figure out how to set the validity checker correctly
    //(following create car method as of now, pretty sure wrong)
    ompl::base::SpaceInformationPtr si = ss.getSpaceInformation();

    //TODO: removed obstacles from call bc get to assume environment with no obstacles, not sure if correct syntax
    //project spec says: "for the pendulum system, the validity checker must ensure the angular velocity of the pendulum is within the bounds that you specify"
    si->setStateValidityChecker(std::bind(isValidStatePoint, std::placeholders::_1));
    si->setup();

    //propogate with the ODE function
    //TODO: need to figure out how to pass torque into controls?
    auto odeSolver(std::make_shared<ompl::control::ODEBasicSolver<>>(ss.getSpaceInformation(), &pendulumODE));

    //TODO: figure out how to set bounds here (more info in createCar comments)
    ss.setStatePropagator(ompl::control::ODESolver::getStatePropagator(odeSolver));

    //TODO: confirm start and goal states okay for environment:
    ompl::base::ScopedState<ompl::base::RealVectorStateSpace> start(space);
    start->setX(-5.0);
    start->setY(0.0);
    start->setYaw(0.0);
  
    ompl::base::ScopedState<ompl::base::RealVectorStateSpace> goal(space);
    goal->setX(2.0);
    goal->setY(0.0);
    goal->setYaw(0.0);

    //TODO: check goal if goal region/radius okay
    ss.setStartAndGoalStates(start, goal, 0.05);

    ss.setup();

    return ss;
}

void planPendulum(ompl::control::SimpleSetupPtr & ss, int choice)
{
    /*
        kept this one the same as the car plan function
    */

    //set the planner based on choice:
    //TODO: need to verify if setting planner right:
    if (choice == 1)
    {
        ss.setPlanner(new ompl::control::RRT(ss.getSpaceInformation()));
    }
    if (choice == 2)
    {
        ss.setPlanner(new ompl::control::KPIECE(ss.getSpaceInformation()));
    }
    if (choice == 3)
    {
        ss.setPlanner(new ompl::control::RGRRT(ss.getSpaceInformation()));
    }

    //solve the problem:
    base::PlannerStatus solved = ss.solve(10.0);

    if (solved)
    {
        std::cout << "Found Solution:" << std::endl;
        //convert to geometric path so we can nicely print path as matrix
        ss.getSolutionPath().asGeometric().printAsMatrix(std::cout);
    }
    else
    {
        std::cout << "No Solution Found" << std::endl;
    }
}

void benchmarkPendulum(ompl::control::SimpleSetupPtr &/* ss */)
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

    ompl::control::SimpleSetupPtr ss = createPendulum(torque);

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
