///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 4
// Authors: Peyton Elebash, Swaha Roy and Audrey Lu
//////////////////////////////////////

#include <iostream>

#include <ompl/base/ProjectionEvaluator.h>

#include <ompl/control/SimpleSetup.h>
#include <ompl/control/ODESolver.h>

// The collision checker routines
#include "CollisionChecking.h"

// Your implementation of RG-RRT
#include "RG-RRT.h"

//includes from the demo documentation that I think are necessary:
#include <ompl/control/SpaceInformation.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/control/ODESolver.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/config.h>

//need includes for the planners too!
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>


// Your projection for the car
class CarProjection : public ompl::base::ProjectionEvaluator
{
public:
    CarProjection(const ompl::base::StateSpace *space) : ProjectionEvaluator(space)
    {
    }

    unsigned int getDimension() const override
    {
        // TODO: The dimension of your projection for the car
        return 0;
    }

    void project(const ompl::base::State * /* state */, Eigen::Ref<Eigen::VectorXd> /* projection */) const override
    {
        // TODO: Your projection for the car
    }
};

void carODE(const ompl::control::ODESolver::StateType & q, const ompl::control::Control * control,
            ompl::control::ODESolver::StateType & qdot)
{
    /*
        parameters: 
        vector q, which describes current state of the system
        control u, which defines the inputs applied to sys at state q
        vector qdot, which is where the output of the computation is stored

        Followed steps for implementing this from: https://ompl.kavrakilab.org/odeint.html
        also from: https://ompl.kavrakilab.org/RigidBodyPlanningWithODESolverAndControls_8cpp_source.html

    */

    //first grab the control inputs, w (angular velocity) and v (acceleration of vehicle):
    const double *u = control->as<ompl::control::RealVectorControlSpace::ControlType>()->values;
    const double w = u[0];
    const double vdot = u[1];

    //get the current orientation of car: q[0] = x, q[1] = y, q[2] = theta, q[3] = v
    const double theta = q[2];
    const double v = q[3];
    //q = (x, y, theta, v) and qdot = (x, y, w, v)
    //ensure qdot same size as q and zero out all values (still not sure abt this, got from ompl documentation)
    qdot.resize(q.size(), 0);

    //our car system looks like:
    //qdot = (w, y, w, v) = (vcos(theta), vsin(theta), w, v)
    qdot[0] = v * cos(theta);
    qdot[1] = v * sin(theta);
    qdot[2] = w;
    qdot[3] = vdot;

}

void makeStreet(std::vector<Rectangle> & obstacles)
{
    //found dimensions for this environment from the project spec

    Rectangle r1, r2, r3, r4;

    r1.x = -5;
    r1.y = -10;
    r1.width = 10;
    r1.height = 2;

    r2.x = -5;
    r2.y = -4;
    r2.width = 4;
    r2.height = 10;

    r3.x = 2;
    r3.y = -4;
    r3.width = 3;
    r3.height = 10;

    r4.x = -5;
    r4.y = 6;
    r4.width = 10;
    r4.height = 2;

    obstacles.push_back(r1);
    obstacles.push_back(r2);
    obstacles.push_back(r3);
    obstacles.push_back(r4);

}

ompl::control::SimpleSetupPtr createCar(std::vector<Rectangle> & obstacles)
{
    /*
        Followed along with ompl demo as much as I could: https://ompl.kavrakilab.org/RigidBodyPlanningWithODESolverAndControls_8cpp_source.html
    */

    //create car's state space:

    auto space(std::make_shared<ompl::base::SE2StateSpace>());

    //TODO: investigate if these are correct for the environment given in the assignment (might want to set specifically for each dimension)
    ompl::base::RealVectorBounds bounds(2);
    bounds.setLow(-10);
    bounds.setHigh(10);
    space->setBounds(bounds);

    //setting the control space up:

    //TODO: check that the dimensions are correct
    auto cspace(std::make_shared<ompl::control::RealVectorControlSpace>(space, 2));

    //set the control space bounds:

    //TODO: see if need multiple high and low bounds for each dimension/variable in the control space
    //right now setting them to the numbers seen on the demo
    //project spec says: "the car cannot source an arbitrarily large acceleration or angular velocity, you need to set bounds in the control space to ensure a dynamically feasible trajectory"
    ompl::base::RealVectorBounds cbounds(2);
    cbounds.setLow(-0.3);
    cbounds.setHigh(0.3);
    cspace->setBounds(cbounds);

    //set up our simple setup class:
    //TODO: needs to be of type SimpleSetupPtr when returning, investigate how to do this
    //making it a simple setup ptr instead of just simple setup the same as we did with space information ptr in project 3
    //ompl::control::SimpleSetupPtr ss(cspace);
    ompl::control::SimpleSetupPtr ss = std::make_shared<ompl::control::SimpleSetup>(cspace);

    //TODO: figure out how to set the validity checker correctly

    //first get the space information pointer from ss:
    ompl::base::SpaceInformationPtr si = ss->getSpaceInformation();


    //TODO: not sure how to do this yet- I think we need to pull out just the SE(2) space from our space information?

    //below is following exactly like how we did in project 3, but thinking this is wrong
    //the placeholder is for the state, but we want just state space, so maybe want to define?

    //something that the documentation does is: (which I think does what we want, just not sure how to integrate here)
    //const auto *se2state = state->as<ob::SE2StateSpace::StateType>();
    //const auto *pos = se2state->as<ob::RealVectorStateSpace::StateType>(0);
    //const auto *rot = se2state->as<ob::SO2StateSpace::StateType>(1);

    //project spec says: "you must verify that the velocity of the vehicle is within the bounds you set"
    //TODO: compiler error with linking bc not correct parameters for isValidStateSquare
    si->setStateValidityChecker(std::bind(isValidStateSquare, std::placeholders::_1, 1, obstacles));
    si->setup();

    //use the odeSolver to propagate:
    auto odeSolver(std::make_shared<ompl::control::ODEBasicSolver<>>(ss->getSpaceInformation(), &carODE));

    //TODO: investigate  &KinematicCarPostIntegration from demo!
    //it seems to be enforcing rotation constraints on the car
    //we probably want it to enforce constraints, but it is an optional feature, so removing for now
    //including the demo example commented out below:
    //ss.setStatePropagator(oc::ODESolver::getStatePropagator(odeSolver, &KinematicCarPostIntegration));
    ss->setStatePropagator(ompl::control::ODESolver::getStatePropagator(odeSolver));
  
    //TODO: check start and goal states are okay in our environment roughly based off of project spec car environment
    ompl::base::ScopedState<ompl::base::SE2StateSpace> start(space);
    start->setX(-4.0);
    start->setY(-5.0);
    start->setYaw(0.0);
  
    ompl::base::ScopedState<ompl::base::SE2StateSpace> goal(space);
    goal->setX(4.0);
    goal->setY(5.0);
    goal->setYaw(0.0);
    
    //TODO: verify goal region radius okay, this is from demo again so not sure if applies to our environment
    ss->setStartAndGoalStates(start, goal, 0.05);
  
    ss->setup();

    return ss;
}

void planCar(ompl::control::SimpleSetupPtr & ss, int choice)
{
    /*
        choice is what planner to choose:
        if 1 --> RRT
        if 2 --> KPIECE
        if 3 --> RG-RRT

        given a simple setup pointer (ss) and a planner (choice), set up a motion planning problem
    */

    //set the planner based on choice:
    if (choice == 1)
    {
        //setPlanner requires a PlannerPtr not just a Planner
        //ss->setPlanner(new ompl::control::RRT(ss->getSpaceInformation()));
        //follow similar method as SimpleSetupPtr, i.e., std::make_shared<Planner> gives a PlannerPtr

        ss->setPlanner(std::make_shared<ompl::control::RRT>(ss->getSpaceInformation()));
    }
    if (choice == 2)
    {
        ss->setPlanner(std::make_shared<ompl::control::KPIECE1>(ss->getSpaceInformation()));
    }

    //comment out RGRRT for now- wait until we implement bc it causes comiler errors rn
    /*if (choice == 3)
    {
        ss->setPlanner(std::make_shared<ompl::control::RGRRT>(ss->getSpaceInformation()));
    }*/

    //solve the problem:
    ompl::base::PlannerStatus solved = ss->solve(10.0);

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

void benchmarkCar(ompl::control::SimpleSetupPtr &/* ss */)
{
    // TODO: Do some benchmarking for the car
}

int main(int /* argc */, char ** /* argv */)
{
    std::vector<Rectangle> obstacles;
    makeStreet(obstacles);

    int choice;
    do
    {
        std::cout << "Plan or Benchmark? " << std::endl;
        std::cout << " (1) Plan" << std::endl;
        std::cout << " (2) Benchmark" << std::endl;

        std::cin >> choice;
    } while (choice < 1 || choice > 2);

    ompl::control::SimpleSetupPtr ss = createCar(obstacles);

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

        planCar(ss, planner);
    }
    // Benchmarking
    else if (choice == 2)
        benchmarkCar(ss);

    else
        std::cerr << "How did you get here? Invalid choice." << std::endl;

    return 0;
}
