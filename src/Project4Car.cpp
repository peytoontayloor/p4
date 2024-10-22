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

// do we need to include?
#include <ompl/base/spaces/RealVectorStateSpace.h>

//need includes for the planners too!
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>

namespace ob = ompl::base;
namespace oc = ompl::control;

// Your projection for the car
class CarProjection : public ob::ProjectionEvaluator
{
public:
    CarProjection(const ob::StateSpace *space) : ProjectionEvaluator(space)
    {
    }

    unsigned int getDimension() const override
    {
        // TODO: The dimension of your projection for the car
        return 0;
    }

    void project(const ob::State * /* state */, Eigen::Ref<Eigen::VectorXd> /* projection */) const override
    {
        // TODO: Your projection for the car
    }
};

/*
    carODE described the ODE of a simple car dynamic system and stores the output
    of applying the control to the current state in qdot.

    parameters: 
    vector q, which describes current state of the system
    control u, which defines the inputs applied to sys at state q
    vector qdot, which is where the output of the computation is stored

    Followed steps for implementing this from: https://ompl.kavrakilab.org/odeint.html
    also from: https://ompl.kavrakilab.org/RigidBodyPlanningWithODESolverAndControls_8cpp_source.html

*/
void carODE(const oc::ODESolver::StateType & q, const oc::Control * control,
            oc::ODESolver::StateType & qdot)
{
    // Retrieve control values: w (angular velocity), vdot (forward acceleration).
    const double *u = control->as<oc::RealVectorControlSpace::ControlType>()->values;
    const double w = u[0];
    const double vdot = u[1];

    // Retrieve current orientation of car: q[0] = x, q[1] = y, q[2] = theta, q[3] = v
    const double theta = q[2];
    const double v = q[3];

    // Ensure qdot same size as q. Zero out all values. 
    // q = (x, y, theta, v), qdot = (xdot, ydot, w, vdot)
    qdot.resize(q.size(), 0);

    // System dynamics: qdot = (xdot, ydot, w, vdot) = (vcos(theta), vsin(theta), w, v)
    qdot[0] = v * cos(theta);
    qdot[1] = v * sin(theta);
    qdot[2] = w;
    qdot[3] = vdot;
}

void makeStreet(std::vector<Rectangle> & obstacles)
{
    // Dimensions from Project 4, Figure 2.

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

// ADDED THIS FUNCTION
// Intersect the point (x,y) with the set of rectangles. If the point lies outside of all obstacles 
// and velocity/acceleration within cspace bounds, return true.
bool isValidStatePointCar(const ob::State *state, const oc::SpaceInformation *si, const std::vector<Rectangle>& obstacles)
{ 
    const auto *se2state = state->as<ob::SE2StateSpace::StateType>();
    const auto *pos = se2state->as<ob::RealVectorStateSpace::StateType>(0)->values;
    const double x =  pos[0];
    const double y = pos[1];

    // Checks if x, y, theta, and velocity are within set bounds and checks for obstacle collision
    return  si->satisfiesBounds(state) && isValidStatePoint(x, y, obstacles);

}

// ADDED THIS FUNCTION
void PostIntegration (const ob::State* /*state*/, const oc::Control* /*control*/, const double /*duration*/, ob::State *result)
{
    // Normalize orientation between 0 and 2*pi
    ob::SO2StateSpace SO2;
    SO2.enforceBounds(result->as<ob::SE2StateSpace::StateType>()->as<ob::SO2StateSpace::StateType>(1));
}

/*
    Set up simple setup ptr for car.

    OMPL demo reference: https://ompl.kavrakilab.org/RigidBodyPlanningWithODESolverAndControls_8cpp_source.html
*/
oc::SimpleSetupPtr createCar(std::vector<Rectangle> & obstacles)
{
    // Create car's state space.
    //auto space(std::make_shared<ob::SE2StateSpace>());

    const auto se2 = std::make_shared<ob::SE2StateSpace>();
    const auto r = std::make_shared<ob::RealVectorStateSpace>(1);
    const auto space = se2 + r;

    
    // Set bounds to 3, need to set the bounds for each state space individually
    // SE(2) bounds:
    ob::RealVectorBounds se2bounds(2);
    // x
    se2bounds.setLow(0, -5);
    se2bounds.setHigh(0, 5);
    // y
    se2bounds.setLow(1, -10);
    se2bounds.setHigh(1, 10);
    // theta
    // TODO: don't need to set theta I think!
    //se2bounds.setLow(-pi/2)
    //se2bounds.setHigh(pi/2)
    se2->setBounds(se2bounds);

    //R bounds:
    ob::RealVectorBounds rbounds(1);
    // forward velocity
    // TODO: find what these values should actually be
    rbounds.setLow()
    rbounds.setHigh()
    r->setBounds(rbounds);

    // Create a control space.
    auto cspace(std::make_shared<oc::RealVectorControlSpace>(space, 2));

    // Set the control space bounds (axis 0 - angular velocity, axis 1 - forward acceleration)
    // TODO: check, used average max angular velocity + forward acceleration found online
    ob::RealVectorBounds cbounds(2);
    cbounds.setLow(0, -0.3);
    cbounds.setHigh(0, 0.3);
    cbounds.setLow(1, -5);
    cbounds.setHigh(1, 5);
    cspace->setBounds(cbounds);

    // Create simple ssetup ptr. 
    oc::SimpleSetupPtr ss = std::make_shared<oc::SimpleSetup>(cspace);

    // Get the space information pointer from ss.
    oc::SpaceInformationPtr si = ss->getSpaceInformation();

    // Set validity checker for point car
    ss->setStateValidityChecker(
         [si, obstacles](const ob::State *state) { return isValidStatePointCar(state, si.get(), obstacles); });

    // Use the odeSolver to propagate:
    auto odeSolver(std::make_shared<oc::ODEBasicSolver<>>(ss->getSpaceInformation(), &carODE));

    // Set propgator with post integration to enforce that theta is in [0, 2pi].
    ss->setStatePropagator(oc::ODESolver::getStatePropagator(odeSolver,  &PostIntegration));
  
    //TODO: check, Set start and goal states based on Project 4, Figure 2
    ob::ScopedState<ob::SE2StateSpace> start(space);
    start->setX(-4.5);
    start->setY(-5.0);
    start->setYaw(0.0);
  
    ob::ScopedState<ob::SE2StateSpace> goal(space);
    goal->setX(4.5);
    goal->setY(5.0);
    goal->setYaw(0.0);
    
    //TODO: verify goal region radius okay, this is from demo again so not sure if applies to our environment
    ss->setStartAndGoalStates(start, goal, 0.05);
  
    ss->setup();

    return ss;
}

/*
    choice is what planner to choose:
    if 1 --> RRT
    if 2 --> KPIECE
    if 3 --> RG-RRT

    given a simple setup pointer (ss) and a planner (choice), set up a motion planning problem
*/
void planCar(oc::SimpleSetupPtr & ss, int choice)
{
    // Set the planner based on choice.
    if (choice == 1)
    {
        //setPlanner requires a PlannerPtr not just a Planner
        //ss->setPlanner(new oc::RRT(ss->getSpaceInformation()));
        //follow similar method as SimpleSetupPtr, i.e., std::make_shared<Planner> gives a PlannerPtr

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

    // Solve the problem.
    ob::PlannerStatus solved = ss->solve(10.0);

    if (solved)
    {
        std::cout << "Found Solution:" << std::endl;
        
        // Convert to geometric path so we can nicely print path as matrix.
        ss->getSolutionPath().asGeometric().printAsMatrix(std::cout);
    }
    else
    {
        std::cout << "No Solution Found" << std::endl;
    }

}

void benchmarkCar(oc::SimpleSetupPtr &/* ss */)
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

    oc::SimpleSetupPtr ss = createCar(obstacles);

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