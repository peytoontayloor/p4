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

// need for benchmarking
#include "ompl/tools/benchmark/Benchmark.h"

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
        // Following the demo at:
        //https://ompl.kavrakilab.org/projections.html

        // Here we want to say what dimension are going to project the car into
        return 2;
    }

    void project(const ob::State * state, Eigen::Ref<Eigen::VectorXd> projection) const override
    {
        // Projecting x from se2 into pos0 and y from se2 into pos1 

        const auto *cmpd = state->as<ob::CompoundStateSpace::StateType>();
        const auto *se2state = cmpd->as<ob::SE2StateSpace::StateType>(0);

        // Can use getX and getY from SE2StateSpace (makes life easier lol)
        projection(0) = se2state->getX();
        projection(1) = se2state->getY();

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
    r2.height = 8;

    r3.x = 2;
    r3.y = -4;
    r3.width = 3;
    r3.height = 8;

    r4.x = -5;
    r4.y = 6;
    r4.width = 10;
    r4.height = 2;

    obstacles.push_back(r1);
    obstacles.push_back(r2);
    obstacles.push_back(r3);
    obstacles.push_back(r4);
}

// Intersect the point (x,y) with the set of rectangles. If the point lies outside of all obstacles 
// and velocity/acceleration within cspace bounds, return true.
bool isValidStatePointCar(const ob::State *state, const oc::SpaceInformation *si, const std::vector<Rectangle>& obstacles)
{ 
    const auto *se2state = state->as<ob::CompoundStateSpace::StateType>()->as<ob::SE2StateSpace::StateType>(0);

    // Using getX and getY from SE2StateSpace
    const double x = se2state->getX();
    const double y = se2state->getY();

    // Checks if x, y, theta, and velocity are within set bounds and checks for obstacle collision
    return  si->satisfiesBounds(state) && isValidStatePoint(x, y, obstacles);

}


void PostIntegration (const ob::State* /*state*/, const oc::Control* /*control*/, const double /*duration*/, ob::State *result)
{
    // Normalize orientation between 0 and 2*pi
    ob::SO2StateSpace SO2;
    SO2.enforceBounds(result->as<ob::CompoundStateSpace::StateType>()->as<ob::SE2StateSpace::StateType>(0)->as<ob::SO2StateSpace::StateType>(1));
}

/*
    Set up simple setup ptr for car.

    OMPL demo reference: https://ompl.kavrakilab.org/RigidBodyPlanningWithODESolverAndControls_8cpp_source.html
*/
oc::SimpleSetupPtr createCar(std::vector<Rectangle> & obstacles)
{
    // Create car's state space.
    const auto se2 = std::make_shared<ob::SE2StateSpace>();
    const auto r = std::make_shared<ob::RealVectorStateSpace>(1);
    const auto space = se2 + r;

    
    // SE(2) bounds:
    ob::RealVectorBounds se2bounds(2);
    // x
    se2bounds.setLow(0, -5);
    se2bounds.setHigh(0, 5);
    // y
    se2bounds.setLow(1, -10);
    se2bounds.setHigh(1, 10);

    se2->setBounds(se2bounds);

    //R bounds:
    ob::RealVectorBounds rbounds(1);
    // forward velocity
    rbounds.setLow(-10);
    rbounds.setHigh(10);
    r->setBounds(rbounds);

    // Create a control space.
    auto cspace(std::make_shared<oc::RealVectorControlSpace>(space, 2));

    // Set the control space bounds (axis 0 - angular velocity, axis 1 - forward acceleration) // TODO: might need to change these to smaller values to fix collision issues
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
  
    // Set start and goal states based on Project 4, Figure 2
    ob::ScopedState<ob::CompoundStateSpace> start(space);
    start[0] = -4.5; // x
    start[1] = -5.5; // y
    start[2] = 0.0; // yaw 
    start[3] = 0.0; // v


    ob::ScopedState<ob::CompoundStateSpace> goal(space);
    goal[0] = 4.5; // x
    goal[1] = 5.0; // y
    goal[2] = 0.0; // yaw 
    goal[3] = 0.0; // v
  
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
        ss->setPlanner(std::make_shared<oc::RRT>(ss->getSpaceInformation()));
    }
    if (choice == 2)
    {
        // If KPIECE, register projection.
        // Need to get state space pointer from the simple set up and then need to get the actual state space with get
        ss->getStateSpace()->registerDefaultProjection(ob::ProjectionEvaluatorPtr(new CarProjection(ss->getStateSpace().get())));
        ss->setPlanner(std::make_shared<oc::KPIECE1>(ss->getSpaceInformation()));
    }

    //RGRRT,
    if (choice == 3)
    {
        ss->setPlanner(std::make_shared<oc::RGRRT>(ss->getSpaceInformation()));
    }

    // Solve the problem.
    ob::PlannerStatus solved = ss->solve(10.0);

    if (solved)
    {
        std::cout << "Found Solution:" << std::endl;
        
        // Convert to geometric path so we can nicely print path as matrix.
        // auto path = ss->getSolutionPath().asGeometric().printAsMatrix(std::cout);

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

void benchmarkCar(oc::SimpleSetupPtr & ss)
{
    // Create benchmark class
    ompl::tools::Benchmark b(*ss, "my experiment");

    // For KPIECE:
    ss->getStateSpace()->registerDefaultProjection(ob::ProjectionEvaluatorPtr(new CarProjection(ss->getStateSpace().get())));

    // Add planners
    b.addPlanner(ob::PlannerPtr(new oc::RRT(ss->getSpaceInformation())));
    b.addPlanner(ob::PlannerPtr(new oc::KPIECE1(ss->getSpaceInformation())));
    b.addPlanner(ob::PlannerPtr(new oc::RGRRT(ss->getSpaceInformation())));

    // Create benchmark request
    ompl::tools::Benchmark::Request req;
    req.maxTime = 100.0;
    req.maxMem = 100.0;
    req.runCount = 50;
    req.displayProgress = true;
    b.benchmark(req);

    // Generate file
    b.saveResultsToFile();
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