///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 4
// Authors: FILL ME OUT!!
//////////////////////////////////////

#include <iostream>

#include <ompl/base/ProjectionEvaluator.h>

#include <ompl/control/SimpleSetup.h>
#include <ompl/control/ODESolver.h>

// The collision checker routines
#include "CollisionChecking.h"

// Your implementation of RG-RRT
#include "RG-RRT.h"

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

    //TODO: set the velocity and acceleration bounds (not sure if happends here)

    //first grab the control inputs, w (angular velocity) and v (acceleration of vehicle):
    const double *controls = u->as<ompl::control::RealVectorControlSpace::ControlType>()->values;
    const double w = controls[0];
    const double vdot = controls[1];

    //get the current orientation of car: q[0] = x, q[1] = y, q[2] = theta, q[3] = v
    const double theta = q[2];
    //q = (x, y, theta, v) and qdot = (x, y, w, v)
    //ensure qdot same size as q and zero out all values (still not sure abt this, got from ompl documentation)
    qdot.resize(q.size(), 0);

    //our car system looks like:
    //qdot = (w, y, w, v) = (vcos(theta), vsin(theta), w, v)
    qdot[0] = v * cos(theta);
    qdot[1] = v * sin(theta);
    qdot[2] = w;
    qdot[3] = v;

}

void makeStreet(std::vector<Rectangle> & obstacles)
{
    // TODO: Fill in the vector of rectangles with your street environment.
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

ompl::control::SimpleSetupPtr createCar(std::vector<Rectangle> & /* obstacles */)
{
    // TODO: Create and setup the car's state space, control space, validity checker, everything you need for planning.
    return nullptr;
}

void planCar(ompl::control::SimpleSetupPtr &/* ss */, int /* choice */)
{
    // TODO: Do some motion planning for the car
    // choice is what planner to use.
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
