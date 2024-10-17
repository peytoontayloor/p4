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

    //not sure about which are the control inputs? is torque one? 
    //TODO: investigate ^^ to figure out correct ways to grab parts of the problem

    //TODO: may need to remove this
    const double *controls = u->as<ompl::control::RealVectorControlSpace::ControlType>()->values;
    const double theta = controls[0];
    const double w = controls[1];

    qdot.resize(q.size(), 0);

    //q[1] = w, q[0] = theta?

    //TODO: define torque (need to verify if control input)
    qdot[0] = q[1];
    qdot[1] = ((-9.81)*(cos(q[0])) + torque)
    qdot[2] = w;
    qdot[3] = v;
}

ompl::control::SimpleSetupPtr createPendulum(double /* torque */)
{
    /*
       As of now, kept same as car create function, will need to adjust when
       we figure out how to correctly set up.
    */

    //create pendulum's state space: (SE2)
    auto space(std::make_shared<base::SE2StateSpace());

    //TODO: investigate if these are correct for the environment
    base::RealVectorBounds bounds(2);
    bounds.setLow(-10);
    bounds.setHigh(10);
    
    space->setBounds(bounds);

    //TODO: figure out how to set the control space
    //TODO: figure out how to set the validity checker correctly
    //TODO: propogate with the ODE function?
    //TODO: set start and goal states including the radius for the goal region
    //TODO: return the simple setup pointer

    return nullptr;
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
        ss.setPlanner(new ompl::geometric::RRT(ss.getSpaceInformation()));
    }
    if (choice == 2)
    {
        ss.setPlanner(new ompl::geometric::KPIECE(ss.getSpaceInformation()));
    }
    if (choice == 3)
    {
        //TODO: make sure RG-RRT in 'right' format
        ss.setPlanner(new ompl::geometric::RG-RRT(ss.getSpaceInformation()));
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
        std::cout << "No Solution Found" << std:endl;
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
