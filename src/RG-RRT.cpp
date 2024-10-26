///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 4
// Authors: Peyton Elebash and Swaha Roy
//////////////////////////////////////

#include "RG-RRT.h"

// Copied over RRT from OMPL as a starting point, added some comments and left comments from original RRT

#include "ompl/control/planners/rrt/RRT.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"
#include <limits>

// ADDED: macro for the steps we will propagate to get R(q)
// TODO: get a good step/time value for here, left as 2 right now
#define FIXEDSTEPS 2

// ADDED: needed for reachables vector for each state in the tree
#include "ompl/base/ScopedState.h"

// ADDED: for setting the control input
#include <ompl/control/spaces/RealVectorControlSpace.h>

// Constructor:
ompl::control::RGRRT::RGRRT(const SpaceInformationPtr &si) : base::Planner(si, "RGRRT")
{
    specs_.approximateSolutions = true;
    siC_ = si.get();

    Planner::declareParam<double>("goal_bias", this, &RGRRT::setGoalBias, &RGRRT::getGoalBias, "0.:.05:1.");
    Planner::declareParam<bool>("intermediate_states", this, &RGRRT::setIntermediateStates, &RGRRT::getIntermediateStates,"0,1");
}

// Destructor:
ompl::control::RGRRT::~RGRRT()
{
    freeMemory();
}

// Set up:
void ompl::control::RGRRT::setup()
{
    // Initializes planner
    base::Planner::setup();

    // Sets up our nn DS storing motions
    if (!nn_)
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    nn_->setDistanceFunction([this](const Motion *a, const Motion *b) { return distanceFunction(a, b); });
}

// Clear:
void ompl::control::RGRRT::clear()
{
    Planner::clear();
    sampler_.reset();
    controlSampler_.reset();
    freeMemory();
    if (nn_)
        nn_->clear();
    lastGoalMotion_ = nullptr;
}

// Frees DS memory
void ompl::control::RGRRT::freeMemory()
{
    if (nn_)
    {
        std::vector<Motion *> motions;
        nn_->list(motions);
        for (auto &motion : motions)
        {
            if (motion->state)
                si_->freeState(motion->state);
            if (motion->control)
                siC_->freeControl(motion->control);

            // ADDED: clearing memory for the reachable set vector:
            motion->reachables.clear();

            delete motion;
        }
    }
}

// Solve:
ompl::base::PlannerStatus ompl::control::RGRRT::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();
    base::Goal *goal = pdef_->getGoal().get();
    auto *goal_s = dynamic_cast<base::GoalSampleableRegion *>(goal);

    if (goal_s == nullptr)
    {
        OMPL_ERROR("%s: Unknown type of goal", getName().c_str());
        return base::PlannerStatus::UNRECOGNIZED_GOAL_TYPE;
    }

    // Start states initialized and added to tree
    while (const base::State *st = pis_.nextStart())
    {
        auto *motion = new Motion(siC_);
        si_->copyState(motion->state, st);
        siC_->nullControl(motion->control);

        // TODO: verify if we need to populate the reachability set here
        // ADDED:
        // see notes in main loop for more detail on what needs to be done here
        // initializing and populating the reachable state for start state(s):
        // TODO: may need to somehow allocate memory to result state?
        base::State *resultState = si_->allocState();
        // Trying new method of setting control values:
        double *controlInpt = (motion->control)->as<ompl::control::RealVectorControlSpace::ControlType>()->values;

        // TODO: verify, i think bounds should be the bounds on the 0th dimension of the control space
        // instead of hardcoding 10
        auto bounds = siC_->getControlSpace()->as<ompl::control::RealVectorControlSpace>()->getBounds();
        stepSize = bounds->getDifference() / 11
        for(double i = bounds.low[0]; i <= bounds.high[0]; i += stepSize)
        {
            // TODO: verify setting control input in pos 0 correctly, no compiler errors but iffy if this is valid
            // got idea for setting it this way from the ODE functions in car and pendulum
            controlInpt[0] = i;

            siC_->propagateWhileValid(motion->state, motion->control, FIXEDSTEPS, resultState);

            motion->reachables.push_back(ompl::base::ScopedState<>(siC_->getStateSpace(), resultState));

        }

        // TODO: do we need to free memory allocated to resultState? or at least do we need to make it empty again?

        nn_->add(motion);

    }

    if (nn_->size() == 0)
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    if (!goal_s->couldSample())
    {
        OMPL_ERROR("%s: Insufficient states in sampleable goal region", getName().c_str());
        return base::PlannerStatus::INVALID_GOAL;
    }

    // Setting up our control and state samplers if hasn't been done yet
    if (!sampler_)
        sampler_ = si_->allocStateSampler();
    if (!controlSampler_)
        controlSampler_ = siC_->allocDirectedControlSampler();

    OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(), nn_->size());

    Motion *solution = nullptr;
    Motion *approxsol = nullptr;
    double approxdif = std::numeric_limits<double>::infinity();

    auto *rmotion = new Motion(siC_);
    base::State *rstate = rmotion->state;
    Control *rctrl = rmotion->control;

    //  ADDED, anywhere new motion, make sure to initialize its reachability as well
    // right now, don't need to populate since this will store different r(q)s depending on what the state is
    std::vector<ompl::base::ScopedState<>> reach = rmotion->reachables;

    base::State *xstate = si_->allocState();

    while (!ptc)
    {
        /* sample random state (with goal biasing) */
        if (rng_.uniform01() < goalBias_ && goal_s->canSample())
            goal_s->sampleGoal(rstate);
        else
            sampler_->sampleUniform(rstate);

        /* find closest state in the tree */
        Motion *nmotion = nn_->nearest(rmotion);

        /* sample a random control that attempts to go towards the random state, and also sample a control duration */
        unsigned int cd = controlSampler_->sampleTo(rctrl, nmotion->control, nmotion->state, rmotion->state);

        // ADDED: populate reachable based on rcontrol and rstate:
        base::State *resultState = si_->allocState();
        // the control should be in [-10, 10], supposed to sample 11 uniformly (start at bottom, increment by 2):
        double *controlInpt = (rctrl)->as<ompl::control::RealVectorControlSpace::ControlType>()->values;

        for(double i = -10; i <= 10; i += 2)
        {
            // TODO: check setting controlInpt right, if make changes, make sure to change other r(q) loops
            // doing torque for pend and u[0] for car, so want to set position 0 of control no matter what
            controlInpt[0] = i;

            // TODO: check if can use propagateWhileValid to collision check instead of just propagate?
            siC_->propagateWhileValid(rstate, rctrl, FIXEDSTEPS, resultState);

            // With using propagateWhileValid, it will put the original state into resultState if the final state is invalid
            // TODO: can we include the state itself in reachable, or should we skip over adding it?

            // adding the reachable state to R(q), only if valid! 
            // TODO: if invalid state, do we need to sample more so we have exactly 11 reachable in set? (I asked this on Piazza hopefully someone responds lol)
            reach.push_back(ompl::base::ScopedState<>(siC_->getStateSpace(), resultState));

        }

        // TODO: do we need to free memory allocated to resultState? or at least do we need to make it empty again?

        if (addIntermediateStates_)
        {
            // If intermediate states, propagate the control in smaller steps, to get intermediate states
            std::vector<base::State *> pstates;
            cd = siC_->propagateWhileValid(nmotion->state, rctrl, cd, pstates, true);

            if (cd >= siC_->getMinControlDuration())
            {
                Motion *lastmotion = nmotion;
                bool solved = false;
                size_t p = 0;
                for (; p < pstates.size(); ++p)
                {
                    /* create a motion */
                    auto *motion = new Motion();
                    motion->state = pstates[p];
                    // we need multiple copies of rctrl
                    motion->control = siC_->allocControl();
                    siC_->copyControl(motion->control, rctrl);

                    // ADDED:
                    base::State *resultState = si_->allocState();
                    // the control should be in [-10, 10], supposed to sample 11 uniformly (start at bottom, increment by 2):
                    double *controlInpt = (motion->control)->as<ompl::control::RealVectorControlSpace::ControlType>()->values;

                    for(double i = -10; i <= 10; i += 2)
                    {
                        controlInpt[0] = i;

                        siC_->propagateWhileValid(rstate, rctrl, FIXEDSTEPS, resultState);

                        motion->reachables.push_back(ompl::base::ScopedState<>(siC_->getStateSpace(), resultState));

                    }

                    // TODO: do we need to free memory allocated to resultState? or at least do we need to make it empty again?


                    motion->steps = 1;
                    motion->parent = lastmotion;
                    lastmotion = motion;
                    nn_->add(motion);
                    double dist = 0.0;
                    solved = goal->isSatisfied(motion->state, &dist);
                    if (solved)
                    {
                        approxdif = dist;
                        solution = motion;
                        break;
                    }
                    if (dist < approxdif)
                    {
                        approxdif = dist;
                        approxsol = motion;
                    }
                }

                // free any states after we hit the goal
                while (++p < pstates.size())
                    si_->freeState(pstates[p]);
                if (solved)
                    break;
            }
            else
                for (auto &pstate : pstates)
                    si_->freeState(pstate);
        }
        else
        {
            if (cd >= siC_->getMinControlDuration())
            {
                /* create a motion */
                // Copying our sampled state and control into a new motion getting aded to the tree
                auto *motion = new Motion(siC_);
                si_->copyState(motion->state, rmotion->state);
                siC_->copyControl(motion->control, rctrl);

                // ADDED:
                // TODO: confirm can correctly copy our r(q) like this
                motion->reachables = reach;
                
                motion->steps = cd;
                motion->parent = nmotion;

                nn_->add(motion);
                double dist = 0.0;
                bool solv = goal->isSatisfied(motion->state, &dist);
                if (solv)
                {
                    approxdif = dist;
                    solution = motion;
                    break;
                }
                if (dist < approxdif)
                {
                    approxdif = dist;
                    approxsol = motion;
                }
            }
        }
    }

    bool solved = false;
    bool approximate = false;
    if (solution == nullptr)
    {
        solution = approxsol;
        approximate = true;
    }

    if (solution != nullptr)
    {
        lastGoalMotion_ = solution;

        /* construct the solution path */
        std::vector<Motion *> mpath;
        while (solution != nullptr)
        {
            mpath.push_back(solution);
            solution = solution->parent;
        }

        /* set the solution path */
        auto path(std::make_shared<PathControl>(si_));
        for (int i = mpath.size() - 1; i >= 0; --i)
            if (mpath[i]->parent)
                path->append(mpath[i]->state, mpath[i]->control, mpath[i]->steps * siC_->getPropagationStepSize());
            else
                path->append(mpath[i]->state);
        solved = true;
        pdef_->addSolutionPath(path, approximate, approxdif, getName());
    }

    if (rmotion->state)
        si_->freeState(rmotion->state);
    if (rmotion->control)
        siC_->freeControl(rmotion->control);

    // ADDED: clearing memory for the reachable set vector:
    rmotion->reachables.clear();
    
    delete rmotion;
    si_->freeState(xstate);

    OMPL_INFORM("%s: Created %u states", getName().c_str(), nn_->size());

    return {solved, approximate};
}

// Gets the DS of the planner:
void ompl::control::RGRRT::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    std::vector<Motion *> motions;
    if (nn_)
        nn_->list(motions);

    double delta = siC_->getPropagationStepSize();

    if (lastGoalMotion_)
        data.addGoalVertex(base::PlannerDataVertex(lastGoalMotion_->state));

    for (auto m : motions)
    {
        if (m->parent)
        {
            if (data.hasControls())
                data.addEdge(base::PlannerDataVertex(m->parent->state), base::PlannerDataVertex(m->state),
                             control::PlannerDataEdgeControl(m->control, m->steps * delta));
            else
                data.addEdge(base::PlannerDataVertex(m->parent->state), base::PlannerDataVertex(m->state));
        }
        else
            data.addStartVertex(base::PlannerDataVertex(m->state));
    }
}