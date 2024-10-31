///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 4
// Authors: Peyton Elebash and Swaha Roy
//////////////////////////////////////

#include "RG-RRT.h"

#include "ompl/control/planners/rrt/RRT.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"
#include <limits>

// For reachables vector for each state in the tree
#include "ompl/base/ScopedState.h"

// For setting the control input
#include <ompl/control/spaces/RealVectorControlSpace.h>

namespace ob = ompl::base;
namespace oc = ompl::control;

// Constructor:
oc::RGRRT::RGRRT(const SpaceInformationPtr &si) : base::Planner(si, "RGRRT")
{
    specs_.approximateSolutions = true;
    siC_ = si.get();

    Planner::declareParam<double>("goal_bias", this, &RGRRT::setGoalBias, &RGRRT::getGoalBias, "0.:.05:1.");
    Planner::declareParam<bool>("intermediate_states", this, &RGRRT::setIntermediateStates, &RGRRT::getIntermediateStates,"0,1");
}

// Destructor:
oc::RGRRT::~RGRRT()
{
    freeMemory();
}

// Set up:
void oc::RGRRT::setup()
{
    // Initializes planner
    base::Planner::setup();

    // Sets up our nn DS storing motions
    if (!nn_)
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    nn_->setDistanceFunction([this](const Motion *a, const Motion *b) { return distanceFunction(a, b); });
}

// Clear:
void oc::RGRRT::clear()
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
void oc::RGRRT::freeMemory()
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
            motion->reachables.clear();
            delete motion;
        }
    }
}

// Helper function that populates reachability set
void oc::RGRRT::generateReachabilitySet(oc::RGRRT::Motion *motion) {
    // Allocate space state and control
    ob::State *resultState = si_->allocState();
    motion->control = siC_->allocControl();
    // Get bounds of index 0 of control space
    auto bounds = siC_->getControlSpace()->as<oc::RealVectorControlSpace>()->getBounds();
   
    double stepSize = bounds.getDifference()[0] / 11.0;
    OMPL_INFORM("%s: [%lf, %lf], %lf step size", getName().c_str(), bounds.low[0], bounds.high[0], stepSize);

    for(double i = bounds.low[0]; i <= bounds.high[0]; i += stepSize)
    {
        (motion->control)->as<oc::RealVectorControlSpace::ControlType>()->values[0] = i;

        // Propgates forward and performes collision checking       
        int cd = siC_->getMinControlDuration();
        int stepsNoCollision = siC_->propagateWhileValid(motion->state, motion->control, cd, resultState);
        OMPL_INFORM("%s: %d cd, %d steps wo collision", getName().c_str(), cd, stepsNoCollision);


        if (stepsNoCollision > 0) {
            // Only add the last valid state if it is not the start state
            motion->reachables.push_back(ob::ScopedState<>(siC_->getStateSpace(), resultState));
        }
    }
    
    OMPL_INFORM("%s: %u states in reachability set", getName().c_str(), motion->reachables.size());

    // We can safely free the resultState!
    si_->freeState(resultState);
}

// Solve:
ob::PlannerStatus oc::RGRRT::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();
    base::Goal *goal = pdef_->getGoal().get();
    auto *goal_s = dynamic_cast<base::GoalSampleableRegion *>(goal);

    if (goal_s == nullptr)
    {
        OMPL_ERROR("%s: Unknown type of goal", getName().c_str());
        return ob::PlannerStatus::UNRECOGNIZED_GOAL_TYPE;
    }

    // Start states initialized and added to tree
    while (const ob::State *st = pis_.nextStart())
    {
        auto *motion = new Motion(siC_);
        si_->copyState(motion->state, st);
        siC_->nullControl(motion->control);
        // Populate reachability set for start state
        generateReachabilitySet(motion);
        
        OMPL_INFORM("%s: Init start state, %u reachables", getName().c_str(), motion->reachables.size());

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

    // Create motion for the random sample
    auto *rmotion = new Motion(siC_);
    ob::State *rstate = rmotion->state;
    Control *rctrl = rmotion->control;

    auto *nmotion = new Motion(siC_);

    while (!ptc)
    {
        bool expand = false;
        // Sample random states until we find a qrand s.t. there exists a point in R(qnear)
        // closer to qrand than qnear is to qrand
        while (!expand && !ptc) {
             /* sample random state (with goal biasing) */
            if (rng_.uniform01() < goalBias_ && goal_s->canSample())
                goal_s->sampleGoal(rstate); //qrand
            else
                sampler_->sampleUniform(rstate); //qrand
            

            // TEST: see if qrand is not a valid state, shouldn't move toward it? 
            // This decision seemed to lead to less collision prone paths
            // TODO: maybe remove... pretty pretty sure that the sampler has this built in check already
            // if (!(si_->isValid(rstate)))
            // {
            //     continue;
            // }

            // TODO: is it safe to assume qnear is collision free since already in the tree? -> yep! i think so
            nmotion = nn_->nearest(rmotion); //qnear

            // Distance between qnear and qrand
            double distRandToNear = distanceFunction(rmotion, nmotion);
            for (ob::ScopedState<> reachState: nmotion->reachables) {
                double distRandToReach = si_->distance(rmotion->state, reachState.get());
                if (distRandToReach < distRandToNear) {
                    expand = true;
                    break;              
                }
            }
        }
       
        /* sample a random control that attempts to go towards the random state, and also sample a control duration */
        unsigned int cd = controlSampler_->sampleTo(rctrl, nmotion->control, nmotion->state, rmotion->state);
        
        // Populate reachability set for the new state
        OMPL_INFORM("%s: Generate reachability set for qr: %p", getName().c_str(), rmotion);

        // Ensure reachability set is cleared
        rmotion->reachables.clear();
        generateReachabilitySet(rmotion);

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

                    generateReachabilitySet(motion);

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
                // Copying our sampled state and control into a new motion getting added to the tree
                auto *motion = new Motion(siC_);
                si_->copyState(motion->state, rmotion->state);
                siC_->copyControl(motion->control, rctrl);

                // ADDED: solved major bugs yippee!
                motion->reachables = rmotion->reachables;
                
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

    OMPL_INFORM("%s: Created %u states", getName().c_str(), nn_->size());

    return {solved, approximate};
}

// Gets the DS of the planner:
void oc::RGRRT::getPlannerData(base::PlannerData &data) const
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
