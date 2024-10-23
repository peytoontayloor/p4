///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 4
// Authors: Peyton Elebash and Swaha Roy
//////////////////////////////////////

#ifndef RGRRT_H
#define RGRRT_H

namespace ompl
{
    namespace control
    {
        // TODO: Implement RGRRT as described

        class RGRRT : public base::Planner
        {

            // RRT From OMPL:
            public:
            
                RRT(const SpaceInformationPtr &si);

                ~RRT() override;

                // Continue solving for some amount of time. Return true if solution was found.
                base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;

                /*  Clear datastructures. Call this function if the
                    input data to the planner has changed and you do not
                    want to continue planning */
                void clear() override;

                /* In the process of randomly selecting states in the state
                    space to attempt to go towards, the algorithm may in fact
                    choose the actual goal state, if it knows it, with some
                    probability. This probability is a real number between 0.0
                    and 1.0; its value should usually be around 0.05 and
                    should not be too large. It is probably a good idea to use
                    the default value. */
                void setGoalBias(double goalBias)
                {
                    goalBias_ = goalBias;
                }

                // Get the goal bias the planner is using
                double getGoalBias() const
                {
                    return goalBias_;
                }

                // Return true if the intermediate states generated along motions are to be added to the tree itself
                bool getIntermediateStates() const
                {
                    return addIntermediateStates_;
                }

                // Specify whether the intermediate states generated along motions are to be added to the tree itself
                void setIntermediateStates(bool addIntermediateStates)
                {
                    addIntermediateStates_ = addIntermediateStates;
                }

                void getPlannerData(base::PlannerData &data) const override;

                // Set a different nearest neighbors datastructure
                template <template <typename T> class NN>
                void setNearestNeighbors()
                {
                    if (nn_ && nn_->size() != 0)
                        OMPL_WARN("Calling setNearestNeighbors will clear all states.");
                    clear();
                    nn_ = std::make_shared<NN<Motion *>>();
                    setup();
                }

                void setup() override;

            protected:
                /* Representation of a motion

                    This only contains pointers to parent motions as we
                    only need to go backwards in the tree. */
                class Motion
                {
                public:
                    Motion() = default;

                    // Constructor that allocates memory for the state and the control
                    Motion(const SpaceInformation *si)
                    : state(si->allocState()), control(si->allocControl())
                    {
                    }

                    ~Motion() = default;

                    // The state contained by the motion
                    base::State *state{nullptr};

                    // The control contained by the motion
                    Control *control{nullptr};

                    // The number of steps the control is applied for 
                    unsigned int steps{0};

                    // The parent motion in the exploration tree
                    Motion *parent{nullptr};
                };

                // Free the memory allocated by this planner
                void freeMemory();

                // Compute distance between motions (actually distance between contained states)
                double distanceFunction(const Motion *a, const Motion *b) const
                {
                    return si_->distance(a->state, b->state);
                }

                // State sampler
                base::StateSamplerPtr sampler_;

                // Control sampler
                DirectedControlSamplerPtr controlSampler_;

                // The base::SpaceInformation cast as control::SpaceInformation, for convenience */
                const SpaceInformation *siC_;

                // A nearest-neighbors datastructure containing the tree of motions */
                std::shared_ptr<NearestNeighbors<Motion *>> nn_;

                /* The fraction of time the goal is picked as the state to expand towards (if such a state is
                * available) */
                double goalBias_{0.05};

                // Flag indicating whether intermediate states are added to the built tree of motions
                bool addIntermediateStates_{false};

                // The random number generator
                RNG rng_;

                // The most recent goal motion.  Used for PlannerData computation
                Motion *lastGoalMotion_{nullptr};
            };

    }  // namespace control 
}  // namespace ompl

#endif
