#ifndef OMPL_MULTIROBOT_GEOMETRIC_PLANNERS_MRSYCLOP_
#define OMPL_MULTIROBOT_GEOMETRIC_PLANNERS_MRSYCLOP_

#include "ompl/multirobot/geometric/planners/PlannerIncludes.h"
#include "ompl/base/Planner.h"
#include "ompl/control/planners/syclop/Decomposition.h"
#include <vector>

namespace ompl
{
    namespace multirobot
    {
        namespace geometric
        {
            /**
            @anchor gMRSyCLoP
            @par Short description
            Multi-Robot SyCLoP (Synergistic Combination of Layers of Planning) is a
            geometric multi-robot motion planning algorithm that combines:
            - High-level discrete planning through workspace decomposition
            - Low-level geometric sampling-based planning for individual robots
            - Multi-robot coordination strategies

            This planner extends the SyCLoP framework to handle multiple robots
            simultaneously in a geometric (non-kinodynamic) setting.

            @par External documentation
            Based on the SyCLoP algorithm described in:
            E. Plaku, L.E. Kavraki, and M.Y. Vardi,
            Motion Planning with Dynamics by a Synergistic Combination of Layers of Planning,
            IEEE Transactions on Robotics, vol. 26, no. 3, pp. 469-482, June 2010.
            */

            /** \brief Multi-Robot Geometric SyCLoP Planner */
            class MRSyCLoP : public multirobot::base::Planner
            {
            public:
                /** \brief Constructor */
                MRSyCLoP(const multirobot::base::SpaceInformationPtr &si);

                /** \brief Destructor */
                ~MRSyCLoP() override;

                /** \brief Get planner data for visualization and debugging */
                void getPlannerData(ompl::base::PlannerData &data) const override;

                /** \brief Solve the multi-robot planning problem */
                ompl::base::PlannerStatus solve(const ompl::base::PlannerTerminationCondition &ptc) override;

                /** \brief Clear all internal datastructures */
                void clear() override;

                /** \brief Setup the planner (allocate low-level planners, etc.) */
                void setup() override;

                /** \brief Set the decomposition for the workspace */
                void setDecomposition(const ompl::control::DecompositionPtr &decomp);

                /** \brief Get the workspace decomposition */
                const ompl::control::DecompositionPtr &getDecomposition() const
                {
                    return decomposition_;
                }

                /** \brief Set the number of region expansions per lead */
                void setNumRegionExpansions(unsigned int numExpansions)
                {
                    numRegionExpansions_ = numExpansions;
                }

                /** \brief Get the number of region expansions per lead */
                unsigned int getNumRegionExpansions() const
                {
                    return numRegionExpansions_;
                }

                /** \brief Set the number of tree selections per region expansion */
                void setNumTreeSelections(unsigned int numSelections)
                {
                    numTreeSelections_ = numSelections;
                }

                /** \brief Get the number of tree selections per region expansion */
                unsigned int getNumTreeSelections() const
                {
                    return numTreeSelections_;
                }

                /** \brief Set the probability of using shortest path (A*) vs random exploration */
                void setProbShortestPath(double prob)
                {
                    probShortestPath_ = prob;
                }

                /** \brief Get the probability of using shortest path */
                double getProbShortestPath() const
                {
                    return probShortestPath_;
                }

            protected:
                /** \brief Free the memory allocated by this planner */
                void freeMemory();

                /** \brief Initialize the region graph from the decomposition */
                void initRegionGraph();

                /** \brief Estimate free volume for each region */
                void estimateFreeVolumes();

                /** \brief Add start states to the search trees */
                void addStartStates();

                /** \brief Sample and identify goal regions */
                void identifyGoalRegions();

                /** \brief Compute a lead (sequence of regions) from start to goal */
                bool computeLead(std::vector<int> &lead);

                /** \brief Select a region from the lead for expansion */
                int selectRegion(const std::vector<int> &lead);

                /** \brief Expand the search tree in the selected region */
                bool expandTreeInRegion(int regionId);

                /** \brief Update coverage estimates after adding a new state */
                void updateCoverage(int regionId);

                /** \brief Update edge costs in the region graph */
                void updateEdgeCosts();

                /** \brief Check if a solution has been found */
                bool hasSolution();

                /** \brief Extract the solution path */
                void extractSolution();

                /** \brief An ordered container containing a low-level planner for each robot */
                std::vector<ompl::base::PlannerPtr> llSolvers_;

                /** \brief The base::SpaceInformation for convenience */
                const multirobot::base::SpaceInformation *siG_;

                /** \brief The workspace decomposition */
                ompl::control::DecompositionPtr decomposition_;

                /** \brief Number of region expansions per lead */
                unsigned int numRegionExpansions_{100};

                /** \brief Number of tree selections per region expansion */
                unsigned int numTreeSelections_{1};

                /** \brief Probability of using shortest path (A*) vs random exploration */
                double probShortestPath_{0.95};

                /** \brief Number of samples for free volume estimation */
                unsigned int numFreeVolSamples_{10000};

                /** \brief Probability of early lead abandonment */
                double probAbandonLeadEarly_{0.25};

                // Region-related data structures
                struct Region
                {
                    int id;
                    double freeVolume{0.0};
                    double coverage{0.0};
                    unsigned int numSelections{0};
                    double alpha{1.0};
                    std::vector<ompl::base::State *> states;
                };

                struct Adjacency
                {
                    int region1;
                    int region2;
                    double cost{1.0};
                    double coverage{0.0};
                    unsigned int numSelections{0};
                };

                /** \brief Region information */
                std::vector<Region> regions_;

                /** \brief Adjacency information between regions */
                std::vector<Adjacency> adjacencies_;

                /** \brief Start region indices for each robot */
                std::vector<int> startRegions_;

                /** \brief Goal region indices for each robot */
                std::vector<int> goalRegions_;
            };
        }
    }
}

#endif
