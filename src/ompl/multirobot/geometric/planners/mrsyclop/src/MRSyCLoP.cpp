#include "ompl/multirobot/geometric/planners/mrsyclop/MRSyCLoP.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"
#include "ompl/geometric/planners/rrt/RRT.h"
#include "ompl/geometric/PathGeometric.h"
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <algorithm>
#include <cmath>
#include <limits>

ompl::multirobot::geometric::MRSyCLoP::MRSyCLoP(const ompl::multirobot::base::SpaceInformationPtr &si)
  : ompl::multirobot::base::Planner(si, "MRSyCLoP")
{
    siG_ = si.get();

    // Set default lead computation function
    leadComputeFn_ = [this](unsigned int robotIdx, std::vector<int> &lead)
                     { return computeLeadForRobot(robotIdx, lead); };

    // Declare planner parameters
    Planner::declareParam<unsigned int>("num_region_expansions", this,
                                        &MRSyCLoP::setNumRegionExpansions,
                                        &MRSyCLoP::getNumRegionExpansions,
                                        "1:1:1000");
    Planner::declareParam<unsigned int>("num_tree_selections", this,
                                        &MRSyCLoP::setNumTreeSelections,
                                        &MRSyCLoP::getNumTreeSelections,
                                        "1:1:100");
    Planner::declareParam<double>("prob_shortest_path", this,
                                  &MRSyCLoP::setProbShortestPath,
                                  &MRSyCLoP::getProbShortestPath,
                                  "0.0:0.05:1.0");
}

ompl::multirobot::geometric::MRSyCLoP::~MRSyCLoP()
{
    freeMemory();
}

void ompl::multirobot::geometric::MRSyCLoP::clear()
{
    base::Planner::clear();
    freeMemory();

    // Clear regions and adjacencies
    regions_.clear();
    adjacencies_.clear();
    startRegions_.clear();
    goalRegions_.clear();
    leads_.clear();
    currentTimestep_.clear();
    segments_.clear();

    // Free current states
    for (unsigned int r = 0; r < siG_->getIndividualCount(); r++)
    {
        if (currentStates_.size() > r && currentStates_[r])
        {
            siG_->getIndividual(r)->freeState(currentStates_[r]);
            currentStates_[r] = nullptr;
        }
    }
    currentStates_.clear();
}

void ompl::multirobot::geometric::MRSyCLoP::setup()
{
    base::Planner::setup();

    // Verify that we have a decomposition
    if (!decomposition_)
    {
        OMPL_ERROR("%s: No decomposition set", getName().c_str());
        throw ompl::Exception(getName(), "No decomposition provided");
    }

    // Verify planner allocator exists
    if (!siG_->hasPlannerAllocator())
    {
        OMPL_ERROR("%s: No PlannerAllocator provided", getName().c_str());
        throw ompl::Exception(getName(), "No PlannerAllocator provided");
    }

    // Initialize data structures for each robot
    const unsigned int numRobots = siG_->getIndividualCount();
    llSolvers_.resize(numRobots);
    leads_.resize(numRobots);
    currentTimestep_.resize(numRobots, 0);
    segments_.resize(numRobots);
    currentStates_.resize(numRobots, nullptr);
    startRegions_.resize(numRobots, -1);
    goalRegions_.resize(numRobots, -1);

    // Allocate current states for each robot
    for (unsigned int r = 0; r < numRobots; r++)
    {
        currentStates_[r] = siG_->getIndividual(r)->allocState();
    }

    // Initialize the region graph
    initRegionGraph();

    // Estimate free volumes
    estimateFreeVolumes();
}

void ompl::multirobot::geometric::MRSyCLoP::freeMemory()
{
    // Clear low-level planners
    for (unsigned int r = 0; r < siG_->getIndividualCount(); r++)
    {
        if (llSolvers_[r])
        {
            llSolvers_[r]->clear();
        }
        siG_->getIndividual(r)->clearDynamicObstacles();
    }

    // Clear regions (states will be managed by low-level planners)
    for (auto &region : regions_)
    {
        region.states.clear();
    }
}

void ompl::multirobot::geometric::MRSyCLoP::setDecomposition(const ompl::control::DecompositionPtr &decomp)
{
    decomposition_ = decomp;
}

ompl::base::PlannerStatus ompl::multirobot::geometric::MRSyCLoP::solve(
    const ompl::base::PlannerTerminationCondition &ptc)
{
    checkValidity();

    // Add start states and identify goal regions
    addStartStates();
    identifyGoalRegions();

    // Compute leads for each robot
    const unsigned int numRobots = siG_->getIndividualCount();
    bool allLeadsValid = true;
    for (unsigned int r = 0; r < numRobots; ++r)
    {
        if (!leadComputeFn_(r, leads_[r]))
        {
            OMPL_ERROR("%s: Failed to compute lead for robot %u", getName().c_str(), r);
            allLeadsValid = false;
        }
        else
        {
            OMPL_INFORM("%s: Robot %u lead has %zu regions", getName().c_str(), r, leads_[r].size());
        }
    }

    if (!allLeadsValid)
        return ompl::base::PlannerStatus::INVALID_GOAL;

    // Main planning loop
    unsigned int iteration = 0;
    while (!ptc && !hasSolution())
    {
        ++iteration;

        // Group robots by their current region in their leads
        std::unordered_map<int, std::vector<unsigned int>> regionToRobots;

        for (unsigned int r = 0; r < numRobots; ++r)
        {
            // Skip robots that have completed their leads
            if (currentTimestep_[r] >= leads_[r].size())
                continue;

            const int currentRegion = leads_[r][currentTimestep_[r]];
            regionToRobots[currentRegion].push_back(r);
        }

        // For each region with robots, grow RRT
        for (const auto &[regionId, robotIndices] : regionToRobots)
        {
            // Perform multiple expansions in this region
            for (unsigned int exp = 0; exp < numRegionExpansions_ && !ptc; ++exp)
            {
                RRTSegment segment;
                segment.regionId = regionId;
                segment.robotIndices = robotIndices;

                bool success = false;
                if (robotIndices.size() == 1)
                {
                    // Single robot - use individual space
                    segment.isComposite = false;
                    segment.timestep = currentTimestep_[robotIndices[0]];
                    success = growIndividualRRT(robotIndices[0], regionId, segment);
                }
                else
                {
                    // Multiple robots - use composite space
                    segment.isComposite = true;
                    segment.timestep = currentTimestep_[robotIndices[0]];  // All should have same timestep
                    success = growCompositeRRT(robotIndices, regionId, segment);
                }

                if (success)
                {
                    // Store segment for each robot involved
                    for (unsigned int r : robotIndices)
                    {
                        segments_[r].push_back(segment);
                    }
                }
            }
        }

        // Check if any robots have reached their next region and should advance timestep
        for (unsigned int r = 0; r < numRobots; ++r)
        {
            if (currentTimestep_[r] + 1 < leads_[r].size())
            {
                if (isNearNextRegion(r, currentStates_[r]))
                {
                    ++currentTimestep_[r];
                    OMPL_INFORM("%s: Robot %u advanced to timestep %u (region %d)",
                               getName().c_str(), r, currentTimestep_[r], leads_[r][currentTimestep_[r]]);
                }
            }
        }

        // Periodic logging
        if (iteration % 100 == 0)
        {
            OMPL_INFORM("%s: Iteration %u, checking solution...", getName().c_str(), iteration);
            // Debug: print robot states
            for (unsigned int r = 0; r < numRobots; ++r)
            {
                int currentRegion = decomposition_->locateRegion(currentStates_[r]);
                OMPL_INFORM("  Robot %u: timestep=%u/%zu, current_region=%d, lead=[%d,%d...]",
                           r, currentTimestep_[r], leads_[r].size()-1, currentRegion,
                           leads_[r][0], leads_[r].size() > 1 ? leads_[r][1] : -1);

                // Check goal distance
                const auto &pdef = pdef_->getIndividual(r);
                const auto &goal = pdef->getGoal();
                if (goal)
                {
                    OMPL_INFORM("    Goal satisfied: %s", goal->isSatisfied(currentStates_[r]) ? "YES" : "NO");
                }
            }
        }
    }

    if (hasSolution())
    {
        extractSolution();
        return ompl::base::PlannerStatus::EXACT_SOLUTION;
    }

    return ompl::base::PlannerStatus::TIMEOUT;
}

void ompl::multirobot::geometric::MRSyCLoP::getPlannerData(ompl::base::PlannerData &data) const
{
    // TODO: Implement planner data extraction for visualization/debugging
    // This can include:
    // - States in the search trees
    // - Region information
    // - Coverage statistics
    // - Edge information
}

void ompl::multirobot::geometric::MRSyCLoP::initRegionGraph()
{
    const int numRegions = decomposition_->getNumRegions();
    regions_.resize(numRegions);

    // Initialize each region
    for (int i = 0; i < numRegions; ++i)
    {
        regions_[i].id = i;
        regions_[i].freeVolume = decomposition_->getRegionVolume(i);
        regions_[i].coverage = 0.0;
        regions_[i].numSelections = 0;
        regions_[i].alpha = 1.0;
    }

    // Build adjacency list
    std::vector<int> neighbors;
    for (int i = 0; i < numRegions; ++i)
    {
        decomposition_->getNeighbors(i, neighbors);
        for (int j : neighbors)
        {
            // Add edge (avoid duplicates by only adding if i < j)
            if (i < j)
            {
                Adjacency adj;
                adj.region1 = i;
                adj.region2 = j;
                adj.cost = 1.0;
                adj.coverage = 0.0;
                adj.numSelections = 0;
                adjacencies_.push_back(adj);
            }
        }
        neighbors.clear();
    }
}

void ompl::multirobot::geometric::MRSyCLoP::estimateFreeVolumes()
{
    // Use the first robot's space to sample and estimate free volumes
    // This is a simplification - in future could use composite space
    const auto &si = siG_->getIndividual(0);
    auto sampler = si->allocStateSampler();
    auto state = si->allocState();

    std::vector<int> totalSamples(regions_.size(), 0);
    std::vector<int> validSamples(regions_.size(), 0);

    // Sample states and count valid ones per region
    for (unsigned int i = 0; i < numFreeVolSamples_; ++i)
    {
        sampler->sampleUniform(state);
        int rid = decomposition_->locateRegion(state);
        if (rid >= 0 && rid < static_cast<int>(regions_.size()))
        {
            ++totalSamples[rid];
            if (si->isValid(state))
                ++validSamples[rid];
        }
    }

    // Update free volume estimates
    for (size_t i = 0; i < regions_.size(); ++i)
    {
        if (totalSamples[i] > 0)
        {
            double ratio = static_cast<double>(validSamples[i]) / totalSamples[i];
            regions_[i].freeVolume = ratio * decomposition_->getRegionVolume(i);
        }
        // Ensure non-zero free volume
        if (regions_[i].freeVolume < std::numeric_limits<double>::epsilon())
            regions_[i].freeVolume = std::numeric_limits<double>::epsilon();

        // Update alpha value (similar to SyCLoP)
        regions_[i].alpha = 1.0 / ((1 + regions_[i].coverage) *
                                   std::pow(regions_[i].freeVolume, 4.0));
    }

    si->freeState(state);
}

void ompl::multirobot::geometric::MRSyCLoP::addStartStates()
{
    const unsigned int numRobots = siG_->getIndividualCount();

    for (unsigned int r = 0; r < numRobots; ++r)
    {
        const auto &pdef = pdef_->getIndividual(r);
        if (pdef->getStartStateCount() > 0)
        {
            // Get the start state for this robot
            const ompl::base::State *startState = pdef->getStartState(0);

            // Copy to current state
            siG_->getIndividual(r)->copyState(currentStates_[r], startState);

            // Identify start region
            startRegions_[r] = decomposition_->locateRegion(startState);

            if (startRegions_[r] < 0)
            {
                OMPL_ERROR("%s: Start state for robot %u is not in any region", getName().c_str(), r);
            }
        }
        else
        {
            OMPL_ERROR("%s: No start state for robot %u", getName().c_str(), r);
        }
    }
}

void ompl::multirobot::geometric::MRSyCLoP::identifyGoalRegions()
{
    const unsigned int numRobots = siG_->getIndividualCount();

    for (unsigned int r = 0; r < numRobots; ++r)
    {
        const auto &pdef = pdef_->getIndividual(r);
        const auto &goal = pdef->getGoal();

        if (goal)
        {
            // Try to sample a goal state
            auto goalSampleable = std::dynamic_pointer_cast<ompl::base::GoalSampleableRegion>(goal);
            if (goalSampleable && goalSampleable->canSample())
            {
                auto goalState = siG_->getIndividual(r)->allocState();
                goalSampleable->sampleGoal(goalState);
                goalRegions_[r] = decomposition_->locateRegion(goalState);
                siG_->getIndividual(r)->freeState(goalState);

                if (goalRegions_[r] < 0)
                {
                    OMPL_ERROR("%s: Goal state for robot %u is not in any region", getName().c_str(), r);
                }
            }
            else
            {
                OMPL_WARN("%s: Goal for robot %u is not sampleable, cannot identify goal region", getName().c_str(), r);
                goalRegions_[r] = -1;
            }
        }
        else
        {
            OMPL_ERROR("%s: No goal for robot %u", getName().c_str(), r);
            goalRegions_[r] = -1;
        }
    }
}

bool ompl::multirobot::geometric::MRSyCLoP::computeLeadForRobot(unsigned int robotIdx, std::vector<int> &lead)
{
    lead.clear();

    const int startRegion = startRegions_[robotIdx];
    const int goalRegion = goalRegions_[robotIdx];

    if (startRegion < 0 || goalRegion < 0)
    {
        OMPL_ERROR("%s: Invalid start or goal region for robot %u", getName().c_str(), robotIdx);
        return false;
    }

    if (startRegion == goalRegion)
    {
        lead.push_back(startRegion);
        return true;
    }

    // Use A* to find shortest path in region graph
    // Priority queue: (f_cost, region_id)
    using PQElement = std::pair<double, int>;
    std::priority_queue<PQElement, std::vector<PQElement>, std::greater<PQElement>> pq;

    std::unordered_map<int, double> gScore;  // Cost from start
    std::unordered_map<int, int> cameFrom;   // Parent map
    std::unordered_set<int> closedSet;

    // Initialize
    gScore[startRegion] = 0.0;
    pq.push({0.0, startRegion});

    // Build adjacency map for quick lookup
    std::unordered_map<int, std::vector<std::pair<int, double>>> adjMap;
    for (const auto &adj : adjacencies_)
    {
        adjMap[adj.region1].push_back({adj.region2, adj.cost});
        adjMap[adj.region2].push_back({adj.region1, adj.cost});
    }

    while (!pq.empty())
    {
        int current = pq.top().second;
        pq.pop();

        if (closedSet.count(current) > 0)
            continue;

        closedSet.insert(current);

        if (current == goalRegion)
        {
            // Reconstruct path
            std::vector<int> reversePath;
            int node = goalRegion;
            while (node != startRegion)
            {
                reversePath.push_back(node);
                node = cameFrom[node];
            }
            reversePath.push_back(startRegion);

            // Reverse to get start->goal path
            lead.resize(reversePath.size());
            std::reverse_copy(reversePath.begin(), reversePath.end(), lead.begin());
            return true;
        }

        // Explore neighbors
        if (adjMap.count(current) > 0)
        {
            for (const auto &[neighbor, cost] : adjMap[current])
            {
                if (closedSet.count(neighbor) > 0)
                    continue;

                double tentativeG = gScore[current] + cost;

                if (gScore.count(neighbor) == 0 || tentativeG < gScore[neighbor])
                {
                    cameFrom[neighbor] = current;
                    gScore[neighbor] = tentativeG;

                    // Simple heuristic: use region alpha values
                    double heuristic = regions_[neighbor].alpha;
                    double fScore = tentativeG + heuristic;

                    pq.push({fScore, neighbor});
                }
            }
        }
    }

    OMPL_WARN("%s: Could not find lead from region %d to %d for robot %u",
              getName().c_str(), startRegion, goalRegion, robotIdx);
    return false;
}

bool ompl::multirobot::geometric::MRSyCLoP::computeLead(std::vector<int> &lead)
{
    // This method is kept for compatibility but is not used in the multi-robot version
    // Multi-robot version computes leads independently for each robot
    return false;
}

int ompl::multirobot::geometric::MRSyCLoP::selectRegion(const std::vector<int> &lead)
{
    // TODO: Select a region from the lead for expansion
    // 1. Build probability distribution over regions in lead
    // 2. Weight by coverage and selection history
    // 3. Sample from distribution
    // Return selected region index

    return -1;
}

bool ompl::multirobot::geometric::MRSyCLoP::expandTreeInRegion(int regionId)
{
    // TODO: Expand the search tree in the selected region
    // 1. Sample a random state in the region
    // 2. Find nearest state in tree (possibly restricted to region)
    // 3. Extend tree toward sampled state
    // 4. Check for collisions
    // 5. Add new state to tree if valid
    // 6. Update coverage
    // Return true if expansion successful

    return false;
}

void ompl::multirobot::geometric::MRSyCLoP::updateCoverage(int regionId)
{
    // TODO: Update coverage estimates after adding a new state
    // 1. Update coverage grid for the region
    // 2. Update adjacency coverage if state is near region boundary
    // 3. Update region alpha values
}

void ompl::multirobot::geometric::MRSyCLoP::updateEdgeCosts()
{
    // TODO: Update edge costs in the region graph
    // Use formula: cost(r,s) = [1 + sel²(r,s)] / [1 + conn²(r,s)] * alpha(r) * alpha(s)
    // where:
    //   alpha(t) = 1 / [(1 + cov(t)) * freeVol⁴(t)]
    //   sel(r,s) = number of selections of edge (r,s)
    //   conn(r,s) = coverage cells along edge (r,s)
}

bool ompl::multirobot::geometric::MRSyCLoP::isNearNextRegion(unsigned int robotIdx, ompl::base::State *state)
{
    // Check if state is in the next region of the lead
    const unsigned int currentTS = currentTimestep_[robotIdx];
    if (currentTS + 1 >= leads_[robotIdx].size())
    {
        // Already at the last region in the lead
        return false;
    }

    const int nextRegion = leads_[robotIdx][currentTS + 1];
    const int stateRegion = decomposition_->locateRegion(state);

    return (stateRegion == nextRegion);
}

bool ompl::multirobot::geometric::MRSyCLoP::hasSolution()
{
    // Check if all robots have reached their goal regions
    const unsigned int numRobots = siG_->getIndividualCount();

    for (unsigned int r = 0; r < numRobots; ++r)
    {
        // Check if robot has completed its lead
        if (currentTimestep_[r] + 1 < leads_[r].size())
            return false;

        // Check if current state satisfies goal
        const auto &pdef = pdef_->getIndividual(r);
        const auto &goal = pdef->getGoal();
        if (goal && !goal->isSatisfied(currentStates_[r]))
            return false;
    }

    return true;
}

bool ompl::multirobot::geometric::MRSyCLoP::growIndividualRRT(unsigned int robotIdx, int regionId, RRTSegment &segment)
{
    const auto &si = siG_->getIndividual(robotIdx);
    const auto &pdef = pdef_->getIndividual(robotIdx);
    const auto &goal = pdef->getGoal();

    // If goal is already satisfied, don't move anymore
    if (goal && goal->isSatisfied(currentStates_[robotIdx]))
    {
        return false;  // Stay at goal, don't wander
    }

    // Sample a random state - prefer sampling from next region in lead to make progress
    std::vector<double> coord;
    int sampleRegion = regionId;  // Default to current region

    // If not at end of lead, sample from next region with high probability
    const unsigned int currentTS = currentTimestep_[robotIdx];
    if (currentTS + 1 < leads_[robotIdx].size())
    {
        // Sample from next region 80% of the time to encourage progress
        if (rng_.uniform01() < 0.8)
        {
            sampleRegion = leads_[robotIdx][currentTS + 1];
        }
    }
    // If at goal region, sample directly toward goal state
    else
    {
        // In final region - bias heavily toward goal
        if (goal)
        {
            auto goalSampleable = std::dynamic_pointer_cast<ompl::base::GoalSampleableRegion>(goal);
            if (goalSampleable && goalSampleable->canSample() && rng_.uniform01() < 0.9)
            {
                // Sample goal directly 90% of the time
                auto randomState = si->allocState();
                goalSampleable->sampleGoal(randomState);

                // Try to extend toward goal
                auto newState = si->allocState();
                double maxDist = si->getMaximumExtent() * 0.1;
                double dist = si->distance(currentStates_[robotIdx], randomState);

                if (dist > maxDist)
                {
                    si->getStateSpace()->interpolate(currentStates_[robotIdx], randomState, maxDist / dist, newState);
                }
                else
                {
                    si->copyState(newState, randomState);
                }

                if (si->isValid(newState) && si->checkMotion(currentStates_[robotIdx], newState))
                {
                    si->copyState(currentStates_[robotIdx], newState);
                    segment.startState = si->cloneState(currentStates_[robotIdx]);
                    segment.endState = si->cloneState(newState);
                    si->freeState(randomState);
                    si->freeState(newState);
                    return true;
                }

                si->freeState(randomState);
                si->freeState(newState);
                return false;
            }
        }
    }

    decomposition_->sampleFromRegion(sampleRegion, rng_, coord);

    auto randomState = si->allocState();
    auto sampler = si->allocStateSampler();
    decomposition_->sampleFullState(sampler, coord, randomState);

    // Check if sampled state is valid
    if (!si->isValid(randomState))
    {
        si->freeState(randomState);
        return false;
    }

    // Simple RRT extension: move from current state toward random state
    auto newState = si->allocState();
    double maxDist = si->getMaximumExtent() * 0.1;  // 10% of space extent as step size
    double dist = si->distance(currentStates_[robotIdx], randomState);

    if (dist > maxDist)
    {
        // Interpolate
        si->getStateSpace()->interpolate(currentStates_[robotIdx], randomState, maxDist / dist, newState);
    }
    else
    {
        // Use random state directly
        si->copyState(newState, randomState);
    }

    // Check if new state is valid
    if (!si->isValid(newState) || !si->checkMotion(currentStates_[robotIdx], newState))
    {
        si->freeState(randomState);
        si->freeState(newState);
        return false;
    }

    // Update current state
    si->copyState(currentStates_[robotIdx], newState);

    // Store in segment (for now, just mark as successful)
    segment.startState = si->cloneState(currentStates_[robotIdx]);
    segment.endState = si->cloneState(newState);

    si->freeState(randomState);
    si->freeState(newState);

    return true;
}

bool ompl::multirobot::geometric::MRSyCLoP::growCompositeRRT(const std::vector<unsigned int> &robotIndices,
                                                              int regionId,
                                                              RRTSegment &segment)
{
    // Create composite space for this set of robots
    auto compositeSpace = createCompositeSpaceInfo(robotIndices);
    if (!compositeSpace)
    {
        OMPL_ERROR("%s: Failed to create composite space", getName().c_str());
        return false;
    }

    // Create composite current state
    std::vector<ompl::base::State *> individualStates;
    for (unsigned int r : robotIndices)
    {
        individualStates.push_back(currentStates_[r]);
    }
    auto compositeCurrentState = createCompositeState(robotIndices, individualStates, compositeSpace);

    // Sample a random composite state - prefer next region for progress
    std::vector<double> coord;
    int sampleRegion = regionId;

    // Sample from next region for first robot (heuristic for composite)
    const unsigned int firstRobot = robotIndices[0];
    const unsigned int currentTS = currentTimestep_[firstRobot];
    if (currentTS + 1 < leads_[firstRobot].size())
    {
        if (rng_.uniform01() < 0.8)
        {
            sampleRegion = leads_[firstRobot][currentTS + 1];
        }
    }

    decomposition_->sampleFromRegion(sampleRegion, rng_, coord);

    auto randomState = compositeSpace->allocState();
    auto sampler = compositeSpace->allocStateSampler();
    decomposition_->sampleFullState(sampler, coord, randomState);

    // Check if sampled state is valid (this should check inter-robot collisions)
    if (!compositeSpace->isValid(randomState))
    {
        compositeSpace->freeState(randomState);
        compositeSpace->freeState(compositeCurrentState);
        return false;
    }

    // Simple RRT extension in composite space
    auto newState = compositeSpace->allocState();
    double maxDist = compositeSpace->getMaximumExtent() * 0.1;
    double dist = compositeSpace->distance(compositeCurrentState, randomState);

    if (dist > maxDist)
    {
        compositeSpace->getStateSpace()->interpolate(compositeCurrentState, randomState, maxDist / dist, newState);
    }
    else
    {
        compositeSpace->copyState(newState, randomState);
    }

    // Check validity and motion
    if (!compositeSpace->isValid(newState) || !compositeSpace->checkMotion(compositeCurrentState, newState))
    {
        compositeSpace->freeState(randomState);
        compositeSpace->freeState(newState);
        compositeSpace->freeState(compositeCurrentState);
        return false;
    }

    // Extract individual states from composite newState and update current states
    for (size_t i = 0; i < robotIndices.size(); ++i)
    {
        unsigned int r = robotIndices[i];
        extractIndividualState(newState, i, currentStates_[r], compositeSpace);
    }

    // Store segment info
    segment.startState = compositeSpace->cloneState(compositeCurrentState);
    segment.endState = compositeSpace->cloneState(newState);

    compositeSpace->freeState(randomState);
    compositeSpace->freeState(newState);
    compositeSpace->freeState(compositeCurrentState);

    return true;
}

ompl::base::SpaceInformationPtr ompl::multirobot::geometric::MRSyCLoP::createCompositeSpaceInfo(
    const std::vector<unsigned int> &robotIndices)
{
    if (robotIndices.empty())
        return nullptr;

    // Create a compound state space from individual robot spaces
    auto compoundSpace = std::make_shared<ompl::base::CompoundStateSpace>();

    for (unsigned int r : robotIndices)
    {
        // Add each robot's state space as a subspace with equal weight
        compoundSpace->addSubspace(siG_->getIndividual(r)->getStateSpace(), 1.0);
    }

    // Lock the compound space (no more subspaces can be added)
    compoundSpace->lock();

    // Create SpaceInformation for the compound space
    auto compositeSI = std::make_shared<ompl::base::SpaceInformation>(compoundSpace);

    // Set up a state validity checker for the composite space
    // This checker will check both individual robot validity and inter-robot collisions
    compositeSI->setStateValidityChecker(
        [this, robotIndices](const ompl::base::State *state) -> bool
        {
            const auto *compState = state->as<ompl::base::CompoundState>();

            // Check validity of each individual robot state
            for (size_t i = 0; i < robotIndices.size(); ++i)
            {
                unsigned int r = robotIndices[i];
                const ompl::base::State *robotState = compState->components[i];

                if (!siG_->getIndividual(r)->isValid(robotState))
                    return false;
            }

            // Check inter-robot collisions using state validity checkers
            // Each robot's state validity checker can implement areStatesValid()
            // to check collision with another robot's state
            for (size_t i = 0; i < robotIndices.size(); ++i)
            {
                for (size_t j = i + 1; j < robotIndices.size(); ++j)
                {
                    unsigned int r1 = robotIndices[i];
                    unsigned int r2 = robotIndices[j];

                    const ompl::base::State *state1 = compState->components[i];
                    const ompl::base::State *state2 = compState->components[j];

                    const auto &si1 = siG_->getIndividual(r1);
                    const auto &si2 = siG_->getIndividual(r2);

                    // Check if the state validity checker supports inter-robot collision checking
                    auto checker1 = si1->getStateValidityChecker();
                    if (checker1)
                    {
                        // Use areStatesValid if available (for multi-robot aware checkers)
                        if (!checker1->areStatesValid(state1, std::make_pair(si2, state2)))
                        {
                            return false;  // Collision detected
                        }
                    }
                }
            }

            return true;
        });

    compositeSI->setup();

    return compositeSI;
}

ompl::base::State *ompl::multirobot::geometric::MRSyCLoP::createCompositeState(
    const std::vector<unsigned int> &robotIndices,
    const std::vector<ompl::base::State *> &individualStates,
    const ompl::base::SpaceInformationPtr &compositeSpace)
{
    if (individualStates.empty() || robotIndices.size() != individualStates.size())
        return nullptr;

    // Allocate a compound state
    auto *compositeState = compositeSpace->allocState()->as<ompl::base::CompoundState>();

    // Copy each individual robot state into the corresponding component
    for (size_t i = 0; i < robotIndices.size(); ++i)
    {
        unsigned int r = robotIndices[i];
        const auto &si = siG_->getIndividual(r);

        // Copy the individual state to the i-th component of the compound state
        si->copyState(compositeState->components[i], individualStates[i]);
    }

    return compositeState;
}

void ompl::multirobot::geometric::MRSyCLoP::extractIndividualState(
    ompl::base::State *compositeState,
    unsigned int componentIndex,
    ompl::base::State *individualState,
    const ompl::base::SpaceInformationPtr &compositeSpace)
{
    // Cast to compound state
    const auto *compState = compositeState->as<ompl::base::CompoundState>();

    // Get the component state space
    const auto &compoundSpace = compositeSpace->getStateSpace()->as<ompl::base::CompoundStateSpace>();
    const auto &componentSpace = compoundSpace->getSubspace(componentIndex);

    // Copy the component state to the individual state
    componentSpace->copyState(individualState, compState->components[componentIndex]);
}

void ompl::multirobot::geometric::MRSyCLoP::extractSolution()
{
    // Create solution paths by reconstructing from segments
    const unsigned int numRobots = siG_->getIndividualCount();

    for (unsigned int r = 0; r < numRobots; ++r)
    {
        const auto &si = siG_->getIndividual(r);
        const auto &pdef = pdef_->getIndividual(r);

        auto path = std::make_shared<ompl::geometric::PathGeometric>(si);

        // Add start state
        if (pdef->getStartStateCount() > 0)
        {
            path->append(pdef->getStartState(0));
        }

        // Add intermediate states from segments
        // This is simplified - in a full implementation, you'd properly reconstruct paths
        // For now, just add the current final state
        path->append(currentStates_[r]);

        // Add solution to problem definition
        pdef->addSolutionPath(path);

        OMPL_INFORM("%s: Solution path for robot %u has %u states",
                   getName().c_str(), r, path->getStateCount());
    }
}
