#include "planners/planner_api.h"
#include "planning_common.h"

#include <chrono>
#include <random>
#include <vector>

namespace {

std::vector<armplanner::Config> solveRRTPath(const armplanner::Config& start,
                                             const armplanner::Config& goal,
                                             int numofDOFs,
                                             double* map,
                                             int x_size,
                                             int y_size,
                                             unsigned int seed,
                                             double timeoutSeconds) {
    using namespace armplanner;

    if (isEdgeValid(start, goal, numofDOFs, map, x_size, y_size)) {
        recordVerticesGenerated(2);
        recordFirstSolutionTime(0.0);
        return {start, goal};
    }

    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> unitDist(0.0, 1.0);
    std::vector<MotionNode> tree;
    tree.reserve(30000);
    addTreeNode(tree, start, -1, 0.0);

    const double stepSize = 0.42;
    const double goalBias = 0.22;
    const int maxIterations = 35000;
    auto startTime = std::chrono::steady_clock::now();

    for (int iter = 0; iter < maxIterations && elapsedSeconds(startTime) < timeoutSeconds; ++iter) {
        Config qRand;
        if (unitDist(rng) < goalBias) {
            qRand = goal;
        } else if (!sampleValidConfig(qRand, numofDOFs, map, x_size, y_size, rng)) {
            continue;
        }

        int nearIdx = nearestNode(tree, qRand);
        Config qNew = steerToward(tree[nearIdx].q, qRand, stepSize);
        if (!isEdgeValid(tree[nearIdx].q, qNew, numofDOFs, map, x_size, y_size)) {
            continue;
        }

        int newIdx = addTreeNode(tree, qNew, nearIdx, tree[nearIdx].cost + edgeCost(tree[nearIdx].q, qNew));
        if (isEdgeValid(qNew, goal, numofDOFs, map, x_size, y_size)) {
            int goalIdx = addTreeNode(tree, goal, newIdx, tree[newIdx].cost + edgeCost(qNew, goal));
            recordVerticesGenerated(static_cast<int>(tree.size()));
            recordFirstSolutionTime(elapsedSeconds(startTime));
            return traceTreePath(tree, goalIdx);
        }
    }

    recordVerticesGenerated(static_cast<int>(tree.size()));
    return {};
}

}  // namespace

void planRRT(
    double* map,
    int x_size,
    int y_size,
    double* armstart_anglesV_rad,
    double* armgoal_anglesV_rad,
    int numofDOFs,
    double*** plan,
    int* planlength) {
    using namespace armplanner;

    Config exactStart = configFromArray(armstart_anglesV_rad, numofDOFs, false);
    Config exactGoal = configFromArray(armgoal_anglesV_rad, numofDOFs, false);
    Config start = configFromArray(armstart_anglesV_rad, numofDOFs, true);
    Config goal = configFromArray(armgoal_anglesV_rad, numofDOFs, true);

    std::mt19937 shortcutRng(plannerSeed(14011u + static_cast<unsigned int>(numofDOFs)));
    std::vector<Config> path = solveRRTPath(
        start, goal, numofDOFs, map, x_size, y_size, plannerSeed(12001u), 3.7);
    if (path.empty()) {
        recordFallbackUsed();
        path = solveRRTConnectPath(
            start, goal, numofDOFs, map, x_size, y_size, plannerSeed(12097u), 1.2);
    }
    shortcutPath(path, numofDOFs, map, x_size, y_size, shortcutRng, 180);
    setPlanFromPath(path, exactStart, exactGoal, numofDOFs, plan, planlength);
}
