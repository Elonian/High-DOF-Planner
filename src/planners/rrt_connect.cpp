#include "planners/planner_api.h"
#include "planning_common.h"

#include <algorithm>
#include <chrono>
#include <random>
#include <vector>

namespace armplanner {

std::vector<Config> solveRRTConnectPath(const Config& start,
                                        const Config& goal,
                                        int numofDOFs,
                                        double* map,
                                        int x_size,
                                        int y_size,
                                        unsigned int seed,
                                        double timeoutSeconds) {
    if (isEdgeValid(start, goal, numofDOFs, map, x_size, y_size)) {
        recordVerticesGenerated(2);
        recordFirstSolutionTime(0.0);
        return {start, goal};
    }

    std::mt19937 rng(seed);
    std::vector<MotionNode> treeA;
    std::vector<MotionNode> treeB;
    treeA.reserve(20000);
    treeB.reserve(20000);
    addTreeNode(treeA, start, -1, 0.0);
    addTreeNode(treeB, goal, -1, 0.0);

    const double stepSize = 0.48;
    const int maxIterations = 25000;
    bool treeAStartsAtStart = true;
    auto startTime = std::chrono::steady_clock::now();

    for (int iter = 0; iter < maxIterations && elapsedSeconds(startTime) < timeoutSeconds; ++iter) {
        Config qRand;
        if (!sampleValidConfig(qRand, numofDOFs, map, x_size, y_size, rng)) {
            continue;
        }

        int idxA = -1;
        ExtendStatus statusA = extendTree(treeA, qRand, stepSize, numofDOFs, map, x_size, y_size, idxA);
        if (statusA != TRAPPED) {
            int idxB = -1;
            ExtendStatus statusB = connectTree(treeB, treeA[idxA].q, stepSize, numofDOFs, map, x_size, y_size, idxB);
            if (statusB == REACHED) {
                recordVerticesGenerated(static_cast<int>(treeA.size() + treeB.size()));
                recordFirstSolutionTime(elapsedSeconds(startTime));
                return combineTreePaths(treeA, idxA, treeB, idxB, treeAStartsAtStart);
            }
        }

        std::swap(treeA, treeB);
        treeAStartsAtStart = !treeAStartsAtStart;
    }

    recordVerticesGenerated(static_cast<int>(treeA.size() + treeB.size()));
    return {};
}

}  // namespace armplanner

void planRRTConnect(
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

    std::mt19937 shortcutRng(plannerSeed(24011u + static_cast<unsigned int>(numofDOFs)));
    std::vector<Config> path = solveRRTConnectPath(
        start, goal, numofDOFs, map, x_size, y_size, plannerSeed(22003u), 4.6);
    shortcutPath(path, numofDOFs, map, x_size, y_size, shortcutRng, 260);
    setPlanFromPath(path, exactStart, exactGoal, numofDOFs, plan, planlength);
}
