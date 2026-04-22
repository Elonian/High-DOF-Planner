#include "planners/planner_api.h"
#include "planning_common.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <random>
#include <vector>

namespace {

armplanner::Config sampleAroundPath(const std::vector<armplanner::MotionNode>& tree,
                                    int goalIdx,
                                    int numofDOFs,
                                    std::mt19937& rng) {
    using namespace armplanner;

    std::vector<Config> path = traceTreePath(tree, goalIdx);
    if (path.empty()) {
        return sampleUniformConfig(numofDOFs, rng);
    }

    std::uniform_int_distribution<int> nodeDist(0, static_cast<int>(path.size()) - 1);
    std::normal_distribution<double> noise(0.0, 0.38);
    Config q = path[nodeDist(rng)];
    for (int i = 0; i < numofDOFs; ++i) {
        q[i] = wrapAngle(q[i] + noise(rng));
    }
    return q;
}

std::vector<armplanner::Config> solveRRTStarPath(const armplanner::Config& start,
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

    const double stepSize = 0.40;
    const int maxIterations = 32000;
    int bestGoalIdx = -1;
    double bestCost = std::numeric_limits<double>::infinity();
    auto startTime = std::chrono::steady_clock::now();

    for (int iter = 0; iter < maxIterations && elapsedSeconds(startTime) < timeoutSeconds; ++iter) {
        if (bestGoalIdx < 0 && elapsedSeconds(startTime) > timeoutSeconds * 0.65) {
            break;
        }

        Config qRand;
        if (bestGoalIdx >= 0 && unitDist(rng) < 0.38) {
            qRand = sampleAroundPath(tree, bestGoalIdx, numofDOFs, rng);
            if (!isConfigValid(qRand, numofDOFs, map, x_size, y_size)) {
                continue;
            }
        } else if (unitDist(rng) < 0.12) {
            qRand = goal;
        } else if (!sampleValidConfig(qRand, numofDOFs, map, x_size, y_size, rng)) {
            continue;
        }

        int nearIdx = nearestNode(tree, qRand);
        Config qNew = steerToward(tree[nearIdx].q, qRand, stepSize);
        if (!isEdgeValid(tree[nearIdx].q, qNew, numofDOFs, map, x_size, y_size)) {
            continue;
        }

        int n = static_cast<int>(tree.size()) + 1;
        double radius = 2.2 * std::pow(std::log(static_cast<double>(n) + 1.0) / static_cast<double>(n),
                                       1.0 / std::max(1, numofDOFs));
        radius = std::min(1.65, std::max(stepSize * 1.45, radius));
        std::vector<int> near = nearNodes(tree, qNew, radius);

        int parent = nearIdx;
        double parentCost = tree[nearIdx].cost + edgeCost(tree[nearIdx].q, qNew);
        for (int idx : near) {
            double candidateCost = tree[idx].cost + edgeCost(tree[idx].q, qNew);
            if (candidateCost + 1e-9 < parentCost &&
                isEdgeValid(tree[idx].q, qNew, numofDOFs, map, x_size, y_size)) {
                parent = idx;
                parentCost = candidateCost;
            }
        }

        int newIdx = addTreeNode(tree, qNew, parent, parentCost);

        for (int idx : near) {
            if (idx == parent || idx == 0) {
                continue;
            }
            double candidateCost = tree[newIdx].cost + edgeCost(tree[newIdx].q, tree[idx].q);
            if (candidateCost + 1e-9 < tree[idx].cost &&
                isEdgeValid(tree[newIdx].q, tree[idx].q, numofDOFs, map, x_size, y_size)) {
                removeChild(tree, tree[idx].parent, idx);
                tree[idx].parent = newIdx;
                tree[newIdx].children.push_back(idx);
                tree[idx].cost = candidateCost;
                updateSubtreeCosts(tree, idx);
            }
        }

        if (isEdgeValid(tree[newIdx].q, goal, numofDOFs, map, x_size, y_size)) {
            double candidateGoalCost = tree[newIdx].cost + edgeCost(tree[newIdx].q, goal);
            if (candidateGoalCost + 1e-9 < bestCost) {
                bestGoalIdx = addTreeNode(tree, goal, newIdx, candidateGoalCost);
                bestCost = candidateGoalCost;
                recordVerticesGenerated(static_cast<int>(tree.size()));
                recordFirstSolutionTime(elapsedSeconds(startTime));
            }
        }
    }

    if (bestGoalIdx >= 0) {
        recordVerticesGenerated(static_cast<int>(tree.size()));
        return traceTreePath(tree, bestGoalIdx);
    }

    recordVerticesGenerated(static_cast<int>(tree.size()));
    recordFallbackUsed();
    return solveRRTConnectPath(start, goal, numofDOFs, map, x_size, y_size, seed + 97, timeoutSeconds * 0.35);
}

}  // namespace

void planRRTStar(
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

    std::mt19937 shortcutRng(plannerSeed(34011u + static_cast<unsigned int>(numofDOFs)));
    std::vector<Config> path = solveRRTStarPath(
        start, goal, numofDOFs, map, x_size, y_size, plannerSeed(32003u), 4.45);
    shortcutPath(path, numofDOFs, map, x_size, y_size, shortcutRng, 320);
    setPlanFromPath(path, exactStart, exactGoal, numofDOFs, plan, planlength);
}
