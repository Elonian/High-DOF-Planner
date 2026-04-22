#include "planning_common.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <limits>

namespace armplanner {

namespace {
PlannerStats gStats = {0, -1.0, false};
}

void resetPlannerStats() {
    gStats.verticesGenerated = 0;
    gStats.firstSolutionTime = -1.0;
    gStats.usedFallback = false;
}

PlannerStats getPlannerStats() {
    return gStats;
}

void recordVerticesGenerated(int vertices) {
    if (vertices > gStats.verticesGenerated) {
        gStats.verticesGenerated = vertices;
    }
}

void recordFirstSolutionTime(double seconds) {
    if (gStats.firstSolutionTime < 0.0) {
        gStats.firstSolutionTime = seconds;
    }
}

void recordFallbackUsed() {
    gStats.usedFallback = true;
}

unsigned int plannerSeed(unsigned int baseSeed) {
    const char* envSeed = std::getenv("HDOF_PLANNER_SEED");
    if (!envSeed || envSeed[0] == '\0') {
        return baseSeed;
    }

    char* end = nullptr;
    unsigned long parsed = std::strtoul(envSeed, &end, 10);
    if (end == envSeed) {
        return baseSeed;
    }

    return baseSeed + static_cast<unsigned int>((parsed % 1000003UL) * 104729UL);
}

double elapsedSeconds(const std::chrono::steady_clock::time_point& start) {
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
}

double wrapAngle(double angle) {
    double wrapped = std::fmod(angle, kTwoPi);
    if (wrapped < 0.0) {
        wrapped += kTwoPi;
    }
    return wrapped;
}

double signedAngleDiff(double from, double to) {
    double diff = wrapAngle(to) - wrapAngle(from);
    if (diff > kPi) {
        diff -= kTwoPi;
    } else if (diff < -kPi) {
        diff += kTwoPi;
    }
    return diff;
}

Config configFromArray(const double* values, int numofDOFs, bool normalizeAngles) {
    Config q(numofDOFs);
    for (int i = 0; i < numofDOFs; ++i) {
        q[i] = normalizeAngles ? wrapAngle(values[i]) : values[i];
    }
    return q;
}

bool isConfigValid(const Config& q, int numofDOFs, double* map, int x_size, int y_size) {
    return IsValidArmConfiguration(const_cast<double*>(q.data()), numofDOFs, map, x_size, y_size) != 0;
}

double jointSpaceDistance(const Config& a, const Config& b) {
    double sum = 0.0;
    for (size_t i = 0; i < a.size(); ++i) {
        double d = signedAngleDiff(a[i], b[i]);
        sum += d * d;
    }
    return std::sqrt(sum);
}

double maxJointDistance(const Config& a, const Config& b) {
    double maxDelta = 0.0;
    for (size_t i = 0; i < a.size(); ++i) {
        maxDelta = std::max(maxDelta, std::fabs(signedAngleDiff(a[i], b[i])));
    }
    return maxDelta;
}

double edgeCost(const Config& a, const Config& b) {
    double cost = 0.0;
    for (size_t i = 0; i < a.size(); ++i) {
        cost += std::fabs(signedAngleDiff(a[i], b[i]));
    }
    return cost;
}

Config interpolateConfig(const Config& a, const Config& b, double t) {
    if (t <= 0.0) {
        return a;
    }
    if (t >= 1.0) {
        return b;
    }
    Config q(a.size());
    for (size_t i = 0; i < a.size(); ++i) {
        q[i] = wrapAngle(a[i] + t * signedAngleDiff(a[i], b[i]));
    }
    return q;
}

Config steerToward(const Config& from, const Config& to, double stepSize) {
    double dist = jointSpaceDistance(from, to);
    if (dist <= stepSize || dist <= kGoalConnectEps) {
        return to;
    }
    return interpolateConfig(from, to, stepSize / dist);
}

bool isEdgeValid(const Config& a, const Config& b, int numofDOFs, double* map, int x_size, int y_size) {
    if (!isConfigValid(a, numofDOFs, map, x_size, y_size) ||
        !isConfigValid(b, numofDOFs, map, x_size, y_size)) {
        return false;
    }

    int steps = std::max(1, static_cast<int>(std::ceil(maxJointDistance(a, b) / kEdgeCheckResolution)));
    for (int i = 1; i < steps; ++i) {
        Config q = interpolateConfig(a, b, static_cast<double>(i) / static_cast<double>(steps));
        if (!isConfigValid(q, numofDOFs, map, x_size, y_size)) {
            return false;
        }
    }
    return true;
}

Config sampleUniformConfig(int numofDOFs, std::mt19937& rng) {
    std::uniform_real_distribution<double> angleDist(0.0, kTwoPi);
    Config q(numofDOFs);
    for (int i = 0; i < numofDOFs; ++i) {
        q[i] = angleDist(rng);
    }
    return q;
}

bool sampleValidConfig(Config& q, int numofDOFs, double* map, int x_size, int y_size, std::mt19937& rng) {
    for (int attempt = 0; attempt < 200; ++attempt) {
        q = sampleUniformConfig(numofDOFs, rng);
        if (isConfigValid(q, numofDOFs, map, x_size, y_size)) {
            return true;
        }
    }
    return false;
}

int addTreeNode(std::vector<MotionNode>& tree, const Config& q, int parent, double cost) {
    MotionNode node;
    node.q = q;
    node.parent = parent;
    node.cost = cost;
    tree.push_back(node);
    int idx = static_cast<int>(tree.size()) - 1;
    if (parent >= 0) {
        tree[parent].children.push_back(idx);
    }
    return idx;
}

int nearestNode(const std::vector<MotionNode>& tree, const Config& q) {
    int best = 0;
    double bestDist = std::numeric_limits<double>::infinity();
    for (int i = 0; i < static_cast<int>(tree.size()); ++i) {
        double d = jointSpaceDistance(tree[i].q, q);
        if (d < bestDist) {
            bestDist = d;
            best = i;
        }
    }
    return best;
}

std::vector<int> nearNodes(const std::vector<MotionNode>& tree, const Config& q, double radius) {
    std::vector<int> result;
    for (int i = 0; i < static_cast<int>(tree.size()); ++i) {
        if (jointSpaceDistance(tree[i].q, q) <= radius) {
            result.push_back(i);
        }
    }
    return result;
}

void removeChild(std::vector<MotionNode>& tree, int parent, int child) {
    if (parent < 0) {
        return;
    }
    std::vector<int>& children = tree[parent].children;
    children.erase(std::remove(children.begin(), children.end(), child), children.end());
}

void updateSubtreeCosts(std::vector<MotionNode>& tree, int idx) {
    for (int child : tree[idx].children) {
        tree[child].cost = tree[idx].cost + edgeCost(tree[idx].q, tree[child].q);
        updateSubtreeCosts(tree, child);
    }
}

std::vector<Config> traceTreePath(const std::vector<MotionNode>& tree, int idx) {
    std::vector<Config> path;
    while (idx >= 0) {
        path.push_back(tree[idx].q);
        idx = tree[idx].parent;
    }
    std::reverse(path.begin(), path.end());
    return path;
}

void appendWithoutDuplicate(std::vector<Config>& dst, const std::vector<Config>& src) {
    for (const Config& q : src) {
        if (dst.empty() || jointSpaceDistance(dst.back(), q) > 1e-8) {
            dst.push_back(q);
        }
    }
}

std::vector<Config> densifyPath(const std::vector<Config>& sparsePath) {
    std::vector<Config> dense;
    if (sparsePath.empty()) {
        return dense;
    }
    dense.push_back(sparsePath.front());
    for (int i = 1; i < static_cast<int>(sparsePath.size()); ++i) {
        const Config& a = sparsePath[i - 1];
        const Config& b = sparsePath[i];
        int steps = std::max(1, static_cast<int>(std::ceil(maxJointDistance(a, b) / kOutputResolution)));
        for (int s = 1; s <= steps; ++s) {
            Config q = interpolateConfig(a, b, static_cast<double>(s) / static_cast<double>(steps));
            if (jointSpaceDistance(dense.back(), q) > 1e-8) {
                dense.push_back(q);
            }
        }
    }
    return dense;
}

void shortcutPath(std::vector<Config>& path, int numofDOFs, double* map, int x_size, int y_size,
                  std::mt19937& rng, int iterations) {
    if (path.size() <= 2) {
        return;
    }
    for (int iter = 0; iter < iterations && path.size() > 2; ++iter) {
        std::uniform_int_distribution<int> idxDist(0, static_cast<int>(path.size()) - 1);
        int a = idxDist(rng);
        int b = idxDist(rng);
        if (a > b) {
            std::swap(a, b);
        }
        if (b <= a + 1) {
            continue;
        }
        if (isEdgeValid(path[a], path[b], numofDOFs, map, x_size, y_size)) {
            path.erase(path.begin() + a + 1, path.begin() + b);
        }
    }
}

void setPlanFromPath(std::vector<Config> sparsePath,
                     const Config& exactStart,
                     const Config& exactGoal,
                     int numofDOFs,
                     double*** plan,
                     int* planlength) {
    if (sparsePath.empty()) {
        sparsePath.push_back(exactStart);
        sparsePath.push_back(exactGoal);
    }

    sparsePath.front() = exactStart;
    sparsePath.back() = exactGoal;
    std::vector<Config> dense = densifyPath(sparsePath);
    if (dense.empty()) {
        dense.push_back(exactStart);
        dense.push_back(exactGoal);
    }
    dense.front() = exactStart;
    dense.back() = exactGoal;

    *planlength = static_cast<int>(dense.size());
    *plan = static_cast<double**>(std::malloc((*planlength) * sizeof(double*)));
    for (int i = 0; i < *planlength; ++i) {
        (*plan)[i] = static_cast<double*>(std::malloc(numofDOFs * sizeof(double)));
        for (int j = 0; j < numofDOFs; ++j) {
            (*plan)[i][j] = dense[i][j];
        }
    }
}

ExtendStatus extendTree(std::vector<MotionNode>& tree,
                        const Config& target,
                        double stepSize,
                        int numofDOFs,
                        double* map,
                        int x_size,
                        int y_size,
                        int& newIndex) {
    int nearIdx = nearestNode(tree, target);
    if (jointSpaceDistance(tree[nearIdx].q, target) <= kGoalConnectEps) {
        newIndex = nearIdx;
        return REACHED;
    }
    Config qNew = steerToward(tree[nearIdx].q, target, stepSize);
    if (!isEdgeValid(tree[nearIdx].q, qNew, numofDOFs, map, x_size, y_size)) {
        newIndex = nearIdx;
        return TRAPPED;
    }
    double newCost = tree[nearIdx].cost + edgeCost(tree[nearIdx].q, qNew);
    newIndex = addTreeNode(tree, qNew, nearIdx, newCost);
    return jointSpaceDistance(qNew, target) <= kGoalConnectEps ? REACHED : ADVANCED;
}

ExtendStatus connectTree(std::vector<MotionNode>& tree,
                         const Config& target,
                         double stepSize,
                         int numofDOFs,
                         double* map,
                         int x_size,
                         int y_size,
                         int& reachedIndex) {
    ExtendStatus status = TRAPPED;
    int idx = -1;
    do {
        status = extendTree(tree, target, stepSize, numofDOFs, map, x_size, y_size, idx);
        if (status == TRAPPED) {
            reachedIndex = idx;
            return TRAPPED;
        }
    } while (status == ADVANCED);
    reachedIndex = idx;
    return REACHED;
}

std::vector<Config> combineTreePaths(const std::vector<MotionNode>& treeA,
                                     int idxA,
                                     const std::vector<MotionNode>& treeB,
                                     int idxB,
                                     bool treeAStartsAtStart) {
    std::vector<Config> pathA = traceTreePath(treeA, idxA);
    std::vector<Config> pathB = traceTreePath(treeB, idxB);
    std::vector<Config> combined;

    if (treeAStartsAtStart) {
        appendWithoutDuplicate(combined, pathA);
        std::reverse(pathB.begin(), pathB.end());
        appendWithoutDuplicate(combined, pathB);
    } else {
        appendWithoutDuplicate(combined, pathB);
        std::reverse(pathA.begin(), pathA.end());
        appendWithoutDuplicate(combined, pathA);
    }
    return combined;
}

}  // namespace armplanner
