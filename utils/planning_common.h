#pragma once

#include <chrono>
#include <random>
#include <vector>

int IsValidArmConfiguration(double* angles, int numofDOFs, double* map, int x_size, int y_size);

namespace armplanner {

using Config = std::vector<double>;

struct MotionNode {
    Config q;
    int parent;
    double cost;
    std::vector<int> children;
};

struct PlannerStats {
    int verticesGenerated;
    double firstSolutionTime;
    bool usedFallback;
};

enum ExtendStatus {
    TRAPPED = 0,
    ADVANCED = 1,
    REACHED = 2
};

static const double kPi = 3.141592654;
static const double kTwoPi = 2.0 * kPi;
static const double kOutputResolution = kPi / 55.0;
static const double kEdgeCheckResolution = kPi / 55.0;
static const double kGoalConnectEps = 1e-6;

void resetPlannerStats();
PlannerStats getPlannerStats();
void recordVerticesGenerated(int vertices);
void recordFirstSolutionTime(double seconds);
void recordFallbackUsed();
unsigned int plannerSeed(unsigned int baseSeed);

double elapsedSeconds(const std::chrono::steady_clock::time_point& start);
double wrapAngle(double angle);
double signedAngleDiff(double from, double to);
Config configFromArray(const double* values, int numofDOFs, bool normalizeAngles);
bool isConfigValid(const Config& q, int numofDOFs, double* map, int x_size, int y_size);
double jointSpaceDistance(const Config& a, const Config& b);
double maxJointDistance(const Config& a, const Config& b);
double edgeCost(const Config& a, const Config& b);
Config interpolateConfig(const Config& a, const Config& b, double t);
Config steerToward(const Config& from, const Config& to, double stepSize);
bool isEdgeValid(const Config& a, const Config& b, int numofDOFs, double* map, int x_size, int y_size);
Config sampleUniformConfig(int numofDOFs, std::mt19937& rng);
bool sampleValidConfig(Config& q, int numofDOFs, double* map, int x_size, int y_size, std::mt19937& rng);

int addTreeNode(std::vector<MotionNode>& tree, const Config& q, int parent, double cost);
int nearestNode(const std::vector<MotionNode>& tree, const Config& q);
std::vector<int> nearNodes(const std::vector<MotionNode>& tree, const Config& q, double radius);
void removeChild(std::vector<MotionNode>& tree, int parent, int child);
void updateSubtreeCosts(std::vector<MotionNode>& tree, int idx);
std::vector<Config> traceTreePath(const std::vector<MotionNode>& tree, int idx);
void appendWithoutDuplicate(std::vector<Config>& dst, const std::vector<Config>& src);
std::vector<Config> densifyPath(const std::vector<Config>& sparsePath);
void shortcutPath(std::vector<Config>& path, int numofDOFs, double* map, int x_size, int y_size,
                  std::mt19937& rng, int iterations);
void setPlanFromPath(std::vector<Config> sparsePath,
                     const Config& exactStart,
                     const Config& exactGoal,
                     int numofDOFs,
                     double*** plan,
                     int* planlength);

ExtendStatus extendTree(std::vector<MotionNode>& tree,
                        const Config& target,
                        double stepSize,
                        int numofDOFs,
                        double* map,
                        int x_size,
                        int y_size,
                        int& newIndex);
ExtendStatus connectTree(std::vector<MotionNode>& tree,
                         const Config& target,
                         double stepSize,
                         int numofDOFs,
                         double* map,
                         int x_size,
                         int y_size,
                         int& reachedIndex);
std::vector<Config> combineTreePaths(const std::vector<MotionNode>& treeA,
                                     int idxA,
                                     const std::vector<MotionNode>& treeB,
                                     int idxB,
                                     bool treeAStartsAtStart);

std::vector<Config> solveRRTConnectPath(const Config& start,
                                        const Config& goal,
                                        int numofDOFs,
                                        double* map,
                                        int x_size,
                                        int y_size,
                                        unsigned int seed,
                                        double timeoutSeconds);

}  // namespace armplanner
