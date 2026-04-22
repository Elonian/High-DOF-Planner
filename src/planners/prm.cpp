#include "planners/planner_api.h"
#include "planning_common.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <queue>
#include <random>
#include <utility>
#include <vector>

namespace {

struct RoadmapEdge {
    int to;
    double cost;
};

void connectRoadmapNode(int idx,
                        std::vector<armplanner::Config>& nodes,
                        std::vector<std::vector<RoadmapEdge> >& graph,
                        int k,
                        int numofDOFs,
                        double* map,
                        int x_size,
                        int y_size) {
    using namespace armplanner;

    std::vector<std::pair<double, int> > candidates;
    candidates.reserve(nodes.size());
    for (int i = 0; i < static_cast<int>(nodes.size()); ++i) {
        if (i == idx) {
            continue;
        }
        candidates.push_back(std::make_pair(jointSpaceDistance(nodes[idx], nodes[i]), i));
    }
    std::sort(candidates.begin(), candidates.end());

    int connected = 0;
    for (const auto& candidate : candidates) {
        if (connected >= k) {
            break;
        }
        int other = candidate.second;
        if (isEdgeValid(nodes[idx], nodes[other], numofDOFs, map, x_size, y_size)) {
            double c = edgeCost(nodes[idx], nodes[other]);
            graph[idx].push_back({other, c});
            graph[other].push_back({idx, c});
            ++connected;
        }
    }
}

std::vector<armplanner::Config> dijkstraRoadmapPath(const std::vector<armplanner::Config>& nodes,
                                                    const std::vector<std::vector<RoadmapEdge> >& graph,
                                                    int startIdx,
                                                    int goalIdx) {
    const double inf = std::numeric_limits<double>::infinity();
    std::vector<double> dist(nodes.size(), inf);
    std::vector<int> parent(nodes.size(), -1);
    typedef std::pair<double, int> QueueEntry;
    std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<QueueEntry> > pq;

    dist[startIdx] = 0.0;
    pq.push(std::make_pair(0.0, startIdx));
    while (!pq.empty()) {
        QueueEntry cur = pq.top();
        pq.pop();
        double d = cur.first;
        int u = cur.second;
        if (d > dist[u] + 1e-12) {
            continue;
        }
        if (u == goalIdx) {
            break;
        }
        for (const RoadmapEdge& e : graph[u]) {
            double nd = d + e.cost;
            if (nd + 1e-12 < dist[e.to]) {
                dist[e.to] = nd;
                parent[e.to] = u;
                pq.push(std::make_pair(nd, e.to));
            }
        }
    }

    if (!std::isfinite(dist[goalIdx])) {
        return {};
    }

    std::vector<armplanner::Config> path;
    for (int cur = goalIdx; cur >= 0; cur = parent[cur]) {
        path.push_back(nodes[cur]);
        if (cur == startIdx) {
            break;
        }
    }
    std::reverse(path.begin(), path.end());
    return path;
}

std::vector<armplanner::Config> solvePRMPath(const armplanner::Config& start,
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
    std::vector<Config> nodes;
    std::vector<std::vector<RoadmapEdge> > graph;
    nodes.reserve(2600);
    graph.reserve(2600);

    nodes.push_back(start);
    graph.push_back(std::vector<RoadmapEdge>());
    nodes.push_back(goal);
    graph.push_back(std::vector<RoadmapEdge>());

    const int kRoadmap = std::max(12, std::min(28, 6 + 2 * numofDOFs));
    const int targetSamples = 900 + 220 * numofDOFs;
    auto startTime = std::chrono::steady_clock::now();

    connectRoadmapNode(1, nodes, graph, kRoadmap, numofDOFs, map, x_size, y_size);

    while (static_cast<int>(nodes.size()) < targetSamples + 2 &&
           elapsedSeconds(startTime) < timeoutSeconds * 0.82) {
        Config q;
        if (!sampleValidConfig(q, numofDOFs, map, x_size, y_size, rng)) {
            continue;
        }
        int idx = static_cast<int>(nodes.size());
        nodes.push_back(q);
        graph.push_back(std::vector<RoadmapEdge>());
        connectRoadmapNode(idx, nodes, graph, kRoadmap, numofDOFs, map, x_size, y_size);
    }

    connectRoadmapNode(0, nodes, graph, 50, numofDOFs, map, x_size, y_size);
    connectRoadmapNode(1, nodes, graph, 50, numofDOFs, map, x_size, y_size);
    std::vector<Config> path = dijkstraRoadmapPath(nodes, graph, 0, 1);
    if (!path.empty()) {
        recordVerticesGenerated(static_cast<int>(nodes.size()));
        recordFirstSolutionTime(elapsedSeconds(startTime));
        return path;
    }

    recordVerticesGenerated(static_cast<int>(nodes.size()));
    recordFallbackUsed();
    return solveRRTConnectPath(start, goal, numofDOFs, map, x_size, y_size, seed + 211, timeoutSeconds * 0.35);
}

}  // namespace

void planPRM(
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

    std::mt19937 shortcutRng(plannerSeed(44011u + static_cast<unsigned int>(numofDOFs)));
    std::vector<Config> path = solvePRMPath(
        start, goal, numofDOFs, map, x_size, y_size, plannerSeed(42007u), 4.45);
    shortcutPath(path, numofDOFs, map, x_size, y_size, shortcutRng, 260);
    setPlanFromPath(path, exactStart, exactGoal, numofDOFs, plan, planlength);
}
