/*=================================================================
 *
 * planner.cpp
 *
 * Main planner entrypoint. Planner implementations live under
 * src/planners/ and shared sampling utilities live under utils/.
 *
 *=================================================================*/

#include "planners/planner_api.h"
#include "planning_common.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cctype>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <regex>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>

/* Planner Ids */
#define RRT         0
#define RRTCONNECT  1
#define RRTSTAR     2
#define PRM         3

#define GETMAPINDEX(X, Y, XSIZE, YSIZE) (Y*XSIZE + X)

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#define PI 3.141592654

// The length of each link in the arm.
#define LINKLENGTH_CELLS 10

#ifndef MAPS_DIR
#define MAPS_DIR "../maps"
#endif
#ifndef OUTPUT_DIR
#define OUTPUT_DIR "../output"
#endif

using std::cout;
using std::endl;
using std::make_tuple;
using std::runtime_error;
using std::string;
using std::tie;
using std::tuple;
using std::vector;

//*******************************************************************************************************************//
//                                                GIVEN FUNCTIONS                                                    //
//*******************************************************************************************************************//

tuple<double*, int, int> loadMap(string filepath) {
    std::FILE* f = fopen(filepath.c_str(), "r");
    if (!f) {
        printf("Opening file failed! \n");
        throw runtime_error("Opening map file failed!");
    }

    int height, width;
    if (fscanf(f, "height %d\nwidth %d\n", &height, &width) != 2) {
        throw runtime_error("Invalid loadMap parsing map metadata");
    }

    double* map = new double[height * width];
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            char c;
            do {
                if (fscanf(f, "%c", &c) != 1) {
                    throw runtime_error("Invalid parsing individual map data");
                }
            } while (isspace(c));

            map[GETMAPINDEX(x, y, width, height)] = (c == '0') ? 0 : 1;
        }
    }
    fclose(f);
    return make_tuple(map, width, height);
}

vector<string> split(const string& str, const string& delim) {
    const std::regex ws_re(delim);
    return {
        std::sregex_token_iterator(str.begin(), str.end(), ws_re, -1),
        std::sregex_token_iterator()
    };
}

double* doubleArrayFromString(string str) {
    vector<string> vals = split(str, ",");
    double* ans = new double[vals.size()];
    for (int i = 0; i < static_cast<int>(vals.size()); ++i) {
        ans[i] = std::stod(vals[i]);
    }
    return ans;
}

bool equalDoubleArrays(double* v1, double* v2, int size) {
    for (int i = 0; i < size; ++i) {
        if (std::fabs(v1[i] - v2[i]) > 1e-3) {
            cout << endl;
            return false;
        }
    }
    return true;
}

typedef struct {
    int X1, Y1;
    int X2, Y2;
    int Increment;
    int UsingYIndex;
    int DeltaX, DeltaY;
    int DTerm;
    int IncrE, IncrNE;
    int XIndex, YIndex;
    int Flipped;
} bresenham_param_t;

void ContXY2Cell(double x, double y, short unsigned int* pX, short unsigned int* pY, int x_size, int y_size) {
    double cellsize = 1.0;
    *pX = static_cast<int>(x / cellsize);
    if (x < 0) {
        *pX = 0;
    }
    if (*pX >= x_size) {
        *pX = x_size - 1;
    }

    *pY = static_cast<int>(y / cellsize);
    if (y < 0) {
        *pY = 0;
    }
    if (*pY >= y_size) {
        *pY = y_size - 1;
    }
}

void get_bresenham_parameters(int p1x, int p1y, int p2x, int p2y, bresenham_param_t* params) {
    params->UsingYIndex = 0;

    if (std::fabs(static_cast<double>(p2y - p1y) / static_cast<double>(p2x - p1x)) > 1) {
        (params->UsingYIndex)++;
    }

    if (params->UsingYIndex) {
        params->Y1 = p1x;
        params->X1 = p1y;
        params->Y2 = p2x;
        params->X2 = p2y;
    } else {
        params->X1 = p1x;
        params->Y1 = p1y;
        params->X2 = p2x;
        params->Y2 = p2y;
    }

    if ((p2x - p1x) * (p2y - p1y) < 0) {
        params->Flipped = 1;
        params->Y1 = -params->Y1;
        params->Y2 = -params->Y2;
    } else {
        params->Flipped = 0;
    }

    if (params->X2 > params->X1) {
        params->Increment = 1;
    } else {
        params->Increment = -1;
    }

    params->DeltaX = params->X2 - params->X1;
    params->DeltaY = params->Y2 - params->Y1;
    params->IncrE = 2 * params->DeltaY * params->Increment;
    params->IncrNE = 2 * (params->DeltaY - params->DeltaX) * params->Increment;
    params->DTerm = (2 * params->DeltaY - params->DeltaX) * params->Increment;
    params->XIndex = params->X1;
    params->YIndex = params->Y1;
}

void get_current_point(bresenham_param_t* params, int* x, int* y) {
    if (params->UsingYIndex) {
        *y = params->XIndex;
        *x = params->YIndex;
        if (params->Flipped) {
            *x = -*x;
        }
    } else {
        *x = params->XIndex;
        *y = params->YIndex;
        if (params->Flipped) {
            *y = -*y;
        }
    }
}

int get_next_point(bresenham_param_t* params) {
    if (params->XIndex == params->X2) {
        return 0;
    }

    params->XIndex += params->Increment;
    if (params->DTerm < 0 || (params->Increment < 0 && params->DTerm <= 0)) {
        params->DTerm += params->IncrE;
    } else {
        params->DTerm += params->IncrNE;
        params->YIndex += params->Increment;
    }
    return 1;
}

int IsValidLineSegment(double x0, double y0, double x1, double y1, double* map, int x_size, int y_size) {
    bresenham_param_t params;
    int nX, nY;
    short unsigned int nX0, nY0, nX1, nY1;

    if (x0 < 0 || x0 >= x_size ||
        x1 < 0 || x1 >= x_size ||
        y0 < 0 || y0 >= y_size ||
        y1 < 0 || y1 >= y_size) {
        return 0;
    }

    ContXY2Cell(x0, y0, &nX0, &nY0, x_size, y_size);
    ContXY2Cell(x1, y1, &nX1, &nY1, x_size, y_size);

    get_bresenham_parameters(nX0, nY0, nX1, nY1, &params);
    do {
        get_current_point(&params, &nX, &nY);
        if (map[GETMAPINDEX(nX, nY, x_size, y_size)] == 1) {
            return 0;
        }
    } while (get_next_point(&params));

    return 1;
}

int IsValidArmConfiguration(double* angles, int numofDOFs, double* map, int x_size, int y_size) {
    double x0, y0, x1, y1;

    x1 = static_cast<double>(x_size) / 2.0;
    y1 = 0;
    for (int i = 0; i < numofDOFs; i++) {
        x0 = x1;
        y0 = y1;
        x1 = x0 + LINKLENGTH_CELLS * std::cos(2 * PI - angles[i]);
        y1 = y0 - LINKLENGTH_CELLS * std::sin(2 * PI - angles[i]);

        if (!IsValidLineSegment(x0, y0, x1, y1, map, x_size, y_size)) {
            return 0;
        }
    }
    return 1;
}

//*******************************************************************************************************************//
//                                          DEFAULT PLANNER FUNCTION                                                 //
//*******************************************************************************************************************//

void planner(
    double* map,
    int x_size,
    int y_size,
    double* armstart_anglesV_rad,
    double* armgoal_anglesV_rad,
    int numofDOFs,
    double*** plan,
    int* planlength) {
    *plan = NULL;
    *planlength = 0;

    double distance = 0;
    for (int j = 0; j < numofDOFs; j++) {
        if (distance < std::fabs(armstart_anglesV_rad[j] - armgoal_anglesV_rad[j])) {
            distance = std::fabs(armstart_anglesV_rad[j] - armgoal_anglesV_rad[j]);
        }
    }

    int numofsamples = static_cast<int>(distance / (PI / 20));
    if (numofsamples < 2) {
        printf("the arm is already at the goal\n");
        return;
    }

    *plan = static_cast<double**>(std::malloc(numofsamples * sizeof(double*)));
    for (int i = 0; i < numofsamples; i++) {
        (*plan)[i] = static_cast<double*>(std::malloc(numofDOFs * sizeof(double)));
        for (int j = 0; j < numofDOFs; j++) {
            (*plan)[i][j] = armstart_anglesV_rad[j] +
                (static_cast<double>(i) / (numofsamples - 1)) *
                (armgoal_anglesV_rad[j] - armstart_anglesV_rad[j]);
        }
        if (!IsValidArmConfiguration((*plan)[i], numofDOFs, map, x_size, y_size)) {
            printf("ERROR: Invalid arm configuration!!!\n");
        }
    }
    *planlength = numofsamples;
}

static void plannerRRT(
    double* map,
    int x_size,
    int y_size,
    double* armstart_anglesV_rad,
    double* armgoal_anglesV_rad,
    int numofDOFs,
    double*** plan,
    int* planlength) {
    planRRT(map, x_size, y_size, armstart_anglesV_rad, armgoal_anglesV_rad, numofDOFs, plan, planlength);
}

static void plannerRRTConnect(
    double* map,
    int x_size,
    int y_size,
    double* armstart_anglesV_rad,
    double* armgoal_anglesV_rad,
    int numofDOFs,
    double*** plan,
    int* planlength) {
    planRRTConnect(map, x_size, y_size, armstart_anglesV_rad, armgoal_anglesV_rad, numofDOFs, plan, planlength);
}

static void plannerRRTStar(
    double* map,
    int x_size,
    int y_size,
    double* armstart_anglesV_rad,
    double* armgoal_anglesV_rad,
    int numofDOFs,
    double*** plan,
    int* planlength) {
    planRRTStar(map, x_size, y_size, armstart_anglesV_rad, armgoal_anglesV_rad, numofDOFs, plan, planlength);
}

static void plannerPRM(
    double* map,
    int x_size,
    int y_size,
    double* armstart_anglesV_rad,
    double* armgoal_anglesV_rad,
    int numofDOFs,
    double*** plan,
    int* planlength) {
    planPRM(map, x_size, y_size, armstart_anglesV_rad, armgoal_anglesV_rad, numofDOFs, plan, planlength);
}

//*******************************************************************************************************************//
//                                                MAIN FUNCTION                                                      //
//*******************************************************************************************************************//

int main(int argc, char** argv) {
    double* map;
    int x_size, y_size;

    std::string mapDirPath = MAPS_DIR;
    std::string mapFilePath = mapDirPath + "/" + argv[1];
    std::cout << "Reading problem definition from: " << mapFilePath << std::endl;
    tie(map, x_size, y_size) = loadMap(mapFilePath);

    const int numOfDOFs = std::stoi(argv[2]);
    double* startPos = doubleArrayFromString(argv[3]);
    double* goalPos = doubleArrayFromString(argv[4]);
    int whichPlanner = std::stoi(argv[5]);

    std::string outputDir = OUTPUT_DIR;
    string outputFile = outputDir + "/" + argv[6];
    std::cout << "Writing solution to: " << outputFile << std::endl;

    if (!IsValidArmConfiguration(startPos, numOfDOFs, map, x_size, y_size) ||
        !IsValidArmConfiguration(goalPos, numOfDOFs, map, x_size, y_size)) {
        throw runtime_error("Invalid start or goal configuration!\n");
    }

    double** plan = NULL;
    int planlength = 0;
    armplanner::resetPlannerStats();

    if (whichPlanner == PRM) {
        plannerPRM(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength);
    } else if (whichPlanner == RRT) {
        plannerRRT(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength);
    } else if (whichPlanner == RRTCONNECT) {
        plannerRRTConnect(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength);
    } else if (whichPlanner == RRTSTAR) {
        plannerRRTStar(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength);
    } else {
        planner(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength);
    }

    if (!plan || planlength <= 0 ||
        !equalDoubleArrays(plan[0], startPos, numOfDOFs) ||
        !equalDoubleArrays(plan[planlength - 1], goalPos, numOfDOFs)) {
        throw std::runtime_error("Start or goal position not matching");
    }

    std::ofstream m_log_fstream;
    m_log_fstream.open(outputFile, std::ios::trunc);
    if (!m_log_fstream.is_open()) {
        throw std::runtime_error("Cannot open file");
    }

    m_log_fstream << mapFilePath << endl;
    m_log_fstream << std::setprecision(12);
    for (int i = 0; i < planlength; ++i) {
        for (int k = 0; k < numOfDOFs; ++k) {
            m_log_fstream << plan[i][k] << ",";
        }
        m_log_fstream << endl;
    }

    const char* statsFile = std::getenv("HDOF_PLANNER_STATS");
    if (statsFile) {
        armplanner::PlannerStats stats = armplanner::getPlannerStats();
        std::ofstream stats_stream(statsFile, std::ios::trunc);
        if (stats_stream.is_open()) {
            stats_stream << "vertices_generated,first_solution_time,used_fallback,plan_waypoints\n";
            stats_stream << stats.verticesGenerated << ","
                         << stats.firstSolutionTime << ","
                         << (stats.usedFallback ? 1 : 0) << ","
                         << planlength << "\n";
        }
    }
}
