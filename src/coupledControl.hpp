#ifndef _COUPLED_CONTROL_HPP_
#define _COUPLED_CONTROL_HPP_

#include "MotionCommand.h"
#include "Waypoint.hpp"
#include <cstdlib>
#include <iostream>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <vector>

#define PI 3.1415

using namespace base;
using namespace proxy_library;
using namespace std;

namespace coupled_control
{
class coupledControl
{
public:
    int findMaxValue(std::vector<float> vect);
    void modifyMotionCommand(double gain,
		                         std::vector<double> nextConfig,
                                         std::vector<double> lastConfig,
                                         const double mMaxSpeed,
                                         std::vector<double> &vd_arm_abs_speed,
                                         MotionCommand &rover_command);

    bool selectNextManipulatorPosition(
        unsigned int current_waypoint,
        std::vector<std::vector<double>> *armConfig,
        std::vector<double> *nextConfig,
        int negative);

    double constrainAngle(double angle, int negative);
};

} // end namespace coupled_control

#endif
