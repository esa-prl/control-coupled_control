#ifndef _COUPLED_CONTROL_HPP_
#define _COUPLED_CONTROL_HPP_

#include <cstdlib>
#include <iostream>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <base/commands/Motion2D.hpp>

#define PI 3.1415

namespace coupled_control
{
class coupledControl
{
public:
    int findMaxValue(std::vector<float> vect);

    void getArmSpeed(double gain,
		                         std::vector<double> nextConfig,
                                 std::vector<double> lastConfig,
                                 std::vector<double> &vd_arm_abs_speed);

    void modifyMotionCommand(double gain,
		                         std::vector<double> nextConfig,
                                         std::vector<double> lastConfig,
                                         const double mMaxSpeed,
                                         std::vector<double> &vd_arm_abs_speed,
                                         base::commands::Motion2D &rover_command);

    bool selectNextManipulatorPosition(
        unsigned int current_waypoint,
        std::vector<std::vector<double>> *armConfig,
        std::vector<double> *nextConfig,
        int negative);

    double constrainAngle(double angle, int negative);
};

} // end namespace coupled_control

#endif
