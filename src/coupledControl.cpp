#include "coupledControl.hpp"

using namespace coupled_control;

bool coupledControl::selectNextManipulatorPosition(
    unsigned int current_waypoint,
    std::vector<std::vector<double>> *armConfig,
    std::vector<double> *nextConfig,
    int negative)
{
    // Selection of the next manipulator configuration depending on the current
    // waypoint (current_segment)
    uint pointer = current_waypoint;

    for (unsigned int i = 0; i < nextConfig->size(); i++)
    {
        nextConfig->at(i) = constrainAngle((*armConfig)[pointer][i], negative);
    }

    // Returns true if it is the last position
    return pointer == (*armConfig).size() - 1;
}

void coupledControl::getArmSpeed(double mGoalSpeed,
                                 std::vector<double> nextConfig,
                                 std::vector<double> lastConfig,
                                 std::vector<double> &vd_arm_abs_speed)
{
    std::vector<double> error(nextConfig.size());
    for (unsigned int i = 0; i < nextConfig.size(); i++)
    {
        error[i] = nextConfig[i] - lastConfig[i];

        if (lastConfig[i] > nextConfig[i])
        {
            if (abs(error[i]) < 2 * PI - abs(error[i]))
                error[i] = -abs(error[i]);
            else
                error[i] = 2 * PI - abs(error[i]);
        }
        else
        {
            if (abs(error[i]) > 2 * PI - abs(error[i]))
                error[i] = -(2 * PI - abs(error[i]));
            else
                error[i] = abs(error[i]);
        }
        if (error[i] > PI)
            error[i] = error[i] - 2 * PI;
        else if (error[i] < -PI)
            error[i] = error[i] + 2 * PI;

        if (abs(error[i]) < 0.005) error[i] = 0;
    }
    double maxError = *std::max_element(error.begin(), error.end());
    if (maxError == 0) maxError = 999999;

    for (unsigned int i = 0; i < vd_arm_abs_speed.size(); i++)
    {
        vd_arm_abs_speed[i] = error[i] * mGoalSpeed / maxError;
    }
}

void coupledControl::modifyMotionCommand(
    std::vector<double> nextConfig,
    std::vector<double> lastConfig,
    std::vector<double> goalPose,
    std::vector<double> currentPose,
    double mGoalSpeed,
    std::vector<double> &vd_arm_abs_speed,
    base::commands::Motion2D &rover_command)
{
    // Speed adaptation ratio
    int saturation = 0;
    getArmSpeed(mGoalSpeed, nextConfig, lastConfig, vd_arm_abs_speed);

    double remainingDistance = sqrt(pow(goalPose[0] - currentPose[0], 2)
                                    + pow(goalPose[1] - currentPose[1], 2));
    double remainingTurn = abs(goalPose[2] - currentPose[2]);

    double time2BaseArrival;
    if (!(rover_command.translation == 0))
        time2BaseArrival = abs(remainingDistance / rover_command.translation);
    else
    {
        if (!(rover_command.rotation == 0))
            time2BaseArrival = abs(remainingTurn / rover_command.rotation);
        else
            time2BaseArrival = 0;
    }

    double time2ArmArrival = 0;
    for (int i = 0; i < nextConfig.size(); i++)
    {

        if (vd_arm_abs_speed[i] != 0)
        {
            time2ArmArrival
                = abs((nextConfig[i] - lastConfig[i]) / vd_arm_abs_speed[i]);
            break;
        }
    }

    // If the arm is slower than the rover body, rover translation and rotation
    // are decelerated
    if (time2ArmArrival > time2BaseArrival)
    {
        double decelerationRatio = time2BaseArrival / time2ArmArrival;
        rover_command.translation
            *= decelerationRatio; // decelerating the rover
        rover_command.rotation *= decelerationRatio;
    }

    // If the arm is faster than the rover body, arm speed is decelerated
    else if (time2ArmArrival < time2BaseArrival)
    {

        double decelerationRatio = time2ArmArrival / time2BaseArrival;
        for (int i = 0; i < vd_arm_abs_speed.size(); i++)
        {
            vd_arm_abs_speed[i] *= decelerationRatio;
        }
    }
}

int coupledControl::findMaxValue(std::vector<float> vect)
{
    unsigned int max = 0;
    for (unsigned int i = 1; i < vect.size(); i++)
    {
        if (abs(vect.at(i)) > abs(vect.at(max))) max = i;
    }
    return max;
}

double coupledControl::constrainAngle(double angle, int negative)
{
    double c = cos(angle);
    double s = sin(angle);

    double na = atan2(s, c);
    if (negative)
    {
        if (na < -PI) na = na + 2 * PI;
        if (na > PI) na = na - 2 * PI;
    }
    else
    {
        if (na < 0) na = na + 2 * PI;
        if (na > 2 * PI) na = na - 2 * PI;
    }
    return na;
}
