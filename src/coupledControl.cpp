#include "coupledControl.hpp"

using namespace coupled_control;

void coupledControl::modifyMotionCommand(double mMaxSpeed, double maxJW, std::vector<double>& jW, base::commands::Motion2D rover_command, base::commands::Motion2D& modified_rover_command)
{
	// Speed adaption relation
	double R = mMaxSpeed/maxJW;
	// Addapt all the arm motion commands to the maximum speed of the real motors
	for (unsigned int i = 0; i < jW.size() ; i++) jW.at(i) = jW.at(i) * R;
	
	// Addapt the rover global speed to the speed of the arm
	double vA = rover_command.translation;
	double vR = rover_command.rotation;
	modified_rover_command.translation = vA * R;
	modified_rover_command.rotation = vR * R;
}

void coupledControl::selectNextManipulatorPosition(int current_waypoint, std::vector<int> assign, std::vector<double> armConfig, std::vector<double>& nextConfig)
{
	// Selection of the next manipulator configuration depending on the current waypoint (current_segment)
	int pointer = assign.at(current_waypoint);
	for (unsigned int i = 0; i < nextConfig.size(); i++)
	{
		nextConfig.at(i) = armConfig.at(nextConfig.size()*pointer + i);
	}
}

void coupledControl::manipulatorMotionControl(double gain, int& saturation, double mMaxSpeed, std::vector<double> nextConfig, std::vector<double> lastConfig, std::vector<double>& jW)
{
	for(unsigned int i = 0; i < nextConfig.size(); i++)
	{
		jW.at(i) = gain * (nextConfig.at(i) - lastConfig.at(i));
		if (jW.at(i) > mMaxSpeed) saturation = 1;
	}
}

int coupledControl::findMaxValue(std::vector<double> vect)
{
	unsigned int max = 0;
	for(unsigned int i = 1; i < vect.size(); i++)
	{
		if(vect.at(i) > vect.at(max)) max = i;
	}
	return max;
}

