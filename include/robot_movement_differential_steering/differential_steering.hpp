
#ifndef RMC_DIFFERENTIAL_STEERING_PLUGIN_H_
#define RMC_DIFFERENTIAL_STEERING_PLUGIN_H_

#include <robot_movement_config/plugin_interface.hpp>

namespace robot_movement_config
{
	/**
	 **/
	class DifferentialSteering : public PluginInterface
	{
		//typedef PluginInterface::wheel_data wheel_data;

		//geometric data
		double axis_length; //urdf? parser as protected var
		double max_mps; //more restrictions; ->interface
		bool use_superposition;
		bool preferRotation;

		bool firstOdom;
		double lastPositionLeft;
		double lastPositionRight;

		int onInit(ros::NodeHandle roshandle);

		std::vector<PluginInterface::wheel_data> getWheelVelFromCmdVel(geometry_msgs::Twist cmd_vel);

		geometry_msgs::Transform getOdomDiff(std::vector<wheel_data> current_position);

	public:
		DifferentialSteering();	/**< Intentionally left empty **/
		virtual ~DifferentialSteering(){}	/**< Intentionally left empty **/
	};
};
#endif //RMC_DIFFERENTIAL_STEERING_PLUGIN_H_
