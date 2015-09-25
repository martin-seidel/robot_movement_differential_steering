
#ifndef RMC_DIFFERENTIAL_STEERING_PLUGIN_H_
#define RMC_DIFFERENTIAL_STEERING_PLUGIN_H_

#include <robot_movement_config/plugin_interface.hpp>

namespace robot_movement_config
{
	/**This class create a direct link between the start and target pose.
	 * The resulting path only contain these two points.
	 **/
	class DifferentialSteering : public PluginInterface
	{
		//geometric data
		double axis_length;
		double max_mps;
		bool use_superposition;
		bool preferRotation;

		int onInit(ros::NodeHandle roshandle);

		std::vector<PluginInterface::wheel_velocity> getWheelVelFromCmdVel(geometry_msgs::Twist cmd_vel);

	public:
		DifferentialSteering();	/**< Intentionally left empty **/
		virtual ~DifferentialSteering(){}	/**< Intentionally left empty **/
	};
};
#endif //RMC_DIFFERENTIAL_STEERING_PLUGIN_H_
