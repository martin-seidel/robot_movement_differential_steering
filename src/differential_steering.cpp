
#include <pluginlib/class_list_macros.h>
#include <robot_movement_differential_steering/differential_steering.hpp>

PLUGINLIB_EXPORT_CLASS(robot_movement_config::DifferentialSteering, robot_movement_config::PluginInterface)


namespace robot_movement_config
{
   DifferentialSteering::DifferentialSteering():max_mps(0.5), axis_length(0.46), use_superposition(true), preferRotation(true) {}

   int DifferentialSteering::onInit(ros::NodeHandle roshandle) {


      addWheel("left_tread", "velocity_left/command", 0.108, (2*M_PI)/60, (2*M_PI)/2000);

      addWheel("right_tread", "velocity_right/command", 0.108, (2*M_PI)/60, (2*M_PI)/2000);

      ROS_INFO("[plugin direct] done init.");
   }

   std::vector<PluginInterface::wheel_velocity> DifferentialSteering::getWheelVelFromCmdVel( geometry_msgs::Twist cmd_vel ) {
      double lx = cmd_vel.linear.x; // m/s
      double az = cmd_vel.angular.z; // rad/s
      double sl, sr; // m/s

      if (!use_superposition) {
         if (preferRotation) {
            if ( fabs(az) > 0.001 ) {
               lx = 0;
            }
         } else {
            if ( fabs(lx) > 0.001 ) {
               az = 0;
            }
         }
      }

      if ( fabs(az) > 0.001 ) {
         double r = fabs(lx/az) - axis_length * 0.5;
         double ds = axis_length * fabs(az) * 0.5;

         double a;
         if (fabs(lx) + ds > max_mps) {
            a = max_mps / (r + axis_length);
         } else {
            a = (lx - ds) / r;
         }

         sl = (az > 0) ? r * a : a * (r + axis_length);
         sr = (az < 0) ? r * a : a * (r + axis_length);
      } else {
         sl = sr = (lx < max_mps) ? lx : max_mps;
      }

      std::vector<PluginInterface::wheel_velocity> temp;
      temp.push_back({"left_tread", sl});
      temp.push_back({"right_tread", sr});
      return temp;
   }

};
