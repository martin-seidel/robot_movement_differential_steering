
#include <pluginlib/class_list_macros.h>
#include <robot_movement_differential_steering/differential_steering.hpp>

PLUGINLIB_EXPORT_CLASS(robot_movement_config::DifferentialSteering, robot_movement_config::PluginInterface)


namespace robot_movement_config
{
   DifferentialSteering::DifferentialSteering():max_mps(1.5), axis_length(0.46), use_superposition(true), preferRotation(true) {}

   int DifferentialSteering::onInit(ros::NodeHandle roshandle) {


      addWheel("left_tread", "velocity_left/command", 0.108, (2*M_PI)/60, (2*M_PI)/2000);

      addWheel("right_tread", "velocity_right/command", 0.108, (2*M_PI)/60, (2*M_PI)/2000);

      firstOdom = true;
		lastPositionLeft = 0;
		lastPositionRight = 0;

      ROS_INFO("[plugin DifferentialSteering] done init.");
   }

   std::vector<PluginInterface::wheel_data> DifferentialSteering::getWheelVelFromCmdVel( geometry_msgs::Twist cmd_vel ) {
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

      std::vector<PluginInterface::wheel_data> msg;
      msg.push_back({"left_tread", sl});
      msg.push_back({"right_tread", sr});
      return msg;
   }

   geometry_msgs::Transform DifferentialSteering::getOdomDiff(std::vector<wheel_data> current_position) {
      bool found_left = false, found_right = false;
      double d_left, d_right;

      for (wheel_data wheel : current_position){
         if (wheel.link.compare("left_tread") == 0) {
            d_left = wheel.data - lastPositionLeft;
            lastPositionLeft = wheel.data;
            found_left = true;
         } else if (wheel.link.compare("right_tread") == 0) {
            d_right = wheel.data - lastPositionRight;
            lastPositionRight = wheel.data;
            found_right = true;
         }
      }

      if (firstOdom) {
         d_left = d_right = 0.0;
         firstOdom = false;
      }

      geometry_msgs::Transform msg;
      msg.translation.z = 0;
      if (found_left && found_right) {
         if ( fabs(d_left - d_right) > 0.001 ) {
            double yaw = (d_right - d_left)/axis_length;
            double d_middle = d_left / yaw + axis_length * 0.5;
            //ROS_INFO("%.3lf %.3lf", yaw, d_middle);
            msg.translation.x = sin(yaw) * d_middle;
            msg.translation.y = (1 - cos(yaw)) * d_middle;
            msg.rotation = tf::createQuaternionMsgFromYaw(yaw);
         } else {
            msg.translation.x = (d_left + d_right) * 0.5;
            msg.translation.y = 0;
            msg.rotation = tf::createQuaternionMsgFromYaw(0.0);
         }
      } else {
         msg.translation.x = 0;
         msg.translation.y = 0;
         msg.rotation = tf::createQuaternionMsgFromYaw(0.0);
         ROS_WARN("Receive not odom data for at least one wheel.");
      }

      return msg;
   }

};
