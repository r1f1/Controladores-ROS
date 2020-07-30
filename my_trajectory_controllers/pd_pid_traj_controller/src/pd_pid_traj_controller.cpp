#include <control_msgs/JointControllerState.h>
#include <control_msgs/JointControllerState.h>
#include <control_toolbox/pid.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <ros/node_handle.h>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/Float64.h>
#include <locale.h>
#include "traj_msgs/command_msg.h"

namespace pd_pid_traj_controller_ns 
{
	class PdPidTrajController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
	{
		bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n){
			setlocale(LC_ALL, "");

			std::string my_joint;
			if (!n.getParam("joint", my_joint)){
				ROS_ERROR("El nombre de la articulaci%cn no es correcto", 243);
				return false;
			}

			joint_ = hw->getHandle(my_joint);

			if (!pid_controller_.init(ros::NodeHandle(n, "pid")))
    			return false;

			controller_state_publisher_.reset(new realtime_tools::RealtimePublisher<control_msgs::JointControllerState>(n, "state", 1));

			sub_command_ = n.subscribe<traj_msgs::command_msg>("command", 1, &PdPidTrajController::setCommandCB, this);

			return true;
		}

		void getGains(double &p, double &i, double &d, double &i_max, double &i_min){
			pid_controller_.getGains(p,i,d,i_max,i_min);
		}
		
		void update(const ros::Time& time, const ros::Duration& period){
			
			double error = command_position - joint_.getPosition();
            double error_dot = command_velocity - joint_.getVelocity();
			double commanded_effort = pid_controller_.computeCommand(error, error_dot, period);
			
			joint_.setCommand(commanded_effort);
			// publish state
			if (loop_count_ % 10 == 0)
			{
				if(controller_state_publisher_ && controller_state_publisher_->trylock())
				{
				controller_state_publisher_->msg_.header.stamp = time;
				controller_state_publisher_->msg_.set_point = command_position;
				controller_state_publisher_->msg_.process_value = joint_.getPosition();
				controller_state_publisher_->msg_.process_value_dot = joint_.getVelocity();
				controller_state_publisher_->msg_.error = error_dot;
				controller_state_publisher_->msg_.time_step = period.toSec();
				controller_state_publisher_->msg_.command = commanded_effort;

				double dummy;
				getGains(controller_state_publisher_->msg_.p,
					controller_state_publisher_->msg_.i,
					controller_state_publisher_->msg_.d,
					controller_state_publisher_->msg_.i_clamp,
					dummy);
				controller_state_publisher_->unlockAndPublish();
				}
			}
			loop_count_++;
		}

		void setCommandCB(const traj_msgs::command_msgConstPtr& msg){
			command_position = msg->pos;
			command_velocity = msg->vel;
		}

		void starting(const ros::Time& time) { }
		void stopping(const ros::Time& time) { }

		private:
			hardware_interface::JointHandle joint_;
			double command_position;
			double command_velocity;
			ros::Subscriber sub_command_;
			int loop_count_;
			std::unique_ptr<realtime_tools::RealtimePublisher<control_msgs::JointControllerState> > controller_state_publisher_;
			control_toolbox::Pid pid_controller_;
	};

	PLUGINLIB_EXPORT_CLASS(pd_pid_traj_controller_ns::PdPidTrajController, controller_interface::ControllerBase);


}


