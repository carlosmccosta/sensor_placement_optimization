#pragma once

/**\file active_perception.h
 * \brief Description...
 *
 * @version 1.0
 * @author carloscosta
 */

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>  <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include <boost/bind.hpp>
#include <boost/thread.hpp>

#include <ignition/math.hh>

#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/math/Angle.hh>
#include <gazebo/math/Rand.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/rendering/rendering.hh>
#include <gazebo/sensors/sensors.hh>

#include <ros/advertise_options.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <ros/rate.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/String.h>
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

namespace gazebo {
// ############################################################################   ActivePerception   ###########################################################################
/**
 * \brief Class for estimating the best sensor disposition for active perception
 */
class ActivePerception : public WorldPlugin {
	// ========================================================================   <public-section>   ===========================================================================
	public:
		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <typedefs>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </typedefs>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <constructors-destructor>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		ActivePerception();
		virtual ~ActivePerception();
		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </constructors-destructor>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <member-functions>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);
		void Init();
		void QueueThread();
		void ProcessNewObservationPoint(const geometry_msgs::PointStampedConstPtr &_msg);
		void ProcessNewModelNames(const std_msgs::StringConstPtr &_msg);
		void ProcessingThread();
		void LoadSensors(common::Time _wait_time = common::Time(0, 200000000));
		size_t CountNumberOfSamplingSensors(sensors::Sensor_V& _sensors, const std::string& _sensor_name_prefix);
		void OrientSensorsToObservationPoint();
		/*void OnNewRGBPointCloud(const float *_pcd,
				unsigned int _width, unsigned int _height,
				unsigned int _depth, const std::string &_format);*/
		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </member-functions>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <gets>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </gets>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <sets>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </sets>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
	// ========================================================================   </public-section>  ===========================================================================

	// ========================================================================   <protected-section>   ========================================================================
	protected:
		// Gazebo
		physics::WorldPtr world_;
		sdf::ElementPtr sdf_;
		std::vector<sensors::DepthCameraSensorPtr> sensors_;
		std::vector<physics::ModelPtr> sensors_models_;

		// ROS
		boost::shared_ptr<ros::NodeHandle> rosnode_;
		ros::CallbackQueue queue_;

		// Configurations
		size_t number_of_sampling_sensors_;
		size_t number_of_intended_sensors_;
		bool sensor_orientaion_random_roll_;
		std::string sampling_sensors_name_prefix_;

		geometry_msgs::PointStamped observation_point_;
		ros::Subscriber observation_point_subscriber_;
		boost::mutex observation_point_mutex_;
		bool new_observation_point_available_;

		std::string observation_models_names_;
		ros::Subscriber observation_models_names_subscriber_;
		boost::mutex observation_models_names_mutex_;
		bool new_observation_models_names_available_;

		// Processing threads
		boost::thread processing_thread_;
		boost::thread callback_queue_thread_;
	// ========================================================================   </protected-section>  ========================================================================
};
GZ_REGISTER_WORLD_PLUGIN(ActivePerception)
} /* namespace gazebo */
