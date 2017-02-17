#pragma once

/**\file active_perception.h
 * \brief Description...
 *
 * @version 1.0
 * @author carloscosta
 */

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>  <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#include <algorithm>
#include <cstdint>
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

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl_conversions/pcl_conversions.h>

#include <ros/advertise_options.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <ros/rate.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/PointCloud2.h>
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
		void SetSensorsState(bool _active);
		void OnNewDepthFrame(const float *_image,
				unsigned int _width, unsigned int _height,
				unsigned int _depth, const std::string &_format, size_t _sensor_index);
		void OnNewImageFrame(const unsigned char *_image,
						unsigned int _width, unsigned int _height,
						unsigned int _depth, const std::string &_format, size_t _sensor_index);
		void OnNewRGBPointCloud(const float *_pcd,
						unsigned int _width, unsigned int _height,
						unsigned int _depth, const std::string &_format, size_t _sensor_index);
		typename pcl::PointCloud<pcl::PointXYZRGB>::Ptr SegmentSensorDataFromDepthSensor(const float* _xyzrgb_data, size_t _number_of_points);
		bool PublishPointCloud(typename pcl::PointCloud<pcl::PointXYZRGB>::Ptr _pointcloud, size_t _pubisher_index);
		void ProcessSensorData();
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
		std::vector<event::ConnectionPtr> depth_image_connections_;
		std::vector<event::ConnectionPtr> color_image_connections_;
		std::vector<event::ConnectionPtr> color_pointcloud_connections_;

		// ROS
		boost::shared_ptr<ros::NodeHandle> rosnode_;
		ros::CallbackQueue queue_;

		// Configurations
		size_t number_of_sampling_sensors_;
		size_t number_of_intended_sensors_;
		double elapsed_simulation_time_in_seconds_between_sensor_analysis_;
		bool sensor_orientation_random_roll_;
		float sensor_data_segmentation_color_rgb_;
		std::string sampling_sensors_name_prefix_;
		std::string topic_sampling_sensors_prefix_;
		std::vector<typename pcl::PointCloud<pcl::PointXYZRGB>::Ptr> sampling_sensors_pointclouds_;
		std::vector<ros::Publisher> sampling_sensors_depth_image_publishers_;
		std::vector<ros::Publisher> sampling_sensors_color_image_publishers_;
		std::vector<ros::Publisher> sampling_sensors_pointcloud_publishers_;

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
