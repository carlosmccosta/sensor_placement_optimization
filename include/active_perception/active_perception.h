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

#include <Eigen/Core>

#include <ignition/math.hh>

#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/math/Angle.hh>
#include <gazebo/math/Rand.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/rendering/rendering.hh>
#include <gazebo/sensors/sensors.hh>

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>
#include <pcl_conversions/pcl_conversions.h>

#include <ros/advertise_options.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <ros/time.h>
#include <ros/rate.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
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
		void ProcessNewSceneModelPath(const std_msgs::StringConstPtr &_msg);
		void ProcessingThread();
		void LoadSensors();
		size_t CountNumberOfSamplingSensors(sensors::Sensor_V& _sensors, const std::string& _sensor_name_prefix);
		void OrientSensorsToObservationPoint();
		void PublishSensorsPoses();
		void LoadSceneModel();
		void SetSensorsState(bool _active);
		void OnNewImageFrame(const unsigned char *_image,
						unsigned int _width, unsigned int _height,
						unsigned int _depth, const std::string &_format, size_t _sensor_index);
		void OnNewRGBPointCloud(const float *_pcd,
						unsigned int _width, unsigned int _height,
						unsigned int _depth, const std::string &_format, size_t _sensor_index);
		typename pcl::PointCloud<pcl::PointXYZRGB>::Ptr SegmentSensorDataFromDepthSensor(const float* _xyzrgb_data, size_t _number_of_points, Eigen::Affine3f &_transform_sensor_to_world, size_t _sensor_index);
		bool GetSensorTransformToWorld(size_t _sensor_index, Eigen::Affine3f &_transform);
		bool FilterPointCloud(typename pcl::PointCloud<pcl::PointXYZRGB>::Ptr &_pointcloud);
		bool PublishPointCloud(typename pcl::PointCloud<pcl::PointXYZRGB>::Ptr &_pointcloud, size_t _pubisher_index);
		void WaitForSensorData();
		void ProcessSensorData();
		void FillUniqueRandomIntNumbers(size_t number_of_random_numbers, int min, int max, std::vector<int> &random_numbers);
		size_t PublishAnalysisSurfaceCoverageAnalysis();
		void PrepareNextAnalysis();
		geometry_msgs::Pose MathPoseToRosPose(math::Pose _math_pose);
		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </member-functions>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<


		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <member-functions-templates>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		template <typename Scalar>
		Eigen::Transform<Scalar, 3, Eigen::Affine> PoseToTransform(const math::Pose &_pose) {
			Eigen::Translation<Scalar, 3> translation(_pose.pos.x, _pose.pos.y, _pose.pos.z);
			Eigen::Quaternion<Scalar> rotation(_pose.rot.w, _pose.rot.x, _pose.rot.y, _pose.rot.z);
			return Eigen::Transform<Scalar, 3, Eigen::Affine>(translation * rotation);
		}

		template<typename PointCloudT>
		bool LoadPointCloudfromFile(const std::string& filename, PointCloudT& pointcloud) {
			std::string::size_type index = filename.rfind(".");
			if (index == std::string::npos) return false;
			std::string extension = filename.substr(index + 1);
			if (extension == "pcd") {
				if (pcl::io::loadPCDFile(filename, pointcloud) == 0 && !pointcloud.empty()) return true;
			} else if (extension == "ply") {
				if (pcl::io::loadPLYFile(filename, pointcloud) == 0 && !pointcloud.empty()) {
					// fix PLYReader import
					pointcloud.sensor_origin_ = Eigen::Vector4f::Zero();
					pointcloud.sensor_orientation_ = Eigen::Quaternionf::Identity();
					return true;
				}
			} else {
				pcl::PolygonMesh mesh;
				if (pcl::io::loadPolygonFile(filename, mesh) != 0) { // obj | ply | stl | vtk | doesn't load normals curvature | doesn't load normals from .ply .stl
					pcl::fromPCLPointCloud2(mesh.cloud, pointcloud);
					return !pointcloud.empty();
				}
			}
			return false;
		}
		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </member-functions-templates>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

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
		std::vector<event::ConnectionPtr> color_image_connections_;
		std::vector<event::ConnectionPtr> color_pointcloud_connections_;

		// ROS
		boost::shared_ptr<ros::NodeHandle> rosnode_;
		ros::CallbackQueue queue_;

		// Configurations
		size_t number_of_sampling_sensors_;
		size_t number_of_intended_sensors_;
		size_t ransac_number_of_iterations_;
		double ransac_surface_percentage_stop_threshold_;
		double elapsed_simulation_time_in_seconds_between_sensor_analysis_;
		common::Time polling_sleep_time_;
		size_t number_of_sensor_analysis_performed_;
		bool sensor_orientation_random_roll_;
		uint32_t sensor_data_segmentation_color_r_;
		uint32_t sensor_data_segmentation_color_g_;
		uint32_t sensor_data_segmentation_color_b_;
		float sensor_data_segmentation_color_rgb_;
		std::string sdf_sensors_name_prefix_;
		std::string topics_sampling_sensors_prefix_;
		std::string published_msgs_world_frame_id_;
		std::string published_msgs_frame_id_suffix_;
		std::vector<typename pcl::PointCloud<pcl::PointXYZRGB>::Ptr> sampling_sensors_pointclouds_;
		std::vector<sensor_msgs::Image::Ptr> sampling_sensors_images_;
		size_t number_of_sampling_sensors_pointclouds_received_;
		std::vector<ros::Publisher> sampling_sensors_color_image_publishers_;
		std::vector<ros::Publisher> sampling_sensors_pointcloud_publishers_;
		bool publish_messages_only_when_there_is_subscribers_;

		geometry_msgs::PointStamped observation_point_;
		ros::Subscriber observation_point_subscriber_;
		boost::mutex observation_point_mutex_;
		bool new_observation_point_available_;

		std::string scene_model_path_;
		ros::Subscriber scene_model_path_subscriber_;
		boost::mutex scene_model_path_mutex_;
		bool new_scene_model_path_available_;
		ros::Publisher scene_model_publisher_;
		typename pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene_model_;
		double voxel_grid_filter_leaf_size_;

		// analysis
		ros::Publisher sensors_poses_publisher_;
		ros::Publisher sensors_names_publisher_;
		ros::Publisher sensors_voxel_grid_surface_coverage_publisher_;
		ros::Publisher sensors_best_poses_publisher_;
		ros::Publisher sensors_best_voxel_grid_surface_coverages_publisher_;
		ros::Publisher sensors_best_names_publisher_;
		ros::Publisher sensors_best_merged_pointcloud_publisher_;

		// Processing threads
		boost::thread processing_thread_;
		boost::thread callback_queue_thread_;
	// ========================================================================   </protected-section>  ========================================================================
};
GZ_REGISTER_WORLD_PLUGIN(ActivePerception)
} /* namespace gazebo */
