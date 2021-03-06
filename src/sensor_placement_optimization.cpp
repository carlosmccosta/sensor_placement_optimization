/**\file sensor_placement_optimization.cpp
 * \brief Description...
 *
 * @version 1.0
 * @author carloscosta
 */

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#include <sensor_placement_optimization/sensor_placement_optimization.h>
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

namespace gazebo {

// =============================================================================  <public-section>  ============================================================================
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <constructors-destructor>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
SensorPlacementOptimization::SensorPlacementOptimization() :
		number_of_sampling_sensors_(100),
		number_of_intended_sensors_(1),
		ransac_number_of_iterations_(100),
		ransac_surface_percentage_stop_threshold_(90.0),
		sensors_sequential_scene_rendering_(false),
		polling_sleep_time_(0, 200000000),
		number_of_sensor_analysis_performed_(0),
		sensor_orientation_random_roll_(true),
		sensor_data_segmentation_color_r_(0),
		sensor_data_segmentation_color_g_(255),
		sensor_data_segmentation_color_b_(0),
		sensor_data_segmentation_color_rgb_(0xff00),
		number_of_sampling_sensors_pointclouds_received_(0),
		publish_messages_only_when_there_is_subscribers_(true),
		new_observation_point_available_(false),
		new_scene_model_path_available_(false),
		voxel_grid_filter_leaf_size_(0.007) {
}

SensorPlacementOptimization::~SensorPlacementOptimization() {}
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </constructors-destructor>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <member-functions>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void SensorPlacementOptimization::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) {
	world_ = _world;
	sdf_ = _sdf;

	// config
	if (sdf_->HasElement("numberOfSamplingSensors")) number_of_sampling_sensors_ = sdf_->GetElement("numberOfSamplingSensors")->Get<size_t>();
	if (sdf_->HasElement("numberOfIntendedSensors")) number_of_intended_sensors_ = sdf_->GetElement("numberOfIntendedSensors")->Get<size_t>();
	if (sdf_->HasElement("ransacNumberOfIterations")) ransac_number_of_iterations_ = sdf_->GetElement("ransacNumberOfIterations")->Get<size_t>();
	if (sdf_->HasElement("ransacSurfacePercentageStopThreshold")) ransac_surface_percentage_stop_threshold_ = sdf_->GetElement("ransacSurfacePercentageStopThreshold")->Get<double>();
	if (sdf_->HasElement("sensorsSequentialSceneRendering")) sensors_sequential_scene_rendering_ = sdf_->GetElement("sensorsSequentialSceneRendering")->Get<bool>();
	double polling_sleep_time = polling_sleep_time_.Double();
	if (sdf_->HasElement("pollingSleepTime")) polling_sleep_time = sdf_->GetElement("pollingSleepTime")->Get<double>();
	polling_sleep_time_.Set(polling_sleep_time);
	if (sdf_->HasElement("sensorOrientationRandomRoll")) sensor_orientation_random_roll_ = sdf_->GetElement("sensorOrientationRandomRoll")->Get<bool>();

	std::string sensor_data_segmentation_color_rgb_str = "0 255 0";
	if (sdf_->HasElement("sensorDataSegmentationColorRGB")) sensor_data_segmentation_color_rgb_str = sdf_->GetElement("sensorDataSegmentationColorRGB")->Get<std::string>();
	std::stringstream sensor_data_segmentation_color_rgb_ss(sensor_data_segmentation_color_rgb_str);
	if (sensor_data_segmentation_color_rgb_ss >> sensor_data_segmentation_color_r_ && sensor_data_segmentation_color_rgb_ss >> sensor_data_segmentation_color_g_ && sensor_data_segmentation_color_rgb_ss >> sensor_data_segmentation_color_b_) {
		sensor_data_segmentation_color_rgb_ = sensor_data_segmentation_color_r_ << 16 | sensor_data_segmentation_color_g_ << 8 | sensor_data_segmentation_color_b_;
	}

	if (sdf_->HasElement("voxelGridFilterLeafSize")) voxel_grid_filter_leaf_size_ = sdf_->GetElement("voxelGridFilterLeafSize")->Get<double>();
	if (sdf_->HasElement("sceneModelPath")) SetNewSceneModelPath(sdf_->GetElement("sceneModelPath")->Get<std::string>());

	std::string observation_point_str = "0 0 0";
	if (sdf_->HasElement("observationPoint")) observation_point_str = sdf_->GetElement("observationPoint")->Get<std::string>();
	std::stringstream observation_point_ss(observation_point_str);
	double ox, oy, oz;
	if (observation_point_ss >> ox && observation_point_ss >> oy && observation_point_ss >> oz) {
		observation_point_.point.x = ox;
		observation_point_.point.y = oy;
		observation_point_.point.z = oz;
	}

	sdf_sensors_name_prefix_ = "sensor_placement_optimization";
	if (sdf_->HasElement("sdfSensorsNamePrefix")) sdf_sensors_name_prefix_ = sdf_->GetElement("sdfSensorsNamePrefix")->Get<std::string>();

	topics_sampling_sensors_prefix_ = "sampling_point_cloud_";
	if (sdf_->HasElement("topicsSamplingSensorsPrefix")) topics_sampling_sensors_prefix_ = sdf_->GetElement("topicsSamplingSensorsPrefix")->Get<std::string>();

	published_msgs_world_frame_id_ = "world";
	if (sdf_->HasElement("publishedMsgsWorldFrameId")) published_msgs_world_frame_id_ = sdf_->GetElement("publishedMsgsWorldFrameId")->Get<std::string>();

	published_msgs_frame_id_suffix_ = "_frame";
	if (sdf_->HasElement("publishedMsgsFrameIdSuffix")) published_msgs_frame_id_suffix_ = sdf_->GetElement("publishedMsgsFrameIdSuffix")->Get<std::string>();

	if (sdf_->HasElement("publishMessagesOnlyWhenThereIsSubscribers")) publish_messages_only_when_there_is_subscribers_ = sdf_->GetElement("publishMessagesOnlyWhenThereIsSubscribers")->Get<bool>();

	// ros topics
	std::string topic_observation_point = "set_observation_point";
	if (sdf_->HasElement("topicObservationPoint")) topic_observation_point = sdf_->GetElement("topicObservationPoint")->Get<std::string>();

	std::string topic_model_names = "set_model_names";
	if (sdf_->HasElement("topicModelNames")) topic_model_names = sdf_->GetElement("topicModelNames")->Get<std::string>();

	// ros init
	std::string robot_namespace = "sensor_placement_optimization";
	if (sdf_->HasElement("robotNamespace")) robot_namespace = sdf_->GetElement("robotNamespace")->Get<std::string>();

	if (!ros::isInitialized()) {
		int argc = 0;
		char** argv = NULL;
		ros::init(argc, argv, robot_namespace, ros::init_options::NoSigintHandler);
	}
	rosnode_.reset(new ros::NodeHandle(robot_namespace));
	rosnode_->setCallbackQueue(&queue_);

	observation_point_subscriber_ = rosnode_->subscribe(topic_observation_point, 1, &SensorPlacementOptimization::ProcessNewObservationPoint, this);
	scene_model_path_subscriber_ = rosnode_->subscribe(topic_model_names, 1, &SensorPlacementOptimization::ProcessNewSceneModelPath, this);
	scene_model_publisher_ = rosnode_->advertise<sensor_msgs::PointCloud2>("scene_model", 1, true);
	sensors_poses_publisher_ = rosnode_->advertise<geometry_msgs::PoseArray>("sensor_poses", 1, true);
	sensors_names_publisher_ = rosnode_->advertise<std_msgs::String>("sensor_names", 1, true);
	sensors_voxel_grid_surface_coverage_publisher_ = rosnode_->advertise<std_msgs::Float32MultiArray>("sensors_surface_coverage_percentage", 1, true);
	sensors_best_poses_publisher_ = rosnode_->advertise<geometry_msgs::PoseArray>("best_sensor_poses", 1, true);
	sensors_best_voxel_grid_surface_coverages_publisher_ = rosnode_->advertise<std_msgs::Float32MultiArray>("best_sensors_surface_coverage_percentage", 1, true);
	sensors_best_names_publisher_ = rosnode_->advertise<std_msgs::String>("best_sensor_names", 1, true);
	sensors_best_merged_pointcloud_publisher_ = rosnode_->advertise<sensor_msgs::PointCloud2>("best_merged_pointcloud", 1, true);
}

void SensorPlacementOptimization::Init() {
	callback_queue_thread_ = boost::thread(boost::bind(&SensorPlacementOptimization::QueueThread, this));
	processing_thread_ = boost::thread(boost::bind(&SensorPlacementOptimization::ProcessingThread, this));
}

void SensorPlacementOptimization::QueueThread() {
	while (rosnode_->ok())
		queue_.callAvailable(ros::WallDuration(0.01));
}

void SensorPlacementOptimization::ProcessNewObservationPoint(const geometry_msgs::PointStampedConstPtr& _msg) {
	boost::mutex::scoped_lock scoped_lock(observation_point_mutex_);
	observation_point_ = *_msg;
	new_observation_point_available_ = true;
}

void SensorPlacementOptimization::ProcessNewSceneModelPath(const std_msgs::StringConstPtr& _msg) {
	if (!_msg->data.empty()) {
		scene_model_path_mutex_.lock();
		SetNewSceneModelPath(_msg->data);
		scene_model_path_mutex_.unlock();
		LoadSceneModel();
	}
}

void SensorPlacementOptimization::SetNewSceneModelPath(std::string _path_and_model) {
	if (_path_and_model.find('|') != std::string::npos) {
		std::replace(_path_and_model.begin(), _path_and_model.end(), '|', ' ');
		std::stringstream scene_model_ss(_path_and_model);
		scene_model_ss >> scene_model_path_;
		scene_model_ss >> scene_model_name_;
	} else {
		scene_model_path_ = _path_and_model;
		scene_model_name_ = "";
	}

	if (!scene_model_path_.empty()) new_scene_model_path_available_ = true;
}

void SensorPlacementOptimization::ProcessingThread() {
	LoadSensors();
	OrientSensorsToObservationPoint(sensor_orientation_random_roll_);
	PublishSensorsPoses();
	LoadSceneModel();

	while (world_->IsPaused()) {
		common::Time::Sleep(polling_sleep_time_);
	}

	HideSensors();
	ConnectDataCallBacks();

	ROS_INFO_STREAM("SensorPlacementOptimization has started with " << sensors_.size() << " sampling sensors for finding the optimal placement for " << number_of_intended_sensors_ << (number_of_intended_sensors_ == 1 ? " sensor" : " sensors"));
	bool sensor_analysis_required = true;

	while (rosnode_->ok()) {
		observation_point_mutex_.lock();
		if (new_observation_point_available_) {
			OrientSensorsToObservationPoint(sensor_orientation_random_roll_);
			PublishSensorsPoses();
			new_observation_point_available_ = false;
			sensor_analysis_required = true;
		}
		observation_point_mutex_.unlock();

		if (sensor_analysis_required) {
			ROS_INFO_STREAM("Performing sensor analysis number " << number_of_sensor_analysis_performed_);
			HideSensors();
			common::Time::Sleep(polling_sleep_time_);
			WaitForSensorData();
			ProcessSensorData();
			PrepareNextAnalysis();
			++number_of_sensor_analysis_performed_;
			sensor_analysis_required = false;
		} else {
			common::Time::Sleep(polling_sleep_time_);
		}
	}
}

void SensorPlacementOptimization::LoadSensors() {
	sensors_.clear();
	sensors_models_.clear();
	color_image_connections_.clear();
	color_pointcloud_connections_.clear();
	sampling_sensors_pointclouds_.clear();
	sampling_sensors_pointclouds_.resize(number_of_sampling_sensors_);
	sampling_sensors_images_.clear();
	sampling_sensors_images_.resize(number_of_sampling_sensors_);
	ResetNumberOfSamplingSensorsPointcloudsReceived();
	sampling_sensors_color_image_publishers_.clear();
	sampling_sensors_pointcloud_publishers_.clear();
	std::stringstream sensor_names;

	sensors::Sensor_V sensors = sensors::SensorManager::Instance()->GetSensors();
	size_t sampling_sensors_count = CountNumberOfSamplingSensors(sensors, sdf_sensors_name_prefix_);
	while (sampling_sensors_count < number_of_sampling_sensors_) {
		sensors = sensors::SensorManager::Instance()->GetSensors();
		sampling_sensors_count = CountNumberOfSamplingSensors(sensors, sdf_sensors_name_prefix_);
		common::Time::Sleep(polling_sleep_time_);
	}

	for (size_t i = 0; i < number_of_sampling_sensors_; ++i) {
		sensors::SensorPtr sensor = sensors[i];
		if (sensor && sensor->Name().size() >= sdf_sensors_name_prefix_.size() && std::equal(sdf_sensors_name_prefix_.begin(), sdf_sensors_name_prefix_.end(), sensor->Name().begin())) {
			sensors::DepthCameraSensorPtr sensor_depth = std::dynamic_pointer_cast < sensors::DepthCameraSensor > (sensor);
			std::string sensor_parent_name_with_link = sensor_depth->ParentName();
			size_t delimiter_position = sensor_parent_name_with_link.find_first_of("::");
			if (delimiter_position != std::string::npos) {
				std::string sensor_parent_name = sensor_parent_name_with_link.substr(0, delimiter_position);
				if (!sensor_parent_name.empty()) {
					physics::ModelPtr sensor_model = world_->ModelByName(sensor_parent_name);
					if (sensor_model && sensor_depth) {
						if (!sensors_.empty()) sensor_names << "|";
						sensor_names << sensor_parent_name;
						sensors_.push_back(sensor_depth);
						sensors_models_.push_back(sensor_model);
						sampling_sensors_color_image_publishers_.push_back(rosnode_->advertise<sensor_msgs::Image>(topics_sampling_sensors_prefix_ + sensor_parent_name + "_color_image", 1, true));
						sampling_sensors_pointcloud_publishers_.push_back(rosnode_->advertise<sensor_msgs::PointCloud2>(topics_sampling_sensors_prefix_ + sensor_parent_name + "_pointcloud", 1, true));
					}
				}
			}
		}
	}

	std_msgs::String sensor_names_msg;
	sensor_names_msg.data = sensor_names.str();
	sensors_names_publisher_.publish(sensor_names_msg);
}

void SensorPlacementOptimization::ConnectDataCallBacks() {
	for (size_t i = 0; i < sensors_.size(); ++i) {
		color_image_connections_.push_back(sensors_[i]->DepthCamera()->ConnectNewImageFrame(std::bind(&SensorPlacementOptimization::OnNewImageFrame,
				this, std::placeholders::_1, std::placeholders::_2,
				std::placeholders::_3, std::placeholders::_4, std::placeholders::_5, i)));
		depth_image_connections_.push_back(sensors_[i]->DepthCamera()->ConnectNewDepthFrame(std::bind(&SensorPlacementOptimization::OnNewPointCloud,
				this, std::placeholders::_1, std::placeholders::_2,
				std::placeholders::_3, std::placeholders::_4, std::placeholders::_5, i)));
		color_pointcloud_connections_.push_back(sensors_[i]->DepthCamera()->ConnectNewRGBPointCloud(std::bind(&SensorPlacementOptimization::OnNewPointCloud,
				this, std::placeholders::_1, std::placeholders::_2,
				std::placeholders::_3, std::placeholders::_4, std::placeholders::_5, i)));
	}
}

size_t SensorPlacementOptimization::CountNumberOfSamplingSensors(sensors::Sensor_V& _sensors, const std::string& _sensor_name_prefix) {
	size_t sensor_count = 0;
	for (size_t i = 0; i < _sensors.size(); ++i) {
		sensors::SensorPtr sensor = _sensors[i];
		if (sensor && sensor->Name().size() >= _sensor_name_prefix.size() && std::equal(_sensor_name_prefix.begin(), _sensor_name_prefix.end(), sensor->Name().begin())) {
			++sensor_count;
		}
	}
	return sensor_count;
}

void SensorPlacementOptimization::OrientSensorsToObservationPoint(bool _sensor_orientation_random_roll) {
	ignition::math::Vector3d observation_point(observation_point_.point.x, observation_point_.point.y, observation_point_.point.z);
	for (size_t i = 0; i < sensors_models_.size(); ++i) {
		math::Pose model_pose = sensors_models_[i]->GetWorldPose();
		ignition::math::Vector3d model_position(model_pose.pos.x, model_pose.pos.y, model_pose.pos.z);
		ignition::math::Matrix4d matrix = ignition::math::Matrix4d::LookAt(model_position, observation_point);
		ignition::math::Quaterniond new_rotation = matrix.Rotation();
		model_pose.rot.x = new_rotation.X();
		model_pose.rot.y = new_rotation.Y();
		model_pose.rot.z = new_rotation.Z();
		model_pose.rot.w = new_rotation.W();

		if (_sensor_orientation_random_roll) {
			model_pose.rot.SetFromEuler(math::Rand::GetDblUniform(0, 2.0 * math::Angle::TwoPi.Radian()), model_pose.rot.GetPitch(), model_pose.rot.GetYaw());
		}

		sensors_models_[i]->SetWorldPose(model_pose);
	}
}

void SensorPlacementOptimization::HideSensors() {
	for (size_t i = 0; i < sensors_models_.size(); ++i) {
		sensors_models_[i]->SetScale(ignition::math::Vector3d(0.00001,0.00001,0.00001), true);
	}
}

void SensorPlacementOptimization::ShowSensor(size_t _sensor_index) {
	if (_sensor_index < sensors_models_.size()) {
		sensors_models_[_sensor_index]->SetScale(ignition::math::Vector3d(1,1,1), true);
	}
}

void SensorPlacementOptimization::PublishSensorsPoses() {
	geometry_msgs::PoseArray sensor_poses;
	sensor_poses.header.frame_id = published_msgs_world_frame_id_;
	sensor_poses.header.stamp.fromSec(world_->SimTime().Double());
	for (size_t i = 0; i < sensors_models_.size(); ++i) {
		sensor_poses.poses.push_back(MathPoseToRosPose(sensors_models_[i]->GetWorldPose()));
	}
	sensors_poses_publisher_.publish(sensor_poses);
}

void SensorPlacementOptimization::LoadSceneModel() {
	scene_model_path_mutex_.lock();
	if (new_scene_model_path_available_ && !scene_model_path_.empty()) {
		scene_model_ = typename pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
		if (LoadPointCloudfromFile(scene_model_path_, *scene_model_)) {
			if (!scene_model_name_.empty()) {
				std::vector<std::string> model_names;
				if (scene_model_name_.find('+') != std::string::npos) {
					std::replace(scene_model_name_.begin(), scene_model_name_.end(), '+', ' ');
					std::stringstream scene_model_ss(scene_model_name_);
					std::string model_name;
					while (scene_model_ss >> model_name) {
						model_names.push_back(model_name);
					}

					typename pcl::PointCloud<pcl::PointXYZRGB>::Ptr merged_pointcloud(new pcl::PointCloud<pcl::PointXYZRGB>());
					for (size_t i = 0; i < model_names.size(); ++i) {
						Eigen::Affine3f transform_to_world;
						if (!model_names[i].empty() && GetModelTransformToWorld(model_names[i], transform_to_world)) {
							pcl::PointCloud<pcl::PointXYZRGB> transformed_pointcloud;
							pcl::transformPointCloud(*scene_model_, transformed_pointcloud, transform_to_world);
							*merged_pointcloud += transformed_pointcloud;
						}
					}
					scene_model_ = merged_pointcloud;
				} else {
					Eigen::Affine3f transform_to_world;
					if (GetModelTransformToWorld(scene_model_name_, transform_to_world)) {
						pcl::transformPointCloud(*scene_model_, *scene_model_, transform_to_world);
					}
				}
			}

			if (FilterPointCloud(scene_model_)) {
				scene_model_->header.stamp = (pcl::uint64_t)(world_->SimTime().Double() * 1e6);
				scene_model_->header.frame_id = published_msgs_world_frame_id_;
				sensor_msgs::PointCloud2Ptr cloud_msg(new sensor_msgs::PointCloud2());
				pcl::toROSMsg(*scene_model_, *cloud_msg);
				scene_model_publisher_.publish(cloud_msg);
			} else {
				scene_model_ = typename pcl::PointCloud<pcl::PointXYZRGB>::Ptr();
			}
		} else {
			scene_model_ = typename pcl::PointCloud<pcl::PointXYZRGB>::Ptr();
		}
		new_scene_model_path_available_ = false;
	}
	scene_model_path_mutex_.unlock();
}

void SensorPlacementOptimization::SetSensorsState(bool _active) {
	for (size_t i = 0; i < sensors_.size(); ++i) {
		sensors_[i]->SetActive(_active);
	}
}

void SensorPlacementOptimization::OnNewImageFrame(const unsigned char* _image, unsigned int _width, unsigned int _height, unsigned int _depth, const std::string& _format, size_t _sensor_index) {
	if (_image == NULL) {
		ROS_WARN_STREAM("Received NULL color image from sensor " << _sensor_index << " with [width: " << _width << " | height: " << _height << " | depth: " << _depth << " | format: " << _format << "]");
		return;
	}

	if (_sensor_index < sampling_sensors_color_image_publishers_.size() && !sampling_sensors_images_[_sensor_index] && !sampling_sensors_pointclouds_[_sensor_index]) {
		sensor_msgs::Image::Ptr image_msg(new sensor_msgs::Image());
		image_msg->header.stamp.fromSec(world_->SimTime().Double());
		image_msg->header.frame_id = sensors_[_sensor_index]->ParentName() + published_msgs_frame_id_suffix_;
		std::replace(image_msg->header.frame_id.begin(), image_msg->header.frame_id.end(), ':', '_');
		image_msg->width = _width;
		image_msg->height = _height;
		image_msg->step = _width * 3;
		image_msg->encoding = sensor_msgs::image_encodings::BGR8;
		image_msg->is_bigendian = false;
		size_t msg_number_bytes = _width * _height * 3;
		image_msg->data.resize(msg_number_bytes);
		memcpy(&image_msg->data[0], _image,  msg_number_bytes);
		sampling_sensors_images_[_sensor_index] = image_msg;
		ROS_DEBUG_STREAM("Received color image from sensor " << _sensor_index << " with [width: " << _width << " | height: " << _height << " | depth: " << _depth << " | format: " << _format << "]");
		if (!publish_messages_only_when_there_is_subscribers_ || sampling_sensors_color_image_publishers_[_sensor_index].getNumSubscribers() > 0)
			sampling_sensors_color_image_publishers_[_sensor_index].publish(*image_msg);
	}
}

void SensorPlacementOptimization::OnNewPointCloud(const float *_pcd,
				unsigned int _width, unsigned int _height,
				unsigned int _depth, const std::string &_format, size_t _sensor_index) {
	if (_pcd == NULL) {
		ROS_WARN_STREAM("Received NULL point cloud from sensor " << _sensor_index << " with [width: " << _width << " | height: " << _height << " | depth: " << _depth << " | format: " << _format << "]");
		return;
	}

	if (_sensor_index < sensors_.size() && _sensor_index < sampling_sensors_pointclouds_.size() && !sampling_sensors_pointclouds_[_sensor_index]) {
		if (!sampling_sensors_images_[_sensor_index]) return;
		Eigen::Affine3f transform_sensor_to_world;
		if (GetSensorTransformToWorld(_sensor_index, transform_sensor_to_world)) {
			typename pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud = SegmentSensorDataFromDepthSensor(_pcd, _format, _width, _height, transform_sensor_to_world, _sensor_index);
			FilterPointCloud(pointcloud);

			pointcloud->header.stamp = (pcl::uint64_t)(world_->SimTime().Double() * 1e6);
			pointcloud->header.frame_id = published_msgs_world_frame_id_;
			PublishPointCloud(pointcloud, _sensor_index);

			if (!sampling_sensors_pointclouds_[_sensor_index]) {
				IncrementNumberOfSamplingSensorsPointcloudsReceived();
			}

			sampling_sensors_pointclouds_[_sensor_index] = pointcloud;
			sampling_sensors_images_[_sensor_index].reset();

			if (sensors_sequential_scene_rendering_) {
				sensors_[_sensor_index]->SetActive(false);
				if (_sensor_index + 1 < sensors_.size()) sensors_[_sensor_index + 1]->SetActive(true);
			}

			ROS_DEBUG_STREAM("Received point cloud from sensor " << _sensor_index << " with [width: " << _width << " | height: " << _height << " | depth: " << _depth << " | format: " << _format << " | segmented & filtered points: " << pointcloud->size() << "]");
		}
	}
}

typename pcl::PointCloud<pcl::PointXYZRGB>::Ptr SensorPlacementOptimization::SegmentSensorDataFromDepthSensor(const float* _data, const std::string &_format, unsigned int _width, unsigned int _height, Eigen::Affine3f &_transform_sensor_to_world, size_t _sensor_index) {
	typename pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZRGB>());
	sensor_msgs::Image::Ptr color_image = sampling_sensors_images_[_sensor_index];
	if (color_image) {
		bool depth_image = _format == "FLOAT32";
		size_t memory_index = 0;
		size_t color_point_index_ = 0;
		size_t current_point_index_ = 0;
		float fx_inverse, fy_inverse, cx, cy;
		if (depth_image) {
			GetSensorIntrinsics(sensors_[_sensor_index], fx_inverse, fy_inverse, cx, cy);
			Eigen::Quaternion<float> rotation_optical_to_camera_frame(0.5, -0.5, 0.5, -0.5);
			_transform_sensor_to_world = _transform_sensor_to_world * rotation_optical_to_camera_frame;
		} else {
			Eigen::Quaternion<float> rotation_optical_to_camera_frame(0.5, 0.5, -0.5, -0.5);
			_transform_sensor_to_world = _transform_sensor_to_world * rotation_optical_to_camera_frame;
		}

		for (size_t row = 0; row < _height; ++row) {
			for (size_t column = 0; column < _width; ++column) {
				uint8_t c_blue = color_image->data[color_point_index_++];
				uint8_t c_green = color_image->data[color_point_index_++];
				uint8_t c_red = color_image->data[color_point_index_++];
				if (c_blue == sensor_data_segmentation_color_b_ && c_green == sensor_data_segmentation_color_g_ && c_red == sensor_data_segmentation_color_r_) {
					pcl::PointXYZRGB new_point(c_red, c_green, c_blue);
					if (depth_image) {
						if (IsPointWithinValidRange(sensors_[_sensor_index], std::abs(_data[current_point_index_]))) {
							new_point.z = _data[current_point_index_];
							new_point.x = (static_cast<float>(column) - cx) * new_point.z * fx_inverse;
							new_point.y = (static_cast<float>(row) - cy) * new_point.z * fy_inverse;
							pointcloud->push_back(pcl::transformPoint(new_point, _transform_sensor_to_world));
						}
					} else {
						if (IsPointWithinValidRange(sensors_[_sensor_index], std::abs(_data[memory_index + 2]))) {
							memcpy(&new_point.data[0], &_data[memory_index], 3 * sizeof(float));
							pointcloud->push_back(pcl::transformPoint(new_point, _transform_sensor_to_world));
						}
					}
				}
				++current_point_index_;
				memory_index += 4;
			}
		}
	}
	return pointcloud;
}

bool SensorPlacementOptimization::GetSensorTransformToWorld(size_t _sensor_index, Eigen::Affine3f &_transform_sensor_to_world) {
	if (_sensor_index < sensors_.size()) {
		ignition::math::Pose3d pose = sensors_[_sensor_index]->DepthCamera()->WorldPose();
		_transform_sensor_to_world = PoseToTransform<float>(pose);
		return true;
	}
	return false;
}

bool SensorPlacementOptimization::GetModelTransformToWorld(std::string _model_name, Eigen::Affine3f &_transform) {
	physics::ModelPtr model = world_->ModelByName(_model_name);
	if (model) {
		math::Pose pose = model->GetWorldPose();
		_transform = PoseToTransform<float>(pose);
		return true;
	}
	return false;
}

bool SensorPlacementOptimization::IsPointWithinValidRange(const sensors::DepthCameraSensorPtr& _depth_camera, float _point) {
	if (_point >= _depth_camera->DepthCamera()->NearClip() && _point <= _depth_camera->DepthCamera()->FarClip()) return true;
	return false;
}

void SensorPlacementOptimization::GetSensorIntrinsics(const sensors::DepthCameraSensorPtr& _depth_camera, float &_fx_inverse, float &_fy_inverse, float &_cx, float &_cy) {
	_fx_inverse = 1.0 / (_depth_camera->DepthCamera()->ImageWidth() / (2 * std::tan(_depth_camera->DepthCamera()->HFOV().Radian() / 2.0)));
	_fy_inverse = 1.0 / (_depth_camera->DepthCamera()->ImageHeight() / (2 * std::tan(_depth_camera->DepthCamera()->VFOV().Radian() / 2.0)));
	_cx = _depth_camera->DepthCamera()->ImageWidth() / 2.0;
	_cy = _depth_camera->DepthCamera()->ImageHeight() / 2.0;

}

bool SensorPlacementOptimization::FilterPointCloud(typename pcl::PointCloud<pcl::PointXYZRGB>::Ptr &_pointcloud) {
	typename pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::VoxelGrid<pcl::PointXYZRGB> filter;
	filter.setLeafSize(voxel_grid_filter_leaf_size_, voxel_grid_filter_leaf_size_, voxel_grid_filter_leaf_size_);
	filter.setInputCloud(_pointcloud);
	filter.filter(*pointcloud_filtered);
	_pointcloud = pointcloud_filtered;
	return _pointcloud->size() > 0;
}

bool SensorPlacementOptimization::PublishPointCloud(typename pcl::PointCloud<pcl::PointXYZRGB>::Ptr &_pointcloud, size_t _pubisher_index) {
	if (_pointcloud && _pubisher_index < sampling_sensors_pointcloud_publishers_.size() &&
			(!publish_messages_only_when_there_is_subscribers_ || sampling_sensors_pointcloud_publishers_[_pubisher_index].getNumSubscribers() > 0)) {
		sensor_msgs::PointCloud2Ptr cloud_msg(new sensor_msgs::PointCloud2());
		pcl::toROSMsg(*_pointcloud, *cloud_msg);
		sampling_sensors_pointcloud_publishers_[_pubisher_index].publish(cloud_msg);
		return true;
	}
	return false;
}

void SensorPlacementOptimization::WaitForSensorData() {
	if (sensors_sequential_scene_rendering_) {
		SetSensorsState(false);
		if (!sensors_.empty()) sensors_[0]->SetActive(true);
	} else{
		SetSensorsState(true);
	}

	while (GetNumberOfSamplingSensorsPointcloudsReceived() < sampling_sensors_pointclouds_.size()) {
		common::Time::Sleep(polling_sleep_time_);
	}
	SetSensorsState(false);
}

void SensorPlacementOptimization::ProcessSensorData() {
	while (!scene_model_) {
		ROS_WARN("Waiting for scene model");
		common::Time::Sleep(polling_sleep_time_);
	}

	geometry_msgs::PoseArray best_sensors_poses;
	best_sensors_poses.header.frame_id = published_msgs_world_frame_id_;
	best_sensors_poses.header.stamp.fromSec(world_->SimTime().Double());
	std_msgs::Float32MultiArray sensors_best_voxel_grid_surface_coverages_msg;
	std::stringstream best_sensor_names;

	size_t best_sensor_coverage_index = PublishAnalysisSurfaceCoverageAnalysis();
	if (number_of_intended_sensors_ == 1) {
		double best_coverage = ((double)sampling_sensors_pointclouds_[best_sensor_coverage_index]->size() / (double)scene_model_->size()) * 100.0;
		best_sensors_poses.poses.push_back(MathPoseToRosPose(sensors_models_[best_sensor_coverage_index]->GetWorldPose()));
		sensors_best_voxel_grid_surface_coverages_msg.data.push_back(best_coverage);
		best_sensor_names << sensors_models_[best_sensor_coverage_index]->GetName();
		ShowSensor(best_sensor_coverage_index);
		sampling_sensors_pointclouds_[best_sensor_coverage_index]->header.stamp = (pcl::uint64_t)(world_->SimTime().Double() * 1e6);
		sampling_sensors_pointclouds_[best_sensor_coverage_index]->header.frame_id = published_msgs_world_frame_id_;
		sensor_msgs::PointCloud2Ptr pointcloud_msg(new sensor_msgs::PointCloud2());
		pcl::toROSMsg(*sampling_sensors_pointclouds_[best_sensor_coverage_index], *pointcloud_msg);
		sensors_best_merged_pointcloud_publisher_.publish(pointcloud_msg);
		ROS_INFO_STREAM("Finished analyzing the sensor data with the best view achieving " << best_coverage << " surface coverage percentage");
	} else {
		size_t current_ransac_iteration = 0;
		typename pcl::PointCloud<pcl::PointXYZRGB>::Ptr best_merged_pointcloud;
		std::vector<int> best_merged_pointclouds_indexes;
		double best_merged_point_cloud_surface_coverage_percentage = 0.0;
		while (current_ransac_iteration < ransac_number_of_iterations_ && best_merged_point_cloud_surface_coverage_percentage < ransac_surface_percentage_stop_threshold_) {
			std::vector<int> random_numbers;
			FillUniqueRandomIntNumbers(number_of_intended_sensors_, 0, sampling_sensors_pointclouds_.size(), random_numbers);
			typename pcl::PointCloud<pcl::PointXYZRGB>::Ptr merged_pointcloud(new pcl::PointCloud<pcl::PointXYZRGB>());
			for (size_t i = 0; i < random_numbers.size(); ++i) {
				*merged_pointcloud += *sampling_sensors_pointclouds_[random_numbers[i]];
			}
			FilterPointCloud(merged_pointcloud);
			double merged_clouds_coverage = ((double)merged_pointcloud->size() / (double)scene_model_->size()) * 100.0;
			if (merged_clouds_coverage > best_merged_point_cloud_surface_coverage_percentage) {
				best_merged_point_cloud_surface_coverage_percentage = merged_clouds_coverage;
				best_merged_pointcloud = merged_pointcloud;
				best_merged_pointclouds_indexes = random_numbers;
			}
			++current_ransac_iteration;
		}
		ROS_INFO_STREAM("Finished RANSAC with " << best_merged_point_cloud_surface_coverage_percentage << " surface coverage percentage after " << current_ransac_iteration << " iterations");
		for (size_t i = 0; i < best_merged_pointclouds_indexes.size(); ++i) {
			best_sensors_poses.poses.push_back(MathPoseToRosPose(sensors_models_[best_merged_pointclouds_indexes[i]]->GetWorldPose()));
			if (i > 0) best_sensor_names << "|";
			best_sensor_names << sensors_models_[best_merged_pointclouds_indexes[i]]->GetName();
			ShowSensor(best_merged_pointclouds_indexes[i]);
		}
		sensors_best_voxel_grid_surface_coverages_msg.data.push_back(best_merged_point_cloud_surface_coverage_percentage);
		best_merged_pointcloud->header.stamp = (pcl::uint64_t)(world_->SimTime().Double() * 1e6);
		best_merged_pointcloud->header.frame_id = published_msgs_world_frame_id_;
		sensor_msgs::PointCloud2Ptr best_merged_pointcloud_msg(new sensor_msgs::PointCloud2());
		pcl::toROSMsg(*best_merged_pointcloud, *best_merged_pointcloud_msg);
		sensors_best_merged_pointcloud_publisher_.publish(best_merged_pointcloud_msg);
	}

	sensors_best_poses_publisher_.publish(best_sensors_poses);
	sensors_best_voxel_grid_surface_coverages_publisher_.publish(sensors_best_voxel_grid_surface_coverages_msg);
	std_msgs::String best_sensor_names_msg;
	best_sensor_names_msg.data = best_sensor_names.str();
	sensors_best_names_publisher_.publish(best_sensor_names_msg);
}

void SensorPlacementOptimization::FillUniqueRandomIntNumbers(size_t _number_of_random_numbers, int _min, int _max, std::vector<int>& _random_numbers) {
	_random_numbers.clear();
	std::set<int> number_set;
	while (_random_numbers.size() < _number_of_random_numbers) {
		int number = math::Rand::GetIntUniform(_min, _max);
		if (number >= _min && number < _max && number_set.find(number) == number_set.end()) {
			number_set.insert(number);
			_random_numbers.push_back(number);
		}
	}
}

size_t SensorPlacementOptimization::PublishAnalysisSurfaceCoverageAnalysis() {
	size_t best_coverage_sensor_index = 0;
	double best_coverage_percentage = 0;
	std_msgs::Float32MultiArray sensors_coverage;

	for (size_t i = 0; i < sampling_sensors_pointclouds_.size(); ++i) {
		double sensor_coverage = ((double)sampling_sensors_pointclouds_[i]->size() / (double)scene_model_->size()) * 100.0;
		sensors_coverage.data.push_back(sensor_coverage);
		if (sensor_coverage > best_coverage_percentage) {
			best_coverage_percentage = sensor_coverage;
			best_coverage_sensor_index = i;
		}
	}
	sensors_voxel_grid_surface_coverage_publisher_.publish(sensors_coverage);
	return best_coverage_sensor_index;
}

void SensorPlacementOptimization::PrepareNextAnalysis() {
	ResetNumberOfSamplingSensorsPointcloudsReceived();
	sampling_sensors_pointclouds_.clear();
	sampling_sensors_pointclouds_.resize(number_of_sampling_sensors_);
	sampling_sensors_images_.clear();
	sampling_sensors_images_.resize(number_of_sampling_sensors_);
}

geometry_msgs::Pose SensorPlacementOptimization::MathPoseToRosPose(math::Pose _math_pose) {
	geometry_msgs::Pose ros_pose;
	ros_pose.position.x = _math_pose.pos.x;
	ros_pose.position.y = _math_pose.pos.y;
	ros_pose.position.z = _math_pose.pos.z;
	ros_pose.orientation.x = _math_pose.rot.x;
	ros_pose.orientation.y = _math_pose.rot.y;
	ros_pose.orientation.z = _math_pose.rot.z;
	ros_pose.orientation.w = _math_pose.rot.w;
	return ros_pose;
}

void SensorPlacementOptimization::ResetNumberOfSamplingSensorsPointcloudsReceived() {
	number_of_sampling_sensors_pointclouds_received_mutex_.lock();
	number_of_sampling_sensors_pointclouds_received_ = 0;
	number_of_sampling_sensors_pointclouds_received_mutex_.unlock();
}

void SensorPlacementOptimization::SetNumberOfSamplingSensorsPointcloudsReceived(size_t value) {
	number_of_sampling_sensors_pointclouds_received_mutex_.lock();
	number_of_sampling_sensors_pointclouds_received_ = value;
	number_of_sampling_sensors_pointclouds_received_mutex_.unlock();
}

void SensorPlacementOptimization::IncrementNumberOfSamplingSensorsPointcloudsReceived() {
	number_of_sampling_sensors_pointclouds_received_mutex_.lock();
	++number_of_sampling_sensors_pointclouds_received_;
	number_of_sampling_sensors_pointclouds_received_mutex_.unlock();
}

size_t SensorPlacementOptimization::GetNumberOfSamplingSensorsPointcloudsReceived() {
	size_t number;
	number_of_sampling_sensors_pointclouds_received_mutex_.lock();
	number = number_of_sampling_sensors_pointclouds_received_;
	number_of_sampling_sensors_pointclouds_received_mutex_.unlock();
	return number;
}
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </member-functions>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// =============================================================================  </public-section>  ===========================================================================

} /* namespace gazebo */
