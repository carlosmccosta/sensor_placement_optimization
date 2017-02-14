/**\file active_perception.cpp
 * \brief Description...
 *
 * @version 1.0
 * @author carloscosta
 */

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#include <active_perception/active_perception.h>
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

namespace gazebo {

// =============================================================================  <public-section>  ============================================================================
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <constructors-destructor>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
ActivePerception::ActivePerception() :
		number_of_sampling_sensors_(100),
		number_of_intended_sensors_(1),
		sensor_orientaion_random_roll_(true),
		new_observation_point_available_(false),
		new_observation_models_names_available_(false) {
}

ActivePerception::~ActivePerception() {
}
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </constructors-destructor>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <member-functions>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void ActivePerception::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) {
	world_ = _world;
	sdf_ = _sdf;

	// config
	if (sdf_->HasElement("numberOfSamplingSensors")) number_of_sampling_sensors_ = sdf_->GetElement("numberOfSamplingSensors")->Get<size_t>();
	if (sdf_->HasElement("numberOfIntendedSensors")) number_of_intended_sensors_ = sdf_->GetElement("numberOfIntendedSensors")->Get<size_t>();
	if (sdf_->HasElement("sensorOrientaionRandomRoll")) sensor_orientaion_random_roll_ = sdf_->GetElement("sensorOrientaionRandomRoll")->Get<bool>();

	sampling_sensors_name_prefix_ = "active_perception";
	if (sdf_->HasElement("samplingSensorsNamePrefix")) sampling_sensors_name_prefix_ = sdf_->GetElement("samplingSensorsNamePrefix")->Get<std::string>();

	// ros topics
	std::string topic_observation_point = "set_observation_point";
	if (sdf_->HasElement("topicObservationPoint")) topic_observation_point = sdf_->GetElement("topicObservationPoint")->Get<std::string>();

	std::string topic_model_names = "set_model_names";
	if (sdf_->HasElement("topicModelNames")) topic_model_names = sdf_->GetElement("topicModelNames")->Get<std::string>();

	// ros init
	std::string robot_namespace = "active_perception";
	if (sdf_->HasElement("robotNamespace")) robot_namespace = sdf_->GetElement("robotNamespace")->Get<std::string>();

	if (!ros::isInitialized()) {
		int argc = 0;
		char** argv = NULL;
		ros::init(argc, argv, robot_namespace, ros::init_options::NoSigintHandler);
	}
	rosnode_.reset(new ros::NodeHandle(robot_namespace));
	rosnode_->setCallbackQueue(&queue_);

	observation_point_subscriber_ = rosnode_->subscribe(topic_observation_point, 1, &ActivePerception::ProcessNewObservationPoint, this);
	observation_models_names_subscriber_ = rosnode_->subscribe(topic_model_names, 1, &ActivePerception::ProcessNewModelNames, this);
}

void ActivePerception::Init() {
	callback_queue_thread_ = boost::thread(boost::bind(&ActivePerception::QueueThread, this));
	processing_thread_ = boost::thread(boost::bind(&ActivePerception::ProcessingThread, this));
}

void ActivePerception::QueueThread() {
	while (rosnode_->ok())
		queue_.callAvailable(ros::WallDuration(0.01));
}

void ActivePerception::ProcessNewObservationPoint(const geometry_msgs::PointStampedConstPtr& _msg) {
	boost::mutex::scoped_lock scoped_lock(observation_point_mutex_);
	observation_point_ = *_msg;
	new_observation_point_available_ = true;
}

void ActivePerception::ProcessNewModelNames(const std_msgs::StringConstPtr& _msg) {
	boost::mutex::scoped_lock scoped_lock(observation_models_names_mutex_);
	observation_models_names_ = _msg->data;
	new_observation_models_names_available_ = true;
}

void ActivePerception::ProcessingThread() {
	LoadSensors();
	OrientSensorsToObservationPoint();
	ROS_INFO_STREAM("ActivePerception has started with " << sensors_.size() << " sampling sensors for finding the optimal placement for " << number_of_intended_sensors_ << (number_of_intended_sensors_ == 1 ? " sensor" : " sensors"));
	while (rosnode_->ok()) {
		if (new_observation_point_available_) {
			OrientSensorsToObservationPoint();
			observation_models_names_mutex_.lock();
			new_observation_point_available_ = false;
			observation_models_names_mutex_.unlock();
		}
	}
}

void ActivePerception::LoadSensors(common::Time _wait_time) {
	sensors_.clear();
	sensors_models_.clear();
	sensors::Sensor_V sensors = sensors::SensorManager::Instance()->GetSensors();
	size_t sampling_sensors_count = CountNumberOfSamplingSensors(sensors, sampling_sensors_name_prefix_);
	while (sampling_sensors_count < number_of_sampling_sensors_) {
		sensors = sensors::SensorManager::Instance()->GetSensors();
		sampling_sensors_count = CountNumberOfSamplingSensors(sensors, sampling_sensors_name_prefix_);
		common::Time::Sleep(_wait_time);
	}

	for (size_t i = 0; i < number_of_sampling_sensors_; ++i) {
		sensors::SensorPtr sensor = sensors[i];
		if (sensor && sensor->Name().size() >= sampling_sensors_name_prefix_.size() && std::equal(sampling_sensors_name_prefix_.begin(), sampling_sensors_name_prefix_.end(), sensor->Name().begin())) {
			sensors::DepthCameraSensorPtr sensor_depth = std::dynamic_pointer_cast < sensors::DepthCameraSensor > (sensor);
			std::string sensor_parent_name_with_link = sensor_depth->ParentName();
			size_t delimiter_position = sensor_parent_name_with_link.find_first_of("::");
			if (delimiter_position != std::string::npos) {
				std::string sensor_parent_name = sensor_parent_name_with_link.substr(0, delimiter_position);
				if (!sensor_parent_name.empty()) {
					physics::ModelPtr sensor_model = world_->ModelByName(sensor_parent_name);
					if (sensor_model && sensor_depth) {
						sensors_.push_back(sensor_depth);
						sensors_models_.push_back(sensor_model);
					}
				}
			}
		}
	}
}

size_t ActivePerception::CountNumberOfSamplingSensors(sensors::Sensor_V& _sensors, const std::string& _sensor_name_prefix) {
	size_t sensor_count = 0;
	for (size_t i = 0; i < _sensors.size(); ++i) {
		sensors::SensorPtr sensor = _sensors[i];
		if (sensor && sensor->Name().size() >= _sensor_name_prefix.size() && std::equal(_sensor_name_prefix.begin(), _sensor_name_prefix.end(), sensor->Name().begin())) {
			++sensor_count;
		}
	}
	return sensor_count;
}

void ActivePerception::OrientSensorsToObservationPoint() {
	ignition::math::Vector3d observation_point(observation_point_.point.x, observation_point_.point.y, observation_point_.point.z);
	for (size_t i = 0; i < sensors_models_.size(); ++i) {
		math::Pose model_pose = sensors_models_[i]->GetWorldPose();
		ignition::math::Matrix4d matrix = ignition::math::Matrix4d::LookAt(ignition::math::Vector3d(model_pose.pos.x, model_pose.pos.y, model_pose.pos.z), observation_point);
		ignition::math::Quaterniond new_rotation = matrix.Rotation();
		model_pose.rot.x = new_rotation.X();
		model_pose.rot.y = new_rotation.Y();
		model_pose.rot.z = new_rotation.Z();
		model_pose.rot.w = new_rotation.W();

		if (sensor_orientaion_random_roll_) {
			model_pose.rot.SetFromEuler(math::Rand::GetDblUniform(0, 2.0 * math::Angle::TwoPi.Radian()), model_pose.rot.GetPitch(), model_pose.rot.GetYaw());
		}

		sensors_models_[i]->SetWorldPose(model_pose);
	}
}

/*void ActivePerception::OnNewRGBPointCloud(const float *_pcd, unsigned int _width, unsigned int _height, unsigned int _depth, const std::string &_format) {
	ROS_INFO_STREAM("Received PCD with [width: " << _width << " | height: " << _height << " | depth: " << _depth << " | format: " << _format << "]");
}*/
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </member-functions>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// =============================================================================  </public-section>  ===========================================================================

// =============================================================================   <protected-section>   =======================================================================
// =============================================================================   </protected-section>  =======================================================================

} /* namespace gazebo */


