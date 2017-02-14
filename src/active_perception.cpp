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
		new_observation_pose_available_(false),
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

	sampling_sensors_name_prefix_ = "active_perception";
	if (sdf_->HasElement("samplingSensorsNamePrefix")) sampling_sensors_name_prefix_ = sdf_->GetElement("samplingSensorsNamePrefix")->Get<std::string>();

	// ros topics
	std::string topic_observation_pose = "set_observation_pose";
	if (sdf_->HasElement("topicObservationPose")) topic_observation_pose = sdf_->GetElement("topicObservationPose")->Get<std::string>();

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

	observation_pose_subscriber_ = rosnode_->subscribe(topic_observation_pose, 1, &ActivePerception::ProcessNewObservationPose, this);
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

void ActivePerception::ProcessNewObservationPose(const geometry_msgs::PoseStampedConstPtr& _msg) {
	boost::mutex::scoped_lock scoped_lock(observation_pose_mutex_);
	observation_pose_ = *_msg;
	new_observation_pose_available_ = true;
}

void ActivePerception::ProcessNewModelNames(const std_msgs::StringConstPtr& _msg) {
	boost::mutex::scoped_lock scoped_lock(observation_models_names_mutex_);
	observation_models_names_ = _msg->data;
	new_observation_models_names_available_ = true;
}

void ActivePerception::ProcessingThread() {
	LoadSensors();
	ROS_INFO_STREAM("ActivePerception has started  with " << sensors_.size() << "sampling sensors and for finding the optimal placement for " << number_of_intended_sensors_ << " sensors");
}

void ActivePerception::LoadSensors(common::Time _wait_time) {
	sensors::Sensor_V sensors = sensors::SensorManager::Instance()->GetSensors();
	size_t sampling_sensors_count = CountNumberOfSamplingSensors(sensors, sampling_sensors_name_prefix_);
	while (sampling_sensors_count < number_of_sampling_sensors_) {
		sensors = sensors::SensorManager::Instance()->GetSensors();
		sampling_sensors_count = CountNumberOfSamplingSensors(sensors, sampling_sensors_name_prefix_);
		common::Time::Sleep(_wait_time);
	}

	for (size_t i = 0; i < sensors.size(); ++i) {
		sensors::SensorPtr sensor = sensors[i];
		if (sensor && sensor->Name().size() >= sampling_sensors_name_prefix_.size() && std::equal(sampling_sensors_name_prefix_.begin(), sampling_sensors_name_prefix_.end(), sensor->Name().begin())) {
			sensors::DepthCameraSensorPtr sensor_depth = std::dynamic_pointer_cast < sensors::DepthCameraSensor > (sensor);
			if (sensor_depth) {
				sensors_.push_back(sensor_depth);
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

/*void ActivePerception::OnNewRGBPointCloud(const float *_pcd, unsigned int _width, unsigned int _height, unsigned int _depth, const std::string &_format) {
	ROS_INFO_STREAM("Received PCD with [width: " << _width << " | height: " << _height << " | depth: " << _depth << " | format: " << _format << "]");
}*/
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </member-functions>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// =============================================================================  </public-section>  ===========================================================================

// =============================================================================   <protected-section>   =======================================================================
// =============================================================================   </protected-section>  =======================================================================

} /* namespace gazebo */

