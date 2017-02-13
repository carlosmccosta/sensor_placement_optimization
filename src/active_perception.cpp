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
		new_observation_pose_available_(false), new_observation_models_names_available_(false) {
}

ActivePerception::~ActivePerception() {
}
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </constructors-destructor>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <member-functions>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void ActivePerception::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) {
	world_ = _world;

	std::string robot_namespace = "active_perception";
	if (_sdf->HasElement("robotNamespace")) robot_namespace = _sdf->GetElement("robotNamespace")->Get<std::string>();

	if (!ros::isInitialized()) {
		int argc = 0;
		char** argv = NULL;
		ros::init(argc, argv, robot_namespace, ros::init_options::NoSigintHandler);
	}
	rosnode_.reset(new ros::NodeHandle(robot_namespace));
	rosnode_->setCallbackQueue(&queue_);

	std::string topic_observation_pose = "set_observation_pose";
	if (_sdf->HasElement("topicObservationPose")) topic_observation_pose = _sdf->GetElement("topicObservationPose")->Get<std::string>();

	std::string topic_model_names = "set_model_names";
	if (_sdf->HasElement("topicModelNames")) topic_model_names = _sdf->GetElement("topicModelNames")->Get<std::string>();

	observation_pose_subscriber_ = rosnode_->subscribe(topic_observation_pose, 1, &ActivePerception::ProcessNewObservationPose, this);
	observation_models_names_subscriber_ = rosnode_->subscribe(topic_model_names, 1, &ActivePerception::ProcessNewModelNames, this);

	std::string topic_sensor_poses_base_name = "optimal_sensor_placement_";
	if (_sdf->HasElement("topicSensorPosesBaseName")) topic_sensor_poses_base_name = _sdf->GetElement("topicSensorPosesBaseName")->Get<std::string>();

	std::string sensor_configuration = "kinect1:1";
	if (_sdf->HasElement("sensorConfiguration")) sensor_configuration = _sdf->GetElement("sensorConfiguration")->Get<std::string>();

	if (!sensor_configuration.empty()) {
		std::replace(sensor_configuration.begin(), sensor_configuration.end(), '+', ' ');
		std::stringstream ss_sensor_configuration(sensor_configuration);
		std::string sensor_model_name;
		while (ss_sensor_configuration >> sensor_model_name && !sensor_model_name.empty()) {
			physics::ModelPtr model = world_->ModelByName(sensor_model_name);
			if (model) {
				sensors::DepthCameraSensorPtr sensor = std::dynamic_pointer_cast < sensors::DepthCameraSensor > (sensors::SensorManager::Instance()->GetSensor(sensor_model_name + "_sensor"));
				if (sensor) {
					sensors_models_.push_back(model);
					sensors_.push_back(sensor);
					color_pointcloud_connections_.push_back(sensor->DepthCamera()->ConnectNewRGBPointCloud(std::bind(&ActivePerception::OnNewRGBPointCloud,
							this, std::placeholders::_1, std::placeholders::_2,
							std::placeholders::_3, std::placeholders::_4, std::placeholders::_5)));
					publishers_sensor_poses_.push_back(rosnode_->advertise<geometry_msgs::PoseArray>(topic_sensor_poses_base_name + sensor_model_name, 1, true));
				}
			}
		}
	}

	callback_queue_thread_ = boost::thread(boost::bind(&ActivePerception::QueueThread, this));
	//processing_thread_ = boost::thread(boost::bind(&ActivePerception::ProcessingThread, this));

	ROS_INFO("ActivePerception has started (namespace=%s, sensor_configuraton=%s)\n", robot_namespace.c_str(), sensor_configuration.c_str());
}

void ActivePerception::ProcessNewObservationPose(const geometry_msgs::PoseStampedConstPtr& msg) {
	boost::mutex::scoped_lock scoped_lock(observation_pose_mutex_);
	observation_pose_ = *msg;
	new_observation_pose_available_ = true;
}

void ActivePerception::ProcessNewModelNames(const std_msgs::StringConstPtr& msg) {
	boost::mutex::scoped_lock scoped_lock(observation_models_names_mutex_);
	observation_models_names_ = msg->data;
	new_observation_models_names_available_ = true;
}

void ActivePerception::ProcessingThread() {
	/*if (sensors_models_.size() > 0)
		sensors_models_[0]->SetWorldPose(math::Pose(1, 0, 1, 0, 0, 0));*/
}

void ActivePerception::OnNewRGBPointCloud(const float *_pcd,
				unsigned int _width, unsigned int _height,
				unsigned int _depth, const std::string &_format) {
	ROS_INFO_STREAM("Received PCD with [width: " << _width << " | height: " << _height << " | depth: " << _depth << " | format: " << _format << "]");
}

void ActivePerception::QueueThread() {
	while (rosnode_->ok())
		queue_.callAvailable(ros::WallDuration(0.01));
}
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </member-functions>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// =============================================================================  </public-section>  ===========================================================================

// =============================================================================   <protected-section>   =======================================================================
// =============================================================================   </protected-section>  =======================================================================

} /* namespace gazebo */

