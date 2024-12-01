#include <gazebo/plugins/CameraPlugin.hh>
#include <sensor_msgs/Image.h>
#include <ros/ros.h>

namespace gazebo {
class CustomCameraPlugin : public CameraPlugin {
public:
  void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf) override {
    CameraPlugin::Load(_sensor, _sdf);

    // Initialize ROS if not already initialized
    if (!ros::isInitialized()) {
      ROS_FATAL_STREAM("ROS node for Gazebo has not been initialized. Please ensure 'roscore' is running.");
      return;
    }

    // Create a ROS NodeHandle
    this->rosNode.reset(new ros::NodeHandle("~"));

    // Read plugin parameters from SDF
    if (_sdf->HasElement("imageTopicName")) {
      this->imageTopicName = _sdf->Get<std::string>("imageTopicName");
    } else {
      this->imageTopicName = "camera/image";
    }

    // Advertise the main image topic
    this->imagePublisher = this->rosNode->advertise<sensor_msgs::Image>(this->imageTopicName, 10);

    ROS_INFO("CustomCameraPlugin loaded and publishing to topic: %s", this->imageTopicName.c_str());
  }

  void OnNewFrame(const unsigned char *_image, unsigned int _width, unsigned int _height, unsigned int _depth, const std::string &_format) override {
    sensor_msgs::Image imgMsg;
    imgMsg.header.stamp = ros::Time::now();
    imgMsg.header.frame_id = this->parentSensor->Name();
    imgMsg.height = _height;
    imgMsg.width = _width;
    imgMsg.encoding = "bgr8";
    imgMsg.step = _width * _depth;
    imgMsg.data.assign(_image, _image + (_width * _height * _depth));

    // Publish the image
    this->imagePublisher.publish(imgMsg);
  }

private:
  std::unique_ptr<ros::NodeHandle> rosNode;
  ros::Publisher imagePublisher;
  std::string imageTopicName;
};

GZ_REGISTER_SENSOR_PLUGIN(CustomCameraPlugin)
}  // namespace gazebo

