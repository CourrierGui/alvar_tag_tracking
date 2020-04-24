#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Transform.h>
#include <geometry_msgs/TransformStamped.h>

#include <sstream>
#include <map>
#include <unordered_set>

#include <alvar_tag_tracking/Calibration.h>
#include "average_tools.hpp"

using calibration_srv = alvar_tag_tracking::Calibration;

bool calibration(calibration_srv::Request& request,
                 calibration_srv::Response& response,
                 const tf2_ros::Buffer& tf_buffer,
                 const tf2_ros::TransformListener& tf_listener,
                 std::unordered_set<std::string>& calibrated_cameras,
                 std::map<std::string, average_tf>& list_of_tfs,
                 tf2_ros::StaticTransformBroadcaster& tf_broadcaster)
{
  std::string marker = "/ar_marker_" + std::to_string(request.id);
  geometry_msgs::TransformStamped source_cam_to_marker;
  geometry_msgs::TransformStamped target_cam_to_marker;
  response.res = false;

  if (   tf_buffer.canTransform(request.target_cam, request.target_cam + marker, ros::Time(0))
      && tf_buffer.canTransform(request.source_cam + marker, request.source_cam, ros::Time(0))
      && calibrated_cameras.find(request.target_cam) == calibrated_cameras.end())
  {
    source_cam_to_marker = tf_buffer.lookupTransform(request.source_cam,
                                                     request.source_cam + marker,
                                                     ros::Time(0));
    target_cam_to_marker = tf_buffer.lookupTransform(request.target_cam,
                              request.target_cam + marker,
                              ros::Time(0));

    tf2::Transform source_to_target =
      convert(source_cam_to_marker) * convert(target_cam_to_marker).inverse();

    list_of_tfs[request.target_cam] = average(list_of_tfs[request.target_cam], source_to_target);

    if (list_of_tfs[request.target_cam].weight == 10) {
      geometry_msgs::TransformStamped transform =
        tf_to_msg(source_to_target, ros::Time::now(), request.source_cam, request.target_cam);

      tf_broadcaster.sendTransform(transform);

      calibrated_cameras.insert(request.target_cam);
      response.res = true;
      return true;
    }
  }
  return false;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "calibration_server");
  ros::NodeHandle n;

  std::string main_cam;
  ros::NodeHandle pn("~");

  if (argc > 1) {
    main_cam = argv[1];
  } else if (pn.hasParam("main_cam")) {
    pn.getParam("main_cam", main_cam);
  } else {
    std::cerr << "Invalid arguments: "
      << argv[0]
      << " <main_cam>\n";
    return 0;
  }

  std::map<std::string, average_tf> list_of_tfs;
  std::unordered_set<std::string>  calibrated_cameras = {main_cam};
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener(tf_buffer);
  tf2_ros::StaticTransformBroadcaster tf_broadcaster;

  auto callback = boost::bind(
    calibration,
    _1,
    _2,
    boost::ref(tf_buffer),
    boost::ref(tf_listener),
    boost::ref(calibrated_cameras),
    boost::ref(list_of_tfs),
    boost::ref(tf_broadcaster));

  ros::ServiceServer service
    = n.advertiseService<calibration_srv::Request,calibration_srv::Response>("calibration", callback);

  ros::spin();
  return 0;
}
