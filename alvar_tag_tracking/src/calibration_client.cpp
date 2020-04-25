#include <iostream>
#include <string>
#include <map>
#include <unordered_set>

#include <ros/ros.h>

#include <alvar_tag_tracking/Calibration.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <std_msgs/String.h>

void marker_pose_callback(
  const boost::shared_ptr<ar_track_alvar_msgs::AlvarMarkers const> msg,
  ros::ServiceClient& calibration_srv,
  std::map<std::string, ros::Time>& cameras,
  std::unordered_set<std::string>& calibrated_cameras)
{
  for (auto iter=cameras.begin(); iter!=cameras.end(); ++iter) {
    if (ros::Time::now() - iter->second > ros::Duration(0.3))
      cameras.erase(iter);
  }

  if (!msg->markers.empty()) {
    std::string camera = msg->markers.front().header.frame_id;
    cameras.insert({camera, msg->markers.front().header.stamp});

    if (calibrated_cameras.find(camera) == calibrated_cameras.end()) {
      for (auto cam: cameras) {
        if (calibrated_cameras.find(cam.first) != calibrated_cameras.end()) {
          alvar_tag_tracking::Calibration srv;
          srv.request.source_cam = cam.first;
          srv.request.target_cam = camera;
          srv.request.id = msg->markers.front().id;
          //TODO: camera recalibrated during shutdown

          if (calibration_srv.call(srv)) {
            if (srv.response.res) {
              calibrated_cameras.insert(camera);
            }
          }
        }
      }
    }
  }
}

void on_reconf(
  const boost::shared_ptr<std_msgs::String const> msg,
  std::unordered_set<std::string>& cameras,
  const std::string& main_cam) //TODO: is main_cam usefull ?
{
  const std::string& cam_name = msg->data;
  if (cameras.count(cam_name)) {
    cameras.erase(cam_name);
    ROS_INFO("Client: removing camera %s's transform", cam_name.c_str());
  }
  else {
    ROS_WARN("Client: camera not found");
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "calibration_client");

  ros::NodeHandle pn("~");
  std::string main_cam;

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

  ros::NodeHandle n;
  ros::ServiceClient service = n.serviceClient<alvar_tag_tracking::Calibration>("calibration");

  std::map<std::string, ros::Time> cameras;
  std::unordered_set<std::string>  calibrated_cameras = {main_cam};

  auto callback = boost::bind(
    &marker_pose_callback,
    _1,
    boost::ref(service),
    boost::ref(cameras),
    boost::ref(calibrated_cameras));

  auto reconf = boost::bind(
      on_reconf,
      _1,
      boost::ref(calibrated_cameras),
      boost::ref(main_cam));

  ros::Subscriber pose_sub =
    n.subscribe<ar_track_alvar_msgs::AlvarMarkers>("/ar_pose_marker", 0, callback);
  ros::Subscriber reconf_sub =
    n.subscribe<std_msgs::String>("/reconfigure", 10, reconf);

  ros::spin();
  return 0;
}
