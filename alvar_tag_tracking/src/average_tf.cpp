#include <iostream>
#include <string>
#include <map>

#include <ros/ros.h>

#include <std_msgs/String.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose.h>
#include <tf2_msgs/TFMessage.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <tf2_ros/transform_broadcaster.h>

#include "average_tools.hpp"

struct cam_info {
  std::string name;
  ros::Time time;
  geometry_msgs::Pose pose;
};

using marker_struct = std::multimap<int, cam_info>;

void marker_callback(
  const boost::shared_ptr<ar_track_alvar_msgs::AlvarMarkers const> msg,
  const std::string& main_cam,
  const std::map<std::string, tf2::Transform>& calibrated_cameras,
  marker_struct& markers,
  tf2_ros::TransformBroadcaster& tf_broadcaster)
{
  const auto& marker = msg->markers.front();

  if (calibrated_cameras.count(marker.header.frame_id) == 0)
    return;

  cam_info ci{
    .name = marker.header.frame_id,
    .time = marker.header.stamp,
    .pose = marker.pose.pose
  };
  markers.insert({marker.id, ci});

  auto [beg, end] = markers.equal_range(marker.id);
  average_tf av_tf;

  for (auto cam=beg; cam!=end; ++cam) {
    const auto& camera = cam->second;
    // calcul un peu lourd pour pas grand chose
    if (camera.name == marker.header.frame_id && camera.time < marker.header.stamp) {
      markers.erase(cam);
      continue;
    }
    if (camera.time - marker.header.stamp > ros::Duration(0.3)) {
      markers.erase(cam);
      continue;
    }
    tf2::Transform new_tf(
      {
        camera.pose.orientation.x,
        camera.pose.orientation.y,
        camera.pose.orientation.z,
        camera.pose.orientation.w
      },
      {
        camera.pose.position.x,
        camera.pose.position.y,
        camera.pose.position.z
      }
    );
    if (auto it=calibrated_cameras.find(camera.name); it!=calibrated_cameras.end()) {
      new_tf = it->second * new_tf;
      av_tf = average(av_tf, new_tf);
    }
  }
  std::string child_frame = "/average/ar_marker_" + std::to_string(marker.id);

  normalize(av_tf.tf);
  geometry_msgs::TransformStamped tf;  //{ av_tf.tf, ros::Time::now(), main_cam, child_frame }
  tf.header.stamp = ros::Time::now();
  tf.header.frame_id = main_cam;
  tf.child_frame_id = child_frame;

  tf.transform.rotation.x = av_tf.tf.getRotation().getX();
  tf.transform.rotation.y = av_tf.tf.getRotation().getY();
  tf.transform.rotation.z = av_tf.tf.getRotation().getZ();
  tf.transform.rotation.w = av_tf.tf.getRotation().getW();

  tf.transform.translation.x = av_tf.tf.getOrigin().getX();
  tf.transform.translation.y = av_tf.tf.getOrigin().getY();
  tf.transform.translation.z = av_tf.tf.getOrigin().getZ();

  tf_broadcaster.sendTransform(tf);
}

void static_tf_callback(
  const boost::shared_ptr<tf2_msgs::TFMessage const> msg,
  std::map<std::string, tf2::Transform>& calibrated_cameras)
{
  std::string child_frame = msg->transforms.front().child_frame_id;
  if (child_frame.front() == '/') {
    child_frame.erase(0, 1);
  }
  tf2::Transform tf(
    {
      msg->transforms.front().transform.rotation.x,
      msg->transforms.front().transform.rotation.y,
      msg->transforms.front().transform.rotation.z,
      msg->transforms.front().transform.rotation.w
    },
    {
      msg->transforms.front().transform.translation.x,
      msg->transforms.front().transform.translation.y,
      msg->transforms.front().transform.translation.z
    }
  );
  calibrated_cameras[child_frame] = tf;
}

void on_reconf(
  const boost::shared_ptr<std_msgs::String const> msg,
  std::map<std::string, tf2::Transform>& cameras,
  const std::string& main_cam)
{
  const std::string& cam_name = msg->data;

  if (auto it = cameras.find(cam_name); it!=cameras.end()) {
    cameras.erase(it);
    ROS_INFO("Removing camera %s's transform", msg->data.c_str());
  } else {
    ROS_WARN("Camera not found");
  }
}

//TODO: segfault if cam doesn't see tag
int main(int argc, char** argv) {
  ros::init(argc, argv, "average_tf");

  ros::NodeHandle pn("~");
  std::string main_cam;

  if (pn.hasParam("main_cam")) {
    pn.getParam("main_cam", main_cam);
  } else if (argc > 1) {
    main_cam = argv[1];
  } else {
    std::cerr << "Invalid arguments: "
      << argv[0]
      << " <main_cam>\n";
    return 0;
  }

  ros::NodeHandle n;
  tf2_ros::TransformBroadcaster tf_broadcaster;
  std::map<std::string, tf2::Transform> calibrated_cameras;
  marker_struct markers;
  calibrated_cameras[main_cam] = tf2::Transform();

  auto pose_cb = boost::bind(
    marker_callback, _1,
    main_cam,
    boost::ref(calibrated_cameras),
    boost::ref(markers),
    boost::ref(tf_broadcaster));

  auto tf_cb = boost::bind(
    static_tf_callback, _1,
    boost::ref(calibrated_cameras));

  auto reconf = boost::bind(
    on_reconf,
    _1,
    boost::ref(calibrated_cameras),
    boost::ref(main_cam));

  ros::Subscriber pose_sub =
    n.subscribe<ar_track_alvar_msgs::AlvarMarkers>("/ar_pose_marker", 0, pose_cb);
  ros::Subscriber tf_sub =
    n.subscribe<tf2_msgs::TFMessage>("/tf_static", 0, tf_cb);
  ros::Subscriber reconf_sub = n.subscribe<std_msgs::String>("/reconfigure", 10, reconf);

  ros::spin();
  return 0;
}
