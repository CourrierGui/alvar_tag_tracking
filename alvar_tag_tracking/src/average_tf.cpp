#include <iostream>
#include <string>
#include <map>

#include <ros/ros.h>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose.h>
#include <tf2_msgs/TFMessage.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <tf/transform_broadcaster.h>

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
  const std::map<std::string, tf::Transform>& calibrated_cameras,
  marker_struct& markers,
  tf::TransformBroadcaster& tf_broadcaster)
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
    tf::Transform new_tf(
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
    new_tf = calibrated_cameras.at(camera.name) * new_tf;
    av_tf = average(av_tf, new_tf);
  }
  std::string child_frame = "/average/ar_marker_" + std::to_string(marker.id);

  normalize(av_tf.tf);
  tf_broadcaster.sendTransform(
    tf::StampedTransform(av_tf.tf, ros::Time::now(), main_cam, child_frame)
  );
}

void static_tf_callback(
  const boost::shared_ptr<tf2_msgs::TFMessage const> msg,
  std::map<std::string, tf::Transform>& calibrated_cameras)
{
  std::string child_frame = msg->transforms.front().child_frame_id;
  if (child_frame.front() == '/') {
    child_frame.erase(0, 1);
  }
  tf::Transform tf(
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
  tf::TransformBroadcaster tf_broadcaster;
  std::map<std::string, tf::Transform> calibrated_cameras;
  marker_struct markers;
  calibrated_cameras[main_cam] = tf::Transform();

  auto pose_cb = boost::bind(
    marker_callback, _1,
    main_cam,
    boost::ref(calibrated_cameras),
    boost::ref(markers),
    boost::ref(tf_broadcaster));

  auto tf_cb = boost::bind(
    static_tf_callback, _1,
    boost::ref(calibrated_cameras));

  ros::Subscriber pose_sub =
    n.subscribe<ar_track_alvar_msgs::AlvarMarkers>("/ar_pose_marker", 0, pose_cb);
  ros::Subscriber tf_sub =
    n.subscribe<tf2_msgs::TFMessage>("/tf_static", 0, tf_cb);

  ros::spin();
  return 0;
}
