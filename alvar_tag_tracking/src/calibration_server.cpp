#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <sstream>
#include <map>
#include <unordered_set>

#include <alvar_tag_tracking/calibration.h>
#include "average_tools.hpp"

using calibration_srv = alvar_tag_tracking::calibration;

bool calibration(calibration_srv::Request& request,
                 calibration_srv::Response& response,
                 const tf::TransformListener& tf_listener,
                 std::unordered_set<std::string>& calibrated_cameras,
                 std::map<std::string, average_tf>& list_of_tfs)
{
  std::string marker = "/ar_marker_" + std::to_string(request.id);
  tf::StampedTransform source_cam_to_marker;
  tf::StampedTransform target_cam_to_marker;
  response.res = false;

  if (   tf_listener.canTransform(request.target_cam, request.target_cam + marker, ros::Time(0))
      && tf_listener.canTransform(request.source_cam + marker, request.source_cam, ros::Time(0))
      && calibrated_cameras.find(request.target_cam) == calibrated_cameras.end())
  {
    tf_listener.lookupTransform(request.source_cam,
                                request.source_cam + marker,
                                ros::Time(0),
                                source_cam_to_marker);
    tf_listener.lookupTransform(request.target_cam,
                                request.target_cam + marker,
                                ros::Time(0),
                                target_cam_to_marker);

    tf::Transform source_to_target = source_cam_to_marker * target_cam_to_marker.inverse();

    list_of_tfs[request.target_cam] = average(list_of_tfs[request.target_cam], source_to_target);

    if (list_of_tfs[request.target_cam].weight == 10) {
      tf::StampedTransform transform(
        list_of_tfs[request.target_cam].tf,
        ros::Time::now(),
        request.source_cam,
        request.target_cam);

      std::stringstream os;
      os << "rosrun tf2_ros static_transform_publisher "
        << transform.getOrigin().getX() << ' '
        << transform.getOrigin().getY() << ' '
        << transform.getOrigin().getZ() << ' '
        << transform.getRotation().getX() << ' '
        << transform.getRotation().getY() << ' '
        << transform.getRotation().getZ() << ' '
        << transform.getRotation().getW() << ' '
        << request.source_cam << ' '
        << request.target_cam << " &";
      std::string cmd = os.str();
      system(cmd.c_str());

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
  tf::TransformListener tf_listener(n);

  auto callback = boost::bind(
    calibration,
    _1,
    _2,
    boost::ref(tf_listener),
    boost::ref(calibrated_cameras),
    boost::ref(list_of_tfs));

  ros::ServiceServer service
    = n.advertiseService<calibration_srv::Request,calibration_srv::Response>("calibration", callback);

  ros::spin();
  return 0;
}
