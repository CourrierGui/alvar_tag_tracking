#pragma once

#include <vector>
#include <string>
#include <fstream>

#include <yaml-cpp/yaml.h>
#include <rqt_gui_cpp/plugin.h>
#include <ui_rqt_calibration.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <rqt_calibration/new_cam_dialog.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include <QList>
#include <QStringList>
#include <QString>
#include <QHBoxLayout>
#include <QWidget>
#include <QPixmap>
#include <QFileDialog>

namespace rqt_calibration {

class RqtCalibration : public rqt_gui_cpp::Plugin {
  Q_OBJECT;

public:
  RqtCalibration();
  virtual void initPlugin(qt_gui_cpp::PluginContext& context);
  virtual void shutdownPlugin();
  virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;
  virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings);

  void image_callback(const sensor_msgs::Image& msg);
  void init_args();
  void update_topic(const std::string& cam_name);

public slots:
  void set_main_cam();
  void set_marker_size();
  void new_cam_win();
  void launch_cam(const NewCamDialog::Settings& settings);
  void change_cam_sub();
  void load_calibration();
  void save_calibration();

private:
  Ui_RqtCalibration ui;
  QWidget* widget;
  ros::Subscriber subscriber;
  QString main_cam;
  double marker_size;
  QPushButton* current_button;
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener;
  tf2_ros::StaticTransformBroadcaster tf_broadcaster;
};

} //namespace rqt_calibration
