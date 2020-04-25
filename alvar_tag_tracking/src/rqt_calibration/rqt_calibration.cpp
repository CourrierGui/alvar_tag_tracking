#include <rqt_calibration/rqt_calibration.hpp>
#include <pluginlib/class_list_macros.h>
#include <QStringList>

namespace rqt_calibration {

RqtCalibration::RqtCalibration()
  : rqt_gui_cpp::Plugin(), widget(0), marker_size(0),
  current_button(0), tf_buffer(), tf_listener(tf_buffer),
  tf_broadcaster()
{
  setObjectName("rqt_calibration");
}

void RqtCalibration::initPlugin(qt_gui_cpp::PluginContext& context) {
  widget = new QWidget();
  ui.setupUi(widget);
  context.addWidget(widget);
  ui.scroll_layout->setAlignment(Qt::AlignTop);

  ros::NodeHandle pn = getNodeHandle();
  if (pn.hasParam("marker_size")) {
    pn.getParam("marker_size", marker_size);
  }
  if (pn.hasParam("main_cam")) {
    std::string tmp;
    pn.getParam("main_cam", tmp);
    main_cam = QString(tmp.c_str());
  }
  init_args();

  ros::NodeHandle n = getNodeHandle();
  launcher_start = n.serviceClient<alvar_tag_tracking::LaunchFile>( "/launcher/launch");
  launcher_stop  = n.serviceClient<alvar_tag_tracking::LaunchFile>( "/launcher/stop"  );
  reconf_pub = n.advertise<std_msgs::String>("/reconfigure", 10);
  calibrated_sub =
    n.subscribe("/calibrated", 0, &RqtCalibration::calibrated_callback, this);

  QObject::connect(ui.main_cam_edit,    SIGNAL(editingFinished()), this, SLOT(set_main_cam())    );
  QObject::connect(ui.marker_size_spin, SIGNAL(editingFinished()), this, SLOT(set_marker_size()) );
  QObject::connect(ui.add_cam_button,   SIGNAL(clicked()),         this, SLOT(new_cam_win())     );
  QObject::connect(ui.load_button,      SIGNAL(clicked()),         this, SLOT(load_calibration()));
  QObject::connect(ui.save_button,      SIGNAL(clicked()),         this, SLOT(save_calibration()));
}

void RqtCalibration::init_args() {
  ui.main_cam_edit->setText(main_cam);
  ui.marker_size_spin->setValue(marker_size);
}

void RqtCalibration::calibrated_callback(const std_msgs::String& camera) {
  QWidget* widget = ui.scroll_contents->findChild<QWidget*>(camera.data.c_str());
  QPushButton* button = widget->findChild<QPushButton*>("calibrate");
  button->setEnabled(true);
}

void RqtCalibration::image_callback(const sensor_msgs::Image& msg) {
  QImage image(&(msg.data[0]), msg.width, msg.height, QImage::Format_RGB888);
  ui.image_label->setPixmap(QPixmap::fromImage(image));
}

void RqtCalibration::new_cam_win() {
  NewCamDialog* win = new NewCamDialog(widget);
  QObject::connect(win,  SIGNAL(new_settings(const NewCamDialog::Settings&)),
                   this, SLOT(launch_cam(const NewCamDialog::Settings&)));
  win->show();
}

void RqtCalibration::remove_cam() {
  QPushButton* stop_button = qobject_cast<QPushButton*>(sender());
  QObject* parent = stop_button->parent();
  QPushButton* button = parent->findChild<QPushButton*>("cam_button");
  if (button != nullptr) {
    const QString& cam_name = button->text();
    alvar_tag_tracking::LaunchFile lf;

    lf.request.type = cam_name.section('_', 0, 0).toStdString();
    lf.request.id = cam_name.section('_', 2).toStdString();
    if (launcher_stop.call(lf)) {
      //Remove buttons
      parent->deleteLater();
      //Notifiy that the camera is no longer calibrated
      std_msgs::String msg;
      msg.data = cam_name.toStdString();
      reconf_pub.publish(msg);
      ROS_INFO("Stopping camera: %s", cam_name.toStdString().c_str());
    }
  }
}

void RqtCalibration::calibrate() {
  QPushButton* calibrate = qobject_cast<QPushButton*>(sender());
  QObject* parent = calibrate->parent();
  QPushButton* button = parent->findChild<QPushButton*>("cam_button");
  std_msgs::String msg;
  msg.data = button->text().toStdString();
  reconf_pub.publish(msg);
}

void RqtCalibration::launch_cam(const NewCamDialog::Settings& settings) {
  alvar_tag_tracking::LaunchFile lf;
  lf.request.package  = "alvar_tag_tracking";
  lf.request.filename = settings.launch_file.toStdString();
  lf.request.main_cam = main_cam.toStdString();
  lf.request.id       = settings.cam_id.toStdString();
  lf.request.type     = settings.type.toStdString();
  //TODO: handle additionnal arguments

  if (!launcher_start.call(lf)) {
    ROS_ERROR("Failed to call service alvar_tag_tracking::LaunchFile.");
    return;
  }

  QString cam_name = settings.type + "_cam_" + settings.cam_id;

  QWidget*     widget   = new QWidget;
  QHBoxLayout* h_layout = new QHBoxLayout;
  QVBoxLayout* v_layout = new QVBoxLayout;
  widget->setObjectName(cam_name);
  widget->setLayout(v_layout);
  //TODO: segfault when adding new cam right after deleting it.
  // maybe because of the object name ?

  QPushButton* button    = new QPushButton(cam_name   );
  QPushButton* stop      = new QPushButton("Stop"     );
  QPushButton* calibrate = new QPushButton("Calibrate");
  v_layout->addWidget(button   );
  h_layout->addWidget(stop     );
  h_layout->addWidget(calibrate);
  v_layout->addLayout(h_layout );
  QObject::connect(button,    SIGNAL(clicked()), this, SLOT(change_cam_sub()));
  QObject::connect(stop,      SIGNAL(clicked()), this, SLOT(remove_cam()    ));
  QObject::connect(calibrate, SIGNAL(clicked()), this, SLOT(calibrate()     ));
  button->setObjectName("cam_button");
  calibrate->setObjectName("calibrate");

  ui.scroll_layout->addWidget(widget);
  update_topic(button->text().toStdString());
  button->setEnabled(false);
  calibrate->setEnabled(false);

  if (current_button != nullptr)
    current_button->setEnabled(true);

  current_button = button;
}

void RqtCalibration::change_cam_sub() {
  ros::NodeHandle n;
  QPushButton* button = qobject_cast<QPushButton*>(sender());
  std::string cam_name = button->text().toStdString();

  update_topic(cam_name);
  button->setEnabled(false);

  if (current_button != nullptr)
    current_button->setEnabled(true);

  current_button = button;
}

void RqtCalibration::update_topic(const std::string& cam_name) {
  ros::NodeHandle n = getNodeHandle();
  image_sub =
    n.subscribe('/'+cam_name+"/image_raw", 0, &RqtCalibration::image_callback, this);
}

void RqtCalibration::set_main_cam() {
  main_cam = ui.main_cam_edit->text();
}

void RqtCalibration::set_marker_size() {
  marker_size = ui.marker_size_spin->value();
}

void RqtCalibration::shutdownPlugin() {
  // unregister all publishers here
  // TODO
}

void RqtCalibration::load_calibration() {
  QString filename = QFileDialog::getOpenFileName(
    nullptr, tr("Open configuration"), "",
    tr("Configuration file (*.yaml)"));
  YAML::Node settings = YAML::LoadFile(filename.toStdString());

  main_cam = settings["main_cam"].as<std::string>().c_str();
  marker_size = settings["marker_size"].as<double>();

  YAML::Node cameras = settings["cameras"];

  for (const auto& cam: cameras) {
    const std::string& cam_name = cam.as<std::string>();
    YAML::Node tf = settings[cam_name];

    if (tf.size() == 7) {
      geometry_msgs::TransformStamped stamped_tf;
      stamped_tf.transform.rotation.x    = tf[0].as<double>();
      stamped_tf.transform.rotation.y    = tf[1].as<double>();
      stamped_tf.transform.rotation.z    = tf[2].as<double>();
      stamped_tf.transform.rotation.w    = tf[3].as<double>();
      stamped_tf.transform.translation.x = tf[4].as<double>();
      stamped_tf.transform.translation.y = tf[5].as<double>();
      stamped_tf.transform.translation.z = tf[6].as<double>();
      stamped_tf.header.frame_id         = main_cam.toStdString();
      stamped_tf.header.stamp            = ros::Time::now();
      stamped_tf.child_frame_id          = cam_name;

      tf_broadcaster.sendTransform(stamped_tf);
    } else {
      ROS_WARN("Invalid transform for: %s", cam_name.c_str());
    }
  }
}

void RqtCalibration::save_calibration() {
  YAML::Node settings;
  std::vector<std::string> cameras;

  for (QPushButton* button: ui.scroll_contents->findChildren<QPushButton*>()) {
    QString cam_name = button->text();
    if (cam_name == main_cam)
      continue;

    cameras.push_back(cam_name.toStdString());
    try{
      geometry_msgs::TransformStamped tf = tf_buffer.lookupTransform(
          main_cam.toStdString(), cam_name.toStdString(), ros::Time(0));

      std::vector<double> values = {
        tf.transform.rotation.x,
        tf.transform.rotation.y,
        tf.transform.rotation.z,
        tf.transform.rotation.w,
        tf.transform.translation.x,
        tf.transform.translation.y,
        tf.transform.translation.z,
      };
      settings[cam_name.toStdString()] = values;
    }
    catch (tf2::TransformException& ex) {
      ROS_WARN("%s", ex.what());
      continue;
    }
  }
  settings["cameras"]     = cameras;
  settings["main_cam"]    = main_cam.toStdString();
  settings["marker_size"] = marker_size;

  QString filename = QFileDialog::getSaveFileName(
    nullptr, tr("Save calibration file"), "",
    tr("Calibration file (*.yaml)"));

  std::ofstream out(filename.toStdString());
  out << settings;
  out.close();
}

void RqtCalibration::saveSettings(qt_gui_cpp::Settings& plugin_settings,
    qt_gui_cpp::Settings& instance_settings) const
{
  instance_settings.setValue("main_cam",    main_cam   );
  instance_settings.setValue("marker_size", marker_size);

  QString topic = image_sub.getTopic().c_str();
  int id = topic.lastIndexOf('/');
  topic.remove(id, topic.size() - id);
  topic.remove(0, 1);

  instance_settings.setValue("topic", topic);
}

void RqtCalibration::restoreSettings(const qt_gui_cpp::Settings& plugin_settings,
    const qt_gui_cpp::Settings& instance_settings)
{
  marker_size = instance_settings.value("marker_size").toDouble();
  ui.marker_size_spin->setValue(marker_size);

  main_cam = instance_settings.value("main_cam").toString();
  ui.main_cam_edit->setText(main_cam);

  QString topic = instance_settings.value("topic").toString();
  update_topic(topic.toStdString());
}

/*bool hasConfiguration() const
{
  return true;
}

void triggerConfiguration()
{
  // Usually used to open a dialog to offer the user a set of configuration
}*/

}; // namespace rqt_calibration

PLUGINLIB_EXPORT_CLASS(rqt_calibration::RqtCalibration, rqt_gui_cpp::Plugin);
