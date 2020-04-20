#pragma once

#include <rqt_gui_cpp/plugin.h>
#include <ui_rqt_calibration.h>
#include <QWidget>

namespace rqt_calibration {

class RqtCalibration : public rqt_gui_cpp::Plugin {
  Q_OBJECT;

public:
  RqtCalibration();
  virtual void initPlugin(qt_gui_cpp::PluginContext& context);
  virtual void shutdownPlugin();
  virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;
  virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings);

private:
  Ui_RqtCalibration ui;
  QWidget* widget;
};

} //namespace rqt_calibration
