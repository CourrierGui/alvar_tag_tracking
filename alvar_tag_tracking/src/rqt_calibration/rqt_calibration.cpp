#include <rqt_calibration/rqt_calibration.hpp>
#include <pluginlib/class_list_macros.h>
#include <QStringList>

namespace rqt_calibration {

RqtCalibration::RqtCalibration() : rqt_gui_cpp::Plugin() , widget(0) {
  // Constructor is called first before initPlugin function, needless to say.

  // give QObjects reasonable names
  setObjectName("rqt_calibration");
}

void RqtCalibration::initPlugin(qt_gui_cpp::PluginContext& context) {
  // access standalone command line arguments
  QStringList argv = context.argv();
  // create QWidget
  widget = new QWidget();
  // extend the widget with all attributes and children from UI file
  ui.setupUi(widget);
  // add widget to the user interface
  context.addWidget(widget);
}

void RqtCalibration::shutdownPlugin() {
  // unregister all publishers here
}

void RqtCalibration::saveSettings(qt_gui_cpp::Settings& plugin_settings,
    qt_gui_cpp::Settings& instance_settings) const
{
  // instance_settings.setValue(k, v)
}

void RqtCalibration::restoreSettings(const qt_gui_cpp::Settings& plugin_settings,
    const qt_gui_cpp::Settings& instance_settings)
{
  // v = instance_settings.value(k)
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
