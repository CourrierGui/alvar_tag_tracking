#include <rqt_calibration/new_cam_dialog.hpp>

namespace fs = std::experimental::filesystem;
namespace rqt_calibration {

NewCamDialog::NewCamDialog(QWidget* parent) : QDialog(parent) {
  ui.setupUi(this);

  load_launchfiles();
  ok_button = ui.buttonBox->button(QDialogButtonBox::Ok);
  cancel_button = ui.buttonBox->button(QDialogButtonBox::Cancel);

  QObject::connect(ok_button,             SIGNAL(clicked()),         this, SLOT(check_values()      ));
  QObject::connect(ui.browse_button,      SIGNAL(clicked()),         this, SLOT(open_browser()      ));
  QObject::connect(ui.camera_id_edit,     SIGNAL(editingFinished()), this, SLOT(set_cam_id()        ));
  QObject::connect(ui.args_edit,          SIGNAL(editingFinished()), this, SLOT(set_args()          ));
  QObject::connect(ui.calibration_button, SIGNAL(clicked()),         this, SLOT(browse_calibration()));
  QObject::connect(ui.type_cb,   SIGNAL(currentIndexChanged(int)),   this, SLOT(set_type(int)       ));
  QObject::connect(ui.launch_cb, SIGNAL(currentIndexChanged(int)),   this, SLOT(set_launch_file(int)));
}

void NewCamDialog::set_launch_file(int index) {
  launch_file = ui.launch_cb->itemText(index);
}

void NewCamDialog::set_type(int index) {
  type = ui.type_cb->itemText(index);
}

void NewCamDialog::set_cam_id() {
  cam_id = ui.camera_id_edit->text();
}

void NewCamDialog::set_args() {
  args = ui.args_edit->text();
}

void NewCamDialog::open_browser() {
  launch_file = QFileDialog::getOpenFileName(this, "Open launchfile", "~", "Launchfiles (*.launch)");
}

void NewCamDialog::browse_calibration() {
  QString file = QFileDialog::getOpenFileName(
    this, "Open calibration file", "~", "Calibration (*.yaml *.yml)");
  ui.calibration_cb->addItem(file);
  ui.calibration_cb->setCurrentIndex(ui.calibration_cb->count()-1);
}

void NewCamDialog::load_launchfiles() {
  fs::path path = ros::package::getPath("alvar_tag_tracking");
  path /= "launch";

  for (auto& f: fs::directory_iterator(path)) {
    ui.launch_cb->addItem(f.path().filename().c_str());
  }
}

void NewCamDialog::check_values() {
  if (launch_file.isEmpty()) {
    launch_file = ui.launch_cb->currentText();
  }
  if (type.isEmpty()) {
    type = ui.type_cb->currentText();
  }
  if (calibration_file.isEmpty()) {
    calibration_file = ui.calibration_cb->currentText();
  }
  Settings settings{
    .launch_file=launch_file,
    .calibration_file=calibration_file,
    .cam_id=cam_id,
    .args=args,
    .type=type
  };
  emit new_settings(settings);
}

}
