#pragma once

#include <QDialog>
#include <QPushButton>
#include <QDialogButtonBox>
#include <QString>
#include <QFileDialog>

#include <ui_new_cam_dialog.h>

#include <ros/package.h>
#include <ros/ros.h>
#include <experimental/filesystem>

namespace rqt_calibration {


  //TODO: add more than one camera at a time ?
class NewCamDialog : public QDialog {
  Q_OBJECT;

public:
  struct Settings {
    QString launch_file;
    QString cam_id;
    QString args;
    QString type;
  };

  NewCamDialog(QWidget* parent);
  void load_launchfiles();

signals:
  void new_settings(const NewCamDialog::Settings& settings);

public slots:
  void check_values();
  void open_browser();
  void set_cam_id();
  void set_args();
  void set_type(int index);
  void set_launch_file(int index);

private:
  Ui_Dialog ui;
  QPushButton* ok_button;
  QPushButton* cancel_button;

  QString launch_file;
  QString cam_id;
  QString args;
  QString type;
};

}
