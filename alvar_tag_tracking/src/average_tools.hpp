#pragma once

#include <tf/tf.h>
#include <cmath>

struct average_tf {
  tf::Transform tf;
  int weight;
  inline average_tf() : tf(tf::Transform::getIdentity()), weight(0) {}
};

inline average_tf average(const average_tf& av_tf, const tf::Transform& new_tf) {
  int n = av_tf.weight;
  tf::Quaternion av_quat = av_tf.tf.getRotation();
  tf::Vector3    av_vect = av_tf.tf.getOrigin();

  tf::Quaternion new_quat = new_tf.getRotation();
  tf::Vector3    new_vect = new_tf.getOrigin();

  double x = (av_vect.getX() * n + new_vect.getX())/(n+1);
  double y = (av_vect.getY() * n + new_vect.getY())/(n+1);
  double z = (av_vect.getZ() * n + new_vect.getZ())/(n+1);

  double angle = (av_quat.getAngle() * n + new_quat.getAngle())/(n+1);
  double qx    = (av_quat.getAxis().getX() * n + new_quat.getAxis().getX())/(n+1);
  double qy    = (av_quat.getAxis().getY() * n + new_quat.getAxis().getY())/(n+1);
  double qz    = (av_quat.getAxis().getZ() * n + new_quat.getAxis().getZ())/(n+1);

  average_tf res;
  res.tf = tf::Transform(tf::Quaternion(tf::Vector3(qx, qy, qz), angle), tf::Vector3(x, y, z));
  res.weight = n+1;
  return res;
}

inline void normalize(tf::Transform& tf) {
  double qx = tf.getRotation().getX();
  double qy = tf.getRotation().getY();
  double qz = tf.getRotation().getZ();
  double qw = tf.getRotation().getW();
  double norm = std::sqrt(qx*qx + qy*qy + qz*qz + qw*qw);
  tf::Quaternion q = {qx/norm, qy/norm, qz/norm, qw/norm};
  tf.setRotation(q);
}
