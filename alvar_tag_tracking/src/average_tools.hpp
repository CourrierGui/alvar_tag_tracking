#pragma once

#include <tf/tf.h>
#include <tf2/LinearMath/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <cmath>

struct average_tf {
  tf2::Transform tf;
  int weight;
  inline average_tf() : tf(tf2::Transform::getIdentity()), weight(0) {}
};

inline average_tf average(const average_tf& av_tf, const tf2::Transform& new_tf) {
  int n = av_tf.weight;
  tf2::Quaternion av_quat = av_tf.tf.getRotation();
  tf2::Vector3    av_vect = av_tf.tf.getOrigin();

  tf2::Quaternion new_quat = new_tf.getRotation();
  tf2::Vector3    new_vect = new_tf.getOrigin();

  double x = (av_vect.getX() * n + new_vect.getX())/(n+1);
  double y = (av_vect.getY() * n + new_vect.getY())/(n+1);
  double z = (av_vect.getZ() * n + new_vect.getZ())/(n+1);

  double angle = (av_quat.getAngle() * n + new_quat.getAngle())/(n+1);
  double qx    = (av_quat.getAxis().getX() * n + new_quat.getAxis().getX())/(n+1);
  double qy    = (av_quat.getAxis().getY() * n + new_quat.getAxis().getY())/(n+1);
  double qz    = (av_quat.getAxis().getZ() * n + new_quat.getAxis().getZ())/(n+1);

  average_tf res;
  res.tf = tf2::Transform(tf2::Quaternion(tf2::Vector3(qx, qy, qz), angle), tf2::Vector3(x, y, z));
  res.weight = n+1;
  return res;
}

inline void normalize(tf2::Transform& tf) {
  double qx = tf.getRotation().getX();
  double qy = tf.getRotation().getY();
  double qz = tf.getRotation().getZ();
  double qw = tf.getRotation().getW();
  double norm = std::sqrt(qx*qx + qy*qy + qz*qz + qw*qw);
  tf2::Quaternion q = {qx/norm, qy/norm, qz/norm, qw/norm};
  tf.setRotation(q);
}

inline tf2::Transform convert(const geometry_msgs::TransformStamped& tf_stamped) {
  return tf2::Transform{
    {
      tf_stamped.transform.rotation.x,
      tf_stamped.transform.rotation.y,
      tf_stamped.transform.rotation.z,
      tf_stamped.transform.rotation.w
    },
    {
      tf_stamped.transform.translation.x,
      tf_stamped.transform.translation.y,
      tf_stamped.transform.translation.z
    }
  };
}

inline geometry_msgs::TransformStamped tf_to_msg(
  const tf2::Transform& tf,
  const ros::Time& time,
  const std::string& frame_id,
  const std::string& child_frame_id)
{
  geometry_msgs::TransformStamped res;
  res.header.stamp = time;
  res.header.frame_id = frame_id;
  res.child_frame_id = child_frame_id;
  
  res.transform.rotation.x = tf.getRotation().getX();
  res.transform.rotation.y = tf.getRotation().getY();
  res.transform.rotation.z = tf.getRotation().getZ();
  res.transform.rotation.w = tf.getRotation().getW();

  res.transform.translation.x = tf.getOrigin().getX();
  res.transform.translation.y = tf.getOrigin().getY();
  res.transform.translation.z = tf.getOrigin().getZ();
  return res;

}
