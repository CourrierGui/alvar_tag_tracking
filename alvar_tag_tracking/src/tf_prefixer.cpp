#include <ros/ros.h>
#include <tf2_msgs/TFMessage.h>

#include <iostream>
#include <string>

std::string concatenate(const std::string& parent, const std::string& child) {
  std::string res = parent;
  if (child[0] !=  '/' && parent.back() != '/')
    res += '/';

  return res + child;
}

void tf_callback(const tf2_msgs::TFMessage& msg, ros::Publisher pub) {
	tf2_msgs::TFMessage prefixed_msg = msg;

	for (auto& tf: prefixed_msg.transforms) {

    if (tf.header.frame_id.find("ar_marker_") != std::string::npos)
      tf.header.frame_id = concatenate(tf.child_frame_id, tf.header.frame_id);
		else
			tf.child_frame_id = concatenate(tf.header.frame_id, tf.child_frame_id);
	}

	pub.publish(prefixed_msg);
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "tf_prefixer");
  std::string tf_topic;

	ros::NodeHandle pn("~");
	if (argc > 1) {
		tf_topic = argv[1];
	} else {
		pn.getParam("tf_topic", tf_topic);
	}

	ros::NodeHandle n;

  ros::Publisher tf_pub = n.advertise<tf2_msgs::TFMessage>("/tf", 0);

  boost::function<void(const tf2_msgs::TFMessage&)> callback = boost::bind(tf_callback, _1, tf_pub);
	ros::Subscriber sub = n.subscribe<tf2_msgs::TFMessage>(tf_topic, 0, callback);

	ros::spin();
	return 0;
}
