#!/usr/bin/python

from alvar_tag_tracking.srv import LaunchFile
from threading import Lock

import roslaunch
import rospy

launch_files = []
mutex = Lock()


class Launcher:
    def __init__(self, uuid):
        rospy.Service("launch", LaunchFile, self.on_launch)
        rospy.Service("stop", LaunchFile, self.on_stop)
        self.launch_files = []
        self.uuid = uuid
        roslaunch.configure_logging(self.uuid)

    def on_launch(self, req):
        args = [
            req.package,
            req.filename,
            "cam_id:="+req.id,
            "main_cam:="+req.main_cam
        ]

        for i in range(len(req.args)):
            args.append(req.args[i]+":="+req.values[i])

        launch_file = roslaunch.rlutil.resolve_launch_arguments(args)[0]
        launch_args = args[2:]

        launch = roslaunch.parent.ROSLaunchParent(self.uuid, [(launch_file, launch_args)])
        mutex.acquire()
        launch_files.append(launch)
        mutex.release()

        rospy.loginfo("Launch file started")

        return True

    def on_stop(self, req):
        pass


if __name__ == '__main__':
    rospy.init_node("launcher")
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    launcher = Launcher(uuid)
    rate = rospy.Rate(0.3)

    while not rospy.is_shutdown():
        mutex.acquire()
        for launch in launch_files:
            launch.start()
            launch_files.remove(launch)
        mutex.release()
        rate.sleep()
